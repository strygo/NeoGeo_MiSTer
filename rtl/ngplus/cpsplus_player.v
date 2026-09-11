/* CPS+ — track playback datapath (mdp_audio heritage, see ASSESSMENT §6.2
    and research/msu1_mister_stack.md §5).

    Abstracted memory read port -> byte serializer -> codec (cpsplus_adx or
    PCM s16le passthrough) -> sample-pair FIFO (1 KB BRAM) -> sample-rate
    tick drain -> volume x fade multiply -> 16-bit stereo out.

    Track index registers (PACK_FORMAT.md §Track index entry) are latched
    from the trk_* inputs on the `start` pulse; inputs may change freely
    afterwards.  Loop semantics: the read pointer wraps after consuming
    trk_loop_end bytes back to trk_loop_start, restoring the ADX predictor
    history latched when the feed first crossed trk_loop_start (byte fields
    are the authoritative wrap points; both are codec-unit aligned by the
    pack builder).  loop_end == 0 means the track does not loop.

    Volume law (v0, PACK_FORMAT.md): effective gain = trigger x track gain,
    linear /127.  Implemented exactly as
        vol_q14   = floor((trig_gain*trk_gain) << 14 / 16129)   (127*127=16129)
        total_q14 = (vol_q14 * fade_lvl_q15) >> 15
        out       = (pcm * total_q14) >>> 14
    so 0x7f x 0x7f at full fade level is bit-transparent (vol_q14 = 16384).
    The one divide runs on a shared 32-cycle sequential divider at start.

    Fade engine (laws + constants are pack-header config, format.py):
        law 1 (Anthology): frames = const1 / arg              (0xffff/arg)
        law 2 (HSF2):      frames = (const1 / arg) * const2   (0x444/arg x 60)
        law 3 (NG MAKOTO): frames = const2 + const1 / arg     (59 + 5860/speed),
                           ramp linear in dB (0..47.25 dB in 0.75 dB steps over
                           the frames) then cut; repeats while fading ignored
    fade_target is the record's volume byte convention 0..0x7f (127 = unity;
    upstream maps the PS2 arg-byte x4 clamp).  Internally the level is Q15
    (32768 = unity) with 8 fractional guard bits; target_q15 = target*258
    (127 -> exactly 32768).  step = floor((|level-target|<<8...)/frames) is
    divider-computed once per command; the level steps once per cen_frame
    and lands on the target exactly on the last frame.  A fade that
    completes at level 0 stops the track ONLY when it was started with
    fade_stop_at0 (verb 3 fade_out, verb 6 master_fade); a verb-4 fade_keep
    that lands on 0 keeps decoding and looping muted (`playing` stays 1,
    sample_vld keeps ticking with zero output) so a later fade-up or restore
    brings the music back bit-exactly -- the arcade Alpha 2 driver mutes the
    BGM in one frame at a super finish (0xff07 argw 0xffff) and fades it back
    in ~146 frames later (0xff06 argw 0x0200 argb 0xff).  fade_loop_off
    (verb 3) clears the loop switch so the track ends at the next arrival at
    loop_end.  restore_trig ramps back to unity over RESTORE_FRAMES.

    stop -> output registers cleared on the spot (silence within the same
    sample tick); back-to-back start pulses are safe at any spacing (an
    in-flight memory read is drained before the datapath re-arms).

    Memory port contract (DDR client attaches here): mem_rd is held with a
    stable mem_addr until the single-cycle mem_ack returns the 8-byte word;
    mem_data is little-endian (byte at address {A[31:3],k} = mem_data[8k+:8]).
    Burst prefetch/FIFO belongs to the DDR client, not here.

    BRAM: 256 x 32-bit sample-pair FIFO (one M10K).  Prefill: output stays
    muted until PREFILL pairs are buffered (or the feed already ended), a
    few ms at 48 kHz.  PREFILL must be < 2**FIFO_AW or the loop-sync waits
    could starve.

    Loop crossfade (PACK_FORMAT.md v1 §Loop crossfade).  For tracks flagged
    trk_xfade_en the stored ADX is byte-exact but retains XFADE_N extra
    samples of natural continuation past loop_end (the "tail").  Playback is
    seamless without re-encoding:
      * the feed decodes linearly through loop_end into the tail
        [loop_end, loop_end+N), then wraps the read pointer to
        loop_start+N (steady loop period stays exactly loop_end-loop_start;
        the ADX predictor snapshot/restore point moves to loop_start+N so
        the wrapped decode is bit-identical to a linear decode);
      * the output stage captures the first N decoded samples of the loop
        body [loop_start, loop_start+N) into a head buffer on the first
        pass, then blends each tail sample with the matching head sample
        with an equal-power cos/sin weighting
        out[k] = w_out(k)*tail[k] + w_in(k)*head[k], k=0..N-1,
        w_out(k)=cos((k+0.5)/N*pi/2), w_in(k)=sin(...) (Q15 LUT, XF_LUT_FILE).
    The blend lives entirely in the audio-output stage (audio-rate, huge
    slack); the feed change is only the wrap/snapshot point offset, so the
    memory address path is untouched.  trk_xfade_en=0 is bit-identical to
    the plain hard-cut loop.
*/

module cpsplus_player #(parameter
    FIFO_AW        = 8,     // FIFO depth = 2**FIFO_AW sample pairs
    PREFILL        = 9'd128,// pairs buffered before output unmutes
    RESTORE_FRAMES = 8,     // restore-volume ramp length (frame ticks)
    XFADE_N        = 7200,  // loop crossfade length in samples; must be a
                            // multiple of 32 and equal the pack header
                            // xfade_samples (the LUT below is built for it)
    XF_LUT_FILE    = "cpsplus_xf_lut.hex"  // Q15 equal-power weight table
)(
    input             rst,
    input             clk,
    input             cen_sample,     // track-rate tick (frac cen, external)
    input             cen_frame,      // ~60 Hz tick: fade laws step on this

    // control (cpsplus_trigger)
    input             start,          // pulse: latch trk_* and (re)start
    input             stop,           // pulse: silence within one tick
    input             osd_pause,

    // track index registers (sampled on start)
    input      [31:0] trk_addr,       // absolute DDR byte address of stream
    input      [31:0] trk_len,        // stream bytes
    input      [31:0] trk_loop_start, // bytes, stream-relative
    input      [31:0] trk_loop_end,   // bytes; 0 = no loop
    input      [ 1:0] trk_loop_cnt,   // 0 = loop forever; 1-3 = wrap N times,
                                      // then play THROUGH loop_end to the end
                                      // of the stored stream (the source's own
                                      // outro/fade).  Finite-count tracks
                                      // store the WHOLE stream.
    input             trk_stereo,     // 0 = mono, 1 = stereo
    input             trk_codec,      // 0 = ADX frames, 1 = PCM s16le
    input      [ 6:0] trk_gain,       // linear, 0x7f = unity
    input signed [15:0] trk_c1,       // ADX predictor coefficients
    input signed [15:0] trk_c2,
    input      [ 6:0] trig_gain,      // trigger row gain, 0x7f = unity
    input             trk_xfade_en,       // per-track loop crossfade enable
    input      [31:0] trk_loop_start_smp, // loop points in samples (frame-
    input      [31:0] trk_loop_end_smp,   // aligned; crossfade region math)

    // fade engine (config from pack header + per-command arguments)
    input      [ 1:0] fade_law,       // 0 none, 1 Anthology, 2 HSF2, 3 NG MAKOTO
    input      [31:0] fade_const1,
    input      [31:0] fade_const2,
    input             fade_trig,      // pulse: start a fade
    input      [ 6:0] fade_target,    // target volume 0..0x7f
    input      [15:0] fade_arg,       // command arg word (law divisor)
    input             fade_loop_off,  // with fade_trig: verb 3, loop off
    input             fade_stop_at0,  // with fade_trig: landing on level 0 ends
                                      // the track (verbs 3/6); 0 = keep playing
                                      // muted (verb 4)
    input             restore_trig,   // pulse: ramp to unity (verb 5)

    // abstracted memory read port
    output            mem_rd,         // held until mem_ack
    output     [31:3] mem_addr,       // 8-byte word address
    input      [63:0] mem_data,       // little-endian
    input             mem_ack,

    // audio
    output reg signed [15:0] audio_l,
    output reg signed [15:0] audio_r,
    output reg        sample_vld,     // 1-clk pulse: pair issued from track
    output            playing,
    output reg        track_done,     // pulse: natural end (drain/fade-to-0)
    // CPSPLUS_DBG read-out: feed FSM state + why the last track stopped
    output     [ 2:0] dbg_fst,        // FD_IDLE/FETCH/FEED/SNAP/WRAP
    output     [ 3:0] dbg_end,        // sticky since start: {eof, loop_off,
                                      //  auto_stop(fade->0), stop verb}
    output            dbg_fempty      // sample FIFO empty
);

localparam FIFO_DEPTH = 1 << FIFO_AW;
localparam XF_FRAMES  = XFADE_N / 32;   // crossfade length in ADX frames
localparam XF_AW      = $clog2(XFADE_N);

// ------------------------------------------------------ track registers ----
reg [31:0] base_r, len_r, lstart_r, lend_r;
reg        stereo_r, codec_r;
reg signed [15:0] c1_r, c2_r;
reg [13:0] gp_r;                    // trig_gain * trk_gain
reg        xfen_r;                  // per-track crossfade enable (latched)
reg  [1:0] cnt_r;                   // finite loop count (0 = infinite)
reg  [1:0] wraps_left;              // feed-side wraps remaining (finite only)
reg [31:0] ls_smp_r, le_smp_r;      // loop points in samples (latched)
// crossfade window boundaries, precomputed once at track load so the
// per-cycle datapath compares against registered constants, never a live add
reg [31:0] le_hi_r;                 // loop_end + N  (tail region end, excl.)
reg [31:0] ls_hi_r;                 // loop_start + N (head region end / resume)
reg [31:0] wrap_thr_r;              // loop_end + N - 1 (dpos value -> wrap)

wire       adx_sel = (codec_r == 1'b0);
wire       loops   = (lend_r != 32'd0);
// crossfade feed behaviour: retain and decode XFADE_N extra samples (the
// tail) past loop_end, and move the wrap/snapshot point to loop_start+N.
wire       xf_feed = xfen_r && loops;
wire [31:0] xfb    = stereo_r ? (XF_FRAMES * 32'd36) : (XF_FRAMES * 32'd18);
wire [31:0] wrap_pos = lstart_r + (xf_feed ? xfb : 32'd0);   // resume position
wire [31:0] snap_pos = wrap_pos;                             // predictor snap
// After the last finite wrap the feed no longer stops at the loop threshold:
// it plays THROUGH loop_end to the stream end, delivering the source's own
// outro/fade verbatim (the stored bytes past loop_end are simply the stream's
// natural continuation).
wire        more_wraps = (cnt_r == 2'd0) || (wraps_left != 2'd0);
// NOTE: deliberately independent of loop_en (verb-3 fade_loop_off): that verb
// documents "the track ends at the next loop_end arrival", which the FD_FEED
// decision below enforces; end_pos still points at the loop threshold then.
wire [31:0] end_pos  = (loops && more_wraps)
                               ? (xf_feed ? (lend_r + xfb) : lend_r)
                               : len_r;

// -------------------------------------------------------- control pulses ---
reg  init_p;                         // start latched, waiting for safe point
reg  halt_p;                         // stop/auto-stop while a fetch in flight
reg  auto_stop;                      // fade completed at level 0 (1-clk)

localparam [2:0] FD_IDLE  = 3'd0,
                 FD_FETCH = 3'd1,
                 FD_FEED  = 3'd2,
                 FD_SNAP  = 3'd3,
                 FD_WRAP  = 3'd4;
reg  [2:0] fst;

// init/halt fire immediately except mid-fetch, where the ack is drained
wire fetching = (fst == FD_FETCH);
wire init_go  = init_p && (!fetching || mem_ack);
wire halt_go  = (halt_p || stop || auto_stop) && !init_p
                && (!fetching || mem_ack);

always @(posedge clk) begin
    if (rst) begin
        init_p <= 1'b0;
        halt_p <= 1'b0;
    end else begin
        if (start) begin
            init_p   <= 1'b1;
            halt_p   <= 1'b0;
            base_r   <= trk_addr;
            len_r    <= trk_len;
            lstart_r <= trk_loop_start;
            lend_r   <= trk_loop_end;
            stereo_r <= trk_stereo;
            codec_r  <= trk_codec;
            c1_r     <= trk_c1;
            c2_r     <= trk_c2;
            gp_r     <= trig_gain * trk_gain;
            xfen_r   <= trk_xfade_en;
            ls_smp_r <= trk_loop_start_smp;
            le_smp_r <= trk_loop_end_smp;
            le_hi_r  <= trk_loop_end_smp   + XFADE_N;
            ls_hi_r  <= trk_loop_start_smp + XFADE_N;
            wrap_thr_r <= trk_loop_end_smp + XFADE_N - 32'd1;
            cnt_r      <= trk_loop_cnt;
        end else if (init_go)
            init_p <= 1'b0;
        if (!start) begin
            if (stop || auto_stop) halt_p <= 1'b1;
            else if (halt_go)      halt_p <= 1'b0;
        end
    end
end

// ------------------------------------------------------------- feed FSM ----
reg  [31:0] cur;                     // absolute byte address
reg  [31:0] pos;                     // bytes fed, stream-relative
reg  [63:0] wbuf;
reg  [31:3] waddr;
reg         wvalid;
reg         feed_done;
reg         loop_en;
reg         snap_taken;
reg  [63:0] hist_snap;
reg         hist_ld;

wire        byte_here = wvalid && (cur[31:3] == waddr);
wire [ 7:0] cur_byte  = wbuf[{cur[2:0], 3'b000} +: 8];

assign mem_rd   = fetching;
assign mem_addr = cur[31:3];

// codec byte-port mux
wire adx_din_ready, adx_busy, adx_eof;
wire signed [15:0] adx_l, adx_r;
wire               adx_pcm_valid;
wire [63:0]        adx_hist_out;
wire               fifo_full;
wire pcm_din_ready;
reg  [1:0] asm_cnt;                  // PCM assembler byte count
wire feed_v  = (fst == FD_FEED) && byte_here && !init_p && !halt_p
               && !feed_done;
wire cod_rdy = adx_sel ? adx_din_ready : pcm_din_ready;
wire accept  = feed_v && cod_rdy;
wire codec_busy = adx_sel ? adx_busy : (asm_cnt != 2'd0);

wire [31:0] pos_n = pos + 32'd1;

always @(posedge clk) begin
    if (rst) begin
        fst        <= FD_IDLE;
        cur        <= 32'd0;
        pos        <= 32'd0;
        wvalid     <= 1'b0;
        feed_done  <= 1'b1;
        loop_en    <= 1'b0;
        snap_taken <= 1'b1;
        hist_snap  <= 64'd0;
        hist_ld    <= 1'b0;
    end else begin
        hist_ld <= 1'b0;
        if (init_go) begin
            fst        <= FD_FEED;
            cur        <= base_r;
            pos        <= 32'd0;
            wvalid     <= 1'b0;
            feed_done  <= 1'b0;
            loop_en    <= loops;
            wraps_left <= cnt_r;   // cnt_r latched at start, stable by init_go
            // snapshot pending for looped ADX unless the resume point is the
            // zero-history stream head (loop_start == 0 with no crossfade
            // tail offset).  Crossfade always resumes at loop_start+N > 0.
            snap_taken <= !(loops && adx_sel && wrap_pos != 32'd0);
            hist_snap  <= 64'd0;
        end else if (halt_go) begin
            fst       <= FD_IDLE;
            feed_done <= 1'b1;
        end else begin
            if (fade_trig && fade_loop_off)
                loop_en <= 1'b0;
            case (fst)
                FD_FETCH: if (mem_ack) begin
                    wbuf   <= mem_data;
                    waddr  <= cur[31:3];
                    wvalid <= 1'b1;
                    fst    <= FD_FEED;
                end
                FD_FEED: begin
                    if (feed_done)
                        fst <= FD_IDLE;
                    else if (accept) begin
                        cur <= cur + 32'd1;
                        pos <= pos_n;
                        if (pos_n == snap_pos && !snap_taken)
                            fst <= FD_SNAP;
                        else if (pos_n == end_pos) begin
                            if (loop_en && loops && more_wraps)
                                fst <= FD_WRAP;
                            else begin
                                feed_done <= 1'b1;
                                fst       <= FD_IDLE;
                            end
                        end
                    end else if (!byte_here)
                        fst <= FD_FETCH;
                end
                FD_SNAP: if (!codec_busy) begin  // decoder drained the frames
                    hist_snap  <= adx_hist_out;  // state entering loop_start
                    snap_taken <= 1'b1;
                    fst        <= FD_FEED;
                end
                FD_WRAP: if (!codec_busy) begin
                    if (adx_sel) hist_ld <= 1'b1;
                    cur <= base_r + wrap_pos;
                    pos <= wrap_pos;
                    if (cnt_r != 2'd0) wraps_left <= wraps_left - 2'd1;
                    fst <= FD_FEED;
                end
                default: ;                       // FD_IDLE
            endcase
            // defensive: an in-stream CRI EOF frame ends the track (packs
            // are validated not to contain any); checked in every state so
            // the decoder's 1-clk pulse is never missed mid-fetch
            if (adx_sel && adx_eof && fst != FD_IDLE) begin
                feed_done <= 1'b1;
                if (!fetching) fst <= FD_IDLE;
            end
        end
    end
end

// ------------------------------------------------------------ ADX codec ----
cpsplus_adx u_adx(
    .rst        ( rst                   ),
    .clk        ( clk                   ),
    .clr        ( init_go               ),
    .stereo     ( stereo_r              ),
    .c1         ( c1_r                  ),
    .c2         ( c2_r                  ),
    .din        ( cur_byte              ),
    .din_valid  ( feed_v && adx_sel     ),
    .din_ready  ( adx_din_ready         ),
    .pcm_l      ( adx_l                 ),
    .pcm_r      ( adx_r                 ),
    .pcm_valid  ( adx_pcm_valid         ),
    .pcm_ready  ( ~fifo_full            ),
    .hist_out   ( adx_hist_out          ),
    .hist_in    ( hist_snap             ),
    .hist_load  ( hist_ld               ),
    .busy       ( adx_busy              ),
    .eof        ( adx_eof               )
);

// ------------------------------------------------- PCM s16le passthrough ---
// stereo: {L_lo, L_hi, R_lo, R_hi} per pair; mono: {lo, hi}, R mirrors L
reg [7:0] asm0, asm1, asm2;
assign pcm_din_ready = ~fifo_full;
wire [1:0] asm_last  = stereo_r ? 2'd3 : 2'd1;
wire pcm_accept      = feed_v && !adx_sel && pcm_din_ready;
wire pcm_push        = pcm_accept && (asm_cnt == asm_last);
wire [31:0] pcm_pair = stereo_r ? {cur_byte, asm2, asm1, asm0}   // {R, L}
                                : {cur_byte, asm0, cur_byte, asm0};

always @(posedge clk) begin
    if (rst || init_go || halt_go)
        asm_cnt <= 2'd0;
    else if (pcm_accept) begin
        case (asm_cnt)
            2'd0: asm0 <= cur_byte;
            2'd1: asm1 <= cur_byte;
            default: asm2 <= cur_byte;
        endcase
        asm_cnt <= pcm_push ? 2'd0 : asm_cnt + 2'd1;
    end
end

// ------------------------------------------------------ sample-pair FIFO ---
reg  [31:0] fifo[0:FIFO_DEPTH-1];
reg  [FIFO_AW:0] fwr, frd;
wire [FIFO_AW:0] fused  = fwr - frd;
assign           fifo_full = fused[FIFO_AW];
wire             fempty = (fwr == frd);
reg  [31:0] fq;
reg         head_fresh;           // head pushed into an EMPTY FIFO last clk

wire        push  = adx_sel ? (adx_pcm_valid && !fifo_full) : pcm_push;
wire [31:0] pdata = adx_sel ? {adx_r, adx_l} : pcm_pair;

always @(posedge clk) begin
    // registered head read, with write-through when pushing into an empty
    // FIFO (fwr == frd).  The output path is fifo -> fq -> fl_q/fr_q -> mul
    // (the fl_q/fr_q stage was added with the crossfade blend), so a head
    // written into an EMPTY FIFO is only valid at the multiplier TWO clocks
    // after the push; head_fresh blocks a pop in the one clock in between
    // (a refill-from-underrun whose push lands exactly one clk before the
    // sample tick otherwise emits the STALE previous head once).
    fq <= (push && fwr == frd) ? pdata : fifo[frd[FIFO_AW-1:0]];
    head_fresh <= push && (fwr == frd);
    if (rst || init_go)
        fwr <= {FIFO_AW+1{1'b0}};
    else if (push) begin
        fifo[fwr[FIFO_AW-1:0]] <= pdata;
        fwr <= fwr + {{FIFO_AW{1'b0}}, 1'b1};
    end
end

// ---------------------------------------------- shared sequential divider --
reg  [31:0] dv_n, dv_q, dv_den;
reg  [32:0] dv_r;
reg  [ 5:0] dv_cnt;
reg         dv_bsy;
reg         dv_go;
reg  [31:0] dv_num_in, dv_den_in;

wire [32:0] dv_trial = {dv_r[31:0], dv_n[31]} - {1'b0, dv_den};

always @(posedge clk) begin
    if (rst) begin
        dv_bsy <= 1'b0;
        dv_cnt <= 6'd0;
    end else if (dv_go && !dv_bsy) begin
        dv_n   <= dv_num_in;
        dv_den <= (dv_den_in == 32'd0) ? 32'd1 : dv_den_in;
        dv_r   <= 33'd0;
        dv_q   <= 32'd0;
        dv_cnt <= 6'd32;
        dv_bsy <= 1'b1;
    end else if (dv_bsy) begin
        if (!dv_trial[32]) begin
            dv_r <= dv_trial;
            dv_q <= {dv_q[30:0], 1'b1};
        end else begin
            dv_r <= {dv_r[31:0], dv_n[31]};
            dv_q <= {dv_q[30:0], 1'b0};
        end
        dv_n   <= {dv_n[30:0], 1'b0};
        dv_cnt <= dv_cnt - 6'd1;
        if (dv_cnt == 6'd1) dv_bsy <= 1'b0;
    end
end

// ------------------------------------------------- volume + fade sequencer -
localparam [23:0] LVL_UNITY = 24'h800000;   // Q15.8, 32768 << 8

reg  [14:0] vol_q14;
reg         vol_rdy, vol_pend;

reg  [23:0] fade_lvl;                // Q15.8 current level
reg  [23:0] fade_tgt;                // Q15.8 target
reg  [23:0] fade_step;               // Q15.8 per-frame step magnitude
reg  [31:0] fade_frames_left;
reg         fade_act, fade_up;
reg         fade_stop0;              // this fade ends the track if it lands on 0

// Law 3 (Neo Geo MAKOTO) ramps LINEARLY IN dB, then cuts: the board adds a
// master attenuation to every FM channel's total level in 0.75 dB steps until
// the channels clamp to silence (measured: 7.6..32 dB/s, cut at ~45-50 dB;
// ngplus/PHASE1_NOTES.md).  Ear-gated 2026-09-11 against the board's own fade
// (C dB-linear closest, B linear-amplitude farthest).  Implementation: the
// attenuation index runs 0..63 (0..47.25 dB) over the law's frames in Q16
// steps, and the level is start_level x lut[index] through a registered
// 24x16 multiply (3 clks after the frame tick; a tick is ~1.6M clks).
reg         db_mode;                 // current fade is a dB ramp
reg  [23:0] db_acc, db_step;         // Q6.16 attenuation index accumulator
reg  [23:0] db_start;                // level when the fade began (Q15.8)
reg  [ 1:0] db_pipe;
reg  [15:0] db_lut_q;
reg  [39:0] db_prod;

function [15:0] db_lut(input [5:0] i);   // Q15 gain for i x 0.75 dB
    case (i)
        6'd 0: db_lut = 16'd32768;
        6'd 1: db_lut = 16'd30057;
        6'd 2: db_lut = 16'd27571;
        6'd 3: db_lut = 16'd25290;
        6'd 4: db_lut = 16'd23198;
        6'd 5: db_lut = 16'd21279;
        6'd 6: db_lut = 16'd19519;
        6'd 7: db_lut = 16'd17904;
        6'd 8: db_lut = 16'd16423;
        6'd 9: db_lut = 16'd15064;
        6'd10: db_lut = 16'd13818;
        6'd11: db_lut = 16'd12675;
        6'd12: db_lut = 16'd11627;
        6'd13: db_lut = 16'd10665;
        6'd14: db_lut = 16'd9783;
        6'd15: db_lut = 16'd8973;
        6'd16: db_lut = 16'd8231;
        6'd17: db_lut = 16'd7550;
        6'd18: db_lut = 16'd6925;
        6'd19: db_lut = 16'd6353;
        6'd20: db_lut = 16'd5827;
        6'd21: db_lut = 16'd5345;
        6'd22: db_lut = 16'd4903;
        6'd23: db_lut = 16'd4497;
        6'd24: db_lut = 16'd4125;
        6'd25: db_lut = 16'd3784;
        6'd26: db_lut = 16'd3471;
        6'd27: db_lut = 16'd3184;
        6'd28: db_lut = 16'd2920;
        6'd29: db_lut = 16'd2679;
        6'd30: db_lut = 16'd2457;
        6'd31: db_lut = 16'd2254;
        6'd32: db_lut = 16'd2068;
        6'd33: db_lut = 16'd1896;
        6'd34: db_lut = 16'd1740;
        6'd35: db_lut = 16'd1596;
        6'd36: db_lut = 16'd1464;
        6'd37: db_lut = 16'd1343;
        6'd38: db_lut = 16'd1232;
        6'd39: db_lut = 16'd1130;
        6'd40: db_lut = 16'd1036;
        6'd41: db_lut = 16'd950;
        6'd42: db_lut = 16'd872;
        6'd43: db_lut = 16'd800;
        6'd44: db_lut = 16'd734;
        6'd45: db_lut = 16'd673;
        6'd46: db_lut = 16'd617;
        6'd47: db_lut = 16'd566;
        6'd48: db_lut = 16'd519;
        6'd49: db_lut = 16'd476;
        6'd50: db_lut = 16'd437;
        6'd51: db_lut = 16'd401;
        6'd52: db_lut = 16'd368;
        6'd53: db_lut = 16'd337;
        6'd54: db_lut = 16'd309;
        6'd55: db_lut = 16'd284;
        6'd56: db_lut = 16'd260;
        6'd57: db_lut = 16'd239;
        6'd58: db_lut = 16'd219;
        6'd59: db_lut = 16'd201;
        6'd60: db_lut = 16'd184;
        6'd61: db_lut = 16'd169;
        6'd62: db_lut = 16'd155;
        6'd63: db_lut = 16'd142;
        default: db_lut = 16'd0;
    endcase
endfunction

reg         fp_pend, fp_restore, fp_stop0;
reg  [15:0] fp_arg;
reg  [23:0] fp_tgt;

localparam [2:0] FS_IDLE = 3'd0, FS_VOL = 3'd1, FS_LAW = 3'd2,
                 FS_SETUP = 3'd3, FS_STEP = 3'd4;
reg  [2:0] fs;
reg  [31:0] frames_r;

wire [15:0] tgt_q15    = (fade_target == 7'd127) ? 16'd32768
                                                 : {9'd0, fade_target} * 16'd258;
wire [15:0] law_q16    = |dv_q[31:16] ? 16'hffff : dv_q[15:0];
wire [23:0] step_delta = fade_up ? (fade_tgt - fade_lvl)
                                 : (fade_lvl - fade_tgt);
wire        law3_busy  = fade_law == 2'd3 &&
                         ((fade_act && !fade_up) || (fp_pend && !fp_restore));
wire        db_sel     = fade_law == 2'd3 && !fade_up;   // law-3 fade DOWN

always @(posedge clk) begin
    if (rst) begin
        fs        <= FS_IDLE;
        vol_pend  <= 1'b0;
        vol_rdy   <= 1'b0;
        vol_q14   <= 15'd0;
        fp_pend   <= 1'b0;
        fade_lvl  <= LVL_UNITY;
        fade_act  <= 1'b0;
        auto_stop <= 1'b0;
        dv_go     <= 1'b0;
        db_mode   <= 1'b0;
        db_pipe   <= 2'd0;
    end else begin
        auto_stop <= 1'b0;
        dv_go     <= 1'b0;
        if (init_go) begin
            vol_pend <= 1'b1;
            vol_rdy  <= 1'b0;
            fp_pend  <= 1'b0;
            fade_lvl <= LVL_UNITY;
            fade_act <= 1'b0;
            db_mode  <= 1'b0;
            db_pipe  <= 2'd0;
            fs       <= FS_IDLE;
        end else begin
            // law-3 dB ramp pipeline: index -> LUT -> multiply -> level
            case (db_pipe)
                2'd1: begin db_lut_q <= db_lut(db_acc[21:16]); db_pipe <= 2'd2; end
                2'd2: begin db_prod  <= db_start * db_lut_q;   db_pipe <= 2'd3; end
                2'd3: begin
                    if (fade_act && db_mode) fade_lvl <= db_prod[38:15];
                    db_pipe <= 2'd0;
                end
                default: ;
            endcase
            // law 3 (Neo Geo MAKOTO driver, measured): a fade command while a
            // fade-down is already running is ignored -- the driver stores the
            // speed and keeps its accumulator; the KO sends 66 of them and the
            // curve equals a single command's.  Restarting the ramp from the
            // current level on every repeat would never reach silence.
            if (fade_trig && !law3_busy) begin
                fp_pend    <= 1'b1;
                fp_restore <= 1'b0;
                fp_stop0   <= fade_stop_at0;
                fp_arg     <= fade_arg;
                fp_tgt     <= {tgt_q15[15:0], 8'd0};
            end else if (restore_trig) begin
                fp_pend    <= 1'b1;
                fp_restore <= 1'b1;
                fp_stop0   <= 1'b0;
                fp_tgt     <= LVL_UNITY;
            end
            case (fs)
                FS_IDLE: if (vol_pend && !dv_bsy) begin
                    dv_num_in <= {4'd0, gp_r, 14'd0};
                    dv_den_in <= 32'd16129;             // 127*127
                    dv_go     <= 1'b1;
                    fs        <= FS_VOL;
                end else if (fp_pend && !dv_bsy) begin
                    fp_pend    <= 1'b0;
                    fade_tgt   <= fp_tgt;
                    fade_up    <= (fp_tgt > fade_lvl);
                    fade_stop0 <= fp_stop0;
                    if (fp_restore) begin
                        frames_r <= RESTORE_FRAMES;
                        fs       <= FS_SETUP;
                    end else if (fade_law != 2'd0) begin
                        dv_num_in <= fade_const1;
                        dv_den_in <= {16'd0, fp_arg};
                        dv_go     <= 1'b1;
                        fs        <= FS_LAW;
                    end else begin
                        frames_r <= 32'd1;              // law 0: immediate
                        fs       <= FS_SETUP;
                    end
                end
                FS_VOL: if (!dv_bsy && !dv_go) begin
                    vol_q14  <= dv_q[14:0];
                    vol_rdy  <= 1'b1;
                    vol_pend <= 1'b0;
                    fs       <= FS_IDLE;
                end
                FS_LAW: if (!dv_bsy && !dv_go) begin
                    // law 1: frames = const1/arg; law 2: x const2;
                    // law 3: const2 + const1/arg (Neo Geo MAKOTO speed byte)
                    if (fade_law == 2'd2)
                        frames_r <= law_q16 * fade_const2[15:0];
                    else if (fade_law == 2'd3)
                        frames_r <= dv_q + fade_const2;
                    else
                        frames_r <= (dv_q == 32'd0) ? 32'd1 : dv_q;
                    fs <= FS_SETUP;
                end
                FS_SETUP: if (!dv_bsy) begin
                    // dB ramp: Q6.16 index step = (63 << 16) / frames
                    dv_num_in <= db_sel ? 32'h003f_0000 : {8'd0, step_delta};
                    dv_den_in <= (frames_r == 32'd0) ? 32'd1 : frames_r;
                    dv_go     <= 1'b1;
                    fs        <= FS_STEP;
                end
                FS_STEP: if (!dv_bsy && !dv_go) begin
                    fade_step        <= dv_q[23:0];
                    db_mode          <= db_sel;
                    db_step          <= dv_q[23:0];
                    db_acc           <= 24'd0;
                    db_start         <= fade_lvl;
                    db_pipe          <= 2'd0;
                    fade_frames_left <= (frames_r == 32'd0) ? 32'd1 : frames_r;
                    fade_act         <= 1'b1;
                    fs               <= FS_IDLE;
                end
                default: fs <= FS_IDLE;
            endcase
            // level stepping, one step per frame tick
            if (fade_act && cen_frame && !osd_pause) begin
                if (fade_frames_left <= 32'd1) begin
                    fade_lvl <= fade_tgt;
                    fade_act <= 1'b0;
                    db_mode  <= 1'b0;
                    // faded out: end the track -- only for fade_out /
                    // master_fade.  A fade_keep that lands on 0 stays
                    // playing at level 0 (muted) until fade-up / restore.
                    if (fade_tgt == 24'd0 && fade_stop0)
                        auto_stop <= 1'b1;
                end else begin
                    fade_frames_left <= fade_frames_left - 32'd1;
                    if (db_mode) begin
                        db_acc  <= db_acc + db_step;
                        db_pipe <= 2'd1;
                    end else
                        fade_lvl <= fade_up ? fade_lvl + fade_step
                                            : fade_lvl - fade_step;
                end
            end
        end
    end
end

// combined multiplier: unity in = unity out (16384 * 32768 >> 15 = 16384)
reg  [14:0] total_q14;
wire [15:0] lvl_q15   = fade_lvl[23:8];
wire [30:0] tot_full  = vol_q14 * lvl_q15;
always @(posedge clk) begin
    total_q14 <= tot_full[29:15];
end

// -------------------------------------------------- play state + output ----
reg  playing_r, mute_r;
assign playing = playing_r;

wire drained = playing_r && fempty && feed_done && !codec_busy;

always @(posedge clk) begin
    if (rst || init_go) begin
        playing_r  <= 1'b0;
        mute_r     <= init_go;       // arm prefill on init, idle on rst
        track_done <= 1'b0;
    end else begin
        track_done <= 1'b0;
        if (stop || auto_stop || halt_p) begin
            playing_r <= 1'b0;
            mute_r    <= 1'b0;
            if (auto_stop) track_done <= 1'b1;
        end else if (mute_r) begin
            if (vol_rdy && (fused >= PREFILL || feed_done)) begin
                mute_r    <= 1'b0;
                playing_r <= 1'b1;
            end
        end else if (drained) begin
            playing_r  <= 1'b0;
            track_done <= 1'b1;
        end
    end
end

// ---------------------------------------------- loop crossfade blend stage --
// Head buffer (first N loop-body samples, captured on the first pass) and the
// Q15 equal-power weight table.  The blend runs as a 4-stage pipeline (window
// -> BRAM/ROM read -> products -> sum/clip); the output is only sampled on the
// cen_sample tick (~2000 clk96 apart), during which dpos/fq are frozen, so the
// pipeline always settles to the current head's blend before the tick — the
// added latency shifts nothing in the output stream.  Each multiply/add owns
// its own clk96 stage so no long combinational chain touches the audio path,
// and it never touches the memory address path.
reg  [15:0] xf_lut  [0:XFADE_N-1];
reg  [31:0] xf_head [0:XFADE_N-1];
initial if (XF_LUT_FILE != "") $readmemh(XF_LUT_FILE, xf_lut);

reg  [31:0] dpos;                 // stream sample index of the current FIFO head
reg         first_pass;           // head buffer not yet captured this loop pass
// The output mirrors the feed's wrap decisions deterministically: both count
// the same wraps in the same order, so no cross-pipeline handshake is needed.
reg  [1:0]  o_wraps_left;
wire        o_more_wraps = (cnt_r == 2'd0) || (o_wraps_left != 2'd0);
wire        xf_on   = xfen_r && loops;
wire [XF_AW-1:0] toff = dpos[XF_AW-1:0] - le_smp_r[XF_AW-1:0]; // tail index
wire [XF_AW-1:0] hoff = dpos[XF_AW-1:0] - ls_smp_r[XF_AW-1:0]; // head index
// no blend on the final pass of a finite-count track: those samples are the
// stream's own continuation into the outro, delivered raw
wire        in_tail = xf_on && o_more_wraps
                      && (dpos >= le_smp_r) && (dpos < le_hi_r);
wire        in_head = xf_on && first_pass
                      && (dpos >= ls_smp_r) && (dpos < ls_hi_r);
wire cap_we = cen_sample && playing_r && !osd_pause && !fempty && in_head;

// stage A: register the window flag and the (small) LUT/head indices
reg  [XF_AW-1:0] idx_a, midx_a;
reg  [31:0] tail_a;
reg         in_tail_a;
// stage B: registered ROM/BRAM reads (weights + head sample) + head capture
reg  [15:0] wo_b, wi_b;
reg  [31:0] head_b, tail_b;
reg         in_tail_b;
// stage C: registered products (infer DSP)
reg  signed [32:0] p_wo_l, p_wi_l, p_wo_r, p_wi_r;
reg         in_tail_c;
// stage D: registered sum + round + arithmetic shift + clip
reg  signed [15:0] blend_l, blend_r;
reg         in_tail_d;

wire signed [34:0] acc_l = p_wo_l + p_wi_l + 35'sd16384;
wire signed [34:0] acc_r = p_wo_r + p_wi_r + 35'sd16384;
wire signed [19:0] sh_l  = acc_l >>> 15;
wire signed [19:0] sh_r  = acc_r >>> 15;

always @(posedge clk) begin
    // A -----------------------------------------------------------------
    idx_a     <= toff;
    midx_a    <= (XFADE_N-1) - toff;      // mirror index: w_in(k) = lut[N-1-k]
    tail_a    <= fq;
    in_tail_a <= in_tail;
    // B -----------------------------------------------------------------
    wo_b      <= xf_lut[idx_a];
    wi_b      <= xf_lut[midx_a];
    head_b    <= xf_head[idx_a];
    tail_b    <= tail_a;
    in_tail_b <= in_tail_a;
    if (cap_we) xf_head[hoff] <= fq;       // head[k] = decoded [loop_start+k]
    // C -----------------------------------------------------------------
    p_wo_l    <= $signed({1'b0, wo_b}) * $signed(tail_b[15:0]);
    p_wi_l    <= $signed({1'b0, wi_b}) * $signed(head_b[15:0]);
    p_wo_r    <= $signed({1'b0, wo_b}) * $signed(tail_b[31:16]);
    p_wi_r    <= $signed({1'b0, wi_b}) * $signed(head_b[31:16]);
    in_tail_c <= in_tail_b;
    // D -----------------------------------------------------------------
    // out[k] = (w_out(k)*tail + w_in(k)*head + 2^14) >>> 15, clipped to s16
    // clip to s16.  The minimum is written as a 16-bit hex literal (0x8000 =
    // -32768); the decimal form -16'sd32768 first builds 16'sd32768, which
    // overflows 16-bit signed (max +32767) -> Quartus Warning 10259 and a
    // fragile two's-complement wrap.  0x8000 fills the field exactly.
    blend_l   <= (sh_l >  20'sd32767) ?  16'sd32767 :
                 (sh_l < -20'sd32768) ?  16'sh8000  : sh_l[15:0];
    blend_r   <= (sh_r >  20'sd32767) ?  16'sd32767 :
                 (sh_r < -20'sd32768) ?  16'sh8000  : sh_r[15:0];
    in_tail_d <= in_tail_c;
end

// combined multiplier: unity in = unity out (16384 * 32768 >> 15 = 16384).
// The head pair passes straight through except in the crossfade tail, where
// the pipelined blend (registered, settled between the ~2000-clk sample ticks)
// replaces it — so trk_xfade_en=0 is bit-identical to the plain hard-cut loop.
// The output-select mux is REGISTERED here (fl_q/fr_q) so it does not share a
// clk96 stage with the volume multiply below; the multiply then matches the
// stock output path.  All mux inputs (blend_l/r, fq, in_tail_d) are registered.
reg  signed [15:0] fl_q, fr_q;
always @(posedge clk) begin
    fl_q <= in_tail_d ? blend_l : fq[15:0];
    fr_q <= in_tail_d ? blend_r : fq[31:16];
end
wire signed [31:0] mul_l = fl_q * $signed({1'b0, total_q14});
wire signed [31:0] mul_r = fr_q * $signed({1'b0, total_q14});

always @(posedge clk) begin
    if (rst || init_go || stop || auto_stop || halt_p || track_done) begin
        audio_l    <= 16'd0;
        audio_r    <= 16'd0;
        sample_vld <= 1'b0;
        if (rst || init_go) begin
            frd        <= {FIFO_AW+1{1'b0}};
            dpos       <= 32'd0;
            first_pass <= 1'b1;
            o_wraps_left <= cnt_r;   // cnt_r latched at start, stable by init_go
        end
    end else begin
        sample_vld <= 1'b0;
        if (cen_sample) begin
            if (playing_r && !osd_pause && !fempty && !head_fresh) begin
                frd        <= frd + {{FIFO_AW{1'b0}}, 1'b1};
                audio_l    <= mul_l[29:14];
                audio_r    <= mul_r[29:14];
                sample_vld <= 1'b1;
                // track the stream position of the FIFO head; the crossfade
                // wrap mirrors the feed (loop_end+N -> loop_start+N) using the
                // boundaries precomputed at track load
                if (xf_on && dpos == wrap_thr_r && o_more_wraps) begin
                    dpos       <= ls_hi_r;
                    first_pass <= 1'b0;
                    if (cnt_r != 2'd0) o_wraps_left <= o_wraps_left - 2'd1;
                end else
                    dpos <= dpos + 32'd1;
            end else begin
                audio_l <= 16'd0;
                audio_r <= 16'd0;
            end
        end
    end
end

// ------------------------------------------------- CPSPLUS_DBG read-out ----
// Why did the music stop?  Four sticky bits, cleared when a track (re)starts:
// the overlay shows them next to `playing`, which separates "the trigger sent
// a stop/fade verb", "the decoder hit an EOF-pattern frame (bad data)" and
// "the feed is simply stuck" without a logic analyser.
reg [3:0] dbg_end_r;
always @(posedge clk) begin
    if (rst || init_go)
        dbg_end_r <= 4'd0;
    else begin
        if (adx_sel && adx_eof)          dbg_end_r[3] <= 1'b1;
        if (fade_trig && fade_loop_off)  dbg_end_r[2] <= 1'b1;
        if (auto_stop)                   dbg_end_r[1] <= 1'b1;
        if (stop)                        dbg_end_r[0] <= 1'b1;
    end
end
assign dbg_fst    = fst;
assign dbg_end    = dbg_end_r;
assign dbg_fempty = fempty;

endmodule
