/* CPS+ — cpsplus_ddr: pack loader + DDR memory backend (v0)
    =========================================================

    Owns the single MiSTer DDRAM-style read port of the CPS+ stack and
    provides three services (rtl/DDR.md is the companion document):

    1. BOOT LOADER.  On `boot_go` it resolves the pack base address
       (optionally dereferencing the MRA image header, see below), reads
       the 4 KB pack header (PACK_FORMAT.md §Binary layout v0), checks the
       "CP2A" magic — on mismatch it stays disabled / fails open — then
       parses the section offsets, protocol descriptor and control-verb
       map into the cpsplus_trigger config bank, streams the trigger
       table into the trigger BRAM, streams the track index into an
       internal index BRAM, and only then writes MODE.enable (LAST, per
       rtl/README.md §Config / table loading).  Fade law + constants are
       latched for the player.

    2. VERB SERVICE.  Trigger evt_* events map to the player control
       ports (PLAYER.md §Open items verb mapping):
         verb 1 play        index BRAM lookup -> trk_* regs -> start
         verb 2 stop        stop pulse
         verb 3 fade_out    fade_trig + fade_loop_off + fade_stop_at0
         verb 4 fade_keep   fade_trig            (a fade that lands on level 0
                                                  keeps the track playing muted)
         verb 5 restore     restore_trig
         verb 6 master_fade fade_trig + fade_stop_at0, target forced to 0
       Fade target = min(evt_argb*4, 127) (PS2 arg-byte x4 clamp,
       render_route.py fade_target); fade_arg = evt_argw.  Play latches
       trk_addr = pack_base + header.data_offset + index.data_offset —
       the player's stream addresses are absolute DDR byte addresses.

    3. PLAYER MEMORY BACKEND.  Implements the PLAYER.md memory-port
       contract (rd held with stable addr until a single-cycle ack with
       the little-endian 64-bit word, one outstanding) with a 128-byte
       ping-pong prefetcher: two 64 B buffers; a miss fetches an 8-word
       burst from the requested address and acks on the first beat; once
       the player consumes past the middle of a buffer, the other buffer
       prefetches the next 64 B in the background, so sequential
       streaming (the player's pattern — it only jumps at loop wraps and
       restarts) never waits on DDR.

    Pack base resolution (delivery per ASSESSMENT §6.3 / jtcps2 research
    §4: the pack rides the MRA ROM image, which stock MiSTer main writes
    to DDR at 0x30000000 where it persists after boot):
      base_indirect = 0: base_addr IS the pack base (bench / F-load).
      base_indirect = 1: base_addr is the ROM image base (0x30000000);
        the loader reads image header bytes 8-9 (a reserved slot of the
        CPS MRA start-pointer header, little-endian, 1 kB units) and uses
        pack_base = base_addr + (ptr << 10).  ptr 0x0000/0xFFFF (the MRA
        header fill values) = no pack appended -> stay disabled.

    Arbitration: ONE DDRAM master is presented upstream.  Boot loading
    and player fetches never overlap: boot stops the player, waits for an
    in-flight prefetch burst to finish, and only then claims the burst
    engine.  A player miss during loading (only reachable in the stop
    window of a pack switch) is answered with a dummy zero ack so the
    player's stop-drain cannot deadlock; the stop discards the data.

    OSD: MODE.enable follows `osd_en`.  The pack stays loaded when the
    user toggles the feature off — re-enabling is one config write;
    disabling also stops the player.  Reset default: disabled = stock.

    DDRAM protocol (as jtframe_mister_dwnld.v / jtframe_lfbuf_ddr_ctrl.v):
    assert ddram_rd with ddram_addr/ddram_burstcnt and hold while
    ddram_busy; the request is accepted on the first !busy cycle, then
    exactly burstcnt ddram_dout_ready beats deliver the data.
    ddram_addr is the 64-bit-word address (byte address >> 3).

    Verilog-2005, no vendor primitives.  BRAM: track index
    2**(TRK_AW+2) x 64 (4 KB at the default 128 tracks).
*/

module cpsplus_ddr #(parameter
    TRIG_AW   = 13,        // trigger table load port address width
    TRIG_ROWS = 4608,      // trigger BRAM rows (v0 packs: 0x1200)
    TRK_AW    = 7,         // supported tracks = 2**TRK_AW
    RETRY_W   = 23,        // self-heal retry period: 2**RETRY_W clks (~87 ms)
    // Mirror the measured CPS1 driver behaviour: a music command identical to
    // the song already playing is IGNORED (Final Fight re-sends its stage-1
    // command 0x28 ~2 s apart during play and the YM2151 shows no song-start
    // burst either time -- work/capture/ffight_ym.tsv).  Without this the
    // arranged track restarts on every re-send.  A DIFFERENT track always
    // starts, and a STOP re-arms.  Off by default: only measured on CPS1.
    SAME_SONG = 0
)(
    input             rst,
    input             clk,

    // control / status
    input      [31:0] base_addr,     // pack (or MRA image) DDR byte address,
                                     // 16 B aligned (0x30000000 in practice)
    input             base_indirect, // 1 = dereference MRA header bytes 8-9
    input             osd_en,        // OSD "arranged audio" enable
    input             boot_go,       // pulse: (re)load the pack
    output reg        ready,         // pack loaded (tables in place)
    output reg        magic_ok,
    output reg [ 3:0] status,        // 0 idle, 1 loading, 2 ok, 4 no pack,
                                     // 5 bad magic, 6 header out of range

    // trigger verb events in
    input             evt_stb,
    input      [ 2:0] evt_verb,
    input      [11:0] evt_track,
    input      [ 6:0] evt_gain,
    input      [15:0] evt_argw,
    input      [ 7:0] evt_argb,

    // trigger config + table load ports
    output reg        cfg_we,
    output reg [ 7:0] cfg_addr,
    output reg [15:0] cfg_data,
    output reg        trig_we,
    output reg [TRIG_AW-1:0] trig_addr,
    output reg [31:0] trig_data,

    // player control
    output reg        pl_start,
    output reg        pl_stop,
    output reg [31:0] trk_addr,
    output reg [31:0] trk_len,
    output reg [31:0] trk_lstart,
    output reg [31:0] trk_lend,
    output reg [31:0] trk_lstart_smp,   // loop points in samples (crossfade)
    output reg [31:0] trk_lend_smp,
    output reg        trk_xfade_en,     // per-track loop crossfade enable
    output reg [ 1:0] trk_loop_cnt,     // 0 = loop forever; 1-3 = wrap N times,
                                        // then play THROUGH loop_end to the end
    output reg        trk_stereo,
    output reg        trk_codec,
    output reg [ 6:0] trk_gain,
    output reg [15:0] trk_c1,
    output reg [15:0] trk_c2,
    output reg [15:0] trk_rate,      // Hz — target layer programs the frac cen
    output reg [ 6:0] trig_gain,
    output reg [ 1:0] fade_law,
    output reg [31:0] fade_const1,
    output reg [31:0] fade_const2,
    output reg        fade_trig,
    output reg        fade_loop_off,
    output reg        fade_stop_at0,     // with fade_trig: a fade reaching level 0
                                        // ENDS the track (verbs 3/6); verb 4 keeps
                                        // it playing muted so a later fade-up /
                                        // restore brings it back bit-exactly
    output reg        restore_trig,
    output reg [ 6:0] fade_target,
    output reg [15:0] fade_arg,

    // player memory port (backend side of the PLAYER.md contract)
    input             pmem_rd,
    input      [31:3] pmem_addr,
    output reg [63:0] pmem_data,
    output reg        pmem_ack,

    // MiSTer DDRAM master
    input             ddram_busy,
    output reg [ 7:0] ddram_burstcnt,
    output reg [28:0] ddram_addr,
    input      [63:0] ddram_dout,
    input             ddram_dout_ready,
    output reg        ddram_rd
);

localparam [31:0] MAGIC  = 32'h4132_5043;   // "CP2A" little-endian
localparam        MAXTRK = 1 << TRK_AW;
localparam [TRIG_AW:0] ROWS_W   = TRIG_ROWS;
localparam [11:0]      MAXTRK_W = MAXTRK;

// ------------------------------------------------------ DDR burst engine ---
// One requester at a time: the boot FSM (bt_go, while `loading`) or the
// prefetcher (pf_go).  Each returned word raises dd_beat for 1 clk with
// the data in dd_data and its in-burst index in dd_bidx.
reg         bt_go, pf_go;
reg  [31:3] bt_addr, pf_addr;
reg  [31:3] trig_base;   // pack_base[31:3]+h_trig_off[31:3], precomputed once
                         // (both constant during the load) so bt_addr is a
                         // single add on clk96 instead of a 3-operand chain
reg  [31:3] idx_base;    // pack_base[31:3]+h_idx_off[31:3], precomputed the same
                         // way so the I_IDX_GO bt_addr is also a single clk96 add
reg  [ 7:0] bt_len;
reg         dd_run;
reg  [ 7:0] dd_bcnt, dd_bidx;
reg         dd_beat;
reg  [63:0] dd_data;

always @(posedge clk) begin
    if (rst) begin
        ddram_rd <= 1'b0;
        dd_run   <= 1'b0;
        dd_beat  <= 1'b0;
    end else begin
        dd_beat <= 1'b0;
        if (bt_go || pf_go) begin
            ddram_addr     <= bt_go ? bt_addr : pf_addr;
            ddram_burstcnt <= bt_go ? bt_len  : 8'd8;
            ddram_rd       <= 1'b1;
            dd_bcnt        <= 8'd0;
            dd_run         <= 1'b1;
        end else if (ddram_rd) begin
            if (!ddram_busy) ddram_rd <= 1'b0;      // request accepted
        end else if (dd_run && ddram_dout_ready) begin
            dd_data <= ddram_dout;
            dd_bidx <= dd_bcnt;
            dd_beat <= 1'b1;
            dd_bcnt <= dd_bcnt + 8'd1;
            if (dd_bcnt == ddram_burstcnt - 8'd1)
                dd_run <= 1'b0;
        end
    end
end

// ------------------------------------------------------------- index BRAM --
// One 32 B track entry = 4 x 64-bit words at {track, word[1:0]}.
reg [63:0]       idx_mem [0:(MAXTRK*4)-1];
reg              idx_we;
reg [TRK_AW+1:0] idx_wa, idx_ra;
reg [63:0]       idx_wd, idx_q;

always @(posedge clk) begin
    if (idx_we) idx_mem[idx_wa] <= idx_wd;
    idx_q <= idx_mem[idx_ra];
end

// ------------------------------------------------------------- boot FSM ----
localparam [3:0] I_IDLE   = 4'd0,  I_STOPW    = 4'd1,  I_PTR_W  = 4'd2,
                 I_HDR_GO = 4'd3,  I_HDR_W    = 4'd4,  I_CFG    = 4'd5,
                 I_TRIG_GO= 4'd6,  I_TRIG_W   = 4'd7,  I_TRIG_WR= 4'd8,
                 I_TRIG_FIL=4'd9,  I_IDX_GO   = 4'd10, I_IDX_W  = 4'd11,
                 I_MODE   = 4'd12, I_DONE     = 4'd13, I_FAIL   = 4'd14;

reg  [ 3:0] ist;
reg         loading;
reg         osd_cur;             // enable value last written to MODE
reg         bt_stop;             // stop request towards the player
reg  [31:0] pack_base;
reg  [31:0] data_base;           // pack_base + header.data_offset

// captured header fields
reg  [31:0] h_trig_off, h_trig_rows, h_idx_off, h_trk_cnt, h_data_lo, h_data_hi;
reg  [23:0] h_page;
reg  [ 7:0] h_off_cmd_hi, h_off_cmd_lo, h_off_arg_hi, h_off_arg_lo;
reg  [ 7:0] h_off_argb, h_off_hs, h_pending, h_ready;
reg  [ 7:0] h_law, h_ctrl_dflt;
reg  [15:0] h_ctrl_start;
reg  [31:0] h_nverbs;
reg  [63:0] vbuf [0:15];         // 32 control-verb entries, 2 per word
// format v2 (Neo Geo dialect): 0x12a dialect byte, 0x130..0x14f argument-
// command bitmap (256 bits, LE).  Emitted to cfg 0x20..0x2f ONLY when the
// dialect is 2: the CPS taps alias every cfg address into their 0x00..0x07
// bank, so a v0/v1 pack must never see those writes.
reg  [ 7:0] h_dialect;
reg  [63:0] abuf [0:3];

// chunk buffer for the trigger table stream (32 words = 64 rows)
reg  [63:0] cbuf [0:31];
reg  [ 6:0] wr_k;                // row unload counter, 0..2*chunk-1
reg  [11:0] tw_done, tw_total;   // trigger words streamed / total
reg  [ 5:0] chunk;               // words in the current chunk
reg  [13:0] iw_done, iw_total;   // index words streamed / total
reg  [ 6:0] cfg_c;               // config write sequencer
reg  [TRIG_AW:0] fil_a;          // trigger zero-fill row counter
reg  [ 7:0] hw_left;             // header beats outstanding
reg  [11:0] cur_track;           // track currently playing (SAME_SONG guard)
reg         cur_valid;

wire [15:0] ptr16    = dd_data[15:0];
wire [11:0] tw_pack  = h_trig_rows[12:1];               // pack rows / 2
wire [11:0] tw_clamp = tw_pack > TRIG_ROWS/2 ? TRIG_ROWS/2 : tw_pack;
wire [11:0] tw_rem   = tw_total - tw_done;
wire [13:0] iw_rem   = iw_total - iw_done;
wire [13:0] iw_want  = h_trk_cnt >= MAXTRK ? MAXTRK*4
                                           : {h_trk_cnt[TRK_AW-1:0], 2'b00};

// control-map entry select for I_CFG (cfg_c = 7..70 -> map_k = 0..63)
wire [ 5:0] map_k   = cfg_c[5:0] - 6'd7;
wire [ 4:0] map_i   = map_k[5:1];
wire [63:0] map_w   = vbuf[map_i[4:1]];
wire [31:0] map_e   = map_i[0] ? map_w[63:32] : map_w[31:0];
wire        map_val = {27'd0, map_i} < h_nverbs;
// argset word select for I_CFG (cfg_c = 71..86 -> arg_k = 0..15)
wire [ 3:0] arg_k   = cfg_c[3:0] - 4'd7;
wire [63:0] arg_w64 = abuf[arg_k[3:2]];
wire [15:0] arg_w   = arg_w64[{arg_k[1:0], 4'd0} +: 16];

wire        pf_busy;             // prefetch burst pending/in flight

wire [15:0] mode_w  = {9'd0, h_ctrl_dflt[2:0], 3'b000, osd_en};

// Self-heal retry.  boot_go is a 1-clk pulse from the game (ioctl_rom falling
// edge) and it CANNOT be relied upon: on real hardware the core asserts its
// startup reset AFTER the download completes, so the loader starts, is reset
// back to idle, and the one-shot pulse never comes again -- the pack then never
// loads (measured on MiSTer: status latched non-zero, live status 0).  Rather
// than chase pulse-vs-reset ordering, retry from idle/fail until the pack is
// loaded: whenever the DDR image becomes valid, the next retry picks it up.
// Retries stop as soon as ready=1, so a working load costs nothing.  A reset
// after a good load clears ready and the retry re-arms automatically.
reg  [RETRY_W-1:0] retry_cnt;
reg                retry_go;
always @(posedge clk) begin
    retry_go <= 1'b0;
    if (rst) begin
        retry_cnt <= {RETRY_W{1'b0}};
    end else if (!ready && (ist==I_IDLE || ist==I_FAIL)) begin
        retry_cnt <= retry_cnt + 1'b1;
        if (&retry_cnt) retry_go <= 1'b1;      // fire as the counter wraps
    end else begin
        retry_cnt <= {RETRY_W{1'b0}};          // loading or loaded: hold off
    end
end

always @(posedge clk) begin
    if (rst) begin
        ist      <= I_IDLE;
        loading  <= 1'b0;
        ready    <= 1'b0;
        magic_ok <= 1'b0;
        status   <= 4'd0;
        osd_cur  <= 1'b0;
        cfg_we   <= 1'b0;
        trig_we  <= 1'b0;
        idx_we   <= 1'b0;
        bt_go    <= 1'b0;
        bt_stop  <= 1'b0;
        fade_law <= 2'd0;
        fade_const1 <= 32'd0;
        fade_const2 <= 32'd0;
    end else begin
        cfg_we  <= 1'b0;
        trig_we <= 1'b0;
        idx_we  <= 1'b0;
        bt_go   <= 1'b0;
        bt_stop <= 1'b0;

        if ((boot_go || retry_go) && (ist==I_IDLE || ist==I_DONE || ist==I_FAIL)) begin
            // disable the trigger first: stock behavior while loading
            cfg_we   <= 1'b1;
            cfg_addr <= 8'h07;
            cfg_data <= 16'h0000;
            bt_stop  <= 1'b1;
            loading  <= 1'b1;
            ready    <= 1'b0;
            magic_ok <= 1'b0;
            osd_cur  <= 1'b0;
            status   <= 4'd1;
            ist      <= I_STOPW;
        end else case (ist)
            I_STOPW: if (!pf_busy && !dd_run && !ddram_rd) begin
                // the burst engine is ours until I_DONE/I_FAIL
                if (base_indirect) begin
                    bt_go   <= 1'b1;
                    bt_addr <= {base_addr[31:4], 1'b1};  // word at byte +8
                    bt_len  <= 8'd1;
                    ist     <= I_PTR_W;
                end else begin
                    pack_base <= base_addr;
                    ist       <= I_HDR_GO;
                end
            end
            I_PTR_W: if (dd_beat) begin
                if (ptr16 == 16'h0000 || ptr16 == 16'hffff) begin
                    status  <= 4'd4;                 // no pack appended
                    loading <= 1'b0;
                    ist     <= I_FAIL;
                end else begin
                    pack_base <= base_addr + {6'd0, ptr16, 10'd0};
                    ist       <= I_HDR_GO;
                end
            end
            I_HDR_GO: begin
                bt_go   <= 1'b1;
                bt_addr <= pack_base[31:3];
                bt_len  <= 8'd42;                    // bytes 0x000..0x14f
                hw_left <= 8'd42;
                h_dialect <= 8'd0;
                ist     <= I_HDR_W;
            end
            I_HDR_W: if (dd_beat) begin
                hw_left <= hw_left - 8'd1;
                case (dd_bidx)
                    8'd0: if (dd_data[31:0] != MAGIC) begin
                        status  <= 4'd5;             // fail open
                        loading <= 1'b0;
                        ist     <= I_FAIL;
                    end else magic_ok <= 1'b1;
                    8'd11: h_trig_off <= dd_data[63:32];
                    8'd12: begin
                        h_trig_rows <= dd_data[31:0];
                        h_idx_off   <= dd_data[63:32];
                    end
                    8'd13: begin
                        h_trk_cnt <= dd_data[31:0];
                        h_data_lo <= dd_data[63:32];
                    end
                    8'd14: h_data_hi <= dd_data[31:0];
                    8'd17: h_page    <= dd_data[55:32];
                    8'd18: begin
                        h_off_cmd_hi <= dd_data[ 7: 0];
                        h_off_cmd_lo <= dd_data[15: 8];
                        h_off_arg_hi <= dd_data[23:16];
                        h_off_arg_lo <= dd_data[31:24];
                        h_off_argb   <= dd_data[39:32];
                        h_off_hs     <= dd_data[47:40];
                        h_pending    <= dd_data[55:48];
                        h_ready      <= dd_data[63:56];
                    end
                    8'd19: begin
                        h_law        <= dd_data[ 7: 0];
                        h_ctrl_dflt  <= dd_data[15: 8];
                        h_ctrl_start <= dd_data[31:16];
                        fade_const1  <= dd_data[63:32];
                    end
                    8'd20: begin
                        fade_const2 <= dd_data[31:0];
                        h_nverbs    <= dd_data[63:32];
                    end
                    8'd37: h_dialect <= dd_data[23:16];  // 0x12a (v2; 0 on v0/v1)
                    8'd38, 8'd39, 8'd40, 8'd41:          // 0x130..0x14f argset
                        abuf[dd_bidx[1:0] - 2'd2] <= dd_data;
                    default:
                        if (dd_bidx >= 8'd21 && dd_bidx <= 8'd36)
                            vbuf[dd_bidx[4:0] - 5'd21] <= dd_data;
                endcase
            end else if (hw_left == 8'd0) begin      // header consumed
                data_base <= pack_base + h_data_lo;
                trig_base <= pack_base[31:3] + h_trig_off[31:3];
                idx_base  <= pack_base[31:3] + h_idx_off[31:3];
                fade_law  <= h_law[1:0];
                if (h_data_hi != 32'd0) begin
                    status  <= 4'd6;                 // >4 GB: out of range
                    loading <= 1'b0;
                    ist     <= I_FAIL;
                end else begin
                    cfg_c <= 7'd0;
                    ist   <= I_CFG;
                end
            end
            I_CFG: begin
                cfg_we <= 1'b1;
                cfg_c  <= cfg_c + 7'd1;
                if (cfg_c < 7'd7) begin
                    cfg_addr <= {1'b0, cfg_c};
                    case (cfg_c[2:0])
                        3'd0: cfg_data <= h_page[15:0];
                        3'd1: cfg_data <= {8'd0, h_page[23:16]};
                        3'd2: cfg_data <= {3'd0, h_off_cmd_lo[4:0],
                                           3'd0, h_off_cmd_hi[4:0]};
                        3'd3: cfg_data <= {3'd0, h_off_arg_lo[4:0],
                                           3'd0, h_off_arg_hi[4:0]};
                        3'd4: cfg_data <= {3'd0, h_off_hs[4:0],
                                           3'd0, h_off_argb[4:0]};
                        3'd5: cfg_data <= {h_ready, h_pending};
                        default: cfg_data <= h_ctrl_start;
                    endcase
                end else if (cfg_c <= 7'd70) begin
                    cfg_addr <= 8'h40 + {2'd0, map_k};
                    cfg_data <= !map_val ? 16'h0000 :
                                map_k[0] ? {13'd0, map_e[18:16]} :
                                           map_e[15:0];
                    if (cfg_c == 7'd70 && h_dialect != 8'd2) begin
                        tw_done  <= 12'd0;
                        tw_total <= tw_clamp;
                        ist      <= I_TRIG_GO;
                    end
                end else begin
                    // v2 Neo Geo dialect only: argument-command bitmap
                    cfg_addr <= 8'h20 + {4'd0, arg_k};
                    cfg_data <= arg_w;
                    if (cfg_c == 7'd86) begin
                        tw_done  <= 12'd0;
                        tw_total <= tw_clamp;
                        ist      <= I_TRIG_GO;
                    end
                end
            end
            I_TRIG_GO: begin
                if (tw_done >= tw_total) begin
                    fil_a <= {tw_done, 1'b0};
                    ist   <= I_TRIG_FIL;
                end else begin
                    bt_go   <= 1'b1;
                    bt_addr <= trig_base + {17'd0, tw_done};
                    bt_len  <= tw_rem > 12'd32 ? 8'd32 : {2'd0, tw_rem[5:0]};
                    chunk   <= tw_rem > 12'd32 ? 6'd32 : tw_rem[5:0];
                    ist     <= I_TRIG_W;
                end
            end
            I_TRIG_W: if (dd_beat) begin
                cbuf[dd_bidx[4:0]] <= dd_data;
                if (dd_bidx == {2'd0, chunk} - 8'd1) begin
                    wr_k <= 7'd0;
                    ist  <= I_TRIG_WR;
                end
            end
            I_TRIG_WR: begin
                trig_we   <= 1'b1;
                trig_addr <= {tw_done, 1'b0} + {6'd0, wr_k};
                trig_data <= wr_k[0] ? cbuf[wr_k[5:1]][63:32]
                                     : cbuf[wr_k[5:1]][31:0];
                wr_k <= wr_k + 7'd1;
                if (wr_k == {chunk, 1'b0} - 7'd1) begin
                    tw_done <= tw_done + {6'd0, chunk};
                    ist     <= I_TRIG_GO;
                end
            end
            I_TRIG_FIL: begin
                if (fil_a >= ROWS_W) begin
                    iw_done  <= 14'd0;
                    iw_total <= iw_want;
                    ist      <= I_IDX_GO;
                end else begin
                    trig_we   <= 1'b1;
                    trig_addr <= fil_a[TRIG_AW-1:0];
                    trig_data <= 32'd0;
                    fil_a     <= fil_a + 1'd1;
                end
            end
            I_IDX_GO: begin
                if (iw_done >= iw_total)
                    ist <= I_MODE;
                else begin
                    bt_go   <= 1'b1;
                    bt_addr <= idx_base + {15'd0, iw_done};
                    bt_len  <= iw_rem > 14'd32 ? 8'd32 : {2'd0, iw_rem[5:0]};
                    chunk   <= iw_rem > 14'd32 ? 6'd32 : iw_rem[5:0];
                    ist     <= I_IDX_W;
                end
            end
            I_IDX_W: if (dd_beat) begin
                idx_we <= 1'b1;
                idx_wa <= iw_done[TRK_AW+1:0] + {4'd0, dd_bidx[4:0]};
                idx_wd <= dd_data;
                if (dd_bidx == {2'd0, chunk} - 8'd1) begin
                    iw_done <= iw_done + {8'd0, chunk};
                    ist     <= I_IDX_GO;
                end
            end
            I_MODE: begin
                // MODE.enable written LAST (trigger README contract)
                cfg_we   <= 1'b1;
                cfg_addr <= 8'h07;
                cfg_data <= mode_w;
                osd_cur  <= osd_en;
                ready    <= 1'b1;
                loading  <= 1'b0;
                status   <= 4'd2;
                ist      <= I_DONE;
            end
            I_DONE: if (osd_en != osd_cur) begin
                cfg_we   <= 1'b1;
                cfg_addr <= 8'h07;
                cfg_data <= mode_w;
                osd_cur  <= osd_en;
                if (!osd_en) bt_stop <= 1'b1;        // silence on disable
            end
            I_FAIL: ;                                // wait for boot_go
            default: ist <= I_IDLE;                  // I_IDLE
        endcase
    end
end

// ---------------------------------------------------- player mem backend ---
// Two 64 B buffers, ping-pong prefetch (see header comment).
reg  [63:0] bufA [0:7], bufB [0:7];
reg  [31:3] baseA, baseB;
reg  [ 3:0] fillA, fillB;
reg         vA, vB;
reg         pf_run;              // prefetch burst in flight
reg         pf_dst;              // 0 = filling A, 1 = filling B

wire [28:0] offA = pmem_addr - baseA;
wire [28:0] offB = pmem_addr - baseB;
wire hitA = vA && offA[28:3] == 26'd0 && {1'b0, offA[2:0]} < fillA;
wire hitB = vB && offB[28:3] == 26'd0 && {1'b0, offB[2:0]} < fillB;

assign pf_busy = pf_run || pf_go;

always @(posedge clk) begin
    if (rst) begin
        vA <= 1'b0;  vB <= 1'b0;
        fillA <= 4'd0;  fillB <= 4'd0;
        pf_run <= 1'b0;  pf_go <= 1'b0;
        pmem_ack <= 1'b0;
    end else begin
        pf_go    <= 1'b0;
        pmem_ack <= 1'b0;
        // burst fill (beats are ours whenever pf_run — boot never overlaps)
        if (pf_run && dd_beat) begin
            if (!pf_dst) begin
                bufA[dd_bidx[2:0]] <= dd_data;
                fillA <= {1'b0, dd_bidx[2:0]} + 4'd1;
            end else begin
                bufB[dd_bidx[2:0]] <= dd_data;
                fillB <= {1'b0, dd_bidx[2:0]} + 4'd1;
            end
            if (dd_bidx == 8'd7) pf_run <= 1'b0;
        end
        // request service (ack is a single cycle; the player drops rd after)
        if (pmem_rd && !pmem_ack) begin
            if (hitA) begin
                pmem_ack  <= 1'b1;
                pmem_data <= bufA[offA[2:0]];
                if (offA[2:0] >= 3'd4 && !pf_run && !pf_go && !loading
                    && (!vB || baseB != baseA + 29'd8)) begin
                    pf_go   <= 1'b1;                 // prefetch ahead into B
                    pf_addr <= baseA + 29'd8;
                    pf_dst  <= 1'b1;
                    baseB   <= baseA + 29'd8;
                    fillB   <= 4'd0;
                    vB      <= 1'b1;
                    pf_run  <= 1'b1;
                end
            end else if (hitB) begin
                pmem_ack  <= 1'b1;
                pmem_data <= bufB[offB[2:0]];
                if (offB[2:0] >= 3'd4 && !pf_run && !pf_go && !loading
                    && (!vA || baseA != baseB + 29'd8)) begin
                    pf_go   <= 1'b1;                 // prefetch ahead into A
                    pf_addr <= baseB + 29'd8;
                    pf_dst  <= 1'b0;
                    baseA   <= baseB + 29'd8;
                    fillA   <= 4'd0;
                    vA      <= 1'b1;
                    pf_run  <= 1'b1;
                end
            end else if (loading || !ready) begin
                pmem_ack  <= 1'b1;                   // stop-drain dummy word
                pmem_data <= 64'd0;
            end else if (!pf_run && !pf_go) begin
                pf_go   <= 1'b1;                     // miss: fetch A here
                pf_addr <= pmem_addr;
                pf_dst  <= 1'b0;
                baseA   <= pmem_addr;
                fillA   <= 4'd0;
                vA      <= 1'b1;
                vB      <= 1'b0;
                pf_run  <= 1'b1;
            end
        end
    end
end

// ------------------------------------------------------------ verb service -
localparam [2:0] E_IDLE = 3'd0, E_R0 = 3'd1, E_R1 = 3'd2, E_R2 = 3'd3,
                 E_R3   = 3'd4, E_C0 = 3'd5, E_GO = 3'd6;

reg  [ 2:0] est;
reg  [11:0] e_track;
reg  [ 6:0] e_gain;
reg         ev_stop;
reg  [63:0] w0, w1, w2;

wire [ 6:0] tgt4 = evt_argb >= 8'd32 ? 7'd127 : {evt_argb[4:0], 2'b00};

always @(posedge clk) begin
    if (rst) begin
        est          <= E_IDLE;
        pl_start     <= 1'b0;
        ev_stop      <= 1'b0;
        fade_trig    <= 1'b0;
        fade_loop_off<= 1'b0;
        fade_stop_at0<= 1'b0;
        restore_trig <= 1'b0;
        idx_ra       <= {TRK_AW+2{1'b0}};
        cur_valid    <= 1'b0;
        cur_track    <= 12'd0;
    end else begin
        pl_start     <= 1'b0;
        ev_stop      <= 1'b0;
        fade_trig    <= 1'b0;
        fade_loop_off<= 1'b0;
        fade_stop_at0<= 1'b0;
        restore_trig <= 1'b0;
        case (est)
            E_IDLE: if (evt_stb && ready) begin
                case (evt_verb)
                    // SAME_SONG: drop a PLAY for the track already playing
                    3'd1: if ({20'd0, evt_track} < {12'd0, h_trk_cnt[19:0]}
                              && evt_track < MAXTRK_W
                              && !(SAME_SONG!=0 && cur_valid && evt_track==cur_track)) begin
                        e_track <= evt_track;
                        e_gain  <= evt_gain;
                        idx_ra  <= {evt_track[TRK_AW-1:0], 2'd0};
                        est     <= E_R0;
                    end
                    3'd2: begin ev_stop <= 1'b1; cur_valid <= 1'b0; end  // STOP re-arms
                    3'd3: begin
                        fade_trig     <= 1'b1;
                        fade_loop_off <= 1'b1;
                        fade_stop_at0 <= 1'b1;
                        // law 3 (Neo Geo MAKOTO): the argument is the driver's
                        // speed byte, not a level -- the target is silence
                        fade_target   <= (fade_law == 2'd3) ? 7'd0 : tgt4;
                        fade_arg      <= evt_argw;
                    end
                    3'd4: begin
                        fade_trig   <= 1'b1;
                        fade_target <= tgt4;
                        fade_arg    <= evt_argw;
                    end
                    3'd5: restore_trig <= 1'b1;
                    3'd6: begin
                        fade_trig     <= 1'b1;
                        fade_stop_at0 <= 1'b1;
                        fade_target   <= 7'd0;
                        fade_arg      <= evt_argw;
                    end
                    default: ;
                endcase
            end
            E_R0: begin
                idx_ra <= {e_track[TRK_AW-1:0], 2'd1};
                est    <= E_R1;
            end
            E_R1: begin
                idx_ra <= {e_track[TRK_AW-1:0], 2'd2};
                w0     <= idx_q;                     // entry word 0
                est    <= E_R2;
            end
            E_R2: begin
                idx_ra <= {e_track[TRK_AW-1:0], 2'd3};
                w1     <= idx_q;                     // entry word 1
                est    <= E_R3;
            end
            E_R3: begin
                w2  <= idx_q;                        // entry word 2
                est <= E_C0;
            end
            E_C0: begin                              // idx_q = entry word 3
                trk_addr   <= data_base + w0[31:0];
                trk_len    <= w0[63:32];
                trk_lstart     <= w1[63:32];         // loop_start_byte  (0x0c)
                trk_lstart_smp <= w1[31:0];          // loop_start_sample(0x08)
                trk_lend       <= w2[63:32];         // loop_end_byte    (0x14)
                trk_lend_smp   <= w2[31:0];          // loop_end_sample  (0x10)
                trk_rate   <= idx_q[15:0];
                trk_stereo <= idx_q[17];             // channels==2
                trk_codec  <= idx_q[20];             // 0 ADX / 1 PCM
                trk_xfade_en <= idx_q[21];           // codec byte bit5
                trk_loop_cnt <= idx_q[23:22];        // codec byte bits6-7
                trk_gain   <= idx_q[30:24];
                trk_c1     <= idx_q[47:32];
                trk_c2     <= idx_q[63:48];
                trig_gain  <= e_gain;
                est        <= E_GO;
            end
            E_GO: begin
                pl_start  <= 1'b1;
                cur_track <= e_track;                 // remember what is playing
                cur_valid <= 1'b1;
                est       <= E_IDLE;
            end
            default: est <= E_IDLE;
        endcase
    end
end

// stop pulses from both the boot FSM (disable/pack switch) and verb 2
always @(posedge clk) pl_stop <= !rst && (bt_stop || ev_stop);

endmodule
