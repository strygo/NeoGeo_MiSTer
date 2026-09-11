/* CPS+ — CRI ADX frame decoder.

    Decodes header-stripped ADX frame streams as stored in .cpk packs
    (PACK_FORMAT.md §Binary layout v0): 18-byte frames = 2-byte big-endian
    scale + 16 nibble bytes = 32 samples per channel; stereo streams
    interleave one L frame then one R frame (36-byte frame group).

    Arithmetic is the ffmpeg convention, matched bit-exactly against
    pack/adxcodec.py py_decode (scale, NOT scale+1; see adxcodec.py notes):

        d    = nibble, signed 4-bit (high nibble of each byte first)
        s0   = d*scale + ((c1*s1 + c2*s2) >>> 12)      // one shift, on the sum
        s1'  = clip16(s0);  s2' = s1                   // both taps post-clip
        out  = clip16(s0)

    Predictor coefficients c1/c2 come precomputed from the pack track index
    (ffmpeg lrint variants) — this module never parses stream headers.

    Predictor history is exposed for the player's loop snapshot/restore:
    hist_out is the running {s1_l,s2_l,s1_r,s2_r}; pulsing hist_load while
    !busy overwrites it with hist_in.  A frame whose scale word has the MSB
    set (CRI EOF frame — packs are validated not to contain them) is
    consumed without emitting samples and flagged on `eof`.

    Throughput: 18/36 clocks to collect a frame group, then 3 clocks per
    output sample pair minimum (CALC_L/CALC_R/OUT) — ~150 clocks per 32
    stereo samples, orders of magnitude faster than any audio rate.
*/

module cpsplus_adx(
    input                    rst,
    input                    clk,
    input                    clr,       // sync clear: history + frame assembly
    input                    stereo,    // 0 = mono (pcm_r mirrors pcm_l)
    input  signed [15:0]     c1,        // predictor coefficients (track index)
    input  signed [15:0]     c2,
    // header-stripped ADX frame byte stream
    input         [ 7:0]     din,
    input                    din_valid,
    output                   din_ready,
    // decoded PCM sample pairs
    output reg signed [15:0] pcm_l,
    output reg signed [15:0] pcm_r,
    output                   pcm_valid,
    input                    pcm_ready,
    // predictor history snapshot/restore (loop support)
    output        [63:0]     hist_out,  // {s1_l, s2_l, s1_r, s2_r}
    input         [63:0]     hist_in,
    input                    hist_load, // assert only while !busy
    output                   busy,      // frame group in flight
    output reg               eof        // EOF frame consumed (1-clk pulse)
);

// CALC_L/CALC_R are pipeline stage A (latch the products); CALC_L2/CALC_R2
// are stage B (sum + shift + clip + writeback).  Splitting the c1*s1+c2*s2
// (+d*scale) datapath across two clocks keeps the 96 MHz path short; the
// decode has ~2000 clk per sample so the extra states are free.
localparam [2:0] COLLECT = 3'd0,
                 CALC_L  = 3'd1,
                 CALC_L2 = 3'd2,
                 CALC_R  = 3'd3,
                 CALC_R2 = 3'd4,
                 OUT     = 3'd5;

reg  [2:0] st;
reg signed [31:0] p1_q, p2_q;   // stage-A product registers
reg signed [21:0] resid_q;      // stage-A residual register
reg  [5:0] cnt;                 // bytes collected in the current group
reg  [4:0] smp;                 // sample index within the frame, 0..31
reg  [7:0] fbuf[0:35];          // one frame group: L frame (+ R frame)

reg  signed [15:0] s1_l, s2_l, s1_r, s2_r;

wire [5:0] fg_bytes = stereo ? 6'd36 : 6'd18;
wire       last_byte = (cnt == fg_bytes-6'd1);
// scale MSB set = CRI EOF/dummy frame (either channel)
wire       eof_grp = fbuf[0][7] | (stereo & fbuf[18][7]);

assign din_ready = (st == COLLECT);
assign pcm_valid = (st == OUT);
assign busy      = (st != COLLECT) || (cnt != 6'd0);
assign hist_out  = {s1_l, s2_l, s1_r, s2_r};

// ------------------------------------------------------------- datapath ----
wire               ch_r    = (st == CALC_R);
wire        [15:0] scale   = ch_r ? {fbuf[18], fbuf[19]} : {fbuf[0], fbuf[1]};
wire        [ 5:0] nib_idx = (ch_r ? 6'd20 : 6'd2) + {2'd0, smp[4:1]};
wire        [ 7:0] nib_b   = fbuf[nib_idx];
wire        [ 3:0] nib     = smp[0] ? nib_b[3:0] : nib_b[7:4]; // high first
wire signed [ 4:0] d       = {nib[3], nib};                    // sign extend
wire signed [15:0] s1_sel  = ch_r ? s1_r : s1_l;
wire signed [15:0] s2_sel  = ch_r ? s2_r : s2_l;

// stage A: the three products (registered in CALC_L / CALC_R)
wire signed [31:0] p1      = c1 * s1_sel;
wire signed [31:0] p2      = c2 * s2_sel;
wire signed [21:0] resid   = d * $signed({1'b0, scale});
// stage B: consumed in CALC_L2 / CALC_R2 from the registered products
wire signed [31:0] pred    = p1_q + p2_q;
wire signed [19:0] pshift  = pred[31:12];                      // >>> 12
wire signed [22:0] s0      = resid_q + pshift;
wire signed [15:0] s0_clip = (s0 > 23'sd32767)  ? 16'sh7fff :
                             (s0 < -23'sd32768) ? 16'sh8000 : s0[15:0];

// ------------------------------------------------------------------ FSM ----
always @(posedge clk) begin
    if (rst) begin
        st   <= COLLECT;
        cnt  <= 6'd0;
        smp  <= 5'd0;
        eof  <= 1'b0;
        s1_l <= 16'd0; s2_l <= 16'd0;
        s1_r <= 16'd0; s2_r <= 16'd0;
    end else if (clr) begin
        st   <= COLLECT;
        cnt  <= 6'd0;
        smp  <= 5'd0;
        eof  <= 1'b0;
        s1_l <= 16'd0; s2_l <= 16'd0;
        s1_r <= 16'd0; s2_r <= 16'd0;
    end else begin
        eof <= 1'b0;
        case (st)
            COLLECT: if (din_valid) begin
                fbuf[cnt] <= din;
                if (last_byte) begin
                    cnt <= 6'd0;
                    // fbuf[0]/fbuf[18] were stored on earlier cycles; the
                    // byte arriving now is never a scale byte
                    if (eof_grp)
                        eof <= 1'b1;      // consume group, emit nothing
                    else begin
                        st  <= CALC_L;
                        smp <= 5'd0;
                    end
                end else
                    cnt <= cnt + 6'd1;
            end
            CALC_L: begin                 // stage A: latch L products
                p1_q <= p1; p2_q <= p2; resid_q <= resid;
                st   <= CALC_L2;
            end
            CALC_L2: begin                // stage B: sum/shift/clip, write L
                pcm_l <= s0_clip;
                if (!stereo) pcm_r <= s0_clip;
                s2_l  <= s1_l;
                s1_l  <= s0_clip;
                st    <= stereo ? CALC_R : OUT;
            end
            CALC_R: begin                 // stage A: latch R products
                p1_q <= p1; p2_q <= p2; resid_q <= resid;
                st   <= CALC_R2;
            end
            CALC_R2: begin                // stage B: sum/shift/clip, write R
                pcm_r <= s0_clip;
                s2_r  <= s1_r;
                s1_r  <= s0_clip;
                st    <= OUT;
            end
            OUT: if (pcm_ready) begin
                if (smp == 5'd31)
                    st <= COLLECT;
                else begin
                    smp <= smp + 5'd1;
                    st  <= CALC_L;
                end
            end
        endcase
        // restore wins over any decode-state update; only legal while !busy
        if (hist_load)
            {s1_l, s2_l, s1_r, s2_r} <= hist_in;
    end
end

endmodule
