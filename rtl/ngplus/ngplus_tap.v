/*  NG+ — ngplus_tap: Neo Geo REG_SOUND byte-latch sniffer + gate (v0)
    ==================================================================

    The Neo Geo sound channel is one byte written by the 68K to REG_SOUND
    ($320000, upper lane) -> NEO-C1 latch -> Z80 NMI.  There is no shared
    RAM and no handshake.  Measured contract (ngplus/PHASE0_REPORT.md,
    Samurai Shodown II, MAKOTO 3.0 driver):

      * music commands are a fixed set of byte values (the pack's trigger
        table, 256 rows); SFX and system commands are other values;
      * some system commands consume the NEXT byte as an argument
        ($0a fade speed, $0e tempo, $14/$15 sample stop, $18-$1e sample
        start).  Argument values routinely fall in the music range, so the
        tap keeps an argument-pending state and never classifies an
        argument byte;
      * suppression = the Z80 must not see the byte: the game top withholds
        the NMI (and keeps the latch untouched) when `gate` is high for the
        write.  The game never polls the echo (SS2 measured), so no reply
        synthesis is needed; a descriptor hook remains for titles that do;
      * control verbs are explicit map entries and apply to ANY byte value
        (the descriptor's control_region_start is 0): $20 stop, $0a fade
        (its speed byte is the argument, reported as evt_argw/evt_argb),
        $11 restore.

    Interface: `wr_stb`/`wr_byte` are the synchronised 68K write (one clk
    strobe per REG_SOUND write, byte valid with it).  `gate` is a level that
    settles ~4 clk after the strobe and holds until the next strobe; the
    game top samples it at its delayed NMI point (16 clk48 after the write
    ends).  evt_* / cfg_* / trig_* ports are identical in meaning to
    cpsplus_cps1_tap / cpsplus_trigger so cpsplus_ddr drives and consumes
    them unchanged.

    Config (cfg_we, 16-bit words):
      0x07  MODE  bit0 enable, bit2 nogate (observe-only), bits6:4 default
            control verb (unused here: unmapped bytes are SFX)
      0x20..0x2f  argument-command bitmap, 16 words little-endian
            (bit c set = command c consumes the next byte); pack format v2
            header 0x130..0x14f, emitted by cpsplus_ddr for dialect 2 only
      0x40+  control-verb map pairs (cmd word, verb word)

    Verilog-2005, no vendor primitives.
*/

module ngplus_tap #(parameter
    TRIG_ROWS = 256,
    TRIG_AW   = 8
)(
    input               rst,
    input               clk,        // 96 MHz, cpsplus_ddr/player domain
    input               cen,        // tie 1

    // synchronised 68K REG_SOUND write: 1-clk strobe, byte valid with it
    input               wr_stb,
    input        [ 7:0] wr_byte,

    // gate: 1 = withhold this write from the Z80 (level until next strobe)
    output              gate,
    output reg          gate_vld,   // classification settled for the current byte

    // verb event, 1-clk strobe (cpsplus_ddr interface)
    output reg          evt_stb,
    output reg   [ 2:0] evt_verb,
    output reg   [11:0] evt_track,
    output reg   [ 6:0] evt_gain,
    output reg   [15:0] evt_argw,
    output reg   [ 7:0] evt_argb,
    output reg          evt_ctrl,
    output reg          evt_sup,

    input               cfg_we,
    input        [ 7:0] cfg_addr,
    input        [15:0] cfg_data,
    input               trig_we,
    input [TRIG_AW-1:0] trig_addr,
    input        [31:0] trig_data
);

// ---------------------------------------------------------------- config ---
reg         mode_en, mode_nogate;
reg  [15:0] argset [0:15];
reg  [15:0] ctrl_cmd [0:31];
reg  [ 2:0] ctrl_vb  [0:31];
integer i;

always @(posedge clk) begin
    if( rst ) begin
        mode_en <= 1'b0; mode_nogate <= 1'b0;
        for( i = 0; i < 16; i = i+1 ) argset[i] <= 16'd0;
    end else if( cfg_we ) begin
        if( cfg_addr[7:4] == 4'h2 )            // 0x20..0x2f argset words
            argset[ cfg_addr[3:0] ] <= cfg_data;
        else if( cfg_addr == 8'h07 ) begin
            mode_en     <= cfg_data[0];
            mode_nogate <= cfg_data[2];
        end else if( cfg_addr[6] ) begin       // 0x40.. control map
            if( !cfg_addr[0] ) ctrl_cmd[ cfg_addr[5:1] ] <= cfg_data;
            else               ctrl_vb [ cfg_addr[5:1] ] <= cfg_data[2:0];
        end
    end
end

// --------------------------------------------------------------- lookups ---
reg  [31:0] trig_mem [0:TRIG_ROWS-1];
reg  [31:0] row_q;
reg  [ 7:0] cmd_cur;
reg         arg_pending;              // next byte is an argument
reg  [ 7:0] arg_owner;                // command awaiting it
reg         is_arg_cur;               // current byte was an argument

wire        argset_hit = argset[ wr_byte[7:4] ][ wr_byte[3:0] ];

always @(posedge clk) begin
    if( trig_we ) trig_mem[trig_addr] <= trig_data;
    row_q <= trig_mem[ cmd_cur[TRIG_AW-1:0] ];
end

reg        ctrl_hit;
reg  [2:0] ctrl_verb_mux;
wire [15:0] cmd16 = { 8'd0, cmd_cur };
integer ci;
always @(*) begin
    ctrl_hit      = 1'b0;
    ctrl_verb_mux = 3'd0;
    for( ci = 0; ci < 32; ci = ci+1 )
        if( ctrl_vb[ci] != 3'd0 && ctrl_cmd[ci] == cmd16 ) begin
            ctrl_hit      = 1'b1;
            ctrl_verb_mux = ctrl_vb[ci];
        end
end

// owner-of-argument classification (for the deferred control row)
reg        own_ctrl_hit;
reg  [2:0] own_ctrl_verb;
wire [15:0] own16 = { 8'd0, arg_owner };
integer oi;
always @(*) begin
    own_ctrl_hit  = 1'b0;
    own_ctrl_verb = 3'd0;
    for( oi = 0; oi < 32; oi = oi+1 )
        if( ctrl_vb[oi] != 3'd0 && ctrl_cmd[oi] == own16 ) begin
            own_ctrl_hit  = 1'b1;
            own_ctrl_verb = ctrl_vb[oi];
        end
end

// -------------------------------------------------------- accept + settle --
reg        cmd_new;
reg  [1:0] settle;
wire       lut_valid = settle == 2'd0;

always @(posedge clk) begin
    if( rst ) begin
        cmd_cur <= 8'd0; cmd_new <= 1'b0; arg_pending <= 1'b0;
        arg_owner <= 8'd0; is_arg_cur <= 1'b0; settle <= 2'd3;
    end else if( cen ) begin
        cmd_new <= 1'b0;
        if( wr_stb ) begin
            cmd_cur    <= wr_byte;
            cmd_new    <= 1'b1;
            is_arg_cur <= arg_pending;
            if( arg_pending ) begin
                arg_pending <= 1'b0;
            end else if( mode_en && argset_hit ) begin
                arg_pending <= 1'b1;
                arg_owner   <= wr_byte;
            end
        end
        if( cmd_new || cfg_we || trig_we ) settle <= 2'd3;
        else if( settle != 2'd0 )          settle <= settle - 2'd1;
    end
end

// two-stage registered classification of the current (non-argument) byte
reg        st_hit;
reg  [2:0] st_cverb;
reg  [2:0]  cls_verb;
reg  [11:0] cls_track;
reg  [ 6:0] cls_gain;
reg         cls_sup, cls_ctrl;

always @(posedge clk) begin
    if( rst ) begin
        st_hit <= 1'b0; st_cverb <= 3'd0;
        cls_verb <= 3'd0; cls_track <= 12'd0; cls_gain <= 7'd0;
        cls_sup <= 1'b0; cls_ctrl <= 1'b0;
    end else begin
        st_hit   <= ctrl_hit;
        st_cverb <= ctrl_verb_mux;
        if( is_arg_cur ) begin               // arguments are never classified
            cls_verb <= 3'd0; cls_track <= 12'd0; cls_gain <= 7'd0;
            cls_sup <= 1'b0; cls_ctrl <= 1'b0;
        end else if( st_hit ) begin          // explicit control verb
            cls_verb  <= st_cverb;
            cls_track <= 12'd0;
            cls_gain  <= 7'd0;
            cls_sup   <= 1'b0;               // control bytes are never gated
            cls_ctrl  <= 1'b1;
        end else begin                       // trigger row (0 = SFX / unmapped)
            cls_verb  <= row_q[2:0];
            cls_track <= { row_q[31:28], row_q[15:8] };
            cls_gain  <= row_q[22:16];
            cls_sup   <= row_q[24];
            cls_ctrl  <= 1'b0;
        end
    end
end

// ---------------------------------------------------------------- outputs ---
assign gate = mode_en && !mode_nogate && lut_valid && cls_sup;
always @(posedge clk) gate_vld <= lut_valid;

// Events.  A non-argument byte fires once its lookup settles.  A control
// verb whose command takes an argument (fade + speed) is deferred: it fires
// when the argument byte arrives, carrying the argument in argw/argb.
reg pend;
always @(posedge clk) begin
    if( rst ) begin
        evt_stb <= 1'b0; evt_verb <= 3'd0; evt_track <= 12'd0; evt_gain <= 7'd0;
        evt_argw <= 16'd0; evt_argb <= 8'd0; evt_ctrl <= 1'b0; evt_sup <= 1'b0;
        pend <= 1'b0;
    end else if( cen ) begin
        evt_stb <= 1'b0;
        if( cmd_new ) begin
            if( is_arg_cur ) begin
                // deferred control verb of the owner (e.g. $0a + speed)
                if( mode_en && own_ctrl_hit ) begin
                    evt_stb   <= 1'b1;
                    evt_verb  <= own_ctrl_verb;
                    evt_track <= 12'd0;
                    evt_gain  <= 7'd0;
                    evt_argw  <= { 8'd0, cmd_cur };
                    evt_argb  <= cmd_cur;
                    evt_ctrl  <= 1'b1;
                    evt_sup   <= 1'b0;
                end
            end else
                pend <= 1'b1;
        end else if( pend && lut_valid ) begin
            pend <= 1'b0;
            // a control verb whose command takes an argument waits for it
            if( mode_en && cls_verb != 3'd0 && !(cls_ctrl && arg_pending) ) begin
                evt_stb   <= 1'b1;
                evt_verb  <= cls_verb;
                evt_track <= cls_track;
                evt_gain  <= cls_gain;
                evt_argw  <= 16'd0;
                evt_argb  <= 8'd0;
                evt_ctrl  <= cls_ctrl;
                evt_sup   <= cls_sup && !mode_nogate;
            end
        end
    end
end

endmodule
