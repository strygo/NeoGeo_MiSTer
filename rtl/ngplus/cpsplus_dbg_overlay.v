// CPS+ visual pack-load indicator -- CPSPLUS_DBG builds only.
//
// Replaces counting beeps.  The audible diagnostic encodes the load state as a
// beep count (1..6), which is ambiguous in practice: an early bring-up round of
// this project burned several hardware cycles disagreeing about whether a group
// was 3 beeps or 4.  A colour is unambiguous at a glance and needs no counting.
//
// Draws a small solid square in the top-left of the active picture:
//
//   GREEN    a track is PLAYING          -- arranged audio is reaching the mixer
//   BLUE     pack loaded, no track       -- load path fine; fault is trigger /
//                                          suppression / player
//   YELLOW   loading, stuck mid-load     -- DDR bus never granted
//   WHITE    loader idle                 -- should be transient (self-heal retry)
//   RED      pack pointer never arrived  -- MRA <patch offset=8> did not reach
//                                          the DDR image (wrong --rom-len!)
//   MAGENTA  bad magic at pack_base      -- pointer read but base wrong
//   CYAN     pack header out of range
//
// RED and MAGENTA are the two that indict the MRA rather than the core, which is
// exactly the ambiguity that cost a full test cycle on Final Fight.
//
// Under the square, 8 cells print the last non-idle sound-latch byte (MSB left,
// lit = 1) and a 9th cell says whether the pack has a trigger row for it --
// green yes, red no.  That is what makes a silent cue diagnosable: read the
// byte, find it in the pack's cue sheet, and a red flag means the map lacks it
// while a green flag with a non-green square means the map has it but nothing
// started.
//
// Coordinates come from the blanking signals, so no core-specific counters are
// needed: hcnt advances on pxl_cen inside LHBL, vcnt on each LHBL falling edge,
// both cleared by LVBL.  The square sits at (X0,Y0)+SIZE in *active* pixels.
module cpsplus_dbg_overlay #(
    parameter X0   = 8,
    parameter Y0   = 8,
    parameter SIZE    = 16,
    parameter CELL_SH = 3,        // bit-cell width = 1<<CELL_SH; MUST be a shift
    parameter CW      = 8         // bits per colour component in the host core
)(
    input                   clk,
    input                   pxl_cen,
    input                   LHBL,        // 1 = active line
    input                   LVBL,        // 1 = active frame
    input       [3:0]       status,      // cpsplus_ddr status
    input                   playing,
    input       [7:0]       last_cmd,    // last non-idle sound-latch byte
    input                   last_mapped, // that byte had a trigger row
    // second/third cell rows (silence diagnosis): see `row2`/`row3` below
    input       [2:0]       last_verb,   // verb of the last decoded command
    input                   last_ctrl,   // it was a control-region command
    input       [2:0]       fst,         // player feed FSM
    input       [3:0]       end_cause,   // {eof, loop_off, auto_stop, stop}
    input                   fifo_empty,  // player sample FIFO empty
    input       [CW-1:0]    red_in,
    input       [CW-1:0]    green_in,
    input       [CW-1:0]    blue_in,
    output reg  [CW-1:0]    red_out,
    output reg  [CW-1:0]    green_out,
    output reg  [CW-1:0]    blue_out
);

localparam [CW-1:0] HI = {CW{1'b1}}, LO = {CW{1'b0}};

// ------------------------------------------------------ clock crossing ----
// status/playing/last_cmd/last_mapped are generated in the CPS+ clock domain
// (96 MHz) while this module runs on the video clock, so they are resynced
// before use.  Two flops remove the metastability risk; the byte can still
// tear for the one frame in which it changes, which is invisible at a glance
// and much cheaper than a handshake for a read-only debug display.
reg [3:0] status_s0,  status_s;
reg [7:0] cmd_s0,     cmd_s;
reg       play_s0,    play_s;
reg       mapped_s0,  mapped_s;
// row 2: why is it silent?  verb of the last command (2=stop 3=fade_out
// 6=master_fade are the trigger-side causes), control flag, and the feed
// FSM: a stuck FETCH (waiting on DDR) vs WRAP/SNAP (waiting on the decoder)
// vs IDLE (track ended) vs FEED with an empty FIFO (starved).
// row 3: sticky end cause since the track started: EOF-pattern frame seen
// (bad data from DDR), verb-3 loop-off, fade-to-zero auto-stop, stop verb.
wire [7:0] row2 = { last_verb, last_ctrl, fst != 3'd0, fst == 3'd1,
                    (fst == 3'd3) | (fst == 3'd4), fifo_empty };
wire [7:0] row3 = { end_cause, 4'b0000 };
reg [7:0] row2_s0, row2_s, row3_s0, row3_s;

always @(posedge clk) begin
    { status_s, status_s0 } <= { status_s0, status      };
    { cmd_s,    cmd_s0    } <= { cmd_s0,    last_cmd    };
    { play_s,   play_s0   } <= { play_s0,   playing     };
    { mapped_s, mapped_s0 } <= { mapped_s0, last_mapped };
    { row2_s,   row2_s0   } <= { row2_s0,   row2        };
    { row3_s,   row3_s0   } <= { row3_s0,   row3        };
end

reg [9:0] hcnt, vcnt;
reg       lhbl_l, lvbl_l;

always @(posedge clk) begin
    lhbl_l <= LHBL;
    lvbl_l <= LVBL;
    if( !LVBL ) begin                       // vertical blank: reset both
        hcnt <= 10'd0;
        vcnt <= 10'd0;
    end else begin
        if( !LHBL ) hcnt <= 10'd0;          // horizontal blank: reset column
        else if( pxl_cen ) hcnt <= hcnt + 10'd1;
        if( lhbl_l && !LHBL ) vcnt <= vcnt + 10'd1;   // one row per line end
    end
end

wire in_box = LHBL && LVBL &&
              hcnt >= X0 && hcnt < (X0+SIZE) &&
              vcnt >= Y0 && vcnt < (Y0+SIZE);

// ------------------------------------------------- last-command read-out --
// The status square alone cannot answer "why is THIS song not playing": BLUE
// (loaded, nothing playing) is also the normal state during silence, so it is
// ambiguous exactly when it matters.  So print the last non-idle latch byte as
// 8 cells, MSB first, directly under the square -- lit = 1 -- plus a 9th cell:
//   GREEN  that command HAS a trigger row  (so it should have played)
//   RED    it does NOT                     (unmapped -> fails open by design)
// Read the byte, look it up in the pack's cue sheet, and the two cases separate
// without guesswork: a red 9th cell means the map is missing the cue; a green
// 9th cell with no green square means the map has it but the player did not
// start it.
// The cell width MUST stay a power of two.  This was originally CELL = SIZE/2
// = 6 with cell_ix = (hcnt-X0)/CELL, and Quartus turns a divide-by-6 into a
// combinational divider: it missed the 96 MHz video clock by 4.816 ns and every
// one of the 30 violating paths in that build ran through it (Add3 -> red_out).
// As a shift it is free.  A debug read-out must not cost timing -- if it does,
// it cannot be built alongside the thing it is meant to diagnose.
localparam CELL = 1 << CELL_SH;
wire [9:0] BITY = Y0 + SIZE + 2;
wire in_bits = LHBL && LVBL &&
               vcnt >= BITY && vcnt < (BITY + CELL) &&
               hcnt >= X0   && hcnt < (X0 + 9*CELL);
wire [3:0] cell_ix = (hcnt - X0) >> CELL_SH;      // 0..8
wire       bit_on  = (cell_ix < 4'd8) ? cmd_s[7 - cell_ix[2:0]] : 1'b0;
wire       is_flag = cell_ix == 4'd8;
// rows 2 and 3 sit under the command row, same cell geometry, 8 cells each
wire [9:0] BITY2 = BITY + CELL + 2;
wire [9:0] BITY3 = BITY2 + CELL + 2;
wire in_bits2 = LHBL && LVBL && vcnt >= BITY2 && vcnt < (BITY2 + CELL) &&
                hcnt >= X0 && hcnt < (X0 + 8*CELL);
wire in_bits3 = LHBL && LVBL && vcnt >= BITY3 && vcnt < (BITY3 + CELL) &&
                hcnt >= X0 && hcnt < (X0 + 8*CELL);
wire bit2_on  = row2_s[7 - cell_ix[2:0]];
wire bit3_on  = row3_s[7 - cell_ix[2:0]];

// state -> colour.  `playing` wins: it is the only state that means success.
reg [2:0] rgb;                              // {r,g,b} on/off
always @(*) begin
    if( play_s )             rgb = 3'b010;  // GREEN   playing
    else case( status_s )
        4'd2:                rgb = 3'b001;  // BLUE    loaded, no track
        4'd1:                rgb = 3'b110;  // YELLOW  loading
        4'd0:                rgb = 3'b111;  // WHITE   idle
        4'd4:                rgb = 3'b100;  // RED     pointer never arrived
        4'd5:                rgb = 3'b101;  // MAGENTA bad magic
        4'd6:                rgb = 3'b011;  // CYAN    header out of range
        default:             rgb = 3'b111;
    endcase
end

// bit cells: white when set, dark when clear; the flag cell is green/red
wire [2:0] bits_rgb = is_flag ? (mapped_s ? 3'b010 : 3'b100)
                              : (bit_on ? 3'b111 : 3'b000);

always @(posedge clk) begin
    if( in_box ) begin
        red_out   <= rgb[2] ? HI : LO;
        green_out <= rgb[1] ? HI : LO;
        blue_out  <= rgb[0] ? HI : LO;
    end else if( in_bits ) begin
        red_out   <= bits_rgb[2] ? HI : LO;
        green_out <= bits_rgb[1] ? HI : LO;
        blue_out  <= bits_rgb[0] ? HI : LO;
    end else if( in_bits2 ) begin               // cyan when set
        red_out   <= LO;
        green_out <= bit2_on ? HI : LO;
        blue_out  <= bit2_on ? HI : LO;
    end else if( in_bits3 ) begin               // yellow when set
        red_out   <= bit3_on ? HI : LO;
        green_out <= bit3_on ? HI : LO;
        blue_out  <= LO;
    end else begin
        red_out   <= red_in;
        green_out <= green_in;
        blue_out  <= blue_in;
    end
end

endmodule
