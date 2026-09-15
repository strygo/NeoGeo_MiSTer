/*  NG+ — ngplus_top: Neo Geo tap + CPS+ pack loader/DDR backend + player
    =====================================================================

    Neo Geo sibling of cpsplus_cps1_top.v.  Same shared playback engine
    (cpsplus_ddr + cpsplus_player + cpsplus_adx, reused unchanged apart
    from the format-v2 additions: dialect byte, argument-command bitmap,
    fade law 3); only the front-end sniffer differs (ngplus_tap.v).

        REG_SOUND write ─▶ ngplus_tap ──evt_*──▶ cpsplus_ddr ◀─▶ DDRAM (via ngplus_ddrmux)
        (wr_stb/wr_byte)      │  ▲ cfg/table load          │ trk/fade/start
                            gate                           ▼   ▲ mem port
                              │                     cpsplus_player ─▶ audio (+ volume)
                              ▼
                   game top: withhold the Z80 NMI for the gated write

    Everything runs on `clk` = CLK_96M (= DDRAM_CLK in the Neo Geo core).
    The sample clock enable is generated here (jtframe_frac_cen at the
    current track's rate from 96 MHz); `cen_frame` is the ~60 Hz fade tick
    (vblank start, one clk pulse, from the game top).

    With no pack loaded (`boot_go` never pulsed or the magic check failed),
    or `osd_en` low, the tap stays disabled and `gate` is constant 0: the
    host core is stock.
*/

module ngplus_top #(parameter
    TRIG_AW     = 8,          // Neo Geo command is a single byte -> 256 rows
    TRIG_ROWS   = 256,
    TRK_AW      = 7,
    XF_LUT_FILE = "cpsplus_xf_lut.hex"
)(
    input             rst,
    input             clk,          // 96 MHz

    // synchronised 68K REG_SOUND write (one clk strobe, byte valid with it)
    input             wr_stb,
    input      [ 7:0] wr_byte,
    output            gate,         // withhold the NMI for this write
    output            gate_vld,

    // audio
    input             cen_frame,    // ~60 Hz fade tick
    input             osd_pause,
    input      [ 1:0] vol,          // 0 100%, 1 75%, 2 50%, 3 150%
    output signed [15:0] audio_l,
    output signed [15:0] audio_r,
    output            sample_vld,
    output            playing,

    // control / status
    input      [31:0] base_addr,    // pack DDR byte address (0x38040000)
    input             osd_en,
    input             boot_go,      // pulse after reset release with a pack staged
    output            ready,
    output            magic_ok,
    output     [ 3:0] status,
    output reg [ 7:0] last_cmd,
    output reg        last_mapped,
    // NGPLUS_DBG overlay read-outs (cpsplus_dbg_overlay rows 2/3)
    output reg [ 2:0] last_verb,
    output reg        last_ctrl,
    output     [ 2:0] dbg_fst,
    output     [ 3:0] dbg_end,
    output            dbg_fempty,

    // DDRAM master (read only; arbitrated by ngplus_ddrmux upstream)
    input             ddram_busy,
    output     [ 7:0] ddram_burstcnt,
    output     [28:0] ddram_addr,
    input      [63:0] ddram_dout,
    input             ddram_dout_ready,
    output            ddram_rd
);

// tap <-> ddr
wire        evt_stb, evt_ctrl, evt_sup;
wire [ 2:0] evt_verb;
wire [11:0] evt_track;
wire [ 6:0] evt_gain;
wire [15:0] evt_argw;
wire [ 7:0] evt_argb;
wire        cfg_we, trig_we;
wire [ 7:0] cfg_addr;
wire [15:0] cfg_data;
wire [TRIG_AW-1:0] trig_addr;
wire [31:0] trig_data;

// ddr <-> player
wire        pl_start, pl_stop;
wire [31:0] trk_addr, trk_len, trk_lstart, trk_lend;
wire [31:0] trk_lstart_smp, trk_lend_smp;
wire        trk_xfade_en;
wire [ 1:0] trk_loop_cnt;
wire        trk_stereo, trk_codec;
wire [ 6:0] trk_gain, trig_gain;
wire [15:0] trk_c1, trk_c2, trk_rate;
wire [ 1:0] fade_law;
wire [31:0] fade_const1, fade_const2;
wire        fade_trig, fade_loop_off, fade_stop_at0, restore_trig;
wire [ 6:0] fade_target;
wire [15:0] fade_arg;
wire        pmem_rd, pmem_ack;
wire [31:3] pmem_addr;
wire [63:0] pmem_data;

// local reset (see cpsplus_cps1_top: keeps the external reset net short)
reg rst_p = 1'b1, rst_i = 1'b1;
always @(posedge clk) begin
    rst_p <= rst;
    rst_i <= rst_p;
end

always @(posedge clk) begin
    if( rst_i ) begin
        last_cmd    <= 8'd0;
        last_mapped <= 1'b0;
        last_verb   <= 3'd0;
        last_ctrl   <= 1'b0;
    end else if( wr_stb ) begin
        last_cmd    <= wr_byte;
        last_mapped <= 1'b0;
    end else if( evt_stb ) begin
        last_mapped <= evt_verb != 3'd0;
        last_verb   <= evt_verb;
        last_ctrl   <= evt_ctrl;
    end
end

ngplus_tap #(
    .TRIG_ROWS ( TRIG_ROWS ),
    .TRIG_AW   ( TRIG_AW   )
) u_tap (
    .rst        ( rst_i      ),
    .clk        ( clk        ),
    .cen        ( 1'b1       ),
    .wr_stb     ( wr_stb     ),
    .wr_byte    ( wr_byte    ),
    .gate       ( gate       ),
    .gate_vld   ( gate_vld   ),
    .evt_stb    ( evt_stb    ),
    .evt_verb   ( evt_verb   ),
    .evt_track  ( evt_track  ),
    .evt_gain   ( evt_gain   ),
    .evt_argw   ( evt_argw   ),
    .evt_argb   ( evt_argb   ),
    .evt_ctrl   ( evt_ctrl   ),
    .evt_sup    ( evt_sup    ),
    .cfg_we     ( cfg_we     ),
    .cfg_addr   ( cfg_addr   ),
    .cfg_data   ( cfg_data   ),
    .trig_we    ( trig_we    ),
    .trig_addr  ( trig_addr  ),
    .trig_data  ( trig_data  )
);

cpsplus_ddr #(
    .TRIG_AW   ( TRIG_AW   ),
    .TRIG_ROWS ( TRIG_ROWS ),
    .TRK_AW    ( TRK_AW    ),
    .SAME_SONG ( 0         )   // a re-sent music command restarts the song (CD driver semantics)
) u_ddr (
    .rst            ( rst_i          ),
    .clk            ( clk            ),
    .base_addr      ( base_addr      ),
    .base_indirect  ( 1'b0           ),
    .osd_en         ( osd_en         ),
    .boot_go        ( boot_go        ),
    .ready          ( ready          ),
    .magic_ok       ( magic_ok       ),
    .status         ( status         ),
    .evt_stb        ( evt_stb        ),
    .evt_verb       ( evt_verb       ),
    .evt_track      ( evt_track      ),
    .evt_gain       ( evt_gain       ),
    .evt_argw       ( evt_argw       ),
    .evt_argb       ( evt_argb       ),
    .cfg_we         ( cfg_we         ),
    .cfg_addr       ( cfg_addr       ),
    .cfg_data       ( cfg_data       ),
    .trig_we        ( trig_we        ),
    .trig_addr      ( trig_addr      ),
    .trig_data      ( trig_data      ),
    .pl_start       ( pl_start       ),
    .pl_stop        ( pl_stop        ),
    .trk_addr       ( trk_addr       ),
    .trk_len        ( trk_len        ),
    .trk_lstart     ( trk_lstart     ),
    .trk_lend       ( trk_lend       ),
    .trk_lstart_smp ( trk_lstart_smp ),
    .trk_lend_smp   ( trk_lend_smp   ),
    .trk_xfade_en   ( trk_xfade_en   ),
    .trk_loop_cnt   ( trk_loop_cnt   ),
    .trk_stereo     ( trk_stereo     ),
    .trk_codec      ( trk_codec      ),
    .trk_gain       ( trk_gain       ),
    .trk_c1         ( trk_c1         ),
    .trk_c2         ( trk_c2         ),
    .trk_rate       ( trk_rate       ),
    .trig_gain      ( trig_gain      ),
    .fade_law       ( fade_law       ),
    .fade_const1    ( fade_const1    ),
    .fade_const2    ( fade_const2    ),
    .fade_trig      ( fade_trig      ),
    .fade_loop_off  ( fade_loop_off  ),
    .fade_stop_at0  ( fade_stop_at0  ),
    .restore_trig   ( restore_trig   ),
    .fade_target    ( fade_target    ),
    .fade_arg       ( fade_arg       ),
    .pmem_rd        ( pmem_rd        ),
    .pmem_addr      ( pmem_addr      ),
    .pmem_data      ( pmem_data      ),
    .pmem_ack       ( pmem_ack       ),
    .ddram_busy     ( ddram_busy     ),
    .ddram_burstcnt ( ddram_burstcnt ),
    .ddram_addr     ( ddram_addr     ),
    .ddram_dout     ( ddram_dout     ),
    .ddram_dout_ready( ddram_dout_ready ),
    .ddram_rd       ( ddram_rd       )
);

// sample clock enable at the current track's rate (n/m of 96 MHz)
wire [1:0] cen_v;                    // jtframe_frac_cen needs W>=2
wire       cen_sample = cen_v[0];
jtframe_frac_cen #( .W(2), .WC(27) ) u_cen (
    .clk  ( clk                    ),
    .n    ( {11'd0, trk_rate}      ),
    .m    ( 27'd96_000_000         ),
    .cen  ( cen_v                  ),
    .cenb (                        )
);

wire signed [15:0] pl_audio_l, pl_audio_r;
cpsplus_player #( .XF_LUT_FILE( XF_LUT_FILE ) ) u_player (
    .rst            ( rst_i         ),
    .clk            ( clk           ),
    .cen_sample     ( cen_sample    ),
    .cen_frame      ( cen_frame     ),
    .start          ( pl_start      ),
    .stop           ( pl_stop       ),
    .osd_pause      ( osd_pause     ),
    .trk_addr       ( trk_addr      ),
    .trk_len        ( trk_len       ),
    .trk_loop_start ( trk_lstart    ),
    .trk_loop_end   ( trk_lend      ),
    .trk_stereo     ( trk_stereo    ),
    .trk_codec      ( trk_codec     ),
    .trk_gain       ( trk_gain      ),
    .trk_c1         ( trk_c1        ),
    .trk_c2         ( trk_c2        ),
    .trig_gain      ( trig_gain     ),
    .trk_xfade_en       ( trk_xfade_en   ),
    .trk_loop_cnt       ( trk_loop_cnt   ),
    .trk_loop_start_smp ( trk_lstart_smp ),
    .trk_loop_end_smp   ( trk_lend_smp   ),
    .fade_law       ( fade_law      ),
    .fade_const1    ( fade_const1   ),
    .fade_const2    ( fade_const2   ),
    .fade_trig      ( fade_trig     ),
    .fade_target    ( fade_target   ),
    .fade_arg       ( fade_arg      ),
    .fade_loop_off  ( fade_loop_off ),
    .fade_stop_at0  ( fade_stop_at0 ),
    .restore_trig   ( restore_trig  ),
    .mem_rd         ( pmem_rd       ),
    .mem_addr       ( pmem_addr     ),
    .mem_data       ( pmem_data     ),
    .mem_ack        ( pmem_ack      ),
    .audio_l        ( pl_audio_l    ),
    .audio_r        ( pl_audio_r    ),
    .sample_vld     ( sample_vld    ),
    .playing        ( playing       ),
    .track_done     (               ),
    .dbg_fst        ( dbg_fst       ),
    .dbg_end        ( dbg_end       ),
    .dbg_fempty     ( dbg_fempty    )
);

// OSD volume: 100 / 75 / 50 / 150 %, saturated
function signed [15:0] scale_sat(input signed [15:0] x, input [1:0] v);
    reg signed [17:0] w, y;
    begin
        w = {{2{x[15]}}, x};
        case (v)
            2'd1:    y = w - (w >>> 2);
            2'd2:    y = w >>> 1;
            2'd3:    y = w + (w >>> 1);
            default: y = w;
        endcase
        scale_sat = (y > 18'sd32767)  ? 16'sd32767 :
                    (y < -18'sd32768) ? -16'sd32768 : y[15:0];
    end
endfunction

reg signed [15:0] out_l, out_r;
always @(posedge clk) begin
    out_l <= scale_sat(pl_audio_l, vol);
    out_r <= scale_sat(pl_audio_r, vol);
end
assign audio_l = out_l;
assign audio_r = out_r;

endmodule
