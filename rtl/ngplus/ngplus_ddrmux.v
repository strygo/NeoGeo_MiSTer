/*  NG+ — ngplus_ddrmux: two-master DDRAM arbiter for the Neo Geo core
    ==================================================================

    The stock core has one DDR3 master (rtl/mem/ddram.sv: V ROM / M1 live
    reads, the legacy ioctl write path, and the memcp burst engine).  NG+
    adds the CPS+ pack loader/player as a second, read-only, LOWEST
    priority master.  Modelled on jtframe_mr_ddrmux's CPS+ branch: the
    grant switches only on burst boundaries (outstanding read beats are
    tracked), the core master is granted whenever it has a request pending,
    and the pack master runs only while the core is quiet.

    ADPCM deadline (neogeo.sv ADPCMA_ACK_COUNTER = 128 DDRAM_CLK): a pack
    burst is at most 8 beats, so the core's next request waits < ~40 clk
    including DDR latency, inside the deadline.  The player prefetches
    2x64 B, so the pack side tolerates the core's 128-beat memcp bursts.

    Verilog-2005.
*/

module ngplus_ddrmux(
    input          rst,
    input          clk,             // DDRAM_CLK

    // core master (ddram.sv)
    input   [ 7:0] core_burstcnt,
    input   [28:0] core_addr,
    input          core_rd,
    input          core_we,
    input   [ 7:0] core_be,
    input   [63:0] core_din,
    output         core_busy,
    output         core_dout_ready,

    // pack master (cpsplus_ddr via ngplus_top), read only
    input   [ 7:0] pk_burstcnt,
    input   [28:0] pk_addr,
    input          pk_rd,
    output         pk_busy,
    output         pk_dout_ready,

    // DDR pins
    input          ddr_busy,
    input          ddr_dout_ready,
    output  [ 7:0] ddr_burstcnt,
    output  [28:0] ddr_addr,
    output         ddr_rd,
    output         ddr_we,
    output  [ 7:0] ddr_be,
    output  [63:0] ddr_din
);

reg        pk_en;
reg [ 8:0] beats;                 // outstanding beats of the accepted read
wire       core_req = core_rd | core_we;

always @(posedge clk, posedge rst) begin
    if( rst ) begin
        pk_en <= 1'b0;
        beats <= 9'd0;
    end else begin
        if( ddr_rd && !ddr_busy )
            beats <= {1'b0, ddr_burstcnt};
        else if( ddr_dout_ready && beats != 9'd0 )
            beats <= beats - 9'd1;
        // switch owner only when nothing is in flight
        if( beats == 9'd0 && !ddr_rd && !ddr_we && !ddr_busy )
            pk_en <= !core_req && pk_rd;
    end
end

assign ddr_burstcnt   = pk_en ? pk_burstcnt : core_burstcnt;
assign ddr_addr       = pk_en ? pk_addr     : core_addr;
assign ddr_rd         = pk_en ? pk_rd       : core_rd;
assign ddr_we         = pk_en ? 1'b0        : core_we;
assign ddr_be         = pk_en ? 8'hff       : core_be;
assign ddr_din        = pk_en ? 64'd0       : core_din;

assign core_busy       = pk_en | ddr_busy;
assign pk_busy         = ~pk_en | ddr_busy;
assign core_dout_ready = ddr_dout_ready & ~pk_en;
assign pk_dout_ready   = ddr_dout_ready &  pk_en;

endmodule
