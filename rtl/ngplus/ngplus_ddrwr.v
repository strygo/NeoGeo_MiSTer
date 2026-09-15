/*  NG+ — ngplus_ddrwr: 64-bit DDR writer for the image's DDR-bound regions
    ======================================================================

    The stock core writes ioctl words into DDR one 16-bit word at a time
    through ddram.sv's write port, with a clk_sys <-> DDRAM_CLK toggle
    handshake per word.  That path was only ever meant for the old
    firmware's V-ROM loads; pushing a 116 MiB pack through it makes the
    image load several times slower than the SPI channel.

    This block takes the same (address, 16-bit word) stream on clk_sys,
    packs four consecutive words into one 64-bit DDR word, hands it to the
    DDRAM_CLK side with one handshake per 64-bit word, and issues a
    single-beat write with byte enables through ngplus_ddrmux.  A word that
    is not the successor of the previous one (a region change) flushes the
    partial 64-bit word first; `flush` (download end) flushes the tail.

    `busy` back-pressures hps_io (ioctl_wait) while a hand-off is pending; a
    word already in flight is parked and processed after.  At SPI rates the
    stall never happens (one hand-off per four words).

    Addresses are byte addresses relative to the DDR base (ddram.sv's
    28-bit space: 0x30000000 + addr).  Verilog-2005.
*/

module ngplus_ddrwr(
    // clk_sys side: the stream
    input             clk,
    input             rst,
    input             wr,            // one 16-bit word
    input      [27:0] addr,          // even byte address
    input      [15:0] din,
    input             flush,         // end of stream: write the partial word
    output            busy,          // ioctl_wait contribution

    // DDRAM_CLK side: write master (ngplus_ddrmux port)
    input             dclk,
    input             drst,
    output reg        d_we,
    output reg [28:0] d_addr,        // 64-bit word address (byte >> 3)
    output reg [63:0] d_din,
    output reg [ 7:0] d_be,
    input             d_busy
);

// ---------------------------------------------------------- clk_sys side --
reg  [63:0] acc;
reg  [ 7:0] acc_be;
reg  [27:3] acc_addr;
reg         acc_vld;                 // acc holds at least one word
reg  [27:1] next_addr;

reg         req_tgl;                 // toggles per hand-off
reg  [63:0] req_data;
reg  [ 7:0] req_be;
reg  [27:3] req_addr;
reg  [ 2:0] ack_s;                   // ack toggle synchronised into clk_sys
reg         ack_out;                 // DDRAM_CLK side: toggles per accepted write
wire        ack_in = ack_out;
wire        pending = req_tgl != ack_s[2];

// a word that arrives while a hand-off is pending is parked (one deep:
// hps_io honours ioctl_wait per word, so at most one is ever in flight)
reg         hold_vld, flush_pend;
reg  [27:0] hold_addr;
reg  [15:0] hold_data;

wire        proc   = !pending & (hold_vld | wr);
wire [27:0] paddr  = hold_vld ? hold_addr : addr;
wire [15:0] pdata  = hold_vld ? hold_data : din;
// same 64-bit word AND the next lane: anything else pushes the partial word
// out first (a region change, or a lane-3 word kept from a region change)
wire        contiguous = (paddr[27:1] == next_addr) && acc_vld && (paddr[27:3] == acc_addr);
wire [ 1:0] lane   = paddr[2:1];

assign busy = pending | hold_vld;

reg  [63:0] acc_n;
reg  [ 7:0] be_n;

always @(posedge clk) begin
    if (rst) begin
        acc_vld <= 1'b0; acc_be <= 8'd0; acc <= 64'd0; acc_addr <= 25'd0;
        next_addr <= 27'd0; req_tgl <= 1'b0; req_data <= 64'd0; req_be <= 8'd0;
        req_addr <= 25'd0; ack_s <= 3'd0; hold_vld <= 1'b0; hold_addr <= 28'd0;
        hold_data <= 16'd0; flush_pend <= 1'b0;
    end else begin
        ack_s <= {ack_s[1:0], ack_in};
        if (flush) flush_pend <= 1'b1;
        if (wr && (pending || hold_vld)) begin
            hold_vld  <= 1'b1;            // (a second word here would be lost; cannot happen)
            hold_addr <= addr;
            hold_data <= din;
        end

        if (proc) begin
            if (hold_vld) hold_vld <= 1'b0;
            if (acc_vld && !contiguous) begin
                // region change: push what we have, start over with this word
                req_tgl   <= ~req_tgl;
                req_data  <= acc;
                req_be    <= acc_be;
                req_addr  <= acc_addr;
                acc       <= {48'd0, pdata} << {lane, 4'd0};
                acc_be    <= 8'h03 << {lane, 1'b0};
                acc_addr  <= paddr[27:3];
                acc_vld   <= 1'b1;
                next_addr <= paddr[27:1] + 27'd1;
            end else begin
                acc_n = acc_vld ? acc : 64'd0;
                be_n  = acc_vld ? acc_be : 8'd0;
                acc_n[{lane, 4'd0} +: 16] = pdata;
                be_n[{lane, 1'b0} +: 2]   = 2'b11;
                next_addr <= paddr[27:1] + 27'd1;
                if (lane == 2'b11) begin
                    // word complete (last lane): hand off, nothing kept
                    req_tgl  <= ~req_tgl;
                    req_data <= acc_n;
                    req_be   <= be_n;
                    req_addr <= paddr[27:3];
                    acc_vld  <= 1'b0;
                    acc_be   <= 8'd0;
                end else begin
                    acc      <= acc_n;
                    acc_be   <= be_n;
                    acc_addr <= paddr[27:3];
                    acc_vld  <= 1'b1;
                end
            end
        end else if (flush_pend && !pending && !hold_vld) begin
            // end of stream: write the partial word, if any
            flush_pend <= 1'b0;
            if (acc_vld) begin
                req_tgl  <= ~req_tgl;
                req_data <= acc;
                req_be   <= acc_be;
                req_addr <= acc_addr;
                acc_vld  <= 1'b0;
                acc_be   <= 8'd0;
            end
        end
    end
end

// --------------------------------------------------------- DDRAM_CLK side --
reg  [2:0] req_s;
wire       req_edge = req_s[1] != req_s[2];

always @(posedge dclk) begin
    if (drst) begin
        req_s <= 3'd0; ack_out <= 1'b0; d_we <= 1'b0; d_addr <= 29'd0;
        d_din <= 64'd0; d_be <= 8'd0;
    end else begin
        req_s <= {req_s[1:0], req_tgl};
        if (d_we) begin
            if (!d_busy) begin              // accepted
                d_we    <= 1'b0;
                ack_out <= ~ack_out;
            end
        end else if (req_edge) begin
            d_we   <= 1'b1;
            d_addr <= {4'b0011, req_addr};  // 0x30000000-based, 64-bit words
            d_din  <= req_data;
            d_be   <= req_be;
        end
    end
end

endmodule
