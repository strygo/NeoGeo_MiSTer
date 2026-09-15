/*  NG+ — ngplus_router: .nga image router on the OSD file-load stream
    ==================================================================

    The NG+ core is a generic core to the MiSTer firmware: the launcher's
    "Load Game" entry streams one kit-built image (kit/nga_format.py) over
    the ioctl file channel (16-bit words, WIDE hps_io, one `wr` per word).
    This block sits between hps_io and the stock per-index load paths and
    presents the image body as the sequence of per-file downloads the stock
    firmware would have produced for the same game:

      * bytes 0..4095: the header.  Captures cfg[31:0], the region count and
        the 16-entry region table {dest, flags, file offset, length, base}.
      * then the config record as its own download on index 10 (word 0 =
        0x8000 so cp_op stays 0, word 2 = cfg[15:0], word 4 = cfg[31:16]),
        what the stock loader's notify_conf() sends.
      * region k as its own download on index dest_k: words at file
        positions [off_k, off_k + len_k) go out with address base_k +
        (pos - off_k); padding between regions is dropped.  A system-ROM
        region (index 0) whose BIOS_UNI flag differs from `bios_uni` is
        consumed silently, so the OSD "BIOS" choice selects the copy.
      * the PACK region (flag 0x80) goes out on index 0xff with the pack
        offset (28 bits, `v_paddr`); neogeo.sv writes it into the DDR pack
        window.

    Per-region downloads matter: the core derives the C ROM base and the
    ROM masks from the address it sees when a download ENDS (hps_io leaves
    ioctl_addr one past the last word), so at every region end `v_addr` is
    set to base + length and `v_download` drops for a few clocks before the
    next index is raised.  `v_wait` holds the HPS off during the switch;
    a word that still lands in the gap is parked and replayed.

    `img_start` pulses at the first image word (game top: reset masks and
    the C base as the stock core does on status[0]); `img_done` pulses at
    download end with `pack_present`.

    Index 2 ("Load Arranged Pack", a bare .cpk): index 0xff from offset 0.
    Every other index is passed through unchanged.  The file position is
    counted here in 32 bits (ioctl_addr is 27 bits; images exceed 128 MiB).
    Verilog-2005.
*/

module ngplus_router #(parameter
    IMG_INDEX  = 8'd1,
    PACK_INDEX = 8'd2,
    PACK_VIDX  = 8'hff,
    GAP        = 4                       // clocks with v_download low at a switch
)(
    input             clk,
    input             rst,
    input             bios_uni,          // OSD: 1 = UniBIOS copy, 0 = original

    // from hps_io
    input             hps_download,
    input      [ 7:0] hps_index,
    input             hps_wr,
    input      [26:0] hps_addr,
    input      [15:0] hps_dout,
    output            v_wait,            // ioctl_wait contribution (segment switch)

    // virtual ioctl to the core
    output reg        v_download,
    output reg [ 7:0] v_index,
    output reg        v_wr,
    output reg [26:0] v_addr,
    output reg [15:0] v_dout,
    output reg [27:0] v_paddr,           // pack byte offset for index 0xff words

    output reg        img_start,         // 1-clk pulse: an image load begins
    output reg        img_done,          // 1-clk pulse after an image / pack load
    output reg        pack_present,      // valid with img_done
    output reg [31:0] dbg_cfg,
    output reg [ 4:0] dbg_region
);

localparam [31:0] HDR_BYTES = 32'd4096;
localparam        MAXR      = 16;

// segment machine
localparam [2:0] S_IDLE = 3'd0,   // no image in flight
                 S_HDR  = 3'd1,   // header words
                 S_SW   = 3'd2,   // switching: download low
                 S_CFG  = 3'd3,   // config record download (3 words)
                 S_REG  = 3'd4,   // region download
                 S_DONE = 3'd5,   // all regions sent; drain to download end
                 S_ENDM = 3'd6;   // one clock after a segment's last write: end marker

reg  [ 2:0] st;
reg  [ 3:0] gap;
reg  [31:0] pos;                         // file byte position of the next word
reg         dl_d;
reg         is_img, is_pack;

reg  [31:0] cfg;
reg  [ 4:0] nreg;
reg  [ 7:0] r_dest  [0:MAXR-1];
reg  [ 7:0] r_flags [0:MAXR-1];
reg  [31:0] r_off   [0:MAXR-1];
reg  [31:0] r_len   [0:MAXR-1];
reg  [31:0] r_base  [0:MAXR-1];
reg  [31:0] pack_len;

reg  [ 4:0] cur;
reg  [ 4:0] sw_next;                     // 0 = next segment is the cfg record, else region (sw_next-1)
reg  [ 1:0] cfg_k;
reg  [26:0] endm_addr;                   // end marker (one past the last word) for S_ENDM
reg         endm_last;                   // no segment follows the marker

// one-word park for a word that lands during a switch
reg         hold_vld;
reg  [15:0] hold_data;

wire        img_start_w  = hps_download & ~dl_d & (hps_index == IMG_INDEX);
wire        pack_start_w = hps_download & ~dl_d & (hps_index == PACK_INDEX);
wire        in_hdr       = pos < HDR_BYTES;
wire [11:0] hw   = pos[11:0];
wire        hdr_entry = (hw >= 12'h080) && (hw < 12'h080 + MAXR*16);
wire [ 3:0] ent  = (hw - 12'h080) >> 4;
wire [ 3:0] efld = hw[3:0];

wire        cur_valid  = cur < nreg;
wire [31:0] cur_end    = r_off[cur] + r_len[cur];
wire        cur_skip   = (r_dest[cur] == 8'd0) && (r_flags[cur][0] != bios_uni);

// a word to process: live, or the parked one once the switch is over
wire        switching  = (st == S_SW) || (st == S_ENDM) || (st == S_CFG);
wire        take_live  = is_img & hps_wr & ~switching & ~hold_vld;
wire        take_hold  = is_img & hold_vld & ~switching;
wire        take       = take_live | take_hold;
wire [15:0] wdata      = take_hold ? hold_data : hps_dout;

assign v_wait = is_img & (switching || hold_vld);

always @(posedge clk) begin
    v_wr      <= 1'b0;
    img_done  <= 1'b0;
    img_start <= 1'b0;
    dl_d      <= hps_download;

    if (rst) begin
        st <= S_IDLE; gap <= 4'd0; pos <= 32'd0; is_img <= 1'b0; is_pack <= 1'b0;
        cur <= 5'd0; sw_next <= 5'd0; cfg_k <= 2'd0; nreg <= 5'd0; cfg <= 32'd0;
        endm_addr <= 27'd0; endm_last <= 1'b0;
        pack_len <= 32'd0; hold_vld <= 1'b0; hold_data <= 16'd0;
        v_download <= 1'b0; v_index <= 8'd0; v_addr <= 27'd0; v_dout <= 16'd0;
        v_paddr <= 28'd0; pack_present <= 1'b0; dbg_cfg <= 32'd0; dbg_region <= 5'd0;
    end else begin
        // ------------------------------------------------ start / end ---
        if (img_start_w) begin
            is_img <= 1'b1; is_pack <= 1'b0; pos <= 32'd0; cur <= 5'd0;
            nreg <= 5'd0; pack_len <= 32'd0; hold_vld <= 1'b0;
            st <= S_HDR; img_start <= 1'b1;
            v_download <= 1'b1; v_index <= IMG_INDEX;
        end
        if (pack_start_w) begin
            is_pack <= 1'b1; is_img <= 1'b0; pos <= 32'd0;
            v_download <= 1'b1; v_index <= PACK_VIDX;
        end
        if (!hps_download && dl_d) begin
            if (is_img || is_pack) begin
                img_done     <= 1'b1;
                pack_present <= is_pack | (pack_len != 32'd0);
            end
            is_img <= 1'b0; is_pack <= 1'b0; st <= S_IDLE; hold_vld <= 1'b0;
            v_download <= 1'b0;
        end

        // ---------------------------------------------- pass-through ---
        if (!is_img && !is_pack && !img_start_w && !pack_start_w) begin
            v_download <= hps_download;
            v_index    <= hps_index;
            if (hps_wr) begin
                v_wr   <= 1'b1;
                v_addr <= hps_addr;
                v_dout <= hps_dout;
            end
        end

        // ----------------------------------------------- bare pack ------
        if (is_pack && hps_wr) begin
            v_wr    <= 1'b1;
            v_addr  <= pos[26:0];
            v_paddr <= pos[27:0];
            v_dout  <= hps_dout;
            pos     <= pos + 32'd2;
        end

        // ----------------------------------------------- image ----------
        if (is_img) begin
            // park a live word that arrives during a switch
            if (hps_wr && (switching || hold_vld)) begin
                hold_vld  <= 1'b1;
                hold_data <= hps_dout;
            end
            if (take_hold) hold_vld <= 1'b0;

            case (st)
            S_HDR: if (take) begin
                pos <= pos + 32'd2;
                case (hw)
                    12'h00c: cfg[15:0]  <= wdata;
                    12'h00e: cfg[31:16] <= wdata;
                    12'h014: nreg       <= wdata[4:0];
                    12'h06c: pack_len[15:0]  <= wdata;
                    12'h06e: pack_len[31:16] <= wdata;
                    default: ;
                endcase
                if (hdr_entry) begin
                    case (efld)
                        4'd0:  begin r_dest[ent] <= wdata[7:0]; r_flags[ent] <= wdata[15:8]; end
                        4'd4:  r_off [ent][15:0]  <= wdata;
                        4'd6:  r_off [ent][31:16] <= wdata;
                        4'd8:  r_len [ent][15:0]  <= wdata;
                        4'd10: r_len [ent][31:16] <= wdata;
                        4'd12: r_base[ent][15:0]  <= wdata;
                        4'd14: r_base[ent][31:16] <= wdata;
                        default: ;
                    endcase
                end
                if (hw == 12'hffe) begin         // header complete
                    dbg_cfg    <= cfg;
                    v_addr     <= 27'd0;
                    v_download <= 1'b0;
                    sw_next    <= 5'd0;          // config record next
                    gap        <= GAP[3:0];
                    st         <= S_SW;
                end
            end

            S_SW: begin
                // download low for GAP clocks, then raise the next index
                if (gap != 4'd0) gap <= gap - 4'd1;
                else begin
                    if (sw_next == 5'd0) begin
                        v_index <= 8'd10; cfg_k <= 2'd0; st <= S_CFG;
                    end else begin
                        cur     <= sw_next - 5'd1;
                        v_index <= (r_flags[sw_next - 5'd1][7]) ? PACK_VIDX : r_dest[sw_next - 5'd1];
                        st      <= S_REG;
                    end
                    v_download <= 1'b1;
                end
            end

            S_CFG: begin
                // three consecutive words on index 10 (the memcp record path)
                v_wr <= 1'b1;
                case (cfg_k)
                    2'd0: begin v_addr <= 27'd0; v_dout <= 16'h8000; end
                    2'd1: begin v_addr <= 27'd2; v_dout <= cfg[15:0];  end
                    default: begin v_addr <= 27'd4; v_dout <= cfg[31:16]; end
                endcase
                cfg_k <= cfg_k + 2'd1;
                if (cfg_k == 2'd2) begin
                    endm_addr <= 27'd6;          // "one past" like hps_io
                    endm_last <= (nreg == 5'd0);
                    sw_next   <= 5'd1;
                    st        <= S_ENDM;
                end
            end

            S_ENDM: begin
                // the last write went out on the previous clock: now the end
                // marker the core samples when the download drops
                v_addr     <= endm_addr;
                v_download <= 1'b0;
                gap        <= GAP[3:0];
                st         <= endm_last ? S_DONE : S_SW;
            end

            S_REG: if (take) begin
                pos <= pos + 32'd2;
                if (pos >= r_off[cur] && pos < cur_end) begin
                    if (!cur_skip) begin
                        v_wr    <= 1'b1;
                        v_addr  <= r_base[cur][26:0] + (pos[26:0] - r_off[cur][26:0]);
                        v_paddr <= pos[27:0] - r_off[cur][27:0];
                        v_dout  <= wdata;
                    end
                    dbg_region <= cur;
                    if (pos + 32'd2 >= cur_end) begin
                        // region complete: end marker next clock, then switch
                        endm_addr <= r_base[cur][26:0] + r_len[cur][26:0];
                        endm_last <= !(cur + 5'd1 < nreg);
                        sw_next   <= cur + 5'd2;
                        st        <= S_ENDM;
                    end
                end
                // padding words (pos < r_off[cur]) are dropped
            end

            S_DONE: if (take) pos <= pos + 32'd2;   // trailing bytes, if any
            default: ;
            endcase
        end
    end
end

endmodule
