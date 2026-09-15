/*  NG+ — ngplus_router: .nga image router on the OSD file-load stream
    ==================================================================

    The NG+ core is a generic core to the MiSTer firmware: the launcher's
    "Load Game" entry streams one kit-built image (kit/nga_format.py) over
    the ioctl file channel (16-bit words, WIDE hps_io, one `wr` per word).
    This block sits between hps_io and the stock per-index load paths and
    presents the image body as the virtual (index, addr, data) stream the
    stock firmware would have produced for the same game:

      * bytes 0..4095: the header.  Captures cfg[31:0], the region count and
        the 16-entry region table {dest, flags, file offset, length, base}.
        After the header it emits the core's config record on index 10
        (word 0 = 0x8000 so cp_op stays 0, word 2 = cfg[15:0], word 4 =
        cfg[31:16]) — exactly what the stock loader's notify_conf() sends.
      * region k: words at file positions [off_k, off_k + len_k) are emitted
        with index = dest_k and address = base_k + (pos - off_k).  Padding
        between regions is dropped.  A system-ROM region (index 0) whose
        BIOS_UNI flag differs from `bios_uni` is consumed silently, so the
        OSD "BIOS" choice selects between the two copies in the image.
      * the PACK region (flag 0x80) is emitted with index 0xff and address
        = its offset within the pack; neogeo.sv routes index 0xff into the
        DDR pack window through the same write port V ROMs use.
      * download end -> `img_done` pulse (game top: boot the pack loader).

    Index 2 ("Load Arranged Pack", a bare .cpk): passed through as index
    0xff from address 0 with `img_done` at the end.  Every other index is
    passed through unchanged.

    The file position is counted here in 32 bits: hps_io's ioctl_addr is
    27 bits and images exceed 128 MiB.  Verilog-2005.
*/

module ngplus_router #(parameter
    IMG_INDEX  = 8'd1,
    PACK_INDEX = 8'd2,
    PACK_VIDX  = 8'hff
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

    // virtual ioctl to the core
    output reg        v_download,
    output reg [ 7:0] v_index,
    output reg        v_wr,
    output reg [26:0] v_addr,
    output reg [15:0] v_dout,

    output reg        img_done,          // 1-clk pulse after an image / pack load
    output reg        pack_present,      // valid with img_done: the load carried a pack
    output reg [27:0] v_paddr,           // pack byte offset for index 0xff words (28 bits: > 128 MiB)
    output reg [31:0] dbg_cfg,
    output reg [ 4:0] dbg_region
);

localparam [31:0] HDR_BYTES = 32'd4096;
localparam        MAXR      = 16;

reg  [31:0] pos;                         // file byte position of the next word
reg         dl_d;
reg         is_img, is_pack;

// header capture
reg  [31:0] cfg;
reg  [ 4:0] nreg;
reg  [ 7:0] r_dest  [0:MAXR-1];
reg  [ 7:0] r_flags [0:MAXR-1];
reg  [31:0] r_off   [0:MAXR-1];
reg  [31:0] r_len   [0:MAXR-1];
reg  [31:0] r_base  [0:MAXR-1];

// region walk
reg  [ 4:0] cur;                         // current region index
reg  [31:0] cur_end;                     // off + len of the current region
reg         cur_skip;                    // consume without emitting
reg  [ 2:0] cfg_seq;                     // config record emitter
reg  [31:0] pack_len;                    // header 0x06c: 0 = no pack in this image

wire        img_start  = hps_download & ~dl_d & (hps_index == IMG_INDEX);
wire        pack_start = hps_download & ~dl_d & (hps_index == PACK_INDEX);
wire        in_hdr     = pos < HDR_BYTES;

// header word decode (little-endian words at even byte positions)
wire [11:0] hw = pos[11:0];              // byte offset within the header
wire        hdr_entry = (hw >= 12'h080) && (hw < 12'h080 + MAXR*16);
wire [ 3:0] ent  = (hw - 12'h080) >> 4;
wire [ 3:0] efld = hw[3:0];              // 0 dest/flags, 4/6 offset, 8/10 length, 12/14 base

integer i;
always @(posedge clk) begin
    v_wr     <= 1'b0;
    img_done <= 1'b0;
    dl_d     <= hps_download;

    if (rst) begin
        is_img <= 1'b0; is_pack <= 1'b0; pos <= 32'd0; cur <= 5'd0;
        cur_skip <= 1'b0; cur_end <= 32'd0; cfg_seq <= 3'd0; nreg <= 5'd0;
        v_download <= 1'b0; v_index <= 8'd0; v_addr <= 27'd0; v_dout <= 16'd0;
        cfg <= 32'd0; dbg_cfg <= 32'd0; dbg_region <= 5'd0;
        pack_present <= 1'b0; pack_len <= 32'd0; v_paddr <= 28'd0;
    end else begin
        // ------------------------------------------------ start / end ---
        if (img_start) begin
            is_img <= 1'b1; is_pack <= 1'b0; pos <= 32'd0; cur <= 5'd0;
            cur_skip <= 1'b0; cfg_seq <= 3'd0; nreg <= 5'd0; pack_len <= 32'd0;
        end
        if (pack_start) begin
            is_pack <= 1'b1; is_img <= 1'b0; pos <= 32'd0;
        end
        if (!hps_download && dl_d) begin
            if (is_img || is_pack) begin
                img_done     <= 1'b1;
                pack_present <= is_pack | (pack_len != 32'd0);
            end
            is_img <= 1'b0; is_pack <= 1'b0;
        end

        // ---------------------------------------------- pass-through ---
        v_download <= hps_download;
        if (!is_img && !is_pack && !img_start && !pack_start) begin
            v_index <= hps_index;
            if (hps_wr) begin
                v_wr   <= 1'b1;
                v_addr <= hps_addr;
                v_dout <= hps_dout;
            end
        end

        // ----------------------------------------------- bare pack ------
        if (is_pack && hps_wr) begin
            v_index <= PACK_VIDX;
            v_wr    <= 1'b1;
            v_addr  <= pos[26:0];
            v_paddr <= pos[27:0];
            v_dout  <= hps_dout;
            pos     <= pos + 32'd2;
        end

        // ----------------------------------------------- image ----------
        if (is_img && hps_wr) begin
            pos <= pos + 32'd2;
            if (in_hdr) begin
                case (hw)
                    12'h00c: cfg[15:0]  <= hps_dout;
                    12'h00e: cfg[31:16] <= hps_dout;
                    12'h014: nreg       <= hps_dout[4:0];
                    12'h06c: pack_len[15:0]  <= hps_dout;
                    12'h06e: pack_len[31:16] <= hps_dout;
                    default: ;
                endcase
                if (hdr_entry) begin
                    case (efld)
                        4'd0:  begin r_dest[ent] <= hps_dout[7:0]; r_flags[ent] <= hps_dout[15:8]; end
                        4'd4:  r_off [ent][15:0]  <= hps_dout;
                        4'd6:  r_off [ent][31:16] <= hps_dout;
                        4'd8:  r_len [ent][15:0]  <= hps_dout;
                        4'd10: r_len [ent][31:16] <= hps_dout;
                        4'd12: r_base[ent][15:0]  <= hps_dout;
                        4'd14: r_base[ent][31:16] <= hps_dout;
                        default: ;
                    endcase
                end
                if (hw == 12'hffe) begin         // last header word: arm region 0
                    cfg_seq  <= 3'd1;
                    cur      <= 5'd0;
                    dbg_cfg  <= cfg;
                end
            end else if (cur < nreg) begin
                // region walk on the byte position that this word occupies
                if (pos >= r_off[cur] && pos < r_off[cur] + r_len[cur]) begin
                    if (!( r_dest[cur] == 8'd0 && (r_flags[cur][0] != bios_uni) )) begin
                        v_wr    <= 1'b1;
                        v_index <= (r_flags[cur][7]) ? PACK_VIDX : r_dest[cur];
                        v_addr  <= r_base[cur][26:0] + (pos[26:0] - r_off[cur][26:0]);
                        v_paddr <= pos[27:0] - r_off[cur][27:0];
                        v_dout  <= hps_dout;
                    end
                    dbg_region <= cur;
                end
                // advance past a finished region (may skip several padding words)
                if (pos + 32'd2 >= r_off[cur] + r_len[cur])
                    cur <= cur + 5'd1;
            end
        end

        // ------------------------------------- config record after header
        // emitted in the gaps (the padding after the header is >= 0 bytes,
        // so use three idle clocks; hps_io words arrive many clocks apart)
        if (cfg_seq != 3'd0 && !(is_img && hps_wr)) begin
            v_index <= 8'd10;
            v_wr    <= 1'b1;
            case (cfg_seq)
                3'd1: begin v_addr <= 27'd0; v_dout <= 16'h8000; end   // cp_op = 0
                3'd2: begin v_addr <= 27'd2; v_dout <= cfg[15:0];  end
                3'd3: begin v_addr <= 27'd4; v_dout <= cfg[31:16]; end
                default: ;
            endcase
            cfg_seq <= (cfg_seq == 3'd3) ? 3'd0 : cfg_seq + 3'd1;
        end
    end
end

endmodule
