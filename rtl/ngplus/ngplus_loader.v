/*  NG+ — ngplus_loader: .nga image loader for a DDR-resident image
    ==============================================================

    The launcher's "Load Game" entry carries a DDR load address, so the
    MiSTer firmware writes the whole kit-built image (kit/nga_format.py,
    format 2) straight into DDR at 0x30000000 through shared memory — the
    way it stages ROM sets for the stock core and the way CPS+ MRA loads
    work — and the core sees only an empty download on index 1 (download
    high with no words, then low).  Nothing streams over the SPI word
    channel.

    At the download's end this block does what the stock firmware's
    neogeo_loader does after staging each ROM:

      1. reads the image header (first 1 KiB) through ddram.sv's copy port
         into a small dual-clock RAM and checks magic / version;
      2. emits the config record on index 10 (word 0 = 0x8000 so cp_op
         stays 0, word 2 = cfg[15:0], word 4 = cfg[31:16]);
      3. walks the region table.  SDRAM-bound regions (system ROM, SFIX,
         P, S1, C) become memcp records on index 10 (word 0 = dest index,
         words 2/4 = length, word 6 = 1) with `cp_src` pointing at the
         region's DDR offset; the core's own memcp copies the bytes to
         SDRAM and derives the masks / C base from the record, and this
         block waits for `memcp_wait` to clear.  DDR-resident regions (M1,
         V1, V2) get the same record with word 6 = 0 (masks only) and their
         DDR offsets are published as `m1_base` / `v_base`.  The LO ROM
         (64 KiB) is read byte by byte through ddram.sv's third read port
         (borrowed while the Z80 is in reset) and written as index-1 ioctl
         words.  A system-ROM copy whose BIOS_UNI flag differs from
         `bios_uni` is skipped, so the OSD BIOS choice selects the copy.
         The PACK region only sets `pack_base` / `pack_present`.
      4. pulses `img_done`; `busy` drops (the core's reset releases).

    `busy` holds the core in reset from the download start to `img_done`.
    `img_start` pulses at the download start (reset masks and the C base
    as the stock core does on status[0]); `fill` is high while the loader
    runs so the core's mask-fill step (stock: while status[0]) applies.

    Index 2 ("Load Arranged Pack") is a bare .cpk written by the firmware
    to F2_BASE: at its download end `pack_base` <= F2_BASE, `pack_present`
    <= 1, `img_done` pulses (the pack loader reboots).  Every other index
    is passed through unchanged.  Verilog-2005.
*/

module ngplus_loader #(parameter
    IMG_INDEX  = 8'd1,
    PACK_INDEX = 8'd2,
    IMG_BASE   = 28'h0000000,        // image DDR byte offset (from 0x30000000)
    F2_BASE    = 28'h8000000,        // bare pack DDR byte offset (0x38000000)
    LO_BYTES   = 17'h10000           // LO ROM bytes copied (fixed, even)
)(
    input             clk,           // clk_sys
    input             rst,
    input             bios_uni,      // OSD: 1 = UniBIOS copy, 0 = original

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

    // memcp source (DDR byte offset of the region being copied; static
    // for the whole copy) and the core's copy-in-progress flag
    output reg [27:0] cp_src,
    input             memcp_wait,

    // header capture through ddram.sv's copy port (DDRAM_CLK domain)
    input             dclk,
    input             drst,
    output reg        hdr_cpreq,     // OR into ddram cpreq
    output reg        hdr_sel,       // cpaddr = IMG_BASE while high
    input             cp_busy,
    input             cp_wr,
    input      [63:0] cp_dout,

    // LO ROM copy through ddram.sv read port 3 (toggle handshake)
    output reg        lo_sel,        // port 3 belongs to the loader
    output reg        lo_req,
    output reg [27:0] lo_addr,
    input             lo_ack,
    input      [ 7:0] lo_dout,

    // results
    output            busy,          // hold the core in reset (download start .. img_done)
    output            fill,          // apply the mask-fill step
    output reg        img_start,     // 1-clk pulse: an image load begins
    output reg        img_done,      // 1-clk pulse: image / pack load complete
    output reg        pack_present,  // valid with img_done and after
    output reg [27:0] v_base,        // V ROM window DDR offset (64 KiB aligned)
    output reg [27:0] m1_base,       // M1 ROM DDR offset (64 KiB aligned)
    output reg [27:0] pack_base,     // pack DDR offset
    output reg [31:0] dbg_cfg,
    output reg [ 4:0] dbg_region,
    output reg [ 4:0] dbg_state,
    output reg        dbg_err        // last image load failed (magic / version)
);

localparam [31:0] MAGIC   = 32'h3241474E;   // "NGA2" little-endian
localparam [15:0] VERSION = 16'd2;
localparam [ 4:0] MAXR    = 5'd16;
localparam [16:0] LO_N    = LO_BYTES;
localparam [ 7:0] IDX_MEMCP = 8'd10, IDX_LOROM = 8'd1, IDX_SPROM = 8'd0, IDX_M1 = 8'd9,
                  IDX_V1 = 8'd16, IDX_V2 = 8'd48;

// -------------------------------------------------- header RAM (1 KiB) --
reg  [63:0] hram [0:127];
reg  [63:0] hq;
reg  [ 6:0] haddr;
always @(posedge clk) hq <= hram[haddr];

// ----------------------------------------------- DDRAM_CLK side: capture --
reg         hreq_tgl;                 // clk: toggles per header request
reg  [ 2:0] hreq_s;
reg  [ 6:0] hcnt;
reg         hdone_tgl;                // dclk: toggles per completed capture
reg  [ 1:0] hst;

always @(posedge dclk) begin
    if (drst) begin
        hreq_s <= 3'd0; hcnt <= 7'd0; hdone_tgl <= 1'b0; hdr_cpreq <= 1'b0;
        hdr_sel <= 1'b0; hst <= 2'd0;
    end else begin
        hreq_s <= {hreq_s[1:0], hreq_tgl};
        case (hst)
            2'd0: if (hreq_s[2] != hreq_s[1]) begin
                      hdr_sel <= 1'b1; hdr_cpreq <= 1'b1; hcnt <= 7'd0; hst <= 2'd1;
                  end
            2'd1: if (cp_busy) begin hdr_cpreq <= 1'b0; hst <= 2'd2; end
            2'd2: if (!cp_busy) begin hdr_sel <= 1'b0; hdone_tgl <= ~hdone_tgl; hst <= 2'd0; end
            default: hst <= 2'd0;
        endcase
        if (hdr_sel && cp_wr) begin
            hram[hcnt] <= cp_dout;
            hcnt <= hcnt + 7'd1;
        end
    end
end

// ------------------------------------------------------ clk side: sequencer --
localparam [4:0] S_IDLE   = 5'd0,
                 S_HREQ   = 5'd1,    // header burst requested; wait for the capture
                 S_MAG    = 5'd2,    // magic / version
                 S_CFG    = 5'd3,    // cfg word
                 S_NREG   = 5'd4,    // region count
                 S_REC    = 5'd5,    // record words (cfg or memcp)
                 S_CPW    = 5'd6,    // settle, then wait for the copy to finish
                 S_NEXT   = 5'd7,    // fetch the next region entry
                 S_ENT0   = 5'd8,    // dest / flags / offset
                 S_ENT1   = 5'd9,    // length / base -> dispatch
                 S_LO0    = 5'd10,   // LO: align the request toggle to the port's ack
                 S_LO1    = 5'd11,   // LO: take the port
                 S_LO2    = 5'd12,   // LO: address out
                 S_LO3    = 5'd13,   // LO: request
                 S_LO4    = 5'd14,   // LO: wait for the byte, write it
                 S_LO5    = 5'd15,   // LO: release the port
                 S_FIN    = 5'd16,   // img_done
                 S_ERR    = 5'd17;   // bad image: finish without records

reg  [ 4:0] st;
reg         dl_d;
reg         is_img, is_pack, active;
reg  [ 1:0] wt;                        // header RAM read latency
reg  [ 2:0] hdone_s;
reg  [ 1:0] mw_s, ack_s;
reg  [31:0] cfg;
reg  [ 4:0] nreg, cur;
reg  [ 7:0] e_dest, e_flags;
reg  [27:0] e_off;
reg  [26:0] e_len;
reg  [ 1:0] rk;                        // record word index
reg         rec_cfg, rec_copy;
reg  [ 7:0] rec_idx;
reg  [ 7:0] settle;
reg  [16:0] lo_cnt;
reg  [27:0] lo_src;

wire img_start_w  = hps_download & ~dl_d & (hps_index == IMG_INDEX);
wire pack_start_w = hps_download & ~dl_d & (hps_index == PACK_INDEX);
wire dl_fall      = ~hps_download & dl_d;

assign busy = is_img | active;
assign fill = active;

always @(posedge clk) begin
    v_wr      <= 1'b0;
    img_done  <= 1'b0;
    img_start <= 1'b0;
    dl_d      <= hps_download;
    hdone_s   <= {hdone_s[1:0], hdone_tgl};
    mw_s      <= {mw_s[0], memcp_wait};
    ack_s     <= {ack_s[0], lo_ack};

    if (rst) begin
        st <= S_IDLE; is_img <= 1'b0; is_pack <= 1'b0; active <= 1'b0; wt <= 2'd0;
        hreq_tgl <= 1'b0; cfg <= 32'd0; nreg <= 5'd0; cur <= 5'd0;
        e_dest <= 8'd0; e_flags <= 8'd0; e_off <= 28'd0; e_len <= 27'd0;
        rk <= 2'd0; rec_cfg <= 1'b0; rec_copy <= 1'b0; rec_idx <= 8'd0; settle <= 8'd0;
        lo_cnt <= 17'd0; lo_src <= 28'd0; lo_sel <= 1'b0; lo_req <= 1'b0; lo_addr <= 28'd0;
        v_download <= 1'b0; v_index <= 8'd0; v_addr <= 27'd0; v_dout <= 16'd0;
        cp_src <= 28'd0; pack_present <= 1'b0; v_base <= 28'd0; m1_base <= 28'd0;
        pack_base <= 28'd0; haddr <= 7'd0;
        dbg_cfg <= 32'd0; dbg_region <= 5'd0; dbg_state <= 5'd0; dbg_err <= 1'b0;
    end else begin
        dbg_state <= st;

        // -------------------------------------------- download edges ---
        if (img_start_w) begin
            is_img <= 1'b1; is_pack <= 1'b0; img_start <= 1'b1;
            v_download <= 1'b0;
        end
        if (pack_start_w) begin
            is_pack <= 1'b1; is_img <= 1'b0;
            v_download <= 1'b0;
        end
        if (dl_fall && is_pack) begin
            is_pack      <= 1'b0;
            pack_base    <= F2_BASE;
            pack_present <= 1'b1;
            img_done     <= 1'b1;
        end
        if (dl_fall && is_img && !active) begin
            // the image is in DDR: fetch the header
            active   <= 1'b1;
            hreq_tgl <= ~hreq_tgl;
            cur      <= 5'd0;
            pack_present <= 1'b0;
            dbg_err  <= 1'b0;
            st       <= S_HREQ;
        end

        // ---------------------------------------------- pass-through ---
        if (!is_img && !is_pack && !active && !img_start_w && !pack_start_w) begin
            v_download <= hps_download;
            v_index    <= hps_index;
            if (hps_wr) begin
                v_wr   <= 1'b1;
                v_addr <= hps_addr;
                v_dout <= hps_dout;
            end
        end

        // ------------------------------------------------- sequencer ---
        if (wt != 2'd0) wt <= wt - 2'd1;
        else case (st)
            S_IDLE: ;

            S_HREQ: if (hdone_s[2] != hdone_s[1]) begin
                haddr <= 7'd0; wt <= 2'd1; st <= S_MAG;
            end

            S_MAG: if (hq[31:0] == MAGIC && hq[47:32] == VERSION) begin
                haddr <= 7'd1; wt <= 2'd1; st <= S_CFG;
            end else begin
                dbg_err <= 1'b1; st <= S_ERR;
            end

            S_CFG: begin
                cfg     <= hq[63:32];
                dbg_cfg <= hq[63:32];
                haddr   <= 7'd2; wt <= 2'd1; st <= S_NREG;
            end

            S_NREG: begin
                nreg <= (hq[63:32] > {27'd0, MAXR}) ? MAXR : hq[36:32];
                // config record first
                rec_cfg <= 1'b1; rec_copy <= 1'b0; rk <= 2'd0;
                v_download <= 1'b1; v_index <= IDX_MEMCP;
                st <= S_REC;
            end

            S_REC: begin
                // one word per clock on index 10: 0 / 2 / 4 [/ 6]
                v_wr <= 1'b1;
                v_addr <= {24'd0, rk, 1'b0};
                case (rk)
                    2'd0: v_dout <= rec_cfg ? 16'h8000 : {8'd0, rec_idx};
                    2'd1: v_dout <= rec_cfg ? cfg[15:0] : e_len[15:0];
                    2'd2: v_dout <= rec_cfg ? cfg[31:16] : {5'd0, e_len[26:16]};
                    default: v_dout <= {15'd0, rec_copy};
                endcase
                rk <= rk + 2'd1;
                if ((rec_cfg && rk == 2'd2) || rk == 2'd3) begin
                    settle <= 8'd8; st <= S_CPW;
                end
            end

            S_CPW: begin
                // the core toggles memcp_req on the word-6 write; let it show
                // through the synchroniser, then wait for the copy to finish
                if (settle != 8'd0) settle <= settle - 8'd1;
                else if (!mw_s[1]) begin
                    v_download <= 1'b0;
                    st <= S_NEXT;
                end
            end

            S_NEXT: if (cur < nreg) begin
                haddr <= 7'd16 + {cur, 1'b0}; wt <= 2'd1; st <= S_ENT0;
            end else st <= S_FIN;

            S_ENT0: begin
                e_dest  <= hq[7:0];
                e_flags <= hq[15:8];
                e_off   <= hq[59:32];
                haddr   <= 7'd17 + {cur, 1'b0}; wt <= 2'd1; st <= S_ENT1;
            end

            S_ENT1: begin
                e_len      <= hq[26:0];
                dbg_region <= cur;
                cur        <= cur + 5'd1;
                rec_cfg    <= 1'b0; rk <= 2'd0; rec_idx <= e_dest;
                if (e_flags[7]) begin
                    // pack: stays in DDR
                    pack_base    <= IMG_BASE + e_off;
                    pack_present <= 1'b1;
                    st <= S_NEXT;
                end else if (e_dest == IDX_SPROM && e_flags[0] != bios_uni) begin
                    st <= S_NEXT;                        // the other BIOS copy
                end else if (e_dest == IDX_LOROM) begin
                    lo_src <= IMG_BASE + e_off;
                    st <= S_LO0;
                end else if (e_dest == IDX_M1) begin
                    m1_base <= IMG_BASE + e_off; rec_copy <= 1'b0;
                    v_download <= 1'b1; v_index <= IDX_MEMCP; st <= S_REC;
                end else if (e_dest == IDX_V1) begin
                    v_base <= IMG_BASE + e_off; rec_copy <= 1'b0;
                    v_download <= 1'b1; v_index <= IDX_MEMCP; st <= S_REC;
                end else if (e_dest == IDX_V2) begin
                    rec_copy <= 1'b0;
                    v_download <= 1'b1; v_index <= IDX_MEMCP; st <= S_REC;
                end else if (e_dest == 8'd0 || e_dest == 8'd2 || e_dest == 8'd4 ||
                             e_dest == 8'd5 || e_dest == 8'd6 || e_dest == 8'd8 || e_dest == 8'd15) begin
                    cp_src <= IMG_BASE + e_off; rec_copy <= 1'b1;
                    v_download <= 1'b1; v_index <= IDX_MEMCP; st <= S_REC;
                end else begin
                    st <= S_NEXT;                        // unknown: skip
                end
            end

            // ---- LO ROM: 64 Ki byte reads through port 3, one index-1 word each
            S_LO0: begin
                lo_req <= ack_s[1];                      // port idle: req == ack
                lo_cnt <= 17'd0;
                st <= S_LO1;
            end
            S_LO1: begin
                lo_sel <= 1'b1;
                v_download <= 1'b1; v_index <= IDX_LOROM;
                st <= S_LO2;
            end
            S_LO2: begin
                lo_addr <= lo_src + {11'd0, lo_cnt};
                st <= S_LO3;
            end
            S_LO3: begin
                lo_req <= ~lo_req;
                st <= S_LO4;
            end
            S_LO4: if (ack_s[1] == lo_req) begin
                v_wr   <= 1'b1;
                v_addr <= {10'd0, lo_cnt[15:0], 1'b0};
                v_dout <= {8'd0, lo_dout};
                lo_cnt <= lo_cnt + 17'd1;
                st <= (lo_cnt + 17'd1 == LO_N) ? S_LO5 : S_LO2;
            end
            S_LO5: begin
                lo_sel <= 1'b0;                          // even count: ack == the Z80's req again
                v_download <= 1'b0;
                st <= S_NEXT;
            end

            S_ERR: begin
                v_download <= 1'b0;
                st <= S_FIN;
            end

            S_FIN: begin
                v_download <= 1'b0;
                img_done <= 1'b1;
                active   <= 1'b0;
                is_img   <= 1'b0;
                st <= S_IDLE;
            end

            default: st <= S_IDLE;
        endcase
    end
end

endmodule
