// =============================================================================
// ptp_latency_hist.sv
//
// Latency histogram engine: a hardware profiler for  delta = t_out - t_in.
//
//   Pairing:  the n-th egress timestamp is matched with the n-th ingress
//             timestamp (in-order pipeline assumption).  Both come from
//             ptp_ts_capture FIFOs; a sample is taken whenever both are
//             valid.
//
//   Delta:    sec_diff = out_sec - in_sec
//             delta    = sec_diff * 1e9 + (out_ns - in_ns)
//               sec_diff in 0..5 : computed exactly (constant mux, one add)
//               delta   >= 2^32  : saturates to 2^32-1  (last bin)
//               delta   <  0     : counted in neg_count, not binned
//
//   Binning:  log2 with SUB_BITS sub-bins per octave (HdrHistogram style),
//             dense index space:
//               delta <  2^(SUB_BITS+1):  bin = delta            (linear)
//               otherwise:                msb = highest set bit of delta
//                                         sub = the SUB_BITS bits below it
//                                         bin = {msb, sub} - OFFSET
//             where OFFSET = ((SUB_BITS+1) << SUB_BITS) - 2^(SUB_BITS+1)
//             makes the first log region start right after the linear one.
//             With SUB_BITS = 2 the bin lower edges are
//               0 1 2 3 4 5 6 7 8 10 12 14 16 20 24 28 32 40 ... ns
//             (25 % relative width) up to 2^32 ns, 124 bins used of 128.
//             The msb priority encoder is nearly free in LUTs; the sub-bit
//             select is one 32:1 mux per sub bit.
//
//   Storage:  bin counters live in a (* ram_style = "block" *) RAM,
//             read-modify-write with one-deep forwarding so back-to-back
//             samples in the same bin are counted correctly (and the
//             read-during-write collision value is never used).
//
//   Stats:    count, sum (64b), min, max, neg_count -> mean = sum / count
//             computed by software (no divider in hardware).
//
//   Host access: `rd_addr/rd_req -> rd_data/rd_ack` reads one bin; the
//             sample pipeline is stalled for that cycle so the read port is
//             free.  `clear` zeroes all bins and stats (NBINS cycles,
//             `busy` high, sampling stalled).  The same sweep runs
//             automatically out of reset, so the RAM contents are defined
//             without relying on a BRAM initialisation file.
// =============================================================================

`default_nettype none

module ptp_latency_hist #(
    parameter int SEC_W    = 48,
    parameter int NS_W     = 32,
    parameter int SUB_BITS = 2,
    parameter int CNT_W    = 32
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // ingress timestamps (from ptp_ts_capture)
    input  wire                     in_valid,
    input  wire  [SEC_W-1:0]        in_sec,
    input  wire  [NS_W-1:0]         in_ns,
    output logic                    in_ready,

    // egress timestamps
    input  wire                     out_valid,
    input  wire  [SEC_W-1:0]        out_sec,
    input  wire  [NS_W-1:0]         out_ns,
    output logic                    out_ready,

    // control / status
    input  wire                     clear,
    output logic                    busy,

    // bin read port
    input  wire                     rd_req,
    input  wire  [5+SUB_BITS-1:0]   rd_addr,
    output logic                    rd_ack,
    output logic [CNT_W-1:0]        rd_data,

    // statistics
    output logic [CNT_W-1:0]        stat_count,
    output logic [63:0]             stat_sum,
    output logic [31:0]             stat_min,
    output logic [31:0]             stat_max,
    output logic [CNT_W-1:0]        stat_neg
);

    localparam int BIN_W  = 5 + SUB_BITS;
    localparam int NBINS  = 1 << BIN_W;
    localparam int LIN_N  = 1 << (SUB_BITS + 1);                     // linear region size
    localparam int OFFSET = ((SUB_BITS + 1) << SUB_BITS) - LIN_N;   // index shift for log region

    // ------------------------------------------------------------------
    // Clear state machine
    // ------------------------------------------------------------------
    logic             clearing;
    logic [BIN_W-1:0] clr_addr;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            clearing <= 1'b1;                    // sweep the RAM out of reset
            clr_addr <= '0;
        end else if (clear && !clearing) begin
            clearing <= 1'b1;
            clr_addr <= '0;
        end else if (clearing) begin
            clr_addr <= clr_addr + 1'b1;
            if (clr_addr == BIN_W'(NBINS-1)) clearing <= 1'b0;
        end
    end

    assign busy = clearing;

    // ------------------------------------------------------------------
    // Host bin read: held until the pipeline's read slot is free (a sample
    // taken the cycle before a request still needs the read port), then
    // served in one cycle.  Sampling is stalled while a read is pending.
    // ------------------------------------------------------------------
    logic             rd_wait, rd_go, rd_pend;
    logic [BIN_W-1:0] rd_addr_q;
    logic             s1_valid;

    assign rd_go = (rd_req | rd_wait) & ~s1_valid;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            rd_wait <= 1'b0;
            rd_pend <= 1'b0;
            rd_ack  <= 1'b0;
        end else begin
            if (rd_req)  rd_addr_q <= rd_addr;
            if (rd_req && !rd_go) rd_wait <= 1'b1;
            else if (rd_go)       rd_wait <= 1'b0;
            rd_pend <= rd_go;
            rd_ack  <= rd_pend;
        end
    end

    // ------------------------------------------------------------------
    // Stage 0: take a sample when both FIFOs have data and no one else
    // needs the RAM ports.
    // ------------------------------------------------------------------
    logic take;
    assign take      = in_valid & out_valid & ~clearing & ~rd_req & ~rd_wait;
    assign in_ready  = take;
    assign out_ready = take;

    // ------------------------------------------------------------------
    // Stage 1: delta and bin (registered)
    // ------------------------------------------------------------------
    logic signed [SEC_W:0]  sec_diff;
    logic signed [34:0]     ns_diff;         // out_ns - in_ns, in (-1e9, 1e9)
    logic signed [34:0]     sec_ns;          // sec_diff * 1e9 for sec_diff 0..5
    logic signed [34:0]     full_c;          // up to 5e9 + 1e9 -> 35-bit signed
    logic [31:0]            delta_c;
    logic                   neg_c, sat_c;

    always_comb begin
        sec_diff = $signed({1'b0, out_sec}) - $signed({1'b0, in_sec});
        ns_diff  = $signed({3'b000, out_ns}) - $signed({3'b000, in_ns});
        case (sec_diff[2:0])
            3'd0:    sec_ns = 35'sd0;
            3'd1:    sec_ns = 35'sd1_000_000_000;
            3'd2:    sec_ns = 35'sd2_000_000_000;
            3'd3:    sec_ns = 35'sd3_000_000_000;
            3'd4:    sec_ns = 35'sd4_000_000_000;
            default: sec_ns = 35'sd5_000_000_000;   // sec_diff 5 with negative ns can still fit
        endcase
        full_c  = sec_ns + ns_diff;
        neg_c   = 1'b0;
        sat_c   = 1'b0;
        delta_c = full_c[31:0];
        if (sec_diff < 0 || (sec_diff == 0 && ns_diff < 0)) begin
            neg_c = 1'b1;
        end else if (sec_diff > 5 || full_c[34:32] != 3'b000) begin
            sat_c   = 1'b1;
            delta_c = 32'hFFFF_FFFF;
        end
    end

    // priority encoder: msb position
    logic [4:0] msb_c;
    always_comb begin
        msb_c = '0;
        for (int i = 0; i < 32; i++) if (delta_c[i]) msb_c = 5'(i);
    end

    // sub-bin bits: the SUB_BITS bits just below the msb; dense index
    logic [SUB_BITS-1:0] sub_c;
    logic [31:0]         shifted_c;
    logic [BIN_W-1:0]    bin_c;
    always_comb begin
        shifted_c = delta_c << (5'd31 - msb_c);      // msb now at bit 31
        sub_c     = shifted_c[30 -: SUB_BITS];
        if (delta_c < LIN_N) bin_c = BIN_W'(delta_c);
        else                 bin_c = {msb_c, sub_c} - BIN_W'(OFFSET);
    end

    logic             s1_neg;
    logic [31:0]      s1_delta;
    logic [BIN_W-1:0] s1_bin;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            s1_valid <= 1'b0;
        end else begin
            s1_valid <= take;
            s1_neg   <= neg_c;
            s1_delta <= delta_c;
            s1_bin   <= bin_c;
        end
    end

    // ------------------------------------------------------------------
    // Bin RAM: port A = read (pipeline or host), port B = write
    //
    //   cycle N   : sample X in stage 1, read bin_mem[X.bin]
    //   cycle N+1 : X in stage 2, ra_data valid, write bin_mem[X.bin] <= +1
    //
    //   A sample Y one cycle behind X into the same bin reads the RAM in
    //   cycle N+1, the same cycle X writes it, and would get the stale
    //   value: Y instead takes the write data from the registered copy of
    //   the last write (fwd_hit).  A sample two cycles behind reads after
    //   the write has landed and needs nothing.  The RAM's read-during-
    //   write collision output is therefore never used.
    // ------------------------------------------------------------------
    (* ram_style = "block" *) logic [CNT_W-1:0] bin_mem [0:NBINS-1];

    logic [BIN_W-1:0] ra_addr;
    logic [CNT_W-1:0] ra_data;
    logic             we;
    logic [BIN_W-1:0] waddr;
    logic [CNT_W-1:0] wdata;
    logic             last_we;
    logic [BIN_W-1:0] last_addr;
    logic [CNT_W-1:0] last_data;

    // stage 2 state
    logic             s2_valid;
    logic [BIN_W-1:0] s2_bin;
    logic [31:0]      s2_delta;
    logic             fwd_hit;
    logic [CNT_W-1:0] cur_val, new_val;

    assign ra_addr = rd_go ? (rd_req ? rd_addr : rd_addr_q) : s1_bin;

    assign fwd_hit = last_we && (last_addr == s2_bin);
    assign cur_val = fwd_hit ? last_data : ra_data;
    assign new_val = (&cur_val) ? cur_val : cur_val + 1'b1;     // saturate

    assign we    = clearing | s2_valid;
    assign waddr = clearing ? clr_addr : s2_bin;
    assign wdata = clearing ? '0       : new_val;

    always_ff @(posedge clk) begin
        ra_data <= bin_mem[ra_addr];
        if (we) bin_mem[waddr] <= wdata;
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            s2_valid <= 1'b0;
            last_we  <= 1'b0;
        end else begin
            s2_valid  <= s1_valid & ~s1_neg & ~clearing;
            s2_bin    <= s1_bin;
            s2_delta  <= s1_delta;
            last_we   <= we;
            last_addr <= waddr;
            last_data <= wdata;
        end
    end

    // host read result (one cycle after the RAM read)
    always_ff @(posedge clk) begin
        if (rd_pend) rd_data <= ra_data;
    end

    // ------------------------------------------------------------------
    // Statistics (updated at stage 2)
    // ------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (!rst_n || (clear && !clearing)) begin
            stat_count <= '0;
            stat_sum   <= '0;
            stat_min   <= 32'hFFFF_FFFF;
            stat_max   <= '0;
            stat_neg   <= '0;
        end else begin
            if (s2_valid) begin
                if (~&stat_count) stat_count <= stat_count + 1'b1;
                stat_sum <= stat_sum + {32'd0, s2_delta};
                if (s2_delta < stat_min) stat_min <= s2_delta;
                if (s2_delta > stat_max) stat_max <= s2_delta;
            end
            if (s1_valid && s1_neg && !clearing && ~&stat_neg) stat_neg <= stat_neg + 1'b1;
        end
    end

endmodule

`default_nettype wire
