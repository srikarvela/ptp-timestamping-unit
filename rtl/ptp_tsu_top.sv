// =============================================================================
// ptp_tsu_top.sv
//
// PTP Timestamping Unit, top level.
//
//   network clock domain (net_clk)          AXI clock domain (aclk)
//   ────────────────────────────────         ──────────────────────────
//   ptp_clock_core                           axi_lite_regs
//   ptp_ts_capture  x2 (in / out)   <──────  cdc_bus_bridge  (registers)
//   ptp_latency_hist                ──────>  ptp_time_snapshot (time read)
//   ptp_net_regs
// =============================================================================

`default_nettype none

module ptp_tsu_top #(
    parameter int SEC_W           = 48,
    parameter int NS_W            = 32,
    parameter int FRAC_W          = 32,
    parameter int INT_W           = 8,
    parameter int SUB_BITS        = 2,
    parameter int FIFO_DEPTH_LOG2 = 4,
    parameter int EVENT_SYNC_STAGES = 0
)(
    // ---- network clock domain ----
    input  wire              net_clk,
    input  wire              net_rst_n,
    input  wire              event_in,       // ingress timestamp strobe
    input  wire              event_out,      // egress timestamp strobe
    output logic             pps,
    output logic [SEC_W-1:0] time_sec,       // live time (for in-domain users)
    output logic [NS_W-1:0]  time_ns,

    // ---- AXI4-Lite ----
    input  wire              aclk,
    input  wire              aresetn,
    input  wire  [11:0]      s_axi_awaddr,
    input  wire              s_axi_awvalid,
    output logic             s_axi_awready,
    input  wire  [31:0]      s_axi_wdata,
    input  wire  [3:0]       s_axi_wstrb,
    input  wire              s_axi_wvalid,
    output logic             s_axi_wready,
    output logic [1:0]       s_axi_bresp,
    output logic             s_axi_bvalid,
    input  wire              s_axi_bready,
    input  wire  [11:0]      s_axi_araddr,
    input  wire              s_axi_arvalid,
    output logic             s_axi_arready,
    output logic [31:0]      s_axi_rdata,
    output logic [1:0]       s_axi_rresp,
    output logic             s_axi_rvalid,
    input  wire              s_axi_rready
);

    localparam int BIN_W = 5 + SUB_BITS;

    // ---- clock core --------------------------------------------------------
    logic [INT_W+FRAC_W-1:0]  nom_incr;
    logic signed [FRAC_W-1:0] freq_adj;
    logic                     set_valid, adj_valid;
    logic [SEC_W-1:0]         set_sec;
    logic [NS_W-1:0]          set_ns;
    logic signed [31:0]       adj_sec, adj_ns;
    logic [FRAC_W-1:0]        time_frac;

    ptp_clock_core #(.SEC_W(SEC_W), .NS_W(NS_W), .FRAC_W(FRAC_W), .INT_W(INT_W)) u_clk (
        .clk(net_clk), .rst_n(net_rst_n),
        .nom_incr(nom_incr), .freq_adj(freq_adj),
        .set_valid(set_valid), .set_sec(set_sec), .set_ns(set_ns),
        .adj_valid(adj_valid), .adj_sec(adj_sec), .adj_ns(adj_ns),
        .time_sec(time_sec), .time_ns(time_ns), .time_frac(time_frac), .pps(pps)
    );

    // ---- timestamp capture (ingress / egress) ------------------------------
    logic                     in_valid, in_ready, out_valid, out_ready, drop_clear;
    logic [SEC_W-1:0]         in_sec, out_sec;
    logic [NS_W-1:0]          in_ns, out_ns;
    logic [FIFO_DEPTH_LOG2:0] fifo_in_count, fifo_out_count;
    logic [15:0]              drop_in, drop_out;

    ptp_ts_capture #(.SEC_W(SEC_W), .NS_W(NS_W), .FIFO_DEPTH_LOG2(FIFO_DEPTH_LOG2),
                     .EVENT_SYNC_STAGES(EVENT_SYNC_STAGES)) u_cap_in (
        .clk(net_clk), .rst_n(net_rst_n), .time_sec(time_sec), .time_ns(time_ns),
        .event_in(event_in),
        .ts_valid(in_valid), .ts_sec(in_sec), .ts_ns(in_ns), .ts_ready(in_ready),
        .fifo_count(fifo_in_count), .drop_count(drop_in), .drop_clear(drop_clear)
    );

    ptp_ts_capture #(.SEC_W(SEC_W), .NS_W(NS_W), .FIFO_DEPTH_LOG2(FIFO_DEPTH_LOG2),
                     .EVENT_SYNC_STAGES(EVENT_SYNC_STAGES)) u_cap_out (
        .clk(net_clk), .rst_n(net_rst_n), .time_sec(time_sec), .time_ns(time_ns),
        .event_in(event_out),
        .ts_valid(out_valid), .ts_sec(out_sec), .ts_ns(out_ns), .ts_ready(out_ready),
        .fifo_count(fifo_out_count), .drop_count(drop_out), .drop_clear(drop_clear)
    );

    // ---- latency histogram -------------------------------------------------
    logic             hist_clear, hist_busy, hist_rd_req, hist_rd_ack;
    logic [BIN_W-1:0] hist_rd_addr;
    logic [31:0]      hist_rd_data, stat_count, stat_min, stat_max, stat_neg;
    logic [63:0]      stat_sum;

    ptp_latency_hist #(.SEC_W(SEC_W), .NS_W(NS_W), .SUB_BITS(SUB_BITS)) u_hist (
        .clk(net_clk), .rst_n(net_rst_n),
        .in_valid(in_valid), .in_sec(in_sec), .in_ns(in_ns), .in_ready(in_ready),
        .out_valid(out_valid), .out_sec(out_sec), .out_ns(out_ns), .out_ready(out_ready),
        .clear(hist_clear), .busy(hist_busy),
        .rd_req(hist_rd_req), .rd_addr(hist_rd_addr), .rd_ack(hist_rd_ack), .rd_data(hist_rd_data),
        .stat_count(stat_count), .stat_sum(stat_sum), .stat_min(stat_min),
        .stat_max(stat_max), .stat_neg(stat_neg)
    );

    // ---- network-domain register file --------------------------------------
    logic        s_valid, s_we, s_rvalid;
    logic [11:0] s_addr;
    logic [31:0] s_wdata, s_rdata;

    ptp_net_regs #(.SEC_W(SEC_W), .NS_W(NS_W), .FRAC_W(FRAC_W), .INT_W(INT_W),
                   .BIN_W(BIN_W), .FIFO_CNT_W(FIFO_DEPTH_LOG2+1)) u_regs (
        .clk(net_clk), .rst_n(net_rst_n),
        .s_valid(s_valid), .s_addr(s_addr), .s_wdata(s_wdata), .s_we(s_we),
        .s_rvalid(s_rvalid), .s_rdata(s_rdata),
        .nom_incr(nom_incr), .freq_adj(freq_adj),
        .set_valid(set_valid), .set_sec(set_sec), .set_ns(set_ns),
        .adj_valid(adj_valid), .adj_sec(adj_sec), .adj_ns(adj_ns),
        .hist_clear(hist_clear), .hist_busy(hist_busy),
        .hist_rd_req(hist_rd_req), .hist_rd_addr(hist_rd_addr),
        .hist_rd_ack(hist_rd_ack), .hist_rd_data(hist_rd_data),
        .stat_count(stat_count), .stat_sum(stat_sum), .stat_min(stat_min),
        .stat_max(stat_max), .stat_neg(stat_neg),
        .drop_clear(drop_clear), .drop_in(drop_in), .drop_out(drop_out),
        .fifo_in_count(fifo_in_count), .fifo_out_count(fifo_out_count)
    );

    // ---- CDC: register bridge + time snapshot ------------------------------
    logic        br_req, br_we, br_busy, br_done;
    logic [11:0] br_addr;
    logic [31:0] br_wdata, br_rdata;

    cdc_bus_bridge #(.ADDR_W(12), .DATA_W(32)) u_bridge (
        .dst_clk(aclk), .dst_rst_n(aresetn),
        .m_req(br_req), .m_addr(br_addr), .m_wdata(br_wdata), .m_we(br_we),
        .m_busy(br_busy), .m_done(br_done), .m_rdata(br_rdata),
        .src_clk(net_clk), .src_rst_n(net_rst_n),
        .s_valid(s_valid), .s_addr(s_addr), .s_wdata(s_wdata), .s_we(s_we),
        .s_rvalid(s_rvalid), .s_rdata(s_rdata)
    );

    logic             snap_req, snap_busy, snap_done;
    logic [SEC_W-1:0] snap_sec;
    logic [NS_W-1:0]  snap_ns;

    ptp_time_snapshot #(.SEC_W(SEC_W), .NS_W(NS_W)) u_snap (
        .src_clk(net_clk), .src_rst_n(net_rst_n),
        .src_time_sec(time_sec), .src_time_ns(time_ns),
        .dst_clk(aclk), .dst_rst_n(aresetn),
        .dst_req(snap_req), .dst_busy(snap_busy), .dst_done(snap_done),
        .dst_sec(snap_sec), .dst_ns(snap_ns)
    );

    // ---- AXI-Lite slave ----------------------------------------------------
    axi_lite_regs #(.SEC_W(SEC_W), .NS_W(NS_W)) u_axi (
        .aclk(aclk), .aresetn(aresetn),
        .s_axi_awaddr, .s_axi_awvalid, .s_axi_awready,
        .s_axi_wdata, .s_axi_wstrb, .s_axi_wvalid, .s_axi_wready,
        .s_axi_bresp, .s_axi_bvalid, .s_axi_bready,
        .s_axi_araddr, .s_axi_arvalid, .s_axi_arready,
        .s_axi_rdata, .s_axi_rresp, .s_axi_rvalid, .s_axi_rready,
        .snap_req(snap_req), .snap_busy(snap_busy), .snap_done(snap_done),
        .snap_sec(snap_sec), .snap_ns(snap_ns),
        .br_req(br_req), .br_addr(br_addr), .br_wdata(br_wdata), .br_we(br_we),
        .br_busy(br_busy), .br_done(br_done), .br_rdata(br_rdata)
    );

endmodule

`default_nettype wire
