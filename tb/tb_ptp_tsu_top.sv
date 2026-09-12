// =============================================================================
// tb_ptp_tsu_top.sv
//
// System test of ptp_tsu_top through its AXI4-Lite port, with the network
// clock at 156.25 MHz and the AXI clock at 100 MHz (unrelated, random phase).
//
//   T1  ID / VERSION readback (AXI domain)
//   T2  write NOM_INCR / SET time through the bridge, read them back
//   T3  coherent snapshot: CTRL.SNAP_REQ -> poll STATUS -> SNAP_* words;
//       value must lie in [t_req, t_done] measured on the network side
//   T4  FREQ_ADJ = +100 ppm: two snapshots ~1 ms apart, network-side
//       elapsed time scales by 1 + 1e-4 (checked to 1 ns)
//   T5  ADJ_NS phase step of +1234 ns visible in the next snapshot
//   T6  latency histogram: 2000 in/out strobe pairs with known cycle
//       delays; HIST_COUNT, every bin, MIN, MAX, SUM read over AXI must
//       match a reference built from the exact network-side timestamps
//   T7  NET_CTRL.HIST_CLEAR zeroes everything; DROP counters are 0
//
// Run:  make sim-ptp_tsu_top
// =============================================================================

`timescale 1ns/1ps
`default_nettype none

module tb_ptp_tsu_top;

    localparam int  SEC_W = 48, NS_W = 32, SUB_BITS = 2;
    localparam int  BIN_W = 5 + SUB_BITS, NBINS = 1 << BIN_W;
    localparam real NET_PERIOD = 6.4, AXI_PERIOD = 10.0;

    // ---- clocks / resets ------------------------------------------------
    logic net_clk = 0, aclk = 0;
    logic net_rst_n = 0, aresetn = 0;
    always #(NET_PERIOD/2.0) net_clk = ~net_clk;
    initial begin #($urandom_range(0,1000)/1000.0 * NET_PERIOD); forever #(AXI_PERIOD/2.0) aclk = ~aclk; end

    // ---- DUT ---------------------------------------------------------------
    logic             event_in = 0, event_out = 0, pps;
    logic [SEC_W-1:0] time_sec;
    logic [NS_W-1:0]  time_ns;
    logic [11:0] awaddr = 0, araddr = 0;
    logic        awvalid = 0, awready, wvalid = 0, wready, bvalid, bready = 0;
    logic        arvalid = 0, arready, rvalid, rready = 0;
    logic [31:0] wdata = 0, rdata;
    logic [1:0]  bresp, rresp;

    ptp_tsu_top #(.SUB_BITS(SUB_BITS), .FIFO_DEPTH_LOG2(4)) dut (
        .net_clk(net_clk), .net_rst_n(net_rst_n),
        .event_in(event_in), .event_out(event_out), .pps(pps),
        .time_sec(time_sec), .time_ns(time_ns),
        .aclk(aclk), .aresetn(aresetn),
        .s_axi_awaddr(awaddr), .s_axi_awvalid(awvalid), .s_axi_awready(awready),
        .s_axi_wdata(wdata), .s_axi_wstrb(4'hF), .s_axi_wvalid(wvalid), .s_axi_wready(wready),
        .s_axi_bresp(bresp), .s_axi_bvalid(bvalid), .s_axi_bready(bready),
        .s_axi_araddr(araddr), .s_axi_arvalid(arvalid), .s_axi_arready(arready),
        .s_axi_rdata(rdata), .s_axi_rresp(rresp), .s_axi_rvalid(rvalid), .s_axi_rready(rready)
    );

    // ---- register offsets ----------------------------------------------------
    localparam int R_ID = 'h000, R_VERSION = 'h004, R_CTRL = 'h008, R_STATUS = 'h00C,
                   R_SNAP_NS = 'h010, R_SNAP_SEC_LO = 'h014, R_SNAP_SEC_HI = 'h018,
                   R_NOM_INCR_LO = 'h100, R_NOM_INCR_HI = 'h104, R_FREQ_ADJ = 'h108,
                   R_ADJ_SEC = 'h10C, R_ADJ_NS = 'h110, R_SET_SEC_LO = 'h114,
                   R_SET_SEC_HI = 'h118, R_SET_NS = 'h11C, R_HIST_COUNT = 'h120,
                   R_HIST_SUM_LO = 'h124, R_HIST_SUM_HI = 'h128, R_HIST_MIN = 'h12C,
                   R_HIST_MAX = 'h130, R_HIST_NEG = 'h134, R_DROP_IN = 'h138,
                   R_DROP_OUT = 'h13C, R_FIFO_LEVELS = 'h140, R_NET_CTRL = 'h144,
                   R_NET_STATUS = 'h148, R_HIST_BIN = 'h800;

    // ---- AXI-Lite BFM -------------------------------------------------------
    int errors = 0, checks = 0;
    task automatic fail(string msg); errors++; $display("[%0t] FAIL: %s", $time, msg); endtask

    task automatic axi_write(input int addr, input logic [31:0] data);
        @(negedge aclk);
        awaddr = addr; awvalid = 1; wdata = data; wvalid = 1; bready = 1;
        do @(negedge aclk); while (!(awready && wready));
        awvalid = 0; wvalid = 0;
        while (!bvalid) @(negedge aclk);
        if (bresp != 0) fail($sformatf("write 0x%03x bresp=%0d", addr, bresp));
        @(negedge aclk); bready = 0;
    endtask

    task automatic axi_read(input int addr, output logic [31:0] data);
        @(negedge aclk);
        araddr = addr; arvalid = 1; rready = 1;
        do @(negedge aclk); while (!arready);
        arvalid = 0;
        while (!rvalid) @(negedge aclk);
        data = rdata;
        if (rresp != 0) fail($sformatf("read 0x%03x rresp=%0d", addr, rresp));
        @(negedge aclk); rready = 0;
    endtask

    task automatic expect_reg(input int addr, input logic [31:0] exp, string name);
        logic [31:0] v;
        axi_read(addr, v);
        checks++;
        if (v !== exp) fail($sformatf("%s (0x%03x) = 0x%08x, expected 0x%08x", name, addr, v, exp));
    endtask

    function automatic longint unsigned to_ns(input logic [SEC_W-1:0] s, input logic [NS_W-1:0] n);
        return longint'(s) * 64'd1_000_000_000 + longint'(n);
    endfunction

    // snapshot via registers; returns value in ns and the net-side bracket
    task automatic snapshot(output longint unsigned t_snap, output longint unsigned t_req,
                            output longint unsigned t_done);
        logic [31:0] st, ns, lo, hi;
        t_req = to_ns(time_sec, time_ns);
        axi_write(R_CTRL, 32'h1);
        do axi_read(R_STATUS, st); while (st[0]);
        t_done = to_ns(time_sec, time_ns);
        axi_read(R_SNAP_NS, ns); axi_read(R_SNAP_SEC_LO, lo); axi_read(R_SNAP_SEC_HI, hi);
        t_snap = to_ns({hi[15:0], lo}, ns);
        checks++;
        if (t_snap < t_req || t_snap > t_done)
            fail($sformatf("snapshot %0d outside [%0d, %0d]", t_snap, t_req, t_done));
        if (ns >= 1_000_000_000) fail("snapshot ns field >= 1e9");
    endtask

    // ---- histogram reference ---------------------------------------------------
    int unsigned     ref_bins [NBINS];
    int unsigned     ref_count = 0, ref_min = 32'hFFFF_FFFF, ref_max = 0;
    longint unsigned ref_sum = 0;

    function automatic int ref_bin(input longint unsigned delta);
        int msb = 0, sub;
        longint unsigned d = delta > 64'hFFFF_FFFF ? 64'hFFFF_FFFF : delta;
        if (d < (1 << (SUB_BITS + 1))) return int'(d);
        for (int i = 0; i < 32; i++) if (d[i]) msb = i;
        sub = int'((d >> (msb - SUB_BITS)) & ((1 << SUB_BITS) - 1));
        return (msb << SUB_BITS) + sub - (((SUB_BITS + 1) << SUB_BITS) - (1 << (SUB_BITS + 1)));
    endfunction

    // fire an in strobe, wait `gap` net cycles, fire an out strobe; record
    // the exact timestamps the hardware will capture
    task automatic strobe_pair(input int gap);
        longint unsigned t_in, t_out, d;
        @(negedge net_clk); event_in = 1;  t_in = to_ns(time_sec, time_ns);
        @(negedge net_clk); event_in = 0;
        repeat (gap - 1) @(negedge net_clk);
        event_out = 1; t_out = to_ns(time_sec, time_ns);
        @(negedge net_clk); event_out = 0;
        d = t_out - t_in;
        ref_bins[ref_bin(d)]++; ref_count++; ref_sum += d;
        if (d < ref_min) ref_min = d;
        if (d > ref_max) ref_max = d;
    endtask

    // ---- main ---------------------------------------------------------------
    logic [31:0] v;
    longint unsigned s0, s1, r0, r1, d0, d1;
    real elapsed, expected;

    initial begin
        if ($test$plusargs("WAVES")) begin
            $dumpfile("build/tb_ptp_tsu_top.vcd");
            $dumpvars(0, tb_ptp_tsu_top);
        end
        for (int b = 0; b < NBINS; b++) ref_bins[b] = 0;
        repeat (5) @(negedge net_clk); net_rst_n = 1;
        repeat (5) @(negedge aclk);    aresetn = 1;
        repeat (5) @(negedge aclk);

        // ---------------- T1 -------------------------------------------------
        $display("T1: ID / VERSION");
        expect_reg(R_ID, 32'h5054_5355, "ID");
        expect_reg(R_VERSION, 32'h0001_0000, "VERSION");
        $display("  ok");

        // ---------------- T2 -------------------------------------------------
        $display("T2: control registers through the bridge");
        expect_reg(R_NOM_INCR_LO, 32'h6666_6666, "NOM_INCR_LO reset");
        expect_reg(R_NOM_INCR_HI, 32'h6, "NOM_INCR_HI reset");
        axi_write(R_FREQ_ADJ, 32'h1234_5678);
        expect_reg(R_FREQ_ADJ, 32'h1234_5678, "FREQ_ADJ rw");
        axi_write(R_FREQ_ADJ, 0);
        axi_write(R_SET_SEC_HI, 32'h0000_0001);
        axi_write(R_SET_SEC_LO, 32'h0000_0100);          // sec = 0x1_0000_0100
        axi_write(R_SET_NS, 32'd500_000_000);
        repeat (2) @(negedge net_clk);
        checks++;
        if (time_sec != 48'h1_0000_0100 || time_ns < 500_000_000 || time_ns > 500_001_000)
            fail($sformatf("SET did not load: %0d.%09d", time_sec, time_ns));
        else $display("  ok  SET loaded %0d.%09d", time_sec, time_ns);

        // ---------------- T3 -------------------------------------------------
        $display("T3: coherent snapshot over AXI");
        for (int i = 0; i < 20; i++) snapshot(s0, r0, d0);
        $display("  ok  20 snapshots bracketed by net-side time (last: %0d ns in [%0d, %0d])", s0, r0, d0);

        // ---------------- T4 -------------------------------------------------
        $display("T4: FREQ_ADJ +100 ppm measured through snapshots");
        axi_write(R_FREQ_ADJ, 32'd2748779);                // 100 ppm at 6.4 ns
        repeat (4) @(negedge net_clk);
        snapshot(s0, r0, d0);
        r0 = to_ns(time_sec, time_ns);                     // net-side reference start
        repeat (156_250) @(negedge net_clk);               // 1 ms nominal
        r1 = to_ns(time_sec, time_ns);
        elapsed  = real'(r1 - r0);
        expected = 156_250 * NET_PERIOD * (1.0 + 100.0e-6);
        checks++;
        if (elapsed < expected - 1.0 || elapsed > expected + 1.0)
            fail($sformatf("T4 elapsed %.1f ns, expected %.1f", elapsed, expected));
        else $display("  ok  1 ms at +100 ppm = %.1f ns (nominal 1000000.0)", elapsed);
        axi_write(R_FREQ_ADJ, 0);

        // ---------------- T5 -------------------------------------------------
        $display("T5: phase step via ADJ_NS");
        snapshot(s0, r0, d0);
        axi_write(R_ADJ_SEC, 0);
        axi_write(R_ADJ_NS, 32'd1234);
        snapshot(s1, r1, d1);
        checks++;
        // wall time between the two snapshots is ~tens of AXI cycles; the
        // step must appear as an extra 1234 ns on top of that
        if (s1 - s0 < 1234 || s1 - s0 > 1234 + 5000)
            fail($sformatf("T5 delta between snapshots %0d ns, expected 1234 + small", s1 - s0));
        else $display("  ok  snapshot delta %0d ns includes the 1234 ns step", s1 - s0);

        // ---------------- T6 -------------------------------------------------
        $display("T6: latency histogram over AXI, 2000 strobe pairs");
        for (int i = 0; i < 2000; i++) begin
            int gap;
            case ($urandom_range(0, 3))
                0: gap = $urandom_range(1, 4);          // 6.4 .. 25.6 ns
                1: gap = $urandom_range(5, 40);
                2: gap = $urandom_range(40, 400);
                default: gap = $urandom_range(400, 2000);
            endcase
            strobe_pair(gap);
            repeat ($urandom_range(0, 3)) @(negedge net_clk);
        end
        repeat (50) @(negedge net_clk);
        expect_reg(R_HIST_COUNT, ref_count, "HIST_COUNT");
        expect_reg(R_HIST_MIN, ref_min, "HIST_MIN");
        expect_reg(R_HIST_MAX, ref_max, "HIST_MAX");
        expect_reg(R_HIST_SUM_LO, ref_sum[31:0], "HIST_SUM_LO");
        expect_reg(R_HIST_SUM_HI, ref_sum[63:32], "HIST_SUM_HI");
        expect_reg(R_HIST_NEG, 0, "HIST_NEG");
        expect_reg(R_DROP_IN, 0, "DROP_IN");
        expect_reg(R_DROP_OUT, 0, "DROP_OUT");
        for (int b = 0; b < NBINS; b++) expect_reg(R_HIST_BIN + 4*b, ref_bins[b], $sformatf("BIN[%0d]", b));
        $display("  ok  count=%0d min=%0d max=%0d mean=%.1f ns, all %0d bins match",
                 ref_count, ref_min, ref_max, real'(ref_sum)/ref_count, NBINS);
        for (int b = 0; b < NBINS; b++) if (ref_bins[b] > 0)
            $display("       bin %3d  [%6d ns .. ) : %0d", b, bin_lo(b), ref_bins[b]);

        // ---------------- T7 -------------------------------------------------
        $display("T7: HIST_CLEAR");
        axi_write(R_NET_CTRL, 32'h1);
        do axi_read(R_NET_STATUS, v); while (v[0]);
        expect_reg(R_HIST_COUNT, 0, "HIST_COUNT after clear");
        expect_reg(R_HIST_MIN, 32'hFFFF_FFFF, "HIST_MIN after clear");
        for (int b = 0; b < NBINS; b += 17) expect_reg(R_HIST_BIN + 4*b, 0, $sformatf("BIN[%0d] after clear", b));
        $display("  ok");

        $display("");
        $display("checks: %0d   errors: %0d", checks, errors);
        if (errors == 0) $display("TEST PASSED"); else $display("TEST FAILED");
        $finish;
    end

    function automatic longint unsigned bin_lo(input int k);
        int j, msb, sub;
        if (k < (1 << (SUB_BITS + 1))) return k;                       // linear region
        j   = k - (1 << (SUB_BITS + 1));
        msb = (j >> SUB_BITS) + SUB_BITS + 1;
        sub = j & ((1 << SUB_BITS) - 1);
        return (longint'((1 << SUB_BITS) + sub)) << (msb - SUB_BITS);
    endfunction

endmodule

`default_nettype wire
