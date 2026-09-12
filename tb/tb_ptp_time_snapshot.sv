// =============================================================================
// tb_ptp_time_snapshot.sv
//
// Two-clock CDC testbench for ptp_time_snapshot.
//
// src clock: 156.25 MHz (6.4 ns) driving a live ptp_clock_core.
// dst clock: swept through three regimes, each started at a random phase:
//     T1  slower   (10.0 ns, 100 MHz)
//     T2  faster   ( 4.0 ns, 250 MHz)
//     T3  near-equal (6.41 ns) -- the edges sweep slowly through every
//         relative phase, which is the regime that exposes setup/hold
//         races in a wrong design
//
// For every snapshot the TB checks:
//   coherence  the 80-bit value is one the src clock actually held
//              (exact match against a history ring of recent src values)
//   bracket    t_req  <=  snapshot  <=  t_done   (measured in src time)
//   latency    done arrives within a bounded number of cycles
//   monotonic  successive snapshots never go backwards
//
// The same src bus also feeds naive_bus_sync (plain double-flop with random
// per-bit wire skew).  Its output is sampled at every dst edge and checked
// against the same history ring; the number of torn reads is reported.
//
// Run:  make sim-ptp_time_snapshot
// =============================================================================

`timescale 1ns/1ps
`default_nettype none

module tb_ptp_time_snapshot;

    localparam int  SEC_W = 48, NS_W = 32, TS_W = 80;
    localparam real SRC_PERIOD = 6.4;
    localparam int  HIST = 128;

    // ---- clocks ------------------------------------------------------------
    logic src_clk = 0;
    logic dst_clk = 0;
    real  dst_period = 10.0;
    bit   dst_run = 0;
    always #(SRC_PERIOD/2.0) src_clk = ~src_clk;
    always begin
        if (dst_run) begin #(dst_period/2.0) dst_clk = ~dst_clk; end
        else         begin dst_clk = 0; #1; end
    end

    logic src_rst_n = 0, dst_rst_n = 0;

    // ---- live clock core in src domain ------------------------------------
    logic [SEC_W-1:0] time_sec;
    logic [NS_W-1:0]  time_ns;
    logic [31:0]      time_frac;
    logic             pps;

    ptp_clock_core u_clk (
        .clk(src_clk), .rst_n(src_rst_n),
        .nom_incr(40'h6_6666_6666), .freq_adj(32'sd0),
        .set_valid(1'b0), .set_sec(48'd0), .set_ns(32'd0),
        .adj_valid(1'b0), .adj_sec(32'sd0), .adj_ns(32'sd0),
        .time_sec(time_sec), .time_ns(time_ns), .time_frac(time_frac), .pps(pps)
    );

    // ---- DUT ---------------------------------------------------------------
    logic             dst_req = 0, dst_busy, dst_done;
    logic [SEC_W-1:0] dst_sec;
    logic [NS_W-1:0]  dst_ns;

    ptp_time_snapshot dut (
        .src_clk(src_clk), .src_rst_n(src_rst_n),
        .src_time_sec(time_sec), .src_time_ns(time_ns),
        .dst_clk(dst_clk), .dst_rst_n(dst_rst_n),
        .dst_req(dst_req), .dst_busy(dst_busy), .dst_done(dst_done),
        .dst_sec(dst_sec), .dst_ns(dst_ns)
    );

    // ---- the wrong answer, for comparison ---------------------------------
    logic [TS_W-1:0] naive_out;
    naive_bus_sync #(.W(TS_W), .MAX_SKEW_NS(3.0)) u_naive (
        .dst_clk(dst_clk), .bus({time_sec, time_ns}), .bus_sync(naive_out)
    );

    // ---- src-domain history ring (values the clock actually held) ---------
    logic [TS_W-1:0] hist [HIST];
    int              hist_wr = 0;

    always @(posedge src_clk) begin
        hist[hist_wr] <= {time_sec, time_ns};       // pre-edge value = value held this cycle
        hist_wr       <= (hist_wr + 1) % HIST;
    end

    function automatic bit in_history(input logic [TS_W-1:0] v);
        for (int i = 0; i < HIST; i++) if (hist[i] === v) return 1;
        return 0;
    endfunction

    function automatic longint unsigned to_ns(input logic [TS_W-1:0] v);
        return longint'(v[TS_W-1:NS_W]) * 64'd1_000_000_000 + longint'(v[NS_W-1:0]);
    endfunction

    // ---- bookkeeping -------------------------------------------------------
    int errors = 0;
    int snaps = 0;
    int naive_samples = 0, naive_torn = 0;
    longint unsigned last_snap = 0;

    task automatic fail(string msg);
        errors++;
        $display("[%0t] FAIL: %s", $time, msg);
    endtask

    // naive sampler: every dst edge, is the double-flopped bus a real value?
    bit naive_check = 0;
    always @(posedge dst_clk) begin
        if (naive_check) begin
            naive_samples++;
            if (!in_history(naive_out)) naive_torn++;
        end
    end

    // ---- one snapshot transaction -----------------------------------------
    task automatic take_snapshot();
        longint unsigned t_req, t_done, t_snap;
        int cycles;
        @(negedge dst_clk);
        t_req = to_ns({time_sec, time_ns});
        dst_req = 1;
        @(negedge dst_clk);
        dst_req = 0;
        cycles = 0;
        while (!dst_done && cycles <= 40) begin
            @(negedge dst_clk);
            cycles++;
        end
        if (!dst_done) fail("snapshot timed out");
        t_done = to_ns({time_sec, time_ns});
        t_snap = to_ns({dst_sec, dst_ns});
        snaps++;
        if (!in_history({dst_sec, dst_ns}))
            fail($sformatf("torn read: %0d.%09d never existed", dst_sec, dst_ns));
        if (t_snap < t_req || t_snap > t_done)
            fail($sformatf("bracket: req %0d <= snap %0d <= done %0d violated", t_req, t_snap, t_done));
        if (t_snap < last_snap)
            fail("snapshots not monotonic");
        if (dst_busy)
            fail("busy still set after done");
        last_snap = t_snap;
    endtask

    // ---- regime runner ----------------------------------------------------
    task automatic run_regime(string name, real period, int n);
        int e0 = errors;
        int torn0 = naive_torn, samp0 = naive_samples;
        real phase;
        // stop dst clock, change period, restart at random phase
        naive_check = 0;
        dst_rst_n = 0;
        dst_run = 0;
        #(3 * SRC_PERIOD);
        dst_period = period;
        phase = $urandom_range(0, 1000) / 1000.0 * SRC_PERIOD;
        #(phase);
        dst_run = 1;
        repeat (3) @(negedge dst_clk);
        dst_rst_n = 1;
        repeat (3) @(negedge dst_clk);
        last_snap = 0;
        naive_check = 1;
        for (int i = 0; i < n; i++) begin
            take_snapshot();
            // random idle between requests so the ack/req toggles land at
            // varying phases relative to src
            repeat ($urandom_range(0, 7)) @(negedge dst_clk);
        end
        naive_check = 0;
        $display("  %-24s dst %.2f ns, phase +%.2f ns: %0d snapshots, %0d errors | naive double-flop: %0d / %0d samples torn (%.1f%%)",
                 name, period, phase, n, errors - e0,
                 naive_torn - torn0, naive_samples - samp0,
                 100.0 * (naive_torn - torn0) / (naive_samples - samp0));
    endtask

    // ---- main ---------------------------------------------------------------
    initial begin
        if ($test$plusargs("WAVES")) begin
            $dumpfile("build/tb_ptp_time_snapshot.vcd");
            $dumpvars(0, tb_ptp_time_snapshot);
        end
        for (int i = 0; i < HIST; i++) hist[i] = '0;

        repeat (4) @(negedge src_clk);
        src_rst_n = 1;
        repeat (4) @(negedge src_clk);

        $display("CDC snapshot: src 6.40 ns (156.25 MHz), history ring %0d entries", HIST);
        run_regime("T1 dst slower",     10.00, 2000);
        run_regime("T2 dst faster",      4.00, 2000);
        run_regime("T3 dst near-equal",  6.41, 4000);
        run_regime("T4 dst near-equal", 6.39, 4000);

        $display("");
        $display("snapshots: %0d   errors: %0d   naive torn reads: %0d / %0d",
                 snaps, errors, naive_torn, naive_samples);
        if (naive_torn == 0)
            $display("NOTE: naive model produced no torn reads (skew model ineffective?)");
        if (errors == 0 && naive_torn > 0) $display("TEST PASSED");
        else                               $display("TEST FAILED");
        $finish;
    end

endmodule

`default_nettype wire
