// =============================================================================
// tb_ptp_ts_capture.sv
//
// Self-checking testbench for ptp_ts_capture (with a live ptp_clock_core
// underneath so the captured values are real PTP time).
//
//   T1  random synchronous strobes, random pop back-pressure:
//       every popped timestamp must equal the time the clock held in the
//       cycle the strobe's rising edge was sampled  (exact match, in order)
//   T2  FIFO overflow: 20 back-to-back strobes with pops stalled ->
//       16 captured, 4 dropped, drop_count == 4, then clears
//   T3  asynchronous strobe from an unrelated 73 MHz clock through a
//       2-flop synchroniser: each timestamp must land within
//       [t_event, t_event + 4 periods]  (bounded, monotonic latency)
//
// Run:  make sim-ptp_ts_capture
// =============================================================================

`timescale 1ns/1ps
`default_nettype none

module tb_ptp_ts_capture;

    localparam int  SEC_W  = 48;
    localparam int  NS_W   = 32;
    localparam int  TS_W   = SEC_W + NS_W;
    localparam real CLK_PERIOD_NS = 6.4;
    localparam real ASYNC_PERIOD_NS = 13.7;     // ~73 MHz, unrelated

    // ---- clocks / reset --------------------------------------------------
    logic clk = 0;
    logic aclk = 0;
    logic rst_n = 0;
    always #(CLK_PERIOD_NS/2.0)   clk  = ~clk;
    always #(ASYNC_PERIOD_NS/2.0) aclk = ~aclk;

    // ---- clock core -------------------------------------------------------
    logic [SEC_W-1:0] time_sec;
    logic [NS_W-1:0]  time_ns;
    logic [31:0]      time_frac;
    logic             pps;

    ptp_clock_core u_clk (
        .clk(clk), .rst_n(rst_n),
        .nom_incr(40'h6_6666_6666), .freq_adj(32'sd0),
        .set_valid(1'b0), .set_sec(48'd0), .set_ns(32'd0),
        .adj_valid(1'b0), .adj_sec(32'sd0), .adj_ns(32'sd0),
        .time_sec(time_sec), .time_ns(time_ns), .time_frac(time_frac), .pps(pps)
    );

    // ---- DUT A: synchronous strobe ---------------------------------------
    logic             ev_sync = 0;
    logic             ts_valid, ts_ready = 0;
    logic [SEC_W-1:0] ts_sec;
    logic [NS_W-1:0]  ts_ns;
    logic [4:0]       fifo_count;
    logic [15:0]      drop_count;
    logic             drop_clear = 0;

    ptp_ts_capture #(
        .FIFO_DEPTH_LOG2(4), .EVENT_SYNC_STAGES(0)
    ) dut_sync (
        .clk(clk), .rst_n(rst_n),
        .time_sec(time_sec), .time_ns(time_ns),
        .event_in(ev_sync),
        .ts_valid(ts_valid), .ts_sec(ts_sec), .ts_ns(ts_ns), .ts_ready(ts_ready),
        .fifo_count(fifo_count), .drop_count(drop_count), .drop_clear(drop_clear)
    );

    // ---- DUT B: asynchronous strobe --------------------------------------
    logic             ev_async = 0;
    logic             a_valid, a_ready = 1;
    logic [SEC_W-1:0] a_sec;
    logic [NS_W-1:0]  a_ns;
    logic [4:0]       a_count;
    logic [15:0]      a_drops;

    ptp_ts_capture #(
        .FIFO_DEPTH_LOG2(4), .EVENT_SYNC_STAGES(2)
    ) dut_async (
        .clk(clk), .rst_n(rst_n),
        .time_sec(time_sec), .time_ns(time_ns),
        .event_in(ev_async),
        .ts_valid(a_valid), .ts_sec(a_sec), .ts_ns(a_ns), .ts_ready(a_ready),
        .fifo_count(a_count), .drop_count(a_drops), .drop_clear(1'b0)
    );

    // ---- bookkeeping -------------------------------------------------------
    int errors = 0, checks = 0;
    task automatic fail(string msg);
        errors++;
        $display("[%0t] FAIL: %s", $time, msg);
    endtask

    function automatic longint unsigned to_ns(input logic [SEC_W-1:0] s, input logic [NS_W-1:0] n);
        return longint'(s) * 64'd1_000_000_000 + longint'(n);
    endfunction

    // ---- expected-timestamp queue for DUT A --------------------------------
    // The TB drives ev_sync at negedge; the DUT samples it (and time) at the
    // following posedge.  Record time at the negedge where a rising edge is
    // produced -- that is exactly the value the clock holds at that posedge.
    logic [TS_W-1:0] expq[$];
    logic            ev_sync_prev = 0;
    bit              stall_pops = 0;
    bit              a_checking = 0;

    // pop-side compare (DUT A).  Sampled at posedge with blocking reads so
    // the values seen are the pre-edge ones the DUT is about to act on --
    // no race with the stimulus, which drives at negedge.
    logic [TS_W-1:0] exp_head;
    always @(posedge clk) begin
        if (ts_valid && ts_ready) begin
            checks++;
            if (expq.size() == 0)
                fail("popped a timestamp but none expected");
            else begin
                exp_head = expq.pop_front();
                if ({ts_sec, ts_ns} !== exp_head)
                    fail($sformatf("ts mismatch: got %0d.%09d expected %0d.%09d",
                         ts_sec, ts_ns, exp_head[TS_W-1:NS_W], exp_head[NS_W-1:0]));
            end
        end
    end

    // ---- async strobe generator + check (DUT B) ----------------------------
    longint unsigned a_evt_q[$];     // time (ns) at which each async event rose
    longint unsigned a_last = 0;

    always @(posedge clk) begin
        if (a_checking && a_valid && a_ready) begin
            longint unsigned got, t0;
            checks++;
            got = to_ns(a_sec, a_ns);
            if (a_evt_q.size() == 0)
                fail("async: popped a timestamp but none expected");
            else begin
                t0 = a_evt_q.pop_front();
                // synchroniser (2) + edge detect (1) + sampling uncertainty (1)
                if (got < t0 || got > t0 + 4 * 7)
                    fail($sformatf("async latency out of range: event at %0d ns, ts %0d ns (delta %0d)",
                         t0, got, longint'(got) - longint'(t0)));
                if (got < a_last)
                    fail("async timestamps not monotonic");
                a_last = got;
            end
        end
    end

    // ---- stimulus ----------------------------------------------------------
    int n_events;
    int i;

    initial begin
        if ($test$plusargs("WAVES")) begin
            $dumpfile("build/tb_ptp_ts_capture.vcd");
            $dumpvars(0, tb_ptp_ts_capture);
        end

        repeat (4) @(negedge clk);
        rst_n = 1;
        repeat (4) @(negedge clk);

        // ---------------- T1: random strobes, random back-pressure ---------
        $display("T1: random synchronous strobes with random pop back-pressure");
        n_events = 0;
        for (i = 0; i < 20_000; i++) begin
            @(negedge clk);
            // strobe: ~10% duty of rising edges, random pulse widths
            if ($urandom_range(0, 9) == 0 && !ev_sync) begin
                ev_sync = 1;
                // Only count it if the FIFO has room to take it (a pop in
                // the same cycle does not free a slot for this write).
                if (fifo_count < 16) begin
                    expq.push_back({time_sec, time_ns});
                    n_events++;
                end
            end else if (ev_sync && $urandom_range(0, 2) == 0) begin
                ev_sync = 0;
            end
            // consumer: 70% ready
            ts_ready = ($urandom_range(0, 9) < 7);
        end
        ev_sync = 0;
        ts_ready = 1;
        repeat (40) @(negedge clk);
        if (expq.size() != 0) fail($sformatf("T1: %0d timestamps never popped", expq.size()));
        if (drop_count != 0)  fail($sformatf("T1: unexpected drops %0d", drop_count));
        $display("  ok  %0d events captured and matched in order, %0d drops", n_events, drop_count);

        // ---------------- T2: overflow --------------------------------------
        $display("T2: FIFO overflow / drop counter");
        ts_ready = 0;
        @(negedge clk);
        for (i = 0; i < 20; i++) begin
            @(negedge clk); ev_sync = 1;
            if (i < 16) expq.push_back({time_sec, time_ns});
            @(negedge clk); ev_sync = 0;
        end
        repeat (3) @(negedge clk);
        if (fifo_count != 16) fail($sformatf("T2: fifo_count %0d, expected 16", fifo_count));
        if (drop_count != 4)  fail($sformatf("T2: drop_count %0d, expected 4", drop_count));
        else $display("  ok  16 captured, drop_count = %0d", drop_count);
        // drain and verify the surviving 16 are the FIRST 16 (not the last)
        ts_ready = 1;
        repeat (20) @(negedge clk);
        if (expq.size() != 0) fail("T2: surviving timestamps did not match first 16");
        else $display("  ok  surviving entries are the first 16 events (drop-on-full, not drop-oldest)");
        drop_clear = 1; @(negedge clk); drop_clear = 0; @(negedge clk);
        if (drop_count != 0) fail("T2: drop_clear did not clear");

        // ---------------- T3: async strobe ---------------------------------
        $display("T3: asynchronous strobe (73 MHz) through 2-flop synchroniser");
        a_checking = 1;
        for (i = 0; i < 3000; i++) begin
            @(negedge aclk);
            if ($urandom_range(0, 4) == 0 && !ev_async) begin
                ev_async = 1;
                a_evt_q.push_back(to_ns(time_sec, time_ns));
                repeat ($urandom_range(2, 5)) @(negedge aclk);   // hold >= 2 net cycles
                ev_async = 0;
            end
        end
        repeat (40) @(negedge clk);
        if (a_evt_q.size() != 0) fail($sformatf("T3: %0d async events not captured", a_evt_q.size()));
        else $display("  ok  all async events captured within 4 periods, monotonic");

        // ---------------- summary -------------------------------------------
        $display("");
        $display("timestamp checks: %0d   errors: %0d", checks, errors);
        if (errors == 0) $display("TEST PASSED");
        else             $display("TEST FAILED");
        $finish;
    end

endmodule

`default_nettype wire
