// =============================================================================
// tb_ptp_clock_core.sv
//
// Self-checking testbench for ptp_clock_core.
//
// Reference model: the DUT keeps time as split {sec, ns, frac} fields with
// carry/borrow normalisation.  The TB keeps time as ONE flat 96-bit
// fixed-point number  total = (sec*1e9 + ns) * 2^32 + frac  and advances it
// with the same stimulus.  Every cycle the DUT fields are compared against
// total / 1e9, total % 1e9 and total[31:0].  Because the two formulations
// share no code, agreement is a real check of the normalisation logic.
//
// Directed tests on top of the cycle-by-cycle compare:
//   T1  reset value and nominal 6.4 ns rate over 1e6 cycles
//   T2  +100 ppm frequency slew -> elapsed time scales exactly
//   T3  -1 ppb slew resolves (sub-ppb LSB)
//   T4  second rollover produces exactly one pps pulse
//   T5  positive ns offset that carries into seconds
//   T6  negative ns offset that borrows from seconds
//   T7  randomised freq_adj / offsets / set_time  (100k cycles)
//
// Run:  make sim-ptp_clock_core          (add WAVES=1 for a VCD)
// =============================================================================

`timescale 1ns/1ps
`default_nettype none

module tb_ptp_clock_core;

    localparam int SEC_W  = 48;
    localparam int NS_W   = 32;
    localparam int FRAC_W = 32;
    localparam int INT_W  = 8;
    localparam int INCR_W = INT_W + FRAC_W;

    localparam real CLK_PERIOD_NS = 6.4;                        // 156.25 MHz
    localparam logic [INCR_W-1:0] NOM_INCR = 40'h6_6666_6666;   // 6.4 * 2^32 (truncated)

    // ---- DUT I/O ---------------------------------------------------------
    logic                     clk = 0;
    logic                     rst_n = 0;
    logic [INCR_W-1:0]        nom_incr = NOM_INCR;
    logic signed [FRAC_W-1:0] freq_adj = 0;
    logic                     set_valid = 0;
    logic [SEC_W-1:0]         set_sec = 0;
    logic [NS_W-1:0]          set_ns = 0;
    logic                     adj_valid = 0;
    logic signed [31:0]       adj_sec = 0;
    logic signed [31:0]       adj_ns = 0;
    logic [SEC_W-1:0]         time_sec;
    logic [NS_W-1:0]          time_ns;
    logic [FRAC_W-1:0]        time_frac;
    logic                     pps;

    ptp_clock_core #(
        .SEC_W(SEC_W), .NS_W(NS_W), .FRAC_W(FRAC_W), .INT_W(INT_W)
    ) dut (.*);

    always #(CLK_PERIOD_NS/2.0) clk = ~clk;

    // ---- bookkeeping -----------------------------------------------------
    int errors = 0;
    int checks = 0;
    int pps_count = 0;

    task automatic fail(string msg);
        errors++;
        $display("[%0t] FAIL: %s", $time, msg);
    endtask

    // ---- flat reference model -------------------------------------------
    // total = (sec*1e9 + ns) << 32 | frac      (96 bits is plenty: 2^48 s)
    logic [95:0]        ref_total;
    logic [INCR_W-1:0]  ref_incr_eff;      // mirrors the DUT's 1-cycle registered incr
    logic signed [95:0] ref_delta;
    logic [95:0]        ref_int;           // total >> 32 = whole ns
    logic [SEC_W-1:0]   exp_sec;
    logic [NS_W-1:0]    exp_ns;
    logic [FRAC_W-1:0]  exp_frac;
    logic               checking = 0;

    // Advance the reference model on the same edge as the DUT.
    // Uses non-blocking style ordering: sample inputs before the edge.
    always @(posedge clk) begin
        if (!rst_n) begin
            ref_total    <= '0;
            ref_incr_eff <= '0;
        end else begin
            ref_incr_eff <= nom_incr + INCR_W'($signed(freq_adj));
            if (set_valid) begin
                ref_total <= ({48'd0, set_sec} * 96'd1_000_000_000 + {64'd0, set_ns}) << 32;
            end else begin
                ref_delta = 96'($signed({{(96-INCR_W){1'b0}}, ref_incr_eff}));
                if (adj_valid)
                    ref_delta = ref_delta
                              + ((96'($signed(adj_sec)) * 96'sd1_000_000_000
                                 + 96'($signed(adj_ns))) <<< 32);
                ref_total <= ref_total + ref_delta;
            end
        end
    end

    // Compare DUT vs reference every cycle (after both have updated).
    always @(negedge clk) begin
        if (checking) begin
            ref_int  = ref_total >> 32;
            exp_sec  = SEC_W'(ref_int / 96'd1_000_000_000);
            exp_ns   = NS_W'(ref_int % 96'd1_000_000_000);
            exp_frac = ref_total[FRAC_W-1:0];
            checks++;
            if (time_sec !== exp_sec || time_ns !== exp_ns || time_frac !== exp_frac) begin
                fail($sformatf("DUT %0d.%09d.%08h != REF %0d.%09d.%08h",
                     time_sec, time_ns, time_frac, exp_sec, exp_ns, exp_frac));
                if (errors > 20) begin
                    $display("Too many errors, aborting.");
                    $finish;
                end
            end
        end
        if (pps) pps_count++;
    end

    // ---- helpers ---------------------------------------------------------
    // time as real ns (for rate checks; ~1e-7 ns resolution is plenty here)
    function automatic real now_ns();
        return real'(time_sec) * 1.0e9 + real'(time_ns)
             + real'(time_frac) / 4294967296.0;
    endfunction

    task automatic do_set(input logic [SEC_W-1:0] s, input logic [NS_W-1:0] n);
        @(negedge clk);
        set_valid = 1; set_sec = s; set_ns = n;
        @(negedge clk);
        set_valid = 0;
    endtask

    task automatic do_adj(input logic signed [31:0] s, input logic signed [31:0] n);
        @(negedge clk);
        adj_valid = 1; adj_sec = s; adj_ns = n;
        @(negedge clk);
        adj_valid = 0;
    endtask

    task automatic run_cycles(input int n);
        repeat (n) @(negedge clk);
    endtask

    task automatic expect_time(input logic [SEC_W-1:0] s, input logic [NS_W-1:0] n, string what);
        if (time_sec !== s || time_ns !== n)
            fail($sformatf("%s: got %0d.%09d expected %0d.%09d", what, time_sec, time_ns, s, n));
        else
            $display("  ok  %s: %0d.%09d", what, time_sec, time_ns);
    endtask

    // ---- test sequence ---------------------------------------------------
    real t0, t1, elapsed, expected, tol;
    int  n;

    initial begin
        if ($test$plusargs("WAVES")) begin
            $dumpfile("build/tb_ptp_clock_core.vcd");
            $dumpvars(0, tb_ptp_clock_core);
        end

        // reset
        rst_n = 0;
        run_cycles(5);
        rst_n = 1;
        run_cycles(2);
        checking = 1;

        // ---------------- T1: reset value, nominal rate ------------------
        $display("T1: reset value + nominal rate");
        do_set(0, 0);
        run_cycles(1);                       // incr_eff registered
        t0 = now_ns();
        n  = 1_000_000;
        run_cycles(n);
        elapsed  = now_ns() - t0;
        expected = n * CLK_PERIOD_NS;
        // truncation of 0.4*2^32 costs 0.4 LSB/cycle = 9.3e-5 ns over 1e6 cycles
        if (elapsed < expected - 0.01 || elapsed > expected + 0.01)
            fail($sformatf("T1 nominal rate: elapsed %.6f ns, expected %.6f", elapsed, expected));
        else
            $display("  ok  %0d cycles -> %.6f ns (expected %.6f)", n, elapsed, expected);

        // ---------------- T2: +100 ppm slew ------------------------------
        $display("T2: +100 ppm frequency slew");
        // 100 ppm of 6.4 ns = 6.4e-4 ns/cycle = 6.4e-4 * 2^32 = 2748779.07 LSB
        freq_adj = 32'sd2748779;
        run_cycles(2);
        t0 = now_ns();
        run_cycles(n);
        elapsed  = now_ns() - t0;
        expected = n * CLK_PERIOD_NS * (1.0 + 100.0e-6);
        if (elapsed < expected - 0.01 || elapsed > expected + 0.01)
            fail($sformatf("T2 +100ppm: elapsed %.6f ns, expected %.6f", elapsed, expected));
        else
            $display("  ok  +100 ppm: %.6f ns over %0d cycles (nominal %.1f)", elapsed, n, n*CLK_PERIOD_NS);

        // ---------------- T3: -1 ppb slew --------------------------------
        $display("T3: -1 ppb frequency slew");
        // 1 ppb of 6.4 ns = 6.4e-9 ns/cycle = 27.49 LSB  -> use -27 LSB (0.982 ppb)
        freq_adj = -32'sd27;
        run_cycles(2);
        t0 = now_ns();
        run_cycles(n);
        elapsed  = now_ns() - t0;
        expected = n * (CLK_PERIOD_NS - 27.0/4294967296.0);
        if (elapsed < expected - 0.001 || elapsed > expected + 0.001)
            fail($sformatf("T3 -1ppb: elapsed %.9f ns, expected %.9f", elapsed, expected));
        else
            $display("  ok  -0.98 ppb: %.9f ns vs nominal %.1f (delta %.4f ns)",
                     elapsed, n*CLK_PERIOD_NS, elapsed - n*CLK_PERIOD_NS);
        freq_adj = 0;

        // ---------------- T4: second rollover + pps ----------------------
        $display("T4: second rollover / pps");
        do_set(48'd99, 32'd999_999_000);     // 1000 ns before the second boundary
        run_cycles(1);
        pps_count = 0;
        run_cycles(200);                     // 201 periods total = 1286.4 ns
        // 999_999_000 + 1286.4 = 1_000_000_286.4 -> 100 s + 286 ns
        expect_time(48'd100, 32'd286, "rollover");
        if (pps_count != 1) fail($sformatf("T4 pps pulses = %0d, expected 1", pps_count));
        else $display("  ok  pps pulsed exactly once");

        // ---------------- T5: positive offset with carry -----------------
        $display("T5: +offset carrying into seconds");
        do_set(48'd10, 32'd999_999_990);
        do_adj(0, 32'sd500);                 // adj cycle also adds one period
        // after do_set returns, one cycle of increment happened before adj:
        //   10.999999990 +6.4 (set->adj gap) +6.4 +500  = 11.000000502.8
        expect_time(48'd11, 32'd502, "carry");

        // ---------------- T6: negative offset with borrow ----------------
        $display("T6: -offset borrowing from seconds");
        do_set(48'd10, 32'd100);
        do_adj(0, -32'sd500);
        //   10.000000100 +6.4 +6.4 -500 = 9.999999612.8
        expect_time(48'd9, 32'd999_999_612, "borrow");

        // seconds offset too
        do_set(48'd10, 32'd100);
        do_adj(-32'sd3, 32'sd0);
        expect_time(48'd7, 32'd112, "sec offset");

        // ---------------- T7: random ------------------------------------
        $display("T7: randomised stimulus (100k cycles, checked every cycle vs flat model)");
        do_set(48'd1000, 32'd0);
        for (int i = 0; i < 100_000; i++) begin
            @(negedge clk);
            set_valid = 0;
            adj_valid = 0;
            // occasionally change frequency (+/- 500 ppm max)
            if ($urandom_range(0, 999) == 0)
                freq_adj = $urandom_range(0, 2*13_743_895) - 13_743_895;
            // occasionally apply a phase step
            if ($urandom_range(0, 99) == 0) begin
                adj_valid = 1;
                adj_sec   = $urandom_range(0, 6) - 3;
                adj_ns    = $urandom_range(0, 1_999_999_998) - 999_999_999;
            end
            // rarely reload absolute time
            if ($urandom_range(0, 9999) == 0) begin
                set_valid = 1;
                set_sec   = $urandom_range(500, 5000);
                set_ns    = $urandom_range(0, 999_999_999);
            end
        end
        @(negedge clk);
        set_valid = 0; adj_valid = 0;
        run_cycles(10);

        // ---------------- summary ---------------------------------------
        checking = 0;
        $display("");
        $display("cycle-compares: %0d   errors: %0d", checks, errors);
        if (errors == 0) $display("TEST PASSED");
        else             $display("TEST FAILED");
        $finish;
    end

endmodule

`default_nettype wire
