// =============================================================================
// tb_servo_golden.sv
//
// Replays the golden vectors exported by matlab/run_servo_model.m against
// ptp_clock_core and diffs the clock state bit-for-bit after every servo
// step.
//
// Vector file  matlab/vectors/servo_vectors.txt  (hex tokens):
//   K
//   F ADJ_VALID ADJ_SEC ADJ_NS N EXP_SEC EXP_NS EXP_FRAC MASTER_END_NS   (x K)
//
// Protocol per step (must match ptp_clock_step.m):
//   negedge: freq_adj <= F, adj_valid <= ADJ_VALID (+ adj_sec/adj_ns)
//   posedge 1 .. N                                    (adj_valid dropped after 1)
//   negedge after posedge N: compare {sec, ns, frac} with EXP_*
//
// Writes build/servo_rtl_trace.txt (k, rtl sec ns frac, offset to master)
// for python/servo_diff.py.
//
// Run:  make sim-servo_golden
// =============================================================================

`timescale 1ns/1ps
`default_nettype none

module tb_servo_golden;

    localparam int MAXK = 4096;

    logic clk = 0, rst_n = 0;
    always #3.2 clk = ~clk;

    logic [39:0]        nom_incr = 40'h6_6666_6666;
    logic signed [31:0] freq_adj = 0;
    logic               set_valid = 0, adj_valid = 0;
    logic [47:0]        set_sec = 0;
    logic [31:0]        set_ns = 0;
    logic signed [31:0] adj_sec = 0, adj_ns = 0;
    logic [47:0]        time_sec;
    logic [31:0]        time_ns, time_frac;
    logic               pps;

    ptp_clock_core dut (.*);

    logic [63:0] vec [0:9*MAXK];
    int K, errors = 0, fd;
    int b, N;                       // module-scope: loop-body declarations with
                                    // initialisers are static in Icarus and
                                    // would be evaluated before $readmemh
    longint unsigned master_end, slave_ns;
    real offset;

    initial begin
        $readmemh("matlab/vectors/servo_vectors.txt", vec);
        K = vec[0];
        if (K <= 0 || K > MAXK) begin
            $display("bad vector file (K=%0d) - run 'make matlab' first", K);
            $display("TEST FAILED"); $finish;
        end
        fd = $fopen("build/servo_rtl_trace.txt", "w");
        $fdisplay(fd, "# k rtl_sec rtl_ns rtl_frac offset_to_master_ns");

        // reset, then SET 1000.000000000 -> incr_eff = nom_incr next cycle
        repeat (3) @(negedge clk);
        rst_n = 1;
        @(negedge clk);
        set_valid = 1; set_sec = 48'd1000; set_ns = 0;
        @(negedge clk);
        set_valid = 0;

        $display("replaying %0d servo steps x %0d cycles", K, vec[5]);
        for (int k = 0; k < K; k++) begin
            b = 1 + 9*k;
            N = vec[b+4];
            // drive this step's commands (we are at a negedge)
            freq_adj  = vec[b];
            adj_valid = vec[b+1][0];
            adj_sec   = vec[b+2];
            adj_ns    = vec[b+3];
            @(negedge clk);                 // posedge 1 consumed
            adj_valid = 0;
            repeat (N - 1) @(negedge clk);  // posedges 2..N
            // compare
            if (time_sec !== vec[b+5][47:0] || time_ns !== vec[b+6][31:0] || time_frac !== vec[b+7][31:0]) begin
                errors++;
                if (errors <= 10)
                    $display("[step %0d] FAIL: rtl %0d.%09d.%08h  golden %0d.%09d.%08h",
                             k, time_sec, time_ns, time_frac, vec[b+5][47:0], vec[b+6][31:0], vec[b+7][31:0]);
            end
            master_end = vec[b+8];
            slave_ns   = longint'(time_sec) * 64'd1_000_000_000 + longint'(time_ns);
            offset     = real'(longint'(slave_ns) - longint'(master_end)) + real'(time_frac) / 4294967296.0;
            $fdisplay(fd, "%0d %0d %0d %0d %.4f", k, time_sec, time_ns, time_frac, offset);
            if (k % 50 == 0 || k == K-1)
                $display("  step %3d: rtl %0d.%09d  offset to master %+9.2f ns  freq_adj %0d", k, time_sec, time_ns, offset, freq_adj);
        end
        $fclose(fd);

        $display("");
        $display("steps: %0d   bit-exact mismatches: %0d", K, errors);
        if (errors == 0) $display("TEST PASSED"); else $display("TEST FAILED");
        $finish;
    end

endmodule

`default_nettype wire
