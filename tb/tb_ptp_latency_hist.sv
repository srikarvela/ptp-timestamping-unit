// =============================================================================
// tb_ptp_latency_hist.sv
//
// Unit test for ptp_latency_hist against a behavioural reference histogram.
//
//   T1  directed bin edges: deltas 0,1,2,3,4,5,6,7,8,10,12,14,16,... land
//       in the bins the formula predicts
//   T2  back-to-back samples into the SAME bin every cycle (forwarding
//       hazard) -> exact count
//   T3  10k random pairs incl. second-boundary crossings, negative deltas
//       and saturating deltas, with random FIFO back-pressure and host bin
//       reads interleaved -> every bin, count/sum/min/max/neg exact
//   T4  clear -> all bins and stats zero, busy for NBINS cycles
//
// Run:  make sim-ptp_latency_hist
// =============================================================================

`timescale 1ns/1ps
`default_nettype none

module tb_ptp_latency_hist;

    localparam int SEC_W = 48, NS_W = 32, SUB_BITS = 2;
    localparam int BIN_W = 5 + SUB_BITS, NBINS = 1 << BIN_W;

    logic clk = 0, rst_n = 0;
    always #3.2 clk = ~clk;

    logic             in_valid = 0, out_valid = 0, in_ready, out_ready;
    logic [SEC_W-1:0] in_sec = 0, out_sec = 0;
    logic [NS_W-1:0]  in_ns = 0, out_ns = 0;
    logic             clear = 0, busy;
    logic             rd_req = 0, rd_ack;
    logic [BIN_W-1:0] rd_addr = 0;
    logic [31:0]      rd_data;
    logic [31:0]      stat_count, stat_min, stat_max, stat_neg;
    logic [63:0]      stat_sum;

    ptp_latency_hist #(.SUB_BITS(SUB_BITS)) dut (.*);

    // ---- reference model ----------------------------------------------------
    int unsigned     ref_bins [NBINS];
    int unsigned     ref_count = 0, ref_neg = 0, ref_min = 32'hFFFF_FFFF, ref_max = 0;
    longint unsigned ref_sum = 0;
    int errors = 0, checks = 0;

    task automatic fail(string msg);
        errors++;
        $display("[%0t] FAIL: %s", $time, msg);
    endtask

    function automatic int ref_bin(input longint unsigned delta);
        int msb = 0, sub;
        longint unsigned d = delta > 64'hFFFF_FFFF ? 64'hFFFF_FFFF : delta;
        if (d < (1 << (SUB_BITS + 1))) return int'(d);
        for (int i = 0; i < 32; i++) if (d[i]) msb = i;
        sub = int'((d >> (msb - SUB_BITS)) & ((1 << SUB_BITS) - 1));
        return (msb << SUB_BITS) + sub - (((SUB_BITS + 1) << SUB_BITS) - (1 << (SUB_BITS + 1)));
    endfunction

    // Push one pair through the reference model.
    task automatic ref_sample(input longint t_in, input longint t_out);
        longint d = t_out - t_in;
        if (d < 0) ref_neg++;
        else begin
            if (d > 64'hFFFF_FFFF) d = 64'hFFFF_FFFF;
            ref_bins[ref_bin(d)]++;
            ref_count++;
            ref_sum += d;
            if (d < ref_min) ref_min = d;
            if (d > ref_max) ref_max = d;
        end
    endtask

    // ---- stimulus queues (drive FIFO-like sources) --------------------------
    longint in_q[$], out_q[$];

    // Pops are decided at posedge from pre-edge values (exactly what the
    // DUT sees); new values are driven at negedge.  Keeps the drivers
    // race-free against host_read, which also drives at negedge.
    always @(posedge clk) begin
        if (in_valid && in_ready)   void'(in_q.pop_front());
        if (out_valid && out_ready) void'(out_q.pop_front());
    end

    // in-stream driver
    always @(negedge clk) begin
        if (in_q.size() > 0 && ($urandom_range(0,3) != 0)) begin
            in_valid = 1;
            in_sec   = in_q[0] / 1_000_000_000;
            in_ns    = in_q[0] % 1_000_000_000;
        end else in_valid = 0;
    end
    // out-stream driver
    always @(negedge clk) begin
        if (out_q.size() > 0 && ($urandom_range(0,3) != 0)) begin
            out_valid = 1;
            out_sec   = out_q[0] / 1_000_000_000;
            out_ns    = out_q[0] % 1_000_000_000;
        end else out_valid = 0;
    end

    task automatic push_pair(input longint t_in, input longint t_out);
        in_q.push_back(t_in);
        out_q.push_back(t_out);
        ref_sample(t_in, t_out);
    endtask

    task automatic wait_drain();
        while (in_q.size() > 0 || out_q.size() > 0) @(negedge clk);
        repeat (8) @(negedge clk);
    endtask

    task automatic host_read(input int addr, output int unsigned val);
        @(negedge clk);
        rd_req = 1; rd_addr = addr;
        @(negedge clk);
        rd_req = 0;
        while (!rd_ack) @(negedge clk);
        val = rd_data;
    endtask

    task automatic check_all(string what);
        int unsigned v;
        for (int b = 0; b < NBINS; b++) begin
            host_read(b, v);
            checks++;
            if (v !== ref_bins[b]) fail($sformatf("%s bin %0d: dut %0d ref %0d", what, b, v, ref_bins[b]));
        end
        checks += 5;
        if (stat_count !== ref_count) fail($sformatf("%s count: dut %0d ref %0d", what, stat_count, ref_count));
        if (stat_sum   !== ref_sum)   fail($sformatf("%s sum: dut %0d ref %0d", what, stat_sum, ref_sum));
        if (stat_min   !== ref_min)   fail($sformatf("%s min: dut %0d ref %0d", what, stat_min, ref_min));
        if (stat_max   !== ref_max)   fail($sformatf("%s max: dut %0d ref %0d", what, stat_max, ref_max));
        if (stat_neg   !== ref_neg)   fail($sformatf("%s neg: dut %0d ref %0d", what, stat_neg, ref_neg));
    endtask

    // ---- main --------------------------------------------------------------
    int unsigned v;
    longint base;
    int e0;

    initial begin
        if ($test$plusargs("WAVES")) begin
            $dumpfile("build/tb_ptp_latency_hist.vcd");
            $dumpvars(0, tb_ptp_latency_hist);
        end
        for (int b = 0; b < NBINS; b++) ref_bins[b] = 0;
        repeat (4) @(negedge clk);
        rst_n = 1;
        while (busy) @(negedge clk);             // reset-time RAM sweep
        repeat (4) @(negedge clk);

        // ---------------- T1: bin edges ----------------------------------
        $display("T1: directed bin edges");
        base = 64'd5_000_000_000;
        begin
            longint dl[30];
            dl[0] = 64'd0;
            dl[1] = 64'd1;
            dl[2] = 64'd2;
            dl[3] = 64'd3;
            dl[4] = 64'd4;
            dl[5] = 64'd5;
            dl[6] = 64'd6;
            dl[7] = 64'd7;
            dl[8] = 64'd8;
            dl[9] = 64'd10;
            dl[10] = 64'd12;
            dl[11] = 64'd14;
            dl[12] = 64'd16;
            dl[13] = 64'd20;
            dl[14] = 64'd24;
            dl[15] = 64'd28;
            dl[16] = 64'd32;
            dl[17] = 64'd40;
            dl[18] = 64'd48;
            dl[19] = 64'd56;
            dl[20] = 64'd64;
            dl[21] = 64'd100;
            dl[22] = 64'd1000;
            dl[23] = 64'd6400;
            dl[24] = 64'd65535;
            dl[25] = 64'd65536;
            dl[26] = 64'd1000000;
            dl[27] = 64'd999999999;
            dl[28] = 64'd1000000000;
            dl[29] = 64'd3999999999;
            for (int i = 0; i < 30; i++) push_pair(base + i*7, base + i*7 + dl[i]);
        end
        wait_drain();
        e0 = errors;
        check_all("T1");
        $display("  %s  30 edge deltas binned as predicted", errors == e0 ? "ok " : "BAD");

        // ---------------- T2: same bin back-to-back -----------------------
        $display("T2: 500 back-to-back samples into one bin (forwarding)");
        for (int i = 0; i < 500; i++) push_pair(base + i, base + i + 100);   // all delta 100 -> same bin
        wait_drain();
        e0 = errors;
        check_all("T2");
        $display("  %s  bin count exact under RMW hazard", errors == e0 ? "ok " : "BAD");

        // ---------------- T3: random --------------------------------------
        $display("T3: 10k random pairs with sec crossings, negatives, saturation, host reads interleaved");
        begin
            longint t_in, d;
            int kind;
            for (int i = 0; i < 10_000; i++) begin
                // t_in random near a second boundary half the time
                t_in = ($urandom_range(0,1)) ? 64'd7_000_000_000 - $urandom_range(0, 5000)
                                             : 64'd7_000_000_000 + $urandom_range(0, 900_000_000);
                kind = $urandom_range(0, 99);
                if      (kind < 60) d = $urandom_range(0, 20_000);           // typical ns latencies
                else if (kind < 85) d = $urandom_range(0, 32'hFFFF_FFFF);    // whole range
                else if (kind < 93) d = -$urandom_range(1, 5000);            // negative
                else if (kind < 97) d = 64'd2_500_000_000 + $urandom_range(0, 100);  // sec_diff 2 -> saturate
                else                d = 64'd6_000_000_000;                    // sec_diff 6 -> saturate
                push_pair(t_in, t_in + d);
                // occasionally a host read while traffic flows
                if ($urandom_range(0, 49) == 0)
                    host_read($urandom_range(0, NBINS-1), v);   // moving target; exercises the port
            end
        end
        wait_drain();
        e0 = errors;
        check_all("T3");
        $display("  %s  all %0d bins + count/sum/min/max/neg exact (count=%0d neg=%0d)",
                 errors == e0 ? "ok " : "BAD", NBINS, stat_count, stat_neg);

        // ---------------- T4: clear ----------------------------------------
        $display("T4: clear");
        @(negedge clk); clear = 1; @(negedge clk); clear = 0;
        begin
            int n;
            n = 0;
            while (busy) begin @(negedge clk); n++; end
            if (n < NBINS - 2 || n > NBINS + 2) fail($sformatf("T4 busy for %0d cycles, expected ~%0d", n, NBINS));
        end
        for (int b = 0; b < NBINS; b++) ref_bins[b] = 0;
        ref_count = 0; ref_sum = 0; ref_neg = 0; ref_min = 32'hFFFF_FFFF; ref_max = 0;
        repeat (4) @(negedge clk);
        e0 = errors;
        check_all("T4");
        $display("  %s  bins and stats cleared", errors == e0 ? "ok " : "BAD");
        // and it still counts afterwards
        push_pair(base, base + 640);
        wait_drain();
        check_all("T4b");

        $display("");
        $display("checks: %0d   errors: %0d", checks, errors);
        if (errors == 0) $display("TEST PASSED"); else $display("TEST FAILED");
        $finish;
    end

endmodule

`default_nettype wire
