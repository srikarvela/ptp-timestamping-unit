// =============================================================================
// naive_bus_sync.sv   (TESTBENCH ONLY -- the wrong answer, for comparison)
//
// "Just double-flop the bus."  Each bit gets its own 2-flop synchroniser.
// In zero-delay RTL simulation every bit flips at the same instant and this
// looks fine, which is exactly why the bug survives to silicon.  To expose
// it, each bit is given a random wire delay in [0, MAX_SKEW_NS) before its
// synchroniser -- the routing skew that real fabric always has.  A dst edge
// that lands inside the skew window samples a mix of old and new bits.
// =============================================================================

`timescale 1ns/1ps
`default_nettype none

module naive_bus_sync #(
    parameter int  W = 80,
    parameter real MAX_SKEW_NS = 3.0
)(
    input  wire          dst_clk,
    input  wire [W-1:0]  bus,
    output logic [W-1:0] bus_sync
);

    logic [W-1:0] skewed;
    logic [W-1:0] s1;
    real          d [W];

    initial for (int i = 0; i < W; i++) d[i] = $urandom_range(0, 1000) / 1000.0 * MAX_SKEW_NS;

    genvar g;
    generate
        for (g = 0; g < W; g++) begin : g_bit
            always @(bus[g]) skewed[g] <= #(d[g]) bus[g];     // transport delay
        end
    endgenerate

    always @(posedge dst_clk) begin
        s1       <= skewed;
        bus_sync <= s1;
    end

endmodule

`default_nettype wire
