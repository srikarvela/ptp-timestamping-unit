// =============================================================================
// ptp_time_snapshot.sv
//
// Coherent capture of the PTP time across an asynchronous clock boundary.
//
//   src domain (network clock)         dst domain (core / AXI clock)
//   ──────────────────────────         ────────────────────────────
//   time_sec/time_ns change every      software wants to read a single,
//   cycle, all 80 bits at once         coherent {sec, ns} sample
//
// Why the obvious answer is wrong
// -------------------------------
//   You cannot double-flop an 80-bit bus.  Each bit has its own routing
//   delay, so at a destination edge that lands mid-transition some bits
//   have flipped and some have not, and the sampled word is one that never
//   existed (a "torn" read).  Gray code fixes this for a counter that
//   increments by exactly 1 (only one bit changes per step), but a PTP
//   clock advances by an arbitrary Q8.32 period every cycle and
//   renormalises at 1e9, so many bits change per step.  Gray is off the
//   table and an asynchronous FIFO is overkill for a value that is only
//   read on demand.
//
// What this does instead: request / latch / handshake / read
// -----------------------------------------------------------
//   1. dst pulses `dst_req`.  A toggle flag req_tgl flips (2-phase
//      handshake: one toggle per request, no return-to-zero cycle).
//   2. req_tgl crosses into src through a 2-flop ASYNC_REG synchroniser.
//      src detects the toggle (req_s != req_seen) and on that ONE src
//      cycle latches {time_sec, time_ns} into `snap` -- a src-domain
//      register that is otherwise frozen.
//   3. src toggles ack_tgl (a src-domain flop) in the same cycle.
//   4. ack_tgl crosses back into dst through another 2-flop synchroniser.
//      When dst sees ack_s == req_tgl, `snap` has been stable for at least
//      two dst cycles (it was written one src cycle before ack toggled,
//      and the ack took >= 2 dst cycles to arrive), so dst copies `snap`
//      into `dst_sec/dst_ns` and pulses `dst_done`.
//
//   Only the single-bit toggles are synchronised.  The 80-bit bus crosses
//   as a multi-cycle path qualified by the synchronised ack -- the
//   standard "MCP formulation" (Cummings, SNUG 2008).  It is safe because
//   the source register does not change while the destination might be
//   sampling it.
//
//   Constraints (synth/ptp_cdc.xdc):
//     set_false_path / set_max_delay -datapath_only on snap -> dst_*,
//     ASYNC_REG on the toggle synchronisers.
//
//   Latency: ~2 src + 2 dst cycles + 2 for edge detect/copy.  Throughput is
//   one snapshot per round trip, which is exactly what an on-demand
//   register read needs.
// =============================================================================

`default_nettype none

module ptp_time_snapshot #(
    parameter int SEC_W = 48,
    parameter int NS_W  = 32,
    parameter int SYNC_STAGES = 2
)(
    // ---- source (network clock) domain ----
    input  wire              src_clk,
    input  wire              src_rst_n,
    input  wire [SEC_W-1:0]  src_time_sec,
    input  wire [NS_W-1:0]   src_time_ns,

    // ---- destination (core / AXI) domain ----
    input  wire              dst_clk,
    input  wire              dst_rst_n,
    input  wire              dst_req,      // pulse: take a snapshot
    output logic             dst_busy,     // request in flight
    output logic             dst_done,     // pulse: dst_sec/dst_ns updated
    output logic [SEC_W-1:0] dst_sec,
    output logic [NS_W-1:0]  dst_ns
);

    localparam int TS_W = SEC_W + NS_W;

    // ---------------------------------------------------------------
    // dst side: request toggle
    // ---------------------------------------------------------------
    logic req_tgl;
    logic ack_tgl;                                   // src-domain flop
    (* ASYNC_REG = "TRUE" *) logic [SYNC_STAGES-1:0] ack_sync;
    logic ack_s;

    assign ack_s = ack_sync[SYNC_STAGES-1];

    always_ff @(posedge dst_clk) begin
        if (!dst_rst_n) ack_sync <= '0;
        else            ack_sync <= {ack_sync[SYNC_STAGES-2:0], ack_tgl};
    end

    // ---------------------------------------------------------------
    // src side: synchronise the request, latch on toggle, ack
    // ---------------------------------------------------------------
    (* ASYNC_REG = "TRUE" *) logic [SYNC_STAGES-1:0] req_sync;
    logic req_s;
    logic req_seen;                                  // last acknowledged level
    logic [TS_W-1:0] snap;                           // holding register
    logic latch_now;

    assign req_s     = req_sync[SYNC_STAGES-1];
    assign latch_now = (req_s != req_seen);

    always_ff @(posedge src_clk) begin
        if (!src_rst_n) begin
            req_sync <= '0;
            req_seen <= 1'b0;
            ack_tgl  <= 1'b0;
            snap     <= '0;
        end else begin
            req_sync <= {req_sync[SYNC_STAGES-2:0], req_tgl};
            if (latch_now) begin
                snap     <= {src_time_sec, src_time_ns};
                req_seen <= req_s;
                ack_tgl  <= ~ack_tgl;
            end
        end
    end

    // ---------------------------------------------------------------
    // dst side: issue request, wait for ack, copy the stable snapshot
    // ---------------------------------------------------------------
    always_ff @(posedge dst_clk) begin
        if (!dst_rst_n) begin
            req_tgl  <= 1'b0;
            dst_busy <= 1'b0;
            dst_done <= 1'b0;
            dst_sec  <= '0;
            dst_ns   <= '0;
        end else begin
            dst_done <= 1'b0;
            if (dst_req && !dst_busy) begin
                req_tgl  <= ~req_tgl;
                dst_busy <= 1'b1;
            end else if (dst_busy && (ack_s == req_tgl)) begin
                dst_sec  <= snap[TS_W-1:NS_W];       // multi-cycle path, stable
                dst_ns   <= snap[NS_W-1:0];
                dst_busy <= 1'b0;
                dst_done <= 1'b1;
            end
        end
    end

endmodule

`default_nettype wire
