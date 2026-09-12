// =============================================================================
// ptp_ts_capture.sv
//
// Timestamp capture unit.  On the rising edge of `event_in` the current PTP
// time {sec, ns} is latched and pushed into a FIFO.  A downstream consumer
// (the latency histogram, or software over AXI) pops timestamps in order.
//
//   Resolution: one network-clock period (6.4 ns at 156.25 MHz).  The
//   fractional accumulator bits are NOT captured -- they describe the
//   clock's rate, not the position of the event inside the period.
//   Sub-period interpolation needs a delay line or oversampled phase
//   detector and is out of scope for a simulation-only design.
//
//   Event input: EVENT_SYNC_STAGES = 0 -> event_in is already synchronous
//   to clk (e.g. a MAC start-of-frame strobe).  EVENT_SYNC_STAGES >= 2 ->
//   event_in is asynchronous (external pin) and is passed through that many
//   flops first; the timestamp then carries a fixed 2-3 cycle latency that
//   cancels in any t_out - t_in difference.
//
//   Latency (sync input): event seen at edge N, time sampled at edge N,
//   FIFO write at edge N+1 -- the captured value is the time the clock
//   held during the cycle the event arrived.
//
//   Overflow: an event arriving while the FIFO is full is dropped and
//   `drop_count` (saturating) is incremented, so software can tell the
//   histogram statistics are incomplete.
// =============================================================================

`default_nettype none

module ptp_ts_capture #(
    parameter int SEC_W             = 48,
    parameter int NS_W              = 32,
    parameter int FIFO_DEPTH_LOG2   = 4,
    parameter int EVENT_SYNC_STAGES = 0,
    parameter int DROP_CNT_W        = 16
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // live time from ptp_clock_core
    input  wire  [SEC_W-1:0]        time_sec,
    input  wire  [NS_W-1:0]         time_ns,

    // event strobe (rising-edge sensitive)
    input  wire                     event_in,

    // captured timestamps, FIFO pop interface
    output logic                    ts_valid,
    output logic [SEC_W-1:0]        ts_sec,
    output logic [NS_W-1:0]         ts_ns,
    input  wire                     ts_ready,

    // status
    output logic [FIFO_DEPTH_LOG2:0] fifo_count,
    output logic [DROP_CNT_W-1:0]   drop_count,
    input  wire                     drop_clear
);

    // ---- optional synchroniser for asynchronous strobes ----------------
    logic event_s;

    generate
        if (EVENT_SYNC_STAGES == 0) begin : g_sync_in
            assign event_s = event_in;
        end else begin : g_async_in
            (* ASYNC_REG = "TRUE" *) logic [EVENT_SYNC_STAGES-1:0] sync_q;
            always_ff @(posedge clk) begin
                if (!rst_n) sync_q <= '0;
                else        sync_q <= {sync_q[EVENT_SYNC_STAGES-2:0], event_in};
            end
            assign event_s = sync_q[EVENT_SYNC_STAGES-1];
        end
    endgenerate

    // ---- rising-edge detect --------------------------------------------
    logic event_d;
    logic event_rise;

    always_ff @(posedge clk) begin
        if (!rst_n) event_d <= 1'b0;
        else        event_d <= event_s;
    end

    assign event_rise = event_s & ~event_d;

    // ---- FIFO ----------------------------------------------------------
    logic fifo_full, fifo_empty;

    sync_fifo #(
        .WIDTH      (SEC_W + NS_W),
        .DEPTH_LOG2 (FIFO_DEPTH_LOG2)
    ) u_fifo (
        .clk     (clk),
        .rst_n   (rst_n),
        .wr_en   (event_rise & ~fifo_full),
        .wr_data ({time_sec, time_ns}),
        .full    (fifo_full),
        .rd_en   (ts_ready & ~fifo_empty),
        .rd_data ({ts_sec, ts_ns}),
        .empty   (fifo_empty),
        .count   (fifo_count)
    );

    assign ts_valid = ~fifo_empty;

    // ---- drop counter (saturating) -------------------------------------
    always_ff @(posedge clk) begin
        if (!rst_n || drop_clear)
            drop_count <= '0;
        else if (event_rise && fifo_full && ~&drop_count)
            drop_count <= drop_count + 1'b1;
    end

endmodule

`default_nettype wire
