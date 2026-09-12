// =============================================================================
// ptp_clock_core.sv
//
// IEEE-1588 (PTP) hardware clock core.
//
//   time = { seconds[SEC_W-1:0], nanoseconds[NS_W-1:0] }  (48 + 32 = 80 bits
//   in the PTP Timestamp format; the 32-bit ns field is always < 1e9)
//
//   Every clk cycle the clock advances by an increment expressed in
//   Q(INT_W).(FRAC_W) fixed-point nanoseconds:
//
//       incr_eff = nom_incr + freq_adj
//
//   nom_incr is the nominal period of clk (6.4 ns at 156.25 MHz ->
//   0x6_6666_6666 with FRAC_W = 32).  freq_adj is a signed addend written by
//   the servo.  Because the increment carries FRAC_W fractional bits, the
//   effective clock rate can be slewed with sub-ppb resolution:
//
//       1 LSB of freq_adj = 2^-32 ns/cycle = 1/(6.4 * 2^32) = 0.036 ppb
//       1 ppb              = 6.4e-9 ns/cycle * 2^32 = 27.49 LSB
//
//   This is the same DDS / phase-accumulator technique used for NCOs: the
//   fractional accumulator carries into the integer ns field on average
//   frac(incr_eff) times per cycle, so the long-run rate is exact even
//   though each individual cycle only adds an integer number of ns.
//
//   Phase adjustment: a one-shot signed offset (adj_sec, adj_ns) is added
//   in the same cycle as the normal increment when adj_valid is high.
//   |adj_ns| must be < 1e9 (split larger offsets across adj_sec).
//
//   Absolute load: set_valid loads (set_sec, set_ns) and clears the
//   fractional accumulator.  set_valid has priority over adj_valid.
//
//   pps: single-cycle pulse whenever the ns field rolls over 1e9
//   (i.e. the seconds field increments by carry).
//
// Timing: the critical path is a 3-operand 34-bit add, a compare against
// 1e9 and a +/- 1e9 correction mux, all in one cycle.  Comfortable at
// 156.25 MHz on 7-series; see synth/ for the OOC report.
// =============================================================================

`default_nettype none

module ptp_clock_core #(
    parameter int SEC_W  = 48,   // seconds field width (PTP spec: 48)
    parameter int NS_W   = 32,   // nanoseconds field width (PTP spec: 32)
    parameter int FRAC_W = 32,   // fractional ns bits of the increment
    parameter int INT_W  = 8     // integer ns bits of the increment (period < 256 ns)
)(
    input  wire                         clk,
    input  wire                         rst_n,

    // ---- rate control -------------------------------------------------
    input  wire  [INT_W+FRAC_W-1:0]     nom_incr,   // Q(INT_W).(FRAC_W) ns per cycle
    input  wire  signed [FRAC_W-1:0]    freq_adj,   // signed, same LSB as nom_incr
                                                    // (nom_incr + freq_adj must be > 0)

    // ---- absolute time load -------------------------------------------
    input  wire                         set_valid,
    input  wire  [SEC_W-1:0]            set_sec,
    input  wire  [NS_W-1:0]             set_ns,     // must be < 1e9

    // ---- one-shot phase offset ----------------------------------------
    input  wire                         adj_valid,
    input  wire  signed [31:0]          adj_sec,
    input  wire  signed [31:0]          adj_ns,     // |adj_ns| < 1e9

    // ---- time output --------------------------------------------------
    output logic [SEC_W-1:0]            time_sec,
    output logic [NS_W-1:0]             time_ns,
    output logic [FRAC_W-1:0]           time_frac,
    output logic                        pps
);

    localparam int INCR_W = INT_W + FRAC_W;
    localparam int NSS_W  = NS_W + 2;                // signed ns arithmetic width
    localparam logic signed [NSS_W-1:0] NS_PER_SEC = NSS_W'(1_000_000_000);

    // ------------------------------------------------------------------
    // Effective increment, registered so that the nom_incr + freq_adj add
    // is not in series with the accumulator add.
    // ------------------------------------------------------------------
    logic [INCR_W-1:0] incr_eff;

    always_ff @(posedge clk) begin
        if (!rst_n) incr_eff <= '0;
        else        incr_eff <= nom_incr + INCR_W'($signed(freq_adj));
    end

    // ------------------------------------------------------------------
    // Fractional accumulator (DDS).  Carry-out bumps the integer ns.
    // ------------------------------------------------------------------
    logic [FRAC_W:0] frac_sum;
    logic            frac_carry;

    assign frac_sum   = {1'b0, time_frac} + {1'b0, incr_eff[FRAC_W-1:0]};
    assign frac_carry = frac_sum[FRAC_W];

    // ------------------------------------------------------------------
    // Nanosecond add and normalisation into [0, 1e9).
    //
    //   time_ns  in [0, 1e9)
    //   incr_int in [0, 2^INT_W)
    //   adj_ns   in (-1e9, 1e9)
    //   => ns_sum in (-1e9, 2e9 + 2^INT_W)  -> fits NS_W+2 signed bits,
    //      and a single +/- 1e9 correction always lands in range.
    // ------------------------------------------------------------------
    logic [INT_W-1:0]        incr_int;   // integer-ns part of the increment
    logic signed [NSS_W-1:0] ns_sum;
    logic signed [NSS_W-1:0] ns_norm;
    logic                    ns_ge;      // rolled over  -> carry into seconds
    logic                    ns_lt;      // went negative -> borrow from seconds

    assign incr_int = incr_eff[INCR_W-1:FRAC_W];
    assign ns_lt    = ns_sum[NSS_W-1];        // sign bit

    always_comb begin
        ns_sum = $signed({2'b00, time_ns})
               + $signed({{(NSS_W-INT_W){1'b0}}, incr_int})
               + $signed({{(NSS_W-1){1'b0}}, frac_carry})
               + (adj_valid ? NSS_W'(adj_ns) : NSS_W'(0));

        ns_ge = (ns_sum >= NS_PER_SEC);

        if (ns_ge)      ns_norm = ns_sum - NS_PER_SEC;
        else if (ns_lt) ns_norm = ns_sum + NS_PER_SEC;
        else            ns_norm = ns_sum;
    end

    // ------------------------------------------------------------------
    // Seconds: add the one-shot offset plus the carry/borrow from ns.
    // ------------------------------------------------------------------
    logic signed [SEC_W-1:0] sec_delta;

    always_comb begin
        sec_delta = (adj_valid ? SEC_W'(adj_sec) : SEC_W'(0))
                  + (ns_ge ? SEC_W'(1) : SEC_W'(0))
                  - (ns_lt ? SEC_W'(1) : SEC_W'(0));
    end

    // ------------------------------------------------------------------
    // State update
    // ------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            time_sec  <= '0;
            time_ns   <= '0;
            time_frac <= '0;
            pps       <= 1'b0;
        end else if (set_valid) begin
            time_sec  <= set_sec;
            time_ns   <= set_ns;
            time_frac <= '0;
            pps       <= 1'b0;
        end else begin
            time_sec  <= time_sec + sec_delta;
            time_ns   <= ns_norm[NS_W-1:0];
            time_frac <= frac_sum[FRAC_W-1:0];
            pps       <= ns_ge;
        end
    end

endmodule

`default_nettype wire
