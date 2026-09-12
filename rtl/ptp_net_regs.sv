// =============================================================================
// ptp_net_regs.sv
//
// Network-clock-domain register file.  Sits behind cdc_bus_bridge and owns
// every control/status register that lives next to the clock core, the
// capture units and the histogram engine.  See docs/REGMAP.md.
//
//   Simple registers answer in the same cycle (s_rvalid = s_valid).
//   Histogram bin reads (0x800..) are forwarded to the engine's read port
//   and answer when rd_ack returns.
// =============================================================================

`default_nettype none

module ptp_net_regs #(
    parameter int SEC_W    = 48,
    parameter int NS_W     = 32,
    parameter int FRAC_W   = 32,
    parameter int INT_W    = 8,
    parameter int BIN_W    = 7,
    parameter int FIFO_CNT_W = 5,
    parameter int DROP_CNT_W = 16
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // bridge slave port
    input  wire                     s_valid,
    input  wire  [11:0]             s_addr,
    input  wire  [31:0]             s_wdata,
    input  wire                     s_we,
    output logic                    s_rvalid,
    output logic [31:0]             s_rdata,

    // clock core control
    output logic [INT_W+FRAC_W-1:0] nom_incr,
    output logic signed [FRAC_W-1:0] freq_adj,
    output logic                    set_valid,
    output logic [SEC_W-1:0]        set_sec,
    output logic [NS_W-1:0]         set_ns,
    output logic                    adj_valid,
    output logic signed [31:0]      adj_sec,
    output logic signed [31:0]      adj_ns,

    // histogram engine
    output logic                    hist_clear,
    input  wire                     hist_busy,
    output logic                    hist_rd_req,
    output logic [BIN_W-1:0]        hist_rd_addr,
    input  wire                     hist_rd_ack,
    input  wire  [31:0]             hist_rd_data,
    input  wire  [31:0]             stat_count,
    input  wire  [63:0]             stat_sum,
    input  wire  [31:0]             stat_min,
    input  wire  [31:0]             stat_max,
    input  wire  [31:0]             stat_neg,

    // capture units
    output logic                    drop_clear,
    input  wire  [DROP_CNT_W-1:0]   drop_in,
    input  wire  [DROP_CNT_W-1:0]   drop_out,
    input  wire  [FIFO_CNT_W-1:0]   fifo_in_count,
    input  wire  [FIFO_CNT_W-1:0]   fifo_out_count
);

    // ---- address map (byte offsets) -------------------------------------
    localparam logic [11:0]
        A_NOM_INCR_LO = 12'h100, A_NOM_INCR_HI = 12'h104, A_FREQ_ADJ = 12'h108,
        A_ADJ_SEC     = 12'h10C, A_ADJ_NS      = 12'h110,
        A_SET_SEC_LO  = 12'h114, A_SET_SEC_HI  = 12'h118, A_SET_NS   = 12'h11C,
        A_HIST_COUNT  = 12'h120, A_HIST_SUM_LO = 12'h124, A_HIST_SUM_HI = 12'h128,
        A_HIST_MIN    = 12'h12C, A_HIST_MAX    = 12'h130, A_HIST_NEG = 12'h134,
        A_DROP_IN     = 12'h138, A_DROP_OUT    = 12'h13C, A_FIFO_LEVELS = 12'h140,
        A_NET_CTRL    = 12'h144, A_NET_STATUS  = 12'h148;

    logic is_bin;
    assign is_bin = (s_addr[11] == 1'b1);          // 0x800 .. 0xBFF

    // ---- one-shot write strobe (first cycle of s_valid only) -------------
    logic s_valid_d;
    logic wr_strobe;
    always_ff @(posedge clk) begin
        if (!rst_n) s_valid_d <= 1'b0;
        else        s_valid_d <= s_valid;
    end
    assign wr_strobe = s_valid & ~s_valid_d & s_we;

    // ---- writes ------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            nom_incr   <= 40'h6_6666_6666;         // 6.4 ns default
            freq_adj   <= '0;
            adj_sec    <= '0;
            adj_ns     <= '0;
            adj_valid  <= 1'b0;
            set_sec    <= '0;
            set_ns     <= '0;
            set_valid  <= 1'b0;
            hist_clear <= 1'b0;
            drop_clear <= 1'b0;
        end else begin
            adj_valid  <= 1'b0;
            set_valid  <= 1'b0;
            hist_clear <= 1'b0;
            drop_clear <= 1'b0;
            if (wr_strobe) begin
                case (s_addr)
                    A_NOM_INCR_LO: nom_incr[FRAC_W-1:0]        <= s_wdata;
                    A_NOM_INCR_HI: nom_incr[INT_W+FRAC_W-1:FRAC_W] <= s_wdata[INT_W-1:0];
                    A_FREQ_ADJ:    freq_adj  <= s_wdata;
                    A_ADJ_SEC:     adj_sec   <= s_wdata;
                    A_ADJ_NS:      begin adj_ns <= s_wdata; adj_valid <= 1'b1; end
                    A_SET_SEC_LO:  set_sec[31:0]  <= s_wdata;
                    A_SET_SEC_HI:  set_sec[SEC_W-1:32] <= s_wdata[SEC_W-33:0];
                    A_SET_NS:      begin set_ns <= s_wdata; set_valid <= 1'b1; end
                    A_NET_CTRL:    begin hist_clear <= s_wdata[0]; drop_clear <= s_wdata[1]; end
                    default: ;
                endcase
            end
        end
    end

    // ---- reads ------------------------------------------------------------
    assign hist_rd_req  = s_valid & ~s_valid_d & ~s_we & is_bin;
    assign hist_rd_addr = s_addr[BIN_W+1:2];

    always_comb begin
        s_rdata  = 32'hDEAD_BEEF;
        s_rvalid = 1'b1;
        if (is_bin) begin
            s_rvalid = s_we ? 1'b1 : hist_rd_ack;
            s_rdata  = hist_rd_data;
        end else begin
            case (s_addr)
                A_NOM_INCR_LO: s_rdata = nom_incr[FRAC_W-1:0];
                A_NOM_INCR_HI: s_rdata = 32'(nom_incr[INT_W+FRAC_W-1:FRAC_W]);
                A_FREQ_ADJ:    s_rdata = freq_adj;
                A_ADJ_SEC:     s_rdata = adj_sec;
                A_ADJ_NS:      s_rdata = adj_ns;
                A_SET_SEC_LO:  s_rdata = set_sec[31:0];
                A_SET_SEC_HI:  s_rdata = 32'(set_sec[SEC_W-1:32]);
                A_SET_NS:      s_rdata = set_ns;
                A_HIST_COUNT:  s_rdata = stat_count;
                A_HIST_SUM_LO: s_rdata = stat_sum[31:0];
                A_HIST_SUM_HI: s_rdata = stat_sum[63:32];
                A_HIST_MIN:    s_rdata = stat_min;
                A_HIST_MAX:    s_rdata = stat_max;
                A_HIST_NEG:    s_rdata = stat_neg;
                A_DROP_IN:     s_rdata = 32'(drop_in);
                A_DROP_OUT:    s_rdata = 32'(drop_out);
                A_FIFO_LEVELS: s_rdata = {16'(fifo_out_count), 16'(fifo_in_count)};
                A_NET_STATUS:  s_rdata = {31'd0, hist_busy};
                default: ;
            endcase
        end
    end

endmodule

`default_nettype wire
