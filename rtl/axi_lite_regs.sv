// =============================================================================
// axi_lite_regs.sv
//
// AXI4-Lite slave (32-bit data, 12-bit byte address, one transaction at a
// time).  Registers at 0x000..0x0FF live in the AXI clock domain; anything
// at 0x100 and above is forwarded through cdc_bus_bridge to ptp_net_regs
// in the network clock domain and completes when the bridge returns.
//
//   0x000 ID            RO   'PTSU'
//   0x004 VERSION       RO
//   0x008 CTRL          WO   [0] SNAP_REQ  (self-clearing)
//   0x00C STATUS        RO   [0] snap_busy  [1] bridge_busy
//   0x010 SNAP_NS       RO   last snapshot, coherent {sec, ns}
//   0x014 SNAP_SEC_LO   RO
//   0x018 SNAP_SEC_HI   RO   sec[47:32]
// =============================================================================

`default_nettype none

module axi_lite_regs #(
    parameter int SEC_W = 48,
    parameter int NS_W  = 32
)(
    input  wire              aclk,
    input  wire              aresetn,

    // AXI4-Lite slave
    input  wire  [11:0]      s_axi_awaddr,
    input  wire              s_axi_awvalid,
    output logic             s_axi_awready,
    input  wire  [31:0]      s_axi_wdata,
    input  wire  [3:0]       s_axi_wstrb,
    input  wire              s_axi_wvalid,
    output logic             s_axi_wready,
    output logic [1:0]       s_axi_bresp,
    output logic             s_axi_bvalid,
    input  wire              s_axi_bready,
    input  wire  [11:0]      s_axi_araddr,
    input  wire              s_axi_arvalid,
    output logic             s_axi_arready,
    output logic [31:0]      s_axi_rdata,
    output logic [1:0]       s_axi_rresp,
    output logic             s_axi_rvalid,
    input  wire              s_axi_rready,

    // time snapshot block (dst side)
    output logic             snap_req,
    input  wire              snap_busy,
    input  wire              snap_done,
    input  wire  [SEC_W-1:0] snap_sec,
    input  wire  [NS_W-1:0]  snap_ns,

    // bridge master port (to network domain)
    output logic             br_req,
    output logic [11:0]      br_addr,
    output logic [31:0]      br_wdata,
    output logic             br_we,
    input  wire              br_busy,
    input  wire              br_done,
    input  wire  [31:0]      br_rdata
);

    localparam logic [11:0] A_ID = 12'h000, A_VERSION = 12'h004, A_CTRL = 12'h008,
                            A_STATUS = 12'h00C, A_SNAP_NS = 12'h010,
                            A_SNAP_SEC_LO = 12'h014, A_SNAP_SEC_HI = 12'h018;

    typedef enum logic [2:0] { IDLE, WR_LOCAL, WR_BRIDGE, WR_RESP, RD_LOCAL, RD_BRIDGE, RD_RESP } state_t;
    state_t state;

    logic [11:0] addr_q;
    logic [31:0] wdata_q;
    logic        is_net;
    assign is_net = (addr_q >= 12'h100);

    // Snapshot result latched in this domain on snap_done so a partial
    // read sequence (NS, SEC_LO, SEC_HI) is always self-consistent.
    logic [SEC_W-1:0] snap_sec_q;
    logic [NS_W-1:0]  snap_ns_q;
    always_ff @(posedge aclk) begin
        if (!aresetn) begin
            snap_sec_q <= '0;
            snap_ns_q  <= '0;
        end else if (snap_done) begin
            snap_sec_q <= snap_sec;
            snap_ns_q  <= snap_ns;
        end
    end

    // local read mux
    logic [31:0] local_rdata;
    always_comb begin
        case (addr_q)
            A_ID:          local_rdata = 32'h5054_5355;              // "PTSU"
            A_VERSION:     local_rdata = 32'h0001_0000;
            A_STATUS:      local_rdata = {30'd0, br_busy, snap_busy};
            A_SNAP_NS:     local_rdata = snap_ns_q;
            A_SNAP_SEC_LO: local_rdata = snap_sec_q[31:0];
            A_SNAP_SEC_HI: local_rdata = 32'(snap_sec_q[SEC_W-1:32]);
            default:       local_rdata = 32'h0;
        endcase
    end

    // ---- FSM ---------------------------------------------------------------
    always_ff @(posedge aclk) begin
        if (!aresetn) begin
            state         <= IDLE;
            s_axi_awready <= 1'b0;
            s_axi_wready  <= 1'b0;
            s_axi_bvalid  <= 1'b0;
            s_axi_bresp   <= 2'b00;
            s_axi_arready <= 1'b0;
            s_axi_rvalid  <= 1'b0;
            s_axi_rresp   <= 2'b00;
            s_axi_rdata   <= '0;
            snap_req      <= 1'b0;
            br_req        <= 1'b0;
            br_we         <= 1'b0;
            br_addr       <= '0;
            br_wdata      <= '0;
        end else begin
            s_axi_awready <= 1'b0;
            s_axi_wready  <= 1'b0;
            s_axi_arready <= 1'b0;
            snap_req      <= 1'b0;
            br_req        <= 1'b0;

            case (state)
                IDLE: begin
                    if (s_axi_awvalid && s_axi_wvalid) begin
                        addr_q        <= s_axi_awaddr;
                        wdata_q       <= s_axi_wdata;
                        s_axi_awready <= 1'b1;
                        s_axi_wready  <= 1'b1;
                        state         <= WR_LOCAL;
                    end else if (s_axi_arvalid) begin
                        addr_q        <= s_axi_araddr;
                        s_axi_arready <= 1'b1;
                        state         <= RD_LOCAL;
                    end
                end

                WR_LOCAL: begin
                    if (is_net) begin
                        br_req   <= 1'b1;
                        br_addr  <= addr_q;
                        br_wdata <= wdata_q;
                        br_we    <= 1'b1;
                        state    <= WR_BRIDGE;
                    end else begin
                        if (addr_q == A_CTRL && wdata_q[0]) snap_req <= 1'b1;
                        s_axi_bvalid <= 1'b1;
                        s_axi_bresp  <= 2'b00;
                        state        <= WR_RESP;
                    end
                end

                WR_BRIDGE: begin
                    if (br_done) begin
                        s_axi_bvalid <= 1'b1;
                        s_axi_bresp  <= 2'b00;
                        state        <= WR_RESP;
                    end
                end

                WR_RESP: begin
                    if (s_axi_bready) begin
                        s_axi_bvalid <= 1'b0;
                        state        <= IDLE;
                    end
                end

                RD_LOCAL: begin
                    if (is_net) begin
                        br_req  <= 1'b1;
                        br_addr <= addr_q;
                        br_we   <= 1'b0;
                        state   <= RD_BRIDGE;
                    end else begin
                        s_axi_rdata  <= local_rdata;
                        s_axi_rresp  <= 2'b00;
                        s_axi_rvalid <= 1'b1;
                        state        <= RD_RESP;
                    end
                end

                RD_BRIDGE: begin
                    if (br_done) begin
                        s_axi_rdata  <= br_rdata;
                        s_axi_rresp  <= 2'b00;
                        s_axi_rvalid <= 1'b1;
                        state        <= RD_RESP;
                    end
                end

                RD_RESP: begin
                    if (s_axi_rready) begin
                        s_axi_rvalid <= 1'b0;
                        state        <= IDLE;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire
