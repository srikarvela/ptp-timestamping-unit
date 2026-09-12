// =============================================================================
// cdc_bus_bridge.sv
//
// Carries one register transaction (addr, wdata, we) from the dst (AXI)
// clock domain into the src (network) clock domain and returns rdata --
// the same 2-phase toggle handshake as ptp_time_snapshot, generalised to
// carry a payload in both directions.
//
//   dst:  m_req pulse with m_addr/m_wdata/m_we held until m_done.
//   src:  s_valid is held high with s_addr/s_wdata/s_we until the
//         register file answers with s_rvalid (same cycle for simple
//         registers, later for e.g. a BRAM read); s_rdata is captured on
//         s_valid & s_rvalid.  Writes must be applied on the first cycle
//         of s_valid (use s_valid & s_rvalid, or edge-detect s_valid).
//   dst:  m_done pulse, m_rdata valid.
//
//   Payload registers (m_addr/m_wdata/m_we on the dst side, rdata_hold on
//   the src side) are frozen while the corresponding toggle is in flight,
//   so they cross as multi-cycle paths qualified by the synchronised
//   toggle.  Constrain with set_max_delay -datapath_only / set_bus_skew
//   the same way as ptp_time_snapshot (see synth/ptp_cdc.xdc).
// =============================================================================

`default_nettype none

module cdc_bus_bridge #(
    parameter int ADDR_W = 12,
    parameter int DATA_W = 32,
    parameter int SYNC_STAGES = 2
)(
    // ---- dst (master) side ----
    input  wire               dst_clk,
    input  wire               dst_rst_n,
    input  wire               m_req,
    input  wire  [ADDR_W-1:0] m_addr,
    input  wire  [DATA_W-1:0] m_wdata,
    input  wire               m_we,
    output logic              m_busy,
    output logic              m_done,
    output logic [DATA_W-1:0] m_rdata,

    // ---- src (slave) side ----
    input  wire               src_clk,
    input  wire               src_rst_n,
    output logic              s_valid,
    output logic [ADDR_W-1:0] s_addr,
    output logic [DATA_W-1:0] s_wdata,
    output logic              s_we,
    input  wire               s_rvalid,
    input  wire  [DATA_W-1:0] s_rdata
);

    // dst -> src
    logic              req_tgl;
    logic [ADDR_W-1:0] addr_hold;
    logic [DATA_W-1:0] wdata_hold;
    logic              we_hold;
    (* ASYNC_REG = "TRUE" *) logic [SYNC_STAGES-1:0] req_sync;
    logic              req_s, req_seen;

    // src -> dst
    logic              ack_tgl;
    logic [DATA_W-1:0] rdata_hold;
    (* ASYNC_REG = "TRUE" *) logic [SYNC_STAGES-1:0] ack_sync;
    logic              ack_s;

    assign req_s = req_sync[SYNC_STAGES-1];
    assign ack_s = ack_sync[SYNC_STAGES-1];

    // ---- dst side ----
    always_ff @(posedge dst_clk) begin
        if (!dst_rst_n) begin
            req_tgl  <= 1'b0;
            m_busy   <= 1'b0;
            m_done   <= 1'b0;
            m_rdata  <= '0;
            ack_sync <= '0;
        end else begin
            ack_sync <= {ack_sync[SYNC_STAGES-2:0], ack_tgl};
            m_done   <= 1'b0;
            if (m_req && !m_busy) begin
                req_tgl    <= ~req_tgl;
                addr_hold  <= m_addr;
                wdata_hold <= m_wdata;
                we_hold    <= m_we;
                m_busy     <= 1'b1;
            end else if (m_busy && (ack_s == req_tgl)) begin
                m_rdata <= rdata_hold;               // MCP: frozen since ack toggled
                m_busy  <= 1'b0;
                m_done  <= 1'b1;
            end
        end
    end

    // ---- src side ----
    always_ff @(posedge src_clk) begin
        if (!src_rst_n) begin
            req_sync <= '0;
            req_seen <= 1'b0;
            ack_tgl  <= 1'b0;
            s_valid  <= 1'b0;
        end else begin
            req_sync <= {req_sync[SYNC_STAGES-2:0], req_tgl};
            if (!s_valid && (req_s != req_seen)) begin   // new request landed
                req_seen <= req_s;
                s_valid  <= 1'b1;
                s_addr   <= addr_hold;             // MCP: frozen while req in flight
                s_wdata  <= wdata_hold;
                s_we     <= we_hold;
            end else if (s_valid && s_rvalid) begin      // register file answered
                s_valid    <= 1'b0;
                rdata_hold <= s_rdata;
                ack_tgl    <= ~ack_tgl;
            end
        end
    end

endmodule

`default_nettype wire
