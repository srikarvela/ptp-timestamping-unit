// =============================================================================
// sync_fifo.sv
//
// Single-clock FIFO, registered output, first-word-fall-through style
// (rd_data is valid whenever !empty).  Depth is a power of two.
// Full/empty derived from (DEPTH+1)-bit pointers so all DEPTH slots are used.
// =============================================================================

`default_nettype none

module sync_fifo #(
    parameter int WIDTH = 80,
    parameter int DEPTH_LOG2 = 4
)(
    input  wire              clk,
    input  wire              rst_n,

    input  wire              wr_en,
    input  wire  [WIDTH-1:0] wr_data,
    output logic             full,

    input  wire              rd_en,
    output logic [WIDTH-1:0] rd_data,
    output logic             empty,

    output logic [DEPTH_LOG2:0] count
);

    localparam int DEPTH = 1 << DEPTH_LOG2;

    logic [WIDTH-1:0]      mem [0:DEPTH-1];
    logic [DEPTH_LOG2:0]   wr_ptr, rd_ptr;

    assign empty   = (wr_ptr == rd_ptr);
    assign full    = (wr_ptr[DEPTH_LOG2] != rd_ptr[DEPTH_LOG2]) &&
                     (wr_ptr[DEPTH_LOG2-1:0] == rd_ptr[DEPTH_LOG2-1:0]);
    assign count   = wr_ptr - rd_ptr;
    assign rd_data = mem[rd_ptr[DEPTH_LOG2-1:0]];

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            wr_ptr <= '0;
            rd_ptr <= '0;
        end else begin
            if (wr_en && !full) begin
                mem[wr_ptr[DEPTH_LOG2-1:0]] <= wr_data;
                wr_ptr <= wr_ptr + 1'b1;
            end
            if (rd_en && !empty)
                rd_ptr <= rd_ptr + 1'b1;
        end
    end

endmodule

`default_nettype wire
