
`include "define.v"

module debug_module #(
    parameter ADR_WIDTH = 6
)(
    input  wire                 clk,
    input  wire                 rst,
    input  wire [ADR_WIDTH-1:0] cpu_addr,
    input  wire [15:0]          cpu_di,
    input  wire                 cpu_wr, 
    input  wire                 cpu_rd, 
    output wire [15:0]          cpu_do,
    output wire [15:0]          test_out
);
    // Address definition
    localparam BASE_ADDR      = 6'h00;
    localparam P_OFF_FEED_BACK = 0; 
    localparam P_OFF_CONST     = 1;
    localparam P_OFF_MISC      = 2;
    localparam P_CONST_VAL     = 16'hDEAD;

    reg [15:0] fb_reg;
    reg [15:0] misc_reg;

    // Read Decoding
    wire rd_fb    = (cpu_addr == (BASE_ADDR + P_OFF_FEED_BACK)) && cpu_rd;
    wire rd_const = (cpu_addr == (BASE_ADDR + P_OFF_CONST))     && cpu_rd;
    wire rd_misc  = (cpu_addr == (BASE_ADDR + P_OFF_MISC))      && cpu_rd;

    // Write Decoding
    wire wr_fb    = (cpu_addr == (BASE_ADDR + P_OFF_FEED_BACK)) && cpu_wr;
    wire wr_misc  = (cpu_addr == (BASE_ADDR + P_OFF_MISC))      && cpu_wr;

    // Output Mux
    assign cpu_do = rd_fb    ? fb_reg      :
                    rd_const ? P_CONST_VAL :
                    rd_misc  ? misc_reg    :
                    16'h0000;

    // Register Logic
    always @(posedge clk) begin
        if (rst) begin
            fb_reg   <= 16'h0000;
            misc_reg <= 16'h0000;
        end else begin
            if (wr_fb)   fb_reg   <= cpu_di;
            if (wr_misc) misc_reg <= cpu_di;
        end
    end

    assign test_out = misc_reg; 

endmodule
