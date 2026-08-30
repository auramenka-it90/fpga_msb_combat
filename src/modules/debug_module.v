`include "define.v"
`timescale 1ns / 1ps

// =============================================================================
// MODULE: debug_module (Device ID = 1)
// 
// DESCRIPTION:
//  System Debug, Configuration, Testpoint Routing & MultiBoot Control Module.
//  Target Silicon: Xilinx Spartan-6 (XC6SLX4 / XC6SLX9-TQG144)
//  Toolchain:      ISE 14.7 / XST
//  All comments in ASCII English.
// =============================================================================

module debug_module #(
    parameter ADR_WIDTH  = `_D_S_CHIP_ADDR_WIDTH_, // 6 bits register offset
    parameter DATA_WIDTH = `_D_DATA_WIDTH_         // 16 bits data bus
)(
    input  wire                  clk,
    input  wire                  rst,
    
    // --- Polling SPI CPU Bus Interface ---
    input  wire [ADR_WIDTH-1:0]  cpu_addr,
    input  wire [DATA_WIDTH-1:0] cpu_di,
    input  wire                  cpu_wr, 
    input  wire                  cpu_rd, 
    output wire [DATA_WIDTH-1:0] cpu_do,
    
    // --- Internal Hardware Control Outputs ---
    output wire [DATA_WIDTH-1:0] misc_out,         // Bits [2:0]=LEDs, [3]=UART MUX, [11:4]=Timer Div
    output wire [14:0]           tp_mux_out,       // Dynamic TP Routing: [4:0]=TP5, [9:5]=TP6, [14:10]=TP7
    output reg                   reconfig_trigger  // 1-cycle pulse to trigger MultiBoot IPROG reboot
);

    // =========================================================================
    // 1. REGISTER MAP OFFSETS & CONSTANTS
    // =========================================================================
    localparam P_OFF_FEED_BACK = 6'd0; 
    localparam P_OFF_CONST     = 6'd1;
    localparam P_OFF_MISC      = 6'd2;
    localparam P_OFF_TP_MUX    = 6'd3;
    localparam P_OFF_RECONFIG  = 6'd4;

    localparam [DATA_WIDTH-1:0] P_CONST_VAL    = 16'hDEAD; // Alive check signature
    localparam [DATA_WIDTH-1:0] P_RECONFIG_KEY = 16'hAA55; // MultiBoot Safety Magic Key

    reg [DATA_WIDTH-1:0] fb_reg;
    reg [DATA_WIDTH-1:0] misc_reg;
    reg [14:0]           tp_mux_reg;

    // =========================================================================
    // 2. REGISTER WRITE DECODING & EXECUTION
    // =========================================================================
    wire wr_fb       = (cpu_addr == P_OFF_FEED_BACK) && cpu_wr;
    wire wr_misc     = (cpu_addr == P_OFF_MISC)      && cpu_wr;
    wire wr_tp_mux   = (cpu_addr == P_OFF_TP_MUX)    && cpu_wr;
    wire wr_reconfig = (cpu_addr == P_OFF_RECONFIG)  && cpu_wr && (cpu_di == P_RECONFIG_KEY);

    always @(posedge clk) begin
        if (rst) begin
            fb_reg           <= {DATA_WIDTH{1'b0}};
            misc_reg         <= {DATA_WIDTH{1'b0}};
            tp_mux_reg       <= 15'd0; // Default: All TPs route to signal index 0
            reconfig_trigger <= 1'b0;
        end else begin
            reconfig_trigger <= 1'b0;  // Default: strict 1-cycle pulse

            if (wr_fb)       fb_reg           <= cpu_di;
            if (wr_misc)     misc_reg         <= cpu_di;
            if (wr_tp_mux)   tp_mux_reg       <= cpu_di[14:0];
            if (wr_reconfig) reconfig_trigger <= 1'b1; // Trigger hardware IPROG reboot!
        end
    end

    assign misc_out   = misc_reg;
    assign tp_mux_out = tp_mux_reg;

    // =========================================================================
    // 3. REGISTER READ DECODING
    // =========================================================================
    wire rd_fb     = (cpu_addr == P_OFF_FEED_BACK) && cpu_rd;
    wire rd_const  = (cpu_addr == P_OFF_CONST)     && cpu_rd;
    wire rd_misc   = (cpu_addr == P_OFF_MISC)      && cpu_rd;
    wire rd_tp_mux = (cpu_addr == P_OFF_TP_MUX)    && cpu_rd;

    assign cpu_do = 
        rd_fb     ? fb_reg             :
        rd_const  ? P_CONST_VAL        :
        rd_misc   ? misc_reg           :
        rd_tp_mux ? {1'b0, tp_mux_reg} :
        {DATA_WIDTH{1'b0}};

endmodule