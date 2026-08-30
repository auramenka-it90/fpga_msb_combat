`include "define.v"
`timescale 1ns / 1ps

// =============================================================================
// MODULE: interrupt_controller (Device ID = 3)
// 
// DESCRIPTION:
//  Universal Hardware Interrupt Controller with Write-1-to-Clear (W1C) logic.
//  - 3-Stage Metastability Synchronizer on raw interrupt lines.
//  - Configurable Edge Detectors (Rising or Falling edge per line).
//  - Hardware Masking: Reading PENDING register automatically applies MASK.
//  - Active-Low Output 'irq_out_N' (P51) driven to '0' while IRQ is active and enabled.
//
//  Target Silicon: Xilinx Spartan-6 (XC6SLX4 / XC6SLX9-TQG144)
//  Toolchain:      Aldec Active-HDL 9.2 / ISE 14.7 / XST
//  All comments in pure ASCII English.
// =============================================================================

module interrupt_controller #(
    parameter ADR_WIDTH = `_D_S_CHIP_ADDR_WIDTH_, // 6 bits
    parameter IRQ_LINES = 1                      // 1 active line for MSB (100 Hz timer strobe)
)(
    // --- System Signals ---
    input  wire                 clk,         // System clock (100 MHz)
    input  wire                 rst,         // Synchronous reset (Active-High)

    // --- CPU Polling SPI Bus Interface ---
    input  wire [ADR_WIDTH-1:0] cpu_addr,    // Register address offset
    input  wire [15:0]          cpu_di,      // Data from STM32 to FPGA
    input  wire                 cpu_wr,      // Write strobe (Active-High)
    input  wire                 cpu_rd,      // Read strobe (Active-High)
    output wire [15:0]          cpu_do,      // Data from FPGA to STM32

    // --- Hardware Interrupt Lines ---
    input  wire [IRQ_LINES-1:0] irq_inputs,  // Raw event strobe from internal timer
    output wire                 irq_out_N    // Global active-low interrupt to STM32 PC3
);

    // =========================================================================
    // 1. REGISTER ADDRESS MAP
    // =========================================================================
    localparam BASE_ADDR      = {ADR_WIDTH{1'b0}};
    localparam P_OFF_PENDING  = 6'd0; // [R] Masked Pending / [W1C] Clear Pending
    localparam P_OFF_MASK     = 6'd1; // [RW] Mask (1 = Enabled, 0 = Disabled)
    localparam P_OFF_EDGE_SEL = 6'd2; // [RW] Edge Select (0 = Rising, 1 = Falling)
    localparam P_OFF_CTRL     = 6'd3; // [RW] Control (Bit 0 = Global Interrupt Enable)

    // Internal state registers
    reg [IRQ_LINES-1:0] pending_reg;
    reg [IRQ_LINES-1:0] mask_reg;
    reg [IRQ_LINES-1:0] edge_sel_reg;
    reg [15:0]          ctrl_reg;

    // =========================================================================
    // 2. SYNCHRONIZERS & EDGE DETECTORS (3-Stage Shift Register)
    // =========================================================================
    reg [IRQ_LINES-1:0] sync1_reg, sync2_reg, sync3_reg;

    always @(posedge clk) begin
        if (rst) begin
            sync1_reg <= {IRQ_LINES{1'b0}};
            sync2_reg <= {IRQ_LINES{1'b0}};
            sync3_reg <= {IRQ_LINES{1'b0}};
        end else begin
            sync1_reg <= irq_inputs;
            sync2_reg <= sync1_reg;
            sync3_reg <= sync2_reg;
        end
    end

    // Edge detectors: Generates a single clock cycle pulse on selected edge
    wire [IRQ_LINES-1:0] irq_events;
    genvar i;
    generate
        for (i = 0; i < IRQ_LINES; i = i + 1) begin : edge_detectors
            assign irq_events[i] = edge_sel_reg[i] ? 
                                   (sync3_reg[i] & ~sync2_reg[i]) : // Falling edge
                                   (~sync3_reg[i] & sync2_reg[i]);  // Rising edge
        end
    endgenerate

    // =========================================================================
    // 3. BUS INTERFACE DECODING
    // =========================================================================
    wire wr_pending  = (cpu_addr == (BASE_ADDR + P_OFF_PENDING))  && cpu_wr;
    wire wr_mask     = (cpu_addr == (BASE_ADDR + P_OFF_MASK))     && cpu_wr;
    wire wr_edge_sel = (cpu_addr == (BASE_ADDR + P_OFF_EDGE_SEL)) && cpu_wr;
    wire wr_ctrl     = (cpu_addr == (BASE_ADDR + P_OFF_CTRL))     && cpu_wr;

    wire rd_pending  = (cpu_addr == (BASE_ADDR + P_OFF_PENDING))  && cpu_rd;
    wire rd_mask     = (cpu_addr == (BASE_ADDR + P_OFF_MASK))     && cpu_rd;
    wire rd_edge_sel = (cpu_addr == (BASE_ADDR + P_OFF_EDGE_SEL)) && cpu_rd;
    wire rd_ctrl     = (cpu_addr == (BASE_ADDR + P_OFF_CTRL))     && cpu_rd;

    // =========================================================================
    // 4. MAIN CONTROL LOGIC (Industrial W1C Standard)
    // =========================================================================
    always @(posedge clk) begin
        if (rst) begin
            pending_reg  <= {IRQ_LINES{1'b0}};
            mask_reg     <= {IRQ_LINES{1'b0}};
            edge_sel_reg <= {IRQ_LINES{1'b0}};
            ctrl_reg     <= 16'h0000; 
        end else begin
            
            // --- Write-1-to-Clear (W1C) Execution ---
            if (wr_pending) begin
                pending_reg <= (pending_reg & ~cpu_di[IRQ_LINES-1:0]) | irq_events;
            end else begin
                pending_reg <= pending_reg | irq_events;
            end

            // Configuration writes
            if (wr_mask)     mask_reg     <= cpu_di[IRQ_LINES-1:0];
            if (wr_edge_sel) edge_sel_reg <= cpu_di[IRQ_LINES-1:0];
            if (wr_ctrl)     ctrl_reg     <= cpu_di;
        end
    end

    // =========================================================================
    // 5. BUS READ MULTIPLEXER (Hardware Masking Applied Automatically)
    // =========================================================================  
    assign cpu_do = rd_pending  ? { {(16-IRQ_LINES){1'b0}}, (pending_reg & mask_reg) } :
                    rd_mask     ? { {(16-IRQ_LINES){1'b0}}, mask_reg }                 :
                    rd_edge_sel ? { {(16-IRQ_LINES){1'b0}}, edge_sel_reg }             :
                    rd_ctrl     ? ctrl_reg                                             :
                    16'h0000;

    // =========================================================================
    // 6. PHYSICAL INTERRUPT OUTPUT (Active-Low)
    // Line goes Low ('0') if unmasked IRQ is active AND Global Enable is ON (ctrl_reg[0])
    // =========================================================================
    wire interrupt_active = |(pending_reg & mask_reg);

    assign irq_out_N = (ctrl_reg[0] && interrupt_active) ? 1'b0 : 1'b1;

endmodule