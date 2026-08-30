`include "define.v"
`timescale 1ns / 1ps

// =============================================================================
// MODULE: system_clk_rst
// 
// DESCRIPTION:
//  System Clock Management, Power-On Reset (POR), and Master Tick Generator.
//  - Frequency Synthesis: 60.000 MHz (In) -> 100.000 MHz (Out) via DCM_SP.
//  - Startup Safety: Integrated 16-cycle Power-On Reset pulse for DCM.
//  - System Reset: 8-stage Asynchronous Assert Synchronous Deassert (AASD).
//  - Timebase: 1 kHz single-cycle heartbeat tick generator.
//
//  Target Silicon: Xilinx Spartan-6 (XC6SLX...)
//  Toolchain:      ISE 14.7 / XST
//  All comments in ASCII English.
// =============================================================================

module system_clk_rst (
    input  wire  ext_clk_60,      // Primary External Oscillator Input (60 MHz)
    output wire  clk_100,         // Internal Global System Clock (100 MHz)
    output wire  rst_sync,        // Master Synchronous System Reset (Active High)
    output reg   tick_1khz        // 1 kHz Single-Cycle System Heartbeat Strobe
);

    // =========================================================================
    // CONSTANT BIT-WIDTH CALCULATION FUNCTION
    // Fully synthesizable compile-time function for ISE 14.7 / Verilog-2001
    // =========================================================================
    function integer f_clog2;
        input integer depth;
        integer temp;
        begin
            temp = depth;
            for (f_clog2 = 0; temp > 0; f_clog2 = f_clog2 + 1)
                temp = temp >> 1;
        end
    endfunction

    // =========================================================================
    // CLOCK DISTRIBUTION WIRES & BUFFERS
    // =========================================================================
    wire clk_60_ibufg;
    wire clk_fx_raw;
    wire clk_0_raw;
    wire clk_fb;
    wire locked;

    // Dedicated Global Input Clock Buffer
    IBUFG clk_in_buf (
        .I (ext_clk_60), 
        .O (clk_60_ibufg)
    );

    // =========================================================================
    // POWER-ON RESET (POR) SEQUENCER FOR DCM
    // Generates a clean 16-cycle reset pulse on raw 60 MHz clock to ensure 
    // robust DCM locking during power-rail ramps (Spartan-6 Errata mitigation).
    // =========================================================================
    reg [3:0] dcm_por_cnt = 4'hF; // Initialized via FPGA Bitstream INIT attribute

    always @(posedge clk_60_ibufg) begin
        if (dcm_por_cnt != 4'd0) begin
            dcm_por_cnt <= dcm_por_cnt - 1'b1;
        end
    end

    wire dcm_rst = (dcm_por_cnt != 4'd0);

    // =========================================================================
    // SPARTAN-6 DIGITAL CLOCK MANAGER (DCM_SP)
    // Multiplication: 60 MHz * (5 / 3) = 100 MHz
    // =========================================================================
    DCM_SP #(
        .CLKFX_MULTIPLY    (5),             // Multiplier M = 5
        .CLKFX_DIVIDE      (3),             // Divider D = 3
        .CLKIN_PERIOD      (16.666667),     // Input Clock Period (60.000 MHz = 16.666667 ns)
        .CLK_FEEDBACK      ("1X"),          // 1X Feedback on CLK0 for precise DLL deskew
        .STARTUP_WAIT      ("FALSE"),       // Non-blocking bitstream startup
        .PHASE_SHIFT       (0),
        .CLKOUT_PHASE_SHIFT("NONE"),
        .DESKEW_ADJUST     ("SYSTEM_SYNCHRONOUS")
    ) dcm_inst (
        .CLKIN             (clk_60_ibufg),
        .CLKFB             (clk_fb),
        .RST               (dcm_rst),       // Driven by POR counter
        .CLKFX             (clk_fx_raw),    // 100 MHz synthesized output
        .CLK0              (clk_0_raw),     // 60 MHz feedback output
        .LOCKED            (locked),        // Frequency & Phase lock status flag
        .PSEN              (1'b0),
        .PSINCDEC          (1'b0),
        .PSCLK             (1'b0),
        .DSSEN             (1'b0),
        .CLK90             (),
        .CLK180            (),
        .CLK270            (),
        .CLK2X             (),
        .CLK2X180          (),
        .CLKDV             (),
        .CLKFX180          (),
        .STATUS            (),
        .PSDONE            ()
    );

    // Primary 100 MHz Global Clock Tree Driver
    BUFG clk_out_buf (
        .I (clk_fx_raw), 
        .O (clk_100)
    );

    // 60 MHz Feedback Loop Global Clock Tree Driver
    BUFG clk_fb_buf (
        .I (clk_0_raw),  
        .O (clk_fb)
    );

    // =========================================================================
    // MASTER RESET BRIDGE (AASD - Asynchronous Assert, Synchronous Deassert)
    // Guarantees zero metastability on reset release across the 100 MHz domain.
    // =========================================================================
    reg [7:0] sync_reg;
    wire rst_async = ~locked;

    always @(posedge clk_100 or posedge rst_async) begin
        if (rst_async) begin
            sync_reg <= 8'hFF;
        end else begin
            sync_reg <= {sync_reg[6:0], 1'b0};
        end
    end

    assign rst_sync = sync_reg[7];

    // =========================================================================
    // 1 kHz SYSTEM HEARTBEAT TICK GENERATOR
    // Period: 100,000 cycles of 100 MHz = 1.000 ms (1 kHz)
    // =========================================================================
    localparam CNT_LIMIT = `_D_DIV_1kHz_; // 99999 ticks (from define.v)
    localparam CNT_WIDTH = f_clog2(CNT_LIMIT); 
    
    reg [CNT_WIDTH-1:0] tick_cnt;

    always @(posedge clk_100) begin
        if (rst_sync) begin
            tick_cnt  <= {CNT_WIDTH{1'b0}};
            tick_1khz <= 1'b0;
        end else begin
            if (tick_cnt >= CNT_LIMIT) begin
                tick_cnt  <= {CNT_WIDTH{1'b0}};
                tick_1khz <= 1'b1;         // Single clock cycle strobe
            end else begin
                tick_cnt  <= tick_cnt + 1'b1;
                tick_1khz <= 1'b0;
            end
        end
    end

endmodule