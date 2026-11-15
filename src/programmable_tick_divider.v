`timescale 1ns / 1ps

// =============================================================================
// MODULE: programmable_tick_divider
// 
// DESCRIPTION:
//  Programmable Millisecond Tick Prescaler / Rate Generator.
//  Divides the incoming 1 kHz (1.0 ms) heartbeat strobe by the value in 'din'
//  to generate periodic sample-and-hold latch pulses and timer interrupts.
//
//  Features:
//   - Zero-period lockout: Setting din = 0 disables the timer (output remains 0).
//   - Dynamic protection: Uses '>=' comparison to prevent counter lockup if 'din'
//     is reduced on the fly by the CPU.
//   - Single-cycle output pulse: 'tick_out' is asserted for exactly 1 system clock cycle.
//
//  Target Silicon: Xilinx Spartan-6 (XC6SLX4 / XC6SLX9-TQG144)
//  Toolchain:      Aldec Active-HDL 9.2 / ISE 14.7 / XST
//  All comments in pure ASCII English.
// =============================================================================

module programmable_tick_divider (
    input  wire       clk,        // System clock (100 MHz)
    input  wire       rst,        // Synchronous reset (Active-High)
    input  wire       tick,       // 1 kHz input strobe (1 clock cycle wide from system_clk_rst)
    input  wire [7:0] din,        // Target period in milliseconds (0..255 ms, default: 10 ms = 100 Hz)
    output reg        tick_out    // Generated periodic output pulse (1 clock cycle wide)
);

    // =========================================================================
    // 1. DYNAMIC COUNT LIMIT CALCULATION & ZERO-PERIOD PROTECTION
    // If din = 0, timer is disabled (tick_out remains 0).
    // If din > 0, terminal count limit is (din - 1).
    // =========================================================================
    wire [7:0] count_limit = (din > 8'd0) ? (din - 8'd1) : 8'd0;

    reg [7:0] tick_counter;

    // =========================================================================
    // 2. PRESCALER SEQUENTIAL LOGIC
    // =========================================================================
    always @(posedge clk) begin
        if (rst) begin
            tick_counter <= 8'd0;
            tick_out     <= 1'b0;
        end else begin
            if (tick) begin
                // Use '>=' to safely recover if 'din' is dynamically decreased during counting
                if (tick_counter >= count_limit) begin
                    tick_counter <= 8'd0;
                    // Assert output pulse only if module is enabled (din > 0)
                    tick_out     <= (din > 8'd0); 
                end else begin
                    tick_counter <= tick_counter + 8'd1;
                    tick_out     <= 1'b0;
                end
            end else begin
                // Ensure output pulse lasts strictly for 1 system clock cycle
                tick_out <= 1'b0;
            end
        end
    end

endmodule