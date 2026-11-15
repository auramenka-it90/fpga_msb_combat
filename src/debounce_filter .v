`timescale 1ns / 1ps

// =============================================================================
// MODULE: debounce_filter
//
// DESCRIPTION:
//  Metastability guard (2-stage synchronizer) and parameterized digital
//  debounce filter with true zero-overhead hardware bypass mode.
//  (Ported from Sosna project; shared f_clog2 via clog2_func.vh)
//
//  Modes:
//   - DEBOUNCE_TICKS == 0     : Hardware Bypass (2-FF sync only, 20ns latency)
//   - DEBOUNCE_TICKS == 1..31 : Active filter (Threshold: 1 to 31 milliseconds)
//
//  Target Silicon: Xilinx Spartan-6 (XC6SLX4 / XC6SLX9-TQG144)
//  Toolchain:      ISE 14.7 / Active-HDL 9.2 (Verilog-2001)
//  All comments in ASCII English.
// =============================================================================

module debounce_filter #(
    parameter DEBOUNCE_TICKS = 3  // Filter threshold in ms (0 = Bypass, 1..31 = Filter)
)(
    input  wire  clk,        // Master System Clock (100 MHz)
    input  wire  rst,        // Synchronous Reset (Active High)
    input  wire  tick_1ms,   // 1 kHz (1 ms) enable strobe from system_clk_rst
    input  wire  noisy_in,   // Asynchronous input from PCB pad
    output reg   clean_out   // Synchronized, debounced stable output
);

    // Constant bit-width calculation (shared implementation)
`include "clog2_func.vh"

    // 1. Metastability Guard: Two-stage Flip-Flop Synchronizer (Always Active)
    reg [1:0] sync_reg;

    always @(posedge clk) begin
        if (rst) begin
            sync_reg <= 2'b00;
        end else begin
            sync_reg <= {sync_reg[0], noisy_in};
        end
    end

    wire synced_in = sync_reg[1];

    // 2. Hardware Mode Selection via Compile-Time Generate
    generate
        if (DEBOUNCE_TICKS == 0) begin : GEN_BYPASS
            // TRUE ZERO-OVERHEAD BYPASS MODE (Latency = 2 clock cycles / 20 ns)
            always @(posedge clk) begin
                if (rst) begin
                    clean_out <= 1'b0;
                end else begin
                    clean_out <= synced_in;
                end
            end
        end else begin : GEN_DEBOUNCE
            // ACTIVE DEBOUNCE FILTER (Threshold = 1 to 31 milliseconds)
            localparam integer CNT_LIMIT = DEBOUNCE_TICKS - 1;
            localparam integer CALC_BITS = f_clog2(CNT_LIMIT);
            localparam integer CNT_WIDTH = (CALC_BITS > 0) ? CALC_BITS : 1;

            reg [CNT_WIDTH-1:0] cnt;

            always @(posedge clk) begin
                if (rst) begin
                    cnt       <= {CNT_WIDTH{1'b0}};
                    clean_out <= 1'b0;
                end else begin
                    if (synced_in == clean_out) begin
                        // Signal is stable: reset filter counter
                        cnt <= {CNT_WIDTH{1'b0}};
                    end else if (tick_1ms) begin
                        // Signal changed: accumulate 1 ms ticks
                        if (cnt >= CNT_LIMIT[CNT_WIDTH-1:0]) begin
                            clean_out <= synced_in;
                            cnt       <= {CNT_WIDTH{1'b0}};
                        end else begin
                            cnt <= cnt + 1'b1;
                        end
                    end
                end
            end
        end
    endgenerate

endmodule
