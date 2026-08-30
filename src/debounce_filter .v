 
`timescale 1ns / 1ps
// =============================================================================
// Module Name:    debounce_filter
// Description:    Ultra-compact Metastability & Parameterized Debounce Filter.
//                 Uses global 1 kHz (1 ms) tick and dynamic bit-width calculation
//                 (f_clog2) to eliminate synthesis warnings (Xst:1710).
// =============================================================================

module debounce_filter #(
    parameter DEBOUNCE_TICKS = 3  // Filter duration in 1ms ticks (Supported: 1 to 8)
)(
    input  wire  clk,        // System Clock (100 MHz)
    input  wire  rst,        // Synchronous Reset active high
    input  wire  tick_1ms,   // Global 1 kHz clock enable tick from system_clk_rst
    input  wire  noisy_in,   // Raw asynchronous noisy input from PCB pad
    output reg   clean_out   // Synchronized and debounced stable output
);

    // --- Manual function for bit width calculation (ISE 14.7 fix) ---
    function integer f_clog2;
        input integer value;
        begin
            for (f_clog2 = 0; value > 0; f_clog2 = f_clog2 + 1)
                value = value >> 1;
        end
    endfunction

    // =========================================================================
    // 1. METASTABILITY GUARD (2-Stage Flip-Flop Synchronizer)
    // =========================================================================
    reg [1:0] sync_reg;

    always @(posedge clk) begin
        if (rst) begin
            sync_reg <= 2'b00;
        end else begin
            sync_reg <= {sync_reg[0], noisy_in};
        end
    end

    // This signal is now safely synchronized to the 100 MHz clock domain
    wire synced_in = sync_reg[1];

    // =========================================================================
    // 2. PARAMETERIZED DEBOUNCE FILTER LOGIC
    // =========================================================================
    localparam integer CNT_LIMIT = (DEBOUNCE_TICKS > 1) ? (DEBOUNCE_TICKS - 1) : 0;
    localparam integer CALC_BITS = f_clog2(CNT_LIMIT);
    localparam integer CNT_WIDTH = (CALC_BITS > 0) ? CALC_BITS : 1;

    reg [CNT_WIDTH-1:0] debounce_cnt;

    always @(posedge clk) begin
        if (rst) begin
            debounce_cnt <= {CNT_WIDTH{1'b0}};
            clean_out    <= 1'b0;
        end else begin
            if (synced_in == clean_out) begin
                // Input is stable and matches current output state, reset counter
                debounce_cnt <= {CNT_WIDTH{1'b0}};
            end else if (tick_1ms) begin
                // Input differs from output, increment counter on every 1 ms tick
                if (debounce_cnt >= CNT_LIMIT[CNT_WIDTH-1:0]) begin
                    // Target threshold reached -> Latch the new stable state
                    clean_out    <= synced_in;
                    debounce_cnt <= {CNT_WIDTH{1'b0}};
                end else begin
                    debounce_cnt <= debounce_cnt + 1'b1;
                end
            end
        end
    end

endmodule