
`timescale 1ns / 1ps
// =============================================================================
// Module Name:    debounce_filter
// Description:    Ultra-compact Metastability & Debounce Filter (3 ms).
//                 Uses global 1 kHz (1 ms) tick to reduce counter size 
//                 from 19-bit to 2-bit, saving massive FPGA resources.
// =============================================================================

module debounce_filter (
    input  wire  clk,        // System Clock (100 MHz)
    input  wire  rst,        // Synchronous Reset active high
    input  wire  tick_1ms,   // Global 1 kHz clock enable tick from system_clk_rst
    input  wire  noisy_in,   // Raw asynchronous noisy input from PCB pad
    output reg   clean_out   // Synchronized and debounced stable output
);

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
    // 2. DEBOUNCE FILTER LOGIC (3 ms using 2-bit counter)
    // =========================================================================
    reg [1:0] debounce_cnt;

    always @(posedge clk) begin
        if (rst) begin
            debounce_cnt <= 2'd0;
            clean_out    <= 1'b0;
        end else begin
            if (synced_in == clean_out) begin
                // Input is stable and matches current output state, reset counter
                debounce_cnt <= 2'd0;
            end else if (tick_1ms) begin
                // Input differs from output, increment counter on every 1 ms tick
                if (debounce_cnt == 2'd2) begin
                    // 3 ms threshold reached (0 -> 1 -> 2 -> Latch)
                    clean_out    <= synced_in; // Latch the new stable state
                    debounce_cnt <= 2'd0;
                end else begin
                    debounce_cnt <= debounce_cnt + 2'd1;
                end
            end
        end
    end

endmodule