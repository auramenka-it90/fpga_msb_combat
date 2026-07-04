`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Senior Embedded Systems & FPGA Architect
// 
// Module Name: spi_stm32_multi_word_bridge
// Description: Multi-word burst SPI bridge for STM32 (Mode 0, max 20 MHz).
//              Supports up to 16-word transfers. Fully synchronous to 100 MHz.
//              Implements zero-latency look-ahead address and length decoding.
//
//////////////////////////////////////////////////////////////////////////////////

module spi_stm32_multi_word_bridge #(
    parameter NUM_WORDS = 16 // Support up to 16 words (parameterizable)
)(
    input  wire                         clk,       // System clock (100 MHz)
    input  wire                         rst,       // Synchronous reset active high
    
    // --- SPI Interface (Mode 0) ---
    input  wire                         spi_in,    // MOSI
    input  wire                         sck_in,    // SCK
    input  wire                         nss_in,    // CS (Active Low)
    output reg                          spi_out,   // MISO
    
    // --- Flat Local Bus Interfaces ---
    // Word 0 is mapped to din[15:0], Word 1 to din[31:16], etc.
    output reg  [(16*NUM_WORDS)-1:0]    dout,      // Flat write bus to FPGA
    input  wire [(16*NUM_WORDS)-1:0]    din,       // Flat read bus from FPGA
    output reg                          wr_strobe, // Write pulse (1 clk wide)
    output reg                          rd_strobe, // Read pulse (1 clk wide)
    output wire                         busy       // Bridge status active high
);

    // --- Input Synchronizers ---
    reg [2:0] sck_sync;
    reg [1:0] nss_sync;
    reg [1:0] mosi_sync;

    always @(posedge clk) begin
        if (rst) begin
            sck_sync  <= 3'b000;
            nss_sync  <= 2'b11;
            mosi_sync <= 2'b00;
        end else begin
            sck_sync  <= {sck_sync[1:0], sck_in};
            nss_sync  <= {nss_sync[0],   nss_in};
            mosi_sync <= {mosi_sync[0],  spi_in};
        end
    end

    // --- Edge Detectors ---
    wire sck_rising  = (sck_sync[1:0] == 2'b01);
    wire sck_falling = (sck_sync[1:0] == 2'b10);
    wire nss_active  = ~nss_sync[1];

    assign busy = nss_active;

    // --- Control and Shift Registers ---
    reg [8:0]                  bit_cnt;          // Up to 272 bits (16 + 16*16)
    reg [(16*NUM_WORDS)-1:0]   rx_shifter_burst; // MOSI shift register
    reg [(16*NUM_WORDS)-1:0]   tx_shifter_burst; // MISO shift register
    reg [4:0]                  burst_len;        // Number of words to transfer
    reg                        is_write;
    reg                        is_read;

    // Local variable for loops (Verilog-2001 compatible)
    integer w;

    // --- Main State Machine & Data Path ---
    always @(posedge clk) begin
        if (rst) begin
            bit_cnt          <= 9'd0;
            rx_shifter_burst <= 0;
            tx_shifter_burst <= 0;
            dout             <= 0;
            wr_strobe        <= 1'b0;
            rd_strobe        <= 1'b0;
            is_write         <= 1'b0;
            is_read          <= 1'b0;
            burst_len        <= 5'd0;
            spi_out          <= 1'b0;
        end else if (!nss_active) begin
            bit_cnt          <= 9'd0;
            rx_shifter_burst <= 0;
            tx_shifter_burst <= 0;
            wr_strobe        <= 1'b0;
            rd_strobe        <= 1'b0;
            is_write         <= 1'b0;
            is_read          <= 1'b0;
            burst_len        <= 5'd0;
            spi_out          <= 1'b0;
        end else begin
            wr_strobe <= 1'b0;
            rd_strobe <= 1'b0;

            // --- MOSI Path (Sampling on SCK Rising Edge) ---
            if (sck_rising) begin
                // Safety guard to prevent bit counter overflow
                if (bit_cnt < (16 + (NUM_WORDS << 4))) begin
                    bit_cnt          <= bit_cnt + 1'b1;
                    rx_shifter_burst <= {rx_shifter_burst[(16*NUM_WORDS)-2 : 0], mosi_sync[1]};
                end

                // --- Header Word Fully Received (16th rising edge) ---
                if (bit_cnt == 9'd15) begin
                    // Decode Command Type (b15..b14)
                    // 2'b00 = Read, 2'b10 = Write, 2'b11 = Read/Write
                    is_write  <= rx_shifter_burst[14]; // Bit 15 is at [14]
                    is_read   <= (!rx_shifter_burst[14]) || rx_shifter_burst[13]; // Bit 14 is at [13]

                    // Decode and sanitize Burst Length (b4..b0)
                    if (rx_shifter_burst[3:0] == 4'd0 && mosi_sync[1] == 1'b0) begin
                        burst_len <= 5'd1; // Default to 1 word if length is 0
                    end else begin
                        burst_len <= {rx_shifter_burst[3:0], mosi_sync[1]};
                    end

                    // Trigger read strobe immediately if read operation is required
                    if ((!rx_shifter_burst[14]) || rx_shifter_burst[13]) begin
                        rd_strobe <= 1'b1;
                    end
                end
            end

            // --- Latch Read Data from FPGA ---
            // Triggered on the clock cycle following 'rd_strobe'
            if (rd_strobe) begin
                // Map flat din [Word0, Word1, ... WordN] into tx_shifter_burst
                // Word 0 is loaded at the MSB of shifter to be sent first
                for (w = 0; w < NUM_WORDS; w = w + 1) begin
                    tx_shifter_burst[(NUM_WORDS-w)*16 - 1 -: 16] <= din[w*16 +: 16];
                end
            end

            // --- MISO Path (Shifting out on SCK Falling Edge) ---
            if (sck_falling) begin
                if (bit_cnt >= 9'd16) begin
                    if (is_read) begin
                        spi_out          <= tx_shifter_burst[(16*NUM_WORDS)-1];
                        tx_shifter_burst <= {tx_shifter_burst[(16*NUM_WORDS)-2 : 0], 1'b0};
                    end else begin
                        spi_out          <= 1'b0;
                    end
                end
            end

            // --- Write Execution (End of whole multi-word transfer) ---
            // Triggered 1 cycle after the last bit is shifted in
            if (nss_active && (bit_cnt == {4'd0, burst_len, 4'd0} + 9'd16) && !sck_rising && (wr_strobe == 1'b0)) begin
                if (is_write) begin
                    // Demultiplex rx_shifter_burst back to flat dout bus
                    for (w = 0; w < NUM_WORDS; w = w + 1) begin
                        if (w < burst_len) begin
                            dout[w*16 +: 16] <= rx_shifter_burst[(burst_len-w)*16 - 1 -: 16];
                        end
                    end
                    wr_strobe <= 1'b1;
                end
            end
        end
    end

endmodule