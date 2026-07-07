`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: spi_stm32_multi_word_bridge
// Description: Multi-word burst SPI bridge for STM32.
//              Optimized with Strict NSS Edge Detection.
//////////////////////////////////////////////////////////////////////////////////

module spi_stm32_multi_word_bridge #(
    parameter NUM_WORDS = 16 
)(
    input  wire                               clk,       
    input  wire                               rst,       
    
    // --- SPI Interface ---
    input  wire                               spi_in,    
    input  wire                               sck_in,    
    input  wire                               nss_in,    
    output reg                                spi_out,   
    
    // --- Flat Local Bus ---
    output reg  [(16*NUM_WORDS)-1:0]          dout,      
    input  wire [(16*NUM_WORDS)-1:0]          din,       
    output reg                                wr_strobe, 
    output reg                                rd_strobe, 
    output wire                               busy       
);

    // --- Synchronizers & Edge Detectors ---
    reg [2:0] sck_sync;
    reg [2:0] nss_sync; // Increased to 3 bits for higher reliability
    reg [1:0] mosi_sync;

    always @(posedge clk) begin
        if (rst) begin
            sck_sync  <= 3'b000;
            nss_sync  <= 3'b111;
            mosi_sync <= 2'b00;
        end else begin
            sck_sync  <= {sck_sync[1:0], sck_in};
            nss_sync  <= {nss_sync[1:0], nss_in};
            mosi_sync <= {mosi_sync[0],  spi_in};
        end
    end

    wire sck_rising  = (sck_sync[1:0] == 2'b01);
    wire sck_falling = (sck_sync[1:0] == 2'b10);
    
    wire nss_active  = ~nss_sync[1];
    wire nss_falling = (nss_sync[1:0] == 2'b10); // CS Low (Start of Transfer)
    wire nss_rising  = (nss_sync[1:0] == 2'b01); // CS High (End of Transfer)

    assign busy = nss_active;

    // --- Control Logic ---
    reg [8:0]                  bit_cnt;
    reg [15:0]                 rx_word_shifter;
    reg [(16*NUM_WORDS)-1:0]   rx_burst_buffer;
    reg [(16*NUM_WORDS)-1:0]   tx_shifter_burst;
    reg [4:0]                  burst_len;
    reg                        is_write;
    reg                        is_read;
    reg [4:0]                  word_idx;

    integer w;

    always @(posedge clk) begin
        if (rst) begin
            bit_cnt          <= 9'd0;
            rx_word_shifter  <= 16'd0;
            rx_burst_buffer  <= 0;
            tx_shifter_burst <= 0;
            dout             <= 0;
            wr_strobe        <= 1'b0;
            rd_strobe        <= 1'b0;
            is_write         <= 1'b0;
            is_read          <= 1'b0;
            burst_len        <= 5'd0;
            word_idx         <= 5'd0;
            spi_out          <= 1'b0;
        end else begin
            // Default strobe values (guarantees exactly 1 clock cycle pulse)
            wr_strobe <= 1'b0;
            rd_strobe <= 1'b0;

            // --- 1. START TRANSACTION (Reset internal state) ---
            if (nss_falling) begin
                bit_cnt         <= 9'd0;
                rx_word_shifter <= 16'd0;
                word_idx        <= 5'd0;
                spi_out         <= 1'b0;
            end

            // --- 2. ACTIVE TRANSFER (Shifting data) ---
            if (nss_active) begin
                
                // MOSI: Sample on Rising Edge
                if (sck_rising) begin
                    if (bit_cnt < (16 + (NUM_WORDS << 4))) bit_cnt <= bit_cnt + 1'b1;
                    
                    rx_word_shifter <= {rx_word_shifter[14:0], mosi_sync[1]};

                    if (bit_cnt == 9'd15) begin
                        is_write  <= rx_word_shifter[14];
                        is_read   <= (!rx_word_shifter[14]) || rx_word_shifter[13];
                        burst_len <= (rx_word_shifter[3:0] == 4'd0 && mosi_sync[1] == 1'b0) ? 5'd1 : {rx_word_shifter[3:0], mosi_sync[1]};
                        
                        if ((!rx_word_shifter[14]) || rx_word_shifter[13]) rd_strobe <= 1'b1;
                    end
                    else if (bit_cnt > 9'd15 && (bit_cnt[3:0] == 4'd15) && word_idx < NUM_WORDS) begin
                        rx_burst_buffer[word_idx*16 +: 16] <= {rx_word_shifter[14:0], mosi_sync[1]};
                        word_idx <= word_idx + 1'b1;
                    end
                end

                // Latch requested data from FPGA internal bus
                if (rd_strobe) begin
                    for (w = 0; w < NUM_WORDS; w = w + 1)
                        tx_shifter_burst[(NUM_WORDS-w)*16 - 1 -: 16] <= din[w*16 +: 16];
                end

                // MISO: Shift out on Falling Edge (Setup for STM32 Rising Edge sample)
                if (sck_falling && bit_cnt >= 9'd16 && is_read) begin
                    spi_out          <= tx_shifter_burst[(16*NUM_WORDS)-1];
                    tx_shifter_burst <= {tx_shifter_burst[(16*NUM_WORDS)-2 : 0], 1'b0};
                end
            end

            // --- 3. END OF TRANSACTION (Atomic Write Execution) ---
            if (nss_rising) begin
                // Check if it was a Write and if we received the expected amount of bits
                if (is_write && (bit_cnt >= {4'd0, burst_len, 4'd0} + 9'd16)) begin
                    dout      <= rx_burst_buffer;
                    wr_strobe <= 1'b1; // This strobe will now safely fire exactly once
                end
                spi_out <= 1'b0; // Drive MISO to 0 for bus safety when idle
            end
            
        end
    end
endmodule