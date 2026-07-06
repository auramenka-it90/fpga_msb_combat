`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: spi_stm32_fpga_bridge
// Description: High-speed SPI Slave Bridge for STM32 (Mode 0).
//              Optimized for ZERO-LATENCY combinatorial read buses.
//              Registers optimized to prevent synthesis warnings.
//////////////////////////////////////////////////////////////////////////////////

module spi_stm32_fpga_bridge (
    input  wire        clk,       // System clock (100 MHz)
    input  wire        rst,       // Synchronous reset active high
    
    // --- SPI Interface (Mode 0) ---
    input  wire        spi_in,    // MOSI
    input  wire        sck_in,    // SCK 
    input  wire        nss_in,    // CS (Active Low)
    output reg         spi_out,   // MISO
    
    // --- Internal Local Bus ---
    output reg  [9:0]  addr,      // 10-bit Address
    output reg  [15:0] dout,      // Data to FPGA (Write)
    input  wire [15:0] din,       // Data from FPGA (Read)
    output reg         wr_strobe, // Write pulse (1 clk wide)
    output reg         rd,        // Read enable pulse (1 clk wide)
    output wire        busy       // Bridge status
);

    // --- Synchronizers & Edge Detectors ---
    reg [2:0] sck_sync;
    reg [2:0] nss_sync;
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

    assign busy = nss_active;

    // --- Main SPI State Machine ---
    reg [5:0]  bit_cnt;
    reg [15:0] rx_shifter; // Reduced to 16 bits to remove synthesis warnings
    reg [15:0] tx_shifter;
    reg        is_write;

    always @(posedge clk) begin
        if (rst) begin
            bit_cnt    <= 6'd0;
            rx_shifter <= 16'd0;
            tx_shifter <= 16'd0;
            addr       <= 10'd0;
            dout       <= 16'd0;
            wr_strobe  <= 1'b0;
            rd         <= 1'b0;
            is_write   <= 1'b0;
            spi_out    <= 1'b0;
        end else if (!nss_active) begin
            bit_cnt    <= 6'd0;
            rx_shifter <= 16'd0;
            wr_strobe  <= 1'b0;
            rd         <= 1'b0;
            spi_out    <= 1'b0; 
        end else begin
            // Default strobe values
            wr_strobe <= 1'b0;
            rd        <= 1'b0;

            // --- 1. ZERO-LATENCY COMBINATORIAL READ LATCH ---
            if (rd) begin
                tx_shifter <= din;
            end

            // --- 2. MOSI PATH (Sampling on SCK Rising Edge) ---
            if (sck_rising) begin
                bit_cnt    <= bit_cnt + 1'b1;
                // Shift 16 bits to prevent unused node warnings
                rx_shifter <= {rx_shifter[14:0], mosi_sync[1]};
                
                // HEADER COMPLETE: Received 16 bits (Word 1: Command + Address)
                if (bit_cnt == 6'd15) begin
                    is_write <= rx_shifter[14]; // rx_shifter[15] was R/W bit before shift
                    addr     <= {rx_shifter[8:0], mosi_sync[1]};
                    
                    if (rx_shifter[14] == 1'b0) begin
                        rd <= 1'b1; // Trigger read pulse
                    end
                end

                // WRITE EXECUTION: Received 32 bits total (Word 2: Write Data)
                if (bit_cnt == 6'd31) begin
                    if (is_write) begin
                        dout      <= {rx_shifter[14:0], mosi_sync[1]};
                        wr_strobe <= 1'b1;
                    end
                end
            end

            // --- 3. MISO PATH (Shifting on SCK Falling Edge) ---
            if (sck_falling) begin
                if (bit_cnt == 6'd16) begin
                    if (!is_write) begin
                        spi_out    <= tx_shifter[15];
                        tx_shifter <= {tx_shifter[14:0], 1'b0};
                    end else begin
                        spi_out    <= 1'b0;
                    end
                end else if (bit_cnt > 6'd16) begin
                    if (!is_write) begin
                        spi_out    <= tx_shifter[15];
                        tx_shifter <= {tx_shifter[14:0], 1'b0};
                    end else begin
                        spi_out    <= 1'b0;
                    end
                end
            end
        end
    end

endmodule