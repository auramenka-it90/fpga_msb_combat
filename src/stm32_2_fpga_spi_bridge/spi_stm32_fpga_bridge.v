`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Senior Embedded Systems & FPGA Architect
// 
// Create Date: 
// Design Name: spi_stm32_fpga_bridge
// Module Name: spi_stm32_fpga_bridge
// Target Devices: Xilinx Spartan-6
// Tool versions: ISE 14.7 / Vivado
// Description: High-speed SPI Slave Bridge for STM32 (20 MHz, Mode 0).
//              Runs entirely in 100 MHz system clock domain.
//              Uses look-ahead prefetching to meet MISO setup times.
//
//////////////////////////////////////////////////////////////////////////////////

module spi_stm32_fpga_bridge (
    input  wire        clk,       // System clock (100 MHz)
    input  wire        rst,       // Synchronous reset active high
    
    // --- SPI Interface (Mode 0) ---
    input  wire        spi_in,    // MOSI
    input  wire        sck_in,    // SCK (Max 20 MHz)
    input  wire        nss_in,    // CS (Active Low)
    output reg         spi_out,   // MISO (Push-Pull, active-driven)
    
    // --- Internal Local Bus ---
    output reg  [9:0]  addr,      // 10-bit Address
    output reg  [15:0] dout,      // Data to FPGA (Write)
    input  wire [15:0] din,       // Data from FPGA (Read)
    output reg         wr_strobe, // Write pulse (1 clk wide)
    output reg         rd,        // Read enable pulse (1 clk wide)
    output wire        busy       // Bridge status (1 = CS active)
);

    // --- Input Synchronization Stages ---
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

    // --- Edge Detection ---
    wire sck_rising  = (sck_sync[1:0] == 2'b01);
    wire sck_falling = (sck_sync[1:0] == 2'b10);
    wire nss_active  = ~nss_sync[1]; // Active Low

    // --- Shift Registers & Bit Counter ---
    reg [5:0]  bit_cnt;
    reg [31:0] rx_shifter;
    reg [15:0] tx_shifter;
    reg        is_write;

    // Output "busy" state directly linked to synchronized CS
    assign busy = nss_active;

    // --- Main SPI State Machine & Data Path ---
    always @(posedge clk) begin
        if (rst) begin
            bit_cnt    <= 6'd0;
            rx_shifter <= 32'd0;
            tx_shifter <= 16'd0;
            addr       <= 10'd0;
            dout       <= 16'd0;
            wr_strobe  <= 1'b0;
            rd         <= 1'b0;
            is_write   <= 1'b0;
            spi_out    <= 1'b0;
        end else if (!nss_active) begin
            // Reset state machine on CS deassertion
            bit_cnt    <= 6'd0;
            rx_shifter <= 32'd0;
            tx_shifter <= 16'd0;
            wr_strobe  <= 1'b0;
            rd         <= 1'b0;
            is_write   <= 1'b0;
            spi_out    <= 1'b0; // Drive low when CS is inactive (no Z-state)
        end else begin
            // Default strobe pulses to zero
            wr_strobe <= 1'b0;
            rd        <= 1'b0;

            // --- MOSI Path (Sampling on SCK Rising Edge) ---
            if (sck_rising) begin
                bit_cnt    <= bit_cnt + 1'b1;
                rx_shifter <= {rx_shifter[30:0], mosi_sync[1]};
                
                // Pipeline T0: 16th rising edge (index 15) received
                if (bit_cnt == 6'd15) begin
                    // At next cycle (T1), rx_shifter[15:0] will contain the first 16-bit word.
                    // Word 1: [Bit 15 = R/W] [Bits 14:0 = Address]
                end
            end

            // --- Control and Address Decoders (Cycle-accurate Pipeline) ---
            // Pipeline T1: Triggered 1 cycle after the 16th rising edge
            if (nss_active && (bit_cnt == 6'd16) && !sck_rising && (rd == 1'b0) && (tx_shifter == 16'd0)) begin
                // Extract R/W command and Address (mapped to bottom 10 bits)
                is_write <= rx_shifter[15];
                addr     <= rx_shifter[9:0];
                
                if (rx_shifter[15] == 1'b0) begin
                    rd <= 1'b1; // Generate 1-clock pulse for external read
                end
            end

            // Pipeline T2: Latch read data from FPGA local bus to TX shifter
            if (nss_active && (bit_cnt == 6'd16) && rd) begin
                // din must be stable at this cycle!
                tx_shifter <= din;
            end

            // --- MISO Path (Shifting on SCK Falling Edge) ---
            if (sck_falling) begin
                if (bit_cnt == 6'd16) begin
                    if (!is_write) begin
                        // Load MSB of read data immediately
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

            // --- Write Execution (End of 32-bit transaction) ---
            if (sck_rising && (bit_cnt == 6'd31)) begin
                if (is_write) begin
                    // Next cycle rx_shifter will hold [Word 1][Word 2]
                    // We extract Word 2 (Write Data)
                    dout      <= {rx_shifter[14:0], mosi_sync[1]};
                    wr_strobe <= 1'b1;
                end
            end
        end
    end

endmodule