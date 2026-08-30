`include "define.v"
`timescale 1ns / 1ps

// =============================================================================
// MODULE: msb_main (Top-Level with Integrated ChipScope ILA)
// 
// DESCRIPTION:
//  Mode Switching Board (MSB / DIAS.901.02.00.200 E3) FPGA Firmware.
//  Implements:
//   - 31 Discrete Inputs sampling and filtering (FCS Module).
//   - 8 Discrete Relay/Transistor Control Outputs with hardware inversion.
//   - Programmable Tick Timer (100 Hz default Latch).
//   - Single-Register SPI Polling Bridge to STM32 (NSS_P on PB0).
//   - MultiBoot IPROG Reconfiguration Engine (1 MB Image 2).
//   - Dual-Channel RS-485 Hardware Multiplexer (DD19 / DD20).
//   - Dynamic Diagnostic Testpoint Multiplexer (TP5, TP6, TP7).
//   - ChipScope Pro ICON + 64-bit ILA Debug Infrastructure.
//
//  Target Silicon: Xilinx Spartan-6 (XC6SLX4 / XC6SLX9-TQG144)
//  Toolchain:      Aldec Active-HDL 9.2 / ISE 14.7 / XST
//  All comments in pure ASCII English.
// =============================================================================

module msb_main (
    // --- System Clock Input ---
    input  wire                         clk_60mHz,       // 60 MHz reference input (P55)

    // =========================================================================
    // HARDWARE FLASH LOCKOUT (Pin 38 / CSO_B)
    // Driven High in User Mode to lock SPI Flash and prevent MISO bus contention.
    // =========================================================================
    output wire                         w25q128_nss,     // P38 (cso)

    // =========================================================================
    // STM32 <-> FPGA SPI BUS (Control Plane - Polling Bridge)
    // =========================================================================
    input  wire                         spi_stm32_sck,   // P70 (cclk) - SPI Clock from STM32 PA5
    input  wire                         spi_stm32_mosi,  // P64 (mosi) - MOSI from STM32 PA7
    output wire                         spi_stm32_miso,  // P65 (miso) - MISO to STM32 PA6
    input  wire                         spi_stm32_nss_p, // P48 (misc0) - CS Polling from STM32 PB0
	
    // --- Global Hardware Interrupt to STM32 (Active Low) ---
    output wire                         fpga_2_stm32_interrupt_N, // P51 -> STM32 PC3 (EXTI3)
	
    // =========================================================================
    // UART / RS-485 MULTIPLEXER INTERFACE
    // =========================================================================
    input  wire                         usart2_fpga_tx,  // TX from STM32 PA2 (P58)
    output wire                         usart2_fpga_rx,  // RX to STM32 PA3 (P57)

    // --- RS-485 / RS-422 Channel 1 (Transceiver DD19) ---
    output wire                         tx2,             // TX output to DD19 (DI) - P11
    input  wire                         rx2,             // RX input from DD19 (RO) - P10
    output wire                         de2,             // Driver Enable for DD19 - P12

    // --- RS-485 / RS-422 Channel 2 (Transceiver DD20) ---
    output wire                         tx3,             // TX output to DD20 (DI) - P15
    input  wire                         rx3,             // RX input from DD20 (RO) - P14
    output wire                         de3,             // Driver Enable for DD20 - P17
	
    // =========================================================================
    // DISCRETE INPUTS (To be synchronized & filtered in FCS Module)
    // =========================================================================
    output wire [1:0]                   OE_DR,           // Output Enable for DR Sensor Buffers (P126, P124)
    input  wire [10:0]                  DR,              // 11-bit Raw Sensor Bus (P142..P127)
    input  wire                         HEF,             // P123
    input  wire                         APDS,            // P121
    input  wire                         HEAT,            // P120
    input  wire                         MG,              // P119
    input  wire                         GM,              // P118
    input  wire                         CC,              // P117
    input  wire                         DC,              // P116
    input  wire                         SET_R,           // P115
    input  wire                         RESET_R,         // P114
    input  wire                         BC_EN,           // P112
    input  wire                         RL,              // P111
    input  wire                         WS,              // P94
    input  wire                         PSCC,            // P105
    input  wire                         K1,              // P101
    input  wire                         BTN_CANNON,      // P99
    input  wire                         RST_FILTR,       // P98
    input  wire                         UR,              // P93
    input  wire                         REM,             // P88
    input  wire                         SCF_ON,          // P104
    input  wire                         SCF_ON_ADD,      // P102

    // =========================================================================
    // FCS CONTROL OUTPUTS (Discrete Relays & Transistors from STM32 Register)
    // =========================================================================
    output wire                         ENA_SHOOTING,     // Bit 0: Enable Shooting - P1
    output wire                         GMEE,             // Bit 1: Missile Elevation - P6
    output wire                         RANGE_OVER_1280,  // Bit 2: Target Range > 1280m - P2
    output wire                         UOI,              // Bit 3: UOI Signal - P5
    output wire                         INHIBIT_SHOOTING, // Bit 4: Inhibit Shooting - P24
    output wire                         WIND_SENSOR_ON,   // Bit 5: Enable Wind Sensor - P21
    output wire                         RFU4,             // Bit 6: Reserved Output 4 - P22
    output wire                         RFU5,             // Bit 7: Reserved Output 5 - P23
	
    // =========================================================================
    // DIAGNOSTIC TEST POINTS & USER LEDS
    // =========================================================================
    output wire [7:5]                   tp,              // TP[7:5] mapped to P79, P80, P81
    output wire [2:0]                   led              // Status LEDs (P26, P27, P29)
);

    // =========================================================================
    // HARDWARE FLASH LOCKOUT
    // Keep external SPI Flash in disabled/standby mode in User Mode.
    // =========================================================================
    assign w25q128_nss = 1'b1;

    // =========================================================================
    // SYSTEM CLOCK & MASTER RESET
    // =========================================================================
    wire clk;
    wire rst;
    wire tick_1khz;

    system_clk_rst u_sys_clk_rst (
        .ext_clk_60  (clk_60mHz),
        .clk_100     (clk),
        .rst_sync    (rst),
        .tick_1khz   (tick_1khz)
    );

    // =========================================================================
    // 1. SPI BUS INTERFACE (Control Plane - Polling Bridge)
    // =========================================================================
    wire [`_D_S_ADDR_WIDTH_-1:0] spi_addr_raw;
    wire [`_D_DATA_WIDTH_-1:0]   spi_data_to_fpga, spi_data_from_fpga;
    wire                         spi_wr_strobe, spi_rd_active;
    wire                         miso_poll;

    spi_stm32_fpga_bridge spi_bridge_inst (
        .clk       (clk),          
        .rst       (rst),          
        .spi_in    (spi_stm32_mosi),  
        .sck_in    (spi_stm32_sck),   
        .nss_in    (spi_stm32_nss_p),   
        .spi_out   (miso_poll), 
        .addr      (spi_addr_raw),
        .dout      (spi_data_to_fpga), 
        .din       (spi_data_from_fpga),
        .wr_strobe (spi_wr_strobe),
        .rd        (spi_rd_active),      
        .busy      () 
    );
    
    // Address and Device decoding according to define.v
    wire [`_D_S_DEV_ADDR_WIDTH_-1:0]  addr_dev_s  = spi_addr_raw[`_D_S_DEV_HI_:`_D_S_DEV_LO_];
    wire [`_D_S_CHIP_ADDR_WIDTH_-1:0] addr_chip_s = spi_addr_raw[`_D_S_CHIP_HI_:`_D_S_CHIP_LO_];

    wire wr_dev_s [1:`_D_S_NUM_OF_DEV_];
    wire rd_dev_s [1:`_D_S_NUM_OF_DEV_];
    wire [`_D_DATA_WIDTH_-1:0] data_rd_dev_s [1:`_D_S_NUM_OF_DEV_];

    genvar j;
    generate
        for (j=1; j<=`_D_S_NUM_OF_DEV_; j=j+1) begin: SPI_GEN
            assign rd_dev_s[j] = (addr_dev_s == j) ? spi_rd_active : 1'b0;
            assign wr_dev_s[j] = (addr_dev_s == j) ? spi_wr_strobe : 1'b0;
        end
    endgenerate

    // Multi-Device SPI Read multiplexer (Decodes Device 1, Device 2 and Device 3)
    assign spi_data_from_fpga = (addr_dev_s == `_D_S_DEBUG_ID_)    ? data_rd_dev_s[`_D_S_DEBUG_ID_]    :
                                (addr_dev_s == `_D_S_FCS_ID_)      ? data_rd_dev_s[`_D_S_FCS_ID_]      :
                                (addr_dev_s == `_D_S_INT_CTRL_ID_) ? data_rd_dev_s[`_D_S_INT_CTRL_ID_] : 
                                {`_D_DATA_WIDTH_{1'b0}};

    // =========================================================================
    // 2. DEBUG REGISTER MODULE (Device ID = 1)
    // =========================================================================
    wire [`_D_DATA_WIDTH_-1:0] debug_misc_out;
    wire [14:0]                debug_tp_mux_out;
    wire                       reconfig_trigger;

    debug_module #(
        .ADR_WIDTH (`_D_S_CHIP_ADDR_WIDTH_),
        .DATA_WIDTH(`_D_DATA_WIDTH_)
    ) u_debug_module (
        .clk              (clk),
        .rst              (rst),
        .cpu_addr         (addr_chip_s),
        .cpu_di           (spi_data_to_fpga),
        .cpu_wr           (wr_dev_s[`_D_S_DEBUG_ID_]),
        .cpu_rd           (rd_dev_s[`_D_S_DEBUG_ID_]),
        .cpu_do           (data_rd_dev_s[`_D_S_DEBUG_ID_]),
        .misc_out         (debug_misc_out),
        .tp_mux_out       (debug_tp_mux_out),
        .reconfig_trigger (reconfig_trigger)
    );

    // =========================================================================
    // 3. MULTIBOOT / IPROG RECONFIGURATION CONTROLLER
    // Triggers internal FPGA reboot to Image 2 (Flash Offset 0x0010_0000 = 1 MB)
    // =========================================================================
    multiboot_icap #(
        .IMAGE2_ADDR (24'h100000) // 1 MB Flash Offset (0x0010_0000)
    ) u_multiboot (
        .clk     (clk),
        .rst     (rst),
        .trigger (reconfig_trigger)
    );

    // =========================================================================
    // 4. 2-STAGE FLIP-FLOP SYNCHRONIZER FOR DR[10:0] BUS (Metastability Guard)
    // =========================================================================
    reg [10:0] dr_sync1;
    reg [10:0] dr_sync2;

    always @(posedge clk) begin
        if (rst) begin
            dr_sync1 <= 11'd0;
            dr_sync2 <= 11'd0;
        end else begin
            dr_sync1 <= DR;
            dr_sync2 <= dr_sync1;
        end
    end

    wire [10:0] dr_synced = dr_sync2;

    // =========================================================================
    // 5. FIRE CONTROL SYSTEM (FCS) REGISTER MODULE (Device ID = 2)
    // =========================================================================
    wire tick_out_pulse;
    
    // Concatenate all 31 discrete inputs into one bus
    wire [30:0] raw_fcs_in = {
        SCF_ON_ADD,        // bit 30
        SCF_ON,            // bit 29
        REM,               // bit 28
        UR,                // bit 27
        RST_FILTR,         // bit 26
        BTN_CANNON,        // bit 25
        K1,                // bit 24
        PSCC,              // bit 23
        WS,                // bit 22
        RL,                // bit 21
        BC_EN,             // bit 20
        RESET_R,           // bit 19
        SET_R,             // bit 18
        DC,                // bit 17
        CC,                // bit 16
        GM,                // bit 15
        MG,                // bit 14
        HEAT,              // bit 13
        APDS,              // bit 12
        HEF,               // bit 11
        dr_synced[10:0]    // bits 10:0 (2-FF Synchronized)
    };

    wire [7:0] fcs_control_bus;

    fcs_module #(
        .ADR_WIDTH   (`_D_S_CHIP_ADDR_WIDTH_),
        .IN_WIDTH    (31),
        .OUT_WIDTH   (8)
    ) u_fcs_module (
        .clk         (clk),
        .rst         (rst),
        .tick        (tick_1khz),
        .hold        (tick_out_pulse), // 100 Hz Sample-and-Hold Latch
        
        // SPI Bus
        .cpu_addr    (addr_chip_s),
        .cpu_di      (spi_data_to_fpga),
        .cpu_wr      (wr_dev_s[`_D_S_FCS_ID_]),
        .cpu_rd      (rd_dev_s[`_D_S_FCS_ID_]),
        .cpu_do      (data_rd_dev_s[`_D_S_FCS_ID_]),
        
        // Discrete I/O
        .fcs_in      (raw_fcs_in),
        .fcs_control (fcs_control_bus)
    );

    // =========================================================================
    // 6. PROGRAMMABLE TICK TIMER (Controlled by debug_misc_out[11:4])
    // =========================================================================
    programmable_tick_divider u_tick_divider (
        .clk      (clk),
        .rst      (rst),
        .tick     (tick_1khz),
        .din      (debug_misc_out[11:4]), // Controlled by STM32 (Default: 10 ms = 100 Hz)
        .tick_out (tick_out_pulse)
    );
	
    // =========================================================================
    // 7. INTERRUPT CONTROLLER (Device ID = 3)
    // =========================================================================
    interrupt_controller #( 
        .ADR_WIDTH (`_D_S_CHIP_ADDR_WIDTH_),
        .IRQ_LINES (1)
    ) ic_inst (
        .clk        (clk), 
        .rst        (rst), 
        .cpu_addr   (addr_chip_s),
        .cpu_di     (spi_data_to_fpga),
        .cpu_wr     (wr_dev_s[`_D_S_INT_CTRL_ID_]),
        .cpu_rd     (rd_dev_s[`_D_S_INT_CTRL_ID_]),
        .cpu_do     (data_rd_dev_s[`_D_S_INT_CTRL_ID_]),
        .irq_inputs (tick_out_pulse), // 100 Hz timer interrupt
        .irq_out_N  (fpga_2_stm32_interrupt_N) 
    );
	
    // =========================================================================
    // 8. MISO BUS ARBITRATION (Tri-State Output on P65)
    // =========================================================================
    assign spi_stm32_miso = (!spi_stm32_nss_p) ? miso_poll : 1'bZ;		 
	
    // =========================================================================
    // 9. STATUS LEDS (P26, P27, P29)
    // =========================================================================
    assign led = debug_misc_out[2:0];
	
    // =========================================================================
    // 10. UART / RS-485 HARDWARE MULTIPLEXER (Controlled by debug_misc_out[3])
    // =========================================================================
    wire uart_mux_sel = debug_misc_out[3]; // 0: DD19 (Ch 1), 1: DD20 (Ch 2)

    // TX Routing: Active channel gets STM32 TX, inactive channel holds UART IDLE (1'b1)
    assign tx2 = (uart_mux_sel == 1'b0) ? usart2_fpga_tx : 1'b1;
    assign tx3 = (uart_mux_sel == 1'b1) ? usart2_fpga_tx : 1'b1;

    // RX Routing: STM32 RX listens to the selected active channel
    assign usart2_fpga_rx = (uart_mux_sel == 1'b0) ? rx2 : rx3;

    // Driver Enables for RS-485 transceivers (Active-High):
    assign de2 = (uart_mux_sel == 1'b0) ? 1'b1 : 1'b0;
    assign de3 = (uart_mux_sel == 1'b1) ? 1'b1 : 1'b0;
	
    // =========================================================================
    // 11. DIAGNOSTICS TEST POINTS (TP[7:5]) - DYNAMIC MULTIPLEXER
    // =========================================================================
    wire [31:0] tp_signals;

    reg clk_monitor_div2;
    always @(posedge clk or posedge rst) begin
        if (rst) clk_monitor_div2 <= 1'b0;
        else     clk_monitor_div2 <= ~clk_monitor_div2;
    end

    assign tp_signals[0]  = 1'b0;
    assign tp_signals[1]  = clk_monitor_div2;        // 50 MHz monitor clock
    assign tp_signals[2]  = tick_1khz;               // 1 kHz timebase
    assign tp_signals[3]  = tick_out_pulse;          // Programmable timer pulse (100 Hz)
    assign tp_signals[4]  = !fpga_2_stm32_interrupt_N;// Active-High view of IRQ
    assign tp_signals[5]  = spi_stm32_nss_p;
    assign tp_signals[6]  = spi_wr_strobe;
    assign tp_signals[7]  = spi_rd_active;
    assign tp_signals[8]  = usart2_fpga_tx;
    assign tp_signals[9]  = usart2_fpga_rx;
    assign tp_signals[10] = de2;
    assign tp_signals[11] = de3;
    assign tp_signals[12] = ENA_SHOOTING;
    assign tp_signals[13] = GMEE;
    assign tp_signals[14] = RANGE_OVER_1280;
    assign tp_signals[15] = INHIBIT_SHOOTING;
    assign tp_signals[16] = raw_fcs_in[0];           // DR[0]
    assign tp_signals[17] = raw_fcs_in[11];          // HEF
    assign tp_signals[18] = raw_fcs_in[20];          // BC_EN
    assign tp_signals[19] = raw_fcs_in[25];          // BTN_CANNON
    assign tp_signals[20] = rst;
    assign tp_signals[31:21] = 11'd0;

    // Dynamic routing or safe default (TP5=100Hz, TP6=1kHz, TP7=Reset/IRQ)
    assign tp[5] = (debug_tp_mux_out[4:0]   != 5'd0) ? tp_signals[debug_tp_mux_out[4:0]]   : tick_out_pulse;
    assign tp[6] = (debug_tp_mux_out[9:5]   != 5'd0) ? tp_signals[debug_tp_mux_out[9:5]]   : tick_1khz;
    assign tp[7] = (debug_tp_mux_out[14:10] != 5'd0) ? tp_signals[debug_tp_mux_out[14:10]] : rst;

    // =========================================================================
    // 12. DISCRETE OUTPUTS & BUFFER CONTROL
    // =========================================================================
    // Always enable output buffer transceivers for DR sensors (Active-Low)
    assign OE_DR = 2'b00;	  

    // Unpack fcs_control_bus into discrete outputs with hardware inversion
    assign ENA_SHOOTING     = ~fcs_control_bus[0];
    assign GMEE             = ~fcs_control_bus[1];
    assign RANGE_OVER_1280  = ~fcs_control_bus[2];
    assign UOI              = ~fcs_control_bus[3];
    assign INHIBIT_SHOOTING = ~fcs_control_bus[4];
    assign WIND_SENSOR_ON   = ~fcs_control_bus[5];
    assign RFU4             = ~fcs_control_bus[6];
    assign RFU5             = ~fcs_control_bus[7];

   // =========================================================================
    // 13. CHIPSCOPE PRO LOGIC ANALYZER (ICON + ILA) - CONDITIONAL COMPILATION
    // =========================================================================
`ifdef USE_CHIPSCOPE
    wire [35:0] chipscope_control;
    wire [63:0] ila_trig;

    // ICON Core: JTAG Controller Bridge
    chipscope_icon u_icon (
        .CONTROL0 (chipscope_control)
    );

    // ILA Core: 64-bit Logic Analyzer clocked by 100 MHz System Clock
    chipscope_ila u_ila (
        .CONTROL (chipscope_control),
        .CLK     (clk),
        .TRIG0   (ila_trig)
    );

    // Signal Mapping Table for ChipScope Pro Analyzer
    assign ila_trig[0]     = spi_stm32_nss_p;
    assign ila_trig[1]     = spi_stm32_sck;
    assign ila_trig[2]     = spi_stm32_mosi;
    assign ila_trig[3]     = spi_stm32_miso;
    assign ila_trig[4]     = spi_wr_strobe;
    assign ila_trig[5]     = spi_rd_active;
    assign ila_trig[15:6]  = spi_addr_raw[9:0];
    assign ila_trig[31:16] = spi_data_to_fpga[15:0];
    assign ila_trig[47:32] = spi_data_from_fpga[15:0];
    assign ila_trig[48]    = tick_1khz;
    assign ila_trig[49]    = tick_out_pulse;
    assign ila_trig[50]    = fpga_2_stm32_interrupt_N;
    assign ila_trig[51]    = rst;
    assign ila_trig[52]    = reconfig_trigger;
    assign ila_trig[53]    = uart_mux_sel;
    assign ila_trig[54]    = usart2_fpga_tx;
    assign ila_trig[55]    = usart2_fpga_rx;
    assign ila_trig[63:56] = fcs_control_bus[7:0];
`endif

endmodule