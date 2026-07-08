	   `include "define.v"
`timescale 1ns / 1ps

module msb_main (
    // --- System Clock Input ---
    input  wire                         clk_60mHz,       // 60 MHz reference input clock (for DCM)

    // --- STM32 <-> FPGA SPI Interface ---
    input  wire                         spi_stm32_sck,   // P47 (misc1) - SPI Clock (jumper from STM32 PA5)
    input  wire                         spi_stm32_mosi,  // P64 (mosi)  - MOSI data input from STM32
    output wire                         spi_stm32_miso,  // P65 (miso)  - MISO data output to STM32
    
    // --- Chip Selects ---
    input  wire                         spi_stm32_nss_p, // P48 (misc0) - CS for Polling (PB0 STM32)
    
    // --- Flash Lockout (Safe Input) ---
    input  wire                         w25q128_nss,     // P38 (cso)   - Configured as Input to prevent driver contention with STM32!
	
	// --- Global Hardware Interrupt to STM32 (Active Low) ---
    output wire                         fpga_2_stm32_interrupt_N, // Active-Low Global IRQ (goes to EXTI3 on STM32 PC3)
	
    // --- Raw PCB Inputs (To be filtered inside FCS Module) ---   
    output wire [1:0]                   OE_DR,           // Output Enable for DR Sensor Buffer (Active Low)
    input  wire [10:0]                  DR,              // 11-bit Sensor Bus
    input  wire                         HEF,
    input  wire                         APDS,
    input  wire                         HEAT,
    input  wire                         MG,
    input  wire                         GM,
    input  wire                         CC,
    input  wire                         DC,
    input  wire                         SET_R,
    input  wire                         RESET_R,
    input  wire                         BC_EN,
    input  wire                         RL,
    input  wire                         WS,
    input  wire                         PSCC,
    input  wire                         K1,
    input  wire                         BTN_CANNON,
    input  wire                         RST_FILTR,
    input  wire                         UR,
    input  wire                         REM,
    input  wire                         SCF_ON,
    input  wire                         SCF_ON_ADD,

    // --- FCS Control Outputs (Discrete Outputs from STM32 Write Register) ---
    output wire                         ENA_SHOOTING,     // Bit 0: Enable Shooting
    output wire                         GMEE,             // Bit 1: Missile Elevation Output
    output wire                         RANGE_OVER_1280,  // Bit 2: Target Range > 1280m
    output wire                         UOI,              // Bit 3: UOI Signal
    output wire                         INHIBIT_SHOOTING, // Bit 4: Inhibit Shooting Command
    output wire                         WIND_SENSOR_ON,   // Bit 5: Enable Wind Sensor
    output wire                         RFU4,             // Bit 6: Reserved for Future Use 4
    output wire                         RFU5,             // Bit 7: Reserved for Future Use 5
	
    // --- External Test Points (Diagnostic Interface) ---
    output wire [7:5]                   tp,              // TP[7:5] mapped to P79, P80, P81

    // --- User Status LEDs ---
    output wire [2:0]                   led              // Status LEDs
);

    // --- System Reset & Clocks ---
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
    // 1. SPI BUS INTERFACE (STM32 Serial Bus - Polling Bridge)
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
    wire [15:0] data_rd_dev_s [1:`_D_S_NUM_OF_DEV_];

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
                                (addr_dev_s == `_D_S_INT_CTRL_ID_) ? data_rd_dev_s[`_D_S_INT_CTRL_ID_] : 16'h0000;

    // =========================================================================
    // 2. DEBUG REGISTER MODULE (Device ID = 1)
    // =========================================================================
    wire [15:0] debug_out; // Internal debug register bus (misc_reg)

    debug_module #(
        .ADR_WIDTH (`_D_S_CHIP_ADDR_WIDTH_)
    ) u_debug_module (
        .clk      (clk),
        .rst      (rst),
        .cpu_addr (addr_chip_s),
        .cpu_di   (spi_data_to_fpga),
        .cpu_wr   (wr_dev_s[`_D_S_DEBUG_ID_]),
        .cpu_rd   (rd_dev_s[`_D_S_DEBUG_ID_]),
        .cpu_do   (data_rd_dev_s[`_D_S_DEBUG_ID_]),
        .test_out (debug_out) // Exposes misc_reg values internally
    );

    // =========================================================================
    // 3. FIRE CONTROL SYSTEM (FCS) REGISTER MODULE (Device ID = 2)
    // =========================================================================
    
    // Concatenate all 31 raw discrete inputs into one single bus
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
        DR[10:0]           // bits 10:0
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
        .hold        (tick_out_pulse), // ИСПРАВЛЕНО: Добавлено защелкивание по таймеру 100 Гц!
        
        // SPI Bus
        .cpu_addr    (addr_chip_s),
        .cpu_di      (spi_data_to_fpga),
        .cpu_wr      (wr_dev_s[`_D_S_FCS_ID_]),
        .cpu_rd      (rd_dev_s[`_D_S_FCS_ID_]),
        .cpu_do      (data_rd_dev_s[`_D_S_FCS_ID_]),
        
        // Reserved I/O (ИСПРАВЛЕНО: порт fcs_out удален, так как его больше нет в fcs_module)
        .fcs_in      (raw_fcs_in),
        .fcs_control (fcs_control_bus)
    );

    // =========================================================================
    // 4. PROGRAMMABLE TICK TIMER (SPI Controlled)
    // =========================================================================
    //wire tick_out_pulse; // Active high programmable output pulse (1 clk wide)

    // Connection: debug_out[11:4] controls the divider's period
    programmable_tick_divider u_tick_divider (
        .clk      (clk),
        .rst      (rst),
        .tick     (tick_1khz),
        .din      (debug_out[11:4]), // Controlled by STM32
        .tick_out (tick_out_pulse)
    );
	
	
	// =========================================================================
    // 5. INTERRUPT CONTROLLER (Device ID = 3)
    // =========================================================================
    interrupt_controller #( 
        .ADR_WIDTH (`_D_S_CHIP_ADDR_WIDTH_),
        .IRQ_LINES (1) // Используем только 1 прерывание
    ) ic_inst (
        .clk        (clk), 
        .rst        (rst), 
        .cpu_addr   (addr_chip_s),
        .cpu_di     (spi_data_to_fpga),
        .cpu_wr     (wr_dev_s[`_D_S_INT_CTRL_ID_]),
        .cpu_rd     (rd_dev_s[`_D_S_INT_CTRL_ID_]),
        .cpu_do     (data_rd_dev_s[`_D_S_INT_CTRL_ID_]),
        .irq_inputs (tick_out_pulse), // Подключаем только таймер 100 Гц!
        .irq_out_N  (fpga_2_stm32_interrupt_N) 
    );
	
    // =========================================================================
    // 6. MISO BUS ARBITRATION
    // =========================================================================
    assign spi_stm32_miso = (!spi_stm32_nss_p) ? miso_poll : 1'bZ;		 
	
    // =========================================================================
    // 7. STATUS LEDS & TEST POINTS ROUTING
    // =========================================================================
    // Direct static mapping of bottom 3 bits of the debug register to physical LEDs
    assign led = debug_out[2:0];

    // Diagnostics Mapping on TP[7:5]:
    assign tp[5] = tick_out_pulse;             // P79: Output pulse of programmable timer
    assign tp[6] = tick_1khz;                  // P80: Reference 1 kHz tick pulse
    assign tp[7] = rst & w25q128_nss;          // P81: Gated Synchronized global reset status
	
    // Always enable output buffer transceiver for DR sensors (Active-Low)
    assign OE_DR = 2'b00;	  

    // Unpack fcs_control_bus into individual uppercase discrete outputs with hardware inversion
    assign ENA_SHOOTING     = ~fcs_control_bus[0];
    assign GMEE             = ~fcs_control_bus[1];
    assign RANGE_OVER_1280  = ~fcs_control_bus[2];
    assign UOI              = ~fcs_control_bus[3];
    assign INHIBIT_SHOOTING = ~fcs_control_bus[4];
    assign WIND_SENSOR_ON   = ~fcs_control_bus[5];
    assign RFU4             = ~fcs_control_bus[6];
    assign RFU5             = ~fcs_control_bus[7];
	
endmodule