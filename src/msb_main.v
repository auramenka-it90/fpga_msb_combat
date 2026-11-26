`include "define.v"
`timescale 1ns / 1ps

// =============================================================================
// MODULE: msb_main (Top-Level)
// 
// DESCRIPTION:
//  Mode Switching Board (MSB / DIAS.901.02.00.200 E3) FPGA Firmware.
//  Implements:
//   - 31 Discrete Inputs sampling and filtering (FCS Module).
//   - 8 Discrete Relay/Transistor Control Outputs with hardware inversion.
//   - Programmable Tick Timer (100 Hz default Latch).
//   - Single-Register SPI Polling Bridge to STM32 (NSS_P on PB0).
//   - FPGA reset/reconfiguration: external only, via PROG_B (STM32 MR_PROG).
//   - Dual-Channel RS-485 Hardware Multiplexer (DD19 / DD20).
//   - Dynamic Diagnostic Testpoint Multiplexer (TP5, TP6, TP7).
//   - ChipScope Pro ICON JTAG bridge (optional, USE_CHIPSCOPE; ILA removed).
//
//  Target Silicon: Xilinx Spartan-6 (XC6SLX9-TQG144)
//  Toolchain:      Aldec Active-HDL 9.2 / ISE 14.7 / XST
//  All comments in pure ASCII English.
// =============================================================================

module msb_main (
    // --- System Clock Input ---
    input  wire                         clk_60mHz,       // 60 MHz reference input (P55)

    // =========================================================================
    // HARDWARE FLASH LOCKOUT (Pin 38 / CSO_B) + STATUS PIN TIE-OFFS
    // CSO driven High in User Mode to lock SPI Flash and prevent MISO bus
    // contention. /HOLD (P61, shared with Flash DQ2) and /WP (P62, shared
    // with Flash DQ3) are driven High to keep the flash fully responsive.
    // =========================================================================
    output wire                         w25q128_nss,     // P38 (cso)
    output wire                         w25q128_hold_n,  // P61 (d2/miso3) -> Flash /HOLD
    output wire                         w25q128_wp_n,    // P62 (d1/miso2) -> Flash /WP

    // =========================================================================
    // STM32 <-> FPGA SPI BUS (Control Plane - Polling Bridge)														  																									 
    // =========================================================================
    input  wire                         spi_stm32_sck,   // P70 (cclk) - SPI Clock from STM32 PA5
    input  wire                         spi_stm32_mosi,  // P64 (mosi) - MOSI from STM32 PA7
    output wire                         spi_stm32_miso,  // P65 (miso) - MISO to STM32 PA6
    input  wire                         spi_stm32_nss_p, // P48 (misc0) - CS Polling from STM32 PB0
	
    // --- Global Hardware Interrupt to STM32 (Active Low) ---
    output wire                         fpga_2_stm32_interrupt_N, // P51 -> STM32 PB10 (EXTI15_10, polled)
	
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
    input  wire                         UR,              // P92
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
    output wire                         BC_ON,            // Bit 6: Reserved Output 4 - P22
    output wire                         BC_READY,         // Bit 7: Reserved Output 5 - P23
	
    // =========================================================================
    // DIAGNOSTIC TEST POINTS & USER LEDS
    // =========================================================================
    output wire [7:5]                   tp,              // TP5=P81, TP6=P80, TP7=P79
    output wire [2:0]                   led              // Status LEDs (P26, P27, P29)
);

    // =========================================================================
    // HARDWARE FLASH LOCKOUT
    // Keep external SPI Flash in disabled/standby mode in User Mode.
    // =========================================================================
    assign w25q128_nss    = 1'b1;
    assign w25q128_hold_n = 1'b1;
    assign w25q128_wp_n   = 1'b1;

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
        .tp_mux_out       (debug_tp_mux_out)
    );

    // =========================================================================
    // 3. (Multiboot/IPROG engine deleted. FPGA reconfiguration is external,
    //    via PROG_B pulse from STM32 MR_PROG pin.)
    // =========================================================================

    // =========================================================================
    // 4. DR[10:0] SYNCHRONIZATION
    //    Performed inside fcs_module by debounce_filter instances in bypass
    //    mode (DEBOUNCE_TICKS = 0 -> 2-FF metastability guard only).
    // =========================================================================

    // =========================================================================
    // 5. FIRE CONTROL SYSTEM (FCS) REGISTER MODULE (Device ID = 2)
    // =========================================================================
    wire tick_out_pulse;
    wire fcs_wd_expired;
    
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
        DR                     // bits 10:0 (synchronized inside fcs_module)
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
        .fcs_control (fcs_control_bus),
        .dbg_wd_expired (fcs_wd_expired)
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
    //     Each TP routes one of 32 signals selected by debug_module tp_mux
    //     register (5-bit index per TP). Index 0 = logic '0' (GND reference).
    // =========================================================================
    wire [31:0] tp_signals;

    reg clk_monitor_div2;
    always @(posedge clk or posedge rst) begin
        if (rst) clk_monitor_div2 <= 1'b0;
        else     clk_monitor_div2 <= ~clk_monitor_div2;
    end

    // --- Clock / reset health (1..4) ---
    assign tp_signals[0]  = 1'b0;                          // GND reference
    assign tp_signals[1]  = clk_monitor_div2;              // DCM output /2 (clk alive)
    assign tp_signals[2]  = tick_1khz;                     // 1 ms timebase
    assign tp_signals[3]  = tick_out_pulse;                // Programmable timer pulse (100 Hz)
    assign tp_signals[4]  = rst;                           // Synchronous reset

    // --- SPI control-plane bus (5..10) ---
    assign tp_signals[5]  = spi_stm32_sck;                 // SPI clock from STM32
    assign tp_signals[6]  = spi_stm32_mosi;                // SPI data from STM32
    assign tp_signals[7]  = miso_poll;                     // Internal MISO before tri-state
    assign tp_signals[8]  = spi_stm32_nss_p;               // Polling chip select
    assign tp_signals[9]  = spi_wr_strobe;                 // Register write strobe
    assign tp_signals[10] = spi_rd_active;                 // Register read active

    // --- Global interrupt line (11) ---
    assign tp_signals[11] = fpga_2_stm32_interrupt_N;      // IRQ as on the wire (active low)

    // --- UART / RS-485 multiplexer (12..20) ---
    assign tp_signals[12] = usart2_fpga_tx;                // TX from STM32
    assign tp_signals[13] = usart2_fpga_rx;                // RX to STM32 (post mux)
    assign tp_signals[14] = tx2;                           // Ch1 DD19 DI
    assign tp_signals[15] = rx2;                           // Ch1 DD19 RO
    assign tp_signals[16] = de2;                           // Ch1 driver enable
    assign tp_signals[17] = tx3;                           // Ch2 DD20 DI
    assign tp_signals[18] = rx3;                           // Ch2 DD20 RO
    assign tp_signals[19] = de3;                           // Ch2 driver enable
    assign tp_signals[20] = uart_mux_sel;                  // 0: DD19, 1: DD20

    // --- Raw STM32 discrete-output orders (21..28), pre-inversion ---
    assign tp_signals[21] = fcs_control_bus[0];            // ENA_SHOOTING order
    assign tp_signals[22] = fcs_control_bus[1];            // GMEE order
    assign tp_signals[23] = fcs_control_bus[2];            // RANGE_OVER_1280 order
    assign tp_signals[24] = fcs_control_bus[3];            // UOI order
    assign tp_signals[25] = fcs_control_bus[4];            // INHIBIT_SHOOTING order
    assign tp_signals[26] = fcs_control_bus[5];            // WIND_SENSOR_ON order
    assign tp_signals[27] = fcs_control_bus[6];            // BC_ON order
    assign tp_signals[28] = fcs_control_bus[7];            // BC_READY order

    // --- Key health/discrete signals (29..31) ---
    assign tp_signals[29] = fcs_wd_expired;                // Control watchdog violation
    assign tp_signals[30] = raw_fcs_in[25];                // BTN_CANNON (raw input)
    assign tp_signals[31] = raw_fcs_in[20];                // BC_EN (raw input)

    // Routing: default mapping set by debug_module tp_mux reset value
    // (TP5=100 Hz tick, TP6=1 kHz timebase, TP7=reset state).
    assign tp[5] = tp_signals[debug_tp_mux_out[4:0]];
    assign tp[6] = tp_signals[debug_tp_mux_out[9:5]];
    assign tp[7] = tp_signals[debug_tp_mux_out[14:10]];

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
    assign BC_ON            = ~fcs_control_bus[6];
    assign BC_READY         = ~fcs_control_bus[7];

    // =========================================================================
    // 13. CHIPSCOPE DEBUG (ICON JTAG bridge + ILA, only under USE_CHIPSCOPE)
    // =========================================================================
`ifdef USE_CHIPSCOPE
    wire [35:0] chipscope_control;

    chipscope_icon u_icon (
        .CONTROL0 (chipscope_control)
    );

    // 64-bit debug window: full STM32 <-> FCS conversation and its results
    wire [63:0] chipscope_trig0 = {
        9'd0,                              // 63:55 reserved
        fpga_2_stm32_interrupt_N,          // 54    IRQ line (active low)
        fcs_control_bus,                   // 53:46 discrete output orders
        data_rd_dev_s[`_D_S_FCS_ID_],      // 45:30 FCS read answer word
        spi_data_to_fpga,                  // 29:14 write data word from STM32
        addr_chip_s,                       // 13:8  register offset
        tick_out_pulse,                    // 7     sample-and-hold pulse
        tick_1khz,                         // 6     1 ms timebase
        rst,                               // 5     synchronous reset
        rd_dev_s[`_D_S_FCS_ID_],           // 4     FCS read strobe
        wr_dev_s[`_D_S_FCS_ID_],           // 3     FCS write strobe
        spi_stm32_nss_p,                   // 2     chip select
        spi_rd_active,                     // 1     bus read active
        spi_wr_strobe                      // 0     bus write strobe
    };

    chipscope_ila u_ila_bus (
        .CONTROL (chipscope_control),
        .CLK     (clk),
        .TRIG0   (chipscope_trig0)
    );
`endif

endmodule