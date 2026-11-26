`include "define.v"
`timescale 1ns / 1ps

// =============================================================================
//                                  README
// =============================================================================
// MODULE: fcs_module (Device ID = 2 on SPI Polling Bus)
//
// DESCRIPTION:
//  This module is the Fire Control System (FCS) discrete input/output manager.
//  It processes 31 noisy asynchronous PCB inputs, applies metastability guards,
//  parameterized debounce filtering, software input injection (HARD/SOFT override),
//  input signal inversion with hardware defaults, and computes a Reset-Dominant
//  JK Flip-Flop.
//  The outputs are exposed to the CPU SPI bus via a Sample-and-Hold latch
//  or a direct 0-latency combinatorial bypass multiplexer.
//
// =============================================================================
//                           SIGNAL PROCESSING PIPELINE
// =============================================================================
//
//  FCS_IN [Raw Pins] --> [ INVERTER (INV) ] --> [ DEBOUNCE FILTER (3ms) ]
//                                                           |
//                                                           v
//  FCS_OUT [Clean]  <-- [ HARD/SOFT MUX ] <-- [ STABLE PHYSICAL SIGNAL ]
//                             ^
//                             +-- SOFT_VALUE (Software Inject Registers)
//
//  *NOTE: DR[10:0] sensors bypass the INVERTER and use the debounce filter
//         in bypass mode (DEBOUNCE_TICKS = 0: 2-FF synchronizer only).
//         They also support the HARD/SOFT override multiplexer.
//
// =============================================================================
//                          SPI CPU REGISTER ADDRESS MAP
// =============================================================================
// All addresses are offsets from BASE_ADDR = 6'h00.
// Register data width is 16 bits (_D_DATA_WIDTH_ = 16).
//
// -----------------------------------------------------------------------------
// READ-ONLY REGISTERS (FPGA -> STM32)
// -----------------------------------------------------------------------------
// Address: 0x00 | Name: REG_STATUS_1
//  Contains the lower 16 clean, debounced inputs.
//   - Bits [15:11] : GM_clean, APDS_clean, HEAT_clean, MG_clean, HEF_clean
//   - Bits [10:0]  : DR_clean[10:0] (11-bit Sensor Bus)
//
// Address: 0x01 | Name: REG_STATUS_2
//  Contains the upper 15 clean, debounced inputs plus the JK Latch output at MSB.
//   - Bit [15]     : fcs_jk_out (Reset-Dominant JK Flip-Flop Output)
//   - Bits [14:11] : SCF_ON_ADD_clean, SCF_ON_clean, REM_clean, UR_clean
//   - Bits [10:8]  : RST_FILTR_clean, BTN_CANNON_clean, K1_clean
//   - Bits [7:4]   : PSCC_clean, WS_clean, RL_clean, BC_EN_clean
//   - Bits [3:0]   : RESET_R_clean, SET_R_clean, DC_clean, CC_clean
//
// Address: 0x02 | Name: REG_CONST
//  Read-only alive signature 16'h55AA (per-device bus presence check).
//
// Address: 0x03 | Name: REG_WD_TIMEOUT
//  Control-output watchdog timeout in milliseconds, read/write.
//  - Value N (1..65535): if no CPU write to this device happens for N ms
//    (1 ms tick), fcs_control outputs are forced to 0.
//  - Value 0: watchdog disabled. Default on reset = 0.
//    The STM32 arms it at boot from a #define in board_support_package.h.
//  Any SPI write to FCS registers reloads the counter (heartbeat).
//  On expiry the outputs read 0 while reg_fcs_control keeps its value
//  (readable for diagnostics); the next control write restores outputs.
//
// -----------------------------------------------------------------------------
// CONFIGURATION REGISTERS (STM32 <-> FPGA, readable back for diagnostics)
// -----------------------------------------------------------------------------
// Address: 0x10 | Name: reg_hard0_soft1_1
//  - Bits [15:0]   : Selects Hard/Soft override for inputs [15:0].
//                    0 = Use physical input, 1 = Override with soft value.
//
// Address: 0x11 | Name: reg_hard0_soft1_2
//  - Bits [14:0]   : Selects Hard/Soft override for inputs [30:16].
//
// Address: 0x12 | Name: reg_soft_value_1
//  - Bits [15:0]   : Virtual software-injected input values for inputs [15:0].
//
// Address: 0x13 | Name: reg_soft_value_2
//  - Bits [14:0]   : Virtual software-injected input values for inputs [30:16].
//
// Address: 0x14 | Name: reg_inv_1
//  - Bits [15:0]   : Inversion mask for physical inputs [26:11] (DR is excluded).
//                    Default on reset = 16'hFFE0 (Active-Low signals inverted to 1).
//                    Bits [4:0]=0 (HEF..GM Active-High), Bits [15:5]=1 (CC..RST Active-Low).
//
// Address: 0x15 | Name: reg_inv_2
//  - Bits [3:0]    : Inversion mask for physical inputs [30:27].
//                    Default on reset = 4'hF (UR, REM, SCF_ON, SCF_ON_ADD Active-Low).
//
// Address: 0x16 | Name: reg_fcs_control
//  - Bits [7:0]    : Controls 8 external discrete outputs (ENA_SHOOTING, etc.).
//                    Outputs on the PCB are active-low (inverted at top-level).
//
// Address: 0x17 | Name: reg_latch_en
//  - Bit [0]       : LATCH_ENABLE control flag
//                    0 = Combinatorial Bypass (0-latency real-time reading).
//                    1 = Sample-and-Hold Mode (Snapshots latched on "hold" pulse).
//
// =============================================================================
//                             SPECIAL HARDWARE BLOCKS
// =============================================================================
// 1. Reset-Dominant JK Flip-Flop (K-Priority):
//  Driven by J = SET_R (bit 18) and K = RESET_R (bit 19).
//  If RESET_R is active (1), the output 'fcs_jk_out' is forced to '0' (Reset wins).
//  If RESET_R is '0' and SET_R is '1', 'fcs_jk_out' is set to '1'.
//  If both are '0', the state remains unchanged. Toggle/Inversion is omitted.
//  Output is read at MSB (bit 15) of REG_STATUS_2.
//
// 2. Zero-Latency Combinatorial Bypass Multiplexer:
//  If LATCH_ENABLE (reg_latch_en[0]) is 1:
//   Inputs are sampled and held on the rising edge of the "hold" pulse (100 Hz).
//   Reading REG_STATUS_1/2 outputs these stable, frozen values.
//  If LATCH_ENABLE is 0:
//   The registers are frozen, and reading REG_STATUS_1/2 directly outputs the 
//   real-time processed/filtered wires with 0-cycle combinatorial delay.
// =============================================================================

module fcs_module #(
    parameter ADR_WIDTH      = `_D_S_CHIP_ADDR_WIDTH_, // Default: 6 bits
    parameter DATA_WIDTH     = `_D_DATA_WIDTH_,         // Default: 16 bits
    parameter IN_WIDTH       = 31,                      // 11 DR sensors + 20 single inputs
    parameter OUT_WIDTH      = 8,                       // 8 discrete control outputs
    parameter DEBOUNCE_TICKS = 3                        // Default filter duration in ms
)(
    input  wire                  clk,         // System Clock (100 MHz)
    input  wire                  rst,         // Synchronous Reset active high
    input  wire                  tick,        // 1 ms Clock Enable tick for filters
    input  wire                  hold,        // Latch Enable pulse (100 Hz) to sample status

    // --- SPI CPU Interface ---
    input  wire [ADR_WIDTH-1:0]  cpu_addr,
    input  wire [DATA_WIDTH-1:0] cpu_di,     
    input  wire                  cpu_wr,     
    input  wire                  cpu_rd,     
    output wire [DATA_WIDTH-1:0] cpu_do,

    // --- External Interfaces ---
    input  wire [IN_WIDTH-1:0]   fcs_in,      // Raw PCB inputs with bounce
    output wire [OUT_WIDTH-1:0]  fcs_control, // Control outputs (write-register controlled)

    // --- Debug Observation ---
    output wire                  dbg_wd_expired // 1 = control-output watchdog expired
);

    // =========================================================================
    // 1. REGISTER ADDRESS DEFINITIONS & LOCAL CONSTANTS
    // =========================================================================
    localparam [ADR_WIDTH-1:0] BASE_ADDR            = {ADR_WIDTH{1'b0}};
    
    // Writable register offsets (all readable back for diagnostics)
    localparam [ADR_WIDTH-1:0] P_OFF_WD_TIMEOUT     = BASE_ADDR + 6'h03;
    localparam [ADR_WIDTH-1:0] P_OFF_HARD_SOFT_1    = BASE_ADDR + 6'h10;
    localparam [ADR_WIDTH-1:0] P_OFF_HARD_SOFT_2    = BASE_ADDR + 6'h11;
    localparam [ADR_WIDTH-1:0] P_OFF_SOFT_VAL_1     = BASE_ADDR + 6'h12;
    localparam [ADR_WIDTH-1:0] P_OFF_SOFT_VAL_2     = BASE_ADDR + 6'h13;
    localparam [ADR_WIDTH-1:0] P_OFF_INV_1          = BASE_ADDR + 6'h14;
    localparam [ADR_WIDTH-1:0] P_OFF_INV_2          = BASE_ADDR + 6'h15;
    localparam [ADR_WIDTH-1:0] P_OFF_FCS_CONTROL    = BASE_ADDR + 6'h16;
    localparam [ADR_WIDTH-1:0] P_OFF_LATCH_EN       = BASE_ADDR + 6'h17;

    // Read-only register offsets
    localparam [ADR_WIDTH-1:0] P_OFF_STATUS_1       = BASE_ADDR + 6'h00;
    localparam [ADR_WIDTH-1:0] P_OFF_STATUS_2       = BASE_ADDR + 6'h01;
    localparam [ADR_WIDTH-1:0] P_OFF_CONST          = BASE_ADDR + 6'h02;

    localparam [DATA_WIDTH-1:0] P_CONST_VAL         = 16'h55AA; // FCS module alive signature

    // Bitfield constants
    localparam DR_BUS_WIDTH = 11;               // DR[10:0] 11-bit sensor bus
    localparam UPPER_WIDTH  = IN_WIDTH - DATA_WIDTH; // 31 - 16 = 15 bits
    localparam BIT_SET_R    = 18;               // J input bit index (SET_R)
    localparam BIT_RESET_R  = 19;               // K input bit index (RESET_R)

    // Write Decoding
    wire wr_hard_soft_1 = (cpu_addr == P_OFF_HARD_SOFT_1) && cpu_wr;
    wire wr_hard_soft_2 = (cpu_addr == P_OFF_HARD_SOFT_2) && cpu_wr;
    wire wr_soft_val_1  = (cpu_addr == P_OFF_SOFT_VAL_1)  && cpu_wr;
    wire wr_soft_val_2  = (cpu_addr == P_OFF_SOFT_VAL_2)  && cpu_wr;
    wire wr_inv_1       = (cpu_addr == P_OFF_INV_1)       && cpu_wr;
    wire wr_inv_2       = (cpu_addr == P_OFF_INV_2)       && cpu_wr;
    wire wr_fcs_control = (cpu_addr == P_OFF_FCS_CONTROL) && cpu_wr;
    wire wr_latch_en    = (cpu_addr == P_OFF_LATCH_EN)    && cpu_wr;
    wire wr_wd_timeout  = (cpu_addr == P_OFF_WD_TIMEOUT)  && cpu_wr;

    // Read Decoding (config registers echo back at their own offsets)
    wire rd_status_1      = (cpu_addr == P_OFF_STATUS_1)    && cpu_rd;
    wire rd_status_2      = (cpu_addr == P_OFF_STATUS_2)    && cpu_rd;
    wire rd_const         = (cpu_addr == P_OFF_CONST)       && cpu_rd;
    wire rd_hard_soft_1   = (cpu_addr == P_OFF_HARD_SOFT_1) && cpu_rd;
    wire rd_hard_soft_2   = (cpu_addr == P_OFF_HARD_SOFT_2) && cpu_rd;
    wire rd_soft_val_1    = (cpu_addr == P_OFF_SOFT_VAL_1)  && cpu_rd;
    wire rd_soft_val_2    = (cpu_addr == P_OFF_SOFT_VAL_2)  && cpu_rd;
    wire rd_inv_1         = (cpu_addr == P_OFF_INV_1)       && cpu_rd;
    wire rd_inv_2         = (cpu_addr == P_OFF_INV_2)       && cpu_rd;
    wire rd_fcs_control   = (cpu_addr == P_OFF_FCS_CONTROL) && cpu_rd;
    wire rd_latch_en      = (cpu_addr == P_OFF_LATCH_EN)    && cpu_rd;
    wire rd_wd_timeout    = (cpu_addr == P_OFF_WD_TIMEOUT)  && cpu_rd;

    // Control registers
    reg [DATA_WIDTH-1:0]  reg_hard0_soft1_1;
    reg [UPPER_WIDTH-1:0] reg_hard0_soft1_2;
    reg [DATA_WIDTH-1:0]  reg_soft_value_1;
    reg [UPPER_WIDTH-1:0] reg_soft_value_2;
    reg [DATA_WIDTH-1:0]  reg_inv_1;         // Inversion bits for signals 11..26 (Default: 16'hFFE0)
    reg [3:0]             reg_inv_2;         // Inversion bits for signals 27..30 (Default: 4'hF)
    reg [OUT_WIDTH-1:0]   reg_fcs_control;   // Control register for discrete outputs
    reg                   reg_latch_en;      // 1-bit control register for latch enable
    reg [DATA_WIDTH-1:0]  reg_wd_timeout;    // Watchdog timeout in ms (16 bit, max ~65 s), 0 = disabled

    // Register Write Logic with Active-Low Hardware Defaults
    always @(posedge clk) begin
        if (rst) begin
            reg_hard0_soft1_1 <= {DATA_WIDTH{1'b0}};
            reg_hard0_soft1_2 <= {UPPER_WIDTH{1'b0}};
            reg_soft_value_1  <= {DATA_WIDTH{1'b0}};
            reg_soft_value_2  <= {UPPER_WIDTH{1'b0}};
            
            // Hardware Inversion Defaults: Active-Low signals inverted by default (Active = 1)
            reg_inv_1         <= 16'hFFE0; // Bits [4:0]=0 (HEF..GM Active-High), Bits [15:5]=1 (CC..RST Active-Low)
            reg_inv_2         <= 4'hF;     // Bits [3:0]=1 (UR, REM, SCF_ON, SCF_ON_ADD Active-Low)
            
            reg_fcs_control   <= {OUT_WIDTH{1'b0}};
            reg_latch_en      <= 1'b0;
            reg_wd_timeout    <= {DATA_WIDTH{1'b0}}; // Watchdog disabled until STM32 arms it
        end else begin
            if (wr_hard_soft_1) reg_hard0_soft1_1 <= cpu_di;
            if (wr_hard_soft_2) reg_hard0_soft1_2 <= cpu_di[UPPER_WIDTH-1:0];
            if (wr_soft_val_1)  reg_soft_value_1  <= cpu_di;
            if (wr_soft_val_2)  reg_soft_value_2  <= cpu_di[UPPER_WIDTH-1:0];
            if (wr_inv_1)       reg_inv_1         <= cpu_di;
            if (wr_inv_2)       reg_inv_2         <= cpu_di[3:0]; // 4 inverted inputs 27..30 (UR, REM, SCF_ON, SCF_ON_ADD), bits 15:4 ignored
            if (wr_fcs_control) reg_fcs_control   <= cpu_di[OUT_WIDTH-1:0];
            if (wr_latch_en)    reg_latch_en      <= cpu_di[0];
            if (wr_wd_timeout)  reg_wd_timeout    <= cpu_di;
        end
    end

    // =========================================================================
    // 2. INPUT INVERSION LOGIC
    // =========================================================================
    wire [IN_WIDTH-1:0] fcs_in_inv;

    // DR sensors (bits 10:0) bypass inversion
    assign fcs_in_inv[DR_BUS_WIDTH-1:0] = fcs_in[DR_BUS_WIDTH-1:0];

    // Single inputs (bits 30:11) are inverted according to reg_inv_1 and reg_inv_2.
    // Magic boundaries 26/27 come from the fixed PCB input list (see README):
    //   20 inverted inputs split as 16 + 4 because each control register is 16 bits.
    genvar k;
    generate
        // Signals 11..26 (16 inputs CC..RST_FILTR) -> reg_inv_1[15:0]
        // Index shift: input k uses bit (k-11): input 11 -> bit 0, input 26 -> bit 15.
        for (k = DR_BUS_WIDTH; k <= 26; k = k + 1) begin: INV_MAP_1
            assign fcs_in_inv[k] = reg_inv_1[k-DR_BUS_WIDTH] ? ~fcs_in[k] : fcs_in[k];
        end
        // Signals 27..30 (4 inputs UR..SCF_ON_ADD) -> reg_inv_2[3:0]
        // Index shift: input k uses bit (k-27): input 27 -> bit 0, input 30 -> bit 3.
        for (k = 27; k < IN_WIDTH; k = k + 1) begin: INV_MAP_2
            assign fcs_in_inv[k] = reg_inv_2[k-27] ? ~fcs_in[k] : fcs_in[k];
        end
    endgenerate

    // =========================================================================
    // 3. DEBOUNCE FILTERING (Parameterized per channel)
    // =========================================================================
    wire [IN_WIDTH-1:0] fcs_in_deb;

    // All 31 inputs pass through debounce_filter instances:
    //  - DR sensor bus (bits 10:0): DEBOUNCE_TICKS = 0 -> true bypass,
    //    only the 2-FF metastability guard is synthesized (30 ns latency).
    //  - Single inputs (bits 30:11): active filter (default 3 ms threshold).
    genvar d;
    generate
        for (d = 0; d < DR_BUS_WIDTH; d = d + 1) begin: SYNC_ONLY_GEN
            debounce_filter #(
                .DEBOUNCE_TICKS (0)
            ) u_sync_only (
                .clk       (clk),
                .rst       (rst),
                .tick_1ms  (tick),
                .noisy_in  (fcs_in_inv[d]),
                .clean_out (fcs_in_deb[d])
            );
        end
        for (d = DR_BUS_WIDTH; d < IN_WIDTH; d = d + 1) begin: DEBOUNCE_GEN
            debounce_filter #(
                .DEBOUNCE_TICKS (DEBOUNCE_TICKS)
            ) u_deb_filter (
                .clk       (clk),
                .rst       (rst),
                .tick_1ms  (tick),
                .noisy_in  (fcs_in_inv[d]),
                .clean_out (fcs_in_deb[d])
            );
        end
    endgenerate

    // =========================================================================
    // 4. HARD/SOFT OUTPUT MULTIPLEXER (STM32 Simulation Override)
    // =========================================================================
    wire [IN_WIDTH-1:0] fcs_processed;
    
    genvar m;
    generate
        // Signals 0..15 (DR[10:0] and HEF..GM)
        for (m = 0; m < DATA_WIDTH; m = m + 1) begin: MUX_MAP_1
            assign fcs_processed[m] = reg_hard0_soft1_1[m] ? reg_soft_value_1[m] : fcs_in_deb[m];
        end
        // Signals 16..30 (CC to SCF_ON_ADD)
        for (m = 0; m < UPPER_WIDTH; m = m + 1) begin: MUX_MAP_2
            assign fcs_processed[m+DATA_WIDTH] = reg_hard0_soft1_2[m] ? reg_soft_value_2[m] : fcs_in_deb[m+DATA_WIDTH];
        end
    endgenerate

    // =========================================================================
    // 5. SYNCHRONOUS JK FLIP-FLOP (Reset-Dominant, K-Priority)
    // =========================================================================
    reg fcs_jk_out;

    always @(posedge clk) begin
        if (rst) begin
            fcs_jk_out <= 1'b0;
        end else begin
            if (fcs_processed[BIT_RESET_R]) begin        // K (RESET_R) has absolute priority
                fcs_jk_out <= 1'b0;
            end else if (fcs_processed[BIT_SET_R]) begin // J (SET_R)
                fcs_jk_out <= 1'b1;
            end
        end
    end

    // =========================================================================
    // 6. SAMPLE AND HOLD (Latching Registers on "hold" Pulse)
    // =========================================================================
    reg [DATA_WIDTH-1:0]  status_1_reg;
    reg [UPPER_WIDTH-1:0] status_2_reg;
    reg                   status_jk_reg;
    
    wire latch_en = reg_latch_en;

    always @(posedge clk) begin
        if (rst) begin
            status_1_reg  <= {DATA_WIDTH{1'b0}};
            status_2_reg  <= {UPPER_WIDTH{1'b0}};
            status_jk_reg <= 1'b0;
        end else begin
            if (latch_en && hold) begin 
                status_1_reg  <= fcs_processed[DATA_WIDTH-1:0];
                status_2_reg  <= fcs_processed[IN_WIDTH-1:DATA_WIDTH];
                status_jk_reg <= fcs_jk_out;
            end
        end
    end

    // =========================================================================
    // 7. SPI READ REGISTERS & OUTPUT MULTIPLEXER (With Latch Bypass)
    // =========================================================================
    wire [DATA_WIDTH-1:0] status_1_val = latch_en ? status_1_reg : fcs_processed[DATA_WIDTH-1:0];
    wire [DATA_WIDTH-1:0] status_2_val = latch_en ? {status_jk_reg, status_2_reg} : {fcs_jk_out, fcs_processed[IN_WIDTH-1:DATA_WIDTH]};

    assign cpu_do = rd_status_1    ? status_1_val                                        :
                    rd_status_2    ? status_2_val                                        :
                    rd_const       ? P_CONST_VAL                                         :
                    rd_hard_soft_1 ? reg_hard0_soft1_1                                   :
                    rd_hard_soft_2 ? {{(DATA_WIDTH-UPPER_WIDTH){1'b0}}, reg_hard0_soft1_2} :
                    rd_soft_val_1  ? reg_soft_value_1                                    :
                    rd_soft_val_2  ? {{(DATA_WIDTH-UPPER_WIDTH){1'b0}}, reg_soft_value_2} :
                    rd_inv_1       ? reg_inv_1                                           :
                    rd_inv_2       ? {12'd0, reg_inv_2}                                  :
                    rd_fcs_control ? {8'd0, reg_fcs_control}                             :
                    rd_latch_en    ? {15'd0, reg_latch_en}                               :
                    rd_wd_timeout  ? reg_wd_timeout                                          :
                    {DATA_WIDTH{1'b0}};

    // =========================================================================
    // 8. CONTROL-OUTPUT WATCHDOG (link-loss protection)
    // =========================================================================
    // Down-counter loaded from reg_wd_timeout by ANY SPI write to this device.
    // The FCS task writes control outputs every 20 ms, so the counter reloads
    // automatically while the link is alive. On expiry the outputs are forced
    // to 0; reg_fcs_control itself is preserved for diagnostics and the next
    // write restores the outputs. reg_wd_timeout = 0 disables this feature.
    reg [DATA_WIDTH-1:0] wd_cnt;
    reg                  wd_expired;

    always @(posedge clk) begin
        if (rst) begin
            wd_cnt     <= {DATA_WIDTH{1'b0}};
            wd_expired <= 1'b0;
        end else if (wr_wd_timeout) begin
            wd_cnt     <= cpu_di;              // re-arm with the new timeout value
            wd_expired <= 1'b0;
        end else if (cpu_wr) begin
            wd_cnt     <= reg_wd_timeout;      // heartbeat: reload on any write
            wd_expired <= 1'b0;
        end else if (tick && (reg_wd_timeout != {DATA_WIDTH{1'b0}})) begin
            if (wd_cnt != {DATA_WIDTH{1'b0}}) begin
                if (wd_cnt == {{(DATA_WIDTH-1){1'b0}}, 1'b1}) wd_expired <= 1'b1;
                wd_cnt <= wd_cnt - {{(DATA_WIDTH-1){1'b0}}, 1'b1};
            end
        end
    end

    // =========================================================================
    // 9. CONTROL OUTPUTS
    // =========================================================================
    assign fcs_control = wd_expired ? {OUT_WIDTH{1'b0}} : reg_fcs_control;

    // Observation flag for diagnostics (testpoint mux).
    assign dbg_wd_expired = wd_expired;

endmodule