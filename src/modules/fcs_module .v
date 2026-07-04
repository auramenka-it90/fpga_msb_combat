`include "define.v"
`timescale 1ns / 1ps
// =============================================================================
// Module Name:    fcs_module (Device ID = 2)
// Description:    Fire Control System module. Generates a parallel bank of
//                 debounce filters dynamically based on IN_WIDTH parameter.
//                 Laches stable inputs on a "hold" clock-enable pulse (if enabled
//                 by the dedicated reg_latch_en register at address 0x17), 
//                 and exposes them to SPI.
//                 Provides a real-time bypass read option when latching is disabled.
// =============================================================================

// =============================================================================
//                                  README
// =============================================================================
// MODULE: fcs_module (Device ID = 2 on SPI Polling Bus)
//
// DESCRIPTION:
//  This module is the Fire Control System (FCS) discrete input/output manager.
//  It processes 31 noisy asynchronous PCB inputs, applies metastability guards,
//  parameterized debounce filtering, software input injection (HARD/SOFT override),
//  input signal inversion, and computes a Reset-Dominant JK Flip-Flop.
//  The outputs are exposed to the CPU SPI bus via a Sample-and-Hold latch
//  or a direct 0-latency combinatorial bypass multiplexer.
//
// =============================================================================
//                           SIGNAL PROCESSING PIPELINE
// =============================================================================
//
//  FCS_IN [Raw Pins] --> [ INVERTER (INV) ] --> [ DEBOUNCE FILTER (3ms) ]
//                                                           ¦
//                                                           ?
//  FCS_OUT [Clean]  <-- [ HARD/SOFT MUX ] <-- [ STABLE PHYSICAL SIGNAL ]
//                             ?
//                             +-- SOFT_VALUE (Software Inject Registers)
//
//  *NOTE: DR[10:0] sensors bypass both INVERTER and DEBOUNCE FILTER.
//         They only support the HARD/SOFT override multiplexer.
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
// -----------------------------------------------------------------------------
// WRITE-ONLY REGISTERS (STM32 -> FPGA)
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
//                    0 = Normal signal, 1 = Inverted signal on the PCB.
//
// Address: 0x15 | Name: reg_inv_2
//  - Bits [3:0]    : Inversion mask for physical inputs [30:27].
//
// Address: 0x16 | Name: reg_fcs_control
//  - Bits [7:0]    : Controls 8 external discrete outputs (ENA_SHOOTING, etc.).
//                    Outputs on the PCB are active-low (inverted at top-level).
//
// Address: 0x17 | Name: reg_latch_en
//  - Bit [0]       : LATCH_ENABLE control flag (MSB of fcs_control register write)
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
    parameter ADR_WIDTH = 6,
    parameter IN_WIDTH  = 31, // Parameterized input bus width (11 DR + 20 single inputs)
    parameter OUT_WIDTH = 8   // Parameterized control bus width
)(
    input  wire                  clk,         // System Clock (100 MHz)
    input  wire                  rst,         // Synchronous Reset
    input  wire                  tick,        // 1 ms Clock Enable tick for filters
    input  wire                  hold,        // Latch Enable pulse (100 Hz) to sample status

    // --- SPI CPU Interface ---
    input  wire [ADR_WIDTH-1:0]  cpu_addr,
    input  wire [15:0]           cpu_di,     
    input  wire                  cpu_wr,     
    input  wire                  cpu_rd,     
    output wire [15:0]           cpu_do,

    // --- External Interfaces ---
    input  wire [IN_WIDTH-1:0]   fcs_in,      // Raw PCB inputs with bounce
    output wire [OUT_WIDTH-1:0]  fcs_control  // Control outputs (write-register controlled)
);

    // =========================================================================
    // 1. REGISTER ADDRESS DEFINITIONS & BUS DECODING (debug_module style)
    // =========================================================================
    localparam BASE_ADDR            = 6'h00;
    
    // Write register offsets
    localparam P_OFF_HARD_SOFT_1    = 6'h10;
    localparam P_OFF_HARD_SOFT_2    = 6'h11;
    localparam P_OFF_SOFT_VAL_1     = 6'h12;
    localparam P_OFF_SOFT_VAL_2     = 6'h13;
    localparam P_OFF_INV_1          = 6'h14;
    localparam P_OFF_INV_2          = 6'h15;
    localparam P_OFF_FCS_CONTROL    = 6'h16;
    localparam P_OFF_LATCH_EN       = 6'h17;

    // Read register offsets
    localparam P_OFF_STATUS_1       = 6'h00;
    localparam P_OFF_STATUS_2       = 6'h01;

    // Write Decoding (Strictly like your debug_module)
    wire wr_hard_soft_1 = (cpu_addr == (BASE_ADDR + P_OFF_HARD_SOFT_1)) && cpu_wr;
    wire wr_hard_soft_2 = (cpu_addr == (BASE_ADDR + P_OFF_HARD_SOFT_2)) && cpu_wr;
    wire wr_soft_val_1  = (cpu_addr == (BASE_ADDR + P_OFF_SOFT_VAL_1))  && cpu_wr;
    wire wr_soft_val_2  = (cpu_addr == (BASE_ADDR + P_OFF_SOFT_VAL_2))  && cpu_wr;
    wire wr_inv_1       = (cpu_addr == (BASE_ADDR + P_OFF_INV_1))       && cpu_wr;
    wire wr_inv_2       = (cpu_addr == (BASE_ADDR + P_OFF_INV_2))       && cpu_wr;
    wire wr_fcs_control = (cpu_addr == (BASE_ADDR + P_OFF_FCS_CONTROL)) && cpu_wr;
    wire wr_latch_en    = (cpu_addr == (BASE_ADDR + P_OFF_LATCH_EN))    && cpu_wr;

    // Read Decoding
    wire rd_status_1    = (cpu_addr == (BASE_ADDR + P_OFF_STATUS_1))    && cpu_rd;
    wire rd_status_2    = (cpu_addr == (BASE_ADDR + P_OFF_STATUS_2))    && cpu_rd;

    // Control registers
    reg [15:0] reg_hard0_soft1_1;
    reg [14:0] reg_hard0_soft1_2; // 15 bits used for signals 16..30
    reg [15:0] reg_soft_value_1;
    reg [14:0] reg_soft_value_2;  // 15 bits used for signals 16..30
    reg [15:0] reg_inv_1;         // Inversion bits for signals 11..26
    reg [3:0]  reg_inv_2;         // Inversion bits for signals 27..30
    reg [7:0]  reg_fcs_control;   // Control register for 8 discrete outputs
    reg        reg_latch_en;      // 1-bit control register for latch enable (at address 0x17)

    // Register Write Logic
    always @(posedge clk) begin
        if (rst) begin
            reg_hard0_soft1_1 <= 16'h0000;
            reg_hard0_soft1_2 <= 15'h0000;
            reg_soft_value_1  <= 16'h0000;
            reg_soft_value_2  <= 15'h0000;
            reg_inv_1         <= 16'h0000;
            reg_inv_2         <= 4'h0;
            reg_fcs_control   <= 8'h00;
            reg_latch_en      <= 1'b0;
        end else begin
            if (wr_hard_soft_1) reg_hard0_soft1_1 <= cpu_di;
            if (wr_hard_soft_2) reg_hard0_soft1_2 <= cpu_di[14:0];
            if (wr_soft_val_1)  reg_soft_value_1  <= cpu_di;
            if (wr_soft_val_2)  reg_soft_value_2  <= cpu_di[14:0];
            if (wr_inv_1)       reg_inv_1         <= cpu_di;
            if (wr_inv_2)       reg_inv_2         <= cpu_di[3:0];
            if (wr_fcs_control) reg_fcs_control   <= cpu_di[7:0];
            if (wr_latch_en)    reg_latch_en      <= cpu_di[0];
        end
    end

    // =========================================================================
    // 2. INPUT INVERSION LOGIC (At the very front of the pipeline)
    // =========================================================================
    wire [30:0] fcs_in_inv;

    // DR sensors (bits 10:0) do not support inversion, pass directly
    assign fcs_in_inv[10:0] = fcs_in[10:0];

    // Single inputs (bits 30:11) are inverted using reg_inv_1 and reg_inv_2
    genvar k;
    generate
        // Signals 11..26 (HEF to RST_FILTR) mapped to reg_inv_1[0..15]
        for (k = 11; k <= 26; k = k + 1) begin: INV_MAP_1
            assign fcs_in_inv[k] = reg_inv_1[k-11] ? ~fcs_in[k] : fcs_in[k];
        end
        // Signals 27..30 (UR to SCF_ON_ADD) mapped to reg_inv_2[0..3]
        for (k = 27; k <= 30; k = k + 1) begin: INV_MAP_2
            assign fcs_in_inv[k] = reg_inv_2[k-27] ? ~fcs_in[k] : fcs_in[k];
        end
    endgenerate

    // =========================================================================
    // 3. DEBOUNCE FILTERING (For Physical Inputs Only, excluding DR)
    // =========================================================================
    wire [30:0] fcs_in_deb;

    // DR sensors (bits 10:0) bypass the debounce filter completely
    assign fcs_in_deb[10:0] = fcs_in_inv[10:0];

    // Single physical inputs (bits 30:11) are filtered using debounce_filter
    genvar d;
    generate
        for (d = 11; d <= 30; d = d + 1) begin: DEBOUNCE_GEN
            debounce_filter u_deb_filter (
                .clk       (clk),
                .rst       (rst),
                .tick_1ms  (tick),
                .noisy_in  (fcs_in_inv[d]), // Filter physical inverted input
                .clean_out (fcs_in_deb[d])  // Clean debounced physical signal
            );
        end
    endgenerate

    // =========================================================================
    // 4. HARD/SOFT OUTPUT MULTIPLEXER (STM32 Soft Override)
    // =========================================================================
    wire [30:0] fcs_processed; // Final real-time processed internal signals
    
    genvar m;
    generate
        // Signals 0..15 (DR[10:0] and HEF..GM)
        for (m = 0; m <= 15; m = m + 1) begin: MUX_MAP_1
            assign fcs_processed[m] = reg_hard0_soft1_1[m] ? reg_soft_value_1[m] : fcs_in_deb[m];
        end
        // Signals 16..30 (CC to SCF_ON_ADD)
        for (m = 0; m <= 14; m = m + 1) begin: MUX_MAP_2
            assign fcs_processed[m+16] = reg_hard0_soft1_2[m] ? reg_soft_value_2[m] : fcs_in_deb[m+16];
        end
    endgenerate

    // =========================================================================
    // 5. SYNCHRONOUS JK FLIP-FLOP (Reset-Dominant, K-Priority, No Toggle)
    // =========================================================================
    // J = fcs_processed[18] (SET_R)
    // K = fcs_processed[19] (RESET_R)
    reg fcs_jk_out;

    always @(posedge clk) begin
        if (rst) begin
            fcs_jk_out <= 1'b0;
        end else begin
            if (fcs_processed[19]) begin        // K (RESET_R) has absolute priority
                fcs_jk_out <= 1'b0;
            end else if (fcs_processed[18]) begin // J (SET_R)
                fcs_jk_out <= 1'b1;
            end
        end
    end

    // =========================================================================
    // 6. SAMPLE AND HOLD (Latching Registers on "hold" Pulse)
    // =========================================================================
    reg [15:0] status_1_reg;
    reg [14:0] status_2_reg;
    reg        status_jk_reg;
    
    wire       latch_en = reg_latch_en; // Dedicated Latch Enable Control

    always @(posedge clk) begin
        if (rst) begin
            status_1_reg  <= 16'h0000;
            status_2_reg  <= 15'h0000;
            status_jk_reg <= 1'b0;
        end else begin
            // Latch only on "hold" pulse if latch_en is 1, otherwise freeze registers
            if (latch_en && hold) begin 
                status_1_reg  <= fcs_processed[15:0];
                status_2_reg  <= fcs_processed[30:16];
                status_jk_reg <= fcs_jk_out;
            end
        end
    end

    // =========================================================================
    // 7. SPI READ REGISTERS & OUTPUT MULTIPLEXER (With Latch Bypass)
    // =========================================================================
    // Register 0 (Address 0x00): If latch_en=1 -> Read latched, if 0 -> Read real-time wires
    wire [15:0] status_1_val = latch_en ? status_1_reg : fcs_processed[15:0];

    // Register 1 (Address 0x01): If latch_en=1 -> Read latched, if 0 -> Read real-time wires
    wire [15:0] status_2_val = latch_en ? {status_jk_reg, status_2_reg} : {fcs_jk_out, fcs_processed[30:16]};

    // Output Data Multiplexer (debug_module style)
    assign cpu_do = rd_status_1 ? status_1_val :
                    rd_status_2 ? status_2_val : 16'h0000;

    // =========================================================================
    // 8. CONTROL OUTPUTS
    // =========================================================================
    // External outputs are mapped strictly to the 8 bits of reg_fcs_control
    assign fcs_control = reg_fcs_control;

endmodule