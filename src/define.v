// =============================================================================
// GLOBAL SYSTEM DEFINITIONS
// =============================================================================
`define     _D_CLK_                 100000000   // System Clock: 100 MHz
`define     _D_DIV_1kHz_            ((`_D_CLK_/1000) - 1)
`define     _D_DATA_WIDTH_          16          // Standard Data Bus Width

// =============================================================================
// SPI BUS CONFIGURATION (Serial Interface)
// =============================================================================
`define     _D_S_ADDR_WIDTH_        10          // Total SPI address bits
`define     _D_S_DEV_ADDR_WIDTH_    4           // 4 bits for Device ID
`define     _D_S_CHIP_ADDR_WIDTH_   6           // 6 bits for Internal Registers

// Address bit mapping for SPI: [Device ID (9:6)] [Register Address (5:0)]
`define     _D_S_DEV_HI_            9
`define     _D_S_DEV_LO_            6
`define     _D_S_CHIP_HI_           5
`define     _D_S_CHIP_LO_           0

// Number of active SPI modules & Device IDs
`define     _D_S_NUM_OF_DEV_        3           // “≈œ≈–‹ “”“ 3 (·˚ÎÓ 2)
`define     _D_S_DEBUG_ID_          1           // ID 1: SPI Debug Module
`define     _D_S_FCS_ID_            2           // ID 2: SPI Fire Control System Module (FCS)
`define     _D_S_INT_CTRL_ID_       3           // ID 3: SPI Interrupt Controller (ÕÓ‚˚È!)