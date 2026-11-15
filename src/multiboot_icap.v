`timescale 1ns / 1ps

// =============================================================================
// MODULE: multiboot_icap
// 
// DESCRIPTION:
//  Internal Configuration Access Port (ICAP) MultiBoot Controller for Spartan-6.
//  Clocks ICAP_SPARTAN6 primitive at a safe 10.0 MHz (Max limit is 20 MHz per DS162).
//
//  CDC Architecture:
//   - Uses a 100 MHz Sticky Flag (req_sticky) to latch the 1-cycle trigger pulse.
//   - Synchronizes the static level into the 10.0 MHz domain via 2-stage FF.
//   - Self-contained: Resets on system 'rst' or upon FPGA IPROG hardware reload.
//
//  Target Silicon: Xilinx Spartan-6 (XC6SLX4 / XC6SLX9-TQG144)
//  Toolchain:      Aldec Active-HDL 9.2 / ISE 14.7 / XST
//  All comments in ASCII English.
// =============================================================================

module multiboot_icap #(
    parameter [23:0] IMAGE2_ADDR = 24'h100000 // 1 MB Flash Byte Offset (0x0010_0000)
)(
    input  wire clk,     // System Clock (100 MHz)
    input  wire rst,     // Synchronous Reset (Active High)
    input  wire trigger  // 1-cycle pulse from debug_module (100 MHz domain)
);

    // =========================================================================
    // BIT-SWAPPING FUNCTION (Mandatory for Spartan-6 ICAP, UG380 Table 7-1)
    // =========================================================================
    function [15:0] fn_bitswap16;
        input [15:0] in_word;
        begin
            fn_bitswap16 = {
                in_word[8],  in_word[9],  in_word[10], in_word[11],
                in_word[12], in_word[13], in_word[14], in_word[15],
                in_word[0],  in_word[1],  in_word[2],  in_word[3],
                in_word[4],  in_word[5],  in_word[6],  in_word[7]
            };
        end
    endfunction

    // =========================================================================
    // DEDICATED 10.0 MHz ICAP CLOCK GENERATOR (100 MHz / 10 = 10.0 MHz)
    // Guarantees strict compliance with Spartan-6 ICAP max 20 MHz limit (DS162).
    // =========================================================================
    reg [2:0] div_cnt;
    reg       icap_clk_reg; // Internal flip-flop for clock generation
    wire      icap_clk;     // Buffered global clock

    always @(posedge clk) begin
        if (rst) begin
            div_cnt      <= 3'd0;
            icap_clk_reg <= 1'b0;
        end else begin
            if (div_cnt == 3'd4) begin
                div_cnt      <= 3'd0;
                icap_clk_reg <= ~icap_clk_reg; // 10.0 MHz 50% square wave
            end else begin
                div_cnt <= div_cnt + 1'b1;
            end
        end
    end

    // Global Clock Buffer to fix "Gated clock" (PhysDesignRules:372) warning
    BUFG bufg_icap (
        .I (icap_clk_reg),
        .O (icap_clk)
    );

    // =========================================================================
    // CDC PULSE-TO-LEVEL CONVERTER (100 MHz Clock Domain)
    // Converts 1-cycle (10 ns) trigger into a sticky level to prevent CDC drops.
    // =========================================================================
    reg req_sticky;

    always @(posedge clk) begin
        if (rst) begin
            req_sticky <= 1'b0;
        end else if (trigger) begin
            req_sticky <= 1'b1; // Latched until FPGA reboots or system reset occurs
        end
    end

    // =========================================================================
    // CDC 2-STAGE SYNCHRONIZER (10.0 MHz ICAP Clock Domain)
    // =========================================================================
    reg [1:0] trig_sync;

    always @(posedge icap_clk) begin
        if (rst) begin
            trig_sync <= 2'b00;
        end else begin
            trig_sync <= {trig_sync[0], req_sticky};
        end
    end

    wire trig_10mhz = trig_sync[1];

    // =========================================================================
    // IPROG SEQUENCE ROM (13 Words Total - Parameterized Direct Constants)
    // =========================================================================
    reg [3:0]  seq_idx;
    reg [15:0] raw_icap_data;

    always @(*) begin
        case (seq_idx)
            4'd0:  raw_icap_data = 16'hFFFF;                        // Dummy Word
            4'd1:  raw_icap_data = 16'hAA99;                        // Sync Word 1
            4'd2:  raw_icap_data = 16'h5566;                        // Sync Word 2
            4'd3:  raw_icap_data = 16'h30A1;                        // Write WBSTAR Upper (GENERAL_1)
            4'd4:  raw_icap_data = {8'h00, IMAGE2_ADDR[23:16]};     // Upper 8 bits of Flash Address
            4'd5:  raw_icap_data = 16'h30A2;                        // Write WBSTAR Lower (GENERAL_2)
            4'd6:  raw_icap_data = IMAGE2_ADDR[15:0];               // Lower 16 bits of Flash Address
            4'd7:  raw_icap_data = 16'h30A3;                        // Write CMD Register
            4'd8:  raw_icap_data = 16'h000E;                        // IPROG Command (Reboot trigger!)
            4'd9:  raw_icap_data = 16'h2000;                        // NOP Flush 1
            4'd10: raw_icap_data = 16'h2000;                        // NOP Flush 2
            4'd11: raw_icap_data = 16'h2000;                        // NOP Flush 3
            4'd12: raw_icap_data = 16'h2000;                        // NOP Flush 4
            default: raw_icap_data = 16'hFFFF;
        endcase
    end

    // =========================================================================
    // FSM CONTROLLER (Driven synchronously by 10.0 MHz ICAP clock)
    // =========================================================================
    localparam ST_IDLE    = 1'b0;
    localparam ST_EXECUTE = 1'b1;

    reg state;
    reg icap_ce_n;

    always @(posedge icap_clk) begin
        if (rst) begin
            state     <= ST_IDLE;
            seq_idx   <= 4'd0;
            icap_ce_n <= 1'b1;
        end else begin
            case (state)
                ST_IDLE: begin
                    icap_ce_n <= 1'b1;
                    seq_idx   <= 4'd0;
                    
                    if (trig_10mhz) begin
                        state <= ST_EXECUTE;
                    end
                end

                ST_EXECUTE: begin
                    icap_ce_n <= 1'b0; // Assert ICAP Chip Enable (Active-Low)

                    if (seq_idx == 4'd12) begin
                        seq_idx <= 4'd12; // Hold at last NOP while hardware reboots
                    end else begin
                        seq_idx <= seq_idx + 1'b1;
                    end
                end
            endcase
        end
    end

    // =========================================================================
    // SPARTAN-6 HARDWARE CONFIGURATION ACCESS PORT (ICAP_SPARTAN6)
    // Clocked safely at 10.0 MHz (Zero switching limit violations)
    // =========================================================================
    wire [15:0] icap_in_swapped = fn_bitswap16(raw_icap_data);

    ICAP_SPARTAN6 #(
        .DEVICE_ID         (32'h04001093),
        .SIM_CFG_FILE_NAME ("NONE")
    ) u_icap (
        .CLK   (icap_clk),  // 10.0 MHz Clock (Strictly within DS162 20 MHz limit)
        .CE    (icap_ce_n), // Active-Low Enable
        .WRITE (icap_ce_n), // 0 = Write Mode
        .I     (icap_in_swapped),
        .O     (),
        .BUSY  ()
    );

endmodule