`timescale 1ns / 1ps
// =============================================================================
// Testbench: spi_stm32_fpga_bridge
// Simulates an STM32 SPI1 master (Mode 0, 16-bit frames, MSB first, 12.5 Mbit)
// and checks write/read framing, strobes, CS abort and clock-overrun recovery.
// Pure Verilog-2001. Run: vsim work.tb_spi_bridge ; run -all
// =============================================================================
module tb_spi_bridge;

    // ---------------------------------------------------------------- clock
    reg clk = 1'b0;
    always #5 clk = ~clk;              // 100 MHz system clock

    // ------------------------------------------------------------ DUT I/O
    reg  sck  = 1'b0;
    reg  mosi = 1'b0;
    reg  nss  = 1'b1;
    wire miso;
    wire [9:0]  addr;
    wire [15:0] dout;
    wire        wr_strobe;
    wire        rd;
    wire        busy;

    reg  rst = 1'b1;

    // ------------------------------------------ fake FPGA register file (din)
    reg [15:0] fake_reg_002 = 16'h1234;
    reg [15:0] fake_reg_003 = 16'hAAAA;
    wire [15:0] din = (addr == 10'h002) ? fake_reg_002 :
                      (addr == 10'h003) ? fake_reg_003 :
                      16'hBEEF;

    // ------------------------------------------------------------ DUT
    spi_stm32_fpga_bridge dut (
        .clk       (clk),
        .rst       (rst),
        .spi_in    (mosi),
        .sck_in    (sck),
        .nss_in    (nss),
        .spi_out   (miso),
        .addr      (addr),
        .dout      (dout),
        .din       (din),
        .wr_strobe (wr_strobe),
        .rd        (rd),
        .busy      (busy)
    );

    // ------------------------------------------------------------ counters
    integer wr_cnt = 0;
    integer rd_cnt = 0;
    always @(posedge clk) begin
        if (wr_strobe) wr_cnt = wr_cnt + 1;
        if (rd)        rd_cnt = rd_cnt + 1;
    end

    // ------------------------------------------------ STM32 master (Mode 0)
    // SCK low: master changes MOSI. Rising edge: slave samples.
    // SCK high half: master samples MISO (slave shifted it on prev falling).
    integer half_ns = 40;              // ns, 40 -> 12.5 MHz SCK (as in .ioc)

    task shift_bit(input b, output m);
        begin
            mosi = b;
            #(half_ns - 2);
            sck = 1'b1;                // rising
            #2;
            m = miso;                  // master samples on rising
            #(half_ns - 2);
            sck = 1'b0;                // falling
            #2;
        end
    endtask

    // Full 32-clock frame (or nbits clocks for abort/overrun tests)
    task xfer(input [15:0] w1, input [15:0] w2, output [15:0] rx2,
              input integer nbits);
        integer i;
        reg     mb;
        begin
            nss = 1'b0;
            #(half_ns * 4);            // CS setup time
            rx2 = 16'h0000;
            for (i = 0; i < nbits; i = i + 1) begin
                shift_bit((i < 16) ? w1[15-i] : w2[31-i], mb);
                if ((i >= 16) && (i < 32))
                    rx2[31-i] = mb;
            end
            nss = 1'b1;
            #(half_ns * 4);            // CS hold + DUT settle
        end
    endtask

    function [15:0] cmd_wr(input [9:0] a);
        begin cmd_wr = {1'b1, 5'b00000, a}; end
    endfunction

    function [15:0] cmd_rd(input [9:0] a);
        begin cmd_rd = {1'b0, 5'b00000, a}; end
    endfunction

    // -------------------------------------------------------------- checks
    integer errors = 0;
    task check32(input [31:0] got, input [31:0] exp, input [8*30:1] name);
        begin
            if (got !== exp) begin
                errors = errors + 1;
                $display("FAIL %0s: got %h expected %h", name, got, exp);
            end else
                $display("PASS %0s = %h", name, got);
        end
    endtask

    // Speed sweep helper: write/read round trips at current half_ns
    integer sp_err;
    task sweep_and_report(input integer mbit);
        reg [15:0] r;
        begin
            sp_err = 0;
            xfer(cmd_wr(10'h042), 16'h1111, r, 32);
            if (dout !== 16'h1111) sp_err = sp_err + 1;
            xfer(cmd_rd(10'h002), 16'h0000, r, 32);
            if (r !== 16'h1234)    sp_err = sp_err + 1;
            xfer(cmd_wr(10'h042), 16'h2222, r, 32);
            if (dout !== 16'h2222) sp_err = sp_err + 1;
            xfer(cmd_rd(10'h003), 16'h0000, r, 32);
            if (r !== 16'hAAAA)    sp_err = sp_err + 1;
            if (sp_err == 0)
                $display("SPEED %0d Mbit/s : OK", mbit);
            else
                $display("SPEED %0d Mbit/s : BROKEN (%0d failed checks)", mbit, sp_err);
        end
    endtask

    // --------------------------------------------------------------- main
    reg [15:0] rx;
    initial begin
        repeat (5) @(posedge clk);
        rst = 1'b0;
        repeat (2) @(posedge clk);

        // T1: write 0xBEEF to 0x042
        xfer(cmd_wr(10'h042), 16'hBEEF, rx, 32);
        check32({22'd0, addr}, 32'h042, "T1 addr");
        check32(dout, 32'h0000BEEF, "T1 dout");
        check32(wr_cnt, 32'd1, "T1 wr_cnt");
        check32(rd_cnt, 32'd0, "T1 rd_cnt");

        // T2: read 0x002 -> fake_reg_002
        xfer(cmd_rd(10'h002), 16'h0000, rx, 32);
        check32(rx, 32'h00001234, "T2 readback");
        check32(wr_cnt, 32'd1, "T2 wr_cnt");
        check32(rd_cnt, 32'd1, "T2 rd_cnt");

        // T3: read 0x003
        xfer(cmd_rd(10'h003), 16'h0000, rx, 32);
        check32(rx, 32'h0000AAAA, "T3 readback");
        check32(rd_cnt, 32'd2, "T3 rd_cnt");

        // T4: write to 0x003, dout must update, wr_cnt increments once
        xfer(cmd_wr(10'h003), 16'h5A5A, rx, 32);
        check32(dout, 32'h00005A5A, "T4 dout");
        check32(wr_cnt, 32'd2, "T4 wr_cnt");

        // T5: aborted frame (CS pulled up mid-word 2) must not write
        xfer(cmd_wr(10'h009), 16'hFACE, rx, 20);
        check32(wr_cnt, 32'd2, "T5 abort wr_cnt");
        // recovery: next normal write must work
        xfer(cmd_wr(10'h009), 16'hFACE, rx, 32);
        check32(dout, 32'h0000FACE, "T5 recovery dout");
        check32(wr_cnt, 32'd3, "T5 recovery wr_cnt");

        // T6: clock overrun (36 clocks, CS held low) -> single write only
        xfer(cmd_wr(10'h077), 16'h0F0F, rx, 36);
        check32(wr_cnt, 32'd4, "T6 overrun single wr");
        check32(dout, 32'h00000F0F, "T6 overrun dout");
        // recovery: read still functional
        xfer(cmd_rd(10'h002), 16'h0000, rx, 32);
        check32(rx, 32'h00001234, "T6 read after overrun");

        // T7: MISO released after CS deassert
        #(half_ns * 4);
        if (miso !== 1'b0) begin
            // top-level tri-states it, bridge itself idles low
            errors = errors + 1;
            $display("FAIL T7 miso idle: got %b expected 0", miso);
        end else
            $display("PASS T7 miso idle = 0");

        // ---------------------------------------------------------- sweep
        // Move SCK faster and faster; find the frequency where the bridge
        // (2FF SCK synchronizer + edge detect at 100 MHz) starts to break.
        // Informational: does NOT add to main error count.
        $display("--- SCK speed sweep (informational) ---");
        half_ns = 40; sweep_and_report(12);   // presc 8  in .ioc
        half_ns = 32; sweep_and_report(16);   // presc 6
        half_ns = 26; sweep_and_report(19);   // presc 5
        half_ns = 20; sweep_and_report(25);   // presc 4
        half_ns = 16; sweep_and_report(31);   // presc 3
        half_ns = 13; sweep_and_report(38);   // presc 2
        half_ns = 10; sweep_and_report(50);   // presc 2 approx
        half_ns = 40;                         // restore default

        $display("=============================================");
        if (errors == 0) $display("ALL TESTS PASSED");
        else             $display("TOTAL ERRORS: %0d", errors);
        $finish;
    end

    // Safety timeout
    initial begin
        #10_000_000;
        $display("TIMEOUT - test did not finish");
        $finish;
    end

endmodule
