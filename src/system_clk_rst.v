`include "define.v"

module system_clk_rst (
    input  wire  ext_clk_60,      // ѕереименовали входной порт под 60 ћ√ц
    output wire  clk_100,
    output wire  rst_sync,
    output reg   tick_1khz
);

    // --- Manual function for bit width calculation (ISE 14.7 fix) ---
    function integer f_clog2;
        input integer value;
        begin
            for (f_clog2 = 0; value > 0; f_clog2 = f_clog2 + 1)
                value = value >> 1;
        end
    endfunction

    // --- Clock generation (DCM) ---
    wire clk_fx_raw, clk_0_raw, clk_fb, locked, clk_60_ibufg;

    // ¬ходной буфер дл€ опорной частоты 60 ћ√ц
    IBUFG clk_in_buf (.I(ext_clk_60), .O(clk_60_ibufg));

    // Ќастройка DCM_SP под входную частоту 60 ћ√ц и выходную 100 ћ√ц
    DCM_SP #(
        .CLKFX_MULTIPLY(5),       // ”множитель изменен на 5 (было 4)
        .CLKFX_DIVIDE(3),         // ƒелитель изменен на 3 (было 1)
        .CLKIN_PERIOD(16.666667), // ѕериод дл€ 60 ћ√ц = 16.666667 нс (было 40.0)
        .CLK_FEEDBACK("1X")       // ќбратна€ св€зь по CLK0 (60 ћ√ц)
    ) dcm_inst (
        .CLKIN(clk_60_ibufg), .CLKFB(clk_fb), .RST(1'b0),
        .CLKFX(clk_fx_raw), .CLK0(clk_0_raw), .LOCKED(locked),
        .PSEN(1'b0), .PSINCDEC(1'b0), .PSCLK(1'b0), .DSSEN(1'b0),
        .CLK90(), .CLK180(), .CLK270(), .CLK2X(),
        .CLK2X180(), .CLKDV(), .CLKFX180(), .STATUS(), .PSDONE()
    );

    // Ѕуферы глобальных тактовых сетей (Global Clock Buffers)
    BUFG clk_out_buf (.I(clk_fx_raw), .O(clk_100)); // —истемный клок 100 ћ√ц
    BUFG clk_fb_buf  (.I(clk_0_raw),  .O(clk_fb));  //  лок обратной св€зи (60 ћ√ц)

    // --- Reset Bridge ---
    reg [7:0] sync_reg;
    wire rst_async = ~locked;
    always @(posedge clk_100 or posedge rst_async) begin
        if (rst_async) sync_reg <= 8'hFF;
        else           sync_reg <= {sync_reg[6:0], 1'b0};
    end
    assign rst_sync = sync_reg[7];

    // --- 1 kHz Tick Generator ---
    localparam CNT_LIMIT = `_D_DIV_1kHz_; // 99999
    // »спользование ручной функции вместо встроенной $clog2
    localparam CNT_WIDTH = f_clog2(CNT_LIMIT); 
    
    reg [CNT_WIDTH-1:0] tick_cnt;

    always @(posedge clk_100) begin
        if (rst_sync) begin
            tick_cnt  <= {CNT_WIDTH{1'b0}};
            tick_1khz <= 1'b0;
        end else begin
            if (tick_cnt >= CNT_LIMIT) begin
                tick_cnt  <= {CNT_WIDTH{1'b0}};
                tick_1khz <= 1'b1;
            end else begin
                tick_cnt  <= tick_cnt + 1'b1;
                tick_1khz <= 1'b0;
            end
        end
    end

endmodule