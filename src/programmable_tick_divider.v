					   				  
`timescale 1ns / 1ps

module programmable_tick_divider (
    input  wire       clk,        // Системная тактовая частота (например, 100 МГц)
    input  wire       rst,        // Синхронный сброс (активный высокий)
    input  wire       tick,       // Входной импульс 1 мс (длительностью в 1 такт clk)
    input  wire [7:0] din,        // Входной период в миллисекундах (0...255)
    output reg        tick_out    // Выходной импульс заданного периода (длительностью в 1 такт clk)
);

    // --- Защита от нулевого периода ---
    // Если din = 0, то таймер выключается (выход всегда 0).
    // Если din > 0, предел счета равен (din - 1).
    wire [7:0] count_limit = (din > 8'd0) ? (din - 8'd1) : 8'd0;

    reg [7:0] tick_counter;

    always @(posedge clk) begin
        if (rst) begin
            tick_counter <= 8'd0;
            tick_out     <= 1'b0;
        end else begin
            if (tick) begin
                // Используем оператор >= для защиты от зависания счетчика при динамическом уменьшении din
                if (tick_counter >= count_limit) begin
                    tick_counter <= 8'd0;
                    // Выходной импульс генерируется только если din больше нуля (модуль включен)
                    tick_out     <= (din > 8'd0); 
                end else begin
                    tick_counter <= tick_counter + 8'd1;
                    tick_out     <= 1'b0;
                end
            end else begin
                // Импульс на выходе должен длиться строго 1 такт clk
                tick_out <= 1'b0;
            end
        end
    end

endmodule