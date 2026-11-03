// Тестбенч: отправляет несколько байт и декодирует линию TX обратно,
// проверяя старт-бит, данные и стоп-бит на середине каждого бита.
`timescale 1ns/1ps
module tb_uart_tx;
    localparam int CLK_HZ = 50_000_000;
    localparam int BAUD   = 115_200;
    localparam int CLKS_PER_BIT = CLK_HZ / BAUD;
    localparam real CLK_NS = 20.0;

    logic clk = 0, rst_n = 0, tx_start = 0;
    logic [7:0] tx_data = 0;
    logic tx, busy;
    int errors = 0;

    uart_tx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) dut (.*);

    always #(CLK_NS/2) clk = ~clk;

    // Приёмник-эталон: ждёт спада, затем сэмплирует середины битов
    task automatic check_byte(input logic [7:0] expected);
        logic [7:0] got;
        @(negedge tx);
        repeat (CLKS_PER_BIT/2) @(posedge clk);
        if (tx !== 1'b0) begin $display("ОШИБКА: нет старт-бита"); errors++; end
        for (int i = 0; i < 8; i++) begin
            repeat (CLKS_PER_BIT) @(posedge clk);
            got[i] = tx;
        end
        repeat (CLKS_PER_BIT) @(posedge clk);
        if (tx !== 1'b1) begin $display("ОШИБКА: нет стоп-бита"); errors++; end
        if (got !== expected) begin
            $display("ОШИБКА: ожидали 0x%02h, получили 0x%02h", expected, got); errors++;
        end else
            $display("OK: 0x%02h ('%c')", got, got);
    endtask

    task automatic send(input logic [7:0] b);
        @(posedge clk);
        while (busy) @(posedge clk);
        tx_data <= b; tx_start <= 1;
        @(posedge clk); tx_start <= 0;
        @(posedge clk);  // даём busy подняться
    endtask

    logic [7:0] seq [5];
    initial begin
        seq[0] = 8'h55; seq[1] = 8'h00; seq[2] = 8'hFF; seq[3] = 8'h41; seq[4] = 8'hA5;
    end

    initial begin
        repeat (5) @(posedge clk);
        rst_n = 1;
        repeat (5) @(posedge clk);
        if (tx !== 1'b1) begin $display("ОШИБКА: в покое TX != 1"); errors++; end
        fork
            for (int i = 0; i < 5; i++) send(seq[i]);
            for (int j = 0; j < 5; j++) check_byte(seq[j]);
        join
        wait (!busy);
        if (errors == 0) $display("ТЕСТ ПРОЙДЕН");
        else             $display("ТЕСТ НЕ ПРОЙДЕН: %0d ошибок", errors);
        $finish;
    end

    // Сторож: не даёт симуляции зависнуть
    initial begin
        #(CLK_NS * CLKS_PER_BIT * 10 * 20);
        $display("ОШИБКА: таймаут симуляции");
        $finish;
    end

    initial begin
        if ($test$plusargs("vcd")) begin
            $dumpfile("tb_uart_tx.vcd");
            $dumpvars(0, tb_uart_tx);
        end
    end
endmodule
