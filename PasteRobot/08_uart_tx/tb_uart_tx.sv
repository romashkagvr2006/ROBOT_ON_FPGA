// Тестбенч: отправляет несколько байт и декодирует линию TX обратно,
// проверяя старт-бит, данные и стоп-бит на середине каждого бита.
`timescale 1ns/1ps
module tb_uart_tx;
    localparam int CLK_HZ = 50_000_000;
    localparam int BAUD   = 115_200;
    localparam int CLKS_PER_BIT = CLK_HZ / BAUD;
    localparam real CLK_NS = 20.0;

    logic clk = 0, rst = 1, tx_start = 0;
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
        if (tx !== 1'b0) begin $display("ERROR: no start bit"); errors++; end
        for (int i = 0; i < 8; i++) begin
            repeat (CLKS_PER_BIT) @(posedge clk);
            got[i] = tx;
        end
        repeat (CLKS_PER_BIT) @(posedge clk);
        if (tx !== 1'b1) begin $display("ERROR: no stop bit"); errors++; end
        if (got !== expected) begin
            $display("ERROR: expected 0x%02h, got 0x%02h", expected, got); errors++;
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
     localparam int N = 19;
    logic [7:0] seq [19];
    initial begin
        seq[0]  = 8'h55;   // синхробайт 1
    seq[1]  = 8'hFF;   // синхробайт 2
    seq[2]  = 8'h47;   // G
    seq[3]  = 8'h30;   // 0
    seq[4]  = 8'h20;   // пробел
    seq[5]  = 8'h58;   // X
    seq[6]  = 8'h32;   // 2
    seq[7]  = 8'h30;   // 0
    seq[8]  = 8'h20;   // пробел
    seq[9]  = 8'h59;   // Y
    seq[10] = 8'h33;   // 3
    seq[11] = 8'h30;   // 0
    seq[12] = 8'h2C;   // ,
    seq[13] = 8'h35;   // 5
    seq[14] = 8'h20;   // пробел
    seq[15] = 8'h45;   // E
    seq[16] = 8'h33;   // 3
    seq[17] = 8'h0D;   // CR (Enter)
    seq[18] = 8'h0A;   // LF (Enter)
    end

    initial begin
        repeat (5) @(posedge clk);
        rst = 0;
        repeat (5) @(posedge clk);
        if (tx !== 1'b1) begin $display("ERROR: idle TX != 1"); errors++; end
        fork
            for (int i = 0; i < 19; i++) send(seq[i]);
            for (int j = 0; j < 19; j++) check_byte(seq[j]);
        join
        wait (!busy);
        if (errors == 0) $display("TEST PASSED");
        else             $display("TEST FAILED: %0d errors", errors);
        $finish;
    end

    // Сторож: не даёт симуляции зависнуть
    initial begin
        #(CLK_NS * CLKS_PER_BIT * 10 * 30);
        $display("ERROR: simulation timeout");
        errors++;
        $finish;
    end

    initial begin
        if ($test$plusargs("vcd")) begin
            $dumpfile("tb_uart_tx.vcd");
            $dumpvars(0, tb_uart_tx);
        end
    end
endmodule
