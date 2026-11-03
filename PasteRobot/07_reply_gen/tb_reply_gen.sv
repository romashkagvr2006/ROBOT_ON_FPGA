// =============================================================================
// tb_reply_gen.sv — тест генератора ответов вместе с настоящим uart_tx
// -----------------------------------------------------------------------------
// Цепочка как на плате:  reply_gen -> uart_tx -> линия tx.
// Тестбенч сам слушает линию tx как терминал на ПК: ловит старт-бит, читает
// биты в серединах и собирает символы в строки до LF. Каждая строка
// сравнивается с ожидаемой.
//
// Сценарии:
//   1. Все коды: ok, err 1 ... err 6.
//   2. Два результата подряд: второй ждёт (res_ready = 0), пока уходит первый,
//      и ни один байт не теряется.
//   3. Пока ответ отправляется, res_ready = 0.
// =============================================================================
`timescale 1ns/1ps
module tb_reply_gen;
    import robot_pkg::*;

    localparam int  CLK_HZ = 50_000_000;
    localparam int  BAUD   = 115_200;
    localparam real CLK_NS = 1.0e9 / CLK_HZ;
    localparam real BIT_NS = 1.0e9 / BAUD;

    logic       clk = 1'b0;
    logic       rst = 1'b1;
    logic       res_valid = 1'b0;
    err_t       res_err   = ERR_NONE;
    logic       res_ready;
    logic       tx_start;
    logic [7:0] tx_data;
    logic       tx_busy;
    logic       tx;

    int errors = 0;

    reply_gen u_reply (
        .clk, .rst,
        .res_valid, .res_err, .res_ready,
        .tx_start, .tx_data, .tx_busy
    );

    uart_tx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) u_tx (
        .clk, .rst,
        .tx_start (tx_start),
        .tx_data  (tx_data),
        .tx       (tx),
        .busy     (tx_busy)
    );

    always #(CLK_NS / 2) clk = ~clk;

    // ---- "Терминал на ПК": читаем линию tx и собираем строки --------------------
    string lines_q[$];       // принятые строки (без CR LF)
    string cur_line = "";

    initial begin
        logic [7:0] ch;
        forever begin
            @(negedge tx);                       // начало старт-бита
            #(BIT_NS / 2);                       // середина старт-бита
            if (tx !== 1'b0) begin
                $display("ERROR: false start bit"); errors++;
                continue;
            end
            for (int i = 0; i < 8; i++) begin
                #(BIT_NS);
                ch[i] = tx;                      // младший бит первым
            end
            #(BIT_NS);
            if (tx !== 1'b1) begin
                $display("ERROR: no stop bit after 0x%02h", ch); errors++;
            end
            if (ch == 8'h0A) begin               // LF: строка закончилась
                lines_q.push_back(cur_line);
                cur_line = "";
            end else if (ch != 8'h0D)            // CR пропускаем
                cur_line = {cur_line, string'(ch)};
            else if (cur_line.len() == 0) begin
                $display("ERROR: CR without text"); errors++;
            end
        end
    end

    // ---- Отдать результат (как motion_ctrl): держим valid, пока не возьмут ------
    task automatic give_result(input err_t e);
        @(negedge clk);
        res_valid = 1'b1;
        res_err   = e;
        while (!res_ready) @(negedge clk);
        @(negedge clk);
        res_valid = 1'b0;
    endtask

    // Дождаться n строк и сравнить с ожидаемыми
    task automatic expect_lines(input string exp[$]);
        int waited = 0;
        while (lines_q.size() < exp.size() && waited < 200) begin
            #(BIT_NS);
            waited++;
        end
        if (lines_q.size() != exp.size()) begin
            $display("ERROR: expected %0d line(s), got %0d", exp.size(), lines_q.size());
            errors++;
        end
        foreach (exp[i]) begin
            if (i < lines_q.size()) begin
                if (lines_q[i] != exp[i]) begin
                    $display("ERROR: line %0d is \"%s\", expected \"%s\"", i, lines_q[i], exp[i]);
                    errors++;
                end else
                    $display("OK   PC received \"%s\"", lines_q[i]);
            end
        end
        lines_q.delete();
    endtask

    initial begin
        repeat (5) @(posedge clk);
        rst = 1'b0;
        repeat (5) @(posedge clk);

        // 1. Все коды по одному
        give_result(ERR_NONE);    expect_lines('{"ok"});
        give_result(ERR_UNKNOWN); expect_lines('{"err 1"});
        give_result(ERR_SYNTAX);  expect_lines('{"err 2"});
        give_result(ERR_NUMBER);  expect_lines('{"err 3"});
        give_result(ERR_FIELDS);  expect_lines('{"err 4"});
        give_result(ERR_LIMIT);   expect_lines('{"err 5"});
        give_result(ERR_HOMING);  expect_lines('{"err 6"});
        $display("step 1: all reply codes - done");

        // 2 и 3. Два результата подряд; пока идёт первый, res_ready = 0
        fork
            begin
                give_result(ERR_NONE);
                give_result(ERR_FIELDS);
            end
            begin
                @(negedge clk);
                @(negedge clk);
                repeat (1000) begin
                    @(negedge clk);
                    if (res_ready && tx_busy) begin
                        $display("ERROR: res_ready = 1 while reply is being sent"); errors++;
                        break;
                    end
                end
            end
        join
        expect_lines('{"ok", "err 4"});
        $display("step 2: two results back-to-back - done");

        $display("------------------------------------------------");
        if (errors == 0) $display("TEST PASSED");
        else             $display("TEST FAILED: %0d errors", errors);
        $finish;
    end

    initial begin
        #(BIT_NS * 10 * 200);
        $display("ERROR: simulation timeout");
        errors++;
        $finish;
    end

endmodule
