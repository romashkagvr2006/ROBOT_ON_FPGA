// =============================================================================
// tb_cmd_parser.sv — тест разбора строк G-кода
// -----------------------------------------------------------------------------
// Тестбенч подаёт парсеру строки так же, как их подала бы очередь byte_fifo:
// по одному байту через рукопожатие in_valid / in_ready. Каждую выданную
// команду он ловит и сравнивает с ожидаемой: код команды, код ошибки, какие
// поля есть и какие в них числа.
//
// Проверяется:
//   * правильные строки из файла "путь-команды" (G0, G1, G28, M92),
//     запятая и точка, знак минус, маленькие буквы, поля без пробелов, комментарии;
//   * пустые строки и строки-комментарии: команд быть не должно;
//   * каждая ошибка err 1..4 на своём примере;
//   * после строки с ошибкой следующая строка разбирается правильно;
//   * конец строки только CR, только LF и CR LF;
//   * motion_ctrl долго не забирает команду: парсер ждёт и не теряет байты.
// =============================================================================
`timescale 1ns/1ps
module tb_cmd_parser;
    import robot_pkg::*;

    localparam real CLK_NS = 20.0;

    logic       clk = 1'b0;
    logic       rst = 1'b1;
    logic       in_valid = 1'b0;
    logic [7:0] in_data  = '0;
    logic       in_ready;
    logic       cmd_valid;
    cmd_t       cmd;
    logic       cmd_ready = 1'b1;

    int errors   = 0;
    int checks   = 0;

    cmd_parser dut (.*);

    always #(CLK_NS / 2) clk = ~clk;

    // ---- Монитор: каждую выданную команду кладём в очередь -----------------------
    // Передача происходит на фронте, где cmd_valid = 1 и cmd_ready = 1.
    cmd_t got_q[$];

    always @(posedge clk)
        if (!rst && cmd_valid && cmd_ready)
            got_q.push_back(cmd);

    // ---- Подать один байт (как FIFO): держим in_valid, пока парсер не возьмёт ----
    task automatic push_byte(input logic [7:0] b);
        @(negedge clk);
        in_valid = 1'b1;
        in_data  = b;
        while (!in_ready) @(negedge clk);    // парсер занят — ждём
        @(negedge clk);                      // фронт между спадами забрал байт
        in_valid = 1'b0;
    endtask

    // Подать текст без конца строки
    task automatic push_str(input string s);
        for (int i = 0; i < s.len(); i++) push_byte(s[i]);
    endtask

    // Подать строку и CR LF, как Termite по Enter
    task automatic send_line(input string s);
        push_str(s);
        push_byte(8'h0D);
        push_byte(8'h0A);
        repeat (3) @(negedge clk);           // дать парсеру отдать команду
    endtask

    // Удобная запись ожидания: какие поля есть и какие в них числа
    typedef struct {
        opcode_t             opcode;
        err_t                err;
        logic [NUM_AXES-1:0] has;     // порядок битов: {E, Z, Y, X}
        int                  x, y, z, e;
    } expect_t;

    function automatic string fields_str(input cmd_t c);
        return $sformatf("%s has(EZYX)=%b X=%0d Y=%0d Z=%0d E=%0d",
                         c.opcode.name(), c.has,
                         $signed(c.value[AX_X]), $signed(c.value[AX_Y]),
                         $signed(c.value[AX_Z]), $signed(c.value[AX_E]));
    endfunction

    // Проверить, что из очереди вышла ровно одна команда и она равна ожидаемой.
    // Для строк с ошибкой сравниваем только код ошибки.
    task automatic expect_cmd(input string line, input expect_t exp);
        cmd_t c;
        bit   bad;
        checks++;
        if (got_q.size() != 1) begin
            $display("ERROR: line \"%s\": expected 1 command, got %0d", line, got_q.size());
            errors++;
            got_q.delete();
            return;
        end
        c   = got_q.pop_front();
        bad = (c.err != exp.err);
        if (exp.err == ERR_NONE) begin
            bad |= (c.opcode != exp.opcode) || (c.has != exp.has);
            if (exp.has[AX_X]) bad |= ($signed(c.value[AX_X]) != exp.x);
            if (exp.has[AX_Y]) bad |= ($signed(c.value[AX_Y]) != exp.y);
            if (exp.has[AX_Z]) bad |= ($signed(c.value[AX_Z]) != exp.z);
            if (exp.has[AX_E]) bad |= ($signed(c.value[AX_E]) != exp.e);
        end
        if (bad) begin
            $display("ERROR: line \"%s\"", line);
            $display("       got      %s", fields_str(c));
            $display("       expected %s %s has(EZYX)=%b X=%0d Y=%0d Z=%0d E=%0d",
                     exp.opcode.name(), exp.err.name(), exp.has, exp.x, exp.y, exp.z, exp.e);
            errors++;
        end else
            $display("OK   \"%s\" -> %s", line, (c.err == ERR_NONE) ? fields_str(c)
                                                 : $sformatf("err %0d (%s)", c.err, c.err.name()));
    endtask

    task automatic expect_none(input string line);
        checks++;
        if (got_q.size() != 0) begin
            $display("ERROR: line \"%s\": expected no command, got %0d", line, got_q.size());
            errors++;
            got_q.delete();
        end else
            $display("OK   \"%s\" -> (no command, as expected)", line);
    endtask

    // Короткие имена для масок полей {E, Z, Y, X}
    localparam logic [3:0] F_NONE = 4'b0000, F_X = 4'b0001, F_Y = 4'b0010,
                           F_Z = 4'b0100,    F_E = 4'b1000;

    // Проверка одной строки целиком: отправить и сравнить
    task automatic check_line(input string line, input expect_t exp);
        send_line(line);
        expect_cmd(line, exp);
    endtask

    task automatic check_error(input string line, input err_t e);
        send_line(line);
        expect_cmd(line, '{OP_NONE, e, F_NONE, 0, 0, 0, 0});
    endtask

    // ---- Сценарии -------------------------------------------------------------------
    cmd_t second_cmd;

    initial begin
        repeat (3) @(posedge clk);
        @(negedge clk) rst = 1'b0;

        $display("--- correct lines ---");
        check_line("G1 X20 Y30,5 E3",          '{OP_G1,  ERR_NONE, F_X|F_Y|F_E, 2000, 3050, 0, 300});
        check_line("G0 X10 Y5",                '{OP_G0,  ERR_NONE, F_X|F_Y, 1000, 500, 0, 0});
        check_line("G0 Z2",                    '{OP_G0,  ERR_NONE, F_Z, 0, 0, 200, 0});
        check_line("G1 E2",                    '{OP_G1,  ERR_NONE, F_E, 0, 0, 0, 200});
        check_line("G28",                      '{OP_G28, ERR_NONE, F_NONE, 0, 0, 0, 0});
        check_line("G28 X Z",                  '{OP_G28, ERR_NONE, F_X|F_Z, 0, 0, 0, 0});
        check_line("M92 X400 Y400 Z400 E92.2", '{OP_M92, ERR_NONE, F_X|F_Y|F_Z|F_E, 40000, 40000, 40000, 9220});
        check_line("g0 x-1.5 y+0,25",          '{OP_G0,  ERR_NONE, F_X|F_Y, -150, 25, 0, 0});
        check_line("G1X5Y3E.5",                '{OP_G1,  ERR_NONE, F_X|F_Y|F_E, 500, 300, 0, 50});
        check_line("  G1   X1  ",              '{OP_G1,  ERR_NONE, F_X, 100, 0, 0, 0});
        check_line("G1 X99999.99",             '{OP_G1,  ERR_NONE, F_X, 9_999_999, 0, 0, 0});
        check_line("G1 X0.07 Y007",            '{OP_G1,  ERR_NONE, F_X|F_Y, 7, 700, 0, 0});
        check_line("G28 ; home all",           '{OP_G28, ERR_NONE, F_NONE, 0, 0, 0, 0});
        check_line("G1",                       '{OP_G1,  ERR_NONE, F_NONE, 0, 0, 0, 0});

        // Комментарий с русскими буквами (UTF-8 байты) после команды
        push_str("G0 X1 ");
        push_byte(";"); push_byte(8'hD0); push_byte(8'h9D); push_byte(8'hD0); push_byte(8'hB0);
        push_byte(8'h0D); push_byte(8'h0A);
        repeat (3) @(negedge clk);
        expect_cmd("G0 X1 ;<utf-8 text>", '{OP_G0, ERR_NONE, F_X, 100, 0, 0, 0});

        $display("--- lines without a command ---");
        send_line("");                  expect_none("<empty line>");
        send_line("    ");              expect_none("<spaces only>");
        send_line("; only a comment");  expect_none("; only a comment");

        $display("--- errors ---");
        check_error("G2 X1",          ERR_UNKNOWN);   // нет такой команды
        check_error("Q1",             ERR_UNKNOWN);   // строка не с G/M
        check_error("G",              ERR_UNKNOWN);   // нет номера
        check_error("G1000",          ERR_UNKNOWN);   // номер длиннее 3 цифр
        check_error("G1 Q5",          ERR_SYNTAX);    // буква не X/Y/Z/E
        check_error("G1 X1 X2",       ERR_SYNTAX);    // поле повторено
        check_error("G1 X1 #",        ERR_SYNTAX);    // мусор
        check_error("G1# X1",         ERR_SYNTAX);    // мусор сразу после номера
        check_error("G1 X1.234",      ERR_NUMBER);    // 3 цифры после запятой
        check_error("G1 X123456",     ERR_NUMBER);    // больше 99999,99
        check_error("G1 X99999999999",ERR_NUMBER);    // очень длинное число
        check_error("G1 X-",          ERR_NUMBER);    // знак без цифр
        check_error("G1 X.",          ERR_NUMBER);    // точка без цифр
        check_error("G1 X1a",         ERR_NUMBER);    // буква внутри числа
        check_error("G1 X",           ERR_NUMBER);    // поле без числа (можно только в G28)
        check_error("G0 X1 E2",       ERR_FIELDS);    // G0 с пастой
        check_error("G0 Z5 X1",       ERR_FIELDS);    // Z вместе с X
        check_error("G1 Z1",          ERR_FIELDS);    // G1 с Z
        check_error("G28 E1",         ERR_FIELDS);    // хоминг поршня нельзя
        check_error("M92 X0",         ERR_FIELDS);    // k = 0
        check_error("M92 E-5",        ERR_FIELDS);    // k < 0

        $display("--- recovery after an error ---");
        send_line("G1 X1 ? junk");
        expect_cmd("G1 X1 ? junk", '{OP_NONE, ERR_SYNTAX, F_NONE, 0, 0, 0, 0});
        check_line("G0 Y2",            '{OP_G0, ERR_NONE, F_Y, 0, 200, 0, 0});

        $display("--- line endings ---");
        push_str("G0 X3");
        push_byte(8'h0D);                                // только CR
        repeat (3) @(negedge clk);
        expect_cmd("G0 X3 <CR>", '{OP_G0, ERR_NONE, F_X, 300, 0, 0, 0});
        push_str("G0 X4");
        push_byte(8'h0A);                                // только LF
        repeat (3) @(negedge clk);
        expect_cmd("G0 X4 <LF>", '{OP_G0, ERR_NONE, F_X, 400, 0, 0, 0});

        $display("--- motion_ctrl is busy: command waits, bytes are not lost ---");
        cmd_ready = 1'b0;                                // "моторы едут"
        fork
            begin
                send_line("G1 X7");
                send_line("G1 Y8");                      // эта строка ждёт в "очереди"
            end
            begin
                repeat (300) @(negedge clk);
                if (got_q.size() != 0) begin
                    $display("ERROR: parser gave a command while cmd_ready = 0"); errors++;
                end
                if (!cmd_valid) begin
                    $display("ERROR: cmd_valid must stay 1 while waiting"); errors++;
                end
                cmd_ready = 1'b1;                        // "моторы доехали"
            end
        join
        repeat (5) @(negedge clk);
        if (got_q.size() == 2) begin
            second_cmd = got_q.pop_back();               // временно убираем вторую команду
            expect_cmd("G1 X7 (waited)", '{OP_G1, ERR_NONE, F_X, 700, 0, 0, 0});
            got_q.push_back(second_cmd);
            expect_cmd("G1 Y8 (waited in FIFO)", '{OP_G1, ERR_NONE, F_Y, 0, 800, 0, 0});
        end else begin
            $display("ERROR: expected 2 commands after cmd_ready, got %0d", got_q.size());
            errors++;
        end

        $display("------------------------------------------------");
        $display("lines checked: %0d", checks);
        if (errors == 0) $display("TEST PASSED");
        else             $display("TEST FAILED: %0d errors", errors);
        $finish;
    end

    initial begin
        #(CLK_NS * 200_000);
        $display("ERROR: simulation timeout");
        errors++;
        $finish;
    end

endmodule
