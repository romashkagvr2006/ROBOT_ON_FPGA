// =============================================================================
// tb_uart_rx.sv — тест приёмника UART
// -----------------------------------------------------------------------------
// Тестбенч сам играет роль ПК: "дёргает" линию rx бит за битом с нужной
// скоростью (задача send_byte) и проверяет, что приёмник выдал тот же байт.
//
// Сценарии:
//   1. Тишина на линии     — приёмник ничего не выдаёт.
//   2. Граничные байты     — 00, FF, 55, AA, 01, 80.
//   3. Все 256 значений    — каждый байт принят без ошибок.
//   4. Байты подряд        — без паузы между стоп-битом и следующим старт-битом.
//   5. Скорость ±3 %       — ПК и плата никогда не совпадают по частоте точно.
//   6. Короткая помеха     — провал линии на 1/4 бита не должен давать байт.
//   7. Ошибка кадра        — стоп-бит = 0: нужен frame_err и никакого байта,
//                            потом линия "залипает" в 0, затем нормальный байт.
//   8. Строка команды      — "G1 X20 Y30,5 E3" + CR LF, как её шлёт Termite.
//
// Итог: в конце печатается TEST PASSED или TEST FAILED и число ошибок.
// =============================================================================
`timescale 1ns/1ps
module tb_uart_rx;
    localparam int  CLK_HZ  = 50_000_000;
    localparam int  BAUD    = 115_200;
    localparam real CLK_NS  = 1.0e9 / CLK_HZ;   // 20 нс
    localparam real BIT_NS  = 1.0e9 / BAUD;     // 8680,6 нс — длительность одного бита

    logic       clk = 1'b0;
    logic       rst = 1'b1;
    logic       rx  = 1'b1;     // линия в покое = 1
    logic [7:0] rx_data;
    logic       rx_valid;
    logic       frame_err;

    int errors        = 0;
    int bytes_checked = 0;

    uart_rx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) dut (.*);

    always #(CLK_NS / 2) clk = ~clk;

    // ---- Монитор: всё, что выдал приёмник, складываем в очередь ------------
    logic [7:0] got_q[$];        // принятые байты
    int         frame_err_cnt = 0;

    always @(posedge clk) begin
        if (rx_valid)  got_q.push_back(rx_data);
        if (frame_err) frame_err_cnt++;
    end

    // ---- "ПК": отправить байт по линии rx ------------------------------------
    // speed_scale = 1.03 значит "ПК шлёт на 3 % быстрее"
    task automatic send_byte(input logic [7:0] b,
                             input real      speed_scale = 1.0,
                             input logic     stop_bit    = 1'b1);
        real bit_ns;
        bit_ns = BIT_NS / speed_scale;
        rx = 1'b0;            #(bit_ns);             // старт-бит
        for (int i = 0; i < 8; i++) begin
            rx = b[i];        #(bit_ns);             // данные, младший бит первым
        end
        rx = stop_bit;        #(bit_ns);             // стоп-бит
        rx = 1'b1;                                   // линия снова в покое
    endtask

    // ---- Проверка: приёмник выдал ровно один байт, равный expected ----------
    task automatic expect_byte(input logic [7:0] expected, input string what);
        repeat (5) @(posedge clk);                   // запас на синхронизатор
        bytes_checked++;
        if (got_q.size() != 1) begin
            $display("ERROR [%s]: expected 1 byte 0x%02h, receiver gave %0d bytes",
                     what, expected, got_q.size());
            errors++;
        end else if (got_q[0] !== expected) begin
            $display("ERROR [%s]: expected 0x%02h, got 0x%02h", what, expected, got_q[0]);
            errors++;
        end
        got_q.delete();
    endtask

    task automatic expect_nothing(input string what);
        repeat (5) @(posedge clk);
        if (got_q.size() != 0) begin
            $display("ERROR [%s]: receiver gave %0d unexpected byte(s), first 0x%02h",
                     what, got_q.size(), got_q[0]);
            errors++;
        end
        got_q.delete();
    endtask

    // ---- Сценарии ------------------------------------------------------------
    logic [7:0] edge_bytes[6] = '{8'h00, 8'hFF, 8'h55, 8'hAA, 8'h01, 8'h80};
    string      cmd_line      = "G1 X20 Y30,5 E3";
    int         frame_err_before;

    initial begin
        repeat (5) @(posedge clk);
        rst = 1'b0;

        // 1. Тишина
        #(BIT_NS * 20);
        expect_nothing("1 idle line");
        $display("step 1: idle line - done");

        // 2. Граничные байты
        foreach (edge_bytes[i]) begin
            send_byte(edge_bytes[i]);
            expect_byte(edge_bytes[i], "2 edge bytes");
        end
        $display("step 2: edge bytes - done");

        // 3. Все 256 значений, с небольшой паузой между байтами
        for (int v = 0; v < 256; v++) begin
            send_byte(8'(v));
            #(BIT_NS / 2);
            expect_byte(8'(v), "3 all values");
        end
        $display("step 3: all 256 values - done");

        // 4. Подряд без пауз: 4 байта, проверяем после всех
        send_byte(8'h12); send_byte(8'h34); send_byte(8'h56); send_byte(8'h78);
        repeat (5) @(posedge clk);
        if (got_q.size() != 4 || got_q[0] != 8'h12 || got_q[1] != 8'h34 ||
            got_q[2] != 8'h56 || got_q[3] != 8'h78) begin
            $display("ERROR [4 back-to-back]: got %0d bytes", got_q.size());
            errors++;
        end
        bytes_checked += 4;
        got_q.delete();
        $display("step 4: back-to-back bytes - done");

        // 5. Скорость ±3 %
        send_byte(8'hA5, 1.03);  expect_byte(8'hA5, "5 baud +3%");
        send_byte(8'h3C, 1.03);  expect_byte(8'h3C, "5 baud +3%");
        send_byte(8'hA5, 0.97);  expect_byte(8'hA5, "5 baud -3%");
        send_byte(8'h3C, 0.97);  expect_byte(8'h3C, "5 baud -3%");
        $display("step 5: baud +-3%% - done");

        // 6. Короткая помеха: провал на 1/4 бита
        frame_err_before = frame_err_cnt;
        rx = 1'b0; #(BIT_NS / 4); rx = 1'b1;
        #(BIT_NS * 12);
        expect_nothing("6 glitch");
        if (frame_err_cnt != frame_err_before) begin
            $display("ERROR [6 glitch]: frame_err must stay 0"); errors++;
        end
        $display("step 6: short glitch ignored - done");

        // 7. Ошибка кадра, затем линия залипает в 0 на 3 бита, затем нормальный байт
        frame_err_before = frame_err_cnt;
        send_byte(8'h5A, 1.0, 1'b0);     // стоп-бит = 0
        rx = 1'b0; #(BIT_NS * 3);        // линия ещё "лежит" в нуле
        rx = 1'b1; #(BIT_NS * 2);        // вернулась в покой
        expect_nothing("7 frame error");
        if (frame_err_cnt != frame_err_before + 1) begin
            $display("ERROR [7 frame error]: expected exactly 1 frame_err, got %0d",
                     frame_err_cnt - frame_err_before);
            errors++;
        end
        send_byte(8'hC3);
        expect_byte(8'hC3, "7 byte after frame error");
        $display("step 7: frame error and recovery - done");

        // 8. Строка команды, как от Termite: текст, затем CR LF
        for (int i = 0; i < cmd_line.len(); i++) begin
            send_byte(cmd_line[i]);
            expect_byte(cmd_line[i], "8 command line");
        end
        send_byte(8'h0D); expect_byte(8'h0D, "8 CR");
        send_byte(8'h0A); expect_byte(8'h0A, "8 LF");
        $display("step 8: command line '%s' + CR LF - done", cmd_line);

        // Итог
        $display("------------------------------------------------");
        $display("bytes checked: %0d, frame errors seen: %0d", bytes_checked, frame_err_cnt);
        if (errors == 0) $display("TEST PASSED");
        else             $display("TEST FAILED: %0d errors", errors);
        $finish;
    end

    // Сторож: если что-то зависло, симуляция не будет идти вечно
    initial begin
        #(BIT_NS * 10 * 600);
        $display("ERROR: simulation timeout");
        errors++;
        $display("TEST FAILED: %0d errors", errors);
        $finish;
    end

endmodule
