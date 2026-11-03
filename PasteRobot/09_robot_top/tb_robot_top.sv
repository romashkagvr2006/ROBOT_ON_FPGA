// =============================================================================
// tb_robot_top.sv — сквозной тест всей цепочки, как на настоящем роботе
// -----------------------------------------------------------------------------
// Тестбенч играет сразу три роли:
//   * ПК с Termite: шлёт строки команд по линии uart_rxd на 115200 бод
//     (бит за битом, как настоящий COM-порт) и читает ответы с uart_txd;
//   * механику: считает импульсы motor_pul, двигает каретки по motor_dir;
//   * концевики: когда каретка доехала до нуля, вывод концевика падает в 0
//     (LIMIT_ACTIVE_LOW = 1, как у концевика, замыкающего на землю).
//
// Частота шагов увеличена до 250 кГц, импульс и пауза DIR — 1 мкс,
// антидребезг — 1 мкс: иначе симуляция шла бы минуты. UART — настоящий, 115200.
//
// Сценарий (каждая строка — Enter в Termite):
//   G28                   -> ok,     хоминг: X 40, Y 25, Z 10 шагов к концевикам
//   M92 X10 Y10 Z10 E10   -> ok,     10 импульсов на мм / на мкл
//   G0 X10 Y5             -> ok,     X 100, Y 50 импульсов
//   G1 X20 Y8 E2          -> ok,     X 100, Y 30, E 20
//   G0 X1 E2              -> err 4,  G0 с пастой нельзя; горит led_error
//   G5                    -> err 1,  нет такой команды
//   (пустая строка)       -> ответа нет
//   G0 X0 Y0              -> ok,     led_error погас
//   G0 X-1                -> err 5,  X стоит на концевике
//   G0 Z1 и G0 Z0 подряд, не дожидаясь ответа -> ok, ok (вторая ждала в FIFO)
//   g1 e-0,5              -> ok,     маленькие буквы, запятая, втягивание пасты
// =============================================================================
`timescale 1ns/1ps
module tb_robot_top;
    import robot_pkg::*;

    localparam int  CLK_HZ = 50_000_000;
    localparam int  BAUD   = 115_200;
    localparam real CLK_NS = 1.0e9 / CLK_HZ;
    localparam real BIT_NS = 1.0e9 / BAUD;

    logic                clk_50    = 1'b0;
    logic                key_rst_n = 1'b0;     // кнопка сброса нажата
    logic                uart_rxd  = 1'b1;     // линия от ПК в покое
    logic                uart_txd;
    logic                limit_x_raw, limit_y_raw, limit_z_raw;
    logic [NUM_AXES-1:0] motor_pul, motor_dir, motor_ena;
    logic                led_busy, led_error, led_overflow;

    int errors = 0;

    robot_top #(
        .CLK_HZ           (CLK_HZ),
        .BAUD             (BAUD),
        .STEP_HZ          (250_000),
        .HOME_STEP_HZ     (250_000),
        .PULSE_US         (1),
        .DIR_SETUP_US     (1),
        .DEBOUNCE_US      (1),
        .HOME_MAX_STEPS   (2_000),
        .LIMIT_ACTIVE_LOW (1'b1),
        .LED_ON_LEVEL     (1'b0)
    ) dut (.*);

    always #(CLK_NS / 2) clk_50 = ~clk_50;

    // =========================================================================
    // Механика и концевики
    // =========================================================================
    int phys [NUM_AXES] = '{40, 25, 10, 0};   // где каретки стоят после включения (в шагах)
    int cnt  [NUM_AXES];                      // импульсов за команду
    int dir_last [NUM_AXES];

    for (genvar a = 0; a < NUM_AXES; a++) begin : g_axis
        always @(posedge motor_pul[a]) begin
            cnt[a]++;
            dir_last[a] = motor_dir[a];
            phys[a] += motor_dir[a] ? 1 : -1;
        end
    end

    // Концевик замыкает на землю, когда каретка в нуле или дальше
    assign limit_x_raw = !(phys[AX_X] <= 0);
    assign limit_y_raw = !(phys[AX_Y] <= 0);
    assign limit_z_raw = !(phys[AX_Z] <= 0);

    // =========================================================================
    // "Termite": отправка строк и приём ответов
    // =========================================================================
    task automatic pc_send_byte(input logic [7:0] b);
        uart_rxd = 1'b0;  #(BIT_NS);                 // старт-бит
        for (int i = 0; i < 8; i++) begin
            uart_rxd = b[i]; #(BIT_NS);              // данные, младший бит первым
        end
        uart_rxd = 1'b1;  #(BIT_NS);                 // стоп-бит
    endtask

    task automatic pc_send_line(input string s);
        for (int i = 0; i < s.len(); i++) pc_send_byte(s[i]);
        pc_send_byte(8'h0D);                         // Enter: CR LF
        pc_send_byte(8'h0A);
    endtask

    string replies_q[$];
    string cur_reply = "";

    initial begin
        logic [7:0] ch;
        forever begin
            @(negedge uart_txd);
            #(BIT_NS / 2);
            for (int i = 0; i < 8; i++) begin
                #(BIT_NS);
                ch[i] = uart_txd;
            end
            #(BIT_NS);
            if (uart_txd !== 1'b1) begin
                $display("ERROR: no stop bit in reply"); errors++;
            end
            if (ch == 8'h0A) begin
                replies_q.push_back(cur_reply);
                cur_reply = "";
            end else if (ch != 8'h0D)
                cur_reply = {cur_reply, string'(ch)};
        end
    end

    // Дождаться n ответов (не дольше max_ms миллисекунд)
    task automatic wait_replies(input int n, input real max_ms = 50.0);
        real waited = 0;
        while (replies_q.size() < n && waited < max_ms * 1.0e6) begin
            #(BIT_NS);
            waited += BIT_NS;
        end
    endtask

    // =========================================================================
    // Отправить строку, дождаться ответа, сверить ответ и импульсы
    // =========================================================================
    task automatic run(input string line, input string exp_reply,
                       input int exp_n [NUM_AXES]);
        bit bad = 0;
        foreach (cnt[a]) cnt[a] = 0;
        pc_send_line(line);
        wait_replies(1);
        if (replies_q.size() != 1) begin
            $display("ERROR: \"%s\": expected 1 reply, got %0d", line, replies_q.size());
            bad = 1;
        end else if (replies_q[0] != exp_reply) begin
            $display("ERROR: \"%s\": reply \"%s\", expected \"%s\"", line, replies_q[0], exp_reply);
            bad = 1;
        end
        foreach (cnt[a])
            if (cnt[a] != exp_n[a]) begin
                $display("ERROR: \"%s\": axis %0d made %0d pulses, expected %0d",
                         line, a, cnt[a], exp_n[a]);
                bad = 1;
            end
        if (bad) errors++;
        else $display("OK   %-22s -> \"%s\"   pulses X=%0d Y=%0d Z=%0d E=%0d",
                      line, exp_reply, cnt[AX_X], cnt[AX_Y], cnt[AX_Z], cnt[AX_E]);
        replies_q.delete();
    endtask

    // =========================================================================
    // Сценарий
    // =========================================================================
    initial begin
        #(CLK_NS * 10);
        key_rst_n = 1'b1;                            // отпустили кнопку сброса
        #(CLK_NS * 10);

        if (uart_txd !== 1'b1)        begin $display("ERROR: TX line not idle"); errors++; end
        if (motor_ena !== 4'b0000)    begin $display("ERROR: drivers not enabled"); errors++; end

        run("G28",                 "ok",    '{40, 25, 10, 0});
        if (phys[AX_X] != 0 || phys[AX_Y] != 0 || phys[AX_Z] != 0) begin
            $display("ERROR: after G28 carriages at %0d %0d %0d", phys[0], phys[1], phys[2]);
            errors++;
        end
        run("M92 X10 Y10 Z10 E10", "ok",    '{0, 0, 0, 0});
        run("G0 X10 Y5",           "ok",    '{100, 50, 0, 0});
        run("G1 X20 Y8 E2",        "ok",    '{100, 30, 0, 20});

        run("G0 X1 E2",            "err 4", '{0, 0, 0, 0});
        if (led_error !== 1'b0) begin $display("ERROR: led_error must be ON after err"); errors++; end
        run("G5",                  "err 1", '{0, 0, 0, 0});

        // Пустая строка: ответа быть не должно
        pc_send_line("");
        wait_replies(1, 1.0);
        if (replies_q.size() != 0) begin
            $display("ERROR: empty line got reply \"%s\"", replies_q[0]); errors++;
            replies_q.delete();
        end else
            $display("OK   %-22s -> (no reply)", "<empty line>");

        run("G0 X0 Y0",            "ok",    '{200, 80, 0, 0});
        if (led_error !== 1'b1) begin $display("ERROR: led_error must be OFF after ok"); errors++; end
        run("G0 X-1",              "err 5", '{0, 0, 0, 0});

        // Две строки подряд, не дожидаясь ответа: вторая ждёт в FIFO
        foreach (cnt[a]) cnt[a] = 0;
        pc_send_line("G0 Z1");
        pc_send_line("G0 Z0");
        wait_replies(2);
        if (replies_q.size() != 2 || replies_q[0] != "ok" || replies_q[1] != "ok" || cnt[AX_Z] != 20) begin
            $display("ERROR: two lines back-to-back: %0d replies, Z pulses %0d (expected ok, ok, 20)",
                     replies_q.size(), cnt[AX_Z]);
            errors++;
        end else
            $display("OK   %-22s -> \"ok\", \"ok\"  pulses Z=%0d (10 up + 10 down)",
                     "G0 Z1 + G0 Z0", cnt[AX_Z]);
        replies_q.delete();

        run("g1 e-0,5",            "ok",    '{0, 0, 0, 5});
        if (dir_last[AX_E] != 0) begin $display("ERROR: E must move back (DIR = 0)"); errors++; end

        if (led_overflow !== 1'b1) begin $display("ERROR: FIFO overflow LED is on"); errors++; end
        if (led_busy     !== 1'b1) begin $display("ERROR: busy LED still on"); errors++; end

        $display("------------------------------------------------");
        $display("carriages at X=%0d Y=%0d Z=%0d steps, E moved %0d", phys[0], phys[1], phys[2], phys[3]);
        if (errors == 0) $display("TEST PASSED");
        else             $display("TEST FAILED: %0d errors", errors);
        $finish;
    end

    initial begin
        #(1.0e9);    // 1 секунда модельного времени
        $display("ERROR: simulation timeout");
        errors++;
        $finish;
    end

endmodule
