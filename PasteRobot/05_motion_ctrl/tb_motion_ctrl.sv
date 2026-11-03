// =============================================================================
// tb_motion_ctrl.sv — тест контроллера движения
// -----------------------------------------------------------------------------
// Тестбенч изображает "механику" робота: считает импульсы PUL на каждой оси,
// двигает виртуальную каретку phys[] по DIR и нажимает концевик, когда
// каретка дошла до нуля. Команды подаются прямо структурой cmd_t (как от парсера).
//
// Чтобы симуляция шла быстро, все длительности уменьшены: период шага 20 тактов,
// импульс 4 такта, пауза после DIR 6 тактов. Логика от этого не меняется.
//
// На КАЖДЫЙ импульс проверяется:
//   * DIR выставлен заранее, не меньше чем за DIR_SETUP_CLKS тактов;
//   * длительность импульса ровно PULSE_CLKS;
//   * период не меньше CLKS_PER_STEP;
//   * DIR не меняется, пока PUL = 1;
//   * при G0/G1 число шагов каждой оси в каждый момент совпадает с формулой
//     Брезенхема: после k периодов ось с числом n сделала (n_max/2 + k·n) / n_max шагов.
// После каждой команды: ответ (ok / err N), число импульсов и направление по
// каждой оси, координаты pos_x/y/z.
//
// Сценарии: хоминг (порядок Z, X, Y), M92, G0, G1 с пастой, втягивание пасты,
// движение в ноль, упор в концевик (err 5), округление дозы, пустая команда,
// строка с ошибкой парсера, хоминг одной оси, обрыв концевика (err 6),
// слишком большое число (err 3), долгий ответ на ПК (res_ready = 0).
// =============================================================================
`timescale 1ns/1ps
module tb_motion_ctrl;
    import robot_pkg::*;

    localparam real CLK_NS             = 20.0;
    localparam int  CLKS_PER_STEP      = 20;
    localparam int  CLKS_PER_HOME_STEP = 24;
    localparam int  PULSE_CLKS         = 4;
    localparam int  DIR_SETUP_CLKS     = 6;
    localparam int  HOME_MAX_STEPS     = 500;

    logic                 clk = 1'b0;
    logic                 rst = 1'b1;
    logic                 cmd_valid = 1'b0;
    cmd_t                 cmd = '0;
    logic                 cmd_ready;
    logic                 res_valid;
    err_t                 res_err;
    logic                 res_ready = 1'b1;
    logic [2:0]           limit;
    logic [NUM_AXES-1:0]  step_pul, step_dir;
    logic                 busy;
    logic signed [31:0]   pos_x, pos_y, pos_z;

    int errors = 0;

    motion_ctrl #(
        .CLKS_PER_STEP      (CLKS_PER_STEP),
        .CLKS_PER_HOME_STEP (CLKS_PER_HOME_STEP),
        .PULSE_CLKS         (PULSE_CLKS),
        .DIR_SETUP_CLKS     (DIR_SETUP_CLKS),
        .HOME_MAX_STEPS     (HOME_MAX_STEPS),
        .HOME_DIR           (3'b000)            // концевики в сторону минуса
    ) dut (.*);

    always #(CLK_NS / 2) clk = ~clk;

    // =========================================================================
    // "Механика": каретки и концевики
    // =========================================================================
    int  phys [NUM_AXES] = '{137, 55, 20, 0};   // где каретки стоят на самом деле (в шагах)
    bit  switch_ok [3]   = '{1, 1, 1};          // 0 = провод концевика оборван

    always_comb
        for (int a = 0; a < 3; a++)
            limit[a] = switch_ok[a] && (phys[a] <= 0);

    // =========================================================================
    // Проверки каждого импульса (по событиям на выводах)
    // Время меряем в тактах clk: счётчик cycle растёт на каждом фронте.
    // =========================================================================
    longint  cycle = 0;
    always @(posedge clk) cycle++;

    int      cnt        [NUM_AXES];             // импульсов за текущую команду
    logic    dir_seen   [NUM_AXES];             // DIR во время импульсов этой команды
    bit      dir_mixed  [NUM_AXES];             // DIR менялся между импульсами одной команды
    longint  c_first    [NUM_AXES];             // такт первого импульса (для порядка хоминга)

    // Для проверки Брезенхема
    bit      bres_on = 0;
    int      bres_n [NUM_AXES];
    int      bres_nmax;

    for (genvar a = 0; a < NUM_AXES; a++) begin : g_axis
        longint c_dir  = 0;        // такт, когда последний раз менялся DIR
        longint c_rise = -1;       // такт предыдущего фронта PUL (-1: ещё не было)

        always @(step_dir[a]) begin
            c_dir = cycle;
            if (step_pul[a]) begin
                $display("ERROR: axis %0d: DIR changed while PUL = 1 at %0t", a, $time);
                errors++;
            end
        end

        always @(posedge step_pul[a]) begin
            if (cycle - c_dir < DIR_SETUP_CLKS) begin
                $display("ERROR: axis %0d: DIR set only %0d clocks before PUL", a, cycle - c_dir);
                errors++;
            end
            if (c_rise >= 0 && cycle - c_rise < CLKS_PER_STEP) begin
                $display("ERROR: axis %0d: step period %0d clocks is too short", a, cycle - c_rise);
                errors++;
            end
            c_rise = cycle;
            if (cnt[a] == 0) begin
                c_first[a]  = cycle;
                dir_seen[a] = step_dir[a];
            end else if (step_dir[a] !== dir_seen[a])
                dir_mixed[a] = 1;
            cnt[a]++;
            phys[a] += step_dir[a] ? 1 : -1;
        end

        always @(negedge step_pul[a]) begin
            // c_rise < 0: импульсов ещё не было (это переход X -> 0 при сбросе)
            if (c_rise >= 0 && cycle - c_rise != PULSE_CLKS) begin
                $display("ERROR: axis %0d: PUL width %0d clocks, expected %0d",
                         a, cycle - c_rise, PULSE_CLKS);
                errors++;
            end
        end
    end

    // Брезенхем: через четверть такта после фронтов сверяем счётчики с формулой
    always @(step_pul) begin
        if (bres_on && step_pul != '0) begin
            #(CLK_NS / 4);
            for (int a = 0; a < NUM_AXES; a++) begin
                automatic longint k_periods = 0;
                for (int m = 0; m < NUM_AXES; m++)
                    if (bres_n[m] == bres_nmax) k_periods = cnt[m];   // главная ось шагает каждый период
                if (cnt[a] != (bres_nmax / 2 + k_periods * bres_n[a]) / bres_nmax) begin
                    $display("ERROR: axis %0d: after %0d periods made %0d steps, Bresenham says %0d",
                             a, k_periods, cnt[a], (bres_nmax / 2 + k_periods * bres_n[a]) / bres_nmax);
                    errors++;
                end
            end
        end
    end

    // =========================================================================
    // Эталонная модель: что должен посчитать контроллер
    // =========================================================================
    longint model_k   [NUM_AXES] = '{40000, 40000, 40000, 9220};  // значения после сброса
    longint model_pos [3]        = '{0, 0, 0};                     // pos_x/y/z контроллера

    function automatic longint mm_to_steps(input longint v, input longint k);
        longint m = (v < 0) ? -v : v;
        return (m * k + 5000) / 10000;
    endfunction

    // Собрать команду так, как её выдал бы парсер
    function automatic cmd_t make_cmd(input opcode_t op, input logic [3:0] has,
                                      input int x = 0, input int y = 0,
                                      input int z = 0, input int e = 0);
        cmd_t c = '0;
        c.opcode       = op;
        c.err          = ERR_NONE;
        c.has          = has;
        c.value[AX_X]  = x;
        c.value[AX_Y]  = y;
        c.value[AX_Z]  = z;
        c.value[AX_E]  = e;
        return c;
    endfunction

    localparam logic [3:0] F_NONE = 4'b0000, F_X = 4'b0001, F_Y = 4'b0010,
                           F_Z = 4'b0100,    F_E = 4'b1000;

    // =========================================================================
    // Выполнить команду и проверить результат
    // =========================================================================
    //   exp_n[a]   — сколько импульсов ждём на оси a
    //   exp_dir[a] — какое направление ждём (смотрим только там, где exp_n > 0)
    //   check_bres — включить проверку Брезенхема (для G0/G1)
    //   hold_res   — сколько тактов держать res_ready = 0 (модуль ответа занят)
    task automatic run_cmd(input string    name,
                           input cmd_t     c,
                           input err_t     exp_err,
                           input int       exp_n [NUM_AXES],
                           input logic [3:0] exp_dir,
                           input bit       check_bres = 0,
                           input int       hold_res   = 0);
        bit bad = 0;
        int nmax = 0;

        foreach (cnt[a]) begin
            cnt[a]       = 0;
            dir_mixed[a] = 0;
            bres_n[a]    = exp_n[a];
            if (exp_n[a] > nmax) nmax = exp_n[a];
        end
        bres_nmax = nmax;
        bres_on   = check_bres && (exp_err == ERR_NONE) && (nmax > 0);

        // Отдать команду (рукопожатие valid/ready)
        @(negedge clk);
        cmd_valid = 1'b1;
        cmd       = c;
        while (!cmd_ready) @(negedge clk);
        @(negedge clk);
        cmd_valid = 1'b0;

        // Дождаться результата
        while (!res_valid) @(negedge clk);
        if (hold_res > 0) begin
            res_ready = 1'b0;
            repeat (hold_res) begin
                @(negedge clk);
                if (!res_valid || cmd_ready) begin
                    $display("ERROR [%s]: result must wait while res_ready = 0", name);
                    errors++;
                end
            end
            res_ready = 1'b1;
        end
        if (res_err !== exp_err) begin
            $display("ERROR [%s]: result %s, expected %s", name, res_err.name(), exp_err.name());
            bad = 1;
        end
        @(negedge clk);
        bres_on = 0;

        // Импульсы и направления
        for (int a = 0; a < NUM_AXES; a++) begin
            if (cnt[a] != exp_n[a]) begin
                $display("ERROR [%s]: axis %0d made %0d pulses, expected %0d", name, a, cnt[a], exp_n[a]);
                bad = 1;
            end
            if (exp_n[a] > 0 && (dir_seen[a] !== exp_dir[a] || dir_mixed[a])) begin
                $display("ERROR [%s]: axis %0d DIR %b (mixed=%0b), expected %b",
                         name, a, dir_seen[a], dir_mixed[a], exp_dir[a]);
                bad = 1;
            end
        end

        // Координаты
        if (pos_x != model_pos[AX_X] || pos_y != model_pos[AX_Y] || pos_z != model_pos[AX_Z]) begin
            $display("ERROR [%s]: pos = (%0d, %0d, %0d), expected (%0d, %0d, %0d)", name,
                     pos_x, pos_y, pos_z, model_pos[AX_X], model_pos[AX_Y], model_pos[AX_Z]);
            bad = 1;
        end

        if (bad) errors++;
        else $display("OK   %-28s -> %-10s pulses X=%0d Y=%0d Z=%0d E=%0d", name,
                      res_err.name(), cnt[AX_X], cnt[AX_Y], cnt[AX_Z], cnt[AX_E]);
    endtask

    // Посчитать ожидания для G0/G1 по модели и выполнить команду
    task automatic run_move(input string name, input cmd_t c, input bit check_bres = 1);
        int         exp_n [NUM_AXES] = '{0, 0, 0, 0};
        logic [3:0] exp_dir = '0;
        longint     target [3];
        for (int a = 0; a < 3; a++) begin
            target[a] = model_pos[a];
            if (c.has[a]) begin
                target[a] = mm_to_steps($signed(c.value[a]), model_k[a]);
                if ($signed(c.value[a]) < 0) target[a] = -target[a];
            end
            exp_n[a]   = (target[a] > model_pos[a]) ? target[a] - model_pos[a]
                                                    : model_pos[a] - target[a];
            exp_dir[a] = (target[a] > model_pos[a]);
        end
        if (c.has[AX_E]) begin
            exp_n[AX_E]   = mm_to_steps($signed(c.value[AX_E]), model_k[AX_E]);
            exp_dir[AX_E] = ($signed(c.value[AX_E]) >= 0);
        end
        for (int a = 0; a < 3; a++) model_pos[a] = target[a];
        run_cmd(name, c, ERR_NONE, exp_n, exp_dir, check_bres);
    endtask

    // =========================================================================
    // Сценарии
    // =========================================================================
    initial begin
        repeat (3) @(posedge clk);
        @(negedge clk) rst = 1'b0;

        // 1. Хоминг всех осей: каретки стоят на 137, 55, 20 шагов от концевиков
        model_pos = '{0, 0, 0};
        run_cmd("G28", make_cmd(OP_G28, F_NONE), ERR_NONE, '{137, 55, 20, 0}, 4'b0000);
        if (!(c_first[AX_Z] < c_first[AX_X] && c_first[AX_X] < c_first[AX_Y])) begin
            $display("ERROR [G28]: homing order must be Z, X, Y"); errors++;
        end else
            $display("OK   homing order Z -> X -> Y");

        // 2. Коэффициенты: 10 имп/мм по осям, 10 имп/мкл для пасты (числа в сотых)
        run_cmd("M92 X10 Y10 Z10 E10", make_cmd(OP_M92, F_X|F_Y|F_Z|F_E, 1000, 1000, 1000, 1000),
                ERR_NONE, '{0, 0, 0, 0}, 4'b0000);
        model_k = '{1000, 1000, 1000, 1000};

        // 3. Переезды и движение с пастой
        run_move("G0 X10 Y5",        make_cmd(OP_G0, F_X|F_Y, 1000, 500));
        run_move("G1 X20 Y8 E2",     make_cmd(OP_G1, F_X|F_Y|F_E, 2000, 800, 0, 200));
        run_move("G1 E-1.5 (retract)", make_cmd(OP_G1, F_E, 0, 0, 0, -150));
        run_move("G0 Z3",            make_cmd(OP_G0, F_Z, 0, 0, 300));
        run_move("G0 Z0",            make_cmd(OP_G0, F_Z, 0, 0, 0));
        run_move("G0 X0 Y0",         make_cmd(OP_G0, F_X|F_Y, 0, 0));

        // 4. Ось X стоит на концевике: ехать в минус нельзя
        run_cmd("G0 X-1 (into switch)", make_cmd(OP_G0, F_X, -100), ERR_LIMIT,
                '{0, 0, 0, 0}, 4'b0000);
        run_move("G0 X5.5",          make_cmd(OP_G0, F_X, 550));

        // 5. Пустая команда и округление дозы
        run_move("G1 (no fields)",   make_cmd(OP_G1, F_NONE));
        run_cmd("M92 E92.2", make_cmd(OP_M92, F_E, 0, 0, 0, 9220), ERR_NONE, '{0, 0, 0, 0}, 4'b0000);
        model_k[AX_E] = 9220;
        run_move("G1 E2  (184.4 -> 184)", make_cmd(OP_G1, F_E, 0, 0, 0, 200));
        run_move("G1 E0.05 (4.61 -> 5)",  make_cmd(OP_G1, F_E, 0, 0, 0, 5));

        // 6. Строка, в которой парсер нашёл ошибку: только ответ, без движения
        begin
            automatic cmd_t bad_line = make_cmd(OP_NONE, F_NONE);
            bad_line.err = ERR_SYNTAX;
            run_cmd("<line with syntax error>", bad_line, ERR_SYNTAX, '{0, 0, 0, 0}, 4'b0000);
        end

        // 7. Хоминг одной оси
        run_move("G0 Y3", make_cmd(OP_G0, F_Y, 0, 300));
        model_pos[AX_Y] = 0;
        run_cmd("G28 Y", make_cmd(OP_G28, F_Y), ERR_NONE, '{0, 30, 0, 0}, 4'b0000);

        // 8. Оборван провод концевика X: хоминг не находит ноль
        switch_ok[AX_X] = 0;
        run_cmd("G28 X (broken switch)", make_cmd(OP_G28, F_X), ERR_HOMING,
                '{HOME_MAX_STEPS, 0, 0, 0}, 4'b0000);
        switch_ok[AX_X] = 1;                     // починили: каретка уже за нулём, концевик нажат
        model_pos[AX_X] = 0;
        run_cmd("G28 X (repaired)", make_cmd(OP_G28, F_X), ERR_NONE, '{0, 0, 0, 0}, 4'b0000);

        // 9. Слишком много импульсов: k = 99999,99 и X = 99999,99 мм
        run_cmd("M92 X99999.99", make_cmd(OP_M92, F_X, 9_999_999), ERR_NONE, '{0, 0, 0, 0}, 4'b0000);
        run_cmd("G0 X99999.99 (too far)", make_cmd(OP_G0, F_X, 9_999_999), ERR_NUMBER,
                '{0, 0, 0, 0}, 4'b0000);
        run_cmd("M92 X10", make_cmd(OP_M92, F_X, 1000), ERR_NONE, '{0, 0, 0, 0}, 4'b0000);

        // 10. Модуль ответа занят 50 тактов: результат ждёт, следующая команда тоже
        begin
            automatic int exp_n [NUM_AXES] = '{20, 0, 0, 0};
            model_pos[AX_X] = 20;
            run_cmd("G0 X2 (slow reply)", make_cmd(OP_G0, F_X, 200), ERR_NONE, exp_n, 4'b0001, 1, 50);
            if (pos_x != 20) begin
                $display("ERROR: pos_x = %0d after slow reply, expected 20", pos_x); errors++;
            end
        end

        $display("------------------------------------------------");
        if (errors == 0) $display("TEST PASSED");
        else             $display("TEST FAILED: %0d errors", errors);
        $finish;
    end

    initial begin
        #(CLK_NS * 2_000_000);
        $display("ERROR: simulation timeout");
        errors++;
        $finish;
    end

endmodule
