// =============================================================================
// motion_ctrl.sv — контроллер движения: команда -> импульсы PUL/DIR на драйверы
// -----------------------------------------------------------------------------
// ЧТО ДЕЛАЕТ
//   Берёт у парсера готовую команду cmd_t и выполняет её:
//     M92  — запоминает коэффициенты k (сколько импульсов на 1 мм / на 1 мкл);
//     G0/G1 — переводит миллиметры в импульсы и выдаёт импульсы на моторы
//             X, Y, Z, E так, чтобы все оси закончили движение одновременно;
//     G28  — хоминг: по очереди Z, X, Y едут к концевикам, там координата = 0.
//   Когда команда выполнена, отдаёт результат (ok или код ошибки) модулю ответа.
//
// ОТКУДА БЕРЁТ И КУДА ОТДАЁТ
//   cmd_valid / cmd / cmd_ready   <- cmd_parser (рукопожатие valid/ready).
//                                    cmd_ready = 1 только в покое, поэтому
//                                    следующая строка ждёт, пока моторы едут.
//   res_valid / res_err / res_ready -> reply_gen (он шлёт "ok" или "err N").
//   limit[2:0]   <- концевики X, Y, Z после антидребезга (1 = нажат).
//   step_pul[3:0], step_dir[3:0] -> выводы ПЛИС -> драйверы TB6600 (PUL, DIR).
//                  Индексы: [0]=X, [1]=Y, [2]=Z, [3]=E.
//   pos_x/y/z    -> текущие координаты в импульсах (для отладки и телеметрии).
//
// ПЕРЕВОД ММ В ИМПУЛЬСЫ (числа в сотых, см. robot_pkg)
//   импульсов = (|значение| × k + 5000) / 10000      (+5000 — это округление)
//   Пример: X20 (2000) при k = 400,00 (40000): (2000 × 40000 + 5000) / 10000 = 8000.
//   X, Y, Z заданы абсолютно: цель = ±импульсов, ехать надо (цель - текущая).
//   E задан на эту команду: выдавить ровно столько, знак '-' — втянуть пасту.
//   Деление делает отдельный модуль udiv (по биту за такт).
//
// СОГЛАСОВАНИЕ ОСЕЙ (алгоритм Брезенхема)
//   Ось с наибольшим числом импульсов n_max шагает в каждом периоде. Каждая
//   ось копит в накопителе acc своё число n; когда acc >= n_max — эта ось
//   шагает, и из acc вычитается n_max. За n_max периодов ось с числом n
//   шагнёт ровно n раз, и шаги размажутся равномерно по всему пути.
//   Так паста ложится равномерно по длине дорожки.
//
// ВРЕМЕННАЯ ДИАГРАММА ОДНОГО ШАГА (все длительности — параметры в тактах)
//
//   DIR  ==X=================================  (выставлен заранее, DIR_SETUP_CLKS)
//   PUL  ___/‾‾‾‾‾‾‾‾\___________________/‾‾‾‾
//          |PULSE_CLKS|                  |
//          |<-------- CLKS_PER_STEP ---->|
//
// БЕЗОПАСНОСТЬ
//   Если ось должна шагнуть В СТОРОНУ своего концевика, а он нажат, движение
//   прерывается: результат ERR_LIMIT ("err 5"). При хоминге, если концевик не
//   нажался за HOME_MAX_STEPS шагов, — ERR_HOMING ("err 6").
//
// КОНЕЧНЫЙ АВТОМАТ
//
//   Команда G0 / G1:
//     S_IDLE -> S_CALC_MUL -> S_CALC_DIV -> S_CALC_WAIT --+   по очереди оси X, Y, Z, E
//                  ^--------- следующая ось --------------+
//     -> S_FIND_MAX -> S_PREPARE -> S_DIR_SETUP -> S_STEP -> S_PULSE_HIGH -> S_PULSE_LOW
//                                                    ^                            |
//                                                    +-------- ещё шаги ----------+
//                                                    все шаги сделаны -> S_DONE
//   Команда G28:
//     S_IDLE -> S_HOME_NEXT -> S_DIR_SETUP -> S_HOME_STEP -> S_PULSE_HIGH -> S_PULSE_LOW
//                  ^                             |     ^                          |
//                  +-- концевик нажат: pos = 0 --+     +-------- ещё шаг ---------+
//               осей не осталось -> S_DONE
//   Команда M92 или строка с ошибкой:  S_IDLE -> S_DONE
//   S_DONE ждёт, пока reply_gen заберёт результат (res_ready = 1), затем S_IDLE.
// =============================================================================
module motion_ctrl
    import robot_pkg::*;
#(
    parameter int CLKS_PER_STEP      = 25_000,   // период импульсов при движении (50 МГц / 2 кГц)
    parameter int CLKS_PER_HOME_STEP = 50_000,   // период импульсов при хоминге (1 кГц, медленнее)
    parameter int PULSE_CLKS         = 500,      // длительность импульса PUL (10 мкс)
    parameter int DIR_SETUP_CLKS     = 500,      // пауза между сменой DIR и первым PUL (10 мкс)
    parameter int HOME_MAX_STEPS     = 400_000,  // сколько шагов искать концевик до ошибки
    parameter logic [2:0] HOME_DIR   = 3'b000,   // уровень DIR "к концевику" для Z,Y,X (бит 0 = X)
    // Коэффициенты после сброса, в сотых: 40000 = 400,00 имп/мм, 9220 = 92,20 имп/мкл
    parameter int K_X_DEFAULT        = 40_000,
    parameter int K_Y_DEFAULT        = 40_000,
    parameter int K_Z_DEFAULT        = 40_000,
    parameter int K_E_DEFAULT        = 9_220
) (
    input  logic                 clk,
    input  logic                 rst,          // сброс, активная 1
    // ---- команда от парсера ----
    input  logic                 cmd_valid,
    input  cmd_t                 cmd,
    output logic                 cmd_ready,
    // ---- результат для ответа на ПК ----
    output logic                 res_valid,
    output err_t                 res_err,
    input  logic                 res_ready,
    // ---- концевики после антидребезга, 1 = нажат: [0]=X [1]=Y [2]=Z ----
    input  logic [2:0]           limit,
    // ---- к драйверам TB6600: [0]=X [1]=Y [2]=Z [3]=E ----
    output logic [NUM_AXES-1:0]  step_pul,
    output logic [NUM_AXES-1:0]  step_dir,     // 1 = в плюс, 0 = в минус
    // ---- состояние ----
    output logic                 busy,
    output logic signed [31:0]   pos_x,        // текущие координаты в импульсах
    output logic signed [31:0]   pos_y,
    output logic signed [31:0]   pos_z
);
    // ---- Разрядности ----------------------------------------------------------
    localparam int K_W        = 24;              // k до 99999,99 -> 9 999 999 < 2^24
    localparam int MAG_W      = 24;              // |значение| до 9 999 999 < 2^24
    localparam int PROD_W     = K_W + MAG_W;     // произведение 48 бит
    localparam int DIV_N_W    = PROD_W + 2;      // делимое с запасом под +5000
    localparam int DIV_D_W    = 14;              // 10000 < 2^14
    localparam int STEP_LIMIT = (1 << 30) - 1;   // больше импульсов за команду не бывает
    localparam int TIMER_W    = 32;

    // Длительность паузы после импульса: весь период минус импульс минус 1 такт
    // на состояние S_STEP / S_HOME_STEP
    localparam int STEP_LOW_CLKS = CLKS_PER_STEP      - PULSE_CLKS - 1;
    localparam int HOME_LOW_CLKS = CLKS_PER_HOME_STEP - PULSE_CLKS - 1;

    // synthesis translate_off
    initial begin
        assert (STEP_LOW_CLKS >= 1 && HOME_LOW_CLKS >= 1)
            else $error("motion_ctrl: step period must be at least PULSE_CLKS + 2 clocks");
        assert (PULSE_CLKS >= 1 && DIR_SETUP_CLKS >= 1)
            else $error("motion_ctrl: PULSE_CLKS and DIR_SETUP_CLKS must be >= 1");
    end
    // synthesis translate_on

    // ---- Состояния автомата -----------------------------------------------------
    typedef enum logic [3:0] {
        S_IDLE,        // ждём команду
        S_CALC_MUL,    // ось calc_axis: |значение| × k
        S_CALC_DIV,    // запускаем деление на 10000
        S_CALC_WAIT,   // ждём частное, считаем число импульсов и направление
        S_FIND_MAX,    // ищем самую длинную ось n_max (отдельный такт: так быстрее по времени)
        S_PREPARE,     // накопители, выставляем DIR
        S_DIR_SETUP,   // выдерживаем паузу после смены DIR
        S_STEP,        // какие оси шагают в этом периоде (Брезенхем), проверка концевиков
        S_PULSE_HIGH,  // PUL = 1 на PULSE_CLKS тактов
        S_PULSE_LOW,   // PUL = 0 до конца периода
        S_HOME_NEXT,   // хоминг: выбираем следующую ось (порядок Z, X, Y)
        S_HOME_STEP,   // хоминг: концевик нажат? если нет — ещё шаг
        S_DONE         // отдаём результат модулю ответа
    } state_t;

    state_t state, state_next;

    // ---- Регистры и их "следующие" значения ------------------------------------------
    logic [NUM_AXES-1:0]   has,        has_next;          // поля текущей команды
    value_t [NUM_AXES-1:0] value,      value_next;
    err_t                  result,     result_next;       // чем закончится команда

    logic [K_W-1:0]        k          [NUM_AXES];         // коэффициенты, в сотых
    logic [K_W-1:0]        k_next     [NUM_AXES];
    logic signed [31:0]    pos        [3];                // координаты X, Y, Z в импульсах
    logic signed [31:0]    pos_next   [3];
    logic [31:0]           n          [NUM_AXES];         // импульсов на ось в этой команде
    logic [31:0]           n_next     [NUM_AXES];
    logic [32:0]           acc        [NUM_AXES];         // накопители Брезенхема
    logic [32:0]           acc_next   [NUM_AXES];

    logic [NUM_AXES-1:0]   dir,        dir_next;          // посчитанное направление
    logic [NUM_AXES-1:0]   dir_out,    dir_out_next;      // то, что сейчас на выводах DIR
    logic [NUM_AXES-1:0]   pul_out,    pul_out_next;      // то, что сейчас на выводах PUL
    logic [31:0]           n_max,      n_max_next;        // импульсов у самой длинной оси
    logic [31:0]           steps_done, steps_done_next;   // сколько периодов уже прошло
    logic [TIMER_W-1:0]    timer,      timer_next;        // счётчик тактов для пауз
    logic [1:0]            calc_axis,  calc_axis_next;    // какую ось сейчас пересчитываем
    logic [PROD_W-1:0]     product,    product_next;      // |значение| × k

    logic                  homing,     homing_next;       // идёт хоминг
    logic [2:0]            home_mask,  home_mask_next;    // какие оси ещё надо отхомить
    logic [1:0]            home_axis,  home_axis_next;    // ось, которая хомится сейчас
    logic [31:0]           home_count, home_count_next;   // шагов сделано в поиске концевика

    // ---- Делитель: (произведение + 5000) / 10000 ----------------------------------
    logic                  div_start;
    logic                  div_done;
    logic [DIV_N_W-1:0]    div_q;

    udiv #(.N_W(DIV_N_W), .D_W(DIV_D_W)) u_div (
        .clk       (clk),
        .rst       (rst),
        .start     (div_start),
        .dividend  (DIV_N_W'(product) + DIV_N_W'(5000)),
        .divisor   (DIV_D_W'(10_000)),
        .busy      (),
        .done      (div_done),
        .quotient  (div_q),
        .remainder ()
    );

    // ---- Вспомогательная комбинационная логика -----------------------------------
    // Модуль числа и его знак для оси calc_axis
    value_t           calc_value;
    logic             calc_neg;
    logic [MAG_W-1:0] calc_mag;

    always_comb begin
        calc_value = value[calc_axis];
        calc_neg   = calc_value[VALUE_W-1];
        calc_mag   = MAG_W'(calc_neg ? -calc_value : calc_value);
    end

    // Цель и путь для осей X, Y, Z (в импульсах)
    logic signed [31:0] target;     // где надо оказаться
    logic signed [31:0] delta;      // сколько ехать (со знаком)

    always_comb begin
        target = calc_neg ? -$signed(32'(div_q)) : $signed(32'(div_q));
        delta  = target - pos[calc_axis];
    end

    // Самая длинная ось: сравниваем "деревом" — пары (X,Y) и (Z,E), потом победителей
    logic [31:0] n_max_xy, n_max_ze, n_max_calc;

    always_comb begin
        n_max_xy   = (n[AX_X] > n[AX_Y]) ? n[AX_X] : n[AX_Y];
        n_max_ze   = (n[AX_Z] > n[AX_E]) ? n[AX_Z] : n[AX_E];
        n_max_calc = (n_max_xy > n_max_ze) ? n_max_xy : n_max_ze;
    end

    // Брезенхем: какие оси шагают в этом периоде
    logic [NUM_AXES-1:0] fire;        // fire[a] = 1: ось a делает шаг
    logic [32:0]         acc_sum [NUM_AXES];

    always_comb begin
        for (int a = 0; a < NUM_AXES; a++) begin
            acc_sum[a] = acc[a] + 33'(n[a]);
            fire[a]    = (acc_sum[a] >= 33'(n_max));
        end
    end

    // Ось шагает к своему концевику, а он нажат — двигаться нельзя
    wire [2:0] toward_home  = ~(dir_out[2:0] ^ HOME_DIR);
    wire       limit_block  = |(fire[2:0] & toward_home & limit);

    // ---- Блок 1: что будет на следующем такте ----------------------------------------
    logic axis_calc_done;   // ось calc_axis пересчитана, переходим к следующей

    always_comb begin
        // По умолчанию всё хранит значения
        state_next      = state;
        has_next        = has;
        value_next      = value;
        result_next     = result;
        k_next          = k;
        pos_next        = pos;
        n_next          = n;
        acc_next        = acc;
        dir_next        = dir;
        dir_out_next    = dir_out;
        pul_out_next    = pul_out;
        n_max_next      = n_max;
        steps_done_next = steps_done;
        timer_next      = timer;
        calc_axis_next  = calc_axis;
        product_next    = product;
        homing_next     = homing;
        home_mask_next  = home_mask;
        home_axis_next  = home_axis;
        home_count_next = home_count;

        div_start       = 1'b0;
        axis_calc_done  = 1'b0;

        case (state)
            // ----------------------------------------------------------------------
            S_IDLE: begin
                if (cmd_valid) begin
                    has_next    = cmd.has;
                    value_next  = cmd.value;
                    result_next = ERR_NONE;
                    for (int a = 0; a < NUM_AXES; a++) begin
                        n_next[a]   = '0;           // оси без поля не двигаются
                        dir_next[a] = dir_out[a];   // и не меняют DIR зря
                    end

                    if (cmd.err != ERR_NONE) begin
                        result_next = cmd.err;      // строка с ошибкой: просто отвечаем
                        state_next  = S_DONE;
                    end else begin
                        case (cmd.opcode)
                            OP_M92: begin
                                for (int a = 0; a < NUM_AXES; a++)
                                    if (cmd.has[a]) k_next[a] = K_W'(cmd.value[a]);
                                state_next = S_DONE;
                            end
                            OP_G0, OP_G1: begin
                                calc_axis_next = 2'd0;
                                state_next     = S_CALC_MUL;
                            end
                            OP_G28: begin
                                // "G28" — все оси; "G28 X" — только X
                                home_mask_next = (cmd.has[2:0] != 3'b000) ? cmd.has[2:0] : 3'b111;
                                homing_next    = 1'b1;
                                state_next     = S_HOME_NEXT;
                            end
                            default: begin
                                result_next = ERR_UNKNOWN;
                                state_next  = S_DONE;
                            end
                        endcase
                    end
                end
            end

            // ---------------------- перевод мм -> импульсы ----------------------
            S_CALC_MUL: begin
                if (!has[calc_axis])
                    axis_calc_done = 1'b1;              // поля нет — ось стоит
                else begin
                    product_next = PROD_W'(calc_mag) * PROD_W'(k[calc_axis]);
                    state_next   = S_CALC_DIV;
                end
            end

            S_CALC_DIV: begin
                div_start  = 1'b1;
                state_next = S_CALC_WAIT;
            end

            S_CALC_WAIT: begin
                if (div_done) begin
                    if (div_q > DIV_N_W'(STEP_LIMIT)) begin
                        result_next = ERR_NUMBER;       // столько импульсов не бывает
                        state_next  = S_DONE;
                    end else if (calc_axis == 2'(AX_E)) begin
                        n_next[AX_E]   = 32'(div_q);    // E: выдавить ровно столько
                        dir_next[AX_E] = !calc_neg;     // '-' — втянуть пасту
                        axis_calc_done = 1'b1;
                    end else begin
                        n_next[calc_axis]   = delta[31] ? 32'(-delta) : 32'(delta);
                        if (delta != 0)
                            dir_next[calc_axis] = !delta[31];
                        axis_calc_done = 1'b1;
                    end
                end
            end

            // ---------------------- подготовка к движению ----------------------
            S_FIND_MAX: begin
                n_max_next = n_max_calc;
                state_next = S_PREPARE;
            end

            S_PREPARE: begin
                if (n_max == '0)
                    state_next = S_DONE;                // никуда ехать не надо
                else begin
                    steps_done_next = '0;
                    for (int a = 0; a < NUM_AXES; a++)
                        acc_next[a] = 33'(n_max >> 1);  // старт с середины: шаги симметричнее
                    dir_out_next    = dir;              // выставляем DIR заранее
                    timer_next      = TIMER_W'(DIR_SETUP_CLKS - 1);
                    state_next      = S_DIR_SETUP;
                end
            end

            S_DIR_SETUP: begin
                if (timer == '0)
                    state_next = homing ? S_HOME_STEP : S_STEP;
                else
                    timer_next = timer - 1'b1;
            end

            // ---------------------- движение ----------------------
            S_STEP: begin
                if (limit_block) begin
                    result_next = ERR_LIMIT;            // упёрлись в концевик
                    state_next  = S_DONE;
                end else begin
                    for (int a = 0; a < NUM_AXES; a++)
                        acc_next[a] = fire[a] ? (acc_sum[a] - 33'(n_max)) : acc_sum[a];
                    for (int a = 0; a < 3; a++)
                        if (fire[a])
                            pos_next[a] = dir_out[a] ? (pos[a] + 1) : (pos[a] - 1);
                    pul_out_next = fire;                // фронт PUL у шагающих осей
                    timer_next   = TIMER_W'(PULSE_CLKS - 1);
                    state_next   = S_PULSE_HIGH;
                end
            end

            S_PULSE_HIGH: begin
                if (timer == '0) begin
                    pul_out_next = '0;
                    timer_next   = homing ? TIMER_W'(HOME_LOW_CLKS - 1)
                                          : TIMER_W'(STEP_LOW_CLKS - 1);
                    state_next   = S_PULSE_LOW;
                end else
                    timer_next = timer - 1'b1;
            end

            S_PULSE_LOW: begin
                if (timer != '0)
                    timer_next = timer - 1'b1;
                else if (homing) begin
                    home_count_next = home_count + 1'b1;
                    state_next      = S_HOME_STEP;
                end else begin
                    steps_done_next = steps_done + 1'b1;
                    state_next      = (steps_done + 1'b1 == n_max) ? S_DONE : S_STEP;
                end
            end

            // ---------------------- хоминг ----------------------
            S_HOME_NEXT: begin
                // Порядок: сначала Z (поднять шприц), потом X, потом Y
                if (home_mask[AX_Z] || home_mask[AX_X] || home_mask[AX_Y]) begin
                    if      (home_mask[AX_Z]) home_axis_next = 2'(AX_Z);
                    else if (home_mask[AX_X]) home_axis_next = 2'(AX_X);
                    else                      home_axis_next = 2'(AX_Y);
                    home_mask_next[home_axis_next] = 1'b0;
                    home_count_next                = '0;
                    dir_out_next[home_axis_next]   = HOME_DIR[home_axis_next];
                    timer_next                     = TIMER_W'(DIR_SETUP_CLKS - 1);
                    state_next                     = S_DIR_SETUP;
                end else
                    state_next = S_DONE;                // все нужные оси на нуле
            end

            S_HOME_STEP: begin
                if (limit[home_axis]) begin
                    pos_next[home_axis] = '0;           // нашли ноль оси
                    state_next          = S_HOME_NEXT;
                end else if (home_count == 32'(HOME_MAX_STEPS)) begin
                    result_next = ERR_HOMING;           // концевик так и не нажался
                    state_next  = S_DONE;
                end else begin
                    pul_out_next[home_axis] = 1'b1;
                    timer_next              = TIMER_W'(PULSE_CLKS - 1);
                    state_next              = S_PULSE_HIGH;
                end
            end

            // ---------------------- ответ ----------------------
            S_DONE: begin
                if (res_ready) begin
                    homing_next = 1'b0;
                    state_next  = S_IDLE;
                end
            end

            default: state_next = S_IDLE;
        endcase

        // Ось пересчитана: следующая ось или подготовка к движению
        if (axis_calc_done) begin
            if (calc_axis == 2'(NUM_AXES - 1))
                state_next = S_FIND_MAX;
            else begin
                calc_axis_next = calc_axis + 2'd1;
                state_next     = S_CALC_MUL;
            end
        end
    end

    // ---- Блок 2: на фронте clk запоминаем посчитанное ---------------------------------
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            state      <= S_IDLE;
            has        <= '0;
            value      <= '0;
            result     <= ERR_NONE;
            k[AX_X]    <= K_W'(K_X_DEFAULT);
            k[AX_Y]    <= K_W'(K_Y_DEFAULT);
            k[AX_Z]    <= K_W'(K_Z_DEFAULT);
            k[AX_E]    <= K_W'(K_E_DEFAULT);
            for (int a = 0; a < 3; a++)        pos[a] <= '0;
            for (int a = 0; a < NUM_AXES; a++) begin
                n[a]   <= '0;
                acc[a] <= '0;
            end
            dir        <= '0;
            dir_out    <= '0;
            pul_out    <= '0;
            n_max      <= '0;
            steps_done <= '0;
            timer      <= '0;
            calc_axis  <= '0;
            product    <= '0;
            homing     <= 1'b0;
            home_mask  <= '0;
            home_axis  <= '0;
            home_count <= '0;
        end else begin
            state      <= state_next;
            has        <= has_next;
            value      <= value_next;
            result     <= result_next;
            k          <= k_next;
            pos        <= pos_next;
            n          <= n_next;
            acc        <= acc_next;
            dir        <= dir_next;
            dir_out    <= dir_out_next;
            pul_out    <= pul_out_next;
            n_max      <= n_max_next;
            steps_done <= steps_done_next;
            timer      <= timer_next;
            calc_axis  <= calc_axis_next;
            product    <= product_next;
            homing     <= homing_next;
            home_mask  <= home_mask_next;
            home_axis  <= home_axis_next;
            home_count <= home_count_next;
        end
    end

    // ---- Выходы -------------------------------------------------------------------------
    assign cmd_ready = (state == S_IDLE);
    assign res_valid = (state == S_DONE);
    assign res_err   = result;
    assign busy      = (state != S_IDLE);
    assign step_pul  = pul_out;      // выходы прямо с триггеров: без "иголок" на линиях
    assign step_dir  = dir_out;
    assign pos_x     = pos[AX_X];
    assign pos_y     = pos[AX_Y];
    assign pos_z     = pos[AX_Z];

endmodule
