// =============================================================================
// robot_top.sv — верхний модуль: вся цепочка от ПК до драйверов моторов
// -----------------------------------------------------------------------------
// Этот модуль ничего не считает сам: он только соединяет остальные модули
// проводами и подстраивает уровни сигналов под плату.
//
//                    байты           байты           команда cmd_t
//   uart_rxd --> [uart_rx] --> [byte_fifo] --> [cmd_parser] ------------+
//   (от ПК)    rx_valid/rx_data   valid/ready      valid/ready          |
//                                                                       v
//   limit_*_raw --> [debounce] x3 --> limit[2:0] -----------------> [motion_ctrl] --> motor_pul[3:0]
//   (концевики)                                                         |   (+udiv)   motor_dir[3:0]
//                                                                       | результат
//                                                                       v valid/ready
//   uart_txd <-- [uart_tx] <-- tx_start/tx_data -- [reply_gen] <--------+
//   (к ПК)                    "ok\r\n" / "err N\r\n"
//
// ПУТЬ ОДНОЙ КОМАНДЫ "G1 X20 Y8 E2" (Enter в Termite)
//   1. uart_rx принимает 14 байт ("G1 X20 Y8 E2" и CR LF), каждый — импульсом rx_valid.
//   2. byte_fifo складывает их в очередь.
//   3. cmd_parser забирает байты по одному и в конце строки выдаёт cmd_t:
//      G1, X=2000, Y=800, E=200 (сотые).
//   4. motion_ctrl переводит мм в импульсы, выставляет DIR и выдаёт импульсы
//      PUL на X, Y и E так, чтобы оси закончили одновременно.
//   5. Когда последний импульс отдан, motion_ctrl отдаёт результат ERR_NONE.
//   6. reply_gen шлёт через uart_tx строку "ok\r\n" — она появляется в Termite.
//   Пока шаги 4–6 не закончились, парсер не берёт новую команду: следующая
//   строка ждёт в byte_fifo.
//
// ПАРАМЕТРЫ ДЛЯ ПЛАТЫ И РОБОТА (проверить перед прошивкой)
//   STEP_HZ, HOME_STEP_HZ — частота импульсов (скорость), см. "путь-команды", п. 6;
//   HOME_DIR    — в какую сторону ехать к концевику (уровень DIR), бит 0 = X;
//   DIR_INVERT  — перевернуть DIR, если мотор крутится не в ту сторону;
//   LIMIT_ACTIVE_LOW = 1 — концевик замыкает вывод на землю (нажат = 0);
//   ENA_LEVEL   — уровень на ENA, при котором драйвер включён (зависит от схемы
//                 подключения TB6600, проверить по паспорту драйвера);
//   LED_ON_LEVEL — у многих плат светодиоды горят от 0.
//   Выходы ПЛИС — 3,3 В, входы TB6600 рассчитаны на 5 В: нужен буфер (вопрос 8 в ТОП).
// =============================================================================
module robot_top
    import robot_pkg::*;
#(
    parameter int         CLK_HZ           = 50_000_000,  // кварц платы
    parameter int         BAUD             = 115_200,     // скорость UART (как в Termite)
    parameter int         FIFO_DEPTH       = 256,         // очередь байт от ПК
    parameter int         STEP_HZ          = 2_000,       // частота импульсов при движении
    parameter int         HOME_STEP_HZ     = 1_000,       // частота импульсов при хоминге
    parameter int         PULSE_US         = 10,          // длительность импульса PUL, мкс
    parameter int         DIR_SETUP_US     = 10,          // пауза после смены DIR, мкс
    parameter int         DEBOUNCE_US      = 2_000,       // антидребезг концевиков, мкс
    parameter int         HOME_MAX_STEPS   = 400_000,     // предел шагов при поиске концевика
    parameter logic [2:0] HOME_DIR         = 3'b000,      // DIR "к концевику" для Z,Y,X
    parameter logic [3:0] DIR_INVERT       = 4'b0000,     // перевернуть DIR для E,Z,Y,X
    parameter bit         LIMIT_ACTIVE_LOW = 1'b1,        // концевик нажат = 0 на выводе
    parameter logic       ENA_LEVEL        = 1'b0,        // уровень ENA "драйвер включён"
    parameter logic       LED_ON_LEVEL     = 1'b0         // уровень, при котором горит светодиод
) (
    input  logic                clk_50,        // кварц 50 МГц
    input  logic                key_rst_n,     // кнопка сброса на плате (нажата = 0)
    // ---- UART к ПК ----
    input  logic                uart_rxd,      // от ПК
    output logic                uart_txd,      // к ПК
    // ---- концевики (сырые сигналы с выводов) ----
    input  logic                limit_x_raw,
    input  logic                limit_y_raw,
    input  logic                limit_z_raw,
    // ---- к драйверам TB6600: [0]=X [1]=Y [2]=Z [3]=E ----
    output logic [NUM_AXES-1:0] motor_pul,
    output logic [NUM_AXES-1:0] motor_dir,
    output logic [NUM_AXES-1:0] motor_ena,
    // ---- светодиоды ----
    output logic                led_busy,      // моторы едут / команда выполняется
    output logic                led_error,     // последняя команда закончилась ошибкой
    output logic                led_overflow   // очередь байт переполнялась (до сброса)
);
    // ---- Пересчёт времени в такты ---------------------------------------------
    localparam int CLKS_PER_US        = CLK_HZ / 1_000_000;
    localparam int CLKS_PER_STEP      = CLK_HZ / STEP_HZ;
    localparam int CLKS_PER_HOME_STEP = CLK_HZ / HOME_STEP_HZ;
    localparam int PULSE_CLKS         = PULSE_US     * CLKS_PER_US;
    localparam int DIR_SETUP_CLKS     = DIR_SETUP_US * CLKS_PER_US;
    localparam int DEBOUNCE_CLKS      = DEBOUNCE_US  * CLKS_PER_US;

    // ---- Сброс: кнопка -> синхронный сброс rst (активная 1) -------------------
    // Нажатие кнопки сразу включает сброс, а отпускание доходит через 2 триггера,
    // чтобы все модули вышли из сброса на одном и том же фронте clk.
    logic [1:0] rst_sync;
    logic       rst;

    always_ff @(posedge clk_50 or negedge key_rst_n) begin
        if (!key_rst_n) rst_sync <= 2'b11;
        else            rst_sync <= {rst_sync[0], 1'b0};
    end
    assign rst = rst_sync[1];

    // ---- Провода между модулями ------------------------------------------------
    // uart_rx -> byte_fifo
    logic [7:0] rx_data;
    logic       rx_valid;
    logic       rx_frame_err;
    // byte_fifo -> cmd_parser
    logic       fifo_valid;
    logic [7:0] fifo_data;
    logic       fifo_ready;
    logic       fifo_full;
    logic       fifo_overflow;
    // cmd_parser -> motion_ctrl
    logic       cmd_valid;
    cmd_t       cmd;
    logic       cmd_ready;
    // motion_ctrl -> reply_gen
    logic       res_valid;
    err_t       res_err;
    logic       res_ready;
    // reply_gen -> uart_tx
    logic       tx_start;
    logic [7:0] tx_data;
    logic       tx_busy;
    // концевики после фильтра, 1 = нажат
    logic [2:0] limit_clean;
    logic [2:0] limit_pressed;
    // мотор
    logic [NUM_AXES-1:0] step_pul;
    logic [NUM_AXES-1:0] step_dir;
    logic                motion_busy;

    // ---- 1. Приём байт с ПК -------------------------------------------------------
    uart_rx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) u_uart_rx (
        .clk       (clk_50),
        .rst       (rst),
        .rx        (uart_rxd),
        .rx_data   (rx_data),
        .rx_valid  (rx_valid),
        .frame_err (rx_frame_err)
    );

    // ---- 2. Очередь байт --------------------------------------------------------
    byte_fifo #(.DEPTH(FIFO_DEPTH)) u_fifo (
        .clk      (clk_50),
        .rst      (rst),
        .wr_valid (rx_valid),
        .wr_data  (rx_data),
        .full     (fifo_full),
        .overflow (fifo_overflow),
        .rd_valid (fifo_valid),
        .rd_data  (fifo_data),
        .rd_ready (fifo_ready)
    );

    // ---- 3. Разбор строки -------------------------------------------------------
    cmd_parser u_parser (
        .clk       (clk_50),
        .rst       (rst),
        .in_valid  (fifo_valid),
        .in_data   (fifo_data),
        .in_ready  (fifo_ready),
        .cmd_valid (cmd_valid),
        .cmd       (cmd),
        .cmd_ready (cmd_ready)
    );

    // ---- 4. Концевики: синхронизатор и антидребезг ----------------------------
    debounce #(.STABLE_CLKS(DEBOUNCE_CLKS), .RESET_LEVEL(LIMIT_ACTIVE_LOW)) u_db_x (
        .clk (clk_50), .rst (rst), .raw (limit_x_raw), .clean (limit_clean[AX_X])
    );
    debounce #(.STABLE_CLKS(DEBOUNCE_CLKS), .RESET_LEVEL(LIMIT_ACTIVE_LOW)) u_db_y (
        .clk (clk_50), .rst (rst), .raw (limit_y_raw), .clean (limit_clean[AX_Y])
    );
    debounce #(.STABLE_CLKS(DEBOUNCE_CLKS), .RESET_LEVEL(LIMIT_ACTIVE_LOW)) u_db_z (
        .clk (clk_50), .rst (rst), .raw (limit_z_raw), .clean (limit_clean[AX_Z])
    );

    // Приводим к виду "1 = нажат", как ждёт motion_ctrl
    assign limit_pressed = LIMIT_ACTIVE_LOW ? ~limit_clean : limit_clean;

    // ---- 5. Движение ------------------------------------------------------------
    motion_ctrl #(
        .CLKS_PER_STEP      (CLKS_PER_STEP),
        .CLKS_PER_HOME_STEP (CLKS_PER_HOME_STEP),
        .PULSE_CLKS         (PULSE_CLKS),
        .DIR_SETUP_CLKS     (DIR_SETUP_CLKS),
        .HOME_MAX_STEPS     (HOME_MAX_STEPS),
        .HOME_DIR           (HOME_DIR)
    ) u_motion (
        .clk       (clk_50),
        .rst       (rst),
        .cmd_valid (cmd_valid),
        .cmd       (cmd),
        .cmd_ready (cmd_ready),
        .res_valid (res_valid),
        .res_err   (res_err),
        .res_ready (res_ready),
        .limit     (limit_pressed),
        .step_pul  (step_pul),
        .step_dir  (step_dir),
        .busy      (motion_busy),
        .pos_x     (),               // координаты пока не выводим (пригодятся для M114)
        .pos_y     (),
        .pos_z     ()
    );

    // ---- 6. Ответ на ПК ---------------------------------------------------------
    reply_gen u_reply (
        .clk       (clk_50),
        .rst       (rst),
        .res_valid (res_valid),
        .res_err   (res_err),
        .res_ready (res_ready),
        .tx_start  (tx_start),
        .tx_data   (tx_data),
        .tx_busy   (tx_busy)
    );

    uart_tx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) u_uart_tx (
        .clk      (clk_50),
        .rst      (rst),
        .tx_start (tx_start),
        .tx_data  (tx_data),
        .tx       (uart_txd),
        .busy     (tx_busy)
    );

    // ---- Выводы к драйверам -----------------------------------------------------
    assign motor_pul = step_pul;
    assign motor_dir = step_dir ^ DIR_INVERT;
    assign motor_ena = {NUM_AXES{ENA_LEVEL}};   // драйверы включены всегда (место для E-Stop)

    // ---- Светодиоды ---------------------------------------------------------------
    // Ошибка горит, пока не придёт команда, закончившаяся "ok"
    logic last_cmd_failed;

    always_ff @(posedge clk_50 or posedge rst) begin
        if (rst)
            last_cmd_failed <= 1'b0;
        else if (res_valid && res_ready)
            last_cmd_failed <= (res_err != ERR_NONE);
    end

    assign led_busy     = motion_busy     ? LED_ON_LEVEL : ~LED_ON_LEVEL;
    assign led_error    = last_cmd_failed ? LED_ON_LEVEL : ~LED_ON_LEVEL;
    assign led_overflow = fifo_overflow   ? LED_ON_LEVEL : ~LED_ON_LEVEL;

endmodule
