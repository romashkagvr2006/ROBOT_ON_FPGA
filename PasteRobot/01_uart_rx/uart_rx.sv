// =============================================================================
// uart_rx.sv — приёмник UART 8N1 (8 бит данных, без чётности, 1 стоп-бит)
// -----------------------------------------------------------------------------
// ЧТО ДЕЛАЕТ
//   Слушает линию rx (в покое на ней 1). Падение линии в 0 — начало старт-бита.
//   Приёмник отсчитывает такты до СЕРЕДИНЫ каждого бита и читает линию там,
//   где сигнал уже устоялся: старт-бит, 8 бит данных (младший первым), стоп-бит.
//
//   rx  ‾‾‾‾‾\_______/‾‾‾‾‾‾‾\_______ ... _______/‾‾‾‾‾‾‾‾‾‾‾‾
//            | старт |  бит0 |  бит1 |     | бит7 | стоп  |
//                ^       ^       ^             ^      ^
//                середины битов: здесь читаем линию
//
//   1 бит = CLKS_PER_BIT тактов (434 такта при 50 МГц и 115200 бод).
//
// КУДА ОТДАЁТ ДАННЫЕ
//   rx_data   — принятый байт;
//   rx_valid  — импульс 1 такт: "в rx_data новый байт". Байт забирает очередь
//               byte_fifo (см. 09_robot_top/robot_top.sv);
//   frame_err — импульс 1 такт: стоп-бит оказался 0 (не та скорость, помеха,
//               обрыв). Такой байт НЕ выдаётся.
//
// ЗАЩИТА ОТ ПОМЕХ
//   1) rx проходит через два триггера (синхронизатор): линия приходит с ПК,
//      она не связана с нашим clk, и без синхронизатора возможна метастабильность.
//   2) В середине старт-бита линия проверяется ещё раз: если там уже 1, это была
//      короткая помеха, а не старт-бит, — возвращаемся в ожидание.
//   3) После ошибки кадра ждём, пока линия вернётся в 1 (WAIT_IDLE), иначе
//      "залипшая" в 0 линия давала бы поток ложных байт.
//
// КОНЕЧНЫЙ АВТОМАТ (два блока: always_comb считает "что будет", always_ff запоминает)
//
//            rx=0             середина старта, rx=0      8 бит прочитаны
//   IDLE ----------> START ----------------------> DATA ----------------> STOP
//    ^ ^               | середина старта, rx=1             стоп=1: rx_valid |
//    | +---------------+  (помеха)                                          |
//    +--------------------------------------------------------------------+ |
//    |                                                    стоп=0: frame_err |
//    +---- WAIT_IDLE <------------------------------------------------------+
//            rx=1
// =============================================================================
module uart_rx #(
    parameter int CLK_HZ = 50_000_000,   // тактовая частота платы, Гц
    parameter int BAUD   = 115_200       // скорость линии, бит/с
) (
    input  logic       clk,        // такт 50 МГц
    input  logic       rst,        // сброс, активная 1
    input  logic       rx,         // линия RX от ПК (в покое 1)
    output logic [7:0] rx_data,    // принятый байт
    output logic       rx_valid,   // 1 такт: rx_data только что обновился
    output logic       frame_err   // 1 такт: ошибка кадра (стоп-бит = 0)
);
    localparam int CLKS_PER_BIT  = CLK_HZ / BAUD;       // 434 такта на бит
    localparam int CLKS_HALF_BIT = CLKS_PER_BIT / 2;    // 217 тактов: до середины старт-бита
    localparam int CNT_W         = $clog2(CLKS_PER_BIT);

    // ---- Синхронизатор входа: 2 триггера подряд ----------------------------
    logic rx_meta, rx_sync;   // rx_meta может "дрожать", дальше используем только rx_sync

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            rx_meta <= 1'b1;      // линия в покое = 1
            rx_sync <= 1'b1;
        end else begin
            rx_meta <= rx;
            rx_sync <= rx_meta;
        end
    end

    // ---- Состояния автомата -------------------------------------------------
    typedef enum logic [2:0] {
        IDLE,        // линия в покое, ждём спада (начала старт-бита)
        START,       // идём к середине старт-бита и проверяем, что там 0
        DATA,        // читаем 8 бит данных в серединах битов
        STOP,        // читаем стоп-бит: должна быть 1
        WAIT_IDLE    // после ошибки кадра ждём, пока линия вернётся в 1
    } state_t;

    state_t state, state_next;

    // Регистры и их "следующие" значения
    logic [CNT_W-1:0] clk_cnt,   clk_cnt_next;    // счётчик тактов внутри бита
    logic [2:0]       bit_idx,   bit_idx_next;    // номер принимаемого бита данных 0..7
    logic [7:0]       shift_reg, shift_reg_next;  // сюда задвигаем принятые биты
    logic [7:0]       rx_data_next;
    logic             rx_valid_next;
    logic             frame_err_next;

    wire half_bit_done = (clk_cnt == CNT_W'(CLKS_HALF_BIT - 1));  // дошли до середины старт-бита
    wire full_bit_done = (clk_cnt == CNT_W'(CLKS_PER_BIT  - 1));  // прошёл целый бит

    // ---- Блок 1: что будет на следующем такте ------------------------------
    always_comb begin
        // По умолчанию всё хранит свои значения, а импульсы равны 0
        state_next     = state;
        clk_cnt_next   = clk_cnt + 1'b1;
        bit_idx_next   = bit_idx;
        shift_reg_next = shift_reg;
        rx_data_next   = rx_data;
        rx_valid_next  = 1'b0;
        frame_err_next = 1'b0;

        case (state)
            IDLE: begin
                clk_cnt_next = '0;                   // в покое счётчик стоит на нуле
                if (rx_sync == 1'b0)                 // линия упала: возможно, старт-бит
                    state_next = START;
            end

            START: begin
                if (half_bit_done) begin
                    clk_cnt_next = '0;
                    if (rx_sync == 1'b0) begin       // в середине старт-бита 0: всё честно
                        bit_idx_next = 3'd0;
                        state_next   = DATA;
                    end else begin                   // там уже 1: это была помеха
                        state_next   = IDLE;
                    end
                end
            end

            DATA: begin
                if (full_bit_done) begin             // середина очередного бита данных
                    clk_cnt_next   = '0;
                    shift_reg_next = {rx_sync, shift_reg[7:1]};  // младший бит пришёл первым,
                                                                 // поэтому задвигаем слева направо
                    if (bit_idx == 3'd7)
                        state_next = STOP;
                    else
                        bit_idx_next = bit_idx + 3'd1;
                end
            end

            STOP: begin
                if (full_bit_done) begin             // середина стоп-бита
                    clk_cnt_next = '0;
                    if (rx_sync == 1'b1) begin       // стоп-бит правильный: байт готов
                        rx_data_next  = shift_reg;
                        rx_valid_next = 1'b1;
                        state_next    = IDLE;
                    end else begin                   // стоп-бит = 0: ошибка кадра
                        frame_err_next = 1'b1;
                        state_next     = WAIT_IDLE;
                    end
                end
            end

            WAIT_IDLE: begin
                clk_cnt_next = '0;
                if (rx_sync == 1'b1)
                    state_next = IDLE;
            end

            default: state_next = IDLE;
        endcase
    end

    // ---- Блок 2: на фронте clk запоминаем посчитанное -----------------------
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            state     <= IDLE;
            clk_cnt   <= '0;
            bit_idx   <= '0;
            shift_reg <= '0;
            rx_data   <= '0;
            rx_valid  <= 1'b0;
            frame_err <= 1'b0;
        end else begin
            state     <= state_next;
            clk_cnt   <= clk_cnt_next;
            bit_idx   <= bit_idx_next;
            shift_reg <= shift_reg_next;
            rx_data   <= rx_data_next;
            rx_valid  <= rx_valid_next;
            frame_err <= frame_err_next;
        end
    end

endmodule
