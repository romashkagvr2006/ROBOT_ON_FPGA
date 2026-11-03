// =============================================================================
// udiv.sv — последовательный делитель беззнаковых чисел ("деление столбиком")
// -----------------------------------------------------------------------------
// ЗАЧЕМ НУЖЕН
//   Контроллер движения переводит миллиметры в импульсы:
//       импульсов = (координата в сотых мм × k в сотых имп/мм + 5000) / 10000
//   Деление "/" одной строкой Quartus превратил бы в огромную цепочку логики,
//   которая не успевает за 50 МГц. Здесь деление идёт по одному биту частного
//   за такт, как деление столбиком в двоичной системе: N_W тактов на результат
//   (50 тактов = 1 мкс — для команды, которая едет секунды, это ничего).
//
// ОТКУДА БЕРЁТ И КУДА ОТДАЁТ
//   Вход:  start (импульс 1 такт), dividend (делимое), divisor (делитель)
//          <- motion_ctrl.
//   Выход: done (импульс 1 такт), quotient (частное), remainder (остаток)
//          -> motion_ctrl. busy = 1, пока идёт деление.
//
// КАК РАБОТАЕТ (деление столбиком)
//   На каждом шаге к остатку справа "сносится" очередной бит делимого
//   (начиная со старшего). Если остаток стал >= делителя, вычитаем делитель
//   и пишем в частное 1, иначе пишем 0. После N_W шагов в частном — результат.
//
//   Пример 13 / 3 (1101 / 11):  остаток 1 -> 0;  11 -> 1, ост 0;
//                               0 -> 0;  01 -> 0  => частное 0100 = 4, остаток 1
//
// КОНЕЧНЫЙ АВТОМАТ:  IDLE --start--> RUN (N_W тактов) --> DONE --> IDLE
// =============================================================================
module udiv #(
    parameter int N_W = 50,   // разрядность делимого и частного
    parameter int D_W = 14    // разрядность делителя и остатка (10000 < 2^14)
) (
    input  logic           clk,
    input  logic           rst,         // сброс, активная 1
    input  logic           start,       // 1 такт: начать деление
    input  logic [N_W-1:0] dividend,    // делимое (запоминается в момент start)
    input  logic [D_W-1:0] divisor,     // делитель (запоминается в момент start)
    output logic           busy,        // идёт деление
    output logic           done,        // 1 такт: результат готов
    output logic [N_W-1:0] quotient,    // частное
    output logic [D_W-1:0] remainder    // остаток
);
    localparam int CNT_W = $clog2(N_W + 1);

    typedef enum logic [1:0] {
        IDLE,   // ждём start
        RUN,    // N_W шагов деления
        DONE    // результат готов, done = 1 на один такт
    } state_t;

    state_t state, state_next;

    logic [N_W-1:0]   quot,    quot_next;     // сюда задвигается делимое, а на его место — частное
    logic [D_W:0]     rem,     rem_next;      // остаток (на 1 бит шире делителя)
    logic [D_W-1:0]   div_r,   div_r_next;    // запомненный делитель
    logic [CNT_W-1:0] step,    step_next;     // номер шага 0..N_W-1

    // Один шаг деления столбиком
    logic [D_W:0] rem_shifted;                // остаток со "снесённым" битом
    logic         fits;                       // делитель помещается в остаток

    always_comb begin
        rem_shifted = {rem[D_W-1:0], quot[N_W-1]};
        fits        = (rem_shifted >= {1'b0, div_r});
    end

    // ---- Блок 1: что будет на следующем такте ----
    always_comb begin
        state_next = state;
        quot_next  = quot;
        rem_next   = rem;
        div_r_next = div_r;
        step_next  = step;

        case (state)
            IDLE: begin
                if (start) begin
                    quot_next  = dividend;
                    rem_next   = '0;
                    div_r_next = divisor;
                    step_next  = '0;
                    state_next = RUN;
                end
            end

            RUN: begin
                // остаток: вычитаем делитель, если он помещается
                rem_next  = fits ? (rem_shifted - {1'b0, div_r}) : rem_shifted;
                // частное: сдвигаем влево, в младший бит пишем результат шага
                quot_next = {quot[N_W-2:0], fits};
                step_next = step + 1'b1;
                if (step == CNT_W'(N_W - 1))
                    state_next = DONE;
            end

            DONE: state_next = IDLE;

            default: state_next = IDLE;
        endcase
    end

    // ---- Блок 2: на фронте clk запоминаем ----
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= IDLE;
            quot  <= '0;
            rem   <= '0;
            div_r <= '0;
            step  <= '0;
        end else begin
            state <= state_next;
            quot  <= quot_next;
            rem   <= rem_next;
            div_r <= div_r_next;
            step  <= step_next;
        end
    end

    // ---- Выходы ----
    assign busy      = (state != IDLE);
    assign done      = (state == DONE);
    assign quotient  = quot;
    assign remainder = rem[D_W-1:0];

endmodule
