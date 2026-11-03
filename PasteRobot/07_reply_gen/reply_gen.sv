// =============================================================================
// reply_gen.sv — ответ на ПК: "ok" или "err N" + CR LF
// -----------------------------------------------------------------------------
// ЧТО ДЕЛАЕТ
//   Когда контроллер движения закончил команду, этот модуль отправляет на ПК
//   строку ответа, байт за байтом, через передатчик uart_tx:
//       ERR_NONE      -> "ok\r\n"       (4 байта)
//       ERR_UNKNOWN.. -> "err 1\r\n" ... "err 6\r\n"   (7 байт)
//   В Termite ответ появляется в том же окне, где набрана команда.
//
// ОТКУДА БЕРЁТ И КУДА ОТДАЁТ
//   res_valid / res_err / res_ready <- motion_ctrl (рукопожатие valid/ready).
//                                     Пока ответ отправляется, res_ready = 0,
//                                     и следующий результат ждёт.
//   tx_start / tx_data -> uart_tx;  tx_busy <- uart_tx (1, пока байт уходит).
//
// КОНЕЧНЫЙ АВТОМАТ
//
//   IDLE --res_valid--> SEND --tx_start--> WAIT_BUSY --busy=1--> WAIT_DONE
//    ^                    ^                                         |
//    |                    +-------------- ещё есть байты ------------+
//    +--------------------------------- последний байт ушёл --------+
//
//   SEND:      ждём, пока передатчик свободен, и даём tx_start с очередным байтом;
//   WAIT_BUSY: ждём, пока передатчик подтвердит, что взял байт (busy = 1);
//   WAIT_DONE: ждём, пока байт уйдёт целиком (busy = 0).
// =============================================================================
module reply_gen
    import robot_pkg::*;
(
    input  logic       clk,
    input  logic       rst,         // сброс, активная 1
    // ---- результат команды (от motion_ctrl) ----
    input  logic       res_valid,
    input  err_t       res_err,
    output logic       res_ready,
    // ---- к передатчику uart_tx ----
    output logic       tx_start,    // 1 такт: отправить tx_data
    output logic [7:0] tx_data,
    input  logic       tx_busy
);
    localparam logic [7:0] CH_CR = 8'h0D;
    localparam logic [7:0] CH_LF = 8'h0A;

    // Длина ответа: "ok\r\n" — 4 байта, "err N\r\n" — 7 байт
    function automatic logic [2:0] reply_len(input err_t e);
        return (e == ERR_NONE) ? 3'd4 : 3'd7;
    endfunction

    // Байт номер idx в строке ответа
    function automatic logic [7:0] reply_char(input err_t e, input logic [2:0] idx);
        if (e == ERR_NONE) begin
            case (idx)
                3'd0:    return "o";
                3'd1:    return "k";
                3'd2:    return CH_CR;
                default: return CH_LF;
            endcase
        end else begin
            case (idx)
                3'd0:    return "e";
                3'd1:    return "r";
                3'd2:    return "r";
                3'd3:    return " ";
                3'd4:    return 8'("0") + 8'(e);     // цифра кода ошибки: '0' + N
                3'd5:    return CH_CR;
                default: return CH_LF;
            endcase
        end
    endfunction

    typedef enum logic [1:0] {
        IDLE,        // ждём результат команды
        SEND,        // отдаём очередной байт передатчику
        WAIT_BUSY,   // передатчик взял байт?
        WAIT_DONE    // байт ушёл?
    } state_t;

    state_t     state, state_next;
    err_t       err_r, err_r_next;        // какой ответ шлём
    logic [2:0] idx,   idx_next;          // номер байта в ответе

    // ---- Блок 1: что будет на следующем такте ----
    always_comb begin
        state_next = state;
        err_r_next = err_r;
        idx_next   = idx;

        case (state)
            IDLE: begin
                if (res_valid) begin
                    err_r_next = res_err;
                    idx_next   = '0;
                    state_next = SEND;
                end
            end

            SEND: begin
                if (!tx_busy)
                    state_next = WAIT_BUSY;          // tx_start выдаётся в этом такте (см. ниже)
            end

            WAIT_BUSY: begin
                if (tx_busy)
                    state_next = WAIT_DONE;
            end

            WAIT_DONE: begin
                if (!tx_busy) begin
                    if (idx == reply_len(err_r) - 3'd1)
                        state_next = IDLE;           // весь ответ отправлен
                    else begin
                        idx_next   = idx + 3'd1;
                        state_next = SEND;
                    end
                end
            end

            default: state_next = IDLE;
        endcase
    end

    // ---- Выходы ----
    assign res_ready = (state == IDLE);
    assign tx_start  = (state == SEND) && !tx_busy;
    assign tx_data   = reply_char(err_r, idx);

    // ---- Блок 2: на фронте clk запоминаем ----
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= IDLE;
            err_r <= ERR_NONE;
            idx   <= '0;
        end else begin
            state <= state_next;
            err_r <= err_r_next;
            idx   <= idx_next;
        end
    end

endmodule
