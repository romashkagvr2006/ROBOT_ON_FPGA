// UART-передатчик: 8 бит данных, без чётности, 1 стоп-бит (8N1).
// Старт передачи: импульс tx_start на 1 такт при busy == 0.
module uart_tx #(
    parameter int CLK_HZ = 50_000_000,  //тактовая частота платы
    parameter int BAUD   = 115_200      // скорость передаччи бит/сек
) (
    input  logic       clk,      // тактовый импульс как обысно
    input  logic       rst,     // импульс сброса
    input  logic       tx_start,  // запрос на отправку байта
    input  logic [7:0] tx_data,   // байт который нуно передать по Uart
    output logic       tx,        // линия TX (в покое = 1)
    output logic       busy       // 1, пока идёт передача
);
    localparam int CLKS_PER_BIT = CLK_HZ / BAUD;  // 434 при 50 МГц / 115200 количесвто тактов на один бит информации
    localparam int CNT_W        = $clog2(CLKS_PER_BIT); // количсетво бит для счетчика

    typedef enum logic [1:0] {
     IDLE, // стоим ждем
     START, // начинаем сроку байт
     DATA, // передаем данные
     STOP // стоп-бит заверили передачу байт
     }
     state_t;
    state_t state, state_next; // настоящее и последущее состояние передатчика

    // Регистры (меняются только в always_ff) и их "следующие" значения (считаются в always_comb)
    logic [CNT_W-1:0] clk_cnt, clk_cnt_next;   // счётчик тактов внутри одного бита
    logic [2:0]       bit_idx, bit_idx_next;   // номер передаваемого бита данных
    logic [7:0]       shreg,   shreg_next;     // защёлкнутый байт то есть когда прихоит наш байт который надо отправить по протоколу то он ложится сюда извне
    logic             tx_next;                 // следующее значение линии TX

    wire bit_done = (clk_cnt == CNT_W'(CLKS_PER_BIT - 1)); // равен единице когда счетчик clk_cnt дошел и отсчитал 434 такта (столько нужно на один бит информации)

// опишем из чего состоит каждое состояние
    always_comb begin : States
        state_next   = state; // если ничего не меняемся то стоим на месте просто
        clk_cnt_next = bit_done ? '0 : clk_cnt + 1'b1; // по умолчанию счетчик тикает и сбрасывается в 0 на конце бита
        bit_idx_next = bit_idx; // по умолчанию все регистры хранят свое значение
        shreg_next   = shreg;
        tx_next      = tx;
        case (state)
           IDLE: begin
              clk_cnt_next = '0; // держим на нуле наш сетчик тактов clk на один бит
              tx_next      = 1'b1; // держим линию на высоком напряжении (логическая 1)
              if (tx_start) begin
                  shreg_next = tx_data; // вот пришел наш байт на отправку мы засунули его в шрег
                  tx_next    = 1'b0; // переводим в старт наш стартовый бит (кидаем в маленькое напряжение)
                  state_next = START; // переводим состояние в стартовое
              end
            end
           START: begin
              if (bit_done) begin // когда счетчикотсчитал 434 такта нужные на наш бит данных
                 bit_idx_next = 3'd0; // первый пришедший бит его индекс нулевой записан 3 битами
                 tx_next      = shreg[0]; // первым битом вперед начинаем запись
                 state_next   = DATA; // старт-бит отправлен, переходим к данным
              end
            end
           DATA: begin
             if (bit_done) begin
                 if (bit_idx == 3'd7) begin
                     tx_next    = 1'b1; // присваиваем на линию передачи стоп-бит
                     state_next = STOP; // следующее состояние будет стоповое логично
                 end
                 else begin
                     bit_idx_next = bit_idx + 3'd1; // увеличиваем номер бита следующего
                     tx_next      = shreg[bit_idx + 3'd1]; // вносим в линию передачи бит
                 end
             end
           end
           STOP: begin
             if (bit_done) state_next = IDLE;
           end
           default : state_next = IDLE; // если вдруг ошибка то линия выйдет на ожидание тут же
        endcase
    end

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            state   <= IDLE;
            clk_cnt <= '0;
            bit_idx <= '0;
            shreg   <= '0;
            tx      <= 1'b1;
        end else begin
            state   <= state_next;   // на каждом фронте clk регистры просто
            clk_cnt <= clk_cnt_next; // забирают значения, посчитанные в always_comb
            bit_idx <= bit_idx_next;
            shreg   <= shreg_next;
            tx      <= tx_next;
        end
    end

    assign busy = (state != IDLE);
endmodule
