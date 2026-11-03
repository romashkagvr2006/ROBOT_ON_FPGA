// =============================================================================
// cmd_parser.sv — разбор строки команды (G-код) по одному символу
// -----------------------------------------------------------------------------
// ЧТО ДЕЛАЕТ
//   Получает из очереди байты строки, например  "G1 X20 Y30,5 E3" + CR LF,
//   и превращает её в структуру cmd_t (см. common/robot_pkg.sv):
//       opcode = OP_G1
//       has    = X, Y, E            (какие поля были в строке)
//       value  = 2000, 3050, 300    (числа в сотых долях)
//       err    = ERR_NONE           (или код ошибки, если строка неправильная)
//   Строку целиком нигде не хранит: разбирает "на лету", символ за символом,
//   и копит только то, что уже понятно (номер команды, текущее число).
//
// ОТКУДА БЕРЁТ И КУДА ОТДАЁТ
//   Вход:  in_valid / in_data / in_ready  <- byte_fifo (рукопожатие valid/ready).
//   Выход: cmd_valid / cmd / cmd_ready   -> motion_ctrl (тоже valid/ready).
//   Пока готовая команда не забрана (cmd_ready = 0, моторы ещё едут), парсер
//   держит in_ready = 0, и следующие байты спокойно ждут в очереди.
//
// ЧТО ПОНИМАЕТ (правила из файла "путь-команды")
//   G0 X.. Y..      переезд без пасты          G0 Z..   отдельно поднять/опустить
//   G1 X.. Y.. E..  движение с пастой          G1 E..   точка пасты на месте
//   G28 [X] [Y] [Z] хоминг всех осей или только указанных
//   M92 X.. Y.. Z.. E..  коэффициенты k (импульсов на мм / на мкл)
//   * число: необязательный знак, цифры, запятая или точка, до 2 цифр после неё:
//     20, -1.5, 30,5, +0.25, .5  -> 2000, -150, 3050, 25, 50 (в сотых)
//   * маленькие буквы тоже можно: "g1 x10";  пробелы между полями необязательны: "G1X5Y3"
//   * ';' — комментарий до конца строки:  "G28 ; ноль"
//   * конец строки — CR или LF (Termite шлёт оба, лишний пустой конец пропускается)
//   * пустые строки и строки из одного комментария молча пропускаются (без ответа)
//
// КОНЕЧНЫЙ АВТОМАТ (одно состояние = "что мы сейчас ждём в строке")
//
//   "G1 X20 Y30,5 E3\n"
//    ^^ ^^^ ^^^^^ ^^ ^
//    |  |   |      |  +-- конец строки: S_EMIT (отдаём команду)
//    |  |   |      +----- S_NUM_START -> S_NUM_INT
//    |  |   +------------ S_NUM_START -> S_NUM_INT -> (",") S_NUM_FRAC
//    |  +---------------- буква поля, затем S_NUM_START -> S_NUM_INT
//    +------------------- S_LINE_START ("G") -> S_CMD_NUM ("1")
//
//   Ошибка в середине строки -> S_ERROR_LINE: пропускаем всё до конца строки,
//   затем S_EMIT отдаёт команду с кодом ошибки — на ПК уйдёт "err N".
// =============================================================================
module cmd_parser
    import robot_pkg::*;
(
    input  logic       clk,
    input  logic       rst,         // сброс, активная 1
    // ---- байты строки (от byte_fifo) ----
    input  logic       in_valid,
    input  logic [7:0] in_data,
    output logic       in_ready,
    // ---- готовая команда (к motion_ctrl) ----
    output logic       cmd_valid,
    output cmd_t       cmd,
    input  logic       cmd_ready
);

    // ---- Коды символов ASCII --------------------------------------------------
    localparam logic [7:0] CH_TAB   = 8'h09;
    localparam logic [7:0] CH_LF    = 8'h0A;   // '\n'
    localparam logic [7:0] CH_CR    = 8'h0D;   // '\r'
    localparam logic [7:0] CH_SPACE = 8'h20;

    // ---- Маленькие функции "что это за символ" --------------------------------
    function automatic logic [7:0] to_upper(input logic [7:0] c);
        return (c >= "a" && c <= "z") ? (c - 8'd32) : c;   // 'a' - 'A' = 32
    endfunction

    function automatic logic is_digit(input logic [7:0] c);
        return (c >= "0" && c <= "9");
    endfunction

    function automatic logic is_space(input logic [7:0] c);
        return (c == CH_SPACE || c == CH_TAB);
    endfunction

    function automatic logic is_eol(input logic [7:0] c);
        return (c == CH_CR || c == CH_LF);
    endfunction

    function automatic logic is_axis_letter(input logic [7:0] c);
        return (c == "X" || c == "Y" || c == "Z" || c == "E");
    endfunction

    function automatic logic [1:0] axis_of(input logic [7:0] c);
        case (c)
            "X":     return 2'(AX_X);
            "Y":     return 2'(AX_Y);
            "Z":     return 2'(AX_Z);
            default: return 2'(AX_E);
        endcase
    endfunction

    // Символ, который заканчивает число: пробел, конец строки, ';' или буква следующего поля
    function automatic logic ends_number(input logic [7:0] c);
        return is_space(c) || is_eol(c) || c == ";" || is_axis_letter(c);
    endfunction

    // Буква и номер -> код команды
    function automatic opcode_t decode_opcode(input logic [7:0] letter, input logic [9:0] number);
        if (letter == "G" && number == 10'd0)  return OP_G0;
        if (letter == "G" && number == 10'd1)  return OP_G1;
        if (letter == "G" && number == 10'd28) return OP_G28;
        if (letter == "M" && number == 10'd92) return OP_M92;
        return OP_NONE;
    endfunction

    // Проверка смысла строки, когда она уже прочитана целиком (правила из "путь-команды")
    function automatic err_t check_fields(input opcode_t                    opcode,
                                          input logic [NUM_AXES-1:0]        has,
                                          input value_t [NUM_AXES-1:0]      value,
                                          input logic                       bare_field);
        case (opcode)
            OP_G0: begin                                   // переезд: E нельзя, Z только отдельно
                if (bare_field)                             return ERR_NUMBER;
                if (has[AX_E])                              return ERR_FIELDS;
                if (has[AX_Z] && (has[AX_X] || has[AX_Y]))  return ERR_FIELDS;
                return ERR_NONE;
            end
            OP_G1: begin                                   // движение с пастой: без Z
                if (bare_field)                             return ERR_NUMBER;
                if (has[AX_Z])                              return ERR_FIELDS;
                return ERR_NONE;
            end
            OP_G28: begin                                  // хоминг: числа не нужны, E нельзя
                if (has[AX_E])                              return ERR_FIELDS;
                return ERR_NONE;
            end
            OP_M92: begin                                  // коэффициенты: только больше нуля
                if (bare_field)                             return ERR_NUMBER;
                for (int a = 0; a < NUM_AXES; a++)
                    if (has[a] && value[a] <= 0)            return ERR_FIELDS;
                return ERR_NONE;
            end
            default:                                        return ERR_UNKNOWN;
        endcase
    endfunction

    // ---- Состояния автомата -----------------------------------------------------
    typedef enum logic [3:0] {
        S_LINE_START,   // начало строки: ждём букву команды G или M
        S_CMD_NUM,      // читаем номер команды (цифры после G/M)
        S_FIELDS,       // между полями: ждём X/Y/Z/E, пробел или конец строки
        S_NUM_START,    // первый символ числа после буквы поля: знак, цифра, точка
        S_NUM_INT,      // целая часть числа
        S_NUM_FRAC,     // дробная часть (после ',' или '.')
        S_COMMENT,      // после ';' в строке с командой: ждём конец строки
        S_SKIP_LINE,    // строка-комментарий без команды: молча ждём конец строки
        S_ERROR_LINE,   // в строке ошибка: ждём конец строки, потом ответим "err N"
        S_EMIT          // команда готова, ждём, пока motion_ctrl её заберёт
    } state_t;

    state_t state, state_next;

    // ---- Регистры строки и их "следующие" значения --------------------------------
    logic [7:0]                cmd_letter,  cmd_letter_next;   // 'G' или 'M'
    logic [9:0]                cmd_num,     cmd_num_next;      // номер команды: 0, 1, 28, 92
    logic [1:0]                cmd_digits,  cmd_digits_next;   // сколько цифр в номере (до 3)
    opcode_t                   opcode,      opcode_next;
    err_t                      err,         err_next;          // первая найденная ошибка
    logic [NUM_AXES-1:0]       has,         has_next;
    value_t [NUM_AXES-1:0]     value,       value_next;
    logic                      bare_field,  bare_field_next;   // было поле без числа ("G28 X")

    // ---- Регистры текущего числа --------------------------------------------------
    logic [1:0]                cur_axis,    cur_axis_next;     // к какой оси относится число
    logic [31:0]               num_acc,     num_acc_next;      // цифры числа без запятой: "30,5" -> 305
    logic                      num_neg,     num_neg_next;      // был знак '-'
    logic                      num_digit,   num_digit_next;    // встретилась хотя бы одна цифра
    logic                      num_big,     num_big_next;      // цифр слишком много
    logic [1:0]                num_frac,    num_frac_next;     // цифр после запятой: 0, 1, 2

    // ---- Текущий символ и вспомогательные сигналы --------------------------------
    wire        take_byte = in_valid && in_ready;     // на этом фронте забираем байт
    wire [7:0]  ch        = to_upper(in_data);
    wire [3:0]  ch_digit  = 4'(in_data - "0");        // значение цифры 0..9

    // Число, приведённое к сотым: 305 с одной цифрой после запятой -> 3050
    logic [39:0] num_scaled;
    value_t      num_signed;

    always_comb begin
        case (num_frac)
            2'd0:    num_scaled = 40'(num_acc) * 40'd100;
            2'd1:    num_scaled = 40'(num_acc) * 40'd10;
            default: num_scaled = 40'(num_acc);
        endcase
        num_signed = num_neg ? -$signed(num_scaled[VALUE_W-1:0]) : $signed(num_scaled[VALUE_W-1:0]);
    end

    wire num_ok = num_digit && !num_big && (num_scaled <= 40'(VALUE_MAX));

    // ---- Блок 1: что будет на следующем такте --------------------------------------
    // Внутри используются "события" — флаги, которые выставляет разбор символа:
    //   finish_num  — число закончилось, надо сохранить его в поле;
    //   after_field — поле закончилось этим символом, надо решить, куда дальше;
    //   start_field — символ является буквой нового поля X/Y/Z/E;
    //   line_end    — конец строки, команду пора отдавать;
    //   raise_err   — найдена ошибка.
    logic finish_num, after_field, start_field, line_end;
    err_t raise_err;

    always_comb begin
        // По умолчанию всё хранит значения
        state_next      = state;
        cmd_letter_next = cmd_letter;
        cmd_num_next    = cmd_num;
        cmd_digits_next = cmd_digits;
        opcode_next     = opcode;
        err_next        = err;
        has_next        = has;
        value_next      = value;
        bare_field_next = bare_field;
        cur_axis_next   = cur_axis;
        num_acc_next    = num_acc;
        num_neg_next    = num_neg;
        num_digit_next  = num_digit;
        num_big_next    = num_big;
        num_frac_next   = num_frac;

        finish_num  = 1'b0;
        after_field = 1'b0;
        start_field = 1'b0;
        line_end    = 1'b0;
        raise_err   = ERR_NONE;

        // ---------- Шаг 1: разбираем пришедший символ по текущему состоянию ----------
        if (take_byte) begin
            case (state)
                S_LINE_START: begin
                    if (is_eol(ch) || is_space(ch))
                        ;                                    // пустые строки и пробелы пропускаем
                    else if (ch == ";")
                        state_next = S_SKIP_LINE;            // строка-комментарий
                    else if (ch == "G" || ch == "M") begin
                        cmd_letter_next = ch;
                        cmd_num_next    = '0;
                        cmd_digits_next = '0;
                        state_next      = S_CMD_NUM;
                    end else
                        raise_err = ERR_UNKNOWN;             // строка начинается не с G/M
                end

                S_CMD_NUM: begin
                    if (is_digit(ch)) begin
                        if (cmd_digits == 2'd3)
                            raise_err = ERR_UNKNOWN;         // номер длиннее трёх цифр
                        else begin
                            cmd_num_next    = 10'(cmd_num * 10 + ch_digit);
                            cmd_digits_next = cmd_digits + 2'd1;
                        end
                    end else begin
                        // Номер закончился: узнаём команду
                        opcode_next = decode_opcode(cmd_letter, cmd_num);
                        if (cmd_digits == 2'd0 || decode_opcode(cmd_letter, cmd_num) == OP_NONE)
                            raise_err = ERR_UNKNOWN;
                        else if (ends_number(ch))
                            after_field = 1'b1;              // дальше как после поля
                        else
                            raise_err = ERR_SYNTAX;
                    end
                end

                S_FIELDS: begin
                    if (is_space(ch))
                        ;                                    // пробелы между полями
                    else if (is_eol(ch))
                        line_end = 1'b1;
                    else if (ch == ";")
                        state_next = S_COMMENT;
                    else if (is_axis_letter(ch))
                        start_field = 1'b1;
                    else
                        raise_err = ERR_SYNTAX;              // буква не X/Y/Z/E или мусор
                end

                S_NUM_START: begin
                    if (ch == "-") begin
                        num_neg_next = 1'b1;
                        state_next   = S_NUM_INT;
                    end else if (ch == "+") begin
                        state_next   = S_NUM_INT;
                    end else if (is_digit(ch)) begin
                        num_acc_next   = 32'(ch_digit);
                        num_digit_next = 1'b1;
                        state_next     = S_NUM_INT;
                    end else if (ch == "." || ch == ",") begin
                        state_next     = S_NUM_FRAC;         // запись вида ".5"
                    end else if (ends_number(ch)) begin
                        // Поле без числа, например "G28 X". Разрешено только для G28,
                        // это проверит check_fields в конце строки.
                        bare_field_next          = 1'b1;
                        has_next[cur_axis]       = 1'b1;
                        value_next[cur_axis]     = '0;
                        after_field              = 1'b1;
                    end else
                        raise_err = ERR_NUMBER;
                end

                S_NUM_INT: begin
                    if (is_digit(ch)) begin
                        if (num_acc > 32'(VALUE_MAX))
                            num_big_next = 1'b1;             // дальше не растим, чтобы не переполнить
                        else
                            num_acc_next = num_acc * 10 + 32'(ch_digit);
                        num_digit_next = 1'b1;
                    end else if (ch == "." || ch == ",")
                        state_next = S_NUM_FRAC;
                    else if (ends_number(ch))
                        finish_num = 1'b1;
                    else
                        raise_err = ERR_NUMBER;
                end

                S_NUM_FRAC: begin
                    if (is_digit(ch)) begin
                        if (num_frac == 2'd2)
                            raise_err = ERR_NUMBER;          // больше двух цифр после запятой
                        else begin
                            if (num_acc > 32'(VALUE_MAX))
                                num_big_next = 1'b1;
                            else
                                num_acc_next = num_acc * 10 + 32'(ch_digit);
                            num_frac_next  = num_frac + 2'd1;
                            num_digit_next = 1'b1;
                        end
                    end else if (ends_number(ch))
                        finish_num = 1'b1;
                    else
                        raise_err = ERR_NUMBER;
                end

                S_COMMENT: begin
                    if (is_eol(ch)) line_end = 1'b1;
                end

                S_SKIP_LINE: begin
                    if (is_eol(ch)) state_next = S_LINE_START;
                end

                S_ERROR_LINE: begin
                    if (is_eol(ch)) state_next = S_EMIT;     // ошибка уже записана в err
                end

                default: ;                                   // в S_EMIT байты не берём
            endcase
        end

        // ---------- Шаг 2: число закончилось — сохраняем его в поле ----------
        if (finish_num) begin
            if (!num_ok)
                raise_err = ERR_NUMBER;
            else begin
                has_next[cur_axis]   = 1'b1;
                value_next[cur_axis] = num_signed;
                after_field          = 1'b1;
            end
        end

        // ---------- Шаг 3: поле (или номер команды) закончилось — куда дальше ----------
        if (after_field) begin
            if (is_space(ch))      state_next = S_FIELDS;
            else if (is_eol(ch))   line_end   = 1'b1;
            else if (ch == ";")    state_next = S_COMMENT;
            else                   start_field = 1'b1;       // сразу буква: "X5Y3"
        end

        // ---------- Шаг 4: начинается новое поле X/Y/Z/E ----------
        if (start_field) begin
            if (has_next[axis_of(ch)])
                raise_err = ERR_SYNTAX;                      // поле повторено: "X1 X2"
            else begin
                cur_axis_next  = axis_of(ch);
                num_acc_next   = '0;
                num_neg_next   = 1'b0;
                num_digit_next = 1'b0;
                num_big_next   = 1'b0;
                num_frac_next  = '0;
                state_next     = S_NUM_START;
            end
        end

        // ---------- Шаг 5: конец строки — команда готова ----------
        if (line_end)
            state_next = S_EMIT;

        // ---------- Шаг 6: ошибка важнее всего остального ----------
        if (raise_err != ERR_NONE) begin
            err_next   = raise_err;
            state_next = is_eol(ch) ? S_EMIT : S_ERROR_LINE;
        end

        // ---------- Шаг 7: команду забрали — готовимся к новой строке ----------
        if (state == S_EMIT && cmd_ready) begin
            state_next      = S_LINE_START;
            opcode_next     = OP_NONE;
            err_next        = ERR_NONE;
            has_next        = '0;
            value_next      = '0;
            bare_field_next = 1'b0;
        end
    end

    // ---- Выходы -----------------------------------------------------------------------
    assign in_ready   = (state != S_EMIT);       // пока команда не забрана, байты ждут в FIFO
    assign cmd_valid  = (state == S_EMIT);

    always_comb begin
        cmd.opcode = opcode;
        cmd.has    = has;
        cmd.value  = value;
        cmd.err    = (err != ERR_NONE) ? err : check_fields(opcode, has, value, bare_field);
    end

    // ---- Блок 2: на фронте clk запоминаем посчитанное ---------------------------------
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            state      <= S_LINE_START;
            cmd_letter <= '0;
            cmd_num    <= '0;
            cmd_digits <= '0;
            opcode     <= OP_NONE;
            err        <= ERR_NONE;
            has        <= '0;
            value      <= '0;
            bare_field <= 1'b0;
            cur_axis   <= '0;
            num_acc    <= '0;
            num_neg    <= 1'b0;
            num_digit  <= 1'b0;
            num_big    <= 1'b0;
            num_frac   <= '0;
        end else begin
            state      <= state_next;
            cmd_letter <= cmd_letter_next;
            cmd_num    <= cmd_num_next;
            cmd_digits <= cmd_digits_next;
            opcode     <= opcode_next;
            err        <= err_next;
            has        <= has_next;
            value      <= value_next;
            bare_field <= bare_field_next;
            cur_axis   <= cur_axis_next;
            num_acc    <= num_acc_next;
            num_neg    <= num_neg_next;
            num_digit  <= num_digit_next;
            num_big    <= num_big_next;
            num_frac   <= num_frac_next;
        end
    end

endmodule
