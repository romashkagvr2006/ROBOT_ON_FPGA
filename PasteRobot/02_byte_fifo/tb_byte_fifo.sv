// =============================================================================
// tb_byte_fifo.sv — тест очереди байт
// -----------------------------------------------------------------------------
// Рядом с очередью тестбенч держит "эталонную" очередь model_q (очередь языка
// SystemVerilog). Каждый записанный байт кладётся и туда, каждый прочитанный
// сравнивается с эталоном. Если порядок или значение не совпали — ошибка.
//
// Сценарии:
//   1. После сброса очередь пуста (rd_valid = 0, full = 0, overflow = 0).
//   2. Один байт: записали, сразу видим его на выходе.
//   3. Заполнение: пишем, не читая, пока не загорится full; сколько влезло.
//   4. Переполнение: ещё один байт в полную очередь -> overflow = 1, байт потерян.
//   5. Опустошение: читаем всё, порядок совпадает с эталоном.
//   6. Случайная нагрузка: 5000 тактов, запись и чтение в случайные моменты,
//      в том числе одновременно.
// =============================================================================
`timescale 1ns/1ps
module tb_byte_fifo;
    localparam int DEPTH  = 16;     // маленькая очередь, чтобы быстро её заполнить
    localparam real CLK_NS = 20.0;

    logic       clk = 1'b0;
    logic       rst = 1'b1;
    logic       wr_valid = 1'b0;
    logic [7:0] wr_data  = '0;
    logic       full, overflow;
    logic       rd_valid;
    logic [7:0] rd_data;
    logic       rd_ready = 1'b0;

    int errors = 0;
    int reads  = 0;

    byte_fifo #(.DEPTH(DEPTH)) dut (.*);

    always #(CLK_NS / 2) clk = ~clk;

    // ---- Эталон ----------------------------------------------------------------
    logic [7:0] model_q[$];

    // Всё, что происходит на фронте clk, проверяем по значениям ДО фронта:
    // тестбенч меняет входы на спаде clk, а смотрит на выходы тоже на спаде.
    always @(posedge clk) begin
        if (!rst) begin
            if (wr_valid && !full)
                model_q.push_back(wr_data);
            if (rd_valid && rd_ready) begin
                reads++;
                if (model_q.size() == 0) begin
                    $display("ERROR: FIFO gave 0x%02h, but model is empty", rd_data);
                    errors++;
                end else begin
                    automatic logic [7:0] expected = model_q.pop_front();
                    if (rd_data !== expected) begin
                        $display("ERROR: FIFO gave 0x%02h, expected 0x%02h", rd_data, expected);
                        errors++;
                    end
                end
            end
        end
    end

    // Записать один байт (на спаде выставили, фронт его забрал)
    task automatic write_byte(input logic [7:0] b);
        @(negedge clk);
        wr_valid = 1'b1;
        wr_data  = b;
        @(negedge clk);
        wr_valid = 1'b0;
    endtask

    initial begin
        int accepted;

        repeat (3) @(posedge clk);
        @(negedge clk) rst = 1'b0;

        // 1. После сброса пусто
        @(negedge clk);
        if (rd_valid || full || overflow) begin
            $display("ERROR [1]: after reset rd_valid=%b full=%b overflow=%b",
                     rd_valid, full, overflow);
            errors++;
        end
        $display("step 1: empty after reset - done");

        // 2. Один байт
        write_byte(8'hA1);
        @(negedge clk);
        if (!rd_valid || rd_data !== 8'hA1) begin
            $display("ERROR [2]: expected rd_valid=1 rd_data=A1, got %b %02h", rd_valid, rd_data);
            errors++;
        end
        rd_ready = 1'b1; @(negedge clk); rd_ready = 1'b0;
        $display("step 2: single byte - done");

        // 3. Заполнение без чтения
        accepted = 0;
        while (!full && accepted < DEPTH + 10) begin
            write_byte(8'(8'h10 + accepted));
            accepted++;
        end
        // Влезает DEPTH байт в память + 1 байт в выходной регистр
        if (accepted != DEPTH + 1) begin
            $display("ERROR [3]: FIFO accepted %0d bytes before full, expected %0d",
                     accepted, DEPTH + 1);
            errors++;
        end
        $display("step 3: filled with %0d bytes - done", accepted);

        // 4. Переполнение
        write_byte(8'hEE);
        @(negedge clk);
        if (!overflow) begin
            $display("ERROR [4]: overflow must be 1"); errors++;
        end
        $display("step 4: overflow flag - done");

        // 5. Опустошение
        rd_ready = 1'b1;
        repeat (DEPTH + 5) @(negedge clk);
        rd_ready = 1'b0;
        if (rd_valid || model_q.size() != 0) begin
            $display("ERROR [5]: after drain rd_valid=%b, model has %0d bytes",
                     rd_valid, model_q.size());
            errors++;
        end
        $display("step 5: drained in order - done");

        // 6. Случайная нагрузка
        repeat (5000) begin
            @(negedge clk);
            wr_valid = ($urandom_range(0, 99) < 45);
            wr_data  = 8'($urandom);
            rd_ready = ($urandom_range(0, 99) < 50);
        end
        @(negedge clk);
        wr_valid = 1'b0;
        rd_ready = 1'b1;
        repeat (DEPTH + 5) @(negedge clk);
        if (model_q.size() != 0) begin
            $display("ERROR [6]: %0d bytes left in model after random test", model_q.size());
            errors++;
        end
        $display("step 6: random traffic, %0d bytes read in total - done", reads);

        $display("------------------------------------------------");
        if (errors == 0) $display("TEST PASSED");
        else             $display("TEST FAILED: %0d errors", errors);
        $finish;
    end

    initial begin
        #(CLK_NS * 20_000);
        $display("ERROR: simulation timeout");
        errors++;
        $finish;
    end

endmodule
