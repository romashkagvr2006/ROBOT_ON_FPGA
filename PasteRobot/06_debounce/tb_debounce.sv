// =============================================================================
// tb_debounce.sv — тест антидребезга
// -----------------------------------------------------------------------------
// Сценарии (STABLE_CLKS = 50 тактов, чтобы тест шёл быстро):
//   1. После сброса выход равен RESET_LEVEL (1).
//   2. Короткие помехи (короче STABLE_CLKS) на выход не проходят.
//   3. Нажатие с дребезгом: 6 "прыжков", затем вход стабильно 0.
//      Выход должен упасть ровно один раз и не раньше, чем через STABLE_CLKS
//      тактов после последнего прыжка.
//   4. Отпускание с дребезгом: выход один раз поднимается в 1.
//   5. Задержка: выход меняется через STABLE_CLKS + 2..3 такта
//      (2 такта синхронизатор + сам счёт).
// =============================================================================
`timescale 1ns/1ps
module tb_debounce;
    localparam int  STABLE_CLKS = 50;
    localparam real CLK_NS      = 20.0;

    logic clk = 1'b0;
    logic rst = 1'b1;
    logic raw = 1'b1;
    logic clean;

    int errors  = 0;
    int changes = 0;     // сколько раз менялся выход

    debounce #(.STABLE_CLKS(STABLE_CLKS), .RESET_LEVEL(1'b1)) dut (.*);

    always #(CLK_NS / 2) clk = ~clk;

    always @(clean) if (!rst) changes++;

    // Подержать вход в значении v заданное число тактов
    task automatic hold(input logic v, input int clks);
        raw = v;
        repeat (clks) @(posedge clk);
    endtask

    // Измерить, через сколько тактов после установки v выход станет v
    task automatic measure_delay(input logic v, output int clks);
        raw  = v;
        clks = 0;
        while (clean !== v && clks < 10 * STABLE_CLKS) begin
            @(posedge clk);
            clks++;
        end
    endtask

    initial begin
        int delay;

        repeat (3) @(posedge clk);
        rst = 1'b0;

        // 1. После сброса
        if (clean !== 1'b1) begin $display("ERROR [1]: clean after reset = %b", clean); errors++; end
        $display("step 1: reset level - done");

        // 2. Короткие помехи
        changes = 0;
        repeat (10) begin
            hold(1'b0, STABLE_CLKS / 3);
            hold(1'b1, STABLE_CLKS / 2);
        end
        hold(1'b1, 2 * STABLE_CLKS);
        if (changes != 0 || clean !== 1'b1) begin
            $display("ERROR [2]: glitches leaked: %0d changes", changes); errors++;
        end
        $display("step 2: short glitches ignored - done");

        // 3. Нажатие с дребезгом
        changes = 0;
        for (int i = 0; i < 6; i++) begin
            hold(1'b0, 3 + i * 4);
            hold(1'b1, 2 + i);
        end
        measure_delay(1'b0, delay);
        hold(1'b0, 2 * STABLE_CLKS);
        if (changes != 1 || clean !== 1'b0) begin
            $display("ERROR [3]: press: %0d changes, clean = %b", changes, clean); errors++;
        end
        if (delay < STABLE_CLKS || delay > STABLE_CLKS + 3) begin
            $display("ERROR [3]: press delay %0d clocks, expected %0d..%0d",
                     delay, STABLE_CLKS, STABLE_CLKS + 3);
            errors++;
        end
        $display("step 3: bouncy press -> one clean edge after %0d clocks - done", delay);

        // 4. Отпускание с дребезгом
        changes = 0;
        for (int i = 0; i < 6; i++) begin
            hold(1'b1, 2 + i * 3);
            hold(1'b0, 4);
        end
        measure_delay(1'b1, delay);
        hold(1'b1, 2 * STABLE_CLKS);
        if (changes != 1 || clean !== 1'b1) begin
            $display("ERROR [4]: release: %0d changes, clean = %b", changes, clean); errors++;
        end
        $display("step 4: bouncy release -> one clean edge after %0d clocks - done", delay);

        $display("------------------------------------------------");
        if (errors == 0) $display("TEST PASSED");
        else             $display("TEST FAILED: %0d errors", errors);
        $finish;
    end

    initial begin
        #(CLK_NS * 100_000);
        $display("ERROR: simulation timeout");
        errors++;
        $finish;
    end

endmodule
