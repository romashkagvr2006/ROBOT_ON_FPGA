// =============================================================================
// tb_udiv.sv — тест последовательного делителя
// -----------------------------------------------------------------------------
// Эталон — обычные операции "/" и "%" языка SystemVerilog над 64-битными
// числами. Делитель должен дать то же частное и тот же остаток.
//
// Сценарии:
//   1. Граничные случаи: 0, делимое меньше делителя, делитель 1, максимальное
//      делимое, деление ровно без остатка.
//   2. Деление на 10000 с округлением — именно так считает motion_ctrl:
//      (20,00 мм × 400,00 имп/мм + 5000) / 10000 = 8000 импульсов и т. п.
//   3. 2000 случайных пар.
//   4. Время: результат ровно через N_W + 1 тактов после start.
// =============================================================================
`timescale 1ns/1ps
module tb_udiv;
    localparam int  N_W    = 50;
    localparam int  D_W    = 14;
    localparam real CLK_NS = 20.0;

    logic           clk = 1'b0;
    logic           rst = 1'b1;
    logic           start = 1'b0;
    logic [N_W-1:0] dividend = '0;
    logic [D_W-1:0] divisor  = '0;
    logic           busy, done;
    logic [N_W-1:0] quotient;
    logic [D_W-1:0] remainder;

    int errors = 0;
    int checks = 0;

    udiv #(.N_W(N_W), .D_W(D_W)) dut (.*);

    always #(CLK_NS / 2) clk = ~clk;

    // Поделить и сравнить с эталоном; заодно меряем, сколько тактов заняло
    task automatic check_div(input longint unsigned a, input longint unsigned b);
        longint unsigned exp_q, exp_r;
        int              cycles;
        exp_q = a / b;
        exp_r = a % b;

        @(negedge clk);
        dividend = N_W'(a);
        divisor  = D_W'(b);
        start    = 1'b1;
        @(negedge clk);
        start    = 1'b0;
        cycles   = 1;
        while (!done) begin
            @(negedge clk);
            cycles++;
        end

        checks++;
        if (quotient !== N_W'(exp_q) || remainder !== D_W'(exp_r)) begin
            $display("ERROR: %0d / %0d: got q=%0d r=%0d, expected q=%0d r=%0d",
                     a, b, quotient, remainder, exp_q, exp_r);
            errors++;
        end
        if (cycles != N_W + 1) begin
            $display("ERROR: %0d / %0d took %0d cycles, expected %0d", a, b, cycles, N_W + 1);
            errors++;
        end
    endtask

    initial begin
        longint unsigned a, b;
        longint unsigned max_dividend;
        max_dividend = (64'd1 << N_W) - 1;

        repeat (3) @(posedge clk);
        @(negedge clk) rst = 1'b0;

        // 1. Граничные случаи
        check_div(0, 7);
        check_div(5, 7);
        check_div(7, 7);
        check_div(13, 3);
        check_div(123_456_789, 1);
        check_div(max_dividend, 1);
        check_div(max_dividend, 10_000);
        check_div(max_dividend, (1 << D_W) - 1);
        check_div(1_000_000, 10_000);
        $display("step 1: edge cases - done");

        // 2. Как в motion_ctrl: (координата × k + 5000) / 10000
        check_div(64'd2000 * 64'd40000 + 5000, 10_000);    // X20 при k = 400  -> 8000
        check_div(64'd200 * 64'd9220 + 5000, 10_000);      // E2 при k = 92,2  -> 184
        check_div(64'd5 * 64'd9220 + 5000, 10_000);        // E0,05 -> 4,61 -> 5
        check_div(64'd9_999_999 * 64'd9_999_999 + 5000, 10_000);  // самый большой случай
        $display("step 2: mm -> pulses with rounding - done");

        // 3. Случайные пары
        repeat (2000) begin
            a = {$urandom, $urandom} & max_dividend;
            b = $urandom_range(1, (1 << D_W) - 1);
            check_div(a, b);
        end
        $display("step 3: 2000 random pairs - done");

        $display("------------------------------------------------");
        $display("divisions checked: %0d (each must take %0d cycles)", checks, N_W + 1);
        if (errors == 0) $display("TEST PASSED");
        else             $display("TEST FAILED: %0d errors", errors);
        $finish;
    end

    initial begin
        #(CLK_NS * 500_000);
        $display("ERROR: simulation timeout");
        errors++;
        $finish;
    end

endmodule
