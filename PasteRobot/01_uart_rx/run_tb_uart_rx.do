# =============================================================================
# run_tb_uart_rx.do — компиляция и запуск теста приёмника UART
# -----------------------------------------------------------------------------
# Консоль (PowerShell), из ЭТОЙ папки:
#     vsim -batch -do run_tb_uart_rx.do
# Окно Questa: File > Change Directory... (эта папка), затем в строке Transcript:
#     do run_tb_uart_rx.do
#   В окне откроются осциллограммы (Wave), симуляция останется загруженной.
#
# Что смотреть в выводе: "Errors: 0" после vlog, строки "step N: ... - done"
# и в конце "TEST PASSED". Любое "ERROR" — тест нашёл проблему.
# =============================================================================

# В консоли: при любой ошибке (например, vlog не скомпилировал) сразу выходим
# с кодом 1, чтобы не запустить старую сборку из папки work.
if {[batch_mode]} { onerror {quit -f -code 1} }

# Чистая библиотека work: старая сборка не сможет "подсунуться" вместо новой
if {![batch_mode]} { catch {quit -sim} }   ;# в окне Questa выгружаем прошлую симуляцию
if {[file exists work]} { vdel -all -lib work }
vlib work

# Компиляция: сначала модуль, потом тест
vlog -sv uart_rx.sv tb_uart_rx.sv

# Запуск. -onfinish stop: на $finish симуляция останавливается, а не закрывает окно.
#         -voptargs=+acc: сохраняем все сигналы видимыми для Wave
vsim -onfinish stop -voptargs=+acc work.tb_uart_rx

if {![batch_mode]} {
    add wave -divider "Line and result"
    add wave /tb_uart_rx/rx
    add wave -radix hex /tb_uart_rx/rx_data /tb_uart_rx/rx_valid /tb_uart_rx/frame_err
    add wave -divider "Receiver inside"
    add wave /tb_uart_rx/dut/state
    add wave -radix unsigned /tb_uart_rx/dut/clk_cnt /tb_uart_rx/dut/bit_idx
    add wave -radix hex /tb_uart_rx/dut/shift_reg
}

run -all

if {[batch_mode]} { quit -f } else { wave zoom full }
