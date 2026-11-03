# =============================================================================
# run_tb_uart_tx.do — компиляция и запуск теста передатчика UART (ваш модуль из Uart/TX)
# -----------------------------------------------------------------------------
# Консоль (PowerShell), из ЭТОЙ папки:
#     vsim -batch -do run_tb_uart_tx.do
# Окно Questa: File > Change Directory... (эта папка), затем в строке Transcript:
#     do run_tb_uart_tx.do
#   В окне откроются осциллограммы (Wave), симуляция останется загруженной.
#
# Что смотреть в выводе: "Errors: 0" после vlog, строки "OK: 0x.." на каждый байт
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
vlog -sv uart_tx.sv tb_uart_tx.sv

# Запуск. -onfinish stop: на $finish симуляция останавливается, а не закрывает окно.
#         -voptargs=+acc: сохраняем все сигналы видимыми для Wave
vsim -onfinish stop -voptargs=+acc work.tb_uart_tx

if {![batch_mode]} {
    add wave -divider "Input byte"
    add wave /tb_uart_tx/tx_start /tb_uart_tx/busy
    add wave -radix hex /tb_uart_tx/tx_data
    add wave -divider "Line and transmitter inside"
    add wave /tb_uart_tx/tx
    add wave /tb_uart_tx/dut/state
    add wave -radix unsigned /tb_uart_tx/dut/clk_cnt /tb_uart_tx/dut/bit_idx
}

run -all

if {[batch_mode]} { quit -f } else { wave zoom full }
