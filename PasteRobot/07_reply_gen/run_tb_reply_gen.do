# =============================================================================
# run_tb_reply_gen.do — компиляция и запуск: теста генератора ответов ok/err (reply_gen + uart_tx)
# -----------------------------------------------------------------------------
# Консоль (PowerShell), из ЭТОЙ папки:
#     vsim -batch -do run_tb_reply_gen.do
# Окно Questa: File > Change Directory... (эта папка), затем в строке Transcript:
#     do run_tb_reply_gen.do
#   В окне откроются осциллограммы (Wave), симуляция останется загруженной.
#
# Что смотреть в выводе: "Errors: 0" после vlog, строки "OK   PC received ..."
# и в конце "TEST PASSED". Любое "ERROR" — тест нашёл проблему.
# =============================================================================

# В консоли: при любой ошибке (например, vlog не скомпилировал) сразу выходим
# с кодом 1, чтобы не запустить старую сборку из папки work.
if {[batch_mode]} { onerror {quit -f -code 1} }

# Чистая библиотека work: старая сборка не сможет "подсунуться" вместо новой
if {![batch_mode]} { catch {quit -sim} }   ;# в окне Questa выгружаем прошлую симуляцию
if {[file exists work]} { vdel -all -lib work }
vlib work

# Компиляция: общий пакет (если нужен), модули, потом тест
vlog -sv ../common/robot_pkg.sv ../08_uart_tx/uart_tx.sv reply_gen.sv tb_reply_gen.sv

# Запуск. -onfinish stop: на $finish симуляция останавливается, а не закрывает окно.
#         -voptargs=+acc: сохраняем все сигналы видимыми для Wave
vsim -onfinish stop -voptargs=+acc work.tb_reply_gen

if {![batch_mode]} {
    add wave -divider "Result from motion_ctrl"
    add wave /tb_reply_gen/res_valid /tb_reply_gen/res_ready /tb_reply_gen/res_err
    add wave -divider "Reply generator"
    add wave /tb_reply_gen/u_reply/state
    add wave -radix unsigned /tb_reply_gen/u_reply/idx
    add wave /tb_reply_gen/tx_start /tb_reply_gen/tx_busy
    add wave -radix ascii /tb_reply_gen/tx_data
    add wave -divider "Line to PC"
    add wave /tb_reply_gen/tx
}

run -all

if {[batch_mode]} { quit -f } else { wave zoom full }
