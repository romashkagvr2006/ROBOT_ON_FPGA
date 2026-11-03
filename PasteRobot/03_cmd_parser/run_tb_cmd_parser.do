# =============================================================================
# run_tb_cmd_parser.do — компиляция и запуск: теста разбора строк G-кода (cmd_parser)
# -----------------------------------------------------------------------------
# Консоль (PowerShell), из ЭТОЙ папки:
#     vsim -batch -do run_tb_cmd_parser.do
# Окно Questa: File > Change Directory... (эта папка), затем в строке Transcript:
#     do run_tb_cmd_parser.do
#   В окне откроются осциллограммы (Wave), симуляция останется загруженной.
#
# Что смотреть в выводе: "Errors: 0" после vlog, строки "OK  ..." на каждую проверенную строку
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
vlog -sv ../common/robot_pkg.sv cmd_parser.sv tb_cmd_parser.sv

# Запуск. -onfinish stop: на $finish симуляция останавливается, а не закрывает окно.
#         -voptargs=+acc: сохраняем все сигналы видимыми для Wave
vsim -onfinish stop -voptargs=+acc work.tb_cmd_parser

if {![batch_mode]} {
    add wave -divider "Bytes in (from FIFO)"
    add wave /tb_cmd_parser/in_valid /tb_cmd_parser/in_ready
    add wave -radix ascii /tb_cmd_parser/in_data
    add wave -divider "Parser inside"
    add wave /tb_cmd_parser/dut/state
    add wave -radix unsigned /tb_cmd_parser/dut/num_acc /tb_cmd_parser/dut/num_frac
    add wave -divider "Command out (to motion_ctrl)"
    add wave /tb_cmd_parser/cmd_valid /tb_cmd_parser/cmd_ready
    add wave -radix decimal /tb_cmd_parser/cmd
}

run -all

if {[batch_mode]} { quit -f } else { wave zoom full }
