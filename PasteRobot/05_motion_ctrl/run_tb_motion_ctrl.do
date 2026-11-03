# =============================================================================
# run_tb_motion_ctrl.do — компиляция и запуск: теста контроллера движения (motion_ctrl)
# -----------------------------------------------------------------------------
# Консоль (PowerShell), из ЭТОЙ папки:
#     vsim -batch -do run_tb_motion_ctrl.do
# Окно Questa: File > Change Directory... (эта папка), затем в строке Transcript:
#     do run_tb_motion_ctrl.do
#   В окне откроются осциллограммы (Wave), симуляция останется загруженной.
#
# Что смотреть в выводе: "Errors: 0" после vlog, строки "OK   <команда> -> ERR_NONE pulses X=.. Y=.."
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
vlog -sv ../common/robot_pkg.sv ../04_udiv/udiv.sv motion_ctrl.sv tb_motion_ctrl.sv

# Запуск. -onfinish stop: на $finish симуляция останавливается, а не закрывает окно.
#         -voptargs=+acc: сохраняем все сигналы видимыми для Wave
vsim -onfinish stop -voptargs=+acc work.tb_motion_ctrl

if {![batch_mode]} {
    add wave -divider "Command and result"
    add wave /tb_motion_ctrl/cmd_valid /tb_motion_ctrl/cmd_ready
    add wave -radix decimal /tb_motion_ctrl/cmd
    add wave /tb_motion_ctrl/res_valid /tb_motion_ctrl/res_err /tb_motion_ctrl/res_ready
    add wave -divider "Driver outputs  [0]=X [1]=Y [2]=Z [3]=E"
    add wave -expand /tb_motion_ctrl/step_pul
    add wave -expand /tb_motion_ctrl/step_dir
    add wave /tb_motion_ctrl/limit
    add wave -divider "Controller inside"
    add wave /tb_motion_ctrl/dut/state
    add wave -radix unsigned /tb_motion_ctrl/dut/n /tb_motion_ctrl/dut/n_max /tb_motion_ctrl/dut/acc /tb_motion_ctrl/dut/steps_done
    add wave -radix decimal /tb_motion_ctrl/pos_x /tb_motion_ctrl/pos_y /tb_motion_ctrl/pos_z
    add wave -divider "Mechanics model"
    add wave -radix decimal /tb_motion_ctrl/phys
}

run -all

if {[batch_mode]} { quit -f } else { wave zoom full }
