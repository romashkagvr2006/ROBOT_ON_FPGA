# =============================================================================
# run_tb_robot_top.do — компиляция и запуск: сквозного теста всей цепочки (robot_top)
# -----------------------------------------------------------------------------
# Консоль (PowerShell), из ЭТОЙ папки:
#     vsim -batch -do run_tb_robot_top.do
# Окно Questa: File > Change Directory... (эта папка), затем в строке Transcript:
#     do run_tb_robot_top.do
#   В окне откроются осциллограммы (Wave), симуляция останется загруженной.
#
# Что смотреть в выводе: "Errors: 0" после vlog, строки "OK   <команда> -> \"ok\" pulses ..."
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
vlog -sv ../common/robot_pkg.sv ../01_uart_rx/uart_rx.sv ../02_byte_fifo/byte_fifo.sv ../03_cmd_parser/cmd_parser.sv ../04_udiv/udiv.sv ../05_motion_ctrl/motion_ctrl.sv ../06_debounce/debounce.sv ../07_reply_gen/reply_gen.sv ../08_uart_tx/uart_tx.sv robot_top.sv tb_robot_top.sv

# Запуск. -onfinish stop: на $finish симуляция останавливается, а не закрывает окно.
#         -voptargs=+acc: сохраняем все сигналы видимыми для Wave
vsim -onfinish stop -voptargs=+acc work.tb_robot_top

if {![batch_mode]} {
    add wave -divider "PC <-> FPGA"
    add wave /tb_robot_top/uart_rxd /tb_robot_top/uart_txd
    add wave -divider "Bytes received / FIFO"
    add wave /tb_robot_top/dut/rx_valid
    add wave -radix ascii /tb_robot_top/dut/rx_data /tb_robot_top/dut/fifo_data
    add wave /tb_robot_top/dut/fifo_valid /tb_robot_top/dut/fifo_ready
    add wave -divider "Parser -> motion"
    add wave /tb_robot_top/dut/u_parser/state
    add wave /tb_robot_top/dut/cmd_valid /tb_robot_top/dut/cmd_ready
    add wave -radix decimal /tb_robot_top/dut/cmd
    add wave /tb_robot_top/dut/u_motion/state
    add wave -divider "Motors  [0]=X [1]=Y [2]=Z [3]=E"
    add wave -expand /tb_robot_top/motor_pul
    add wave -expand /tb_robot_top/motor_dir
    add wave /tb_robot_top/limit_x_raw /tb_robot_top/limit_y_raw /tb_robot_top/limit_z_raw
    add wave -divider "Reply"
    add wave /tb_robot_top/dut/res_valid /tb_robot_top/dut/res_err
    add wave -radix ascii /tb_robot_top/dut/tx_data
    add wave /tb_robot_top/led_busy /tb_robot_top/led_error /tb_robot_top/led_overflow
    add wave -radix decimal /tb_robot_top/phys
}

run -all

if {[batch_mode]} { quit -f } else { wave zoom full }
