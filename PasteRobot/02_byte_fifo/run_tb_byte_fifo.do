# =============================================================================
# run_tb_byte_fifo.do — компиляция и запуск: теста очереди байт (FIFO)
# -----------------------------------------------------------------------------
# Консоль (PowerShell), из ЭТОЙ папки:
#     vsim -batch -do run_tb_byte_fifo.do
# Окно Questa: File > Change Directory... (эта папка), затем в строке Transcript:
#     do run_tb_byte_fifo.do
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

# Компиляция: общий пакет (если нужен), модули, потом тест
vlog -sv byte_fifo.sv tb_byte_fifo.sv

# Запуск. -onfinish stop: на $finish симуляция останавливается, а не закрывает окно.
#         -voptargs=+acc: сохраняем все сигналы видимыми для Wave
vsim -onfinish stop -voptargs=+acc work.tb_byte_fifo

if {![batch_mode]} {
    add wave -divider "Write side (from UART RX)"
    add wave /tb_byte_fifo/wr_valid /tb_byte_fifo/full /tb_byte_fifo/overflow
    add wave -radix hex /tb_byte_fifo/wr_data
    add wave -divider "Read side (to parser)"
    add wave /tb_byte_fifo/rd_valid /tb_byte_fifo/rd_ready
    add wave -radix hex /tb_byte_fifo/rd_data
    add wave -divider "FIFO inside"
    add wave -radix unsigned /tb_byte_fifo/dut/wr_ptr /tb_byte_fifo/dut/rd_ptr /tb_byte_fifo/dut/used
}

run -all

if {[batch_mode]} { quit -f } else { wave zoom full }
