# =============================================================================
# run_tb_debounce.do — компиляция и запуск: теста антидребезга (debounce)
# -----------------------------------------------------------------------------
# Консоль (PowerShell), из ЭТОЙ папки:
#     vsim -batch -do run_tb_debounce.do
# Окно Questa: File > Change Directory... (эта папка), затем в строке Transcript:
#     do run_tb_debounce.do
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
vlog -sv debounce.sv tb_debounce.sv

# Запуск. -onfinish stop: на $finish симуляция останавливается, а не закрывает окно.
#         -voptargs=+acc: сохраняем все сигналы видимыми для Wave
vsim -onfinish stop -voptargs=+acc work.tb_debounce

if {![batch_mode]} {
    add wave /tb_debounce/raw /tb_debounce/dut/sync_1 /tb_debounce/clean
    add wave -radix unsigned /tb_debounce/dut/stable_cnt
}

run -all

if {[batch_mode]} { quit -f } else { wave zoom full }
