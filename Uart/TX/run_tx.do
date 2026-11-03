cd C:/FPGA_PROJECT/Uart
catch {quit -sim}
if {[file exists work]} {vdel -all -lib work}
vlib work
vlog -sv uart_tx.sv tb_uart_tx.sv
vsim -onfinish stop -voptargs=+acc work.tb_uart_tx
add wave -r /tb_uart_tx/*
run -all
wave zoom full