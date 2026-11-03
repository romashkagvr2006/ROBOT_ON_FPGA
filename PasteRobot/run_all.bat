@echo off
rem ===========================================================================
rem  run_all.bat - run ALL PasteRobot tests one after another (Questa, batch)
rem ---------------------------------------------------------------------------
rem  PowerShell:  cd C:\FPGA_PROJECT\PasteRobot
rem               .\run_all.bat
rem  Each test writes its full output to <folder>\sim.log
rem  Close the Questa window first: the free license allows one session only.
rem ===========================================================================
setlocal enabledelayedexpansion
set "ROOT=%~dp0"
set FAILED=0

echo.
echo  Running PasteRobot tests ...
echo  -------------------------------------------------

for %%T in (01_uart_rx:tb_uart_rx
            02_byte_fifo:tb_byte_fifo
            03_cmd_parser:tb_cmd_parser
            04_udiv:tb_udiv
            05_motion_ctrl:tb_motion_ctrl
            06_debounce:tb_debounce
            07_reply_gen:tb_reply_gen
            08_uart_tx:tb_uart_tx
            09_robot_top:tb_robot_top) do (
    for /f "tokens=1,2 delims=:" %%A in ("%%T") do (
        pushd "%ROOT%%%A"
        vsim -batch -do run_%%B.do > sim.log 2>&1
        findstr /c:"TEST PASSED" sim.log >nul
        if !errorlevel! equ 0 (
            echo   PASSED   %%A
        ) else (
            echo   FAILED   %%A   - open %%A\sim.log
            set FAILED=1
        )
        popd
    )
)

echo  -------------------------------------------------
if %FAILED% equ 0 (
    echo  ALL TESTS PASSED
) else (
    echo  SOME TESTS FAILED
)
echo.
endlocal & exit /b %FAILED%
