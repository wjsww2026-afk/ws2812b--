@echo off
chcp 65001 >nul
cd /d "%~dp0"

echo ============================================
echo   ESP32-C3 神光同步 - PC 端控制器
echo ============================================
echo.

python -c "import PySide6, serial" 2>nul
if errorlevel 1 (
    echo [1/2] 缺少依赖，正在安装 PySide6 与 pyserial...
    python -m pip install PySide6 pyserial
    if errorlevel 1 (
        echo 依赖安装失败，请手动执行: pip install PySide6 pyserial
        pause
        exit /b 1
    )
)

echo [1/2] 依赖检查通过
echo [2/2] 正在启动界面...
start "" pythonw argb_controller.py

echo 已启动。窗口标题：ESP32-C3 神光同步工作台
echo 音乐律动（含系统声音回环）需可选依赖: pip install soundcard numpy
echo 若窗口未出现，请改用下面命令查看报错：
echo     python argb_controller.py
echo.
timeout /t 3 >nul
