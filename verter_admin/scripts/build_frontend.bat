@echo off
REM Сборка React фронтенда для Verter Robot Web UI (Windows)

setlocal

set "SCRIPT_DIR=%~dp0"
set "FRONTEND_DIR=%SCRIPT_DIR%src\verter_admin\web\frontend"

echo Building Verter Robot Web UI...
echo Frontend directory: %FRONTEND_DIR%

REM Проверяем наличие npm
where npm >nul 2>nul
if %ERRORLEVEL% neq 0 (
    echo Error: npm is not installed
    exit /b 1
)

REM Переходим в директорию фронтенда
cd /d "%FRONTEND_DIR%"

REM Устанавливаем зависимости если нужно
if not exist "node_modules" (
    echo Installing dependencies...
    call npm install
    if %ERRORLEVEL% neq 0 exit /b %ERRORLEVEL%
)

REM Собираем проект
echo Building...
call npm run build
if %ERRORLEVEL% neq 0 exit /b %ERRORLEVEL%

REM Проверяем результат
if exist "dist" (
    echo Build successful! Files are in: %FRONTEND_DIR%\dist
    echo.
    echo To install the package:
    echo   cd %SCRIPT_DIR%
    echo   colcon build --packages-select verter_admin --symlink-install
) else (
    echo Error: Build failed, dist directory not found
    exit /b 1
)

endlocal
