@echo off
echo ========================================
echo FiaPhy VS Code Extension Builder
echo ========================================
echo.

REM Check if Node.js is installed
where node >nul 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] Node.js is not installed!
    echo.
    echo Please install Node.js from: https://nodejs.org/
    echo Download the LTS version and install it.
    pause
    exit /b 1
)

echo [OK] Node.js found
node --version

REM Check if npm is installed
where npm >nul 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] npm is not installed!
    pause
    exit /b 1
)

echo [OK] npm found
npm --version
echo.

REM Install dependencies
echo Step 1: Installing dependencies...
call npm install
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] Failed to install dependencies
    pause
    exit /b 1
)
echo [OK] Dependencies installed
echo.

REM Compile TypeScript
echo Step 2: Compiling TypeScript...
call npm run compile
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] Failed to compile
    pause
    exit /b 1
)
echo [OK] Compilation successful
echo.

REM Package extension
echo Step 3: Creating VSIX package...
call npm run package
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] Failed to create VSIX
    pause
    exit /b 1
)
echo.

echo ========================================
echo [SUCCESS] VSIX created successfully!
echo ========================================
echo.
echo File: fiaphy-language-support-1.0.0.vsix
echo.
echo To install:
echo   code --install-extension fiaphy-language-support-1.0.0.vsix
echo.
pause
