@echo off
REM Quick start script for Visual Inspection Server

echo ========================================
echo Visual Inspection Server - Quick Start
echo ========================================
echo.

REM Check if venv exists
if not exist "venv\Scripts\activate.bat" (
    echo Error: Virtual environment not found!
    echo Please run: python -m venv venv
    echo Then run: venv\Scripts\activate
    echo Then run: pip install -e .
    pause
    exit /b 1
)

REM Check if .env exists
if not exist ".env" (
    echo Creating .env from .env.example...
    copy .env.example .env
)

echo Starting server on http://0.0.0.0:8000
echo.
echo Press Ctrl+C to stop the server
echo.
echo ========================================
echo.

REM Activate venv and run server
call venv\Scripts\activate.bat
uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload
