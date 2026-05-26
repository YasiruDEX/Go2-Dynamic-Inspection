@echo off
echo ============================================================
echo  YOLO26s Local Training - Environment Setup
echo ============================================================

REM ── Step 0: Check GPU info ──────────────────────────────────
echo.
echo [0/4] Checking GPU information...
nvidia-smi

REM ── Step 1: Create virtual environment ──────────────────────
echo.
echo [1/4] Creating virtual environment 'yolo11_env'...
set PYTHON_PATH="C:\Program Files\PsychoPy\python.exe"
%PYTHON_PATH% -m venv yolo11_env
if errorlevel 1 (
    echo ERROR: Could not create virtual environment.
    echo Make sure PsychoPy is installed at C:\Program Files\PsychoPy\
    pause
    exit /b 1
)

REM ── Step 2: Activate ────────────────────────────────────────
echo.
echo [2/4] Activating environment...
call yolo11_env\Scripts\activate.bat

REM ── Step 3: Upgrade pip ──────────────────────────────────────
echo.
echo [3/4] Upgrading pip...
python -m pip install --upgrade pip

REM ── Step 4: Install packages ─────────────────────────────────
echo.
echo [4/4] Installing packages (this may take a few minutes)...
pip install "ultralytics>=8.4.0"
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu128
pip install notebook ipykernel
pip install matplotlib seaborn pandas numpy pillow scikit-learn
pip install ipywidgets tqdm pyyaml psutil packaging

REM ── Register the kernel for Jupyter ─────────────────────────
echo.
echo Registering Jupyter kernel 'yolo11_env'...
python -m ipykernel install --user --name yolo11_env --display-name "Python (yolo11_env)"

echo.
echo ============================================================
echo  Setup COMPLETE! (YOLO26s environment ready)
echo  Next: run  launch_notebook.bat  to open Jupyter
echo ============================================================
pause
