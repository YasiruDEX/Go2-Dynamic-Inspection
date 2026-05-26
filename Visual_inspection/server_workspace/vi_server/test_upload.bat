@echo off
REM Test upload script wrapper

echo ========================================
echo Visual Inspection Server - Test Upload
echo ========================================
echo.

if not exist "venv\Scripts\activate.bat" (
    echo Error: Virtual environment not found!
    pause
    exit /b 1
)

call venv\Scripts\activate.bat

if "%1"=="" (
    echo Usage: test_upload.bat [image_path] [object_type]
    echo.
    echo Example:
    echo   test_upload.bat sample_images\sample_gauge.jpg gauge
    echo   test_upload.bat sample_images\sample_door.jpg door
    echo.
    echo Available object types: gauge, door, fire_extinguisher, unknown
    pause
    exit /b 1
)

set IMAGE_PATH=%1
set OBJECT_TYPE=%2

if "%OBJECT_TYPE%"=="" (
    set OBJECT_TYPE=gauge
)

echo Uploading: %IMAGE_PATH%
echo Object Type: %OBJECT_TYPE%
echo.

python scripts\upload_test.py --image "%IMAGE_PATH%" --object-type %OBJECT_TYPE%

pause
