@echo off
echo Starting Jupyter Notebook for YOLO26s training...
call yolo11_env\Scripts\activate.bat
jupyter notebook yolo26s_local_training.ipynb
pause
