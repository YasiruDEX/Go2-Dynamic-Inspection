#!/usr/bin/env python3
"""
HEADED mode — shows live camera window via VNC/display + FPS in terminal.
Use this for debugging and tuning.
"""
import sys
import os

# Force headed mode (no --headless flag)
sys.argv = [__file__]

# Run the pipeline
sys.path.insert(0, os.path.dirname(__file__))
import ibvs_pipeline
ibvs_pipeline.main()
