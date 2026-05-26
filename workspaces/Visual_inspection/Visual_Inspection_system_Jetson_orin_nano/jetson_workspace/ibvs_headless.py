#!/usr/bin/env python3
"""
HEADLESS mode — no display window, FPS printed to terminal every second.
Use this for normal operation / production.
"""
import sys
import os

# Force headless mode
sys.argv = [__file__, '--headless']

# Run the pipeline
sys.path.insert(0, os.path.dirname(__file__))
import ibvs_pipeline
ibvs_pipeline.main()
