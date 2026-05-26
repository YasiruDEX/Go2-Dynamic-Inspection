#!/bin/bash
# ============================================================
# Run IBVS Pipeline — handles cleanup automatically
# Usage:
#   ./run.sh          → headed mode (window visible via VNC)
#   ./run.sh headless → headless mode (FPS in terminal only)
# ============================================================

cd "$(dirname "$0")"
source venv/bin/activate 2>/dev/null || true

# Kill any stuck pipeline processes (frees cameras + serial port)
echo "🧹 Cleaning up old processes..."
pkill -f "ibvs_pipeline.py" 2>/dev/null
sleep 1.5

# Set display for headed mode (VNC)
export DISPLAY=:1

# Set max Jetson performance
sudo nvpmodel -m 0 2>/dev/null
sudo jetson_clocks 2>/dev/null

# Grant serial port access
sudo chmod 666 /dev/arduino   2>/dev/null
sudo chmod 666 /dev/ttyACM0  2>/dev/null

echo ""
if [ "$1" == "headless" ]; then
    echo "▶ Starting HEADLESS mode..."
    python3 ibvs_pipeline.py --headless
else
    echo "▶ Starting HEADED mode (window via VNC)..."
    python3 ibvs_pipeline.py
fi
