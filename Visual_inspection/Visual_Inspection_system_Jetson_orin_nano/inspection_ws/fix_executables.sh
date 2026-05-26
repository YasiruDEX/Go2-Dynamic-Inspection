#!/bin/bash
# fix_executables.sh
# Run after colcon build to create executable launchers
# Usage: bash fix_executables.sh
# 
# Required because setuptools>=60 changes how console_scripts
# are installed in ament_python packages

set -e
INSTALL_DIR="$HOME/Documents/Visual_Inspection_ws/inspection_ws/install"
VENV="$HOME/Documents/Visual_Inspection_ws/venv/lib/python3.10/site-packages"
SRC="$HOME/Documents/Visual_Inspection_ws/inspection_ws/visual_inspection_ros"

echo "🔧 Creating ROS2 executables..."

mkdir -p "$INSTALL_DIR/visual_inspection_ros/lib/visual_inspection_ros/"

# ── camera_node ──────────────────────────────────────────────────────────────
cat > "$INSTALL_DIR/visual_inspection_ros/lib/visual_inspection_ros/camera_node" << SCRIPT
#!/usr/bin/env python3
import sys
sys.path.insert(0, '$SRC')
sys.path.insert(0, '$VENV')
from visual_inspection_ros.camera_node import main
main()
SCRIPT
chmod +x "$INSTALL_DIR/visual_inspection_ros/lib/visual_inspection_ros/camera_node"
echo "  ✅ camera_node"

# ── servo_node (add when file exists) ────────────────────────────────────────
# cat > "$INSTALL_DIR/visual_inspection_ros/lib/visual_inspection_ros/servo_node" << SCRIPT
# #!/usr/bin/env python3
# import sys
# sys.path.insert(0, '$SRC')
# sys.path.insert(0, '$VENV')
# from visual_inspection_ros.servo_node import main
# main()
# SCRIPT
# chmod +x "$INSTALL_DIR/visual_inspection_ros/lib/visual_inspection_ros/servo_node"

# ── ibvs_action_server (add when file exists) ─────────────────────────────────
# cat > "$INSTALL_DIR/visual_inspection_ros/lib/visual_inspection_ros/ibvs_action_server" << SCRIPT
# #!/usr/bin/env python3
# import sys
# sys.path.insert(0, '$SRC')
# sys.path.insert(0, '$VENV')
# from visual_inspection_ros.ibvs_action_server import main
# main()
# SCRIPT
# chmod +x "$INSTALL_DIR/visual_inspection_ros/lib/visual_inspection_ros/ibvs_action_server"

echo ""
echo "✅ Done! Run with:"
echo "   ros2 run visual_inspection_ros camera_node"
