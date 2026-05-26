#!/bin/bash
# sync_from_jetson.sh
# Pull ALL captured data from Jetson to local machine.
#
# Usage:  bash sync_from_jetson.sh
#
# Syncs:
#   Jetson: ~/Documents/Visual_Inspection_ws/evaluation/   ->  local: Evaluation_V_I_ws/eval_dataset/
#   Jetson: ~/Documents/Visual_Inspection_ws/captures/     ->  local: data/captures_from_jetson/
#   Jetson: ~/Documents/Visual_Inspection_ws/yolo_dataset/ ->  local: data/yolo_dataset/

JETSON_USER="rgen"
JETSON_IP="192.168.8.181"
JETSON_BASE="~/Documents/Visual_Inspection_ws"

LOCAL_BASE="$(cd "$(dirname "$0")/.." && pwd)"
LOCAL_EVAL="${LOCAL_BASE}/Evaluation_V_I_ws/eval_dataset"
LOCAL_CAPTURES="${LOCAL_BASE}/data/captures_from_jetson"
LOCAL_YOLO="${LOCAL_BASE}/data/yolo_dataset"

echo ""
echo "======================================================"
echo "  SYNC FROM JETSON -> LOCAL"
echo "======================================================"
echo "  Jetson : ${JETSON_USER}@${JETSON_IP}:${JETSON_BASE}"
echo "  Local  : ${LOCAL_BASE}"
echo ""

mkdir -p "${LOCAL_EVAL}"
mkdir -p "${LOCAL_CAPTURES}"
mkdir -p "${LOCAL_YOLO}"

# 1. Evaluation dataset
echo "[ 1/3 ]  Syncing evaluation/ ..."
rsync -avz --progress \
    "${JETSON_USER}@${JETSON_IP}:${JETSON_BASE}/evaluation/" \
    "${LOCAL_EVAL}/"
echo ""

# 2. Captures (all pipeline + MQTT captures)
echo "[ 2/3 ]  Syncing captures/ ..."
rsync -avz --progress \
    "${JETSON_USER}@${JETSON_IP}:${JETSON_BASE}/captures/" \
    "${LOCAL_CAPTURES}/"
echo ""

# 3. YOLO training dataset
echo "[ 3/3 ]  Syncing yolo_dataset/ ..."
rsync -avz --progress \
    "${JETSON_USER}@${JETSON_IP}:${JETSON_BASE}/yolo_dataset/" \
    "${LOCAL_YOLO}/"
echo ""

# Summary
echo "======================================================"
echo "  DONE"
echo ""
echo "  Eval dataset     : $(find "${LOCAL_EVAL}"     -name '*.jpg' 2>/dev/null | wc -l) images"
echo "  Captures         : $(find "${LOCAL_CAPTURES}" -name '*.jpg' 2>/dev/null | wc -l) images"
echo "  YOLO train data  : $(find "${LOCAL_YOLO}"     -name '*.jpg' 2>/dev/null | wc -l) images"
echo "======================================================"
echo ""
