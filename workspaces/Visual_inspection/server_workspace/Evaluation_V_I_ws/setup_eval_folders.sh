#!/bin/bash
# setup_eval_folders.sh
# Run once on Jetson to create all evaluation dataset folders
# Usage: bash ~/Documents/Visual_Inspection_ws/Evaluation_V_I_ws/setup_eval_folders.sh

echo "Creating evaluation dataset folder structure..."

BASE=~/eval_dataset

# Reference images
mkdir -p $BASE/reference

# Angle evaluation
mkdir -p $BASE/angle_eval/horizontal/{0deg,15deg_L,15deg_R,30deg_L,30deg_R,45deg_L,45deg_R}
mkdir -p $BASE/angle_eval/vertical/{0deg,15deg_up,15deg_down,30deg_up,30deg_down}

# Distance evaluation
mkdir -p $BASE/distance_eval/{1m,2m,3m,4m}

# Occlusion evaluation
mkdir -p $BASE/occlusion/{0pct,25pct,50pct,75pct}

# Gauge accuracy ground truth
mkdir -p $BASE/gauge_accuracy

# VLM evaluation images
mkdir -p $BASE/vlm_eval/{fire_ext_pass,fire_ext_fail,exit_pass,exit_fail,cylinder_pass,cylinder_fail,door_pass,door_fail,unknown_various}

# IBVS convergence logs (auto-written by action server)
mkdir -p $BASE/ibvs_logs

# Multi-object scenes
mkdir -p $BASE/multi_object/{2_objects,3_objects}

# Create capture log CSV with header
LOG=$BASE/capture_log.csv
if [ ! -f "$LOG" ]; then
    echo "timestamp,folder,filename,object_type,distance_m,angle_deg,angle_direction,occlusion_pct,n_objects,ibvs_time_s,final_error_px,converged,ground_truth_value,notes" > $LOG
    echo "Created capture_log.csv with header"
else
    echo "capture_log.csv already exists — not overwriting"
fi

echo ""
echo "Done! Structure created at: $BASE"
echo ""
tree $BASE -d 2>/dev/null || find $BASE -type d | sort
