#!/usr/bin/env bash
set -e

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <bag_directory>"
    exit 1
fi

BAG_DIR="$1"

if [ ! -d "$BAG_DIR" ]; then
    echo "[ERROR] Bag directory not found: $BAG_DIR"
    exit 2
fi

echo "Replaying Week 2 E rosbag:"
echo "  $BAG_DIR"
echo
echo "Expected recorded topics:"
echo "  /scan"
echo "  /odom"
echo
echo "Press Ctrl+C to stop playback."

ros2 bag play "$BAG_DIR"
