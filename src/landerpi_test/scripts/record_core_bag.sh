#!/usr/bin/env bash
set -e

BAG_NAME="${1:-week2_scan_odom_$(date +%Y%m%d_%H%M%S)}"

echo "Recording Week 2 E core topics:"
echo "  /scan"
echo "  /odom"
echo
echo "Bag name: $BAG_NAME"
echo "Press Ctrl+C to stop recording."

ros2 bag record -o "$BAG_NAME" /scan /odom
