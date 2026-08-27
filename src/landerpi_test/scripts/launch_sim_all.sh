#!/usr/bin/env bash

set -uo pipefail

echo "=========================================="
echo " LanderPi Simulation Acceptance Launcher"
echo " Role E - Integration Test"
echo "=========================================="

# --------------------------------------------------
# 1. ROS 环境检查
# --------------------------------------------------

if ! command -v ros2 >/dev/null 2>&1; then
    if [ -f /opt/ros/jazzy/setup.bash ]; then
        source /opt/ros/jazzy/setup.bash
    elif [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
        echo "[WARN] 当前使用 ROS 2 Humble"
        echo "[WARN] 项目正式基线为 ROS 2 Jazzy"
    else
        echo "[FAIL] 未找到 ROS 2 环境"
        exit 1
    fi
fi

echo "[PASS] ROS 2 environment: ${ROS_DISTRO:-unknown}"

# --------------------------------------------------
# 2. 工作空间环境检查
# --------------------------------------------------

if [ -f "./install/setup.bash" ]; then
    source ./install/setup.bash
fi

if ! ros2 pkg prefix landerpi_bringup >/dev/null 2>&1; then
    echo "[FAIL] 未找到 landerpi_bringup"
    echo "请先执行："
    echo "  colcon build --symlink-install"
    echo "  source install/setup.bash"
    exit 1
fi

echo "[PASS] landerpi_bringup package found"

# --------------------------------------------------
# 3. 仅执行接口检查
# --------------------------------------------------

check_topic()
{
    local topic=$1

    if ros2 topic list | grep -Fxq "$topic"; then
        echo "[PASS] Topic $topic"
        return 0
    else
        echo "[FAIL] Topic $topic"
        return 1
    fi
}

check_tf()
{
    local parent=$1
    local child=$2
    local temp_file="/tmp/landerpi_tf_${parent}_${child}.log"

    timeout 6s ros2 run tf2_ros tf2_echo "$parent" "$child" \
        > "$temp_file" 2>&1 || true

    if grep -q "Translation:" "$temp_file"; then
        echo "[PASS] TF $parent -> $child"
        return 0
    else
        echo "[FAIL] TF $parent -> $child"
        return 1
    fi
}

run_checks()
{
    echo
    echo "========== Interface Check =========="

    check_topic /scan
    check_topic /odom
    check_topic /cmd_vel

    check_tf odom base_link
    check_tf base_link laser_link

    echo
    echo "Optional runtime interfaces:"

    for topic in /map /robot_pose /goal_pose /plan /local_plan /actual_path
    do
        if ros2 topic list | grep -Fxq "$topic"; then
            echo "[PASS] Topic $topic"
        else
            echo "[WAIT] Topic $topic"
        fi
    done

    echo "====================================="
}

if [ "${1:-}" = "--check-only" ]; then
    run_checks
    exit 0
fi

# --------------------------------------------------
# 4. 启动团队统一仿真入口
# --------------------------------------------------

echo
echo "[INFO] Starting simulation.launch.py ..."

ros2 launch landerpi_bringup simulation.launch.py &
SIM_PID=$!

cleanup()
{
    echo
    echo "[INFO] Stopping simulation..."
    kill "$SIM_PID" 2>/dev/null || true
}

trap cleanup INT TERM EXIT

echo "[INFO] Waiting for simulation initialization..."
sleep 10

run_checks

echo
echo "[INFO] Simulation is running."
echo "[INFO] Press Ctrl+C to stop."

wait "$SIM_PID"
