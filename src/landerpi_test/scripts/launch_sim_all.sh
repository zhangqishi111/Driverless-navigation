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
    if [ -f /opt/ros/humble/setup.bash ]; then
        set +u
        source /opt/ros/humble/setup.bash
        set -u
    elif [ -f /opt/ros/jazzy/setup.bash ]; then
        set +u
        source /opt/ros/jazzy/setup.bash
        set -u
    else
        echo "[FAIL] 未找到 ROS 2 环境"
        exit 1
    fi
fi

if [ "${ROS_DISTRO:-unknown}" = "humble" ]; then
    echo "[PASS] ROS 2 environment: humble"
else
    echo "[WARN] 当前 ROS 2 environment: ${ROS_DISTRO:-unknown}"
    echo "[WARN] 团队正式基线为 ROS 2 Humble"
fi

# --------------------------------------------------
# 2. 工作空间环境检查
# --------------------------------------------------

if [ -f "./install/setup.bash" ]; then
    set +u
    source ./install/setup.bash
    set -u
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

PASS_COUNT=0
FAIL_COUNT=0
WAIT_COUNT=0

check_topic()
{
    local topic=$1

    if ros2 topic list | grep -Fxq "$topic"; then
        echo "[PASS] Topic $topic"
        PASS_COUNT=$((PASS_COUNT + 1))
        return 0
    else
        echo "[FAIL] Topic $topic"
        FAIL_COUNT=$((FAIL_COUNT + 1))
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
        PASS_COUNT=$((PASS_COUNT + 1))
        return 0
    else
        echo "[FAIL] TF $parent -> $child"
        FAIL_COUNT=$((FAIL_COUNT + 1))
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
            PASS_COUNT=$((PASS_COUNT + 1))
        else
            echo "[WAIT] Topic $topic"
            WAIT_COUNT=$((WAIT_COUNT + 1))
        fi
    done

    echo
    echo "Summary:"
    echo "  PASS: $PASS_COUNT"
    echo "  FAIL: $FAIL_COUNT"
    echo "  WAIT: $WAIT_COUNT"
    echo "====================================="

    if [ "$FAIL_COUNT" -gt 0 ]; then
        echo "[RESULT] FAIL"
        return 1
    else
        echo "[RESULT] PASS"
        return 0
    fi
}

if [ "${1:-}" = "--check-only" ]; then
    run_checks
    exit $?
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
