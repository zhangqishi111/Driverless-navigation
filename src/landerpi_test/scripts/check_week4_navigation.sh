#!/usr/bin/env bash

set -u

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"

PASS=0
FAIL=0

pass() {
    echo "[PASS] $1"
    PASS=$((PASS + 1))
}

fail() {
    echo "[FAIL] $1"
    FAIL=$((FAIL + 1))
}

check_file() {
    if [ -f "$1" ]; then
        pass "File exists: $1"
    else
        fail "Missing file: $1"
    fi
}

echo "=========================================="
echo " Week 4 Navigation Offline Acceptance"
echo "=========================================="
echo

echo "===== 1. Required files ====="

check_file "$ROOT_DIR/src/landerpi_bringup/launch/real_system.launch.py"
check_file "$ROOT_DIR/src/landerpi_navigation/launch/navigation_real.launch.py"
check_file "$ROOT_DIR/src/landerpi_navigation/launch/navigation.launch.py"
check_file "$ROOT_DIR/src/landerpi_navigation/scripts/localization_safety_guard.py"
check_file "$ROOT_DIR/src/landerpi_navigation/config/nav2_params.yaml"
check_file "$ROOT_DIR/src/landerpi_bringup/maps/real_sandbox_clean.pgm"
check_file "$ROOT_DIR/src/landerpi_bringup/maps/real_sandbox_clean.yaml"

echo
echo "===== 2. Mapping / navigation mode guard ====="

REAL_SYSTEM="$ROOT_DIR/src/landerpi_bringup/launch/real_system.launch.py"

if grep -q "mode not in ('mapping', 'navigation')" "$REAL_SYSTEM" \
   && grep -q "mode == 'mapping'" "$REAL_SYSTEM" \
   && grep -q "navigation_real.launch.py" "$REAL_SYSTEM"; then
    pass "real_system.launch.py contains mapping/navigation mode separation"
else
    fail "mapping/navigation mode separation not found"
fi

if grep -q "mapping.launch.py" "$REAL_SYSTEM" \
   && grep -q "navigation_real.launch.py" "$REAL_SYSTEM"; then
    pass "mapping and navigation launch paths are separated"
else
    fail "mapping/navigation launch includes incomplete"
fi

echo
echo "===== 3. Safety Guard wiring ====="

if grep -q "localization_safety_guard.py" "$REAL_SYSTEM"; then
    pass "Localization Safety Guard is started in real system"
else
    fail "Localization Safety Guard launch entry missing"
fi

if grep -q "base_input_topic = '/cmd_vel_safe'" "$REAL_SYSTEM"; then
    pass "Navigation mode routes base input to /cmd_vel_safe"
else
    fail "Navigation base input is not /cmd_vel_safe"
fi

if grep -q "('/cmd_vel', '/cmd_vel_safe')" "$REAL_SYSTEM"; then
    pass "Safety Guard output remapped to /cmd_vel_safe"
else
    fail "Safety Guard output remap missing"
fi

echo
echo "===== 4. Guard parameters ====="

GUARD="$ROOT_DIR/src/landerpi_navigation/scripts/localization_safety_guard.py"

if grep -q "self.odom_translation_threshold = 0.35" "$GUARD"; then
    pass "Odom translation threshold = 0.35 m"
else
    fail "Unexpected odom translation threshold"
fi

if grep -q "self.odom_rotation_threshold = 0.30" "$GUARD"; then
    pass "Odom rotation threshold = 0.30 rad"
else
    fail "Unexpected odom rotation threshold"
fi

if grep -q 'required_good_updates", 3' "$GUARD"; then
    pass "required_good_updates = 3"
else
    fail "required_good_updates default is not 3"
fi

echo
echo "===== 5. Map configuration ====="

MAP_YAML="$ROOT_DIR/src/landerpi_bringup/maps/real_sandbox_clean.yaml"

if grep -q "image: real_sandbox_clean.pgm" "$MAP_YAML"; then
    pass "Map YAML points to real_sandbox_clean.pgm"
else
    fail "Map YAML image field is incorrect"
fi

if grep -q "resolution: 0.05" "$MAP_YAML"; then
    pass "Map resolution = 0.05"
else
    fail "Unexpected map resolution"
fi

if grep -q "origin: \[-6.7, -8.12, 0\]" "$MAP_YAML"; then
    pass "Map origin unchanged"
else
    fail "Unexpected map origin"
fi

echo
echo "===== 6. Nav2 configuration ====="

NAV_PARAMS="$ROOT_DIR/src/landerpi_navigation/config/nav2_params.yaml"

if grep -q "robot_model_type" "$NAV_PARAMS"; then
    pass "AMCL robot model configured"
else
    fail "AMCL robot model missing"
fi

if grep -q "update_min_d" "$NAV_PARAMS"; then
    pass "AMCL update_min_d configured"
else
    fail "AMCL update_min_d missing"
fi

if grep -q "update_min_a" "$NAV_PARAMS"; then
    pass "AMCL update_min_a configured"
else
    fail "AMCL update_min_a missing"
fi

echo
echo "===== 7. Python tests ====="

cd "$ROOT_DIR"

if python3 -m pytest src/landerpi_navigation/test -q; then
    pass "Navigation automated tests passed"
else
    fail "Navigation automated tests failed"
fi

echo
echo "===== 8. Git whitespace check ====="

if git diff --check; then
    pass "git diff --check passed"
else
    fail "git diff --check failed"
fi

echo
echo "=========================================="
echo " Result: PASS=$PASS  FAIL=$FAIL"
echo "=========================================="

if [ "$FAIL" -eq 0 ]; then
    echo "Week 4 offline acceptance PASSED."
    exit 0
else
    echo "Week 4 offline acceptance FAILED."
    exit 1
fi
