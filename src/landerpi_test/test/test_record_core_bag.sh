#!/usr/bin/env bash
set -e

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT="$TEST_DIR/../scripts/record_core_bag.sh"

test -f "$SCRIPT"

grep -q "ros2 bag record" "$SCRIPT"
grep -q "/scan" "$SCRIPT"
grep -q "/odom" "$SCRIPT"

! grep -Eq "/tf|/tf_static|/imu|camera" "$SCRIPT"

echo "PASS: record script matches Week 2 E scope"
