#!/usr/bin/env bash
set -e

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT="$TEST_DIR/../scripts/replay_core_bag.sh"

test -f "$SCRIPT"

grep -q "ros2 bag play" "$SCRIPT"
grep -q "Usage:" "$SCRIPT"

set +e
"$SCRIPT" >/dev/null 2>&1
EXIT_CODE=$?
set -e

test "$EXIT_CODE" -ne 0

echo "PASS: replay script requires a bag and uses ros2 bag play"
