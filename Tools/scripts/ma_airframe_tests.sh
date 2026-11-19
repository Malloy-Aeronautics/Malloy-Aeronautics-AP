#!/usr/bin/env bash
# Flies the airframe check (test.Copter.MAAirframeFlight) on every MA airframe that has its SITL
# model: the T150 and T400, each empty and with payload. Runs them all and fails if any fails.
#
#   Tools/scripts/ma_airframe_tests.sh                 every airframe that's ready
#   Tools/scripts/ma_airframe_tests.sh T400_payload    just these
#
# Run from the repo root on Ubuntu, after a SITL build (./waf configure --board sitl && ./waf copter).

set -uo pipefail
cd "$(git rev-parse --show-toplevel)"

LOG_DIR="${TMPDIR:-/tmp}/ma_local_ci"
mkdir -p "$LOG_DIR"

python3 Tools/autotest/ma_airframes.py
names="${*:-$(python3 Tools/autotest/ma_airframes.py --ready)}"

failed=""
for name in $names; do
    log="$LOG_DIR/airframe_$name.log"
    printf '%-40s' "airframe $name"
    start=$SECONDS
    if (cd Tools/autotest && MA_AIRFRAME="$name" python3 autotest.py --no-clean test.Copter.MAAirframeFlight) > "$log" 2>&1; then
        echo "ok ($((SECONDS - start))s)  $(grep -o 'MA airframe result:.*' "$log" | tail -1 | cut -d: -f2-)"
    else
        echo "FAILED, see $log"
        grep -E "MA airframe|NotAchieved|Exception" "$log" | tail -5
        failed="$failed $name"
    fi
done

if [ -n "$failed" ]; then
    echo "airframe checks failed:$failed"
    exit 1
fi
