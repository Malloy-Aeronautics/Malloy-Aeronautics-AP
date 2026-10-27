#!/usr/bin/env bash
# Local version of the CI checks, run before pushing.
#
#   Tools/scripts/ma_local_ci.sh          quick: lint, SITL build, unit tests, param docs   (~10 min, most of it unit tests)
#   Tools/scripts/ma_local_ci.sh --full   also both flight controller builds and the
#                                         speed limiter SITL autotests                (~30+ min)
#
# Run it from the repo root on Ubuntu (WSL). It stops at the first failure.

set -euo pipefail

FULL=0
[ "${1:-}" = "--full" ] && FULL=1

BOARDS="CubeOrange mRoControlZeroOEMH7"     # keep in step with ma_ci.yml
LINT_BASE="${MA_LINT_BASE:-origin/release/MA-4.3.0.17}"
LOG_DIR="${TMPDIR:-/tmp}/ma_local_ci"
mkdir -p "$LOG_DIR"

step() {
    local name="$1"; shift
    local log="$LOG_DIR/$(echo "$name" | tr ' /' '__').log"
    printf '%-40s' "$name"
    local start=$SECONDS
    if "$@" > "$log" 2>&1; then
        echo "ok ($((SECONDS - start))s)"
    else
        echo "FAILED, see $log"
        tail -20 "$log"
        exit 1
    fi
}

if [ ! -f waf ] || [ ! -d libraries ]; then
    echo "run this from the root of the repo"
    exit 1
fi

if [ "$(uname -s)" != "Linux" ]; then
    echo "local CI needs Ubuntu (WSL), skipping on $(uname -s)"
    exit 0
fi

python3 -c 'import sys; sys.exit(sys.version_info >= (3, 11))' || {
    echo "waf in this tree needs Python 3.10 or older, activate the right venv first"
    exit 1
}

step "lint changed lines"    python3 Tools/scripts/ma_lint.py --base "$LINT_BASE"

python3 -c 'import lxml' 2>/dev/null || {
    echo "param docs check needs lxml:  python3 -m pip install lxml"
    exit 1
}

step "param docs"            python3 Tools/autotest/param_metadata/param_parse.py --vehicle ArduCopter
step "configure sitl"        ./waf configure --board sitl
# recompile the C/C++ files this branch touches so their warnings end up in the build log
git diff --name-only "$(git merge-base "$LINT_BASE" HEAD)" -- '*.cpp' '*.c' | xargs -r touch
step "build copter sitl"     ./waf copter
step "compiler warnings"     python3 Tools/scripts/ma_lint.py --base "$LINT_BASE" --build-log "$LOG_DIR/build_copter_sitl.log"
step "unit tests"            ./waf check

if [ $FULL -eq 1 ]; then
    for b in $BOARDS; do
        step "build copter $b"   bash -c "./waf configure --board $b && ./waf copter"
    done
    # put SITL back so the autotests use it
    step "reconfigure sitl"      ./waf configure --board sitl
    step "speed limiter autotests" bash -c \
        "cd Tools/autotest && python3 autotest.py --no-clean \$(python3 -c \"
import re
s = open('arducopter.py').read()
print(' '.join('test.Copter.' + n for n in sorted(set(re.findall(r'def (SpeedLimit\w+)\(self', s)))))
\")"
fi

# param_parse writes its output files into the repo root, tidy them up
rm -f Parameters*.rst Parameters*.html Parameters*.md apm.pdef.xml apm.pdef.json 2>/dev/null || true

echo "all local checks passed"
