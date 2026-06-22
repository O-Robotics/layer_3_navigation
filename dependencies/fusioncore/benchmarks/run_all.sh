#!/bin/bash
# Run all 12 NCLT benchmark sequences sequentially (full length each).
# Each sequence runs to completion at 3x speed.
# Longest sequence ~120 min -> ~40 min wall. Total: ~6-8 hours.
#
# Usage: bash benchmarks/run_all.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$SCRIPT_DIR/.." && pwd)"

SEQUENCES=(
    2012-01-08
    2012-02-04
    2012-03-31
    2012-05-11
    2012-06-15
    2012-08-20
    2012-09-28
    2012-10-28
    2012-11-04
    2012-12-01
    2013-02-23
    2013-04-05
)

for SEQ in "${SEQUENCES[@]}"; do
    bash "$REPO/benchmarks/run_one.sh" "$SEQ"
done

echo ""
echo "All 12 sequences complete. $(date)"
