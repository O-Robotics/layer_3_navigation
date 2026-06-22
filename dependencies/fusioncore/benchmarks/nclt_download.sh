#!/bin/bash
# Download NCLT sensor data for one or more sequences.
#
# Usage:
#   bash benchmarks/nclt_download.sh 2012-01-08
#   bash benchmarks/nclt_download.sh 2012-01-08 2012-02-04 2013-04-05
#   bash benchmarks/nclt_download.sh all          # all 12 sequences
#
# Downloads the five CSV files needed per sequence:
#   ms25.csv              IMU (100 Hz)
#   ms25_euler.csv        IMU Euler angles (100 Hz)
#   gps.csv               GPS fixes (5 Hz, raw including outliers)
#   gps_rtk.csv           RTK GPS ground truth (5 Hz, clean)
#   odometry_mu_100hz.csv Wheel encoder odometry (100 Hz)
#
# Files are placed under:
#   benchmarks/nclt/<date>/raw files/
#
# Dataset: http://robots.engin.umich.edu/nclt/
# Citation: Carlevaris-Bianco et al., "University of Michigan North Campus
#   Long-Term Vision and Lidar Dataset", IJRR 2016.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

NCLT_BASE="http://robots.engin.umich.edu/nclt"

ALL_SEQUENCES=(
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

# Files downloaded per sequence (compressed on server, extracted locally).
SENSOR_FILES=(
    ms25.csv
    ms25_euler.csv
    gps.csv
    gps_rtk.csv
    odometry_mu_100hz.csv
)

if [ $# -eq 0 ]; then
    echo "Usage: bash benchmarks/nclt_download.sh <date> [<date> ...]"
    echo "       bash benchmarks/nclt_download.sh all"
    echo ""
    echo "Available sequences:"
    for s in "${ALL_SEQUENCES[@]}"; do echo "  $s"; done
    exit 1
fi

if [ "$1" = "all" ]; then
    SEQUENCES=("${ALL_SEQUENCES[@]}")
else
    SEQUENCES=("$@")
fi

if ! command -v wget &>/dev/null; then
    echo "ERROR: wget not found. Install it: sudo apt install wget"
    exit 1
fi

for SEQ in "${SEQUENCES[@]}"; do
    DEST="$SCRIPT_DIR/nclt/$SEQ/raw files"
    mkdir -p "$DEST"
    echo ""
    echo "=== $SEQ ==="

    for FILE in "${SENSOR_FILES[@]}"; do
        OUT="$DEST/$FILE"
        if [ -f "$OUT" ]; then
            echo "  [skip] $FILE  (already exists)"
            continue
        fi

        # NCLT server provides gzip-compressed CSVs.
        URL="$NCLT_BASE/$SEQ/sensor_data/${FILE}.gz"
        echo "  Downloading $FILE ..."
        TMP="$DEST/${FILE}.gz"

        if wget -q --show-progress -O "$TMP" "$URL"; then
            gunzip -f "$TMP"
            echo "  [ok]   $FILE"
        else
            rm -f "$TMP"
            echo "  [FAIL] $FILE  — URL: $URL"
            echo "         Check http://robots.engin.umich.edu/nclt/ for the correct download structure."
        fi
    done

    echo "  Done: $DEST"
done

echo ""
echo "Download complete. Run a benchmark:"
echo "  bash benchmarks/run_one.sh 2012-01-08"
