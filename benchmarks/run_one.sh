#!/bin/bash
# Run a single NCLT benchmark sequence (full length).
# Usage: bash benchmarks/run_one.sh 2012-01-08

SEQ=${1:-2012-01-08}

export PATH="/home/manankharwar/.local/bin:$PATH"
source /opt/ros/jazzy/setup.bash
source /mnt/c/Users/Admin/ROS/ROS/fusioncore/install/setup.bash

REPO=/mnt/c/Users/Admin/ROS/ROS/fusioncore
RATE=3.0
# Full-length: duration_s=0 plays every row in the CSV.
# Wall-time cap: longest NCLT sequence is ~120 min -> 40 min at 3x + 10 min buffer = 3000s.
WALL_TIME=3000

DATA_DIR="$REPO/benchmarks/nclt/$SEQ"
BAG_DIR="$DATA_DIR/bag_full"
RESULTS_DIR="$DATA_DIR/results_full"
DATUM_YAML="/tmp/navsat_datum_${SEQ}.yaml"

echo "========================================"
echo "SEQ: $SEQ  ($(date))"
echo "========================================"

mkdir -p "$RESULTS_DIR"

# Generate fixed navsat datum from first valid RTK fix.
python3 << PYEOF
import csv, math, sys
rtk_path = "$DATA_DIR/raw files/gps_rtk.csv"
datum_yaml = "$DATUM_YAML"
lat_deg = lon_deg = None
with open(rtk_path) as f:
    for row in csv.reader(f):
        try:
            if int(row[1]) < 3:
                continue
            lr, lonr = float(row[3]), float(row[4])
        except (ValueError, IndexError):
            continue
        if not (math.isnan(lr) or math.isnan(lonr)):
            lat_deg, lon_deg = math.degrees(lr), math.degrees(lonr)
            break
if lat_deg is None:
    sys.exit("ERROR: no valid RTK fix in " + rtk_path)
with open(datum_yaml, 'w') as f:
    f.write(f"navsat_transform:\n  ros__parameters:\n    datum: [{lat_deg:.8f}, {lon_deg:.8f}, 0.0]\n    wait_for_datum: true\n")
print(f"navsat datum: {lat_deg:.6f}N {lon_deg:.6f}E -> {datum_yaml}")
PYEOF

pkill -9 -f "fusioncore_node|nclt_player|ekf_node|navsat_transform_node|ros2 bag record|ros2 launch|component_container" 2>/dev/null || true
sleep 8

rm -rf "$BAG_DIR"
rm -f "$DATA_DIR/fusioncore.tum" "$DATA_DIR/rl_ekf.tum" "$DATA_DIR/ground_truth.tum"

echo "Launching full-length benchmark (wall cap: ${WALL_TIME}s)..."
timeout --signal=SIGINT $WALL_TIME ros2 launch fusioncore_datasets nclt_benchmark.launch.py \
    data_dir:="$DATA_DIR/raw files" \
    output_bag:="$BAG_DIR" \
    playback_rate:=$RATE \
    duration_s:=0.0 \
    navsat_datum_yaml:="$DATUM_YAML" 2>&1 | tee "$RESULTS_DIR/launch.log" || true

sleep 8
pkill -9 -f "fusioncore_node|nclt_player|ekf_node|navsat_transform_node|ros2 bag record|component_container" 2>/dev/null || true
sleep 3

if [ ! -d "$BAG_DIR" ]; then
    echo "ERROR: bag not created for $SEQ"
    exit 1
fi

echo "Bag ready. Generating ground truth and evaluating..."

python3 "$REPO/tools/nclt_rtk_to_tum.py" \
    --rtk "$DATA_DIR/raw files/gps_rtk.csv" \
    --out "$DATA_DIR/ground_truth.tum"

python3 "$REPO/tools/odom_to_tum.py" \
    --bag "$BAG_DIR" --topic /fusion/odom \
    --out "$DATA_DIR/fusioncore.tum"

python3 "$REPO/tools/odom_to_tum.py" \
    --bag "$BAG_DIR" --topic /rl/odometry \
    --out "$DATA_DIR/rl_ekf.tum"

sort -n "$DATA_DIR/fusioncore.tum" > /tmp/fc_s.tum && cp /tmp/fc_s.tum "$DATA_DIR/fusioncore.tum"
sort -n "$DATA_DIR/rl_ekf.tum"    > /tmp/rl_s.tum && cp /tmp/rl_s.tum "$DATA_DIR/rl_ekf.tum"

python3 "$REPO/tools/evaluate.py" \
    --gt         "$DATA_DIR/ground_truth.tum" \
    --fusioncore "$DATA_DIR/fusioncore.tum" \
    --rl         "$DATA_DIR/rl_ekf.tum" \
    --sequence   "$SEQ" \
    --out_dir    "$RESULTS_DIR"

echo "DONE: $SEQ  ($(date))"
