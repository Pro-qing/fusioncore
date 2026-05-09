# Oxford RobotCar Benchmark: 2014-05-06-12-54-54

Conditions: overcast, sunny intervals. Urban Oxford, ~10 km route.
Ground truth: NovAtel SPAN-CPT RTK GPS/INS (~10 cm accuracy).

Results pending. To reproduce once dataset access is granted:

```bash
# 1. Extract ground truth from INS
python3 tools/robotcar_ins_to_tum.py \
  --ins /path/to/robotcar/2014-05-06-12-54-54/gps/ins.csv \
  --out benchmarks/robotcar/2014-05-06-12-54-54/ground_truth.tum

# 2. Run the benchmark
source install/setup.bash
ros2 launch fusioncore_datasets robotcar_benchmark.launch.py \
  data_dir:=/path/to/robotcar/2014-05-06-12-54-54 \
  output_bag:=./benchmarks/robotcar/2014-05-06-12-54-54/bag \
  playback_rate:=3.0 \
  duration_s:=900.0

# 3. Extract trajectories
python3 tools/odom_to_tum.py \
  --bag   benchmarks/robotcar/2014-05-06-12-54-54/bag \
  --topic /fusion/odom \
  --out   benchmarks/robotcar/2014-05-06-12-54-54/fusioncore.tum

python3 tools/odom_to_tum.py \
  --bag   benchmarks/robotcar/2014-05-06-12-54-54/bag \
  --topic /rl/odometry \
  --out   benchmarks/robotcar/2014-05-06-12-54-54/rl_ekf.tum

# 4. Evaluate
python3 tools/evaluate.py \
  --gt         benchmarks/robotcar/2014-05-06-12-54-54/ground_truth.tum \
  --fusioncore benchmarks/robotcar/2014-05-06-12-54-54/fusioncore.tum \
  --rl         benchmarks/robotcar/2014-05-06-12-54-54/rl_ekf.tum \
  --sequence   2014-05-06-12-54-54 \
  --out_dir    benchmarks/robotcar/2014-05-06-12-54-54/results
```
