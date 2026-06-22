# Field day checklist

## The night before

- [ ] Charge all batteries: rover, phone, laptop
- [ ] Flash a fresh SD card with Ubuntu 24.04 + ROS Jazzy if not already done
- [ ] Run `bash tools/quick_test.sh` on the rover - confirm [PASS] on all checks
- [ ] Test `ros2 launch fusioncore_ros fusioncore.launch.py` with the field config - check `/fusion/odom` is publishing
- [ ] Confirm ZED-F9P is recognized: `ls /dev/ttyACM*` or `ls /dev/ttyUSB*`
- [ ] Test bag recording: `ros2 bag record -o test_bag /imu/data /odom/wheels /gnss/fix /gnss/status` for 30 seconds, then `ros2 bag info test_bag` - verify all topics have messages
- [ ] Pack: rover, laptop, USB-C cable, spare battery, chalk, tape measure, phone tripod/selfie stick, sunscreen

## At the field

### Course setup (do once, before any runs)

- [ ] Walk the full course on foot - identify: open-sky segment, building-wall multipath pass, underpass/canopy dropout, loop start/end mark
- [ ] Mark loop start/end with chalk on the ground (tape measure distance between marks - write it down)
- [ ] Check phone GNSS: open Google Maps or GPSTest app, confirm satellite count and accuracy at each segment
- [ ] Note time: aim to finish before 9am (sun angle, people traffic)

### Before each run

- [ ] Rover stationary, all sensors publishing: `ros2 topic hz /gnss/fix /imu/data /odom/wheels`
- [ ] Wait for GPS lock: at least 6 satellites, HDOP < 2 (check `/gnss/status` or `/fusion/debug/gnss_status`)
- [ ] Start recording: `ros2 bag record -o run_01 /imu/data /odom/wheels /gnss/fix /gnss/status /tf_static /fusion/odom /fusion/debug/gnss_status`
- [ ] Phone recording: start video before rover moves, keep rover in frame through full course
- [ ] Announce on camera: "Run 1, [date], [time], conditions: [sunny/cloudy/wind]"

### The run

- [ ] Drive at consistent speed (~0.5 m/s) - not faster, not slower per run
- [ ] At the loop end mark: stop, hold stationary 3 seconds, announce "loop closed" on camera
- [ ] Tape-measure the distance between rover and start chalk mark - announce it on camera

### After each run

- [ ] Stop bag recording
- [ ] `ros2 bag info run_01` - confirm all topics have expected message counts
- [ ] Quick sanity check: `ros2 bag play run_01 --pause` then unpause and watch `/fusion/odom` in RViz for 30 seconds
- [ ] Note any anomalies before the next run

### Minimum: 3 runs

- Run 1: baseline (good conditions, no interference)
- Run 2: identical path (repeatability check)
- Run 3: identical path (third data point for averaging)
- Run 4+ (optional): try a variation (faster speed, different time of day)

## Post-field (same day)

- [ ] Back up all bags to external drive or cloud immediately
- [ ] Verify bag sizes are reasonable (~500MB per 10min run at full rate)
- [ ] `ros2 bag info run_01 run_02 run_03` - screenshot and save
- [ ] Log the chalk-mark distance for each run (your loop-closure ground truth)
