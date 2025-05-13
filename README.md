## Dependencies
This project uses pillow for image processing.  Install by running
```
pip install pillow
```

## Running
```
colcon build
source install/setup/bash
ros2 launch hw5 person_detection.launch.py
```
Note that this doesn't start the rosbag, that one's up to you.

### Calling the Snapshot Service
To capture the current scan and use it for future filtering, run
```
ros2 service call scan_snapshot hw5_msgs/srv/ScanSnapshot
```

### Changing filtering distance
```
ros2 param set /laser_filter scan_dist 0.5
```