# Follow Human
Controlls the robot so that it follows a detected human.
Currently working: 
- [x] P controler on linear velocity (Linear Velocity is based on height of detected Bounding Box)
- [x] P controler on angular velocity
- [x] Obstacle detection based on LIDAR readings
- [X] Dynamic Reconfigure enabled
- [X] PID Controller for angular velocity 
- [ ] PID Controller for linear velocity 
- [ ] Fuse LIDAR readings with RGB Camera for better tracking of human
- [ ] Set initial pose for Human to determine which to follow (+ switch to OpenPose for Gesture recognition?)
- [X] Publish velocity

## Subscribed Topics
* **` /darknet_ros/bounding_boxes`** ([actionlib_msgs::GoalStatusArray])
* **` /image_topic`** ([sensor_msgs::Image])
* **` /scan_front`** ([sensor_msgs::LaserScan])
* **` /scan_rear`**  ([sensor_msgs::LaserScan])

## Published topics
* **`/cmd_vel`** ([geometry_msgs::Twist])

## Configuration
Configurations can be set via ROS Parameter Server (online) or on initialisation via [config.yaml](config/config.yaml)
```
setpoint_height: 400     [pixel] default: 400
min_probability: 0.5    # [percent]
image_topic: "/camera_rgb/image_raw"
min_range_lidar: 0.4    # [m] 
use_pid: false          # [bool] true currently not working
kp: 0.00038125          # [float] P Gain for angular velocity
kp_lin: 0.168125        # [float] P Gain, equals max linear velocity
```
