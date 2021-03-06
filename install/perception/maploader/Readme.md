# Load Map Tile
Downloads and publishes a map from ArcGIS. If available, displays the robot's orientation on this map. Test of robot_pose_ekf package to improve localisation accuracy.  
Currently working:
- [x] Download satellite image or street map
- [x] Display robot's orientation using odometry
- [ ] Improve localisation using robot_pose_ekf 
- [ ] Optional overlay of satellite image or street map in rviz

## Subscribed Topics
client:   
    depends on ROS parameters: gps_input, odom_input  
service server:  
    * **` /angle_correction`** ([std_msgs.msg::Int16])  

## Published Topics
* **`/gps_map`** ([sensor_msgs.msg::Image])

## Configuration
Configuration can be set via ROS Parameter Server
```
res_x: 1920             [pixel] Horizontal map resolution
res_y: 1920             [pixel] Vertical map resolution
delta: 0.001            "Zoom level". Value added to and subtracted from GPS coordinates to get 4 corners of map
filepath: Images        Path to folder where images will be downloaded and edited. This folder will contain the two folders "loaded" and "edited"
maptype: aerial         "aerial" or "streetmap". Type of map that will be downloaded. Based on this parameter, the URL to the corresponding ArcGIS mapserver will be selected.
delete: True            [bool] Bool value whether downloaded and edited images should be deleted if they are outdated
use_ekf: False          [bool] Bool value whether the service to draw the robot's orientation on the map should use odometry data or the output from robot_pose_ekf
offline: False          [bool] Bool value. If True, no new images will be downloaded, instead the most recent map will be edited and published
gps_input: fix          Topic that client subscribes to for GPS data
odom_input: odom        Topic that client subscribes to for orientation
angle_calibration: 0    Orientation of the robot at startup in degrees (0/360 = East, 90 = North, 180 = West, 270 = South). Calibration can be performed while the robot is operating by publishing the angle on the topic 'angle_correction'
```
