# taurob_explorer
Simple explorer node based on rrt
Currently working: 
- [x] Exploring map by utilizing [taurob_navigation](../taurob_navigation)
- [x] Terminating current goal if execution takes longer than timeout
- [x] Visualize current goal using marker
- [x] Visualize visitied_nodes
- [ ] Sample only from free space (using costmap or lidar)
- [ ] Replan based on previous failed goals
- [ ] Implement rqt_reconfigure

## Subscribed Topics
* **` /move_base_flex/exe_path/status/`** ([darknet_ros_msgs::BoundingBoxes])
* **` /move_base_flex/exe_path/feedback`** ([mbf_msgs::ExePathActionFeedback])


## Published topics
* Current goal: **`/move_base_simple/goal`** ([geometry_msgs::PoseStamped])
* Current goal visualisation: **`/visualization_marker`** ([visualization_msgs::Marker])                
* Visited nodes visualisation: **` /visualization_marker_array`** ([visualization_msgs::MarkerArray])


## Configuration
Configurations can be set via ROS Parameter Server (online) or on initialisation via [config.yaml](config/config.yaml)
```
map_frame: "map"          # leading "/" not allowed
robot_frame: "base_link"  # leading "/" not allowed
timeout: 20               # [s]
max_dist: 2               # [m] Maximum distance to sampe from
range_visited_node: 1     # [m] Circular range in which goals are rejected
rand_angle: 90            # [°]
max_iter: 500             # Max numbers of iterations
max_reject: 5             # Max number of goals to reject, for being to close to visited node, before executing anyways
show_marker: True         # True to visualize current goal on /visualize_marker topic
show_nodes: True         # True to visualize visited nodes
```