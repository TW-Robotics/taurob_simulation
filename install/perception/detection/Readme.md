# detection

Package for visualize humans and signs in rviz which have been found via YOLO.   
Needs yolo to be trained only on persons and signs.  
Visualizes detected objects in rviz using tafel.stl and human_blender.stl located on the operator laptop in /operator/ ROS pkg.   
Requires :
  - [tesseract](https://github.com/tesseract-ocr/tesseract) Version 4.X
  - yaml-cpp
 ## calc_pose.cpp
  - Check if found object is sign or human
  - Check if calculated probabilty of objects is high enough
  - Check if there is no other previous detected object nearby
  - Calculate object position in map with raytracing
  - Publish markers (write to csv and pubolish to rviz) to visualize objects in map
  - Using Tesseract for character recognition of the signs
  - Cropping images to improve tesseract output


## Subscribed Topics
* **`/darknet_ros/bounding_boxes`** ([calc_pose::callback_yolo])
* **`/octomap_binary`** ([calc_pose::callback_octomap])
* **`/camera_rgb/image_raw`** ([calc_pose::imageCallback])

## Published Topics
* **`/visualization_marker_sign`** ([visualization_msgs::Marker])
* **`/visualization_marker_arrow`** ([visualization_msgs::Marker])
* **`/visualization_marker_human`** ([visualization_msgs::Marker])
* **`/detected_pose`** ([geometry_msgs::Pose])

## Configuration
Default Configs are saved in detection/config in cam_config.yaml and detection.yaml
  - cam_config.yaml: Definiton of camera parameters such as resolution, pixel size, sensor dimensions,...
  - detection.yaml: Defintion of map and camera frame. And values for oszilating the ray for raycasting 



