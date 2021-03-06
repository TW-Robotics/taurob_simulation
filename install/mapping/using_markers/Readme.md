# Using Markers

This package is written for visualize nuclear radiation in rviz. Therefore recorded data from SSM1+ radiation sensor is used to estimate the radiaton for the whole map.


## gaussian .py
  - Read gcounts_neu.csv from path /home/workstation/catkin_ws/src/src/sensors/gcount/
  - Predict radiation values for whole are based on Gaussian Process
  - Save predicted values in gauss_out.csv

## basic_shapes.cpp

  - Read gauss_out.csv from path /home/workstation/catkin_ws/src/src/gcount/
  - Save all values from .csv file to variables of type std::vector<double>
  - Create marker array (cubic shape) with all values
  - Depending on value of radiation, color the cubic from green to red
  - Create legend for colorboar
  - Publish Markerarray

## Published Topics
* **`/visualization_marker_array`** ([visualization_msgs::MarkerArray])

## Configuration
  - Size of raster (in gaussian.py and basic_shapes.cpp). Default 0.1m. Has to be slightly higher in basic_shapes.cpp (default 0.12m)
  - Kernel used for Gaussian Process. Default RationalQuadratic Kernel. Can be changed in gaussian.py
 
