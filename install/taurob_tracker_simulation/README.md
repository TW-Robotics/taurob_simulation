# FHTW Taurob Tracker Simulation

This simulation is build on top of the [official Taurob Tracker ROS-Gazebo Simulation](https://github.com/taurob/taurob_tracker_simulation).



## How to start
1. build catkin_ws
`cd ~/catkin_ws/ && catkin_make`
2. Change directory
`roscd tauob_tracker_bringup/scripts`
3. Start script
`bash start_sim.sh -params`

    Supported params:
    ```
        [-s <int> (1-3) default: 1 ... set size of generated map, 1=small 3=large]
        [-o ... start outdoor environments]
        [-b ... simulation with obstacles]
        [-g ... start with gazebo GUI, on low power computers it is recommended not to set this param]
        [-G ... start with GPU support]"
    ```


## Useful commands
```
rosservice call /gazebo/reset_simulation    # resets the gazebo simulation to initial state
killall -9 gzserver                         # kill gazebo server
killall -9 gzclient                         # kill gazebo client
```
