/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 * 
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "ros/ros.h"
#include "ros/package.h"
#include <ros/console.h>
#include "detection/calc_pose.hpp"
#include <yaml-cpp/yaml.h>



int main(int argc, char** argv){
    ros::init(argc, argv, "detection");
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) 
    {
        ros::console::notifyLoggerLevelsChanged();
    }
    

    std::string path = ros::package::getPath("detection");
    std::string path_cam=path;
    std::string path_detection=path;
    path_cam.append("/config/cam_config.yaml");
    path_detection.append("/config/detection.yaml");
    YAML::Node config_cam = YAML::LoadFile(path_cam);
    YAML::Node config_detection = YAML::LoadFile(path_detection);
    ROS_INFO("Reading yaml configuratoin file");
    calc_pose calculator(config_cam, config_detection);


    while (ros::ok()){
        ros::spinOnce();
    }
    return 0;
}
