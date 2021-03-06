/**
 * @file BagOpener.h
 * @author Jan Korecky
 * @brief header file for class which can open bagfiles and publish the data
 * @version 1
 * @date 2020-06-30
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include "ros/ros.h"                
#include "sensor_msgs/Image.h" 
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include "sensor_msgs/CameraInfo.h" 
#include "sensor_msgs/PointCloud2.h" 
#include "tf2_msgs/TFMessage.h"


class BagOpener{
    public:
        sensor_msgs::Image::ConstPtr rgb;
        sensor_msgs::Image::ConstPtr depth;
        sensor_msgs::CameraInfo::ConstPtr info;
        sensor_msgs::PointCloud2::ConstPtr pc;
        tf2_msgs::TFMessage::ConstPtr _tf;

        /**
         * @brief Construct a new Bag Opener object
         * 
         * @param folder the folderpath from where to open the bagfiles 
         */
        BagOpener(std::string folder);

        /**
         * @brief Destroy the Bag Opener object and close the bags
         * 
         */
        ~BagOpener();

        /**
         * @brief get the data from the bagfiles
         * 
         */
        void grabData();

    private:
        std::vector <rosbag::Bag> bags;
        std::vector <std::string> pathNames;
        std::vector <std::string> topicNames;

};