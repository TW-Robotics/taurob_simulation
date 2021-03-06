/**
 * @file ImageSaver.h
 * @author Jan Korecky
 * @brief header file for class which can save camera data
 * @version 1
 * @date 2020-06-30
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include "ros/ros.h"
#include "sensor_msgs/Image.h" 
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/PointCloud2.h" 
#include "openpose_testing/enterPress.h"
#include <rosbag/bag.h>
#include <ros/package.h>
#include "tf2_msgs/TFMessage.h"
#include <sys/stat.h>
#include <sys/types.h>

class ImageSaver{

    public:
        sensor_msgs::Image image;

        /**
         * @brief Construct a new Image Saver object
         * 
         * @param folderName bagfiles will be saved here - new folder will be created if it doesn't exist yet
         */
        ImageSaver(std::string folderName);

        /**
         * @brief callback-function to check if user presses enter
         * 
         * @param input the enterPress message
         */
        void savePicture(const openpose_testing::enterPress input);

        /**
         * @brief callback which saves the rgb image
         * 
         * @param msg pointer to rgb image published by the camera
         */
        void get_image(const sensor_msgs::Image::ConstPtr& msg);

        /**
         * @brief callback which saves the depth image
         * 
         * @param msg pointer to depth image published by the camera
         */
        void get_depth(const sensor_msgs::Image::ConstPtr& msg);

        /**
         * @brief callback which saves the camera info
         * 
         * @param msg pointer to camera info published by the camera
         */
        void get_info(const sensor_msgs::CameraInfo::ConstPtr& msg);

        /**
         * @brief callback which saves the point cloud data
         * 
         * @param msg pointer to point cloud data published by the camera
         */
        void get_pointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg);

        /**
         * @brief callback which saves the tf data
         * 
         * @param msg pointer to tf data
         */
        void get_tf(const tf2_msgs::TFMessage::ConstPtr& msg);

    private:
        sensor_msgs::Image depth;
        sensor_msgs::CameraInfo camInfo;
        sensor_msgs::PointCloud2 pointCloud;
        tf2_msgs::TFMessage _tf;
        std::string pathToFolder;
        std::string pathToBags;
        std::vector <rosbag::Bag> bags;
        std::vector <std::string> pathNames;
        std::vector <std::string> topicNames;
        int pictureCount;
};