/**
 * @file ImageSaver.cpp
 * @author Jan Korecky
 * @brief cpp file for class which can save camera data
 * @version 1
 * @date 2020-06-30
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include "ImageSaver.h"

/**
 * constructor which takes the folder where the date should be saved to as input
 * a path to the folder is created within the "bagfiles" folder of the package 
 * the topic names are initialized
 */

ImageSaver::ImageSaver(std::string folderName){
    std::string pathToPackage = ros::package::getPath("openpose_testing");
    pathToFolder = pathToPackage + "/bagfiles" + "/" + folderName;

    const char* pathToFolder_create = pathToFolder.c_str();
    mkdir(pathToFolder_create, S_IRWXU);


    bags.resize(5);
    pathNames.resize(5);
    topicNames.resize(5);

    

    topicNames[0] = "/rgb_imgs";
    topicNames[1] = "/depth_imgs";
    topicNames[2] = "/info";
    topicNames[3] = "/pc";
    topicNames[4] = "/_tf";

    pictureCount = 0;

}


void ImageSaver::get_image( const sensor_msgs::Image::ConstPtr& msg ) { 
    image = *msg;
}

void ImageSaver::get_depth(const sensor_msgs::Image::ConstPtr& msg){
    depth = *msg;
}

void ImageSaver::get_info(const sensor_msgs::CameraInfo::ConstPtr& msg){
    camInfo = *msg;
}

void ImageSaver::get_pointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg){
    pointCloud = *msg;
}

void ImageSaver::get_tf(const tf2_msgs::TFMessage::ConstPtr& msg){
    _tf = *msg;
}

/**
 * saves the data in the desired folder as bagfiles on enter-press
 * create a folder for every picture that is taken
 * the folders are numbered starting with 1
 * this way multiple pictures can be taken in a single execution of the program
 * the data is then saved in the respective folder in 5 bagfiles
 */

void ImageSaver::savePicture(const openpose_testing::enterPress input){
    ROS_INFO_STREAM("Picture saved!");

    pictureCount += 1;

    pathToBags = pathToFolder + "/" + std::to_string(pictureCount);
    const char* pathToBags_create = pathToBags.c_str();
    mkdir(pathToBags_create, S_IRWXU);

    pathNames[0] = pathToBags + "/rgb_img.bag";
    pathNames[1] = pathToBags + "/depth_img.bag";
    pathNames[2] = pathToBags + "/info.bag";
    pathNames[3] = pathToBags + "/pc.bag";
    pathNames[4] = pathToBags + "/tf.bag";
    
    for (int i = 0; i<5; i++){
        bags[i].open(pathNames[i], rosbag::bagmode::Write);
        if(!bags[i].isOpen()){
            ROS_INFO_STREAM("FAILED TO OPEN!");
        }
        switch(i){
            case 0:
                bags[i].write(topicNames[i], ros::Time::now(), image);
                break;
            case 1:
                bags[i].write(topicNames[i], ros::Time::now(), depth);
                break;
            case 2:
                bags[i].write(topicNames[i], ros::Time::now(), camInfo);
                break;
            case 3:
                bags[i].write(topicNames[i], ros::Time::now(), pointCloud);
                break;
            case 4:
                bags[i].write(topicNames[i], ros::Time::now(), _tf);
                break;
        }
        
        bags[i].close();
    }
    
    
}