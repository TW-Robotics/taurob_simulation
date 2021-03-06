/**
 * @file BagOpener.cpp
 * @author Jan Korecky
 * @brief cpp file for class which can open bagfiles and publish the data
 * @version 1
 * @date 2020-06-30
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include "BagOpener.h"

/**
 * in the constructor the path to the folder is defined based on the input
 * afterwards the paths to the respective bags and the topicnames are defined
 * the bags are then opened
 */

BagOpener::BagOpener(std::string folder){
    ROS_INFO_STREAM("Starting Constructor!");
    std::string pathToPackage = ros::package::getPath("openpose_testing");
    std::string pathToBag = pathToPackage + "/bagfiles" + "/" + folder;

    bags.resize(5);
    pathNames.resize(5);
    topicNames.resize(5);
    
    pathNames[0] = pathToBag + "/rgb_img.bag";
    pathNames[1] = pathToBag + "/depth_img.bag";
    pathNames[2] = pathToBag + "/info.bag";
    pathNames[3] = pathToBag + "/pc.bag";
    pathNames[4] = pathToBag + "/tf.bag";

    topicNames[0] = "/rgb_imgs";
    topicNames[1] = "/depth_imgs";
    topicNames[2] = "/info";
    topicNames[3] = "/pc";
    topicNames[4] = "/_tf";


    for(int i=0; i<5; i++){
        bags[i].open(pathNames[i], rosbag::bagmode::Read);
    }
}

/**
 * in the destructor the bags are closed again
 */
BagOpener::~BagOpener(){
    for (int i=0; i<5; i++){
        bags[i].close();
    }
}

/**
 * the data from the bagfiles is saved in the respective class members
 */

void BagOpener::grabData(){
    rosbag::View rgb_view(bags[0], rosbag::TopicQuery(topicNames[0]));
    rosbag::View depth_view(bags[1], rosbag::TopicQuery(topicNames[1]));
    rosbag::View info_view(bags[2], rosbag::TopicQuery(topicNames[2]));
    rosbag::View pc_view(bags[3], rosbag::TopicQuery(topicNames[3]));
    rosbag::View tf_view(bags[4], rosbag::TopicQuery(topicNames[4]));
    BOOST_FOREACH(rosbag::MessageInstance const m, rgb_view)
    {
        rgb = m.instantiate<sensor_msgs::Image>();
    }
    BOOST_FOREACH(rosbag::MessageInstance const m, depth_view)
    {
        depth = m.instantiate<sensor_msgs::Image>();
    }
    BOOST_FOREACH(rosbag::MessageInstance const m, info_view)
    {
        info = m.instantiate<sensor_msgs::CameraInfo>();
    }
    BOOST_FOREACH(rosbag::MessageInstance const m, pc_view)
    {
        pc = m.instantiate<sensor_msgs::PointCloud2>();
    }
    BOOST_FOREACH(rosbag::MessageInstance const m, tf_view)
    {
        _tf = m.instantiate<tf2_msgs::TFMessage>();
    }
}