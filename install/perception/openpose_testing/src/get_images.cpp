/**
 * @file get_images.cpp
 * @author Jan Korecky
 * @brief main-function for saving the camera data
 * @version 1
 * @date 2020-06-30
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include "ImageSaver.h"


int main( int argc, char** argv ) {
    ros::init( argc, argv, "get_images" ); 
    ros::NodeHandle n( "~" ); 

    /**
     * get the folder name from the parameter server
     */

    std::string folderName;
    n.getParam("/folder", folderName);

    ImageSaver saver (folderName);

    /**
     * subscribe to relevant camera topics
     */

    ros::Subscriber img_sub = n.subscribe( "/camera/color/image_raw", 1, &ImageSaver::get_image, &saver);
    ros::Subscriber depth_sub = n.subscribe( "/camera/aligned_depth_to_color/image_raw", 1, &ImageSaver::get_depth, &saver);   
    ros::Subscriber info_sub = n.subscribe( "/camera/color/camera_info", 1, &ImageSaver::get_info, &saver); 
    ros::Subscriber pc_sub = n.subscribe( "/camera/depth/color/points", 1, &ImageSaver::get_pointCloud, &saver);          
    ros::Subscriber tf_sub = n.subscribe( "/tf_static", 1, &ImageSaver::get_tf, &saver);   

    /**
     * subscribe to EnterPressed topic, which checks if the user pressed enter to take a picture
     */
    ros::Subscriber enter_sub = n.subscribe( "/EnterPressed", 1, &ImageSaver::savePicture, &saver);

    /**
     * publisher for rgb-picture so it can be visualized
     */
    ros::Publisher pub = n.advertise < sensor_msgs::Image >("/image_pub", 1 );
    ros::Rate loop_rate( 1 );

    while( ros::ok( ) ) { 
        pub.publish(saver.image);
        ros::spinOnce();

        }
    return 0;                                          
}