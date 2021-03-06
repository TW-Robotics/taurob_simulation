/**
 * @file publish_picture.cpp
 * @author Jan Korecky
 * @brief main-function for publishing the data saved in bagfiles
 * @version 1
 * @date 2020-06-30
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include "BagOpener.h"


int main( int argc, char** argv ) { 
    
    ros::init( argc, argv, "PicPublisher" );
    ros::NodeHandle n( "~" ); 

    /**
     * get the path name from the parameter server
     */
    std::string folder;
    n.getParam("/folderpath", folder);
    BagOpener myOpener (folder);

    /**
     * create all necessary publishers to the respective topics
     */
    ros::Publisher pic_pub = n.advertise< sensor_msgs::Image >("/camera/color/image_raw", 1 );
    ros::Publisher depth_pub = n.advertise< sensor_msgs::Image >("/camera/aligned_depth_to_color/image_raw", 1 ); 
    ros::Publisher info_pub = n.advertise< sensor_msgs::CameraInfo >("/camera/color/camera_info", 1 );    
    ros::Publisher pc_pub = n.advertise< sensor_msgs::PointCloud2 >("/camera/depth/color/points", 1 );   
    ros::Publisher tf_pub = n.advertise< tf2_msgs::TFMessage >("/tf_static", 1 );  
    ros::Rate loop_rate( 10 ); // Spin with this rate


    while( ros::ok( ) ) {      // Do till ROS error occurs (e.g.: strg+C or ROS shutdown signal)
        /**
         * get data from bagfiles and publish it
         */

        myOpener.grabData();
        pic_pub.publish( myOpener.rgb );    // Send message
        depth_pub.publish( myOpener.depth );
        info_pub.publish( myOpener.info );
        pc_pub.publish(myOpener.pc);
        tf_pub.publish( myOpener._tf );
        ros::spinOnce( );              // Update buffer
    }
    return 0; // Everything is fine :)
}