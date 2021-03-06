/**
 * @file getOpenPoseData.cpp
 * @author Jan Korecky
 * @brief main for receiving and printing the OpenPose-data
 * @version 1
 * @date 2020-06-30
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include "FrameHandler.h"

int main( int argc, char** argv ) {
    ros::init( argc, argv, "DataOutput" ); 
    ros::NodeHandle n( "~" ); 
    FrameHandler h;

    /**
     * subscribe to frame topic to which the OpenPose-Wrapper publishes
     * set handleFrame as callback-method 
     */
    ros::Subscriber sub = n.subscribe( "/frame", 1, &FrameHandler::handleFrame, &h );     
    ros::spin( );                                      
    return 0;                                          
}