/**
 * @file FrameHandler.cpp
 * @author Jan Korecky
 * @brief cpp file for class which can receive and output OpenPose data
 * @version 1
 * @date 2020-06-30
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include "FrameHandler.h"

/**
 * callback for the frame-topic where the OpenPose-Wrapper data is published
 * saves the data in local frame-variable
 * extracts the relevant data and outputs it to the console
 */

void FrameHandler::handleFrame( const ros_openpose::Frame::ConstPtr& input_frame ) {       
    ROS_INFO_STREAM("Frame arrived!");
    frame = *input_frame;

    extractData();
    outputData();
}

/**
 * saves the amount of people in the frame and the amount of body parts
 * can be extended to extract more data if necessary
 */

void FrameHandler::extractData(){
    numberOfPeople = frame.persons.size();
    numberOfBodyKeyPoints = frame.persons[0].bodyParts.size();
}

/**
 * print the position of the input body part of the input person
 */

void FrameHandler::printBodyPart(std::string info, int index, int person){
    ROS_INFO_STREAM(info);
    ROS_INFO_STREAM("x: " << frame.persons[person].bodyParts[index].point.x <<
        " y: " << frame.persons[person].bodyParts[index].point.y <<
        " z: " << frame.persons[person].bodyParts[index].point.z);
}

/**
 * print the position of selected body parts for every person in the frame
 * can be extended to print more/other body part positions
 */

void FrameHandler::outputData(){
    ROS_INFO_STREAM("Number of People in Frame: " << numberOfPeople);
    for (int i =0; i<numberOfPeople; i++){
        ROS_INFO_STREAM("------------Person " << i+1 << "------------");
        printBodyPart("Nose Position:", NOSE_INDEX, i);
        printBodyPart("Left Shoulder Position:", LSHOULDER_INDEX, i);
        printBodyPart("Right Shoulder Position:", RSHOULDER_INDEX, i);
        printBodyPart("Mid Hip Position:", MIDHIP_INDEX, i);
        printBodyPart("Left Knee Position:", LKNEE_INDEX, i);
        printBodyPart("Right Knee Position:", RKNEE_INDEX, i);
        printBodyPart("Left Foot Position:", LBIGTOE_INDEX, i);
        printBodyPart("Right Foot Position:", RBIGTOE_INDEX, i);
        ROS_INFO_STREAM("------------------------------");
    }
    
}