/**
 * @file FrameHandler.h
 * @author Jan Korecky
 * @brief header file for class which can receive and output OpenPose data
 * @version 1
 * @date 2020-06-30
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include "ros/ros.h"
#include "ros_openpose/Frame.h"

/**
 * @brief defines for at which index to look for which body part info
 * 
 */
#define NOSE_INDEX 0
#define NECK_INDEX 1
#define RSHOULDER_INDEX 2
#define RELBOW_INDEX 3
#define RWRIST_INDEX 4
#define LSHOULDER_INDEX 5
#define LELBOW_INDEX 6
#define LWRIST_INDEX 7
#define MIDHIP_INDEX 8
#define RHIP_INDEX 9
#define RKNEE_INDEX 10
#define RANKLE_INDEX 11
#define LHIP_INDEX 12
#define LKNEE_INDEX 13
#define LANKLE_INDEX 14
#define REYE_INDEX 15
#define LEYE_INDEX 16
#define REAR_INDEX 17
#define LEAR_INDEX 18
#define LBIGTOE_INDEX 19
#define LSMALLTOE_INDEX 20
#define LHEEL_INDEX 21
#define RBIGTOE_INDEX 22
#define RSMALLTOE_INDEX 23
#define RSHEEL_INDEX 24
#define BACKGROUND_INDEX 25



class FrameHandler{

    private:
        /**
         * @brief the frame where the OpenPose-Wrapper output will be saved to
         * 
         */
        ros_openpose::Frame frame;
        int numberOfPeople;
        int numberOfBodyKeyPoints;

        /**
         * @brief save relevant data from frame-topic
         * 
         */
        void extractData();

        /**
         * @brief output the position of a body part to console
         * 
         * @param info the text that is added as info, for example "Nose position: "
         * @param index index of the body part info which should be printed
         * @param person which person's body part it is
         */
        void printBodyPart(std::string info, int index, int person);

        /**
         * @brief output the relevant data for each person in the frame
         * 
         */
        void outputData();
        

    public:
    /**
     * @brief callback-method for frame-topic
     * the method is executed when data gets published to the frame topic
     * the data is saved and relevant data is printed to the console
     * @param input_frame pointer to the data of the frame-topic
     */
        void handleFrame(const ros_openpose::Frame::ConstPtr& input_frame);
};
