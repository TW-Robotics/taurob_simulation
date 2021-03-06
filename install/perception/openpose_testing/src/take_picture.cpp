/**
 * @file take_picture.cpp
 * @author Jan Korecky
 * @brief main-function for checking if the user pressed enter to save a picture
 * @version 1
 * @date 2020-06-30
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include "ros/ros.h"                
#include "openpose_testing/enterPress.h"
#include "stdio.h"
#include <ncurses.h>

int main( int argc, char** argv ) { 

    ros::init( argc, argv, "EnterPub" );
    ros::NodeHandle n( "~" ); 
    ros::Publisher myPub = n.advertise< openpose_testing::enterPress >(
        "/EnterPressed", 1 );   
    ros::Rate loop_rate( 10 ); 
    openpose_testing::enterPress msg;    

    /**
     * intialize curses
     */
    initscr();
    char c;
    
    while( ros::ok( ) ) { 
        c = getch();
        /**
         * check if the received char is '\n' and therefore enter was pressed
         */
        
        if(c == '\n'){
            /**
             * publish the message so the callback function of the saver is called
             */
            msg.enterPressed = true; 
            myPub.publish( msg ); 
            msg.enterPressed = false;
            ros::spinOnce( );              
        }
    }

    /**
     * end the curses window
     */
    endwin();
    return 0;
}