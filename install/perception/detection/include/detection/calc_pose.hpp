/*! 
 *  \brief     ROS class for 3D (x,y,z) pose detection of objects
 *  \details   This class is used to 
 *  \author    Georg Novotny (NovoG93)
 *  \author    Christoph Pöschko
 *  \version   1.0
 *  \date      2019
 *  \pre       Install libyaml-cpp-dev and libtesseract-dev tesseract-ocr tesseract-ocr-deu
 *  \todo      Add aditional threshold for objects of type 2 (tafel)
 *  \bug       
 *  \copyright GPLv3.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 * 
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */



#include <math.h>
#include <string.h>
#include <yaml-cpp/yaml.h>

//ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <image_transport/image_transport.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
//yolo
#include <darknet_ros_msgs/BoundingBoxes.h>

//OCTOmap
#include <octomap_msgs/Octomap.h>
#include <octomap/AbstractOcTree.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>

//OPENCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

//sign detect
#include <leptonica/allheaders.h>
#include <tesseract/baseapi.h>





class calc_pose{
    public:
        calc_pose(YAML::Node config_cam, YAML::Node config_detection);


        void callback_yolo(const darknet_ros_msgs::BoundingBoxes &bboxes);
        void callback_octomap(const octomap_msgs::Octomap &msg);
		void imageCallback(const sensor_msgs::ImageConstPtr& msg);

    private:
        ros::NodeHandle n;
        ros::Subscriber sub_yolo;
        ros::Subscriber sub_octo;
        ros::Publisher marker_pub_arrow;
        ros::Publisher marker_pub_cube;
		ros::Publisher marker_pub_sign;
        ros::Publisher pose_pub;
	
		//variables for debug
        std::map< std::string, ros::console::levels::Level> logger;
		bool debug;
		int time_show;
		image_transport::ImageTransport it;
		image_transport::Subscriber sub_image;


        //Variables for pose calculation
        std::string map_frame_id, cam_frame_id;
        float width;
        float oszil_range_y;
        float oszil_range_z;
        float step_size;
        bool init_pose_array;
        std::vector<octomap::point3d> point_vector, point_vector_sign;
        std::vector<double> humanx, humany, humanz, signx, signy, signz;
		int cnt_human, cnt_arrow, cnt_sign;
        const std::string package_path = ros::package::getPath("detection");
        const std::string human_csv_path = package_path+"/human.csv";
        const std::string sign_csv_path = package_path+"/sign.csv";
        tf2_ros::Buffer tfBuffer;
		tf2_ros::Buffer tfBuffer2;
		geometry_msgs::TransformStamped transform_;
		geometry_msgs::TransformStamped transform_direction_;
		geometry_msgs::TransformStamped transformStamped3;

        //Variables for cam configuration
        double xres, yres;
        double psize;
        float cam_dist;
        double xarea, yarea;
        //variables for boundingbox yolo
        std::string object_name;
        double min_probability;
	    int xmin, xmax, ymin,ymax;
        //variables for octomap
        std::vector<signed char, std::allocator<signed char> > data;
        bool binary;
        float resolution;
        octomap::AbstractOcTree* oct_map=NULL;
        octomap::OcTree* tree=NULL, *tree2_oc;

		//variables for sign calculations
		cv::Mat orig_img;
		int width_bb;
		int heigth_bb;
		cv::Mat image;
		int depth;
		cv::Mat im;
        std::unique_ptr<tesseract::TessBaseAPI> api;
		char *outText;
		bool reco;
        int klasse;
		double r, p, y;
		double y_buf;
		octomap::point3d direction_rot;
		double direction_x, direction_y, direction_z; 

        //Functions
        /*!
         * @brief Is an empty function in which preproccesing of the recieved ocotmap could be done
         */
        void preproc_octo();

        /*!
         * @brief Calculates the depth of the detected object using the octomap and interseciton.
         *        Acquires the transformation from "map_frame_id" to "cam_frame_id" and casts an
         *        oszilating ray from origin (tf map_frame_id -> cam_frame_id) to detected object.
         * @param ycenter_sens_m y coordinate of pricipal point of camera in m
         * @param zcenter_sens_m z coordinate of pricipal point of camera in m
         * @param klasse Is used to change calculation for person (0) or sign (1)
         */
        void calc_3d_pose(double ycenter_sens_m, double zcenter_sens_m, int klasse);

        /*!
         * @brief Calculates a transformation matrix from target to source and stores it in
         *          #calc_pose::transform_
         *          Waits 0.5 seconds from transformation untill it throws an exception
         * @param target Target coordinate frame
         * @param source Source coordinate frame
         */
    	void get_tf(std::string target, std::string source);


		void pub_marker_arrow();
        void pub_marker_object(const octomap::point3d intersection);
	    void pub_marker_sign(const octomap::point3d intersection);

        /*!
         * @brief Compares the recognices chars on the detected text to predefined values
         * @param output Output of text recognition
         * @return Returns class of detected sign
         */
		int check_sign(char *output);

        /*!
         * @brief Initializes tesseract for number recognition on signs
         * @return Class of detected sign
         */
		int init_tesseract();
		void destruct_tesseract();

        /*!
         * @brief Compares the recognices chars on the detected text to predefined values
         * @todo Check what foruth try on cropping image does (float height_new)
         * @return Class of detected sign
         */
		int run_sign_detect();


        /*!
         * @brief   Crops the image within the given params and runs tesseract text recognition.
         *          Utilizes #calc_pose::crop_image to determine the type of detected sign.
         * @return  Integar that stands for different types of signs. See #calc_pose::check_sign
         */
		int crop_image(cv::Mat image, cv::Rect roi, cv::Mat crop, int width_bb, int heigth_bb, int xmin, int ymin);
        int write_human(float x, float y, float z);
        int write_sign(float x, float y, float z, int klasse);

		octomap::point3d rotate_vector_by_quaternion();

	

		
		


};
