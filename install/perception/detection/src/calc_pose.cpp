/*
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

#include "detection/calc_pose.hpp"


calc_pose::calc_pose(YAML::Node config_cam, YAML::Node config_detection):
    n("~"),
    sub_yolo(n.subscribe("/darknet_ros/bounding_boxes", 1, &calc_pose::callback_yolo, this)),
    sub_octo(n.subscribe("/octomap_binary", 1, &calc_pose::callback_octomap, this)),
    marker_pub_arrow(n.advertise<visualization_msgs::Marker>("/visualization_marker_arrow",1)),
    marker_pub_cube(n.advertise<visualization_msgs::Marker>("/visualization_marker_human",1)),
    marker_pub_sign(n.advertise<visualization_msgs::Marker>("/visualization_marker_sign",1)),
    pose_pub(n.advertise<geometry_msgs::Pose>("/detected_pose",1)),
    debug(false),
	time_show(200),
	it(n),
    map_frame_id(config_detection["map_frame_id"].as<std::string>()),
    cam_frame_id(config_detection["cam_frame_id"].as<std::string>()),
    oszil_range_y(config_detection["oszil_range_y"].as<double>()),
    oszil_range_z(config_detection["oszil_range_z"].as<double>()),
    step_size(config_detection["step_size"].as<double>()),
    init_pose_array(false),
	cnt_human(0),
	cnt_arrow(0),
	cnt_sign(0),
    xres(config_cam["xres"].as<int>()),
    yres(config_cam["yres"].as<int>()),
    psize(config_cam["psize"].as<double>()),
    cam_dist(config_cam["cam_dist"].as<float>()),
    xarea(config_cam["xarea"].as<double>()),
    yarea(config_cam["yarea"].as<double>()),
    binary(false),
    resolution(0.0),
    orig_img(),
	reco(false),
	y_buf(0)
    {
        ROS_INFO_STREAM("Init node: "<< ros::this_node::getName().c_str());
        ROS_INFO_STREAM("Path to pkg: "<<package_path);
        ros::console::get_loggers(logger);
	    sub_image=it.subscribe("/camera_rgb/image_raw", 1, &calc_pose::imageCallback,this);
        std::string cam_config="Configuration of cam:\n", detection_config="Configuration of detection:\n";
        for (YAML::const_iterator it = config_cam.begin(); it != config_cam.end(); ++it)
        {
            cam_config.append("\t");
            cam_config.append(it->first.as<std::string>());
            cam_config.append(": ");
            cam_config.append(it->second.as<std::string>());
            cam_config.append("\n");
        }
        ROS_INFO_STREAM(cam_config);
        for (YAML::const_iterator it = config_detection.begin(); it != config_detection.end(); ++it)
            {
                detection_config.append("\t");
                detection_config.append(it->first.as<std::string>());
                detection_config.append(": ");
                detection_config.append(it->second.as<std::string>());
                detection_config.append("\n");
            }
        ROS_INFO_STREAM(detection_config);
    }
	
		


/*@brief  Callback for the yolonet boundingboxes. Starts the calculation of the 
 *          depth of the detected object by getting the bounding box values and 
 *          calculating the position in the RGB image.
 */
void calc_pose::callback_yolo(const darknet_ros_msgs::BoundingBoxes &bboxes)
{
	get_tf(map_frame_id, cam_frame_id);
	tf2::Quaternion q(transform_.transform.rotation.x, transform_.transform.rotation.y, transform_.transform.rotation.z, transform_.transform.rotation.w);
	tf2::Matrix3x3 m(q);
	m.getRPY(r, p, y, 2);
	y=y*M_PI;

	int klasse=0;
    double xcenter=0, ycenter=0;
    double ycenter_sens=0, zcenter_sens=0;
    double ycenter_sens_m=0, zcenter_sens_m=0;
    n.param<std::string>("/object_name", object_name, "person");
    n.param<double>("/min_probability", min_probability, 0.5);
    ROS_DEBUG_STREAM_THROTTLE(5,"Sizeof bboxes: "<<(bboxes.bounding_boxes.size()));
    for (int i=0; i<bboxes.bounding_boxes.size(); i++){
        double prob=bboxes.bounding_boxes[i].probability;
        if (prob>=min_probability){
            xmin=bboxes.bounding_boxes[i].xmin;
            xmax=bboxes.bounding_boxes[i].xmax;
            ymin=bboxes.bounding_boxes[i].ymin;
            ymax=bboxes.bounding_boxes[i].ymax;
			//calculate width and heigth of bbx
			width_bb= xmax-xmin;
			heigth_bb= ymax-ymin;
            //calculate center of detected object in pixel
            xcenter= xmin + (xmax-xmin)/2;
            ycenter= ymin + (ymax-ymin)/2;
            //transform distance of boundingbox's center to distance of sensor's center
            ycenter_sens=xres/2-xcenter;
            zcenter_sens=yres/2-ycenter;
            //calculate center of detected object in meter refered to sensor's center
            ycenter_sens_m=ycenter_sens*psize;
            zcenter_sens_m=zcenter_sens*psize;
            ROS_DEBUG_STREAM_THROTTLE(2, "Detected pose of object ("<< object_name.c_str() <<") of Bbox ["<< i <<" referenced to sensor's center:"<< std::endl
                <<"y: "<< ycenter_sens_m <<std::endl
                <<"z: "<< zcenter_sens_m);
			if(bboxes.bounding_boxes[i].Class=="person" || bboxes.bounding_boxes[i].Class=="Person"){
				klasse=0;
                ROS_DEBUG_STREAM_THROTTLE(2,"Detected Person");
                calc_3d_pose(ycenter_sens_m, zcenter_sens_m,klasse);
			}
			else if(bboxes.bounding_boxes[i].Class=="Tafel"){
				klasse=1;
                ROS_DEBUG_STREAM_THROTTLE(2,"Detected Sign");
                calc_3d_pose(ycenter_sens_m, zcenter_sens_m,klasse);
			}
			 else{
			ROS_WARN_STREAM_THROTTLE(3,"Couldn't find correct class "<< bboxes.bounding_boxes[i].Class);
			}
            
        }
        else{
			ROS_WARN_STREAM_THROTTLE(3,"Probabilty of object "<< bboxes.bounding_boxes[i].Class<< "not high enough "<<prob<<"!");
		}

    }

	y_buf=y;
}

void calc_pose::callback_octomap(const octomap_msgs::Octomap &msg)
{
    ROS_DEBUG_THROTTLE(5,"Got octomap");
    oct_map= octomap_msgs::binaryMsgToMap(msg);
    ROS_DEBUG_THROTTLE(5,"msg to map converted successfully");
    tree = dynamic_cast<octomap::OcTree*>(oct_map);
	ROS_DEBUG_THROTTLE(5,"msg to msg converted successfully");
}
				 
void calc_pose::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exeption: %s", e.what());
        return;
    }
    orig_img = cv_ptr->image; 
}
				 
			 


/*@brief  Publishes one arrow marker for rviz visualisation of one detected object.
 */
void calc_pose::pub_marker_arrow(){
    float duration=10.0;
    std::string ns="arrow";
    ns.append("_");
    ns.append(std::to_string(cnt_arrow));
    ros::param::get("/duration", duration);
    geometry_msgs::Point p;
    visualization_msgs::Marker marker;
    marker.header.frame_id = map_frame_id;
    marker.header.stamp = ros::Time();
    marker.lifetime = ros::Duration(duration);

    marker.ns=ns;
    marker.id = cnt_arrow;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    
    p.x=transform_.transform.translation.x*10; 
    p.y=transform_.transform.translation.y*10;
    p.z=transform_.transform.translation.z*10;
    marker.points.push_back(p);
	p.x=direction_rot.x();
    p.y=direction_rot.y();
    p.z=direction_rot.z();
    marker.points.push_back(p);
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker_pub_arrow.publish(marker);
	
	cnt_arrow=cnt_arrow+1;
	if(cnt_arrow==5){
	cnt_arrow=0;
	}

}


/*@brief    Checks if the current detected object is not near (+- tolerance) to other detecte objects.
 *          If not near -> put into array, display human body
 *          Else break 
 */
void calc_pose::pub_marker_object(const octomap::point3d intersection){
    float tolerance=0.50;
    width=0.6; 
    std::string ns="human";
    ns.append("_cube_");
    ns.append(std::to_string(cnt_human));
    ros::param::get("/tolerance", tolerance);

   bool already_found=false;
   if (!humanx.size())
   {
        ROS_INFO_ONCE("Assaigning initial value for pose array");
 
        ROS_INFO_ONCE("Assaigned");
        humanx.push_back(intersection.x());
        humany.push_back(intersection.y());
        humanz.push_back(intersection.z());
        ROS_INFO("First Human X:%f Y:%f Z:%f \n", humanx[0],humany[0], humanz[0]);

    }else{
		//iterate over all already found objects
        int h_size=humanx.size();
		for(int i=0; i<h_size; i++){
                if((humanx[i]-tolerance) < intersection.x() && intersection.x() < (humanx[i]+tolerance) && (humany[i]-tolerance) < intersection.y() && intersection.y() < (humany[i]+tolerance) && already_found==false){
                    ROS_INFO_STREAM_THROTTLE(1, "Found new human at position: " <<intersection);
                    already_found=true;    
                }
        }
        if(already_found==false){
                ROS_DEBUG_STREAM_THROTTLE(3, "Detected similar human at position: " <<intersection);
                fprintf(stdout, "Found new human at position %f %f %f\n", intersection.x(), intersection.y(),intersection.z());
                humanx.push_back(intersection.x());
                humany.push_back(intersection.y());
                humanz.push_back(intersection.z());
        }
    }
    if(already_found==false)
    {
        ROS_DEBUG_STREAM("Drawing human...\n");
        visualization_msgs::Marker marker;
        marker.header.frame_id = map_frame_id;
        marker.header.stamp = ros::Time();
        marker.ns=ns;
        marker.id = cnt_human;
        
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker.mesh_resource = "package://operator/human_blender.stl";
        //marker.mesh_resource = "package://detection/src/human_blender.stl";

        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x=intersection.x();
        marker.pose.position.y=intersection.y();
        marker.pose.position.z=0;
        marker.pose.orientation.x=0.0;
        marker.pose.orientation.y=0.0;
        marker.pose.orientation.z=0.0;
        marker.pose.orientation.w=1.0;

        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 1;
        marker.color.a = 1.0; 
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        
        marker_pub_cube.publish(marker);
        cnt_human=cnt_human+1;
        write_human(intersection.x(),intersection.y(),intersection.z());
    }
    already_found=true;  
}

	/*@brief    Checks if the current detected object is not near (+- tolerance) to other detecte objects.
     *          If not near -> put into array, display sign
     *          Else break 
     */			 			 
void calc_pose::pub_marker_sign(const octomap::point3d intersection)
{
    float tolerance=0.10;
    width=0.6;
    std::string ns="sign";
    ns.append("__");
    ns.append(std::to_string(cnt_sign));
    ros::param::get("/tolerance", tolerance);

   
   bool already_found=false;
   if (!signx.size()){
        ROS_INFO_ONCE("Assaigning initial value for pose array");
        signx.push_back(intersection.x());
        signy.push_back(intersection.y());
        signz.push_back(intersection.z());
        ROS_DEBUG_STREAM("First sign X: "<<signx[0] <<" Y: "<< signy[0] << " Z: "<<signz[0]);
	    ROS_DEBUG_STREAM("Size of already found signs " << signx.size());
        ROS_DEBUG_STREAM("going into tesseract \n");

    }else{
        ROS_DEBUG_STREAM("Size of already found signs: "<<  signx.size());
		//iterate over all already found objects
        int h_size=signx.size();
		for(int i=0; i<h_size; i++){
            ROS_DEBUG("sign old X:%f Y:%f Z:%f || Intersection X:%f Y:%f Z:%f\n", signx[i],signy[i], signz[i], intersection.x(),intersection.y(),intersection.z());
	        ROS_DEBUG("Size of already found signs %d \n", h_size);
		    ROS_DEBUG( "Value of x in vector [%d]: %f, Value of intersection.x: %f, Value of tolerance: %f \n",i, point_vector[i].x(), intersection.x(), tolerance);
            if((signx[i]-tolerance) < intersection.x() && intersection.x() < (signx[i]+tolerance) && (signy[i]-tolerance) < intersection.y() && intersection.y() < (signy[i]+tolerance) && already_found==false){
                ROS_DEBUG_STREAM_THROTTLE(1, "Found new sign at position: " <<intersection);
                ROS_INFO_THROTTLE(2,"Already detected this sign");
                already_found=true;    
            }
        }
        if(already_found==false){
                ROS_DEBUG_STREAM_THROTTLE(3, "Detected similar sign at position: " <<intersection);
                ROS_INFO("Found new sign at position %f %f %f\n", intersection.x(), intersection.y(),intersection.z());
                signx.push_back(intersection.x());
                signy.push_back(intersection.y());
                signz.push_back(intersection.z());
        }
		}

    if(already_found==false)
    {
        ROS_DEBUG_STREAM("Drawing sign...\n");

        visualization_msgs::Marker marker_sign;
        marker_sign.header.frame_id = map_frame_id;
        marker_sign.header.stamp = ros::Time();
        marker_sign.ns=ns;
        marker_sign.id = cnt_sign;
    
	    marker_sign.type = visualization_msgs::Marker::MESH_RESOURCE;
	    marker_sign.mesh_resource = "package://operator/tafel.stl";
        //marker_sign.mesh_resource = "package://detection/src/tafel.stl";
    
	    marker_sign.action = visualization_msgs::Marker::ADD;
        marker_sign.pose.position.x=intersection.x();
        marker_sign.pose.position.y=intersection.y();
        //marker_sign.pose.position.y=intersection.z();
        marker_sign.pose.position.z=0.1;
        marker_sign.pose.orientation.x=0.0;
        marker_sign.pose.orientation.y=0.0;
        marker_sign.pose.orientation.z=0.0;
        marker_sign.pose.orientation.w=1.0;

        marker_sign.scale.x = 1;
        marker_sign.scale.y = 0.7;
        marker_sign.scale.z = 1;
        marker_sign.color.a = 1.0; 
        marker_sign.color.r = 1.0;
        marker_sign.color.g = 0.0;
        marker_sign.color.b = 0.0;
    

        marker_pub_sign.publish(marker_sign);
	    cnt_sign=cnt_sign+1;

        klasse=0;
        write_sign(intersection.x(),intersection.y(),intersection.z(), klasse);
    }
    already_found=true;  
}
				 

int calc_pose::write_human(float x, float y, float z)
{
    FILE *out;
    out=fopen(human_csv_path.c_str(), "a");
    if(out==NULL){
        ROS_ERROR_STREAM("Error opening: "<<human_csv_path);
        return -1;
    }
    fprintf(out,"%f,%f\n",x,y);
    fclose(out);
    ROS_INFO_STREAM_THROTTLE(2,"Wrote to human.csv");
    return 0;
}

int calc_pose::write_sign(float x, float y, float z, int klasse)
{	
    FILE *out;
	out=fopen(sign_csv_path.c_str(), "a");
    if(out==NULL){
        ROS_ERROR_STREAM("Error opening: "<<sign_csv_path);
        return -1;
    }
	fclose(out);
    ROS_INFO_THROTTLE(2,"Wrote: '%i,%f,%f' to sign.csv", klasse, x,y);
    return 0;
}


void calc_pose::get_tf(std::string target, std::string source){		 
				 
	tf2_ros::TransformListener tfListener(tfBuffer);
	
    try{
        transform_ = tfBuffer.lookupTransform(target,source,
                               ros::Time::now(), 
                               ros::Duration(0.5));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
	    ROS_WARN_STREAM("Couldn't transform, try next time");
    }
}


int calc_pose::init_tesseract()
{
    ROS_DEBUG_STREAM("Start initializing tesseract");
    int klasse=0;
    image=orig_img;
    if(logger[ROSCONSOLE_DEFAULT_NAME]==ros::console::levels::Debug)
    {
        cv::imshow("orig_img", image);
	    cv::waitKey(time_show);
    }
	cv::cvtColor(image, im, CV_BGR2RGBA);
	api.reset( new tesseract::TessBaseAPI() );
    ROS_DEBUG_STREAM(ros::this_node::getName() <<": Tesseract Version: " << api->Version());
    ROS_DEBUG("path for tesseract: %s", getenv("TESSDATA_PREFIX"));
	if (api->Init(NULL, "digits_comma")) 
    {  
        ROS_ERROR_STREAM("Could not initialize tesseract!");
        exit(1);
    }
    ROS_DEBUG_STREAM("Done initializing tesseract\n");
	klasse=run_sign_detect();
    return klasse;
    
}

void calc_pose::destruct_tesseract(){
				 
	// Destroy used object and release memory
    api->End();
    delete [] outText;
}

int calc_pose::run_sign_detect(){
//first try to get numbers of plate
	api->SetImage(im.data, im.cols, im.rows, 4, 4*im.cols);
	//set part of image
	api->SetRectangle(xmin, ymin, width_bb, heigth_bb);

    cv::Rect roi;
	cv::Mat crop;
	int out_crop=0;
	float ymin_new;

    // Get OCR result
    outText = api->GetUTF8Text();
    out_crop=check_sign(outText);
	if(out_crop!=0){
		ROS_INFO_STREAM("Found Sign original: "<<outText);
		reco=true;
	}
		
 	//second try: full image with offset
	if(reco==false)
	{
		ymin_new=ymin-(heigth_bb*0.1);
		for(float i=1,j=1; i>0.85; i=i-0.025, j=j-0.05){
			out_crop=crop_image(image,roi,crop,width_bb*j, (heigth_bb)*j, xmin+(width_bb*(1-i)), ymin_new+((heigth_bb)*(1-i)));
			if(out_crop!=0){
				reco=true;
				break;
			}

		}
	}
		
	//third try: cut rectangle in half + 10%offset
	ymin=ymin-(heigth_bb*0.05);
	if(reco==false)
	{
		ymin_new=ymin-(heigth_bb*0.1);
		for(float i=1,j=1; i>0.85; i=i-0.025, j=j-0.05){
			out_crop=crop_image(image,roi,crop,width_bb*j, (heigth_bb/2)*j, xmin+(width_bb*(1-i)), ymin_new+((heigth_bb/2)*(1-i)));
			if(out_crop!=0){
				reco=true;
				break;
			}
		}
	}
		
	//fourth try: cut rectangle from bottom
    // Was passiert hier?
	float heigth_new;
	for(float g=1; g>0.85; g=g-0.025)
    {
		heigth_new=heigth_bb-heigth_bb*g;
        if(reco==false)
        {
            ymin_new=ymin-(heigth_bb*0.1);
            for(float i=1,j=1; i>0.85; i=i-0.025, j=j-0.05){
                out_crop=crop_image(image,roi,crop,width_bb*j, (heigth_bb)*j, xmin+(width_bb*(1-i)), ymin_new+((heigth_bb)*(1-i)));
                if(out_crop!=0){
                    reco=true;
                    break;
                }
            }	
        }
	}
	ROS_DEBUG_STREAM("finished cropping image...");	
	if(out_crop!=0){
    	ROS_INFO("------------!!!FOUND Sign!!!---------------");
        ROS_WARN_STREAM("Out_crop= " << out_crop);
		//ros::Rate r(10000);
		//r.sleep();

	}

    return out_crop;
}
				 

				 
				 
int calc_pose::check_sign(char *output){
	const char* str70="70";
	const char* str72="72";
	const char* str723="723";
	const char* str74="74";
	const char* str75="75";
	const char* str76="76";
	const char* str78="77";

	int size_output=std::strlen(output);
	if(size_output==0){
		return 0;
	}
	
	if(logger[ROSCONSOLE_DEFAULT_NAME]==ros::console::levels::Debug)	ROS_DEBUG_STREAM("number of chars in output " <<std::strlen(output)<<" these are: "<< output);

	
	if(size_output==3){
		if(strncmp(str70,output,2)==0){
            fprintf(stdout,"output: %s \n", output);
	        return 1;
            }
        if(strncmp(str72,output,2)==0){
            fprintf(stdout,"output: %s \n", output);
	        return 2;
        }
        if(strncmp(str74,output,2)==0){
            fprintf(stdout,"output: %s \n", output);
	        return 4;
        }
        if(strncmp(str75,output,2)==0){
            fprintf(stdout,"output: %s \n", output);
	        return 5;
        }
        if(strncmp(str76,output,2)==0){
            fprintf(stdout,"output: %s \n", output);
	        return 6;
        }
        if(strncmp(str78,output,2)==0){
            fprintf(stdout,"output: %s \n", output);
	        return 7;
        }
	}
	
	else if(size_output==4){
		if(strncmp(str723,output,3)==0){
	fprintf(stdout,"output: %s \n", output);
	return 3;}
	}
	
    return 0;
}
				 
int calc_pose::crop_image(cv::Mat image, cv::Rect roi, cv::Mat crop, int width_bb, int heigth_bb, int xmin, int ymin){
		api->SetRectangle(xmin, ymin, width_bb, heigth_bb);
        int output=0;
    	// Get OCR result
    	char* outText = api->GetUTF8Text();
		//first try to get numbers of plate
		roi.x=xmin;
		roi.y=ymin;
		roi.width=width_bb;
		roi.height=heigth_bb;
		crop = image(roi);
		if(logger[ROSCONSOLE_DEFAULT_NAME]==ros::console::levels::Debug)
        {
		    cv::imshow("crop", crop);
		    cv::waitKey(time_show);
		    cv::destroyWindow("crop");
		}
		
        output=check_sign(outText);

	return output;

}
				 
octomap::point3d calc_pose::rotate_vector_by_quaternion()
{
	transform_direction_ = transform_;
    transform_direction_.transform.rotation.x=-transform_.transform.rotation.x;
	transform_direction_.transform.rotation.y=-transform_.transform.rotation.y;
	transform_direction_.transform.rotation.z=-transform_.transform.rotation.z;
	transform_direction_.transform.rotation.w=-transform_.transform.rotation.w;

	ROS_DEBUG_STREAM("-------------------------------------------------------");
	ROS_DEBUG_STREAM(transform_);
	ROS_DEBUG_STREAM(transform_direction_);
	ROS_DEBUG_STREAM("*******************************************************");
	
	geometry_msgs::Pose origin_pos;
	origin_pos.position.x =direction_x;
  	origin_pos.position.y =direction_y;
  	origin_pos.position.z =direction_z;
	origin_pos.orientation.x=0;
	origin_pos.orientation.y=0;
	origin_pos.orientation.z=0;
	origin_pos.orientation.w=1;
	
    geometry_msgs::Pose direction_pose;
    tf2::doTransform(origin_pos,direction_pose,transform_direction_);

	ROS_DEBUG_STREAM("Ausgabe destination: " << direction_pose);
	octomap::point3d direction_octo(direction_pose.position.x,direction_pose.position.y,direction_pose.position.z);
	ROS_DEBUG_STREAM("Vector direction_octo: "<<direction_octo);
	
	
	return direction_octo;
}
				 
void calc_pose::preproc_octo(){
    ROS_DEBUG_THROTTLE(10,"In here goes the preprocessing of the octomap tree");
}

/*@brief  Calculates the depth of the detected object using the octomap and interseciton.
 *        Acquires the transformation from "map_frame_id" to "cam_frame_id" and casts an
 *        oszilating ray from origin (tf map_frame_id -> cam_frame_id) to detected object.
 */
void calc_pose::calc_3d_pose(double ycenter_sens_m, double zcenter_sens_m, int klasse){
    preproc_octo();
    octomap::point3d temp;
    if(oct_map)
    {
		octomap::point3d origin(transform_.transform.translation.x, transform_.transform.translation.y, transform_.transform.translation.z);
  	    octomap::point3d direction;
  	    octomap::point3d ray_end;
		
		
        for(float i=oszil_range_y; i<=oszil_range_y; i+=step_size){
            for(float j=oszil_range_z; j<=oszil_range_z; j+=step_size){
                //direction=octomap::point3d(cam_dist, ycenter_sens_m*(1+i), zcenter_sens_m*(1+j));
				direction_x=cam_dist;
				direction_y=ycenter_sens_m;
				direction_z=zcenter_sens_m;
				//rotate the vector
				direction_rot=rotate_vector_by_quaternion();
				
				ROS_DEBUG_STREAM_THROTTLE(5,"Origin for raycasting" << origin);
				ROS_DEBUG_STREAM_THROTTLE(5,"Direction for raycasting" << direction_rot);
				ROS_DEBUG_STREAM_THROTTLE(5,"Difference of vectors" << direction_rot-origin);
				if(direction_rot.x()!=0.0){
					
                    pub_marker_arrow();
                    ROS_DEBUG_STREAM("Casting ray from "<< origin << " to the "<< direction <<" direction");
                
                    bool success = tree->castRay(origin, direction_rot-origin, ray_end, false);  
                    if(success)
                    {
                        octomap::point3d intersection;
                        success = tree->getRayIntersection(origin, direction_rot-origin, ray_end, intersection, 1.0);
                            if(success){
                                ROS_DEBUG_STREAM_THROTTLE(2,"Intersection at " << intersection);
                                if(klasse==0){
                                    pub_marker_object(intersection);
                                }
                                else {
                                    pub_marker_sign(intersection);
                                }
                                i=oszil_range_y;
                                j=oszil_range_z;
                        }
                    }
                    else ROS_WARN_THROTTLE(1,"No success in ray casting, try moving the robot arround");
                }
                else{
					ROS_INFO("Raycasting with direciton 0 \n");
                }
            }
        }
	}

    
}
				 
				 
