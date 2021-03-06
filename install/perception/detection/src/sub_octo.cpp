#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <list>
#include <stdio.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/AbstractOcTree.h>
#include <octomap/ColorOcTree.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>


//variables for octomap
std::vector<signed char, std::allocator<signed char> > data;
bool binary=0;
float resolution=0.0;
octomap::AbstractOcTree * oct_map=NULL;
octomap::OcTree* tree=NULL, *tree2_oc;

//variables for boundingbox yolo
int xmin=0, xmax=0, ymin=0,ymax=0;
double xcenter=0, ycenter=0;
double xcenter_m=0, ycenter_m=0;
double ycenter_sens=0, zcenter_sens=0;
long double ycenter_sens_m=0, zcenter_sens_m=0;

//Paramters of camera
//resolution
//1448 x 1086
int xres=1280;
int yres=640;
//size of pixel; um to m
//3,45 µm
double psize=5.3*1e-6;
//distance sensor - lens
double cam_dist=0.011;
//sensor area
//4,995 mm x 3,746 mm
double xarea=0.006784;
double yarea=0.005427;
bool found_person;
ros::Publisher vis_pub;

//variables for lookup_transform
float x_ids,y_ids,z_ids;






//rostopic pub -r 10 /octomap_full octomap_msgs/Octomap  '{header: auto,binary: 'True', id: 'OcTree', resolution: 0.0500, data: [1,2,3]}'
	//ASS S100 -->Kameramodell


 	//----------Callback for octomap received ----------------------------//
	void octoCallback(const octomap_msgs::Octomap &msg)
{
	
	if(found_person==true){

				//transform to map
		
		 tf::TransformListener listener;
	    tf::StampedTransform transform;
    try{
		ros::Time t = ros::Time(0);
    listener.waitForTransform("/map", "/ids_mount",
                              t, ros::Duration(3.0));
    listener.lookupTransform("/map", "/ids_mount",
                             t, transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
	  x_ids=transform.getOrigin().x();
	  y_ids=transform.getOrigin().y();
	  z_ids=transform.getOrigin().z();
		
	//std::cout << "Transform from map to ids_mount X: " << x_ids << " Y: " << y_ids << " Z: " << z_ids << std::endl;
		
			ros::Rate r(1);
	oct_map= octomap_msgs::binaryMsgToMap(msg);
	if(oct_map){
	//	ROS_INFO("msg to map converted successfully");
		tree = dynamic_cast<octomap::OcTree*>(oct_map);
			if(tree){
				//std::cout << "map received: " << tree->getTreeDepth() << std::endl;
				}
	}
		

	//-----------use octomap function to calculate distances---------------//
		//for testing origing is equal to origin of axis of the map
	octomap::point3d origin(x_ids,y_ids,z_ids);
  	octomap::point3d direction;
  	octomap::point3d ray_end;
	//std::cout << "Tree Depth " << tree->getTreeDepth() << std::endl;


	// --------------- FIRST TRY ------------- //
     direction = octomap::point3d(cam_dist,ycenter_sens_m, zcenter_sens_m); //voxel of x,y,z
	//std::cout << " X Y Z: " << direction << std::endl;
    //std::cout << "casting ray from " << origin  << " in the " << direction << " direction"<< std::endl;
	std::cout << "1st try casting" << std::endl;
    bool success = tree->castRay(origin, direction, ray_end);  

    if(success){
      std::cout << "ray hit cell with center " << ray_end << std::endl;

      octomap::point3d intersection;
      success = tree->getRayIntersection(origin, direction, ray_end, intersection);
      if(success)
        std::cout << "entrance point is " << intersection << std::endl;
    }
		
		geometry_msgs::Point p;
		visualization_msgs::Marker marker;
		marker.header.frame_id = "ids_mount";
		marker.header.stamp = ros::Time();
		marker.ns = "my_namespace";
		marker.id = 0;
		marker.type = visualization_msgs::Marker::ARROW;
		marker.action = visualization_msgs::Marker::ADD;
		
		p.x=0;
		p.y=0;
		p.z=0;
		marker.points.push_back(p);
		/*p.x=cam_dist*10;
		p.y=ycenter_sens_m*10;
		p.z=zcenter_sens_m*10;*/
		p.x=cam_dist*100;
		p.y=ycenter_sens_m*100;
		p.z= zcenter_sens_m*100;
		marker.points.push_back(p);
		marker.scale.x = 0.1;
		marker.scale.y = 0.1;
		marker.scale.z = 0.1;
		marker.color.a = 1.0; // Don't forget to set the alpha!
		marker.color.r = 1.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;

		vis_pub.publish( marker );
		//std::cout << "Marker published: " << std::endl;
		r.sleep();
		
		// -------- SECOND TRY ------- //
	
	//direction = octomap::point3d(2.38,-0.14,0.68);
    direction = octomap::point3d(cam_dist,0.98*ycenter_sens_m, zcenter_sens_m); //voxel of x,y,z
	std::cout << "2nd try casting" << std::endl;    
//std::cout << "2nd try: casting ray from " << origin  << " in the " << direction << " direction"<< std::endl;
    success = tree->castRay(origin, direction, ray_end);  

    if(success){
      std::cout << "ray hit cell with center " << ray_end << std::endl;

      octomap::point3d intersection;
      success = tree->getRayIntersection(origin, direction, ray_end, intersection);
      if(success)
        std::cout << "entrance point is " << intersection << std::endl;
    }
		

		/*marker.header.frame_id = "map";
		marker.header.stamp = ros::Time();
		marker.ns = "my_namespace";
		marker.id = 0;
		marker.type = visualization_msgs::Marker::ARROW;
		marker.action = visualization_msgs::Marker::ADD;
		
		p.x=-0.62;
		p.y=-0.14;
		p.z= 0.69;
		marker.points.push_back(p);
		/*p.x=cam_dist*10;
		p.y=ycenter_sens_m*10;
		p.z=zcenter_sens_m*10;*
		p.x=cam_dist*50;
		p.y=0.98*ycenter_sens_m*50;
		p.z= zcenter_sens_m*50;
		marker.points.push_back(p);
		marker.scale.x = 0.1;
		marker.scale.y = 0.1;
		marker.scale.z = 0.1;
		marker.color.a = 1.0; // Don't forget to set the alpha!
		marker.color.r = 1.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;

		vis_pub.publish( marker );
		std::cout << "Marker published: " << std::endl;
		r.sleep();*/
	
		//------------ THIRD TRY ----------------- //

	//direction = octomap::point3d(2.38,-0.14,0.67);
    direction = octomap::point3d(cam_dist,ycenter_sens_m, 0.98*zcenter_sens_m); //voxel of x,y,z
	std::cout << "3rd try casting" << std::endl;    
//std::cout << "3rd try: casting ray from " << origin  << " in the " << direction << " direction"<< std::endl;
    success = tree->castRay(origin, direction, ray_end);  

    if(success){
      std::cout << "ray hit cell with center " << ray_end << std::endl;

      octomap::point3d intersection;
      success = tree->getRayIntersection(origin, direction, ray_end, intersection);
      if(success)
        std::cout << "entrance point is " << intersection << std::endl;
    }
		/*marker.header.frame_id = "map";
		marker.header.stamp = ros::Time();
		marker.ns = "my_namespace";
		marker.id = 0;
		marker.type = visualization_msgs::Marker::ARROW;
		marker.action = visualization_msgs::Marker::ADD;
		
		p.x=-0.62;
		p.y=-0.14;
		p.z= 0.69;
		marker.points.push_back(p);
		/*p.x=cam_dist*10;
		p.y=ycenter_sens_m*10;
		p.z=zcenter_sens_m*10;*
		p.x=cam_dist*50;
		p.y=ycenter_sens_m*50;
		p.z= 0.98*zcenter_sens_m*50;
		marker.points.push_back(p);
		marker.scale.x = 0.1;
		marker.scale.y = 0.1;
		marker.scale.z = 0.1;
		marker.color.a = 1.0; // Don't forget to set the alpha!
		marker.color.r = 1.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;

		vis_pub.publish( marker );
		std::cout << "Marker published: " << std::endl;
		r.sleep();*/
	
	  
	//  ROS_INFO_STREAM("Time stamp"<< msg);
	 
  	
found_person=false;
}

}


	//----------- Callback from Yolo--------------//
	void yoloCallback(const darknet_ros_msgs::BoundingBoxes& msg)
{
	//std::cout << "msg from darknet " << msg.bounding_boxes[0] << std::endl;
	//save x/y min/max
	xmin=msg.bounding_boxes[0].xmin;
	xmax=msg.bounding_boxes[0].xmax;
	ymin=msg.bounding_boxes[0].ymin;
	ymax=msg.bounding_boxes[0].ymax;
		/*std::cout << "x min: " << msg.bounding_boxes[0].xmin << std::endl;
		std::cout << "x max: " << msg.bounding_boxes[0].xmax << std::endl;
		std::cout << "y min:  " << msg.bounding_boxes[0].ymin << std::endl;
		std::cout << "y max:  " << msg.bounding_boxes[0].ymax << std::endl;*/
	
	//calculate center of detected object in pixel
	xcenter= xmin + (xmax-xmin)/2;
	ycenter= ymin + (ymax-ymin)/2;
		//std::cout << "center of boundingbox x: [px] " << xcenter<< std::endl;
		//std::cout << "center of boundingbox y: [px]  " << ycenter << std::endl;
		
	//calculate center of detected object in pixel
	xcenter_m=xcenter*psize;
	ycenter_m=ycenter*psize;
	//	std::cout << "center of boundingbox in sensor x [m]:  " << xcenter_m << std::endl;
	//	std::cout << "center of boundingbox in sensor y [m]:  " << ycenter_m << std::endl;
		
	//transform distance of boundingbox's center to distance of sensor's center
	ycenter_sens=xres/2-xcenter;
	zcenter_sens=yres/2-ycenter;
	//	std::cout << "distance from sensor's center to center of Boundingbox x [px]:  " << ycenter_sens << std::endl;
	//	std::cout << "distance from sensor's center to center of boundingbox y [px]:  " << zcenter_sens << std::endl;
		
	//calculate center of detected object in pixel
	ycenter_sens_m=ycenter_sens*psize;
	zcenter_sens_m=zcenter_sens*psize;
		//std::cout << "distance between center of bounding box and center of sensor x [m]:  " << ycenter_sens_m << std::endl;
		//std::cout << "distance between center of bounding box and center of sensor y [m]:  " << zcenter_sens_m << std::endl;

	found_person = true;
			
}

int main(int argc, char **argv)
{	//subscriber to octomap
   ros::init(argc, argv, "subscriber_octo");
	
	
	ros::NodeHandle n("~");
	ros::Rate r(1);
	while (ros::ok()){
	ROS_INFO("spin_start");
	//subscribe to boundingboxes of yolo

	ros::Subscriber yolo_boxes = n.subscribe("/darknet_ros/bounding_boxes", 1, yoloCallback);
		
	
	//subscribe to octomap_full
	ros::Subscriber sub_octomap = n.subscribe("/octomap_binary", 1, octoCallback);

	vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
		
	
	
	
		
	



	r.sleep();
	ROS_INFO("spin_end");
	ros::spinOnce();
	}


	
	
	

  return 0;
}
