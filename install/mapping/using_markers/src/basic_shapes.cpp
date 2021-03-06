#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <unistd.h>
#include <algorithm>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "basic_shapes");
	ros::NodeHandle n;
	ros::Rate ra(1);
	ros::Duration duration(5);
	std::string path = ros::package::getPath("gcount").append("/gauss_out.csv");
	ROS_DEBUG_STREAM("Path to folder= " << path);
	ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 0);

	std::ifstream file1("/home/workstation/catkin_ws/src/src/gcount/gauss_out.csv");
	;
	std::vector<double> x_values;
	std::vector<double> y_values;
	std::vector<double> counts_values;
	std::vector<double> counts_norm;
	std::string tmp1, tmp2, tmp3, tmp4;
	double fx = 0.0, fy = 0.0, fc = 0.0, fcn = 0.0;
	int anz = 0;
	double r = 0.0, g = 0.0, b = 0.0;
	//double raster_size=0.052;
	double raster_size = 0.12;

	//read csv

	if (file1.good())
	{
		ROS_INFO_STREAM("Opening file " << path << " successful");
	}
	else
	{
		ROS_ERROR_STREAM("Error opening file: " << path);
		return -1;
	}

	//read line by line and split them
	while (file1.good())
	{
		getline(file1, tmp1, '\n');
		std::stringstream split(tmp1);
		getline(split, tmp1, ',');
		getline(split, tmp2, ',');
		getline(split, tmp3, ',');
		getline(split, tmp4);

		//save single values
		if (!tmp1.empty())
		{

			std::stringstream ssx(tmp1);
			ssx >> fx;
			x_values.push_back(fx);
			std::stringstream ssy(tmp2);
			ssy >> fy;
			y_values.push_back(fy);
			std::stringstream ssc(tmp3);
			ssc >> fc;
			counts_values.push_back(fc);
			std::stringstream sscn(tmp4);
			sscn >> fcn;
			counts_norm.push_back(fcn);
			anz++;
		}
	}

	//get max value of counts_norm
	double max_counts = *std::max_element(counts_norm.begin(), counts_norm.end());
	std::cout << "Max value: " << max_counts << std::endl;
	//get min value of counts_norm
	double min_counts = *std::min_element(counts_norm.begin(), counts_norm.end());
	std::cout << "Min value: " << min_counts << std::endl;

	//visualize heatmap
	visualization_msgs::MarkerArray Markerarr;
	Markerarr.markers.resize(anz + 15);
	int i = 0;
	for (i = 0; i < anz; i++)
	{
		Markerarr.markers[i].header.frame_id = "maps";
		Markerarr.markers[i].header.stamp = ros::Time::now();
		Markerarr.markers[i].ns = "basic_shapes";
		Markerarr.markers[i].id = i;
		Markerarr.markers[i].action = visualization_msgs::Marker::ADD;
		Markerarr.markers[i].type = visualization_msgs::Marker::CUBE;
		Markerarr.markers[i].pose.position.x = x_values[i];
		Markerarr.markers[i].pose.position.y = y_values[i];
		Markerarr.markers[i].pose.position.z = 0;
		Markerarr.markers[i].pose.orientation.x = 0.0;
		Markerarr.markers[i].pose.orientation.y = 0.0;
		Markerarr.markers[i].pose.orientation.z = 0.0;
		Markerarr.markers[i].pose.orientation.w = 1.0;

		if (counts_norm[i] < 0.25)
		{
			b = 1;
			r = 0;
			g = 4 * counts_norm[i];
			ROS_DEBUG_STREAM("Farbe <0.25 b: " << b << " g: " << g << " r: " << r);
		}
		else if (counts_norm[i] < 0.5)
		{
			b = 1 - (4 * (counts_norm[i] - 0.25));
			r = 0;
			g = 1;
			ROS_DEBUG_STREAM("Farbe <0.50 b " << b << " g: " << g << " r: " << r);
		}
		else if (counts_norm[i] < 0.75)
		{
			b = 0;
			r = 4 * (counts_norm[i] - 0.5);
			g = 1;
			ROS_DEBUG_STREAM("Farbe <0.75 b " << b << " g: " << g << " r: " << r);
		}
		else
		{
			b = 0;
			r = 1;
			g = 1 - (4 * (counts_norm[i] - 0.75));
			ROS_DEBUG_STREAM("Farbe <1.00 b " << b << " g: " << g << " r: " << r);
		}
		Markerarr.markers[i].scale.x = raster_size;
		Markerarr.markers[i].scale.y = raster_size;
		Markerarr.markers[i].scale.z = 0.05;
		Markerarr.markers[i].color.a = 0.25;
		Markerarr.markers[i].color.r = r;
		Markerarr.markers[i].color.g = g;
		Markerarr.markers[i].color.b = b;
		//r.sleep();
		usleep(100);
		ROS_DEBUG_STREAM("nr: " << i);
		ROS_DEBUG_STREAM("X: " << x_values[i] << "Y: " << y_values[i] << "C: " << counts_values[i] << "Cn: " << counts_norm[i]);
		ROS_DEBUG_STREAM("Number of markers" << i);
	}

	//get max value of x
	double min_x = *std::min_element(x_values.begin(), x_values.end());
	std::cout << "Min value x: " << min_x << std::endl;
	double max_x = *std::max_element(x_values.begin(), x_values.end());
	std::cout << "Max value x: " << max_x << std::endl;
	//get min value of y
	double min_y = *std::min_element(y_values.begin(), y_values.end());
	std::cout << "Min value y: " << min_y << std::endl;
	double max_y = *std::max_element(y_values.begin(), y_values.end());
	std::cout << "Max value y: " << max_y << std::endl;

	//get max distance in x
	double distance_x;

	if (min_x > 0 && max_x > 0)
	{
		distance_x = max_x - min_x;
		std::cout << "Distance x pos pos: " << distance_x << std::endl;
	}
	else if (min_x < 0 && max_x < 0)
	{
		distance_x = min_x - max_x;
		std::cout << "Distance x neg neg: " << distance_x << std::endl;
	}
	else if (min_x > 0 && max_x < 0)
	{
		distance_x = max_x - min_x;
		std::cout << "Distance x pos neg: " << distance_x << std::endl;
	}
	else if (min_x < 0 && max_x > 0)
	{
		distance_x = abs(min_x - max_x);
		std::cout << "Distance x neg pos: " << distance_x << std::endl;
	}

	//get distances in y
	double distance_y;

	if (min_y > 0 && max_y > 0)
	{
		distance_y = max_y - min_y;
		std::cout << "Distance y pos pos: " << distance_y << std::endl;
	}
	else if (min_y < 0 && max_y < 0)
	{
		distance_y = min_y - max_x;
		std::cout << "Distance y neg neg: " << distance_y << std::endl;
	}
	else if (min_y > 0 && max_y < 0)
	{
		distance_y = max_x - min_y;
		std::cout << "Distance y pos neg: " << distance_y << std::endl;
	}
	else if (min_y < 0 && max_y > 0)
	{
		distance_y = abs(min_y - max_y);
		std::cout << "Distance y neg pos: " << distance_y << std::endl;
	}

	//get max distance
	double max_dist = std::max(distance_x, distance_y);
	ROS_DEBUG_STREAM("Max distance: " << max_dist);
	ROS_DEBUG_STREAM("Max distance 1/20: " << max_dist / 20);

	//varible to run through color values in 10 steps
	//counts_col/0.1 = index of colorbar array
	float counts_col = 0.0;

	for (; i < anz + 13;)
	{

		//colorbar
		if (counts_col <= 1.05)
		{
			Markerarr.markers[i].header.frame_id = "maps";
			Markerarr.markers[i].header.stamp = ros::Time::now();
			Markerarr.markers[i].ns = "basic_shapes";
			Markerarr.markers[i].id = i;
			Markerarr.markers[i].action = visualization_msgs::Marker::ADD;
			Markerarr.markers[i].type = visualization_msgs::Marker::CUBE;

			//std::cout << "x: " << x_values[i] << "Y: " << y_values[i] << std::endl;
			Markerarr.markers[i].pose.position.x = min_x - 1;
			Markerarr.markers[i].pose.position.y = min_y + 3 - max_dist / 20 * counts_col / 0.1;
			Markerarr.markers[i].pose.position.z = 0;
			Markerarr.markers[i].pose.orientation.x = 0.0;
			Markerarr.markers[i].pose.orientation.y = 0.0;
			Markerarr.markers[i].pose.orientation.z = 0.0;
			Markerarr.markers[i].pose.orientation.w = 1.0;

			std::cout << "color: " << counts_col << std::endl;

			if (counts_col < 0.25)
			{
				b = 1;
				r = 0;
				g = 4 * counts_col;
				//std::cout << "Farbe <0.25 b: " << b << " g: " << g << " r: " << r << std::endl;
			}

			else if (counts_col < 0.5)
			{
				b = 1 - (4 * (counts_col - 0.25));
				r = 0;
				g = 1;
				//std::cout << "Farbe <0.50 b "  << b << " g: " << g << " r: "<< r << std::endl;
			}
			else if (counts_col < 0.75)
			{
				b = 0;
				r = 4 * (counts_col - 0.5);
				g = 1;
				//std::cout << "Farbe <0.75 b "  << b << " g: " << g << " r: " <<r << std::endl;
			}
			else
			{
				b = 0;
				r = 1;
				g = 1 - (4 * (counts_col - 0.75));
				//std::cout << "Farbe <1.00 b "  << b << " g: " << g << " r: " <<r << std::endl;
			}

			Markerarr.markers[i].scale.x = max_dist / 20;
			Markerarr.markers[i].scale.y = max_dist / 20;
			Markerarr.markers[i].scale.z = 0.05;
			Markerarr.markers[i].color.a = 1;
			Markerarr.markers[i].color.r = r;
			Markerarr.markers[i].color.g = g;
			Markerarr.markers[i].color.b = b;
			// }

			//r.sleep();
			usleep(100000);
			std::cout << "Color bar: " << i << std::endl;
			//std::cout << "X: " << x_values[i] << "Y: " << y_values[i] << "C: " << counts_values[i] << "Cn: " << counts_norm[i]<<std::endl;
			//std::cout << "Number of markers" << i << std::endl;
			counts_col = counts_col + 0.1;
			i++;
		}

		else
		{
			counts_col = 1;

			//get max value of counts_norm
			double max_value = *std::max_element(counts_values.begin(), counts_values.end());
			std::cout << "Max value: " << max_value << std::endl;
			//get min value of counts_norm
			double min_value = *std::min_element(counts_values.begin(), counts_values.end());
			std::cout << "Min value: " << min_value << std::endl;

			//convert max_norm and min_norm to strings
			std::ostringstream ss_max;
			ss_max << max_value;
			std::string s_max(ss_max.str());
			std::cout << "Max counts: " << s_max << std::endl;

			std::ostringstream ss_min;
			ss_min << min_value;
			std::string s_min(ss_min.str());
			std::cout << "Max counts: " << s_min << std::endl;

			//legend for colorbar
			//label max value
			Markerarr.markers[i].header.frame_id = "maps";
			Markerarr.markers[i].header.stamp = ros::Time::now();
			Markerarr.markers[i].ns = "basic_shapes";
			Markerarr.markers[i].id = i;
			Markerarr.markers[i].action = visualization_msgs::Marker::ADD;
			Markerarr.markers[i].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
			Markerarr.markers[i].text = s_max + " [cps]";

			//std::cout << "x: " << x_values[i] << "Y: " << y_values[i] << std::endl;
			Markerarr.markers[i].pose.position.x = min_x - 2.5;
			Markerarr.markers[i].pose.position.y = min_y + 3 - max_dist / 20 * counts_col / 0.1;
			Markerarr.markers[i].pose.position.z = 0;
			Markerarr.markers[i].pose.orientation.x = 0.0;
			Markerarr.markers[i].pose.orientation.y = 0.0;
			Markerarr.markers[i].pose.orientation.z = 0.0;
			Markerarr.markers[i].pose.orientation.w = 1.0;

			Markerarr.markers[i].scale.x = 1;
			Markerarr.markers[i].scale.y = 1;
			Markerarr.markers[i].scale.z = max_dist / 20;
			Markerarr.markers[i].color.a = 1;
			Markerarr.markers[i].color.r = 1;
			Markerarr.markers[i].color.g = 1;
			Markerarr.markers[i].color.b = 1;

			i++;
			counts_col = 0;

			//label min value
			Markerarr.markers[i].header.frame_id = "maps";
			Markerarr.markers[i].header.stamp = ros::Time::now();
			Markerarr.markers[i].ns = "basic_shapes";
			Markerarr.markers[i].id = i;
			Markerarr.markers[i].action = visualization_msgs::Marker::ADD;
			Markerarr.markers[i].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
			Markerarr.markers[i].text = "0 [cps]";

			//std::cout << "x: " << x_values[i] << "Y: " << y_values[i] << std::endl;
			Markerarr.markers[i].pose.position.x = min_x - 2.5;
			Markerarr.markers[i].pose.position.y = min_y + 3 - max_dist / 20 * counts_col / 0.1;
			Markerarr.markers[i].pose.position.z = 0;
			Markerarr.markers[i].pose.orientation.x = 0.0;
			Markerarr.markers[i].pose.orientation.y = 0.0;
			Markerarr.markers[i].pose.orientation.z = 0.0;
			Markerarr.markers[i].pose.orientation.w = 1.0;

			Markerarr.markers[i].scale.x = 1;
			Markerarr.markers[i].scale.y = 1;
			Markerarr.markers[i].scale.z = max_dist / 20;
			Markerarr.markers[i].color.a = 1;
			Markerarr.markers[i].color.r = 1;
			Markerarr.markers[i].color.g = 1;
			Markerarr.markers[i].color.b = 1;
			// }

			//r.sleep();
			usleep(100000);
			std::cout << "Legend for colorbar: " << i << std::endl;
			i++;
		}
	}

	marker_pub.publish(Markerarr);

	//open csv
	// std::ifstream file1("./gauss_out.csv");
	//std::ifstream file1("/home/workstation/catkin_ws/src/src/gcount/gauss_out.csv");
	//open csv
	// std::ifstream file1("./gauss_out.csv");
	//std::ifstream file1("/home/workstation/catkin_ws/src/src/gcount/gauss_out.csv");
	//open csv
	// std::ifstream file1("./gauss_out.csv");
	//std::ifstream file1("/home/workstation/catkin_ws/src/src/gcount/gauss_out.csv");
	//  std::co	//open csv
	// std::ifstream file1("./gauss_out.csv");
	//std::ifstream file1("/home/workstation/catkin_ws/src/src/gcount/gauss_out.csv");   ut << " Value of array: " <<  Markerarr.markers[i].scale.x << std::endl;

	std::cout << "published" << std::endl;
}
