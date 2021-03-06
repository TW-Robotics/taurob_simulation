#include "follow_human/follow_human.hpp"
#include <boost/algorithm/string.hpp>
#include <ros/exception.h>

follow_human::follow_human():
  n("~"),
  sub_yolo(n.subscribe("/darknet_ros/bounding_boxes", 1, &follow_human::callback_yolo, this)),
  sub_lidar(n.subscribe("/scan_front", 1, &follow_human::callback_lidar, this)),
  sub_lidar_rear(n.subscribe("/scan_rear", 1, &follow_human::callback_lidar_rear, this)),
  sub_img(n.subscribe("/camera_rgb/image_raw", 1, &follow_human::callback_img, this)),
  sub_pid(n.subscribe("/control_effort", 1, &follow_human::callback_pid, this)),
  pub_cmd_vel(n.advertise<geometry_msgs::Twist>("/cmd_vel", 1)),
  pub_state_pid(n.advertise<std_msgs::Float64>("/state", 1)),
  pub_setpoint_pid(n.advertise<std_msgs::Float64>("/setpoint", 1)),
  f(boost::bind(&follow_human::callback_reconf, this, _1, _2)),
  use_pid(false)
{
  server.setCallback(f);
  n.param<int>("/setpoint_height", setpoint_height, 400);
  n.param<float>("/min_probability", min_probability, 0.5);
  n.param<float>("/min_range_lidar", min_range_lidar, 0.4);
  n.param<double>("/kp_lin", kp_lin, 0.168125);
  n.param<double>("/kp", kp, 0.00038125);
  use_pid = true;
  image_width = NULL;
  image_height = NULL;
  ROS_INFO("Init node %s complete", ros::this_node::getName().c_str());


  rot_vel.data = 0.0;
  velocity.linear.x = 0.0;
  velocity.angular.z = 0.0;
  pub_cmd_vel.publish(velocity);
}

void follow_human::callback_yolo(const darknet_ros_msgs::BoundingBoxes &bboxes)
{
  int xmin = 0, xmax = 0, ymin = 0, ymax = 0;
  if (n.hasParam("/min_probability"))
  {
    ros::param::get("/min_probability", min_probability);
  }
  for (int i = 0; i < bboxes.bounding_boxes.size(); i++)
  {
    float prob = bboxes.bounding_boxes[i].probability;
    if (prob >= min_probability)
    {
      xmin = bboxes.bounding_boxes[i].xmin;
      xmax = bboxes.bounding_boxes[i].xmax;
      ymin = bboxes.bounding_boxes[i].ymin;
      ymax = bboxes.bounding_boxes[i].ymax;

      //calculate width and heigth of bbx
      bbox_width = xmax - xmin;
      bbox_height = ymax - ymin;
      //calculate center of detected object in pixel
      human_pos_x = xmin + (xmax - xmin) / 2;
      human_pos_y = ymin + (ymax - ymin) / 2;
      if (boost::iequals(bboxes.bounding_boxes[i].Class, "person"))
      {
        ROS_DEBUG_THROTTLE(5, "BBox \n   height: %i\n   BBox width: %i", bbox_height, bbox_width);
        if (!flag_lidar)
        {
          if (use_pid == true)
          {
            pub_pid();
          }
          else
          {
            pub_p();
          }
          ROS_INFO_THROTTLE(3, "Publishing linear velocity (%f) and angular velocity (%f)", velocity.linear.x, velocity.angular.z);
        }
        else
        {
          ROS_WARN_THROTTLE(1, "Found obstacle in the way, can not follow");
          velocity.linear.x = 0.0;
          velocity.angular.z = 0.0;
          pub_cmd_vel.publish(velocity);
        }
      }
      else  ROS_WARN_THROTTLE(3, "Couldn't find correct class: %s", bboxes.bounding_boxes[i].Class.c_str());
    }
    else ROS_WARN_THROTTLE(5, "Probabilty of object %s not high enough %f <= %f ! ", bboxes.bounding_boxes[i].Class.c_str(), prob, min_probability);
  }
}

void follow_human::callback_lidar(const sensor_msgs::LaserScan &scan)
{
  if (velocity.linear.x >= 0)      // only use this when going forward
  {
    if (n.hasParam("/min_range_lidar"))
    {
      ros::param::get("/min_range_lidar", min_range_lidar);
    }
    auto min = std::min_element(scan.ranges.begin(), scan.ranges.end());
    ROS_DEBUG_THROTTLE(5, "Min LIDAR range= %f", *min);
    if (*min <= min_range_lidar) flag_lidar = true;
    else flag_lidar = false;

    float phi = scan.angle_min;
    std::vector<float> temp;
    for (const float r : scan.ranges)
    {
      if (r < min_range_lidar)
      {
        temp.push_back(r);
        phi += scan.angle_increment;
        ranges.push_back(temp);
        temp.clear();
      }
      else
      {
        phi += scan.angle_increment;
      }
    }
    ranges.clear();
  }
}

void follow_human::callback_lidar_rear(const sensor_msgs::LaserScan &scan)
{
  if (velocity.linear.x <= 0)      // only use this when going backward
  {
    if (n.hasParam("/min_range_lidar"))
    {
      ros::param::get("/min_range_lidar", min_range_lidar);
    }
    auto min = std::min_element(scan.ranges.begin(), scan.ranges.end());
    ROS_DEBUG_THROTTLE(0.2, "Min LIDAR range= %f", *min);
    if (*min <= min_range_lidar+0.2) flag_lidar = true;
    else flag_lidar = false;

    float phi = scan.angle_min;
    std::vector<float> temp;
    for (const float r : scan.ranges)
    {
      if (r < min_range_lidar)
      {
        temp.push_back(r);
        phi += scan.angle_increment;
        ranges_rear.push_back(temp);
        temp.clear();
      }
      else
      {
        phi += scan.angle_increment;
      }
    }
    ranges_rear.clear();
  }
}

void follow_human::callback_img(const sensor_msgs::ImageConstPtr& msg)
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
  image = cv_ptr->image;
  image_width = image.size().width;
  image_height = image.size().height;
  ROS_INFO_STREAM("Image width= " << image_width << "\nImage height= " << image_height << std::endl);
  sub_img.shutdown();
}

void follow_human::callback_pid(const std_msgs::Float64 &msg)
{
  ROS_INFO_STREAM_THROTTLE(2, "Control of PID= "<< msg.data<<"\nfabs: "<<fabs(msg.data));
  velocity.linear.x = get_lin_vel()*kp_lin;
  if(fabs(msg.data)<0.036) velocity.angular.z = 0.0;
  else velocity.angular.z = msg.data*0.4;

  ROS_INFO_STREAM_THROTTLE(2,"PID Control publishes: \n\tlin x:"<<velocity.linear.x <<"\n\tangular z:"<<velocity.angular.z);  
  if(!flag_lidar) pub_cmd_vel.publish(velocity);
}



void follow_human::pub_pid(void)
{
  if (image_width && image_height)
  {
    setpoint.data = 0; //image_width / 2;
    state.data = (2.0 - (-2.0)) * (human_pos_x - 0.0)/ (image_width - 0.0 ) - 2.0;  //scale human_pose_x between [-2 2]
    ROS_DEBUG_STREAM_THROTTLE(1, "State=  "<<state.data <<"\n\thuman_pos_x: "<<human_pos_x);
    pub_setpoint_pid.publish(setpoint);
    pub_state_pid.publish(state);
  }
}

void follow_human::pub_p(void)
{
  velocity.linear.x = get_lin_vel() * kp_lin;
  velocity.angular.z = (image_width / 2 - human_pos_x) * kp;
  ROS_INFO_STREAM_THROTTLE(2,"P Control publishes: \n\tlin x:"<<velocity.linear.x <<"\n\tangular z:"<<velocity.angular.z);
  if(!flag_lidar) pub_cmd_vel.publish(velocity);
}


float follow_human::get_lin_vel(void)
{
  float lin_vel = 0.0;
  lin_vel = ((static_cast<float>(bbox_height) - 770.0) / (450.0 - 770.0)); // Normalize lin_vel so that if distance is between 430 (top speed) and 800 (0 speed).
  if (std::abs(lin_vel) > 1)
  {
    if (lin_vel < 1) lin_vel = -1.0;
    else lin_vel = 1.0;
  }

  return lin_vel;
}


void follow_human::callback_reconf(follow_human_cfg::follow_human_mainConfig &config, uint32_t)
{
  setpoint_height = config.setpoint_height;
  min_probability = config.min_probability;
  min_range_lidar= config.min_range_lidar;
  kp_lin = config.kp_lin;
  kp = config.kp;
  image_topic = config.image_topic;
  use_pid = config.use_pid;
  ROS_DEBUG_STREAM("Reconfig called");

}
