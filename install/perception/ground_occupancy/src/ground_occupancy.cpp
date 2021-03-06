#include <boost/foreach.hpp>
#include <iostream>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Header.h>
#include <nav_msgs/MapMetaData.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/features/normal_3d.h>
#include <sensor_msgs/Imu.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

/**
 * @brief class to convert a 3D point cloud into a 2D occupancy grid map. Based on 3D LiDAR data, the class is able to sort points into a 2D grid and evaluate each cell by performing a PCA on its containing points.
 *
 * 
 * 
 */
class Cloud2DMap
{

  //container for cloud data     
  PointCloud cloud;

  //Cell grid for Map representation
  std::vector<std::vector<pcl::PointXYZ*> > grid_map;
  std::vector<signed char> data;
    

  //robot frame representation in Cell array
  int robot_frame_cord_x;
  int robot_frame_cord_y;
  
  //max traversabel slope (alpha_max): taurob tracker alpha_max = 40°
  float max_angle;
  
  //robot pose in quaternions in reference to global axis
  Eigen::Vector3f global_z;
  Eigen::Vector3f rotated_z;

  public:
    //coordinate transform listener
    tf::TransformListener listener;
    
    nav_msgs::OccupancyGrid* occ_grid;

    //params which are later imported from parameter server
    double MAX_ANGLE = 35;      //in degrees
    double GRID_RES = 0.4;      //in meters per cell
    double MAP_WIDTH = 16;      //in meters
    double MAP_HEIGHT = 16;     //in meters
    std::string FROM_FRAME = "/velodyne"; 
    std::string TO_FRAME = "/imu_link"; 

    /**
     * @brief subscriber callback function of 3D lidar Sensor. After receiving a message, the function transforms the points into the robot frame and calls member functions for cell occupancy classification.
     * 
     * @param msg massage from which the 3D lidar data is received by the node
     */
    void lidarSensorMsgCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        // Convert to PCL data type
        pcl::fromROSMsg (*msg, cloud);

        //transform point cloud
        pcl_ros::transformPointCloud("/imu_link", cloud, cloud , listener); 

        //sort points
        sortPoints(cloud,grid_map);

        //Estimate plane and determine cell occupancy
        cellOccpancy(grid_map, data, rotated_z);        

        //update occ Grid
        updateOccGrid(occ_grid, data);
    }

    /**
     * @brief subscriber callback function of Imu sensor that stores a rotation quaternion. The 
     * 
     * @param msg massage from which the imu data is received by the node
     */
    void imuSensorMsgCallback(const sensor_msgs::Imu::ConstPtr& msg)
    {
        tf2::Quaternion imu_orientation;
        //convert imu msg to tf2:quaternion
        imu_orientation.setW(msg->orientation.w);
        imu_orientation.setX(msg->orientation.x);
        imu_orientation.setY(msg->orientation.y);
        imu_orientation.setZ(msg->orientation.z);

        //rotate global z_axis
        rotate_vector_by_quaternion(global_z, imu_orientation, rotated_z);
    }

    /**
     * @brief perform a unit quaternion rotation on quaternion v 
     * 
     * @param v the quaternion thats rotated
     * @param r the roation quaternion that defines the rotation
     * @param vprime the resulting vector after the rotation
     */
    void rotate_vector_by_quaternion(const Eigen::Vector3f& v, const tf2::Quaternion& r, Eigen::Vector3f& vprime)
    {   //invert quaternion
        tf2::Quaternion q = r.inverse();

        // Extract the vector part of the quaternion
        Eigen::Vector3f u(q.x(), q.y(), q.z());

        // Extract the scalar part of the quaternion
        float s = q.w();

        //some helper variables for member function calculations
        float u_v = u.dot(v);
        float u_u = u.dot(u);
        Eigen::Vector3f uxv = u.cross(v);

        // Do the math
        vprime = 2.0f * u_v * u
            + (s*s - u_u) * v
            + 2.0f * s * uxv;
    }

    /**
     * @brief initialize occupancy grid massage which is published by the node
     * 
     * @param occ_grid_ pointer of the occupancy grid that is initialized
     */
    void initOccGrid(nav_msgs::OccupancyGrid*& occ_grid_){
        //set header parameters
        occ_grid_->header.seq = 1;
        occ_grid_->header.frame_id = TO_FRAME;

        //set info parameters
        occ_grid_->info.origin.position.x = -robot_frame_cord_x*GRID_RES;
        occ_grid_->info.origin.position.y = -robot_frame_cord_y*GRID_RES;
        occ_grid_->info.origin.position.z = 0;
        occ_grid_->info.origin.orientation.w = 0;
        occ_grid_->info.origin.orientation.x = 0.707;
        occ_grid_->info.origin.orientation.y = 0.707;
        occ_grid_->info.origin.orientation.z = 0;
        occ_grid_->info.resolution = GRID_RES;
        occ_grid_->info.width = MAP_WIDTH/GRID_RES;
        occ_grid_->info.height = MAP_HEIGHT/GRID_RES;
    }

    /**
     * @brief update occupancy grid message with new header, info and data parameters
     * 
     * @param occ_grid_ pointer of occupcancy grid message
     * @param data_ the evaluated grid data that shall be published by the node
     */
    void updateOccGrid(nav_msgs::OccupancyGrid*& occ_grid_, std::vector<signed char>& data_){
        //update header/info and data parameters of occupancy message
        occ_grid->header.seq++;
        occ_grid->header.stamp.sec = ros::Time::now().sec;
        occ_grid->header.stamp.nsec = ros::Time::now().nsec;
        occ_grid->info.map_load_time = ros::Time::now();
        occ_grid->data = data_;

        //clear grid and init with map size
        grid_map.clear();
        grid_map.resize(MAP_WIDTH/GRID_RES * MAP_HEIGHT/GRID_RES);

        //clear data and init as unkown (-1)
        data.resize(MAP_WIDTH/GRID_RES * MAP_HEIGHT/GRID_RES, -1);
    }

    /**
     * @brief calculate cell indicis of a given point in a point cloud object.
     * 
     * @param cloud_t input point cloud
     * @param n index of the point within the point cloud whos cell index shall be calucated
     * @return std::tuple<int, int> the x,y indecies of the point 
     */
    std::tuple<int, int> calcIndex(PointCloud& cloud_t,int n){
        
        //calc Point index
        float i_x = robot_frame_cord_x + cloud_t.points[n].x / GRID_RES;
        float i_y = robot_frame_cord_y + cloud_t.points[n].y / GRID_RES;
        
        //round to nearest integer
        int x = (int)round(i_x);
        int y = (int)round(i_y);

        //return index
        return std::make_tuple(x,y);
    }

    /**
     * @brief Sort points of a point cloud into 2D grid along the robots x,y-plane, based on the points x,y-values.
     * 
     * @param cloud_ point cloud containing the point data
     * @param grid_ grid pointer in which the points shall be sorted
     */
    void sortPoints(PointCloud& cloud_,std::vector<std::vector<pcl::PointXYZ*>>& grid_)
    {   
        //init temp x,y index variables
        int x_index, y_index;

        //Sort points
        for(int n = 0; n < cloud_.size(); n++){
            //calc point indices 
            std::tie(x_index, y_index) = calcIndex(cloud_,n);

            //check if indecis are in grid boundaries
            if((x_index > 0 && y_index > 0) && (x_index < MAP_WIDTH/GRID_RES && y_index < MAP_HEIGHT/GRID_RES))
            {
                //store point adress in Grid                
                grid_[x_index*MAP_WIDTH/GRID_RES + y_index].push_back(&cloud_.points[n]);
            }                      
        }
    }

    /**
     * @brief evaluate occupancy grid and assigne data values to cells, based on slope angle and max. angle.
     * 
     * @param grid_ 2D grid containing cell vise point data that has to be evaluated
     * @param data_ vector that stores the evaluated cell data
     * @param z_rotated_ the global (parent) Z-vector
     */
    void cellOccpancy(std::vector <std::vector<pcl::PointXYZ*>>& grid_, std::vector<signed char>& data_, Eigen::Vector3f& z_rotated_)
    {
        
        //temp PointCloud object
        PointCloud temp;
        
        //container for plane parameters and curvature
        Eigen::Vector4f plane_parameters;
        float curvature;

        Eigen::Vector3f normal;
        float angle;
        float dot_normal;

        //iterate over grid
        for(int n=0; n < grid_.size(); n++)
        {
            if(grid_[n].size() > 3)
            {
                //load all cell points into temporary point cloud object
                for(std::vector<pcl::PointXYZ*>::iterator iter = grid_[n].begin(); iter != grid_[n].end(); iter++) 
                {
                    temp.points.push_back(**iter);
                }
                //estimate plane and compute normal vector
                pcl::computePointNormal(temp,plane_parameters,curvature);

                //extract normal
                normal << plane_parameters(0), plane_parameters(1), plane_parameters(2);

                //calc dot product of normal vector and global Z-axis
                dot_normal = std::abs(normal.dot(z_rotated_));

                //check for floating point errors
                dot_normal = dot_normal > 1.0 ? 1.0 : dot_normal;

                //calculate angle between normal vector and global Z-Axis
                angle = std::acos(dot_normal) * 180. / M_PI;

                //fill cell of occ grid with data
                if(angle < MAX_ANGLE)
                {
                    data_[n] = 0;
                }
                else
                {
                    data_[n] = 100;
                }
                //clear temp PC object                                
                temp.clear();
            }
            
        }
    }

    /**
     * @brief initialize some class members of the Cloud2Map object
     * 
     */
    void initCloud2Map()
    {
        //init global Z-Axis
        global_z << 0, 0, 1;
        //init frame coordinate coefficients at grid center
        robot_frame_cord_x = (int) ceil(MAP_WIDTH/(GRID_RES*2));
        robot_frame_cord_y = (int) ceil(MAP_HEIGHT/(GRID_RES*2));

        //init grid size
        grid_map.resize(MAP_WIDTH/GRID_RES * MAP_HEIGHT/GRID_RES);
        //init data grid as unkown (-1)
        data.resize(MAP_WIDTH/GRID_RES * MAP_HEIGHT/GRID_RES, -1);

        //init new OccGrid object
        occ_grid = new nav_msgs::OccupancyGrid();

        //init occ grid
        initOccGrid(occ_grid);        
    }
};

int main(int argc, char** argv)
{
    //ROS-Setup
    ros::init(argc, argv, "ground_occupancy");

    Cloud2DMap CloudMap;  
    ros::NodeHandle nh;
    ros::Subscriber sub_points = nh.subscribe("/velodyne_points", 1, &Cloud2DMap::lidarSensorMsgCallback, &CloudMap);
    ros::Subscriber sub_imu = nh.subscribe("/taurob_tracker/imu/data", 1, &Cloud2DMap::imuSensorMsgCallback, &CloudMap);
    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("occupancy_map",10);

    //define a capped publishing rate of the node in Hz
    ros::Rate loop_rate(10);
    try
    {
        CloudMap.listener.waitForTransform(CloudMap.TO_FRAME, CloudMap.FROM_FRAME,  ros::Time(0), ros::Duration(3.0));
    }
        catch(tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

    //set the default parameter values
    ros::param::set("GRID_RES", 0.4); //in meters per cell
    ros::param::set("MAP_WIDTH", 16); //in meters
    ros::param::set("MAP_HEIGHT", 16); //in meters
    ros::param::set("MAX_ANGLE", 35); //in degrees
    ros::param::set("FROM_FRAME", "/velodyne");
    ros::param::set("TO_FRAME", "/imu_link");
    
    //init new Cloud2Map object
    CloudMap.initCloud2Map();

    while(ros::ok())
    {
        //get parameters
        if(!nh.param("GRID_RES", CloudMap.GRID_RES, 0.4)){
            ROS_ERROR("Failed to get param GRID_RES");
        }
        if(!nh.param("MAP_WIDTH", CloudMap.MAP_WIDTH, 16.)){
            ROS_ERROR("Failed to get param MAP_WIDTH");
        }
        if(!nh.param("MAP_HEIGHT", CloudMap.MAP_HEIGHT, 16.)){
            ROS_ERROR("Failed to get param MAP_HEIGHT");
        }
        if(!nh.param("MAX_ANGLE", CloudMap.MAX_ANGLE, 35.)){
            ROS_ERROR("Failed to get param MAX_ANGLE");
        }
        if(!nh.param<std::string>("FROM_FRAME", CloudMap.FROM_FRAME, "/velodyne")){
            ROS_ERROR("Failed to get param FROM_FRAME");
        }
        if(!nh.param<std::string>("TO_FRAME", CloudMap.TO_FRAME, "/imu_link")){
            ROS_ERROR("Failed to get param TO_FRAME");
        }

        //Publish occupancy map
        map_pub.publish(*CloudMap.occ_grid);

        loop_rate.sleep();

        ros::spinOnce();    
    }

    return 0;    
}