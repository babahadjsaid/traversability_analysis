#pragma once

//General
#include <vector>
#include <sstream>
#include <string>
#include <array>

// ROS
#include <rclcpp/rclcpp.hpp>

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// Grid Map
#include <grid_map_ros/grid_map_ros.hpp>

//message types definition
#include <nav2_msgs/msg/costmap.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>


// Boost
#include <boost/thread/recursive_mutex.hpp>

typedef pcl::PointXYZINormal PointType;

class ParamServer : public rclcpp::Node
{
public:
    std::string PC_TOPIC,CM_TOPIC,MAP_FRAME;
    float RADIUS, CELL_RESOLUTION,T_DIFF,T_HIGH,T_LOW,MAX_HEIGHT;
   
    ParamServer(std::string node_name, const rclcpp::NodeOptions & options) : Node(node_name, options)
    {
        declare_parameter("pointCloudTopic", "/points");
        get_parameter("pointCloudTopic", PC_TOPIC);

        declare_parameter("costMapTopic", "map");
        get_parameter("costMapTopic", CM_TOPIC);

        declare_parameter("map_frame_id", "map");
        get_parameter("map_frame_id", MAP_FRAME);

        declare_parameter("circle_radius", 20.0);
        get_parameter("circle_radius", RADIUS);

        declare_parameter("cell_resolution", 0.2);
        get_parameter("cell_resolution", CELL_RESOLUTION);

        declare_parameter("T_diff", 0.05);
        get_parameter("T_diff", T_DIFF);

        declare_parameter("T_high", 0.05);
        get_parameter("T_high", T_HIGH);

        declare_parameter("T_low", 0.05);
        get_parameter("T_low", T_LOW);

        declare_parameter("max_height", 1.5);
        get_parameter("max_height", MAX_HEIGHT);

        
  
        usleep(100);
    }

  
};

