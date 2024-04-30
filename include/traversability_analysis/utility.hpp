#pragma once

// General (std)
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <array>
#include <random>

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
#include <Eigen/Eigenvalues>

// Boost
#include <boost/math/special_functions.hpp>
// #include <boost/thread/recursive_mutex.hpp>

typedef pcl::PointXYZI PointType;

class ParamServer : public rclcpp::Node
{
public:
    std::string PC_TOPIC,CM_TOPIC,MAP_FRAME;
    float RADIUS, CELL_RESOLUTION,T_DIFF,T_HIGH,T_LOW,
    MAX_HEIGHT, MEAN_GRASS, VARIANCE_GRASS, T_PROB, UB, LB, T_RATIO, T_L, T_S;
    int NUM_GRIDS_MIN;
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

        declare_parameter("mean_grass", 0.0);
        get_parameter("mean_grass", MEAN_GRASS);

        declare_parameter("variance_grass", 0.0);
        get_parameter("variance_grass", VARIANCE_GRASS);

        declare_parameter("T_prob", 1.0E-7);
        get_parameter("T_prob", T_PROB);

        UB = MEAN_GRASS + sqrt(VARIANCE_GRASS) * sqrt(2) * boost::math::erfc_inv(2 * T_PROB);

        // Compute the lower bound
        LB = MEAN_GRASS - (UB - MEAN_GRASS);

        declare_parameter("num_grids_min", 9);
        get_parameter("num_grids_min", NUM_GRIDS_MIN);

        declare_parameter("T_ratio", 1.7);
        get_parameter("T_ratio", T_RATIO);

        declare_parameter("T_l", 1.7);
        get_parameter("T_l", T_L);
        
        declare_parameter("T_s", 1.7);
        get_parameter("T_s", T_S);

        usleep(100);
    }

  
};




#ifndef NOUTILITY
float PointToPlaneDistance(const PointType& point, float A, float B, float C, float D) {
    return fabs(A * point.x + B * point.y + C * point.z + D) / sqrt(A * A + B * B + C * C);
}


#endif


