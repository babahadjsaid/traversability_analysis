#pragma once

// General (std)
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <array>
#include <random>
#include <filesystem>



// ROS
#include <rclcpp/rclcpp.hpp>
#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
//message types definition
#include <nav_msgs/msg/odometry.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common.h>

// Grid Map
#include <grid_map_ros/grid_map_ros.hpp>




// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <Eigen/Dense>

// Boost
#include <boost/math/special_functions.hpp>
// #include <boost/thread/recursive_mutex.hpp>



#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>


typedef pcl::PointXYZI PointType;

class ParamServer : public rclcpp::Node
{
public:
    std::string PC_TOPIC,CM_TOPIC,MAP_FRAME,POSE_TOPIC;
    float RADIUS, CELL_RESOLUTION,T_DIFF,T_HIGH,T_LOW,
    MAX_HEIGHT, MEAN_GRASS, VARIANCE_GRASS, T_PROB, UB,
    LB, T_RATIO, T_L, T_S, T_NEG, T_POS, T_SEG, T_INLINERS, T_ITERATIONS, MAXANGLE;
    int NUM_GRIDS_MIN, NUM_COLORS, MAX_NUM_GRIDS;
    ParamServer(std::string node_name, const rclcpp::NodeOptions & options) : Node(node_name, options)
    {
        declare_parameter("pointCloudTopic", "/points");
        get_parameter("pointCloudTopic", PC_TOPIC);

        declare_parameter("costMapTopic", "map");
        get_parameter("costMapTopic", CM_TOPIC);

        declare_parameter("pose_topic", "/pose");
        get_parameter("pose_topic", POSE_TOPIC);

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

        declare_parameter("T_neg", -0.2);
        get_parameter("T_neg", T_NEG);

        declare_parameter("T_pos", 0.2);
        get_parameter("T_pos", T_POS);
        
        declare_parameter("T_seg", 0.1);
        get_parameter("T_seg", T_SEG);

        declare_parameter("T_inliners", 0.5);
        get_parameter("T_inliners", T_INLINERS);

        declare_parameter("T_Iterations", 1.1);
        get_parameter("T_Iterations", T_ITERATIONS);
        
        declare_parameter("Max_traversable_angle", 60.0);
        get_parameter("Max_traversable_angle", MAXANGLE);
        MAXANGLE *= (M_PI/180.0);

        declare_parameter("Num_Colors", 50);
        get_parameter("Num_Colors", NUM_COLORS);

        declare_parameter("Max_Num_Grids", 10);
        get_parameter("Max_Num_Grids", MAX_NUM_GRIDS);

        usleep(100);
    }

  
};




#ifndef NOUTILITY
float PointToPlaneDistance(const PointType& point, float A, float B, float C, float D) {
    return fabs(A * point.x + B * point.y + C * point.z + D) / sqrt(A * A + B * B + C * C);
}
template<typename T>
std::vector<T> linspace(T start ,T end, int num_colors) {
    std::vector<T> color_levels;
    double interval = (end - start) / (num_colors - 1); // Calculate the interval between colors

    for (int i = 0; i < num_colors; ++i) {
        T color = start + i * interval;
        color_levels.push_back(color);
    }

    return color_levels;
}

rmw_qos_profile_t qos_profile_imu{
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    2000,
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false
};

auto qos_imu = rclcpp::QoS(
    rclcpp::QoSInitialization(
        qos_profile_imu.history,
        qos_profile_imu.depth
    ),
    qos_profile_imu);

template<typename Func, typename Type, typename... Args>
void BenchmarkFunction(Type* self, Func func, std::string function_name, Args... args)
 {
    const auto methodStartTime = std::chrono::system_clock::now();
    (self->*func)(std::forward<Args>(args)...);
    const std::chrono::duration<double> durationOfFunction = std::chrono::system_clock::now() - methodStartTime;
    double durationOfFunctionMS = 1000 * durationOfFunction.count();
    self->BenchmarkTiming_<<"The function " << function_name << " Took " << durationOfFunctionMS <<" ms ";
  }

#endif


