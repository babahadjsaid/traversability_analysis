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
#include <nlohmann/json.hpp>
#include <sys/stat.h>
#include <errno.h>
#include <thread>
#include <future>

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
#include <grid_map_cv/grid_map_cv.hpp>



// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <Eigen/Dense>

// Boost
#include <boost/math/special_functions.hpp>
// #include <boost/thread/recursive_mutex.hpp>


#include "map_msgs/msg/occupancy_grid_update.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>



// Definitions of macros.
#define MAXHEIGHTLAYER "max_height"
#define MINHEIGHTLAYER "min_height"
#define MEANHEIGHTLAYER "mean_height"
#define SEGMENTATIONLAYER "segmentation"
#define REFRENCENONGRIDLAYER "RNG"
#define COLORLAYER "Color"
#define GRIDSPOINTCLOUD "gridsPointClouds"
#define CATIGORISATION "2D Convolution"



#define GROUND "ground"
#define OBSTACLES "Obstacle"
#define POTHOLE "Pothole"
#define SLOPE  "Slope"
#define NEGATIVESLOPE "Negative Slope"

#define NO_INFORMATION -1
#define LETHAL_OBSTACLE 100
#define MAX_NON_OBSTACLE 99
#define FREE_SPACE 0
// end of Definitions of macros.

typedef pcl::PointXYZI PointType;
inline bool createDirectoryIfNotExists(const std::string& directory) {
    struct stat st;

    // Check if the directory exists
    if (stat(directory.c_str(), &st) == 0 && S_ISDIR(st.st_mode)) {
        // Directory exists
        return true;
    } else {
        // Directory doesn't exist, attempt to create it
        if (mkdir(directory.c_str(), 0755) == 0) {
            std::cout << "Directory created: " << directory << std::endl;
            return true;
        } else {
            std::cerr << "Failed to create directory: " << directory << std::endl;
            return false;
        }
    }
}



class ParamServer 
{
public:
    std::string PC_TOPIC,CM_TOPIC,MAP_FRAME,POSE_TOPIC,GM_TOPIC,L_LVL,
    L_FN,L_DIR;
    bool LOGGER;
    float L_MAP_LENGTH, CELL_RESOLUTION,T_DIFF,T_HIGH,T_LOW,
    T_RATIO_BIG_OVER_SMALL = 20.0,T_RATIO_BIG_OVER_MIDDLE_MIN = 0.1, T_RATIO_BIG_OVER_MIDDLE_MAX = 0.7,
    MAX_HEIGHT, MEAN_GRASS, VARIANCE_GRASS, T_PROB, UB,
    LB, T_RATIO, T_L, T_S, T_NEG, T_POS, T_SEG, VERTICAL_THRESHOLD, T_ITERATIONS, MAXANGLE,GLOBAL_MAP_HEIGHT,GLOBAL_MAP_WIDTH,GLOBAL_MAP_INCR,GLOBAL_MAP_RES;
    int NUM_GRIDS_MIN, NUM_COLORS, MAX_NUM_GRIDS;
    
    const char* log_level;
    std::shared_ptr<rclcpp::Node> nodeHandle_;
    ParamServer(std::shared_ptr<rclcpp::Node> nodeHandle) : nodeHandle_(nodeHandle)
    {   log_level = std::getenv("RCLCPP_LOG_LEVEL");
        nodeHandle_->declare_parameter("pointCloudTopic", "/points");
        nodeHandle_->get_parameter("pointCloudTopic", PC_TOPIC);

        nodeHandle_->declare_parameter("costMapTopic", "map");
        nodeHandle_->get_parameter("costMapTopic", CM_TOPIC);

        nodeHandle_->declare_parameter("gridMapTopic", "map");
        nodeHandle_->get_parameter("gridMapTopic", GM_TOPIC);

        nodeHandle_->declare_parameter("pose_topic", "/pose");
        nodeHandle_->get_parameter("pose_topic", POSE_TOPIC);

        nodeHandle_->declare_parameter("map_frame_id", "map_centric");
        nodeHandle_->get_parameter("map_frame_id", MAP_FRAME);

        nodeHandle_->declare_parameter("map_length", 20.0);
        nodeHandle_->get_parameter("map_length", L_MAP_LENGTH);
        
        nodeHandle_->declare_parameter("cell_resolution", 0.2);
        nodeHandle_->get_parameter("cell_resolution", CELL_RESOLUTION);

        nodeHandle_->declare_parameter("T_diff", 0.05);
        nodeHandle_->get_parameter("T_diff", T_DIFF);

        nodeHandle_->declare_parameter("T_high", 0.05);
        nodeHandle_->get_parameter("T_high", T_HIGH);

        nodeHandle_->declare_parameter("T_low", 0.05);
        nodeHandle_->get_parameter("T_low", T_LOW);

        nodeHandle_->declare_parameter("max_height", 1.5);
        nodeHandle_->get_parameter("max_height", MAX_HEIGHT);

        nodeHandle_->declare_parameter("mean_grass", 0.0);
        nodeHandle_->get_parameter("mean_grass", MEAN_GRASS);

        nodeHandle_->declare_parameter("variance_grass", 0.0);
        nodeHandle_->get_parameter("variance_grass", VARIANCE_GRASS);

        nodeHandle_->declare_parameter("T_prob", 1.0E-7);
        nodeHandle_->get_parameter("T_prob", T_PROB);

        UB = MEAN_GRASS + sqrt(VARIANCE_GRASS) * sqrt(2) * boost::math::erfc_inv(2 * T_PROB);

        // Compute the lower bound
        LB = MEAN_GRASS - (UB - MEAN_GRASS);

        nodeHandle_->declare_parameter("num_grids_min", 9);
        nodeHandle_->get_parameter("num_grids_min", NUM_GRIDS_MIN);

        nodeHandle_->declare_parameter("T_ratio", 1.7);
        nodeHandle_->get_parameter("T_ratio", T_RATIO);

        nodeHandle_->declare_parameter("T_l", 1.7);
        nodeHandle_->get_parameter("T_l", T_L);
        
        nodeHandle_->declare_parameter("T_s", 1.7);
        nodeHandle_->get_parameter("T_s", T_S);

        nodeHandle_->declare_parameter("T_neg", -0.2);
        nodeHandle_->get_parameter("T_neg", T_NEG);

        nodeHandle_->declare_parameter("T_pos", 0.2);
        nodeHandle_->get_parameter("T_pos", T_POS);
        
        nodeHandle_->declare_parameter("T_seg", 0.1);
        nodeHandle_->get_parameter("T_seg", T_SEG);

        nodeHandle_->declare_parameter("T_Vertical", 0.75);
        nodeHandle_->get_parameter("T_Vertical", VERTICAL_THRESHOLD);

        nodeHandle_->declare_parameter("T_Iterations", 1.1);
        nodeHandle_->get_parameter("T_Iterations", T_ITERATIONS);
        
        nodeHandle_->declare_parameter("Max_traversable_angle", 60.0);
        nodeHandle_->get_parameter("Max_traversable_angle", MAXANGLE);
        MAXANGLE *= (M_PI/180.0);

        nodeHandle_->declare_parameter("Num_Colors", 5);
        nodeHandle_->get_parameter("Num_Colors", NUM_COLORS);

        nodeHandle_->declare_parameter("Max_Num_Grids", 10);
        nodeHandle_->get_parameter("Max_Num_Grids", MAX_NUM_GRIDS);

        nodeHandle_->declare_parameter("Global_map_resolution", 3.0);
        nodeHandle_->get_parameter("Global_map_resolution", GLOBAL_MAP_RES);

        GLOBAL_MAP_RES *= CELL_RESOLUTION;
        nodeHandle_->declare_parameter("Global_map_init_height", 50.0);
        nodeHandle_->get_parameter("Global_map_init_height", GLOBAL_MAP_HEIGHT);
        GLOBAL_MAP_HEIGHT =  ceil(GLOBAL_MAP_HEIGHT/GLOBAL_MAP_RES);
        nodeHandle_->declare_parameter("Global_map_init_width", 50.0);
        nodeHandle_->get_parameter("Global_map_init_width", GLOBAL_MAP_WIDTH);
        GLOBAL_MAP_WIDTH =  ceil(GLOBAL_MAP_WIDTH/GLOBAL_MAP_RES);

        nodeHandle_->declare_parameter("Global_map_increment", 20.0);
        nodeHandle_->get_parameter("Global_map_increment", GLOBAL_MAP_INCR);

        nodeHandle_->declare_parameter("Logger", true);
        nodeHandle_->get_parameter("Logger", LOGGER);

        nodeHandle_->declare_parameter("LoggerLevel", "DEBUG");
        nodeHandle_->get_parameter("LoggerLevel", L_LVL);

        nodeHandle_->declare_parameter("LoggerFilename", "TraversabilityAnalysis.json");
        nodeHandle_->get_parameter("LoggerFilename", L_FN);

        nodeHandle_->declare_parameter("LoggerDirectory", "./.log/");
        nodeHandle_->get_parameter("LoggerDirectory", L_DIR);


        usleep(100);
    }
    
  
};


struct Position {
    Position() = default;
    Position(double i, double j){
        x_ = i;
        y_ = j;
    }
    
    template<typename T>
    Position operator+( T other)  {
        return Position(x_ + other.x(), y_ + other.y());
    }
    
    double& x() { return x_; }
    double& y() { return y_; }
    
    friend std::ostream& operator<<(std::ostream& os, const Position& pos) {
        os << "(" << pos.x_ << ", " << pos.y_ << ")";
        return os;
    }
    // create a deep copy when = operator is used
    template<typename T>
    Position& operator=(T& other) {
        x_ = other.x();
        y_ = other.y();
        return *this;
    }
    // create a function called when left multiplied by a rotation matrix that should be a 2x2 matrix
    Position operator*(Eigen::Matrix2f& rotation) {
        return Position(rotation(0, 0) * x_ + rotation(0, 1) * y_, rotation(1, 0) * x_ + rotation(1, 1) * y_);
    }
    private:
        double x_,y_;
};
struct Pose {
    Eigen::Vector3d position; // X, Y, Z
    Eigen::Vector3d orientation; // Roll, Pitch, Yaw
    Pose() {
        position << 0, 0, 0;
        orientation << 0, 0, 0;
    }
    double& x() { return position.x(); }
    double& y() { return position.y(); }
    double& z() { return position.z(); }
    double& roll() { return orientation.x(); }
    double& pitch() { return orientation.y(); }
    double& yaw() { return orientation.z(); }
    // create a deep copy when = operator is used
    Pose& operator=(const Pose& other) {
        position = other.position;
        orientation = other.orientation;
        return *this;
    }
    Pose(double x, double y, double z, double roll, double pitch, double yaw)
        : position(x, y, z), orientation(roll, pitch, yaw) {}
};

struct Index {
    int i,j;
};
struct NonGroundGrid;  
enum ObjectsCategories {
    gROUND=0,
    oBSTACLES,
    pOTHOLE,
    sLOPE,
    nEGATIVESLOPE
};
enum ClusterStatus {
    nEw=0,
    uPTODATE,
    oLD,
    tODELETE
};

  struct Cluster {
    std::vector<NonGroundGrid*> grids;
    pcl::PointCloud<PointType> pc;
    pcl::PointXYZINormal Plane ;
    double min_height,max_height,mean_height=0.0, ratioBigToSmallest, ratioBigTomiddle;
    double H_f,angle=0;
    bool Roughness = false;
    int color;
    double variance;
    grid_map::Position Point_mass = {0,0};
    ClusterStatus Status = nEw;
    ObjectsCategories Type = oBSTACLES;
    Eigen::Matrix2d ClusterCovarianceMatrix_ = 0.5 * Eigen::Matrix2d::Identity();
    Eigen::Vector2d differs;
    Eigen::Vector3f eigenvalues;
    Eigen::Matrix3f eigenvectors;
    int64_t id;
    int count = 0;
    bool matched = false, isVertical=false, isFlat = false;
    std::map<ObjectsCategories, int> category_count;
    Cluster(){
        id = std::chrono::system_clock::now().time_since_epoch().count();
        category_count[sLOPE] = 0;
        category_count[nEGATIVESLOPE] = 0;
        category_count[oBSTACLES] = 0;
        category_count[pOTHOLE] = 0;

    }
    ObjectsCategories getCat(){
        ObjectsCategories cattmp = oBSTACLES;
        int max = 0;
        for (const auto& pair : category_count){
            if(pair.second > max){
                max = pair.second;
                cattmp = pair.first;
            }
        }
        if (cattmp >4)cattmp = oBSTACLES;
        return cattmp;
    }
    std::string GetCategoryName(ObjectsCategories categoryName){
        if (categoryName == oBSTACLES) return OBSTACLES;
        if (categoryName == sLOPE) return SLOPE ;
        if (categoryName == nEGATIVESLOPE) return NEGATIVESLOPE;
        if (categoryName == pOTHOLE) return POTHOLE;
        return GROUND;
        }
    bool UpdateStatus(){
        if (Status == tODELETE) return false;

        if (Status == nEw) Status = oLD;
        
        
        if (pc.points.size() < 10)
        {
            Status = tODELETE;
            return false;
        }
        return true;
      
    }
    std::string savePointCloud(std::string directory) {
        if (pc.points.size() == 0)
        {
            return "empty";
        }
        directory = directory+"data/";
        createDirectoryIfNotExists(directory);
        std::stringstream ss;
        
        ss << directory <<"pointcloud_" << id << ".pcd";
        std::string filename = ss.str();
        
        pcl::PCDWriter writer;
        writer.writeBinaryCompressed(filename, pc);
        
        
    return filename;
    }
    nlohmann::json serializeClusterToJson(std::string directory) {
        
        nlohmann::json json;

        // Serialize primitive fields
        json["id"] = id;
        json["min_height"] = min_height;
        json["max_height"] = max_height;
        json["mean_height"] = mean_height;
        json["Height_Index"] = H_f;
        json["angle"] = angle;
        json["Roughness"] = Roughness;
        json["isVertical"] = isVertical;
        json["ratioBigToSmallest"] = ratioBigToSmallest;
        json["ratioBigTomiddle"] = ratioBigTomiddle;
        json["isFlat"] = isFlat;
        // Serialize PointCloud to a file and get the filename
        json["point_cloud_file"] = savePointCloud(directory);

        // Serialize Eigen::Vector3f eigenvalues
        json["eigenvalues"] = { eigenvalues(0), eigenvalues(1), eigenvalues(2) };

        // Serialize Eigen::Matrix3f eigenvectors (flattened into 1D array)
        json["eigenvectors"] = {
            eigenvectors(0,0), eigenvectors(0,1), eigenvectors(0,2),
            eigenvectors(1,0), eigenvectors(1,1), eigenvectors(1,2),
            eigenvectors(2,0), eigenvectors(2,1), eigenvectors(2,2)
        };

        // Serialize Eigen::Matrix2d ClusterCovarianceMatrix_
        json["ClusterCovarianceMatrix"] = {
            ClusterCovarianceMatrix_(0,0), ClusterCovarianceMatrix_(0,1),
            ClusterCovarianceMatrix_(1,0), ClusterCovarianceMatrix_(1,1)
        };

        // Serialize Point_mass (grid_map::Position)
        json["Point_mass"] = { Point_mass.x(), Point_mass.y() };

        // Serialize category_count map
        for (const auto& category : category_count) {
            json["category_count"][GetCategoryName(category.first)] = category.second;
        }

        // Serialize status
        json["Status"] = Status;  // You may need to define how ClusterStatus is serialized

    return json;
}

};

struct NonGroundGrid {
    grid_map::Index index;
    Cluster* cluster;
    int color;
    bool clustered = false;
    long int idx;
};

inline float PointToPlaneDistance(const PointType& point, float A, float B, float C, float D) {
    return fabs(A * point.x + B * point.y + C * point.z + D) / sqrt(A * A + B * B + C * C);
}
template<typename T>
inline std::vector<T> linspace(T start ,T end, int num_colors) {
    std::vector<T> color_levels;
    double interval = (end - start) / (num_colors - 1); // Calculate the interval between colors

    for (int i = 0; i < num_colors; ++i) {
        T color = start + i * interval;
        color_levels.push_back(color);
    }

    return color_levels;
}


inline rmw_qos_profile_t qos_profile_imu{
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

inline auto qos_imu = rclcpp::QoS(
    rclcpp::QoSInitialization(
        qos_profile_imu.history,
        qos_profile_imu.depth
    ),
    qos_profile_imu);

template<typename Func, typename Type, typename... Args>
inline void BenchmarkFunction(Type* self, Func func, std::string function_name, Args... args)
 {
    const auto methodStartTime = std::chrono::system_clock::now();
    (self->*func)(std::forward<Args>(args)...);
    const std::chrono::duration<double> durationOfFunction = std::chrono::system_clock::now() - methodStartTime;
    double durationOfFunctionMS = 1000 * durationOfFunction.count();
    self->BenchmarkTiming_<<"The function " << function_name << " Took " << durationOfFunctionMS <<" ms ";
    self->times_[function_name].push_back(durationOfFunctionMS);
  }



