#pragma once
#include "traversability_analysis/traversabilityAnalysis.hpp"
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

// utility

#include "traversability_analysis/utility.hpp"

namespace traversability_analysis {
struct NonGroundGrid;  

  struct Cluster {
    std::vector<NonGroundGrid*> grids;
    pcl::PointCloud<PointType> pc;
    float min_height,max_height,mean_height=0.0;
    int color;
    float H_f,angle=0;
    bool Roughness = false;
    std::string Type;
    grid_map::Position Point_mass;
    Eigen::Vector3f vec;
    PointType p1;
    std::stringstream *ss_,*sp_;

};
enum ObjectsCategories {
    gROUND,
    oBSTACLES,
    pOTHOLE,
    sLOPE,
    nEGATIVESLOPE
};

struct NonGroundGrid {
    grid_map::Index index;
    Cluster* cluster;
    int color;
    bool clustered = false;
};


class TraversabilityAnalysis : public ParamServer{
 public:
   long int numFrames=0;
   float avgTime;
    /*!
    * Constructor.
    */
    explicit TraversabilityAnalysis(std::string node_name, const rclcpp::NodeOptions & options);

    void PointCloudHandler(sensor_msgs::msg::PointCloud2::SharedPtr pointCloudMsg);
    double NormalPDF(double x, double mean, double variance);
    void FloodFill(grid_map::Index index,Cluster *cluster, int color);
    bool CalculateRoughness(Cluster &cluster);
    void EstimateAngle(Cluster &cluster);
    void savePointCloud(const pcl::PointCloud<PointType> *cloud, const std::string& filename);
    void SaveData();
    ObjectsCategories checkCategory(std::string &categoryName);
    float getZaxis(Eigen::Vector3f eigenvalues, Eigen::Matrix3f eigenvectors );

 private:
    //Sync
    std::mutex mapMtx_;

    //Map
    grid_map::GridMap elevationMap_;
    grid_map::Size size_;
    //Topics 
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudSub_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr costMapPub_;
    Eigen::MatrixXf kernel_;
    std::vector<NonGroundGrid> C_N_;
    std::vector<Cluster> Clusters_;
    std::vector<pcl::PointCloud<PointType>> gridsPointClouds_;
    std::random_device randomDevice_;
    std::mt19937 generator_;
    
   

};

}  // namespace traversability_analysis
