#pragma once
#include "traversability_analysis/traversabilityAnalysis.hpp"
#define MAXHEIGHTLAYER "max_height"
#define MINHEIGHTLAYER "min_height"
#define MEANHEIGHTLAYER "mean_height"
#define SEGMENTATIONLAYER "segmentation"
#define REFRENCENONGRIDLAYER "RNG"
#define COLORLAYER "Color"
#define GRIDSPOINTCLOUD "gridsPointClouds"

// utility

#include "traversability_analysis/utility.hpp"

namespace traversability_analysis {
struct NonGroundGrid;  

  struct Cluster {
    std::vector<NonGroundGrid*> grids;
    pcl::PointCloud<PointType> pc;
    float min_height,max_height,mean_height=0.0;
    int color;
    float H_f,angle;
    bool Roughness;

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
    bool CalculateRoughness(Cluster cluster);
    float EstimateAngle(Cluster cluster);
    void savePointCloud(const pcl::PointCloud<PointType> *cloud, const std::string& filename);
    void SaveData(std::vector<Cluster>  &clusters);

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
   

};

}  // namespace traversability_analysis
