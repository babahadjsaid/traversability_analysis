#pragma once
#include "traversability_analysis/traversabilityAnalysis.hpp"
#define MAXHEIGHTLAYER "max_height"
#define MINHEIGHTLAYER "min_height"
#define MEANHEIGHTLAYER "mean_height"
#define NUMBEROFPOINTSLAYER "num_points"
#define SEGMENTATIONLAYER "segmentation"
#define REFRENCENONGRIDLAYER "RNG"
#define COLORLAYER "Color"

// utility
#include "traversability_analysis/utility.hpp"

namespace traversability_analysis {
struct NonGroundGrid {
    grid_map::Index index;
    std::vector<NonGroundGrid*>* cluster;
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
    void FloodFill(grid_map::Index index,std::vector<NonGroundGrid*> *cluster, int color);
    
   

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
   std::vector<std::vector<NonGroundGrid*>> Clusters_;
   

};

}  // namespace traversability_analysis
