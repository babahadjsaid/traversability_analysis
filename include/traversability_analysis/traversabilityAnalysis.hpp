#pragma once



//utility
#include "traversability_analysis/utility.hpp"

namespace traversability_analysis {


class TraversabilityAnalysis : public ParamServer{
 public:
   long int numFrames=0;
   float avgTime;
    /*!
    * Constructor.
    */
    explicit TraversabilityAnalysis(std::string node_name, const rclcpp::NodeOptions & options);

    void PointCloudHandler(sensor_msgs::msg::PointCloud2::SharedPtr pointCloudMsg);

    
   

 private:
   //Sync
   std::mutex mapMtx_;

   //Map
   grid_map::GridMap elevationMap_;
   
   //Topics 
   rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudSub_;
   rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr costMapPub_;
   Eigen::MatrixXf kernel;
   

};

}  // namespace traversability_analysis
