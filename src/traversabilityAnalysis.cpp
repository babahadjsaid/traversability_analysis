
#include "traversability_analysis/traversabilityAnalysis.hpp"


namespace traversability_analysis{

TraversabilityAnalysis::TraversabilityAnalysis(std::string node_name, const rclcpp::NodeOptions & options)
: ParamServer(node_name, options),
elevationMap_({"max_height","min_height","mean_height","num_points","segmentation"}),
kernel(3,3)
{
  pointCloudSub_ = create_subscription<sensor_msgs::msg::PointCloud2>(PC_TOPIC, 1,std::bind(&TraversabilityAnalysis::PointCloudHandler, this, std::placeholders::_1));
  costMapPub_ = create_publisher<grid_map_msgs::msg::GridMap>(CM_TOPIC, 1); //nav2_msgs::msg::Costmap
  float side_length = 2* RADIUS;// * (sqrt(2)/2);
  elevationMap_.clearAll();
  elevationMap_.resetTimestamp();
  elevationMap_.setFrameId(MAP_FRAME);
  RCLCPP_INFO(get_logger(),"Hello:%f",side_length);
  elevationMap_.setGeometry(grid_map::Length(side_length,side_length),CELL_RESOLUTION,grid_map::Position(0,0));

  kernel << -1, -1, -1, -1, 8, -1, -1, -1, -1;


}

void TraversabilityAnalysis::PointCloudHandler(sensor_msgs::msg::PointCloud2::SharedPtr pointCloudMsg){
  const auto methodStartTime = std::chrono::system_clock::now();
  pcl::PointCloud<PointType>::Ptr pointCloud(new pcl::PointCloud<PointType>());
  pcl::moveFromROSMsg(*pointCloudMsg, *pointCloud);
  auto& max_heightLayer = elevationMap_["max_height"];
  auto& num_pointsLayer = elevationMap_["num_points"];
  auto& min_heightLayer = elevationMap_["min_height"];
  auto& mean_heightLayer = elevationMap_["mean_height"];
  //                                                      Map Projection.....
  for (unsigned int i = 0; i < pointCloud->size(); ++i)
  {
    auto& point = pointCloud->points[i];
    grid_map::Index index;
    grid_map::Position position(-point.y, point.x);  
    if (!elevationMap_.getIndex(position, index)) {
      continue;
    }
    if (point.z >MAX_HEIGHT)
    {
      continue;
    }
    
    auto& max_height = max_heightLayer(index(0), index(1));
    auto& min_height = min_heightLayer(index(0), index(1));
    auto& mean_height = mean_heightLayer(index(0), index(1));
    auto& num_points = num_pointsLayer(index(0), index(1));
    
    if (!std::isfinite(max_height))
    {
    max_height = point.z;
    min_height = point.z;
    mean_height = point.z;
    num_points = 1;
    continue;
    }
    num_points++;
    mean_height+= point.z;
    if (max_height < point.z)
        max_height = point.z;
    if (min_height > point.z)
        min_height = point.z;
    


  }
  mean_heightLayer = mean_heightLayer.array() / num_pointsLayer.array();
  //                                                      End Map Projection.
  //                                                      Ground Segmentation.
  const std::chrono::duration<double> durations  = std::chrono::system_clock::now() - methodStartTime;
  RCLCPP_INFO(get_logger(), "dDone in  %f ms.", 1000* durations.count());
  auto& mean_height = elevationMap_.get("mean_height");
  
  for (grid_map::GridMapIterator it(elevationMap_); !it.isPastEnd(); ++it) {
    float& segmentation = elevationMap_.at("segmentation", *it);
    float& max_height = elevationMap_.at("max_height", *it);
    float& min_height = elevationMap_.at("min_height", *it);
    segmentation = !((max_height - min_height) >= T_DIFF || max_height >= T_HIGH || min_height <= T_LOW);
    if (!segmentation)
    {
      if (!((*it)(0)>0 && (*it)(1)<mean_height.cols()-1 && (*it)(1)>0 && (*it)(0)< mean_height.rows()-1)) 
      {
        continue;
      }
      segmentation = (mean_height.block((*it)(0)-1,(*it)(1)-1,3,3).array() * kernel.array()).sum() >= T_DIFF ;
    }
    
   }
  
  
  std::unique_ptr<grid_map_msgs::msg::GridMap> message;
  message = grid_map::GridMapRosConverter::toMessage(elevationMap_);
  costMapPub_->publish(std::move(message));
  elevationMap_.clearAll();
  const std::chrono::duration<double> duration  = std::chrono::system_clock::now() - methodStartTime;
  RCLCPP_INFO(get_logger(), "Done in  %f ms.", 1000* duration.count());
  
}
}
