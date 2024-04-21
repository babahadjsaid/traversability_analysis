
#include "traversability_analysis/traversabilityAnalysis.hpp"


namespace traversability_analysis{

TraversabilityAnalysis::TraversabilityAnalysis(std::string node_name, const rclcpp::NodeOptions & options)
: ParamServer(node_name, options),
elevationMap_({"max_height","min_height","mean_height","num_points","segmentation"}),
kernel(3,3)
{
  pointCloudSub_ = create_subscription<sensor_msgs::msg::PointCloud2>(PC_TOPIC, 1,std::bind(&TraversabilityAnalysis::PointCloudHandler, this, std::placeholders::_1));
  costMapPub_ = create_publisher<grid_map_msgs::msg::GridMap>(CM_TOPIC, 1); //nav2_msgs::msg::Costmap
  float side_length = RADIUS * sqrt(2);
  elevationMap_.clearAll();
  elevationMap_.resetTimestamp();
  elevationMap_.setFrameId(MAP_FRAME);
  RCLCPP_INFO(get_logger(),"Hello:%f",side_length);
  elevationMap_.setGeometry(grid_map::Length(side_length,side_length),CELL_RESOLUTION,grid_map::Position(0,0));

  kernel << -1, -1, -1, -1, 8, -1, -1, -1, -1;


}

void TraversabilityAnalysis::PointCloudHandler(sensor_msgs::msg::PointCloud2::SharedPtr pointCloudMsg){
  //std::lock_guard<std::mutex> lock(mapMtx_);
  mapMtx_.lock();
  const auto methodStartTime = std::chrono::system_clock::now();
  pcl::PointCloud<PointType>::Ptr pointCloud(new pcl::PointCloud<PointType>());
  pcl::moveFromROSMsg(*pointCloudMsg, *pointCloud);
  auto& min_heightLayer = elevationMap_["min_height"];
  auto& max_heightLayer = elevationMap_["max_height"];
  auto& mean_heightLayer = elevationMap_["mean_height"];
  auto& num_pointsLayer = elevationMap_["num_points"];
  //                                                      Map Projection.....
  num_pointsLayer.setOnes();
  mean_heightLayer.setZero();
  max_heightLayer.setZero();
  min_heightLayer.setConstant(MAX_HEIGHT);
  for (unsigned int i = 0; i < pointCloud->size(); ++i)
  {
    auto& point = pointCloud->points[i];
    grid_map::Index index;
    grid_map::Position position(point.x, point.y);  
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
    
    if (num_points==1)
    {
    max_height = min_height = mean_height = point.z;// don't forget to include the intensity.
    continue;
    }
    num_points++;
    mean_height += point.z;
    if (max_height < point.z)
        max_height = point.z;
    if (min_height > point.z)
        min_height = point.z;
    


  }
  mean_heightLayer = mean_heightLayer.array() / num_pointsLayer.array();
  //                                                      End Map Projection.
  //                                                      Ground Segmentation.
  const std::chrono::duration<double> durations  = std::chrono::system_clock::now() - methodStartTime;
  //RCLCPP_INFO(get_logger(), "dDone in  %f ms.", 1000* durations.count());
  auto& mean_height = elevationMap_.get("mean_height");
  auto& segmentL = elevationMap_.get("segmentation");
  auto& max_heightL = elevationMap_.get("max_height");
  auto& min_heightL = elevationMap_.get("min_height");
  grid_map::Size size = elevationMap_.getSize();
  
  // #pragma omp parallel for num_threads(5)
  for (int i = 0; i < size(0); i++)
  {
    // #pragma omp parallel for num_threads(5)
    for (int j = 0; j < size(1); j++)
    {
      float& segmentation = segmentL(i,j);
      float& max_height   = max_heightL(i,j);
      float& min_height   = min_heightL(i,j);
      
      
      segmentation = ((max_height - min_height) >= T_DIFF || max_height >= T_HIGH || min_height <= T_LOW);
      if (!(i>0 && i < mean_height.rows()-1 && j>0 && j < mean_height.cols()-1))
          continue;
      // continue;
      Eigen::MatrixXf segs = segmentL.block(i-1,j-1,3,3).array();
      bool allZeros = true;
      for (int k = 0; k < 3; ++k) {
          for (int l = 0; l < 3; ++l) {
              if (segs(k,l)) {
                  allZeros = false;
                  break;
              }
          }
      }
    if (allZeros)
    // #pragma omp critical
    {
      segmentation = (mean_height.block(i-1,j-1,3,3).array() * kernel.array()).sum() >= T_DIFF ;
      if (segmentation)
      
      {
        mean_height(i,j) = T_HIGH;
      }
      }
      
      
    }
    
  }
  
  float measn = min_heightL.minCoeff();
  std::unique_ptr<grid_map_msgs::msg::GridMap> message;
  message = grid_map::GridMapRosConverter::toMessage(elevationMap_);
  costMapPub_->publish(std::move(message));
  elevationMap_.clearAll();
  const std::chrono::duration<double> duration  = std::chrono::system_clock::now() - methodStartTime;
  RCLCPP_INFO(get_logger(), "nuber of loops is %ld Done in  %f ms and the mean is %f.",(int) (mean_height.cols()-1)*mean_height.rows() , 1000* duration.count(),measn);
  mapMtx_.unlock();
  numFrames++;
  avgTime+= duration.count();
}
}



// for (grid_map::GridMapIterator it(elevationMap_); !it.isPastEnd(); ++it) {
//     float& segmentation = elevationMap_.at("segmentation", *it);
//     float& max_height = elevationMap_.at("max_height", *it);
//     float& min_height = elevationMap_.at("min_height", *it);
//     float& mean_heights = elevationMap_.at("mean_height", *it);
//     segmentation = !((max_height - min_height) >= T_DIFF || max_height >= T_HIGH || min_height <= T_LOW);
//     if (!((*it)(0)>0 && (*it)(1)<mean_height.cols()-1 && (*it)(1)>0 && (*it)(0)< mean_height.rows()-1)) 
//       {
//         continue;
//       }
//     Eigen::MatrixXf segs = segmentL.block((*it)(0)-1,(*it)(1)-1,3,3).array();
//     bool allOnes = true;
//     for (int k = 0; k < 3; ++k) {
//         for (int l = 0; l < 3; ++l) {
//             if (!segs(k,l)) {
//                 allOnes = false;
//                 break;
//             }
//         }
//     }
//     if (allOnes)
//     {
      
      
//       segmentation = !(mean_height.block((*it)(0)-1,(*it)(1)-1,3,3).array() * kernel.array()).sum() >= T_DIFF ;
//       if (!segmentation)
//       {
//         mean_heights = T_HIGH;
//       }
      
//       RCLCPP_INFO_STREAM(get_logger(), "segmentation is "<<segmentation<<" and matrix is" <<mean_height.block((*it)(0)-1,(*it)(1)-1,3,3).array());
//     }
    
    
//    }