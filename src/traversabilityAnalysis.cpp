
#include "traversability_analysis/traversabilityAnalysis.hpp"


namespace traversability_analysis{

TraversabilityAnalysis::TraversabilityAnalysis(std::string node_name, const rclcpp::NodeOptions & options)
: ParamServer(node_name, options),
elevationMap_({MAXHEIGHTLAYER,MINHEIGHTLAYER,MEANHEIGHTLAYER,NUMBEROFPOINTSLAYER,SEGMENTATIONLAYER,REFRENCENONGRIDLAYER,COLORLAYER}),
kernel_(3,3)
{
  pointCloudSub_ = create_subscription<sensor_msgs::msg::PointCloud2>(PC_TOPIC, 1,std::bind(&TraversabilityAnalysis::PointCloudHandler, this, std::placeholders::_1));
  costMapPub_ = create_publisher<grid_map_msgs::msg::GridMap>(CM_TOPIC, 1); //nav2_msgs::msg::Costmap
  float side_length = RADIUS * sqrt(2);
  elevationMap_.clearAll();
  elevationMap_.resetTimestamp();
  elevationMap_.setFrameId(MAP_FRAME);
  RCLCPP_INFO(get_logger(),"Hello:%f",side_length);
  elevationMap_.setGeometry(grid_map::Length(side_length,side_length),CELL_RESOLUTION,grid_map::Position(0,0));
  size_ = elevationMap_.getSize();
  kernel_ << -1, -1, -1, -1, 8, -1, -1, -1, -1;


}

void TraversabilityAnalysis::PointCloudHandler(sensor_msgs::msg::PointCloud2::SharedPtr pointCloudMsg){
  mapMtx_.lock();
  const auto methodStartTime = std::chrono::system_clock::now();
  pcl::PointCloud<PointType>::Ptr pointCloud(new pcl::PointCloud<PointType>());
  pcl::moveFromROSMsg(*pointCloudMsg, *pointCloud);
  auto& min_heightLayer = elevationMap_[MINHEIGHTLAYER];
  auto& max_heightLayer = elevationMap_[MAXHEIGHTLAYER];
  auto& mean_heightLayer = elevationMap_[MEANHEIGHTLAYER];
  auto& num_pointsLayer = elevationMap_[NUMBEROFPOINTSLAYER];
  //                                                      Start Map Projection.
  num_pointsLayer.setZero();
  mean_heightLayer.setZero();
  max_heightLayer.setZero();
  min_heightLayer.setConstant(MAX_HEIGHT);
  #pragma omp parallel for num_threads(5)
  for (unsigned int i = 0; i < pointCloud->size(); ++i)
  {
    auto& point = pointCloud->points[i];
    grid_map::Index index;
    grid_map::Position position(point.x, point.y);  
    if (!elevationMap_.getIndex(position, index) || point.z >MAX_HEIGHT) {
      continue;
    }
   
    auto& max_height = max_heightLayer(index(0), index(1));
    auto& min_height = min_heightLayer(index(0), index(1));
    auto& mean_height = mean_heightLayer(index(0), index(1));
    auto& num_points = num_pointsLayer(index(0), index(1));
    
    double pdf = NormalPDF(point.intensity, 24.8704, 20.9897);
      if (point.z>T_HIGH && point.z<1.2 && pdf>0.0000001 )
      #pragma omp critical
      {
        point.z = T_HIGH - 0.001;
        
      }
      if (num_points==0)
      {
         #pragma omp critical
        {
          max_height = min_height = mean_height = point.z;// don't forget to include the intensity.
          num_points++;
        }
      continue;
      }
    #pragma omp critical
    {
      num_points++;
      mean_height += point.z;
      if (max_height < point.z)
          max_height = point.z;
      if (min_height > point.z)
          min_height = point.z;
    }


  }
  // RCLCPP_INFO(get_logger(), "pdf=%f, minimum=%f",mean_pdf/count,min_pdf);
  mean_heightLayer = mean_heightLayer.array() / num_pointsLayer.array();
  const auto EndProjection = std::chrono::system_clock::now();
  //                                                      End Map Projection.
  //                                                      Start Ground Segmentation.
  const std::chrono::duration<double> durations  = std::chrono::system_clock::now() - methodStartTime;
  //RCLCPP_INFO(get_logger(), "dDone in  %f ms.", 1000* durations.count());
  auto& mean_heightL = elevationMap_.get(MEANHEIGHTLAYER);
  auto& segmentL = elevationMap_.get(SEGMENTATIONLAYER);
  auto& max_heightL = elevationMap_.get(MAXHEIGHTLAYER);
  auto& min_heightL = elevationMap_.get(MINHEIGHTLAYER);
  auto& RNGL = elevationMap_.get(REFRENCENONGRIDLAYER);
  
  RNGL.setConstant(-1);
  // #pragma omp parallel for num_threads(5)
  for (int i = 0; i < size_(0); i++)
  {
    // #pragma omp parallel for num_threads(5)
    for (int j = 0; j < size_(1); j++)
    {
      float& segmentation = segmentL(i,j);
      float& max_height   = max_heightL(i,j);
      float& min_height   = min_heightL(i,j);
      float& mean_height   = mean_heightL(i,j);
      float& RNG   = RNGL(i,j);
      if (min_height==MAX_HEIGHT) // if the cell was not visited.
      {
        min_height = max_height = mean_height = 0;
        segmentation = 2;
        continue;
      }
      bool allZeros = true, allOnes = true;
      Eigen::MatrixXf segs;
      segmentation = ((max_height - min_height) >= T_DIFF || max_height >= T_HIGH || min_height <= T_LOW);
      if (!(i>0 && i < mean_heightL.rows()-1 && j>0 && j < mean_heightL.cols()-1))
          goto AddToNonGrid;
      // continue;
      segs = segmentL.block(i-1,j-1,3,3).array();
      
      for (int k = 0; k < 3; ++k) {
          for (int l = 0; l < 3; ++l) {
            if (k==1 && l==1)
            {
              continue;
            }
            if(((bool) segs(0,0))^((bool) segs(k,l)) || segs(k,l)==2){
              allOnes = allZeros = false;
              break;
            }
            if(k==2 && l==2){
                allOnes = segs(k,l);
                allZeros = !allOnes;
            }
            
          }
      }
      if (allZeros && !segmentation)
      // #pragma omp critical
      {
        segmentation = (mean_heightL.block(i-1,j-1,3,3).array() * kernel_.array()).sum() >= T_DIFF ;
        if (segmentation)
        
        {
          mean_height = T_HIGH;
        }
        }
      if(allOnes && segmentation==2)
            segmentation = allOnes;  
      if (allZeros && segmentation == 2)
            segmentation = !allZeros; 
      AddToNonGrid:
      if (segmentation==true)
      {
        grid_map::Index idx(i,j);
        NonGroundGrid tmp;
        if (i==2 && j==5)
        {
          RCLCPP_INFO(get_logger(), "The number is %ld",C_N_.size());
        }
        
        RNG = C_N_.size();
        tmp.index = idx;
        C_N_.push_back(tmp);

      }
      
    }
    
  }
  
  const auto EndSegmentation = std::chrono::system_clock::now();
  //                                                      End Ground Segmentation.
  //                                                      Start Grids clustering.
  std::random_device rd;
  std::mt19937 gen(rd());
  
  // Define the distribution for random numbers between 0 and 255
  std::uniform_int_distribution<int> distribution(0, 255);
  
  for (size_t i = 0; i < C_N_.size(); i++)
  {
    
    if (C_N_[i].clustered) continue;
    std::vector<NonGroundGrid*> new_cluster;
    
    FloodFill(C_N_[i].index,&new_cluster, distribution(gen));
    Clusters_.push_back(new_cluster);
  }
  

  const auto EndClustering = std::chrono::system_clock::now();
  //                                                      End Grids clustering.





  std::unique_ptr<grid_map_msgs::msg::GridMap> message;
  message = grid_map::GridMapRosConverter::toMessage(elevationMap_);
  costMapPub_->publish(std::move(message));
  elevationMap_.clearAll();
  mapMtx_.unlock();
  const std::chrono::duration<double> durationOfProjection = EndProjection - methodStartTime;
  const std::chrono::duration<double> durationOfSegmentation = EndSegmentation - EndProjection;
  const std::chrono::duration<double> durationOfClustering = EndClustering - EndSegmentation;
  double durationOfProjectionMS = 1000 * durationOfProjection.count();
  double durationOfSegmentationMS = 1000 * durationOfSegmentation.count();
  double durationOfClusteringMS = 1000 * durationOfClustering.count();

  RCLCPP_INFO(get_logger(), "Done processing number of non ground grid found is %ld, Projection Step took %f ms, segmentation Step Took %f ms, clustering (Num of clusters %ld) Step Took %f ms.",C_N_.size(),durationOfProjectionMS,durationOfSegmentationMS,Clusters_.size(),durationOfClusteringMS);
  C_N_.clear();
  Clusters_.clear();
}




double TraversabilityAnalysis::NormalPDF(double x, double mean, double variance) {
    return (1.0 / sqrt(2.0 * M_PI * variance)) * exp(-0.5 * pow((x - mean) / sqrt(variance), 2));
}


void TraversabilityAnalysis::FloodFill(grid_map::Index index,std::vector<NonGroundGrid*> *cluster,int color)
{
    // Base cases 
    if (index(0) < 0 || index(0) >= size_(0) || index(1) < 0 || index(1) >= size_(1)) return;
    auto& segmentation = elevationMap_.at(SEGMENTATIONLAYER,index);
    auto& RNG = elevationMap_.at(REFRENCENONGRIDLAYER,index);
    long int idx = RNG;
    if (segmentation==false || segmentation==2 || C_N_[idx].clustered) return;
    
    auto& colorGrid = elevationMap_.at(COLORLAYER,index);
    colorGrid = color;
    // Add Grid to cluster.
    NonGroundGrid* grid = &C_N_[idx];
    //RCLCPP_INFO(get_logger(), "Hello !! %d, %d, with idx=%ld",index(0),index(1),idx);
    grid->cluster = cluster;
    grid->clustered = true;
    cluster->push_back(grid);
    // Recursively call for north, south, east and west.
    FloodFill(grid_map::Index(index(0)+1,index(1)), cluster, color);
    FloodFill(grid_map::Index(index(0)-1,index(1)), cluster, color);
    FloodFill(grid_map::Index(index(0),index(1)+1), cluster, color);
    FloodFill(grid_map::Index(index(0),index(1)-1), cluster, color);

    // Recursively call for the 8 directions.
    // for (int k = -1; k < 2; ++k) {
    //       for (int l = -1; l < 2; ++l) {
    //         if (k==0 && l==0) continue;
    //         FloodFill(grid_map::Index(index(0)+k,index(1)+l),cluster);
            
    //       }
    //   }


}







}// End namespace 

