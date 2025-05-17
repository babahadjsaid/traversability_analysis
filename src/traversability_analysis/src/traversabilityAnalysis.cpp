
#include "traversability_analysis/traversabilityAnalysis.hpp"




   

namespace traversability_analysis{

TraversabilityAnalysis::TraversabilityAnalysis(std::shared_ptr<rclcpp::Node> nodeHandle)
: nodeHandle_(nodeHandle),
  ParamServer(nodeHandle),
  elevationMap_({MAXHEIGHTLAYER,MINHEIGHTLAYER,MEANHEIGHTLAYER,SEGMENTATIONLAYER,REFRENCENONGRIDLAYER,COLORLAYER,GRIDSPOINTCLOUD,CATIGORISATION}),
  Clusters_(),
  robotPose_(nodeHandle_,POSE_TOPIC),
  kernel_dx_(3,3),
  kernel_dy_(3,3),
  BenchmarkTiming_(""),
  visualLoger_(L_DIR,L_FN)
{
  
  pointCloudSub_ = nodeHandle_->create_subscription<sensor_msgs::msg::PointCloud2>(PC_TOPIC, 1,std::bind(&TraversabilityAnalysis::PointCloudHandler, this, std::placeholders::_1));
  costMapPub_ = nodeHandle_->create_publisher<nav2_msgs::msg::Costmap>(CM_TOPIC, 1); //nav_msgs::msg::OccupancyGrid
  GlobalcostMapPub_ = nodeHandle_->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 1); //nav2_msgs::msg::Costmap
  markerPub_ = nodeHandle_->create_publisher<visualization_msgs::msg::Marker>("/clusterNames", 1);
  // if (log_level && std::string(log_level) == "DEBUG") {
    gridMapPub_ = nodeHandle_->create_publisher<grid_map_msgs::msg::GridMap>(GM_TOPIC, 1); //nav2_msgs::msg::Costmap
    // }
  
  elevationMap_.clearAll();
  elevationMap_.resetTimestamp();
  elevationMap_.setFrameId(MAP_FRAME);
  elevationMap_.setGeometry(grid_map::Length(L_MAP_LENGTH,L_MAP_LENGTH),CELL_RESOLUTION,grid_map::Position(0,0));
  size_ = elevationMap_.getSize();
  Colors_ = {0,70,140,200,255};
  message_ = std::make_shared<nav2_msgs::msg::Costmap>();
  message_->data.resize(size_(0) * size_(1),NO_INFORMATION);
  message_->metadata.size_y = size_(0);
  message_->metadata.size_x = size_(1);
  message_->metadata.resolution = CELL_RESOLUTION;
  // The origin is done like this to leave some kind of margin for the global map, the margin is the size of the local map.
  // Set the global map origin slightly offset (1.9 instead of 2.0) to create a margin around the local map, allowing extra space for expansion if the robot moves beyond current boundaries.
  Position origin = {- (size_(1)/1.9f) * CELL_RESOLUTION,- (size_(0)/1.9f) * CELL_RESOLUTION};
  int num_max_cell = size_(1) * CELL_RESOLUTION/GLOBAL_MAP_RES;
  globalCostmap_ = new OccupancyGrid(GLOBAL_MAP_HEIGHT,GLOBAL_MAP_WIDTH,GLOBAL_MAP_RES,origin,num_max_cell,GLOBAL_MAP_INCR);
  kernel_dx_ << -1, -2, -1, 0, 0, 0, 1, 2, 1;// dx
  kernel_dy_ << 1, 0, -1, 2, 0, -2, 1, 0, -1;// dy
  Clusters_.elevationMap_ = &elevationMap_;
  Clusters_.robotPose_ = &robotPose_;
  
}
//this should be turned to Cache 

/**
 * @brief Handles the incoming point cloud message and performs traversability analysis.
 * 
 * This function is called whenever a new point cloud message is received. It checks if the point cloud has a corresponding robot pose at the beginning, and if so, it performs the following steps:
 * 1. Extracts the robot's orientation from the pose message.
 * 2. Extracts the robot's twist from the odometry message.
 * 3. Extracts the robot's covariance matrix from the pose message.
 * 4. Extracts the robot's position from the pose message.
 * 5. Updates the receivedPose_ flag to indicate that a valid pose has been received.
 * 
 * If the robot's twist exceeds a certain threshold, the function ignores the current frame and returns.
 * 
 * The function then removes any clusters that are marked for deletion from the Clusters_ vector.
 * 
 * If this is not the first pose received and a valid pose has been received, the function calculates the displacement between the previous pose and the current pose. It then updates the position of each cluster in the Clusters_ vector based on this displacement.
 * 
 * The function converts the point cloud message to a pcl::PointCloud object and initializes the map layers.
 * 
 * The function then performs the following steps in sequence:
 * 1. Calls the MapProjection function to project the point cloud onto the elevation map.
 * 2. Calls the GroundSegmentation function to segment the ground points from the non-ground points.
 * 3. Calls the NonGroundGridClustering function to cluster the non-ground points into grids.
 * 4. Calls the CostCalculation function to calculate the cost for each grid.
 * 5. Calls the BuildCostMap function to build the local cost map using the calculated costs.
 * 
 * The function converts the elevation map to a grid_map_msgs::msg::GridMap message and publishes it.
 * 
 * The function publishes the cost map.
 * 
 * The function clears the elevation map and updates the previous pose and receivedPose_ flags.
 * 
 * Finally, the function clears the C_N_ vector and unlocks the map mutex.
 * 
 * @param pointCloudMsg The incoming point cloud message.
 */


void TraversabilityAnalysis::PointCloudHandler(sensor_msgs::msg::PointCloud2::SharedPtr pointCloudMsg){
  const auto methodStartTime = std::chrono::system_clock::now();
  mapMtx_.lock();
  robotPose_.UpdatePose(rclcpp::Time(pointCloudMsg->header.stamp,RCL_ROS_TIME));
  mapFrame = pointCloudMsg->header.frame_id;
  if (robotPose_.IsBigVibration())
  {
    mapMtx_.unlock();
    return;
  }
  
  Clusters_.RemoveOldClusters();
  

  if(!robotPose_.firstPose_ && robotPose_.receivedPose_){
    
    Clusters_.PredictClusterPointMass();
  }
  
  
  pcl::PointCloud<PointType>::Ptr pointCloud(new pcl::PointCloud<PointType>());
  pcl::moveFromROSMsg(*pointCloudMsg, *pointCloud);
  InitMapLayers();
  BenchmarkTiming_.str("");//turn this to a class.
  BenchmarkFunction(this,&TraversabilityAnalysis::MapProjection,"Map Projection",pointCloud);
  pointCloud->clear();
  // for each pc in gridsPointClouds_ concatinate them to the pointCloud.
  // for (auto &&pc : gridsPointClouds_)
  // {
  //   *pointCloud += pc;
  // }
  // visualLoger_.saveFrame(*pointCloud);////////////////////////
  BenchmarkFunction(this,&TraversabilityAnalysis::GroundSegmentation,"Ground Segmentation");
  BenchmarkFunction(this,&TraversabilityAnalysis::NonGroundGridClustering, "Non-Ground GridClustering ");
  BenchmarkFunction(this,&TraversabilityAnalysis::CostCalculation,"Cost Calculation");
  BenchmarkFunction(this,&TraversabilityAnalysis::BuildCostMap,"Build local Costmap",message_);
  // std::cout << BenchmarkTiming_.str() << std::endl;
  //////////////////////////////////////////
  // visualLoger_.addClustersField("Current Clusters",Clusters_);
  // visualLoger_.addSceneField("Robot Pose",robotPose_.serializePoseToJson());
  std::unique_ptr<grid_map_msgs::msg::GridMap> message;
  message = grid_map::GridMapRosConverter::toMessage(elevationMap_);
  gridMapPub_->publish(std::move(message));
  costMapPub_->publish(*message_);
  
  robotPose_.UpdatePrev();
  
  auto result = grid_map::GridMapCvConverter::toImage<unsigned short, 4>(elevationMap_, CATIGORISATION, CV_16UC4, 0.0f, 1.0f, Frameimage_);
  // visualLoger_.saveImage(Frameimage_);////////////////////////
  elevationMap_.clearAll();
  gridsPointClouds_.clear();
  C_N_.clear();
  mapMtx_.unlock();
  // visualLoger_.wrapUpScene();////////////////////////
  const std::chrono::duration<double> durationOfFunction = std::chrono::system_clock::now() - methodStartTime;
  double durationOfFunctionMS = 1000 * durationOfFunction.count();
  // std::cout <<"The function  Took " << durationOfFunctionMS <<" ms."<<std::endl;

}

void TraversabilityAnalysis::InitMapLayers(){
  elevationMap_[MINHEIGHTLAYER].setConstant(MAX_HEIGHT);
  elevationMap_[MAXHEIGHTLAYER].setZero();
  elevationMap_[MEANHEIGHTLAYER].setZero();
  elevationMap_[GRIDSPOINTCLOUD].setConstant(-1);
  elevationMap_[REFRENCENONGRIDLAYER].setConstant(-1);
  elevationMap_[CATIGORISATION].setZero();
}

void TraversabilityAnalysis::MapProjection(pcl::PointCloud<PointType>::Ptr pointCloud){
  
  #pragma omp parallel for num_threads(5)
  for (unsigned int i = 0; i < pointCloud->size(); ++i)
  {
    auto& point = pointCloud->points[i];
    grid_map::Index index;
    grid_map::Position position(point.x, point.y);  
    if (!elevationMap_.getIndex(position, index)) {
      continue;
    }
    
    auto& max_height = elevationMap_.at(MAXHEIGHTLAYER,index); 
    auto& min_height = elevationMap_.at(MINHEIGHTLAYER,index);
    auto& mean_height = elevationMap_.at(MEANHEIGHTLAYER,index);
    auto& RTGPC = elevationMap_.at(GRIDSPOINTCLOUD,index);
    if (point.z >MAX_HEIGHT)
    {
      if (RTGPC==-1)// cell was not visited before.
      #pragma omp critical
      {
        pcl::PointCloud<PointType> pc;
        pc.points.push_back(point);
        RTGPC = gridsPointClouds_.size();
        gridsPointClouds_.push_back(pc);
      }
      else
      #pragma omp critical
      {
        gridsPointClouds_[RTGPC].points.push_back(point);
      }
      continue;
    }
    if (point.z>T_HIGH && point.z<1.2 && point.intensity > LB && point.intensity < UB )
    #pragma omp critical
    {
      point.z = T_HIGH - 0.001;
      
    }
    if (RTGPC==-1)// cell was not visited before.
    {
        #pragma omp critical
      {
        pcl::PointCloud<PointType> pc;
        pc.points.push_back(point);
        RTGPC = gridsPointClouds_.size();
        gridsPointClouds_.push_back(pc);
        max_height = min_height = mean_height = point.z;// don't forget to include the intensity.
      }
    continue;
    }
    #pragma omp critical
    {
      gridsPointClouds_[RTGPC].points.push_back(point);
      mean_height += point.z;
      if (max_height < point.z)
          max_height = point.z;
      if (min_height > point.z)
          min_height = point.z;
    }


  }

}

void TraversabilityAnalysis::GroundSegmentation(){
  auto& mean_heightLayer = elevationMap_[MEANHEIGHTLAYER];
  auto& segmentationLayer = elevationMap_[SEGMENTATIONLAYER];
  
  // #pragma omp parallel for num_threads(5)
  for (int i = 0; i < size_(0); i++)
  {
    // #pragma omp parallel for num_threads(5)
    for (int j = 0; j < size_(1); j++)
    {
      float& segmentation = elevationMap_.at(SEGMENTATIONLAYER,grid_map::Index(i,j));
      float& max_height   = elevationMap_.at(MAXHEIGHTLAYER,grid_map::Index(i,j));
      float& min_height   = elevationMap_.at(MINHEIGHTLAYER,grid_map::Index(i,j));
      float& mean_height  = elevationMap_.at(MEANHEIGHTLAYER,grid_map::Index(i,j));
      float& RNG          = elevationMap_.at(REFRENCENONGRIDLAYER,grid_map::Index(i,j));
      float& RTGPC        = elevationMap_.at(GRIDSPOINTCLOUD,grid_map::Index(i,j));

      if (min_height==MAX_HEIGHT) // if the cell was not visited. means no information.
      {
        min_height = max_height = mean_height = 0;
        segmentation = 2; // no information.
        continue;
      }
      mean_height /= gridsPointClouds_[RTGPC].points.size();
      
      segmentation = ((max_height - min_height) >= T_DIFF || max_height >= T_HIGH || min_height <= T_LOW);    
    }
    
  }
  
   // #pragma omp parallel for num_threads(5)
  for (int i = 0; i < size_(0); i++)
  {
    // #pragma omp parallel for num_threads(5)
    for (int j = 0; j < size_(1); j++)
    {
      float& segmentation = elevationMap_.at(SEGMENTATIONLAYER,grid_map::Index(i,j));
      float& cat          = elevationMap_.at(CATIGORISATION,grid_map::Index(i,j));
      float& RNG          = elevationMap_.at(REFRENCENONGRIDLAYER,grid_map::Index(i,j));
      bool allZeros = true, allOnes = true;
      Eigen::MatrixXf segs, means;
      double dx=0,dy=0;
      if (!(i>0 && i < size_(0)-1 && j>0 && j < size_(1)-1)) goto AddToNonGrid;
      segs = segmentationLayer.block(i-1,j-1,3,3);
      means = mean_heightLayer.block(i-1,j-1,3,3);
      for (int k = 0; k < 3; ++k) {
          for (int l = 0; l < 3; ++l) {
            dx += means(k,l) * kernel_dx_(k,l);
            dy += means(k,l) * kernel_dy_(k,l);
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
      
      
      if(allOnes && segmentation==2)
            segmentation = allOnes;  
      if (allZeros && segmentation == 2)
            segmentation = !allZeros;
      
      if (segmentation != 2)
      {
        cat = pow(dx,2);
        cat += pow(dy,2);
        cat = sqrt(cat);
        cat = 0.5-(0.5 / exp(cat));
      }
      
      AddToNonGrid:
      
      
      if (segmentation==1)
      {
        grid_map::Index idx(i,j);
        NonGroundGrid tmp;
        RNG = C_N_.size();
        tmp.index = idx;
        C_N_.push_back(tmp);
      }
      }
    
  }
  

}

void TraversabilityAnalysis::NonGroundGridClustering(){
  
  for (size_t i = 0; i < C_N_.size(); i++)
  {
    if (C_N_[i].clustered) continue;
    Cluster new_cluster;
    FloodFill(C_N_[i].index, C_N_[i].index, &new_cluster);
    

    if (new_cluster.grids.size() < NUM_GRIDS_MIN)
    {
      for (auto &&grid : new_cluster.grids)
      {
        auto& segmentation = elevationMap_.at(SEGMENTATIONLAYER,grid->index);
        auto& RNG          = elevationMap_.at(REFRENCENONGRIDLAYER,grid->index);
        auto& colorGrid    = elevationMap_.at(COLORLAYER,grid->index);
        float& cat         = elevationMap_.at(CATIGORISATION,grid->index);
        segmentation = 0;
        RNG = -1;
        colorGrid = 0;
        cat = 0.8;
        grid->clustered = false;// edit this later.
      }
      new_cluster.Status = tODELETE;
      continue;
    }
    
    new_cluster.mean_height /= new_cluster.grids.size();
    new_cluster.Point_mass /= (float) new_cluster.grids.size();

    if(!robotPose_.firstPose_ && robotPose_.receivedPose_){
      if(Clusters_.MatchCluster(new_cluster)) continue;
    }
    new_cluster.color = Colors_[Clusters_.size()%NUM_COLORS];
    Clusters_.addCluster(new_cluster);
    
  } 
  
}

void TraversabilityAnalysis::CostCalculation(){
  // printf("CostCalculation\n");
   for (size_t i = 0; i < Clusters_.size(); i++) {
    // printf("CostCalculation\n");
    auto &cluster = Clusters_[i];
    if(!cluster.UpdateStatus()) continue;
    // printf("Cluster %d\n",i);
    
    double H_d = cluster.max_height - cluster.min_height;
    
    if(cluster.mean_height>=0)                        cluster.H_f = std::max(H_d,cluster.mean_height);
      else                                            cluster.H_f = std::min(-H_d,cluster.mean_height);
    
    float is_vertical, ratioBigToSmallest, ratioBigTomiddle;
    PcaAnalysis(cluster,is_vertical,ratioBigToSmallest,ratioBigTomiddle);
    // printf("is_vertical: %f, ratioBigToSmallest: %f, ratioBigTomiddle: %f\n",is_vertical,ratioBigToSmallest,ratioBigTomiddle);
    // float ratio = ratioBigTomiddle/ratioBigToSmallest;
    cluster.isVertical = (!std::signbit(cluster.H_f));
    cluster.ratioBigToSmallest = ratioBigToSmallest;
    cluster.ratioBigTomiddle = ratioBigTomiddle;
    cluster.isFlat = (ratioBigToSmallest > 100) || (ratioBigToSmallest > T_RATIO_BIG_OVER_SMALL && ratioBigTomiddle > (1/T_RATIO_BIG_OVER_MIDDLE_MIN));
    if (!std::signbit(cluster.H_f))
    {
      if (is_vertical) cluster.category_count[oBSTACLES] += 1;
      else
      {
        if ((ratioBigToSmallest > 100) || (ratioBigToSmallest > T_RATIO_BIG_OVER_SMALL && ratioBigTomiddle > (1/T_RATIO_BIG_OVER_MIDDLE_MIN))) cluster.category_count[sLOPE] += 1;
        else cluster.category_count[oBSTACLES] += 1;
      }
      
    }else
    {
      if (is_vertical) cluster.category_count[pOTHOLE] += 1;
      else
      {
        if ((ratioBigToSmallest > 100) || (ratioBigToSmallest > T_RATIO_BIG_OVER_SMALL && ratioBigTomiddle > (1/T_RATIO_BIG_OVER_MIDDLE_MIN))) cluster.category_count[nEGATIVESLOPE] += 1;
        else cluster.category_count[pOTHOLE] += 1;
      }
    }
    float cost;
    cluster.Type = cluster.getCat();
    switch (cluster.Type)
    {
    case pOTHOLE: 
    case oBSTACLES:
      cost = 1.0;
      break;
    case sLOPE:      
    case nEGATIVESLOPE:
      EstimateAngle(cluster);
      if (abs(cluster.angle)>= MAXANGLE)
      {
        if (cluster.Type == sLOPE) cluster.category_count[oBSTACLES] += 1;
        else cluster.category_count[pOTHOLE] += 1;
        cost = 1.0;
        break;
      }
      cost = abs(cluster.angle) / (2*MAXANGLE);
      break;
    
    default:
      break;
    }
    // printf("Cluster %d\n",i);
    //std::cout << "the color is "<<Colors_[cluster.Type] << " and type: "<< GetCategoryName(cluster.Type)<<std::endl;
    
    for (auto &&grid : cluster.grids)
    {
      grid_map::Position position;
      if (!elevationMap_.getPosition(grid->index, position)) continue;
      auto& cat  = elevationMap_.at(CATIGORISATION,grid->index);
      auto& color = elevationMap_.at(COLORLAYER,grid->index);
      color = Colors_[cluster.Type]; // 
      
      
      if (cost == 1)
      {
        cat = 1.0;
        continue;
      }
      cat += cost;
      
    }
    if (cluster.Status == oLD) cluster.Status = tODELETE;

    
  }

}







void TraversabilityAnalysis::FloodFill(grid_map::Index index,grid_map::Index prevIndex, Cluster *cluster)
{
    // Base cases 
    if (index(0) < 0 || index(0) >= size_(0) || index(1) < 0 || index(1) >= size_(1)) return;
    auto& segmentation = elevationMap_.at(SEGMENTATIONLAYER,index);
    auto& RNG = elevationMap_.at(REFRENCENONGRIDLAYER,index);
    auto& mean_height_Grid = elevationMap_.at(MEANHEIGHTLAYER,index);
    auto& mean_height_prevGrid = elevationMap_.at(MEANHEIGHTLAYER,prevIndex);
    auto& max_height_Grid = elevationMap_.at(MAXHEIGHTLAYER,index);
    auto& max_height_prevGrid = elevationMap_.at(MAXHEIGHTLAYER,prevIndex);
    long int idx = RNG;
    if (segmentation==false || segmentation==2 || C_N_[idx].clustered || abs(max_height_Grid - max_height_prevGrid) > 0.6 || abs(mean_height_Grid - mean_height_prevGrid) > T_SEG ) return; // ; TODO
    auto& colorGrid = elevationMap_.at(COLORLAYER,index);
    auto& min_height_Grid = elevationMap_.at(MINHEIGHTLAYER,index);
    auto& RTGPC = elevationMap_.at(GRIDSPOINTCLOUD,index);
    // Add Grid to cluster.
    NonGroundGrid* grid = &C_N_[idx];
    grid->cluster = cluster;
    grid->clustered = true;
    grid->idx = idx;
    cluster->grids.push_back(grid);
    cluster->mean_height += mean_height_Grid;
    grid_map::Position pos;
    elevationMap_.getPosition(grid->index,pos);
    cluster->Point_mass += pos;
    if (RTGPC != -1)
    {
      cluster->pc += gridsPointClouds_[RTGPC];
    }
    
    if (cluster->grids.size()==1)
    {
      cluster->max_height = max_height_Grid;
      cluster->min_height = min_height_Grid;
    }else
    {
      if (cluster->max_height < max_height_Grid)
      {
        cluster->max_height = max_height_Grid;
      }
      if (cluster->min_height > min_height_Grid)
      {
        cluster->min_height = min_height_Grid;
      }
    }

    // Recursively call for the 8 directions.
    for (int k = -1; k < 2; ++k) {
          for (int l = -1; l < 2; ++l) {
            if (k==0 && l==0) continue;
            FloodFill(grid_map::Index(index(0)+k,index(1)+l), index, cluster);
          }
      }
   


}

void TraversabilityAnalysis::PcaAnalysis(Cluster &cluster, float &is_vertical, float &ratioBigToSmallest, float &ratioBigTomiddle){
  long int n = cluster.pc.points.size();
  float x_m = 0, y_m = 0, z_m = 0;
  for (auto &&point : cluster.pc.points)
  {
    x_m += point.x; y_m += point.y; z_m += point.z;
  }
  x_m /= n;
  y_m /= n;
  z_m /= n;
  float a_1=0, a_2=0, a_3=0, a_4=0, a_5=0, a_6=0;
  for (auto &&point : cluster.pc.points)
  {
    float t1 = point.x - x_m, t2 = point.y - y_m, t3 = point.z - z_m;
    a_1 += t1 *t1;a_2 += t1 *t2;a_3 += t1 *t3;
    a_4 += t2 *t2;a_5 += t2 *t3;a_6 += t3 *t3;

  }
  
  Eigen::Matrix3f S;
  S << a_1, a_2, a_3,
      a_2, a_4, a_5,
      a_3, a_5, a_6; 
  
  S/= (n-1);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(S);
  Eigen::Vector3f eigenvalues = solver.eigenvalues().real();
  Eigen::Matrix3f eigenvectors = solver.eigenvectors().real();
  
  cluster.Plane.x = x_m;
  cluster.Plane.y = y_m;
  cluster.Plane.z = z_m;
  cluster.Plane.normal_x = eigenvectors.col(0)(0);
  cluster.Plane.normal_y = eigenvectors.col(0)(1);
  cluster.Plane.normal_z = eigenvectors.col(0)(2);
  cluster.eigenvectors = eigenvectors;
  cluster.eigenvalues = eigenvalues;
  float dot_product = eigenvectors.col(0)(2);
  ratioBigToSmallest = eigenvalues[2] / eigenvalues[0];
  ratioBigTomiddle = eigenvalues[1] / eigenvalues[0];// Check this out !!!
  cluster.Roughness = is_vertical = std::abs(dot_product) < VERTICAL_THRESHOLD;

}

void TraversabilityAnalysis::EstimateAngle(Cluster &cluster){
    Eigen::Vector3f point = {cluster.Plane.x,cluster.Plane.y,cluster.Plane.z};
    Eigen::Vector3f normal = {cluster.Plane.normal_x,cluster.Plane.normal_y,cluster.Plane.normal_z};
    const int CloudSize = cluster.pc.points.size();
    std::uniform_int_distribution<int> distribution(0, CloudSize);
    float A = normal(0);
    float B = normal(1);
    float C = normal(2);
    float D = -normal.dot(point);
    int NumofInliers = 0;
    std::vector<int> BestInliers;
    #pragma omp parallel for num_threads(5)
    for (int i = 0; i < CloudSize; ++i) {
        float distance = PointToPlaneDistance(cluster.pc.points[i], A, B, C, D);
        if (distance <= T_SEG)
        #pragma omp critical
        {
          BestInliers.push_back(i);
        }
    }
    int nt = BestInliers.size();
    float a1 = 0, a2 = 0, a3 = 0, a4 = 0, a5 = 0;
    float b1 = 0, b2 = 0, b3 = 0;
    for (auto&& i : BestInliers) {
        float x = cluster.pc.points[i].x;
        float y = cluster.pc.points[i].y;
        float z = cluster.pc.points[i].z;
        a1 += x * x;
        a2 += x * y;
        a3 += x;
        a4 += y * y;
        a5 += y;
        b1 += z * x;
        b2 += z * y;
        b3 += z;
    }

    // Singular value decomposition
    Eigen::Matrix3f MatA;
    MatA << a1, a2, a3,
            a2, a4, a5,
            a3, a5, nt;
    Eigen::Vector3f VecB(b1, b2, b3);
    Eigen::Vector3f VecC = MatA.colPivHouseholderQr().solve(VecB);

    // Calculate slope angle

    cluster.angle = acos(1 / sqrt(pow(VecC(0), 2) + pow(VecC(1), 2) + 1));
   
    // std::cout << "The angle is: "<<cluster.angle * (180.0/M_PI)<<std::endl;
    
}

void TraversabilityAnalysis::savePointCloud(const pcl::PointCloud<PointType> *cloud, const std::string& filename) {
    pcl::PCDWriter writer;
    writer.writeBinaryCompressed(filename, *cloud);
}


void TraversabilityAnalysis::PubGlobalMap(){
    rclcpp::Rate rate(5);
    while (rclcpp::ok()){
      
      nav_msgs::msg::OccupancyGrid* tmpMap = new nav_msgs::msg::OccupancyGrid();
      globalCostmap_->MapMtx_.lock();
      *tmpMap = *globalCostmap_->Map_;
      // Variance_ = Eigen::MatrixXd::Constant(height, width, 1.0);
      globalCostmap_->MapMtx_.unlock();
      tmpMap->info.origin.position.z =  robotPose_.currentPose_.z() - 0.662051; 
      // for (auto &&point : tmpMap->data)
      // {
      //   if (point == -1) continue;
      //   // cout somthing here.
      //   std::cout << "point: " << point << std::endl;
      //   point = LETHAL_OBSTACLE * point;
      // }
      GlobalcostMapPub_->publish(*tmpMap);
      globalCostmap_->CheckAndExpandMap(Position(robotPose_.currentPose_.x(),robotPose_.currentPose_.y()));
      /* like this const std::chrono::duration<double> durationOfFunction = std::chrono::system_clock::now() - methodStartTime;
      double durationOfFunctionMS = 1000 * durationOfFunction.count();
      std::cout<<"The function " << function_name << " Took " << durationOfFunctionMS <<" ms ";
      calculate the 3 times, the time it took to lock, the time it took to unlock, and the time it took to publish.*/
      

      rate.sleep();
    }
}

void TraversabilityAnalysis::BuildCostMap(nav2_msgs::msg::Costmap::SharedPtr localCostmap ){
    localCostmap->header = std_msgs::msg::Header();
    localCostmap->header.frame_id = mapFrame;
    localCostmap->metadata.origin.position.x = - (size_(1)/2.0) * CELL_RESOLUTION;// Since the map frame is robot centric we need to look for 
    localCostmap->metadata.origin.position.y = - (size_(0)/2.0) * CELL_RESOLUTION; 
    Position origin;
    origin.x() = robotPose_.currentPose_.x();
    origin.y() = robotPose_.currentPose_.y();
    // std::cout << "Current Robot pose: "<< robotPose_.CurrentcovarianceM_ << std::endl;
    globalCostmap_->MapMtx_.lock();
    for (size_t i = 0; i < size_(0); i++)
    {
      for (size_t j = 0; j < size_(1); j++)
        {
          float& segmentation = elevationMap_.at(SEGMENTATIONLAYER,grid_map::Index(i,j));
          float& cat          = elevationMap_.at(CATIGORISATION,grid_map::Index(i,j));
          grid_map::Position positions;
          elevationMap_.getPosition(grid_map::Index(i,j),positions);
          int x = size_(1)-j-1;
          int y = size_(0)-i-1;
          auto& cost = localCostmap->data.at(x* size_(1) + y);
          int occupancy;
          if (segmentation == 2) {
            occupancy = NO_INFORMATION; 
            cost = 255;
            continue;
            }
          if (segmentation == 1) {
            occupancy = LETHAL_OBSTACLE;
            if (cat == 1.0) {cost = 254; occupancy = LETHAL_OBSTACLE;}
            else {cost = cat * 252;      occupancy = cat * LETHAL_OBSTACLE;}
            }
          if (segmentation == 0) {
            occupancy = cat * MAX_NON_OBSTACLE;
            cost = cat * 252;
            }
          Position pos;
          // create 2x2 matrix rotation yaw.
          Eigen::Matrix2f rotation = Eigen::Rotation2Df(robotPose_.currentPose_.yaw()).toRotationMatrix();
          pos = positions;
          pos = origin + (pos * rotation); 
          Eigen::Matrix2d CovarianceGlobal = globalCostmap_->ComputeGlobalCovariance(robotPose_.currentPose_,robotPose_.CurrentcovarianceM_,pos);
          
          // update the corresponding cells in the global map.
          globalCostmap_->UpdateCell(pos, cat, CovarianceGlobal);
        }
   }
    globalCostmap_->MapMtx_.unlock();


}




}// End namespace 





// TODO: 
// 1. Add a Testing Class. (Pending)
// 2. Add the benchmarking class. (Pending)
// 3. For each cell in the projection step add the points higher then T_HIGH to the consideration. (Done)
// 4. Make a better scene segmentation. (Pending)
