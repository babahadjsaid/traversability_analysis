
#include "traversability_analysis/traversabilityAnalysis.hpp"




  

namespace traversability_analysis{

TraversabilityAnalysis::TraversabilityAnalysis(std::string node_name, const rclcpp::NodeOptions & options)
: ParamServer(node_name, options),
  elevationMap_({MAXHEIGHTLAYER,MINHEIGHTLAYER,MEANHEIGHTLAYER,SEGMENTATIONLAYER,REFRENCENONGRIDLAYER,COLORLAYER,GRIDSPOINTCLOUD,CATIGORISATION}),
  kernel_(3,3),
  generator_(randomDevice_()),
  firstPose_(true),
  receivedPose_(false),
  BenchmarkTiming_("")
{
  pointCloudSub_ = create_subscription<sensor_msgs::msg::PointCloud2>(PC_TOPIC, 1,std::bind(&TraversabilityAnalysis::PointCloudHandler, this, std::placeholders::_1));
  robotPoseSubscriber_ = create_subscription<nav_msgs::msg::Odometry>(POSE_TOPIC, qos_imu,std::bind(&TraversabilityAnalysis::OdometryHandler, this, std::placeholders::_1));
  costMapPub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(CM_TOPIC, 1); //nav2_msgs::msg::Costmap
  GlobalcostMapPub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 1); //nav2_msgs::msg::Costmap
  
  float side_length = RADIUS * sqrt(2);
  elevationMap_.clearAll();
  elevationMap_.resetTimestamp();
  elevationMap_.setFrameId(MAP_FRAME);
  elevationMap_.setGeometry(grid_map::Length(side_length,side_length),CELL_RESOLUTION,grid_map::Position(0,0));
  size_ = elevationMap_.getSize();
  Colors_ = linspace(0,255,5);
  message_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  message_->data.resize(size_(0) * size_(1),NO_INFORMATION);
  message_->info.height = size_(0);
  message_->info.width = size_(1);
  message_->info.resolution = CELL_RESOLUTION;
  Position origin = {- (size_(1)/2.0f) * CELL_RESOLUTION,- (size_(0)/2.0f) * CELL_RESOLUTION};
  int num_max_cell = (size_(1)) * CELL_RESOLUTION/GLOBAL_MAP_RES;
  globalCostmap_ = new OccupancyGrid(GLOBAL_MAP_HEIGHT,GLOBAL_MAP_WIDTH,GLOBAL_MAP_RES,origin,num_max_cell,GLOBAL_MAP_INCR);
  
  
}
void TraversabilityAnalysis::OdometryHandler(nav_msgs::msg::Odometry::SharedPtr poseMsg){
  poseMtx_.lock();
  currentOdom_ = *poseMsg;
  poseMtx_.unlock();
}
void TraversabilityAnalysis::PointCloudHandler(sensor_msgs::msg::PointCloud2::SharedPtr pointCloudMsg){
  // Check if point cloud has corresponding robot pose at the beginning
  
  mapMtx_.lock();
  poseMtx_.lock();
  auto oldestPoseTime = rclcpp::Time(currentOdom_.header.stamp,RCL_ROS_TIME);
  auto currentPointCloudTime = rclcpp::Time(pointCloudMsg->header.stamp,RCL_ROS_TIME);
  if (abs(currentPointCloudTime.seconds() - oldestPoseTime.seconds()) <0.01 ) {
    
    
    tf2::Quaternion orientation;
    tf2::fromMsg(currentOdom_.pose.pose.orientation, orientation);
    double roll, pitch, yaw;
    tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
    
    currentTwist_= currentOdom_.twist.twist;
    
    int index=-1;
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            covarianceMatrix_(i, j) = currentOdom_.pose.covariance[++index];
        }
    }
    currentPose_.x() = currentOdom_.pose.pose.position.x;
    currentPose_.y() = currentOdom_.pose.pose.position.y;
    currentPose_.z() = yaw;
    receivedPose_ = true;
    
  }
  poseMtx_.unlock();
  mapFrame = pointCloudMsg->header.frame_id;
  if (abs(currentTwist_.angular.x) >= 0.4|| abs(currentTwist_.angular.y) >= 0.4 || abs(currentTwist_.angular.z) >= 0.4 )
  {
    std::cout << "A frame is being ignored due to big vibrations " <<std::endl;
    previousPose_ = currentPose_;
    mapMtx_.unlock();
    return;
  }
  
   
  Clusters_.erase(std::remove_if(Clusters_.begin(), Clusters_.end(), [](traversability_analysis::Cluster& cluster) { 
    if (cluster.Status == tODELETE) return true;
    cluster.Status = oLD;
    return false;
    } ), Clusters_.end());

  if(!firstPose_ && receivedPose_){
    displacement_ =  previousPose_ - currentPose_;
    double c = cos(displacement_.z()), s = sin(displacement_.z());
    for (auto &&cluster : Clusters_)
    {
      grid_map::Position tmp, predicted;
      tmp.x() = cluster.Point_mass.x() + displacement_.x();
      tmp.y() = cluster.Point_mass.y() + displacement_.y();
      predicted.x() = tmp.x() * c + tmp.y() * -s;
      predicted.y() = tmp.x() * s + tmp.y() * c;
      cluster.Point_mass = predicted;
    }
  }
  
  
  pcl::PointCloud<PointType>::Ptr pointCloud(new pcl::PointCloud<PointType>());
  pcl::moveFromROSMsg(*pointCloudMsg, *pointCloud);
  InitMapLayers();
  //std::cout << "hello.."<< std::endl;
  BenchmarkTiming_.str("");//turn this to a class.
  BenchmarkFunction(this,&TraversabilityAnalysis::MapProjection,"Map Projection",pointCloud);
  BenchmarkFunction(this,&TraversabilityAnalysis::GroundSegmentation,"Ground Segmentation");
  BenchmarkFunction(this,&TraversabilityAnalysis::NonGroundGridClustering, "Non-Ground GridClustering ");
  BenchmarkFunction(this,&TraversabilityAnalysis::CostCalculation,"Cost Calculation");
  
  BenchmarkFunction(this,&TraversabilityAnalysis::BuildCostMap,"Build local Costmap",message_);
  
  std::cout << BenchmarkTiming_.str() << std::endl;
 
  
  //grid_map::GridMapRosConverter::toCostmap(elevationMap_,CATIGORISATION,0.0,1.0,message);
  costMapPub_->publish(*message_);
  elevationMap_.clearAll();
  if (receivedPose_)
  {
    previousPose_ = currentPose_;
    receivedPose_ = false;
    firstPose_ = false;
  }
  // SaveData();
  C_N_.clear();
  mapMtx_.unlock();
  
}

void TraversabilityAnalysis::InitMapLayers(){
  auto& min_heightLayer = elevationMap_[MINHEIGHTLAYER];
  auto& max_heightLayer = elevationMap_[MAXHEIGHTLAYER];
  auto& mean_heightLayer = elevationMap_[MEANHEIGHTLAYER];
  auto& RTGPCLayer = elevationMap_[GRIDSPOINTCLOUD];
  auto& RNGLayer = elevationMap_[REFRENCENONGRIDLAYER];
  auto& segmentLayer = elevationMap_[SEGMENTATIONLAYER];
  auto& catigorisationLayer = elevationMap_[CATIGORISATION];
  auto& colorLayer = elevationMap_[COLORLAYER];
  mean_heightLayer.setZero();
  max_heightLayer.setZero();
  min_heightLayer.setConstant(MAX_HEIGHT);
  RTGPCLayer.setConstant(-1);
  RNGLayer.setConstant(-1);
  catigorisationLayer.setZero();
}

void TraversabilityAnalysis::MapProjection(pcl::PointCloud<PointType>::Ptr pointCloud){
  
  #pragma omp parallel for num_threads(5)
  for (unsigned int i = 0; i < pointCloud->size(); ++i)
  {
    auto& point = pointCloud->points[i];
    grid_map::Index index;
    grid_map::Position position(point.y, -point.x);  
    if (!elevationMap_.getIndex(position, index) || point.z >MAX_HEIGHT) {
      continue;
    }
   
    auto& max_height = elevationMap_.at(MAXHEIGHTLAYER,index); 
    auto& min_height = elevationMap_.at(MINHEIGHTLAYER,index);
    auto& mean_height = elevationMap_.at(MEANHEIGHTLAYER,index);
    auto& RTGPC = elevationMap_.at(GRIDSPOINTCLOUD,index);
    
    // double pdf = NormalPDF(point.intensity, MEAN_GRASS, VARIANCE_GRASS);
      if (point.z>T_HIGH && point.z<1.2 && point.intensity > LB && point.intensity < UB )
      #pragma omp critical
      {
        point.z = T_HIGH - 0.001;
        
      }
      if (RTGPC==-1)
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
      float& cat          = elevationMap_.at(CATIGORISATION,grid_map::Index(i,j));

      if (min_height==MAX_HEIGHT) // if the cell was not visited. means no information.
      {
        min_height = max_height = mean_height = 0;
        segmentation = 2;
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
      if (segmentation == 2) continue;
      float& cat          = elevationMap_.at(CATIGORISATION,grid_map::Index(i,j));
      float& RNG          = elevationMap_.at(REFRENCENONGRIDLAYER,grid_map::Index(i,j));
      bool allZeros = true, allOnes = true;
      Eigen::MatrixXf segs;
      if (!(i>0 && i < size_(0)-1 && j>0 && j < size_(1)-1)) goto AddToNonGrid;

      kernel_ << -1, -2, -1, 0, 0, 0, 1, 2, 1;// dx
      cat = pow((mean_heightLayer.block(i-1,j-1,3,3).array() * kernel_.array()).sum(),2);
      kernel_ << 1, 0, -1, 2, 0, -2, 1, 0, -1;// dy
      cat += pow((mean_heightLayer.block(i-1,j-1,3,3).array() * kernel_.array()).sum(),2);
      cat = sqrt(cat);
      cat = 0.5-(0.5 / exp(cat));
       
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
    new_cluster.Point_mass = {0,0};
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
        grid->clustered = true;
      }
      continue;
    }
    
    new_cluster.mean_height /= new_cluster.grids.size();
    new_cluster.Point_mass /= (float) new_cluster.grids.size();

    if(!firstPose_ && receivedPose_){
      bool found = false;
      double diff = MAX_NUM_GRIDS;
      traversability_analysis::Cluster *matchedCluster;

      grid_map::Index predictedIndex,trueIndex,diffIndex; 
      
      elevationMap_.getIndex(new_cluster.Point_mass, trueIndex);
      for (size_t i = 0; i < Clusters_.size(); i++) {
        auto &cluster = Clusters_[i];
        
        if (cluster.Status == tODELETE) continue;
        if (!elevationMap_.getIndex(cluster.Point_mass, predictedIndex))
        {
          cluster.Status = tODELETE;
          continue;
        }
        diffIndex = trueIndex - predictedIndex;
        double ratio = cluster.grids.size() / (double) new_cluster.grids.size();
        if (abs(diffIndex(0)) <= MAX_NUM_GRIDS && abs(diffIndex(1)) <= MAX_NUM_GRIDS)
        {
          double tmp = sqrt(pow(diffIndex(0),2) + pow(diffIndex(1),2));
          if(diff > tmp) {
            diff = tmp;
            matchedCluster = &cluster;
            found = true;
          }
        }
        
      }
      if(found){
        matchedCluster->mean_height = new_cluster.mean_height;
        matchedCluster->max_height = new_cluster.max_height;
        matchedCluster->min_height = new_cluster.min_height;
        matchedCluster->grids = new_cluster.grids;
        matchedCluster->pc = new_cluster.pc;
        matchedCluster->Point_mass = new_cluster.Point_mass;
        matchedCluster->Status = uPTODATE;
        
        continue;
      }
    }
    new_cluster.color = Colors_[Clusters_.size()%NUM_COLORS];
    Clusters_.push_back(new_cluster);
    
  } 
  
}


void TraversabilityAnalysis::CostCalculation(){
  
   for (size_t i = 0; i < Clusters_.size(); i++) {
    auto &cluster = Clusters_[i];
    if (cluster.Status == tODELETE) continue;

    if (cluster.Status == nEw) cluster.Status = oLD;
    else if (cluster.Status == oLD){
      cluster.Status = tODELETE;
      continue;
    }
   
    
    float H_d = cluster.max_height - cluster.min_height;
    
    if(cluster.mean_height>=0)                        cluster.H_f = std::max(H_d,cluster.mean_height);
      else                                            cluster.H_f = std::min(-H_d,cluster.mean_height);
    
    if (T_NEG < cluster.H_f && cluster.H_f < T_POS)   cluster.category_count[oBSTACLES] += 1;
    
    if (cluster.H_f >= T_POS)
    {
      if (CalculateRoughness(cluster)) cluster.category_count[oBSTACLES] += 1;
      else cluster.category_count[sLOPE] += 1;
    } else { // T_NEG >= cluster.H_f 
      if (CalculateRoughness(cluster)) cluster.category_count[pOTHOLE] += 1;
      else cluster.category_count[nEGATIVESLOPE] += 1;;
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
        cost = 1.0;
        break;
      }
      cost = abs(cluster.angle) / (2*MAXANGLE);
      break;
    
    default:
      break;
    }
    
    std::cout << "the color is "<<Colors_[cluster.Type] << " and type: "<< GetCategoryName(cluster.Type)<<std::endl;
    for (auto &&grid : cluster.grids)
    {
      auto& cat  = elevationMap_.at(CATIGORISATION,grid->index);
      auto& color = elevationMap_.at(COLORLAYER,grid->index);
      color = Colors_[cluster.Type]; // cluster.color;
      
      if (cost == 1)
      {
        cat = 1.0;
        continue;
      }
      cat += cost;
      
    }
    
  }


}



float OccupancyGrid::cumulativeDistributionFunction(float x, float mean, float standardDeviation) {
  return 0.5 * erfc(-(x - mean) / (standardDeviation * sqrt(2.0)));
}


std::string TraversabilityAnalysis::GetCategoryName(ObjectsCategories categoryName){
  if (categoryName == oBSTACLES) return OBSTACLES;
  if (categoryName == sLOPE) return SLOPE ;
  if (categoryName == nEGATIVESLOPE) return NEGATIVESLOPE;
  if (categoryName == pOTHOLE) return POTHOLE;
  return GROUND;
}


void TraversabilityAnalysis::FloodFill(grid_map::Index index,grid_map::Index prevIndex, Cluster *cluster)
{
    // Base cases 
    if (index(0) < 0 || index(0) >= size_(0) || index(1) < 0 || index(1) >= size_(1)) return;
    auto& segmentation = elevationMap_.at(SEGMENTATIONLAYER,index);
    auto& RNG = elevationMap_.at(REFRENCENONGRIDLAYER,index);
    auto& max_height_Grid = elevationMap_.at(MAXHEIGHTLAYER,index);
    auto& max_height_prevGrid = elevationMap_.at(MAXHEIGHTLAYER,prevIndex);
    long int idx = RNG;
    if (segmentation==false || segmentation==2 || C_N_[idx].clustered || abs(max_height_Grid - max_height_prevGrid) > 0.4) return; // ; TODO
    auto& colorGrid = elevationMap_.at(COLORLAYER,index);
    auto& min_height_Grid = elevationMap_.at(MINHEIGHTLAYER,index);
    auto& mean_height_Grid = elevationMap_.at(MEANHEIGHTLAYER,index);
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

bool TraversabilityAnalysis::CalculateRoughness(Cluster &cluster){
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
  Eigen::EigenSolver<Eigen::Matrix3f> solver(S);
  Eigen::Vector3f eigenvalues = solver.eigenvalues().real();
  std::sort(eigenvalues.data(), eigenvalues.data() + eigenvalues.size());
  
  bool condition = ((eigenvalues[0]!=0) && (eigenvalues[1] / eigenvalues[0] >= T_RATIO));
  
  
  cluster.Roughness = !condition;
  return cluster.Roughness;

}

void TraversabilityAnalysis::EstimateAngle(Cluster &cluster){
    const auto Start = std::chrono::system_clock::now();
    // Constants and thresholds
    const int CloudSize = cluster.pc.points.size();
    std::uniform_int_distribution<int> distribution(0, CloudSize);

    // Initialize variables
    int NumofInliers = 0;
    std::vector<int> BestInliers;

    // RANSAC for plane segmentation
    for (int i = 0; i < T_ITERATIONS*CloudSize; ++i) {
        // Select three random points
        int n1 = distribution(generator_);
        int n2 = distribution(generator_);
        int n3 = distribution(generator_);
        PointType p1 = cluster.pc.points[n1], p2 = cluster.pc.points[n2], p3 = cluster.pc.points[n3];

        // Compute plane coefficients
        float Ai = (p2.y - p1.y) * (p3.z - p1.z) - (p3.y - p1.y) * (p2.z - p1.z);
        float Bi = (p3.x - p1.x) * (p2.z - p1.z) - (p2.x - p1.x) * (p3.z - p1.z);
        float Ci = (p2.x - p1.x) * (p3.y - p1.y) - (p3.x - p1.x) * (p2.y - p1.y);
        float Di = -Ai * p1.x - Bi * p1.y - Ci * p1.z;

        // Check inliers
        std::vector<int> inliers;
        #pragma omp parallel for num_threads(5)
        for (int j = 0; j < CloudSize; ++j) {
            float distance = PointToPlaneDistance(cluster.pc.points[j], Ai, Bi, Ci, Di);
            if (distance <= T_SEG) 
            #pragma omp critical
            {
                inliers.push_back(j);
            }
        }

        // Update best plane parameters
        if (inliers.size() >= NumofInliers) 
        
        {
            NumofInliers = inliers.size();
            BestInliers = inliers;
        }

        // Check termination condition
        const std::chrono::duration<double> current = std::chrono::system_clock::now() - Start;
        double currentMS = 1000 * current.count();
        if (NumofInliers / (CloudSize+0.0) > T_INLINERS || currentMS >= 10) {
            break;
        }
    }
    const auto RansacEnd = std::chrono::system_clock::now();

    // Least squares method for normal vector estimation
    int nt = NumofInliers;
    float a1 = 0, a2 = 0, a3 = 0, a4 = 0, a5 = 0;
    float b1 = 0, b2 = 0, b3 = 0;
    for (int k = 0; k < nt; ++k) {
        float x = cluster.pc.points[BestInliers[k]].x;
        float y = cluster.pc.points[BestInliers[k]].y;
        float z = cluster.pc.points[BestInliers[k]].z;
        a1 += x * x;
        a2 += x * y;
        a3 += x;
        a4 += y * y;
        a5 += y;
        b1 += z * x;
        b2 += z * y;
        b3 += z;
    }
    const auto leastEnd = std::chrono::system_clock::now();

    // Singular value decomposition
    Eigen::Matrix3f MatA;
    MatA << a1, a2, a3,
            a2, a4, a5,
            a3, a5, nt;
    Eigen::Vector3f VecB(b1, b2, b3);
    Eigen::Vector3f VecC = MatA.colPivHouseholderQr().solve(VecB);

    // Calculate slope angle
    float as = acos(1 / sqrt(pow(VecC(0), 2) + pow(VecC(1), 2) + 1));
    cluster.angle = as;
    
    // continue the inclusion of vector and point to be displayed in open 3d
    const std::chrono::duration<double> duration = std::chrono::system_clock::now() - Start;
    const std::chrono::duration<double> durationR = RansacEnd - Start;
    const std::chrono::duration<double> durationL = leastEnd - RansacEnd;
    float dd = 1000 * duration.count();
    if (dd >5.0) RCLCPP_INFO(get_logger(), "Done estimating angle duration: %fms, ransac: %fms, least:%fms", dd, 1000 * durationR.count(),1000 * durationL.count());
}

void TraversabilityAnalysis::savePointCloud(const pcl::PointCloud<PointType> *cloud, const std::string& filename) {
    pcl::PCDWriter writer;
    writer.writeBinaryCompressed(filename, *cloud);
}

// void TraversabilityAnalysis::SaveData(){
//     std::string csvFileName = "data/data.csv";
//     auto existt = std::filesystem::exists(csvFileName);  
//     std::ofstream file(csvFileName, std::ios::app);
//     if (file.is_open()) {
//       if (!existt)
//       {
//       file << "Path,Type,id,R,angle,H" << std::endl;
//       }   
//       for (auto &cluster : Clusters_)
//       {
//         if (cluster.pc.size()<350 || cluster.Status==tODELETE)
//         {
//           continue;
//         }     
//         std::stringstream ss;
//         auto now = std::chrono::system_clock::now().time_since_epoch();
//         ss << "data/pcd/pointcloud_"<<now.count()<<".pcd";
//         std::string pcdFileName = ss.str();      
//         savePointCloud(&cluster.pc, pcdFileName);
//         file << pcdFileName<<","<<GetCategoryName(cluster.Type)<<","<<cluster.id <<","<<cluster.Roughness<<","<<cluster.angle * (180.0/M_PI)<<","<<cluster.H_f <<std::endl;      
//       }// Customize T_s and T_l
//       }else {
//         std::cerr << "Unable to open file: " << csvFileName << std::endl;
//     }  
// }

void TraversabilityAnalysis::PubGlobalMap(){
    rclcpp::Rate rate(3);
    while (rclcpp::ok()){
        globalCostmap_->MapMtx_.lock();
        GlobalcostMapPub_->publish(*globalCostmap_->Map_);
        globalCostmap_->MapMtx_.unlock();
        
        globalCostmap_->CheckAndExpandMap(Position(currentPose_.x(),currentPose_.y()));
        rate.sleep();
    }
}

void TraversabilityAnalysis::BuildCostMap(nav_msgs::msg::OccupancyGrid::SharedPtr localCostmap ){
  
   localCostmap->header = std_msgs::msg::Header();
   localCostmap->header.frame_id = mapFrame;
   localCostmap->info.origin.position.x = currentPose_.x() - (size_(1)/2.0) * CELL_RESOLUTION;
   localCostmap->info.origin.position.y = currentPose_.y() - (size_(0)/2.0) * CELL_RESOLUTION;
   std::vector<Eigen::Vector3d> relativeIndices = globalCostmap_->getRelativeEllipseIndices(covarianceMatrix_);
   for (size_t i = 0; i < size_(0); i++)
   {
    for (size_t j = 0; j < size_(1); j++)
      {
        float& segmentation = elevationMap_.at(SEGMENTATIONLAYER,grid_map::Index(i,j));
        float& cat          = elevationMap_.at(CATIGORISATION,grid_map::Index(i,j));
        auto& cost = localCostmap->data.at((size_(0)-i-1) * size_(1) + j);
        if (segmentation == 2) cost = NO_INFORMATION; 
        if (segmentation == 1) cost = LETHAL_OBSTACLE;
        if (segmentation == 0) cost = cat * MAX_NON_OBSTACLE;
        Position pos;
        pos.x = localCostmap->info.origin.position.x +        j       * CELL_RESOLUTION;
        pos.y = localCostmap->info.origin.position.y + (size_(0)-i-1) * CELL_RESOLUTION;
        for (const auto& relIdx : relativeIndices) {
                Position globalPos;
                pos.x += relIdx.x() * globalCostmap_->Map_->info.resolution;
                pos.y += relIdx.y() * globalCostmap_->Map_->info.resolution;
                globalCostmap_->UpdateCell(pos, cost,relIdx.z());
                
            }
      }
   }
   


}

std::vector<Eigen::Vector3d> OccupancyGrid::getRelativeEllipseIndices(Eigen::Matrix2d covarianceMatrix) {
    double halfResolution = 0.5 * Map_->info.resolution , uncertaintyFactor = 2.486;  // sqrt(6.18)
    const float minimalWeight = std::numeric_limits<float>::epsilon() * static_cast<float>(2.0);
    Eigen::EigenSolver<Eigen::Matrix2d> solver(covarianceMatrix);
    Eigen::Array2d eigenvalues(solver.eigenvalues().real().cwiseAbs());
    Eigen::Matrix2d invCovariance = covarianceMatrix.inverse();
    Eigen::Array2d::Index maxEigenvalueIndex;
    eigenvalues.maxCoeff(&maxEigenvalueIndex);
    Eigen::Array2d::Index minEigenvalueIndex;
    
    maxEigenvalueIndex == Eigen::Array2d::Index(0) ? minEigenvalueIndex = 1 : minEigenvalueIndex = 0;
    Eigen::Array2d lengths =  2 * uncertaintyFactor *eigenvalues.sqrt() ;
    Eigen::Matrix2d rotationMatrix = solver.eigenvectors().real();
    float maxStandardDeviation = sqrt(eigenvalues(maxEigenvalueIndex));
    float minStandardDeviation = sqrt(eigenvalues(minEigenvalueIndex));
    std::vector<Eigen::Vector3d> relativeIndices;
    // Calculate the bounding box of the ellipse
    double maxLength = std::max(lengths.x(), lengths.y());
    int maxOffset = std::ceil(maxLength / Map_->info.resolution);

    // Iterate over the bounding box
    for (int i = -maxOffset; i <= maxOffset; ++i) {
        for (int j = -maxOffset; j <= maxOffset; ++j) {
            // Calculate the cell's position relative to the ellipse center
            Eigen::Vector2d relativePos(i * Map_->info.resolution, j * Map_->info.resolution);
            float probability1 = cumulativeDistributionFunction(relativePos.x() + halfResolution, 0.0, maxStandardDeviation) -
                           cumulativeDistributionFunction(relativePos.x() - halfResolution, 0.0, maxStandardDeviation);
            float probability2 = cumulativeDistributionFunction(relativePos.y() + halfResolution, 0.0, minStandardDeviation) -
                                cumulativeDistributionFunction(relativePos.y() - halfResolution, 0.0, minStandardDeviation);

            const float weight = std::max(minimalWeight, probability1 * probability2);

            // Transform the position to the ellipse's coordinate system
            Eigen::Vector2d transformedPos = rotationMatrix.transpose() * relativePos;


            // Check if the cell is inside the ellipse
            double ellipseValue = (transformedPos.x() * transformedPos.x()) / (lengths.x() * lengths.x()) +
                                  (transformedPos.y() * transformedPos.y()) / (lengths.y() * lengths.y());

            if (ellipseValue <= 1.0) {
                relativeIndices.emplace_back(i, j,weight);
            }
        }
    }

    return relativeIndices;
}



OccupancyGrid::OccupancyGrid(int height,int width,float res, Position origin_to_world,int maxcell, int numcell)

{
  Map_  = new nav_msgs::msg::OccupancyGrid();
  Map_->header.frame_id = "odom";
  Map_->data.resize(height * width, NO_INFORMATION);
  Variance_.resize(height , width);
  Map_->info.height = height ;
  Map_->info.width = width;
  Map_->info.resolution = res;
  Map_->info.origin.position.x = origin_to_world.x;
  Map_->info.origin.position.y = origin_to_world.y;
  num_cell_to_increment_= numcell;
  max_cell_to_increment_ = maxcell;
  num_cell_to_increment_m_ = num_cell_to_increment_ * res;
}

void OccupancyGrid::UpdateCell(Position pos, int8_t cost,double weight){
    int8_t oldcost = GetCost(pos);
    if (oldcost<-1) return;
    if (oldcost == -1)
    {
      SetCost(pos,cost);
      SetWeight(pos,weight);
      return;
    }
    
    double oldweight = GetWeight(pos);

    int8_t newCost = (int) ((weight*oldcost + oldweight * cost)/(oldweight + weight));
    double newweight = (weight * oldweight)/(weight + oldweight);
    SetCost(pos,newCost);
    SetWeight(pos,newweight);

}

int OccupancyGrid::GetIndexWorldPos(Position pos){
  int i = (pos.x - Map_->info.origin.position.x )/ Map_->info.resolution;
  int j = (pos.y - Map_->info.origin.position.y) / Map_->info.resolution;
  return j*Map_->info.width + i;
}


int8_t OccupancyGrid::GetCost(int index){
  return Map_->data[index];
}


int8_t OccupancyGrid::GetCost(int index_i, int index_j){
  return Map_->data[index_i + Map_->info.width * index_j];
}

int8_t OccupancyGrid::GetCost(Position pos){
  
  int index = GetIndexWorldPos(pos);
  if (!(index > Map_->info.height * Map_->info.width || index<0)) return Map_->data[index];
  return -2;
}

void OccupancyGrid::SetCost(int index, int8_t value){
  if (index > Map_->info.height * Map_->info.width || index<0)
    {
      std::cout<< "error requested cell out of bound: index= "<<index<<std::endl;
      return;
    }
  MapMtx_.lock();
  Map_->data[index] = value;// std::max(Map_->data[index], value);//(Map_->data[index] != NO_INFORMATION)  ? std::min(Map_->data[index], value) : value;
  MapMtx_.unlock();
}

void OccupancyGrid::SetCost(Position pos, int8_t value){
  
  MapMtx_.lock();
  int index = GetIndexWorldPos(pos);
  if (!(index > Map_->info.height * Map_->info.width || index<0)) Map_->data[index] = value;
  else{
    ;//std::cout<< "error requested cell out of bound: index= "<<index<<std::endl;
  }
  MapMtx_.unlock();
}

void OccupancyGrid::SetCost(int index_i, int index_j, int8_t value){
  if (index_i>= Map_->info.width || index_j>= Map_->info.height || index_i<0 || index_j<0)
  {
    std::cout<< "error requested cell out of bound: i= "<<index_i<<" j= "<<index_j<<std::endl;
    return;
  }
  
  MapMtx_.lock();
  
  Map_->data[index_i + Map_->info.width * index_j] =  value;//std::max(Map_->data[index_i + Map_->info.width * index_j], value);//(Map_->data[index_i + Map_->info.width * index_j] != NO_INFORMATION)  ? std::min(Map_->data[index_i + Map_->info.width * index_j], value) : value;
  MapMtx_.unlock();
}

double OccupancyGrid::GetWeight(Position pos){
  int i = (pos.x - Map_->info.origin.position.x )/ Map_->info.resolution;
  int j = (pos.y - Map_->info.origin.position.y) / Map_->info.resolution;
  return Variance_(Variance_.rows() - j -1,i);
}

void OccupancyGrid::SetWeight(Position pos, double value){
  int i = (pos.x - Map_->info.origin.position.x )/ Map_->info.resolution;
  int j = (pos.y - Map_->info.origin.position.y) / Map_->info.resolution;
  MapMtx_.lock();
  Variance_(Variance_.rows() - j -1,i) = value;
  MapMtx_.unlock();
}

void OccupancyGrid::CheckAndExpandMap(Position robotPos){
  if (Map_->data.empty()) return;
  int i = (robotPos.x - Map_->info.origin.position.x )/ Map_->info.resolution;
  int j = (robotPos.y - Map_->info.origin.position.y) / Map_->info.resolution;
  nav_msgs::msg::OccupancyGrid* tmp = new nav_msgs::msg::OccupancyGrid();
  Eigen::MatrixXd tmpVar;
  tmp->info = Map_->info;
  tmp->header = Map_->header;
  bool origin_x_changed = false, origin_y_changed = false, any_updates=false;
  int new_width = Map_->info.width, new_height = Map_->info.height;

  if (i+max_cell_to_increment_ >= Map_->info.width)
  {
    new_width = Map_->info.width + num_cell_to_increment_;
    any_updates = true;
    }
  if (i-max_cell_to_increment_ <= 0)
  {
    new_width = Map_->info.width + num_cell_to_increment_;
    origin_x_changed = any_updates = true;
    }
  if (j+max_cell_to_increment_ >= Map_->info.height)
  {
    new_height = Map_->info.height + num_cell_to_increment_;
    any_updates = true;
    }
  if (j-max_cell_to_increment_ <= 0){
    new_height = Map_->info.height + num_cell_to_increment_;
    origin_y_changed = any_updates = true;
    }
  if(!any_updates){
    delete tmp;
    return;
  }
  if (origin_x_changed)
  {
    tmp->info.origin.position.x -= num_cell_to_increment_m_;
  }
  if (origin_y_changed)
  {
    tmp->info.origin.position.y -= num_cell_to_increment_m_;
  }
  tmp->data.resize(new_width * new_height , NO_INFORMATION);
  tmpVar.resize(new_height,new_width);
  tmp->info.height = new_height;
  tmp->info.width = new_width;


  MapMtx_.lock();
  int incrY = 0;
  if (origin_y_changed) incrY= num_cell_to_increment_ ;
  int incrX = 0;
  if (origin_x_changed) incrX= num_cell_to_increment_;
  for (size_t j = 0; j < Map_->info.height; j++)
  {
    for (size_t i = 0; i < Map_->info.width; i++)
      {
        tmp->data[(j + incrY) * tmp->info.width + i + incrX] = GetCost(i,j);
        tmpVar((j + incrY) , i + incrX) = Variance_(i,j);
      }
  }
  
  delete Map_;
  Map_ = tmp;
  MapMtx_.unlock();
  
}




}// End namespace 

// if (value ==-1)
//     {
//       Map_->data[index] = -1;
//     }else if (value>=61)
//     {
//       Map_->data[index] = 100;
//     }else
//     {
//       Map_->data[index] = 0;
//     }

// Eigen::Vector4f centroid;
//   Eigen::Matrix3f S;
//   pcl::compute3DCentroid(cluster.pc, centroid);
//   computeCovarianceMatrix (cluster.pc, centroid, S); 



// Eigen::Matrix3f S;
//   S << a_1, a_2, a_3,
//       a_2, a_4, a_5,
//       a_3, a_5, a_6; 
//   S /= cluster.pc.points.size()-1;

//   Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver(S);
//   if (eigensolver.info() != Eigen::Success) return 0;

//   Eigen::Vector3f L = eigensolver.eigenvalues();
//   float z = L[2];
//   std::sort(L.data(), L.data() + L.size());
//   cluster.Roughness = false;
//   cluster.z = z;
//   cluster.eig3 = L[2];
//   if(L[0]!=0 && L[1]/L[0]>=T_RATIO  && L[2] > z ) return false;
//   cluster.Roughness = true;
//   return true;
