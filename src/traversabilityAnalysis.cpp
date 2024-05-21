
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
  costMapPub_ = create_publisher<grid_map_msgs::msg::GridMap>(CM_TOPIC, 1); //nav2_msgs::msg::Costmap
  
  float side_length = RADIUS * sqrt(2);
  elevationMap_.clearAll();
  elevationMap_.resetTimestamp();
  elevationMap_.setFrameId(MAP_FRAME);
  elevationMap_.setGeometry(grid_map::Length(side_length,side_length),CELL_RESOLUTION,grid_map::Position(0,0));
  size_ = elevationMap_.getSize();
  Colors_ = linspace(0,255,NUM_COLORS);

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
    
    currentPose_.x() = currentOdom_.pose.pose.position.x;
    currentPose_.y() = currentOdom_.pose.pose.position.y;
    tf2::Quaternion orientation;
    tf2::fromMsg(currentOdom_.pose.pose.orientation, orientation);
    double roll, pitch, yaw;
    tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
    currentPose_.z() = yaw ;
    currentTwist_= currentOdom_.twist.twist;
    receivedPose_ = true;
    
  }
  poseMtx_.unlock();
  if (abs(currentTwist_.angular.x) >= 0.6|| abs(currentTwist_.angular.y) >= 0.6 || abs(currentTwist_.angular.z) >= 0.6 )
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
  std::cout << BenchmarkTiming_.str() << std::endl;
 
  std::unique_ptr<grid_map_msgs::msg::GridMap> message;
  message = grid_map::GridMapRosConverter::toMessage(elevationMap_);
  costMapPub_->publish(std::move(message));
  elevationMap_.clearAll();
  if (receivedPose_)
  {
    previousPose_ = currentPose_;
    receivedPose_ = false;
    firstPose_ = false;
  }
  SaveData();
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
    grid_map::Position position(point.x, point.y);  
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
        auto& RNG = elevationMap_.at(REFRENCENONGRIDLAYER,grid->index);
        auto& colorGrid = elevationMap_.at(COLORLAYER,grid->index);
        segmentation = 0;
        RNG = -1;
        colorGrid = 0;
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
    
    if (T_NEG < cluster.H_f && cluster.H_f < T_POS)   cluster.Type = OBSTACLES;
    
    if (cluster.H_f >= T_POS)
    {
      if (CalculateRoughness(cluster)) cluster.Type = OBSTACLES;
      else cluster.Type = SLOPE ;
    } else { // T_NEG >= cluster.H_f 
      if (CalculateRoughness(cluster)) cluster.Type = POTHOLE;
      else cluster.Type = NEGATIVESLOPE;
      }
    
    float cost;
    switch (checkCategory(cluster.Type))
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
    
    
    for (auto &&grid : cluster.grids)
    {
      auto& cat  = elevationMap_.at(CATIGORISATION,grid->index);
      auto& color = elevationMap_.at(COLORLAYER,grid->index);
      color = cluster.color;
      if (cost == 1)
      {
        cat = 1.0;
        continue;
      }
      cat += cost;
      
    }
    
  }


}


ObjectsCategories TraversabilityAnalysis::checkCategory(std::string &categoryName){
  if (categoryName == OBSTACLES) return oBSTACLES;
  if (categoryName == SLOPE) return sLOPE ;
  if (categoryName == NEGATIVESLOPE) return nEGATIVESLOPE;
  if (categoryName == POTHOLE) return pOTHOLE;
  return gROUND;
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
  cluster.ss_ = new std::stringstream("");
  *(cluster.ss_)<< eigenvalues[1]/eigenvalues[0]<<","<< eigenvalues[0] <<","<<eigenvalues[1] <<","<<eigenvalues[2];
  
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
    
    cluster.vec = VecC;
    cluster.p1 = cluster.pc.points[BestInliers[0]];
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

void TraversabilityAnalysis::SaveData(){
    std::string csvFileName = "data/data.csv";
    auto existt = std::filesystem::exists(csvFileName);
    
    std::ofstream file(csvFileName, std::ios::app);

    if (file.is_open()) {
      if (!existt)
      {
      file << "Path,Type,R,angle,H" << std::endl;
      }
      
      for (auto &cluster : Clusters_)
      {
        if (cluster.pc.size()<350 || cluster.Status==tODELETE)
        {
          continue;
        }
        
        std::stringstream ss;
        auto now = std::chrono::system_clock::now().time_since_epoch();
        ss << "data/pcd/pointcloud_"<<now.count()<<".pcd";
        std::string pcdFileName = ss.str();
        
        savePointCloud(&cluster.pc, pcdFileName);
        file << pcdFileName<<","<<cluster.Type<<","<<cluster.Roughness<<","<<cluster.angle * (180.0/M_PI)<<","<<cluster.H_f <<std::endl;
        delete cluster.ss_;
      }// Customize T_s and T_l
      }else {
        std::cerr << "Unable to open file: " << csvFileName << std::endl;
    }

    
}




}// End namespace 



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
