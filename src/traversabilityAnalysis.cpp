
#include "traversability_analysis/traversabilityAnalysis.hpp"

namespace traversability_analysis{

TraversabilityAnalysis::TraversabilityAnalysis(std::string node_name, const rclcpp::NodeOptions & options)
: ParamServer(node_name, options),
elevationMap_({MAXHEIGHTLAYER,MINHEIGHTLAYER,MEANHEIGHTLAYER,SEGMENTATIONLAYER,REFRENCENONGRIDLAYER,COLORLAYER,GRIDSPOINTCLOUD,CATIGORISATION}),
kernel_(3,3),
generator_(randomDevice_())
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
  
  

}

void TraversabilityAnalysis::PointCloudHandler(sensor_msgs::msg::PointCloud2::SharedPtr pointCloudMsg){
  mapMtx_.lock();
  const auto methodStartTime = std::chrono::system_clock::now();
  pcl::PointCloud<PointType>::Ptr pointCloud(new pcl::PointCloud<PointType>());
  pcl::moveFromROSMsg(*pointCloudMsg, *pointCloud);
  auto& min_heightLayer = elevationMap_[MINHEIGHTLAYER];
  auto& max_heightLayer = elevationMap_[MAXHEIGHTLAYER];
  auto& mean_heightLayer = elevationMap_[MEANHEIGHTLAYER];
  auto& RTGPCLayer = elevationMap_[GRIDSPOINTCLOUD];
  auto& RNGLayer = elevationMap_[REFRENCENONGRIDLAYER];
  auto& segmentLayer = elevationMap_[SEGMENTATIONLAYER];
  auto& catigorisationLayer = elevationMap_[CATIGORISATION];
  mean_heightLayer.setZero();
  max_heightLayer.setZero();
  min_heightLayer.setConstant(MAX_HEIGHT);
  RTGPCLayer.setConstant(-1);
  RNGLayer.setConstant(-1);
  catigorisationLayer.setZero();

  //                                                                                            Start Map Projection.
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
    auto& RTGPC = RTGPCLayer(index(0), index(1));
    
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
  // RCLCPP_INFO(get_logger(), "pdf=%f, minimum=%f",mean_pdf/count,min_pdf);
  const auto EndProjection = std::chrono::system_clock::now();
  //                                                                                            End Map Projection.
  //                                                                                            Start Ground Segmentation.
  const std::chrono::duration<double> durations  = std::chrono::system_clock::now() - methodStartTime;
  //RCLCPP_INFO(get_logger(), "dDone in  %f ms.", 1000* durations.count());
  
  
  // #pragma omp parallel for num_threads(5)
  for (int i = 0; i < size_(0); i++)
  {
    // #pragma omp parallel for num_threads(5)
    for (int j = 0; j < size_(1); j++)
    {
      float& segmentation = segmentLayer(i,j);
      float& max_height   = max_heightLayer(i,j);
      float& min_height   = min_heightLayer(i,j);
      float& mean_height   = mean_heightLayer(i,j);
      float& RNG   = RNGLayer(i,j);
      float& RTGPC   = RTGPCLayer(i,j);
      float& cat   = catigorisationLayer(i,j);

      if (min_height==MAX_HEIGHT) // if the cell was not visited.
      {
        min_height = max_height = mean_height = 0;
        segmentation = 2;
        continue;
      }
      bool allZeros = true, allOnes = true;
      mean_height /= gridsPointClouds_[RTGPC].points.size();
      Eigen::MatrixXf segs;
      segmentation = ((max_height - min_height) >= T_DIFF || max_height >= T_HIGH || min_height <= T_LOW);
      if (!(i>0 && i < mean_heightLayer.rows()-1 && j>0 && j < mean_heightLayer.cols()-1))
          goto AddToNonGrid;
      kernel_ << -1, -2, -1, 0, 0, 0, 1, 2, 1;// dx
      cat = pow((mean_heightLayer.block(i-1,j-1,3,3).array() * kernel_.array()).sum(),2);
      kernel_ << 1, 0, -1, 2, 0, -2, 1, 0, -1;// dy
      cat += pow((mean_heightLayer.block(i-1,j-1,3,3).array() * kernel_.array()).sum(),2);
      cat = sqrt(cat);
      cat = 0.5-(0.5 / exp(cat));
      
      segs = segmentLayer.block(i-1,j-1,3,3).array();
      
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
      if(allOnes && segmentation==2)
            segmentation = allOnes;  
      if (allZeros && segmentation == 2)
            segmentation = !allZeros; 
      AddToNonGrid:
      if (segmentation==true)
      {
        grid_map::Index idx(i,j);
        NonGroundGrid tmp;
        RNG = C_N_.size();
        tmp.index = idx;
        C_N_.push_back(tmp);

      }
      
    }
    
  }
  
  const auto EndSegmentation = std::chrono::system_clock::now();
  //                                                                                            End Ground Segmentation.
  //                                                                                            Start Grids clustering.
  
  
  // Define the distribution for random numbers between 0 and 255
  std::uniform_int_distribution<int> distribution(0, 255);
  
  for (size_t i = 0; i < C_N_.size(); i++)
  {
    
    if (C_N_[i].clustered) continue;
    Cluster new_cluster;
    
    FloodFill(C_N_[i].index,&new_cluster, distribution(generator_));
    if (new_cluster.grids.size()<NUM_GRIDS_MIN)
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
    float H_d = new_cluster.max_height - new_cluster.min_height;
    if(new_cluster.mean_height>=0){
      new_cluster.H_f = std::max(H_d,new_cluster.mean_height);
    }else
    {
      new_cluster.H_f = std::min(-H_d,new_cluster.mean_height);
    }
    
    if (T_NEG < new_cluster.H_f && new_cluster.H_f < T_POS)
    {
      new_cluster.Type = OBSTACLES;
      goto ADD_CLUSTER;
    }
    
    if (new_cluster.H_f >= T_POS)
    {
      if (CalculateRoughness(new_cluster))
      {
        new_cluster.Type = OBSTACLES;
        goto ADD_CLUSTER;
      }
      new_cluster.Type = SLOPE ;
    } else { // T_NEG >= new_cluster.H_f 
      if (CalculateRoughness(new_cluster))
      {
        new_cluster.Type = POTHOLE;
        goto ADD_CLUSTER;
      }
      new_cluster.Type = NEGATIVESLOPE;
      }
      
    EstimateAngle(new_cluster);
    ADD_CLUSTER:
    Clusters_.push_back(new_cluster);
    
  }
  
  // gridsPointClouds_.clear();
  const auto EndClustering = std::chrono::system_clock::now();
  //                                                                                            End Grids clustering.
  //                                                                                            Start Cost Calculation.
  
  for (auto &&cluster : Clusters_)
  {
    float cost;
    switch (checkCategory(cluster.Type))
    {
    case pOTHOLE: 
    case oBSTACLES:
      cost = 1.0;
      break;
    case sLOPE:      
    case nEGATIVESLOPE:
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
      auto& cat = catigorisationLayer(grid->index(0),grid->index(1));
      if (cost == 1)
      {
        cat = 1.0;
        continue;
      }
      cat += cost;
      
    }
    
  }
  


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
  SaveData(Clusters_);
  RCLCPP_INFO(get_logger(), "Done processing number of non ground grid found is %ld, Projection Step took %f ms, segmentation Step Took %f ms, clustering (Num of clusters %ld) Step Took %f ms.",C_N_.size(),durationOfProjectionMS,durationOfSegmentationMS,Clusters_.size(),durationOfClusteringMS);
  C_N_.clear();
  Clusters_.clear();
}




double TraversabilityAnalysis::NormalPDF(double x, double mean, double variance) {
    return (1.0 / sqrt(2.0 * M_PI * variance)) * exp(-0.5 * pow((x - mean) / sqrt(variance), 2));
}

ObjectsCategories TraversabilityAnalysis::checkCategory(std::string &categoryName){
  if (categoryName == OBSTACLES) return oBSTACLES;
  if (categoryName == SLOPE) return sLOPE ;
  if (categoryName == NEGATIVESLOPE) return nEGATIVESLOPE;
  if (categoryName == POTHOLE) return pOTHOLE;
  return gROUND;
}


void TraversabilityAnalysis::FloodFill(grid_map::Index index,Cluster *cluster,int color)
{
    // Base cases 
    if (index(0) < 0 || index(0) >= size_(0) || index(1) < 0 || index(1) >= size_(1)) return;
    auto& segmentation = elevationMap_.at(SEGMENTATIONLAYER,index);
    auto& RNG = elevationMap_.at(REFRENCENONGRIDLAYER,index);
    long int idx = RNG;
    if (segmentation==false || segmentation==2 || C_N_[idx].clustered) return;
    
    auto& colorGrid = elevationMap_.at(COLORLAYER,index);
    auto& max_height_Grid = elevationMap_.at(MAXHEIGHTLAYER,index);
    auto& min_height_Grid = elevationMap_.at(MINHEIGHTLAYER,index);
    auto& mean_height_Grid = elevationMap_.at(MEANHEIGHTLAYER,index);
    auto& RTGPC = elevationMap_.at(GRIDSPOINTCLOUD,index);

    colorGrid = color;
    // Add Grid to cluster.
    NonGroundGrid* grid = &C_N_[idx];
    //RCLCPP_INFO(get_logger(), "Hello !! %d, %d, with idx=%ld",index(0),index(1),idx);
    grid->cluster = cluster;
    grid->clustered = true;
    cluster->grids.push_back(grid);
    cluster->mean_height += mean_height_Grid;
    grid_map::Position pos;
    elevationMap_.getPosition(grid->index,pos);
    cluster->Point_mass += pos;
    cluster->pc += gridsPointClouds_[RTGPC];
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

bool TraversabilityAnalysis::CalculateRoughness(Cluster cluster){
  float x_m, y_m, z_m;
  for (auto &&point : cluster.pc.points)
  {
    x_m += point.x; y_m += point.y; z_m += point.z;
  }
  x_m /= cluster.pc.points.size();
  y_m /= cluster.pc.points.size();
  z_m /= cluster.pc.points.size();
  float a_1, a_2, a_3, a_4, a_5, a_6;
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
  
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver(S);
  if (eigensolver.info() != Eigen::Success) return 0;

  Eigen::Vector3f L = eigensolver.eigenvalues();
  std::sort(L.data(), L.data() + L.size());
  cluster.Roughness = 0;
  if(L[0]!=0 && L[1]/L[0]>=T_RATIO && L[1] >= T_L && L[0] <= T_S ) return false;
  cluster.Roughness = 1;
  return true; 

}

void TraversabilityAnalysis::EstimateAngle(Cluster cluster){
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
        for (int j = 0; j < CloudSize; ++j) {
            float distance = PointToPlaneDistance(cluster.pc.points[j], Ai, Bi, Ci, Di);
            if (distance <= T_SEG) {
                inliers.push_back(j);
            }
        }

        // Update best plane parameters
        if (inliers.size() >= NumofInliers) {
            NumofInliers = inliers.size();
            BestInliers = inliers;
        }

        // Check termination condition
        if (NumofInliers / (CloudSize+0.0) > T_INLINERS) {
            break;
        }
    }

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
  
}

void TraversabilityAnalysis::savePointCloud(const pcl::PointCloud<PointType> *cloud, const std::string& filename) {
    pcl::PCDWriter writer;
    writer.writeBinaryCompressed(filename, *cloud);
}

void TraversabilityAnalysis::SaveData(std::vector<Cluster>  &clusters){
    std::string csvFileName = "data/data.csv";
    auto existt = std::filesystem::exists(csvFileName);
    
    std::ofstream file(csvFileName, std::ios::app);

    if (file.is_open()) {
      if (!existt)
      {
      file << "Path, Type, R, angle, H_f" << std::endl;
      }
      
      for (auto &&cluster : clusters)
      {
        if (cluster.pc.size()<350)
        {
          continue;
        }
        
        std::stringstream ss;
        auto now = std::chrono::system_clock::now().time_since_epoch();
        ss << "data/pcd/pointcloud_"<<now.count()<<".pcd";
        std::string pcdFileName = ss.str();
        // RCLCPP_INFO(get_logger(),pcdFileName.c_str());
        savePointCloud(&cluster.pc, pcdFileName);
        file << pcdFileName<<", "<<cluster.Type<<", "<<cluster.Roughness<<", "<<cluster.angle<<", "<<cluster.H_f<<std::endl;
      }
      }else {
        std::cerr << "Unable to open file: " << csvFileName << std::endl;
    }

    
}





}// End namespace 

