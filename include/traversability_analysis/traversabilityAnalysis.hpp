#pragma once
#define MAXHEIGHTLAYER "max_height"
#define MINHEIGHTLAYER "min_height"
#define MEANHEIGHTLAYER "mean_height"
#define SEGMENTATIONLAYER "segmentation"
#define REFRENCENONGRIDLAYER "RNG"
#define COLORLAYER "Color"
#define GRIDSPOINTCLOUD "gridsPointClouds"
#define CATIGORISATION "2D Convolution"



#define GROUND "ground"
#define OBSTACLES "Obstacle"
#define POTHOLE "Pothole"
#define SLOPE  "Slope"
#define NEGATIVESLOPE "Negative Slope"

#define NO_INFORMATION -1
#define LETHAL_OBSTACLE 100
#define MAX_NON_OBSTACLE 99
#define FREE_SPACE 0


// utility

#include "traversability_analysis/utility.hpp"

namespace traversability_analysis {
struct NonGroundGrid;  
enum ObjectsCategories {
    gROUND,
    oBSTACLES,
    pOTHOLE,
    sLOPE,
    nEGATIVESLOPE
};
enum ClusterStatus {
    nEw=0,
    uPTODATE,
    oLD,
    tODELETE
};

  struct Cluster {
    std::vector<NonGroundGrid*> grids;
    pcl::PointCloud<PointType> pc;
    float min_height,max_height,mean_height=0.0;
    float H_f,angle=0;
    bool Roughness = false;
    int color;
    grid_map::Position Point_mass;
    ClusterStatus Status = nEw;
    ObjectsCategories Type;
    int64_t id;
    std::map<ObjectsCategories, int> category_count;
    Cluster(){
        id = std::chrono::system_clock::now().time_since_epoch().count();
        category_count[sLOPE] = 0;
        category_count[nEGATIVESLOPE] = 0;
        category_count[oBSTACLES] = 0;
        category_count[pOTHOLE] = 0;

    }
    ObjectsCategories getCat(){
        ObjectsCategories cattmp;
        int max = 0;
        for (const auto& pair : category_count){
            if(pair.second >= max){
                max = pair.second;
                cattmp = pair.first;
            }
        }
        return cattmp;
    }

};

struct NonGroundGrid {
    grid_map::Index index;
    Cluster* cluster;
    int color;
    bool clustered = false;
    long int idx;
};
struct Position {
    Position() = default;
    Position(float i, float j){
        x = i;
        y = j;
    }
    float x,y;
};
struct Index {
    int i,j;
};

class OccupancyGrid
{
    public:
        explicit OccupancyGrid(int height,int width,float res, Position origin_to_world, int maxcell, int numcell);
        void Reset();
        int GetIndexWorldPos(Position pos);
        int8_t GetCost(int index_i, int index_j);
        int8_t GetCost(Position pos);
        void SetCost(Position pos, int8_t value);
        void SetCost(int index_i, int index_j, int8_t value);
        double GetWeight(Position pos);
        void SetWeight(Position pos, double value);
        void UpdateCell(Position pos, int8_t value,double Variance);
        void CheckAndExpandMap(Position robotPos);
        std::vector<Eigen::Vector3d> getRelativeEllipseIndices(Eigen::Matrix2d covarianceMatrix);
        float cumulativeDistributionFunction(float x, float mean, float standardDeviation);
        nav_msgs::msg::OccupancyGrid* Map_;
        std::mutex MapMtx_;
        Eigen::MatrixXd Variance_;
    private:
        int num_cell_to_increment_, max_cell_to_increment_;
        float num_cell_to_increment_m_;

};

class TraversabilityAnalysis : public ParamServer{
 public:
    long int numFrames_= 0;
    float avgTime_;
    std::stringstream BenchmarkTiming_;
    /*!
    * Constructor.
    */
    explicit TraversabilityAnalysis(std::string node_name, const rclcpp::NodeOptions & options);
    void InitMapLayers();
    void PointCloudHandler(sensor_msgs::msg::PointCloud2::SharedPtr pointCloudMsg);
    void MapProjection(pcl::PointCloud<PointType>::Ptr pointCloud);
    void NonGroundGridClustering();
    void CostCalculation();
    void GroundSegmentation();
    void OdometryHandler(nav_msgs::msg::Odometry::SharedPtr poseMsg);
    void FloodFill(grid_map::Index index,grid_map::Index prevIndex, Cluster *cluster);
    bool CalculateRoughness(Cluster &cluster);
    void EstimateAngle(Cluster &cluster);
    void savePointCloud(const pcl::PointCloud<PointType> *cloud, const std::string& filename);
    void SaveData();
    void PubGlobalMap();
    void BuildCostMap(nav2_msgs::msg::Costmap::SharedPtr localCostmap );
    std::string GetCategoryName(ObjectsCategories categoryName);

 private:
    //Sync
    std::mutex mapMtx_,poseMtx_,globalMapMtx_;

    //Map
    grid_map::GridMap elevationMap_;
    grid_map::Size size_;
    //Topics 
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudSub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr GlobalcostMapPub_;
    rclcpp::Publisher<nav2_msgs::msg::Costmap>::SharedPtr costMapPub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robotPoseSubscriber_;
    
    Eigen::MatrixXf kernel_;
    std::vector<int> Colors_;
    std::vector<NonGroundGrid> C_N_;
    std::vector<Cluster> Clusters_;
    std::vector<pcl::PointCloud<PointType>> gridsPointClouds_;
    
    std::random_device randomDevice_;
    std::mt19937 generator_;
    nav_msgs::msg::Odometry currentOdom_;
    grid_map::Position3  currentPose_,previousPose_,displacement_;
    geometry_msgs::msg::Twist currentTwist_;
    bool receivedPose_, firstPose_;
    std::string mapFrame;
    nav2_msgs::msg::Costmap::SharedPtr message_;
    OccupancyGrid* globalCostmap_;
    Eigen::Matrix2d covarianceMatrix_;

};

}  // namespace traversability_analysis




// #pragma once
// #define MAXHEIGHTLAYER "max_height"
// #define MINHEIGHTLAYER "min_height"
// #define MEANHEIGHTLAYER "mean_height"
// #define SEGMENTATIONLAYER "segmentation"
// #define REFRENCENONGRIDLAYER "RNG"
// #define COLORLAYER "Color"
// #define GRIDSPOINTCLOUD "gridsPointClouds"
// #define CATIGORISATION "2D Convolution"



// #define GROUND "ground"
// #define OBSTACLES "Obstacle"
// #define POTHOLE "Pothole"
// #define SLOPE  "Slope"
// #define NEGATIVESLOPE "Negative Slope"

// #define NO_INFORMATION -1
// #define LETHAL_OBSTACLE 100
// #define MAX_NON_OBSTACLE 99
// #define FREE_SPACE 0


// // utility

// #include "traversability_analysis/utility.hpp"

// namespace traversability_analysis {
// struct NonGroundGrid;  
// enum ObjectsCategories {
//     gROUND,
//     oBSTACLES,
//     pOTHOLE,
//     sLOPE,
//     nEGATIVESLOPE
// };
// enum ClusterStatus {
//     nEw=0,
//     uPTODATE,
//     oLD,
//     tODELETE
// };

//   struct Cluster {
//     std::vector<NonGroundGrid*> grids;
//     pcl::PointCloud<PointType> pc;
//     float min_height,max_height,mean_height=0.0;
//     float H_f,angle=0;
//     bool Roughness = false;
//     int color;
//     grid_map::Position Point_mass;
//     ClusterStatus Status = nEw;
//     ObjectsCategories Type;
//     int64_t id;
//     std::map<ObjectsCategories, int> category_count;
//     Cluster(){
//         id = std::chrono::system_clock::now().time_since_epoch().count();
//         category_count[sLOPE] = 0;
//         category_count[nEGATIVESLOPE] = 0;
//         category_count[oBSTACLES] = 0;
//         category_count[pOTHOLE] = 0;

//     }
//     ObjectsCategories getCat(){
//         ObjectsCategories cattmp;
//         int max = 0;
//         for (const auto& pair : category_count){
//             if(pair.second >= max){
//                 max = pair.second;
//                 cattmp = pair.first;
//             }
//         }
//         return cattmp;
//     }

// };

// struct NonGroundGrid {
//     grid_map::Index index;
//     Cluster* cluster;
//     int color;
//     bool clustered = false;
//     long int idx;
// };
// struct Position {
//     Position() = default;
//     Position(float i, float j){
//         x = i;
//         y = j;
//     }
//     float x,y;
// };
// struct Index {
//     int i,j;
// };

// class OccupancyGrid
// {
//     public:
//         explicit OccupancyGrid(int height,int width,float res, Position origin_to_world, int maxcell, int numcell);
//         void Reset();
//         int GetIndexWorldPos(Position pos);
//         int8_t GetCost(int index_i, int index_j);
//         int8_t GetCost(Position pos);
//         void SetCost(Position pos, int8_t value);
//         void SetCost(int index_i, int index_j, int8_t value);
//         double GetWeight(Position pos);
//         void SetWeight(Position pos, double value);
//         void UpdateCell(Position pos, int8_t value,double Variance);
//         void CheckAndExpandMap(Position robotPos);
//         std::vector<Eigen::Vector3d> getRelativeEllipseIndices(Eigen::Matrix2d covarianceMatrix);
//         float cumulativeDistributionFunction(float x, float mean, float standardDeviation);
//         nav_msgs::msg::OccupancyGrid* Map_;
//         std::mutex MapMtx_;
//         Eigen::MatrixXd Variance_;
//     private:
//         int num_cell_to_increment_, max_cell_to_increment_;
//         float num_cell_to_increment_m_;

// };

// class TraversabilityAnalysis : public ParamServer{
//  public:
//     long int numFrames_= 0;
//     float avgTime_;
//     std::stringstream BenchmarkTiming_;
//     /*!
//     * Constructor.
//     */
//     explicit TraversabilityAnalysis(std::string node_name, const rclcpp::NodeOptions & options);
//     void InitMapLayers();
//     void PointCloudHandler(sensor_msgs::msg::PointCloud2::SharedPtr pointCloudMsg);
//     void MapProjection(pcl::PointCloud<PointType>::Ptr pointCloud);
//     void NonGroundGridClustering();
//     void CostCalculation();
//     void GroundSegmentation();
//     void OdometryHandler(nav_msgs::msg::Odometry::SharedPtr poseMsg);
//     void FloodFill(grid_map::Index index,grid_map::Index prevIndex, Cluster *cluster);
//     bool CalculateRoughness(Cluster &cluster);
//     void EstimateAngle(Cluster &cluster);
//     void savePointCloud(const pcl::PointCloud<PointType> *cloud, const std::string& filename);
//     void SaveData();
//     void PubGlobalMap();
//     void BuildCostMap(nav_msgs::msg::OccupancyGrid::SharedPtr localCostmap );
//     std::string GetCategoryName(ObjectsCategories categoryName);

//  private:
//     //Sync
//     std::mutex mapMtx_,poseMtx_,globalMapMtx_;

//     //Map
//     grid_map::GridMap elevationMap_;
//     grid_map::Size size_;
//     //Topics 
//     rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudSub_;
//     rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr GlobalcostMapPub_;
//     rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costMapPub_;

//     rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robotPoseSubscriber_;
    
//     Eigen::MatrixXf kernel_;
//     std::vector<int> Colors_;
//     std::vector<NonGroundGrid> C_N_;
//     std::vector<Cluster> Clusters_;
//     std::vector<pcl::PointCloud<PointType>> gridsPointClouds_;
    
//     std::random_device randomDevice_;
//     std::mt19937 generator_;
//     nav_msgs::msg::Odometry currentOdom_;
//     grid_map::Position3  currentPose_,previousPose_,displacement_;
//     geometry_msgs::msg::Twist currentTwist_;
//     bool receivedPose_, firstPose_;
//     std::string mapFrame;
//     nav_msgs::msg::OccupancyGrid::SharedPtr message_;
//     OccupancyGrid* globalCostmap_;
//     Eigen::Matrix2d covarianceMatrix_;

// };

// }  // namespace traversability_analysis
