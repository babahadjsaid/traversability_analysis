#pragma once
#include "traversability_analysis/traversabilityAnalysis.hpp"
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
    std::string GetCategoryName(ObjectsCategories categoryName);

 private:
    //Sync
    std::mutex mapMtx_,poseMtx_;

    //Map
    grid_map::GridMap elevationMap_;
    grid_map::Size size_;
    //Topics 
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudSub_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr costMapPub_;
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
    
   

};

}  // namespace traversability_analysis
