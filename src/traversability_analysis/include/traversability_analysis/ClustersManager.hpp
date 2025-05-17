
#pragma once


#include "traversability_analysis/utilities.hpp"
#include "traversability_analysis/RobotPose.hpp"

class ClustersManager{
    public:
        using iterator = std::vector<Cluster>::iterator;
        using const_iterator = std::vector<Cluster>::const_iterator;
        ClustersManager();
        void PredictClusterPointMass();
        void RemoveOldClusters();
        bool MatchCluster(Cluster new_cluster);
        void ComputeNormlisedCentroidCovariance(const Cluster& cluster,double dx, double dy, Eigen::MatrixXd& covarianceMatrixNormalized);
        // Iterator access
        iterator begin() { return Clusters_.begin(); }
        const_iterator begin() const { return Clusters_.begin(); }
        iterator end() { return Clusters_.end(); }
        const_iterator end() const { return Clusters_.end(); }
        Cluster& operator[](size_t index) { return Clusters_[index]; }
        size_t size() const { return Clusters_.size(); }
        void addCluster(Cluster cluster) { Clusters_.push_back(cluster); }
        nlohmann::json serializeMultipleClustersToJson(std::string directory);
        
        grid_map::GridMap* elevationMap_;
        RobotPose* robotPose_;
    private:
        std::vector<Cluster> Clusters_;
};      




