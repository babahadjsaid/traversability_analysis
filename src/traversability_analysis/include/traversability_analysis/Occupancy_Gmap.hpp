#pragma once



#include "traversability_analysis/utilities.hpp"

class OccupancyGrid
{
    public:
        explicit OccupancyGrid(int height,int width,float res, Position origin_to_world, int maxcell, int numcell);
        void Reset();
        std::pair<int, int> GetIndexWorldPos(Position pos);
        int8_t GetCost(int index_i, int index_j);
        int8_t GetCost(Position pos);
        void SetCost(Position pos, int8_t value);
        void SetCost(int index_i, int index_j, int8_t value);
        double GetWeight(Position pos);
        double GetWeight(int i, int j);
        void SetWeight(Position pos, double value);
        void SetWeight(int i, int j, double value);
        Eigen::Matrix2d ComputeGlobalCovariance(Pose robot_pose, Eigen::Matrix3d pose_covariance, Position local_pos);
        // std::pair<int8_t, double> GetCell(Position pos);
        // std::pair<int8_t, double> GetCell(int index_i, int index_j);
        // void SetCell(Position pos, int8_t cost, double weight);
        // void SetCell(int index_i, int index_j, int8_t cost, double weight);
        void UpdateCell(Position pos, double z, Eigen::Matrix2d CovM);
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

 