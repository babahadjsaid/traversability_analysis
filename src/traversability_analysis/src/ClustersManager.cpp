#include <traversability_analysis/ClustersManager.hpp>



// Covariance damping factor to prevent unlimited growth
constexpr double COVARIANCE_DAMPING = 0.99;
 
ClustersManager::ClustersManager()
{
    Clusters_ = std::vector<Cluster>();
    
}

void ClustersManager::RemoveOldClusters()
{
    Clusters_.erase(std::remove_if(Clusters_.begin(), Clusters_.end(), [this](Cluster& cluster) { 
    
    if (cluster.Status == tODELETE) return true;
    cluster.Status = oLD;
    return false;
    } ), Clusters_.end());
}

void ClustersManager::PredictClusterPointMass()
{
    // Calculate the displacement between previous and current robot pose
    Pose displacement;
    displacement.x() = robotPose_->previousPose_.x() - robotPose_->currentPose_.x();
    displacement.y() = robotPose_->previousPose_.y() - robotPose_->currentPose_.y();
    displacement.yaw() = robotPose_->previousPose_.yaw() - robotPose_->currentPose_.yaw();

    // Extract rotation components for current and previous poses (yaw angles)
    double sin_o_k = sin(-robotPose_->currentPose_.yaw()), cos_o_k = cos(robotPose_->currentPose_.yaw());
    double sin_k1_k = sin(displacement.yaw()), cos_k1_k = cos(displacement.yaw());

    // Rotation matrix from frame k-1 to k
    Eigen::Matrix2d R_k1_k;
    R_k1_k << cos_k1_k, sin_k1_k,
              -sin_k1_k,  cos_k1_k;

    double alpha = 2*2.486* sqrt(displacement.x()*displacement.x() + displacement.y()*displacement.y());
    for (auto &&cluster : Clusters_)
    {
        // Predict cluster point mass position based on motion
        grid_map::Position predicted;
        // predicted = cluster.Point_mass + displacement;
        predicted.x() = cos_k1_k * cluster.Point_mass.x() - sin_k1_k * cluster.Point_mass.y() + cos_o_k * displacement.x() - sin_o_k * displacement.y();
        predicted.y() = sin_k1_k * cluster.Point_mass.x() + cos_k1_k * cluster.Point_mass.y() + sin_o_k * displacement.x() + cos_o_k * displacement.y();
        // Update the cluster's point mass position
        cluster.Point_mass = predicted;

        // Calculate the normalized covariance matrix for the cluster
        Eigen::MatrixXd covarianceMatrix2DNormalized;
        ComputeNormlisedCentroidCovariance(cluster,displacement.x(),displacement.y(), covarianceMatrix2DNormalized);

        // Propagate covariance with the rotation matrix from k-1 to k
        Eigen::MatrixXd propagatedCovariance = R_k1_k * covarianceMatrix2DNormalized * R_k1_k.transpose();

        // Combine with robot's covariance matrices
        cluster.ClusterCovarianceMatrix_ = cluster.ClusterCovarianceMatrix_ 
                                           + robotPose_->covarianceMatrix_ 
                                           + robotPose_->previousCovarianceMatrix_;

        
    }
}



void ClustersManager::ComputeNormlisedCentroidCovariance(const Cluster& cluster,double dx, double dy,
                             Eigen::MatrixXd& covarianceMatrixNormalized) {

    // Ensure the input dimensions are correct
    // if (cluster.eigenvalues.size() != 3 || cluster.eigenvectors.rows() != 3 || cluster.eigenvectors.cols() != 3) {
    //     throw std::invalid_argument("Input dimensions are incorrect.");
    // }

    // Extract the 2D eigenvalues and eigenvectors (x and y components)
    
    Eigen::Vector2d big_mid_eigenvalues =  cluster.eigenvalues.tail(2).cast<double>();
    Eigen::Matrix2d big_mid_eigenvectors =  cluster.eigenvectors.rightCols(2).topRows(2).cast<double>();

    // Construct the 2D covariance matrix
    Eigen::MatrixXd covarianceMatrix = big_mid_eigenvectors * big_mid_eigenvalues.asDiagonal() * big_mid_eigenvectors.transpose();

    covarianceMatrixNormalized = covarianceMatrix / covarianceMatrix.norm();
    // covarianceMatrixNormalized(0,0) = covarianceMatrix(0,0) / (dx * dx);
    // covarianceMatrixNormalized(1,1) = covarianceMatrix(1,1) / (dy * dy);
}

bool ClustersManager::MatchCluster(Cluster new_cluster)
{
    bool found = false;
    Cluster* matchedCluster = nullptr;
    grid_map::Index predictedIndex;

    for (auto& cluster : Clusters_) 
    {
        if (cluster.Status == tODELETE) continue;

        if (!elevationMap_->getIndex(cluster.Point_mass, predictedIndex)) 
        {
            cluster.Status = tODELETE;
            continue;
        }

        Eigen::Vector2d differs;
        differs.x() = new_cluster.Point_mass.x() - cluster.Point_mass.x();
        differs.y() = new_cluster.Point_mass.y() - cluster.Point_mass.y();
        
        // Mahalanobis distance based matching
        float distance = sqrt(differs.transpose() * cluster.ClusterCovarianceMatrix_ * differs);

        if (distance < 0.4) // Distance threshold for matching
        {
            if (!found) {
                matchedCluster = &cluster;
                found = true;
                matchedCluster->differs = differs;
            }
            else {
                // Compare distances, keep the closest match
                if (distance < (matchedCluster->differs.transpose() * matchedCluster->ClusterCovarianceMatrix_ * matchedCluster->differs))
                {
                    matchedCluster = &cluster;
                    matchedCluster->differs = differs;
                }
            }
        }
    }

    if (found) 
    {
        // Update the matched cluster with new values
        matchedCluster->mean_height = new_cluster.mean_height;
        matchedCluster->max_height = new_cluster.max_height;
        matchedCluster->min_height = new_cluster.min_height;
        matchedCluster->grids = new_cluster.grids;
        matchedCluster->pc = new_cluster.pc;
        matchedCluster->Point_mass = new_cluster.Point_mass;
        matchedCluster->Status = uPTODATE;
        matchedCluster->count +=1;

        // Update covariance based on the difference and apply damping
        matchedCluster->ClusterCovarianceMatrix_ = COVARIANCE_DAMPING * (matchedCluster->ClusterCovarianceMatrix_ + matchedCluster->differs * matchedCluster->differs.transpose());

        // std::cout << "Cluster: " << matchedCluster->id << " lifespan: " << matchedCluster->count
        //           << " Covariance Matrix Norm: " << matchedCluster->ClusterCovarianceMatrix_.norm() 
        //           << " Differs Norm: " << matchedCluster->differs.norm() << std::endl;

        return true;
    }

    return false;
}

nlohmann::json ClustersManager::serializeMultipleClustersToJson(std::string directory) {
    nlohmann::json json;
    json["Clusters"] = nlohmann::json::array();  // Create a JSON array for clusters

    for (Cluster& cluster : Clusters_) {
        nlohmann::json cluster_json = cluster.serializeClusterToJson(directory);
        json["Clusters"].push_back(cluster_json);
    }

    return json;
}


// TODO:
// * Add the idea of considering the object's principle axis to the calculation. (Status: Done) Note: Didn't work as expected.
//   - There is no clear formulation on how the Centroid change from instance to instance.
// * Add it to a decoupled node. (Status: Pending)
// * Add a visualization of the clusters. (Status: Pending)


// ISSUES:
// Clusters Sometimes splits from instance to instance.
// Clusters sometimes merge from instance to instance.