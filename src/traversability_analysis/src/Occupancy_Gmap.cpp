// Description: Occupancy grid class implementation

#include "traversability_analysis/Occupancy_Gmap.hpp"



  
std::vector<Eigen::Vector3d> OccupancyGrid::getRelativeEllipseIndices(Eigen::Matrix2d covarianceMatrix) {
    double halfResolution = 0.5 * Map_->info.resolution;
    double uncertaintyFactor = 2.486;  // sqrt(6.18)
    const float minimalWeight = std::numeric_limits<float>::epsilon() * static_cast<float>(2.0);

    // Eigen decomposition of 2x2 covariance matrix
    Eigen::EigenSolver<Eigen::Matrix2d> solver(covarianceMatrix);
    Eigen::Array2d eigenvalues = solver.eigenvalues().real().cwiseAbs();

    // Sort eigenvalues (and corresponding eigenvectors) if needed
    // But generally, the order doesn't affect the correctness since we handle both directions.
    double lambda1 = eigenvalues(0);
    double lambda2 = eigenvalues(1);

    // Extract rotation matrix from eigenvectors
    // solver.eigenvectors() returns a 2x2 complex matrix, we take the real part
    Eigen::Matrix2d rotationMatrix = solver.eigenvectors().real();

    // Standard deviations
    double sigma_x = sqrt(lambda1);
    double sigma_y = sqrt(lambda2);

    // Ellipse lengths: major and minor axis
    Eigen::Array2d lengths = 2 * uncertaintyFactor * Eigen::Array2d(sigma_x, sigma_y);

    // Determine bounding box for the ellipse
    double maxSigma = std::max(sigma_x, sigma_y);
    double maxLength = 2 * uncertaintyFactor * maxSigma;
    int maxOffset = static_cast<int>(std::ceil(maxLength / Map_->info.resolution));

    std::vector<Eigen::Vector3d> relativeIndices;
    relativeIndices.reserve((2*maxOffset+1)*(2*maxOffset+1)); // Reserve space to reduce allocations

    // Iterate over the bounding box
    for (int i = -maxOffset; i <= maxOffset; ++i) {
        for (int j = -maxOffset; j <= maxOffset; ++j) {
            // Cell center relative position (in world frame centered at ellipse origin)
            double rel_x = i * Map_->info.resolution;
            double rel_y = j * Map_->info.resolution;

            // Compute probability weights using the normal CDF differences
            float probability_x = static_cast<float>(
                cumulativeDistributionFunction(rel_x + halfResolution, 0.0, sigma_x) -
                cumulativeDistributionFunction(rel_x - halfResolution, 0.0, sigma_x));

            float probability_y = static_cast<float>(
                cumulativeDistributionFunction(rel_y + halfResolution, 0.0, sigma_y) -
                cumulativeDistributionFunction(rel_y - halfResolution, 0.0, sigma_y));

            // Combine probabilities
            float weight = std::max(minimalWeight, probability_x * probability_y);

            // Transform the relative position into the ellipse coordinate system
            Eigen::Vector2d relativePos(rel_x, rel_y);
            Eigen::Vector2d transformedPos = rotationMatrix.transpose() * relativePos;

            // Check if inside the ellipse
            double ellipseValue = (transformedPos.x() * transformedPos.x()) / (lengths.x() * lengths.x()) +
                                  (transformedPos.y() * transformedPos.y()) / (lengths.y() * lengths.y());

            if (ellipseValue <= 1.0) {
                // Store x-index, y-index, and weight
                relativeIndices.emplace_back(i, j, weight);
            }
        }
    }

    return relativeIndices;
}


OccupancyGrid::OccupancyGrid(int height,int width,float res, Position origin_to_world,int maxcell, int numcell)
{
  Map_  = new nav_msgs::msg::OccupancyGrid();
  Map_->header.frame_id = "map";
  Map_->data.resize(height * width, NO_INFORMATION);
  Variance_ = Eigen::MatrixXd::Constant(height, width, -1.0);


  Map_->info.height = height ;
  Map_->info.width = width;
  Map_->info.resolution = res;
  Map_->info.origin.position.x = origin_to_world.x();
  Map_->info.origin.position.y = origin_to_world.y();
  num_cell_to_increment_= numcell;
  max_cell_to_increment_ = maxcell;
  num_cell_to_increment_m_ = num_cell_to_increment_ * res;
}

void OccupancyGrid::UpdateCell(Position pos, double z, Eigen::Matrix2d CovM) {
    int i,j;
    std::tie(i, j) = GetIndexWorldPos(pos);
    if (i < 0 || i >= (int)Map_->info.width || j < 0 || j >= (int)Map_->info.height) return;

    double mu_prior = static_cast<double>(GetCost(i,j)) / static_cast<double>(LETHAL_OBSTACLE);
    double sigma_prior = GetWeight(i,j);
    // If cell has no prior, initialize it
    double R = CovM.trace();
    if (sigma_prior < 0) {
        SetCost(i,j, (int8_t)(z * LETHAL_OBSTACLE));
        SetWeight(i,j, 0.7);// modify the inittial weight.
        return;
    }


    // Prevent probabilities from reaching 0 or 1, to avoid log(0)
    double p_prior = std::max(std::min(mu_prior, 0.999999), 0.000001);
    double p_meas  = std::max(std::min(z,        0.999999), 0.000001);

    // Convert to log-odds
    double l_prior = std::log(p_prior / (1.0 - p_prior));
    double l_meas  = std::log(p_meas  / (1.0 - p_meas));

    // Kalman gain
    double K = sigma_prior / (sigma_prior + R);

    // Update in log-odds space
    double l_post = l_prior + K * (l_meas - l_prior);

    // Convert back to probability
    double mu_post = 1.0 / (1.0 + std::exp(-l_post));
    double sigma_post = (1.0 - K) * sigma_prior;

    SetCost(i,j, (int8_t)(mu_post * LETHAL_OBSTACLE));
    SetWeight(i,j, sigma_post);
}



std::pair<int, int> OccupancyGrid::GetIndexWorldPos(Position pos){
  // make some checks here.
  // if (Map_->data.empty() || Map_->info.resolution == 0) return -2;
  int i = static_cast<int>((pos.x() - Map_->info.origin.position.x )/ Map_->info.resolution);
  int j = static_cast<int>((pos.y() - Map_->info.origin.position.y) / Map_->info.resolution);
  return {i,j};
}

float OccupancyGrid::cumulativeDistributionFunction(float x, float mean, float standardDeviation) {
  return 0.5 * erfc(-(x - mean) / (standardDeviation * sqrt(2.0)));
}

Eigen::Matrix2d OccupancyGrid::ComputeGlobalCovariance(Pose robot_pose, Eigen::Matrix3d pose_covariance, Position local_pos) {
    /*
     * Compute the covariance of a local cell when projected into the global map frame.
     * 
     * Parameters:
     * - robot_pose: Position (x_r, y_r, theta_r) - Robot's global pose
     * - pose_covariance: 3x3 Eigen::Matrix3d - Robot's pose covariance matrix
     * - local_pos: Position (x_l, y_l) - Cell's local position relative to the robot
     * 
     * Returns:
     * - global_covariance: 2x2 Eigen::Matrix2d - Covariance of the projected cell in the global map
     */
    
    double x_r = robot_pose.x(), y_r = robot_pose.y(), theta_r = robot_pose.yaw();
    double x_l = local_pos.x(), y_l = local_pos.y();

    // Compute sine and cosine of theta_r
    double s_theta = sin(theta_r);
    double c_theta = cos(theta_r);

    // Partial derivatives (Jacobian components)
    double d_xg_dtheta = -s_theta * x_l - c_theta * y_l;
    double d_yg_dtheta = c_theta * x_l - s_theta * y_l;

    // Jacobian matrix J (2x3)
    Eigen::Matrix<double, 2, 3> J;
    J << 1.0, 0, d_xg_dtheta,
         0, 1.0, d_yg_dtheta;

    // Compute the global covariance
    Eigen::Matrix2d global_covariance = J * pose_covariance * J.transpose();

    return global_covariance;
}


int8_t OccupancyGrid::GetCost(int index_i, int index_j){
  if (index_i>=0 && index_i < Map_->info.width && index_j>=0 && index_j < Map_->info.height) return Map_->data[index_i + Map_->info.width * index_j];
 return -2;
   
}

int8_t OccupancyGrid::GetCost(Position pos){
  int i,j;
  std::tie(i, j) = GetIndexWorldPos(pos);
  int index = i + Map_->info.width * j;
  if (i>=0 && i < Map_->info.width && j>=0 && j < Map_->info.height) 
  {
    return Map_->data[index];
  }
  
  return -2;
}


void OccupancyGrid::SetCost(Position pos, int8_t value){
  int i,j;
  std::tie(i, j) = GetIndexWorldPos(pos);
  int index = i + Map_->info.width * j;
  if (i>=0 && i < Map_->info.width && j>=0 && j < Map_->info.height) Map_->data[index] = value;
}

void OccupancyGrid::SetCost(int index_i, int index_j, int8_t value){
  if (index_i>= Map_->info.width || index_j>= Map_->info.height || index_i<0 || index_j<0) return;
  Map_->data[index_i + Map_->info.width * index_j] =  value;//std::max(Map_->data[index_i + Map_->info.width * index_j], value);//(Map_->data[index_i + Map_->info.width * index_j] != NO_INFORMATION)  ? std::min(Map_->data[index_i + Map_->info.width * index_j], value) : value;

}

double OccupancyGrid::GetWeight(Position pos){
  int i,j;
  std::tie(i, j) = GetIndexWorldPos(pos);
  if (i>=0 && i < Map_->info.width && j>=0 && j < Map_->info.height) 
    return Variance_(i,j);

  return -2;
}

double OccupancyGrid::GetWeight(int i, int j){
  if (i>=0 && i < Map_->info.width && j>=0 && j < Map_->info.height) 
    return Variance_(i,j);

  return -2;
}

void OccupancyGrid::SetWeight(Position pos, double value){
  int i,j;
  std::tie(i, j) = GetIndexWorldPos(pos);
  if (i>=0 && i < Map_->info.width && j>=0 && j < Map_->info.height)
    Variance_(i,j) = value;

}

void OccupancyGrid::SetWeight(int i, int j, double value){
  if (i>=0 && i < Map_->info.width && j>=0 && j < Map_->info.height)
    Variance_(i,j) = value;

}

void OccupancyGrid::CheckAndExpandMap(Position robotPos){
  
  if (Map_->data.empty()) return;
  int i = (robotPos.x() - Map_->info.origin.position.x )/ Map_->info.resolution;
  int j = (robotPos.y() - Map_->info.origin.position.y) / Map_->info.resolution;
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
  tmpVar = Eigen::MatrixXd::Constant(new_height, new_width, -1.0);
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
        tmpVar(i + incrX, (j + incrY) ) = Variance_(i,j);
      }
  }
  delete Map_;
  //delete &Variance_;
  Map_ = tmp;
  Variance_ = tmpVar;


  MapMtx_.unlock();
}

