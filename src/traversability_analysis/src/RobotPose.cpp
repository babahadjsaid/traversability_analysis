
#include <traversability_analysis/RobotPose.hpp>


RobotPose::RobotPose(rclcpp::Node::SharedPtr node, std::string POSE_TOPIC)
{
    robotPoseSubscriber_ = node->create_subscription<nav_msgs::msg::Odometry>(POSE_TOPIC, qos_imu,std::bind(&RobotPose::OdometryHandler, this, std::placeholders::_1));
    receivedPose_ = false;
    firstPose_ = true;
}
 
void RobotPose::OdometryHandler(nav_msgs::msg::Odometry::SharedPtr poseMsg){
  poseMtx_.lock();
  currentOdom_ = *poseMsg;
  poseMtx_.unlock();
  
}

void RobotPose::UpdatePose(rclcpp::Time currentPointCloudTime){
    poseMtx_.lock();
    auto oldestPoseTime = rclcpp::Time(currentOdom_.header.stamp,RCL_ROS_TIME);
    // Check if point cloud has corresponding robot pose at the beginning
    if (abs(currentPointCloudTime.seconds() - oldestPoseTime.seconds()) <0.01 ) {
        
        tf2::Quaternion orientation;
        tf2::fromMsg(currentOdom_.pose.pose.orientation, orientation);
        double roll, pitch, yaw;
        tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        
        currentTwist_= currentOdom_.twist.twist;
        int index=-1;
        CurrentcovarianceM_ = Eigen::Matrix3d::Zero();
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 2; ++j) {
                covarianceMatrix_(i, j) = currentOdom_.pose.covariance[++index];
                
            }
        }
        // Fill the 3x3 matrix with x, y, yaw uncertainties and covariances
        CurrentcovarianceM_(0, 0) = currentOdom_.pose.covariance[0];   // Covariance for x-x
        CurrentcovarianceM_(1, 1) = currentOdom_.pose.covariance[7];   // Covariance for y-y
        CurrentcovarianceM_(2, 2) = currentOdom_.pose.covariance[35];  // Covariance for yaw-yaw

        // Fill the off-diagonal terms (covariances between x, y, and yaw)
        CurrentcovarianceM_(0, 1) = CurrentcovarianceM_(1, 0) = currentOdom_.pose.covariance[1];  // Covariance between x and y
        CurrentcovarianceM_(0, 2) = CurrentcovarianceM_(2, 0) = currentOdom_.pose.covariance[5];  // Covariance between x and yaw
        CurrentcovarianceM_(1, 2) = CurrentcovarianceM_(2, 1) = currentOdom_.pose.covariance[11]; // Covariance between y and yaw
        
        currentPose_.x() = currentOdom_.pose.pose.position.x;
        currentPose_.y() = currentOdom_.pose.pose.position.y;
        currentPose_.z() = currentOdom_.pose.pose.position.z;
        currentPose_.roll() = roll;
        currentPose_.pitch() = pitch;
        currentPose_.yaw() = yaw;
        receivedPose_ = true;
    }
    poseMtx_.unlock();
}

void RobotPose::UpdatePrev(){
    if (receivedPose_)
        {
            previousPose_ = currentPose_;
            previousCovarianceMatrix_ = covarianceMatrix_;
            receivedPose_ = false;
            firstPose_ = false;
        }
}

bool RobotPose::IsBigVibration(){
    if ( abs(currentTwist_.angular.x) >= 0.5|| abs(currentTwist_.angular.y) >= 0.5 || abs(currentTwist_.angular.z) >= 0.5 )
    {
        std::cout << "A frame is being ignored due to big vibrations " <<std::endl;
        previousPose_ = currentPose_;
        previousCovarianceMatrix_ = covarianceMatrix_;
        return true;
    }
    return false;
}

nlohmann::json RobotPose::serializePoseToJson(){
    nlohmann::json json;
    json["position"] = {currentPose_.x(), currentPose_.y(), currentPose_.z()};
    json["covariance"] = {
        {covarianceMatrix_(0,0), covarianceMatrix_(0,1)},
        {covarianceMatrix_(1,0), covarianceMatrix_(1,1)}
    };
    return json;
}
