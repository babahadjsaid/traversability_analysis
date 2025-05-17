
#pragma once
#include <traversability_analysis/utilities.hpp>





class RobotPose
{
public:
    RobotPose(rclcpp::Node::SharedPtr node, std::string POSE_TOPIC);
    
    void OdometryHandler(nav_msgs::msg::Odometry::SharedPtr poseMsg);
    void UpdatePose(rclcpp::Time currentPointCloudTime);
    bool IsBigVibration();
    void UpdatePrev();
    nlohmann::json serializePoseToJson();
    std::mutex poseMtx_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robotPoseSubscriber_;
    nav_msgs::msg::Odometry currentOdom_;
    Pose  currentPose_,previousPose_;
    geometry_msgs::msg::Twist currentTwist_;
    bool receivedPose_, firstPose_;
    Eigen::Matrix2d covarianceMatrix_, previousCovarianceMatrix_;
    Eigen::Matrix3d CurrentcovarianceM_;
};