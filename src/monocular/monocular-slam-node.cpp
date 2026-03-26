#include "monocular-slam-node.hpp"

#include<opencv2/core/core.hpp>

#include <sophus/se3.hpp>

#include <iostream>

#include <Eigen/Dense>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2")
{
    declare_parameter("image_topic",  "/low_light_down_misp_decoded");
    declare_parameter("odometry_topic",  "/orbslam3/odom");
    declare_parameter("path_topic",  "/orbslam3/path");
    declare_parameter("parent_frame_id",  "odom");
    declare_parameter("child_frame_id",  "orbslam3/base_link");

    const std::string image_topic  = get_parameter("image_topic").as_string();
    const std::string odom_topic = get_parameter("odometry_topic").as_string();
    const std::string path_topic = get_parameter("path_topic").as_string();
    const std::string parent_frame_id = get_parameter("parent_frame_id").as_string();
    const std::string child_frame_id = get_parameter("child_frame_id").as_string();

    m_SLAM = pSLAM;

    m_image_subscriber = this->create_subscription<ImageMsg>(
        image_topic,
        10,
        std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_topic, 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    
    std::cout << "slam changed" << std::endl;
}

MonocularSlamNode::~MonocularSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    try
    {
        m_cvImPtr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    //std::cout<<"one frame has been sent"<<std::endl;
    Sophus::SE3f Tcw = m_SLAM->TrackMonocular(m_cvImPtr->image, Utility::StampToSec(msg->header.stamp));

    if (Tcw.matrix().isZero(0)) {
        RCLCPP_WARN(this->get_logger(), "Invalid pose from ORB-SLAM3");
        return;
    }

    // 2. Invert to get camera-in-world
    Sophus::SE3f Twc = Tcw.inverse();

    // --- Compute delta and accumulate ---
    if (!has_prev_pose_) {
        prev_pose_ = Twc;
        has_prev_pose_ = true;
    }

    // Delta between frames
    Sophus::SE3f delta = prev_pose_.inverse() * Twc;
    accumulated_pose_ = accumulated_pose_ * delta;
    prev_pose_ = Twc;

    // 3. Extract translation & rotation
    Eigen::Vector3f t_slam = accumulated_pose_.translation();
    Eigen::Matrix3f R_slam = accumulated_pose_.rotationMatrix();

    // 5. Convert to quaternion
    Eigen::Quaternionf q_ros(R_slam);
    q_ros.normalize();

    // 6. Publish Odometry
    nav_msgs::msg::Odometry odom;

    // ESD -> NWU position
    odom.pose.pose.position.x = -t_slam.y();  // north = -south
    odom.pose.pose.position.y = -t_slam.x();  // west = -east
    odom.pose.pose.position.z = -t_slam.z();  // up = -down

    // ESD -> NWU quaternion
    odom.pose.pose.orientation.w =  q_ros.w();
    odom.pose.pose.orientation.x = -q_ros.y();
    odom.pose.pose.orientation.y = -q_ros.x();
    odom.pose.pose.orientation.z = -q_ros.z();

    // --- Twist estimation from delta pose / dt ---
    rclcpp::Time current_stamp = msg->header.stamp;

    if (has_prev_stamp_) {
        double dt = (current_stamp - prev_stamp_).seconds();

        if (dt > 1e-6) {
            // Delta in body frame (already body-relative since delta = prev⁻¹ * curr)
            Eigen::Vector3f dt_slam = delta.translation();
            Eigen::AngleAxisf aa(delta.rotationMatrix());

            // Transform linear velocity from ESD to NWU body frame
            odom.twist.twist.linear.x = -dt_slam.y() / dt;  // north = -south
            odom.twist.twist.linear.y = -dt_slam.x() / dt;  // west = -east
            odom.twist.twist.linear.z = -dt_slam.z() / dt;  // up = -down

            // Angular velocity from angle-axis, also ESD -> NWU
            Eigen::Vector3f omega = aa.angle() * aa.axis() / dt;
            odom.twist.twist.angular.x = -omega.y();  // north = -south
            odom.twist.twist.angular.y = -omega.x();  // west = -east
            odom.twist.twist.angular.z = -omega.z();  // up = -down
        }
    }

    prev_stamp_ = current_stamp;
    has_prev_stamp_ = true;

    odom.header.stamp = msg->header.stamp;
    odom.header.frame_id = get_parameter("parent_frame_id").as_string();
    odom.child_frame_id = get_parameter("child_frame_id").as_string();

    odom_pub_->publish(odom);

    // 7. Publish TF
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header = odom.header;
    tf_msg.child_frame_id = odom.child_frame_id;

    tf_msg.transform.translation.x = odom.pose.pose.position.x;
    tf_msg.transform.translation.y = odom.pose.pose.position.y;
    tf_msg.transform.translation.z = odom.pose.pose.position.z;

    tf_msg.transform.rotation.x = odom.pose.pose.orientation.x;
    tf_msg.transform.rotation.y = odom.pose.pose.orientation.y;
    tf_msg.transform.rotation.z = odom.pose.pose.orientation.z;
    tf_msg.transform.rotation.w = odom.pose.pose.orientation.w;

    tf_broadcaster_->sendTransform(tf_msg);

    // 8. Publish Path
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = odom.header;
    pose_stamped.pose = odom.pose.pose;

    path_msg_.header = odom.header;
    path_msg_.poses.push_back(pose_stamped);

    path_pub_->publish(path_msg_);
}