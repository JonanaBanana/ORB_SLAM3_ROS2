#ifndef __MONOCULAR_INERTIAL_SLAM_NODE_HPP__
#define __MONOCULAR_INERTIAL_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <cv_bridge/cv_bridge.h>
#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"
#include "ImuTypes.h"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "utility.hpp"
#include <mutex>
#include <queue>
#include <thread>

class MonocularInertialSlamNode : public rclcpp::Node
{
public:
    MonocularInertialSlamNode(ORB_SLAM3::System* pSLAM);

    ~MonocularInertialSlamNode();

private:
    using ImageMsg = sensor_msgs::msg::Image;

    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);
    void GrabImu(const sensor_msgs::msg::Imu::SharedPtr msg);
    void SyncWithImu();

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    // TF broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    ORB_SLAM3::System* m_SLAM;

    // Previous camera pose
    Sophus::SE3f prev_pose_;
    bool has_prev_pose_ = false;

    // Accumulated odometry pose
    Sophus::SE3f accumulated_pose_;

    cv_bridge::CvImagePtr m_cvImPtr;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_subscriber;

    // Image buffer
    std::queue<sensor_msgs::msg::Image::SharedPtr> img_buf_;
    std::mutex img_mutex_;

    // IMU buffer
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    std::queue<sensor_msgs::msg::Imu::SharedPtr> imu_buf_;
    std::mutex imu_mutex_;

    // Sync thread
    std::thread* sync_thread_;

    // Path
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    nav_msgs::msg::Path path_msg_;
    rclcpp::Time prev_stamp_;
    bool has_prev_stamp_ = false;
};

#endif