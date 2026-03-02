#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rgbd-slam-node.hpp"

#include "System.h"

#ifdef ORBSLAM3_VIZ
static constexpr int orbslam3_viz_ = ORBSLAM3_VIZ;
std::cout << "\nORBSLAM3_ROS2 VISUALIZATION: "<< orbslam3_viz_ << std::endl;
#else
static constexpr int orbslam3_viz_ = 0; // By default, visualization is off.
std::cout << "\nORBSLAM3_ROS2 VISUALIZATION OFF BY DEFAULT" << std::endl;
#endif

int main(int argc, char **argv)
{
    if(argc < 3)
    {
        std::cerr << "\nUsage: ros2 run orbslam rgbd path_to_vocabulary path_to_settings" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);

    // malloc error using new.. try shared ptr
    // Create SLAM system. It initializes all system threads and gets ready to process frames.

    if (orbslam3_viz_ == 1){
        bool visualization = true;

    }
    else {
        bool visualization = false;
    }
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::RGBD, visualization);

    auto node = std::make_shared<RgbdSlamNode>(&SLAM);
    std::cout << "============================ " << std::endl;

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
