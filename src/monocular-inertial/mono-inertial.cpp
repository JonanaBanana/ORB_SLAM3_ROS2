#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "mono-inertial-slam-node.hpp"

#include "System.h"

#ifdef ORBSLAM3_VIZ
static constexpr int orbslam3_viz_ = ORBSLAM3_VIZ;
#else
static constexpr int orbslam3_viz_ = 0; // By default, visualization is off.
#endif


int main(int argc, char **argv)
{
    if(argc < 3)
    {
        std::cerr << "\nUsage: ros2 run orbslam mono path_to_vocabulary path_to_settings" << std::endl;
        return 1;
    }

    bool visualization = false;
    if (orbslam3_viz_ == 1){
        visualization = true;
    }

    rclcpp::init(argc, argv);
    std::cout << "\nORBSLAM3_ROS2 VISUALIZATION = "<< visualization << std::endl;
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_MONOCULAR, visualization);

    auto node = std::make_shared<MonocularInertialSlamNode>(&SLAM);
    std::cout << "============================ " << std::endl;

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}