#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "stereo-inertial-node.hpp"

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
    if(argc < 4)
    {
        std::cerr << "\nUsage: ros2 run orbslam stereo path_to_vocabulary path_to_settings do_rectify [do_equalize]" << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    if(argc == 4)
    {
        argv[4] = "false";
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
    ORB_SLAM3::System pSLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_STEREO, visualization);

    auto node = std::make_shared<StereoInertialNode>(&pSLAM, argv[2], argv[3], argv[4]);
    std::cout << "============================" << std::endl;

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
