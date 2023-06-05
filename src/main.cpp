// ROS headers
#include <rclcpp/rclcpp.hpp>

// Project headers
#include <sbg_simu.hpp>

using sbg::SbgSimu;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SbgSimu>());
    rclcpp::shutdown();

    return 0;
}
