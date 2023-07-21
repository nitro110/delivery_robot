#include "rclcpp/rclcpp.hpp"
#include "swerve_cartographer/pubPose.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PubPose>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
