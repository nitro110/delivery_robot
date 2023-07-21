#ifndef PUBPOSE_HPP_
#define PUBPOSE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>

class PubPose : public rclcpp::Node
{
public:
    PubPose();
    ~PubPose();
    void timerCallback();

private:
    std::string topic_name_ = "robot_pose";
    std::string from_frame_id = "map";
    std::string to_frame_id = "base_footprint";
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pub_pose_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    geometry_msgs::msg::PoseStamped pose_of_robot_;
    geometry_msgs::msg::Pose2D pose2d_of_robot_;
    rclcpp::TimerBase::SharedPtr timer_{nullptr};
};

#endif // PUBPOSE_HPP_