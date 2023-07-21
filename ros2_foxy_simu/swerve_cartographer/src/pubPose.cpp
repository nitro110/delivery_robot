#include "swerve_cartographer/pubPose.hpp"

PubPose::PubPose()
    : Node("pose_publisher")
{
    pub_pose_ = this->create_publisher<geometry_msgs::msg::Pose2D>(topic_name_, 10);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    pose_of_robot_.header.frame_id = to_frame_id;
    pose_of_robot_.pose.position.x = 0.0;
    pose_of_robot_.pose.position.y = 0.0;
    pose_of_robot_.pose.position.z = 0.0;
    pose_of_robot_.pose.orientation.x = 0.0;
    pose_of_robot_.pose.orientation.y = 0.0;
    pose_of_robot_.pose.orientation.z = 0.0;
    pose_of_robot_.pose.orientation.w = 1.0;
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&PubPose::timerCallback, this));
}

PubPose::~PubPose()
{
}

void PubPose::timerCallback()
{
    try
    {
        auto tf = tf_buffer_->lookupTransform(from_frame_id, to_frame_id, tf2::TimePointZero);
        pose_of_robot_.header.stamp = tf.header.stamp;
        pose_of_robot_.pose.position.x = tf.transform.translation.x;
        pose_of_robot_.pose.position.y = tf.transform.translation.y;
        pose_of_robot_.pose.position.z = tf.transform.translation.z;
        pose_of_robot_.pose.orientation.x = tf.transform.rotation.x;
        pose_of_robot_.pose.orientation.y = tf.transform.rotation.y;
        pose_of_robot_.pose.orientation.z = tf.transform.rotation.z;
        pose_of_robot_.pose.orientation.w = tf.transform.rotation.w;

        tf2Scalar roll, pitch, yaw;
        tf2::Quaternion q(
            pose_of_robot_.pose.orientation.x,
            pose_of_robot_.pose.orientation.y,
            pose_of_robot_.pose.orientation.z,
            pose_of_robot_.pose.orientation.w);
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        // RCLCPP_INFO(this->get_logger(), "Pose of robot: x = %f, y = %f, z = %f, roll = %f, pitch = %f, yaw = %f",
        //             pose_of_robot_.pose.position.x,
        //             pose_of_robot_.pose.position.y,
        //             pose_of_robot_.pose.position.z,
        //             roll,
        //             pitch,
        //             yaw);
        pose2d_of_robot_.x = pose_of_robot_.pose.position.x;
        pose2d_of_robot_.y = pose_of_robot_.pose.position.y;
        pose2d_of_robot_.theta = yaw;
        pub_pose_->publish(pose2d_of_robot_);
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
}
