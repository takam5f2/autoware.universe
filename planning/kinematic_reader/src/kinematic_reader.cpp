#include "kinematic_reader/kinematic_reader.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace kinematic_reader {

KinematicReader::KinematicReader(const rclcpp::NodeOptions & options)
: Node ("kinematic_reader", options)
{
    // Create a subscription to the odometry topic

    auto no_executed = [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) -> void {
      assert(false);
      current_kinematics_ = msg;
      RCLCPP_INFO(this->get_logger(), "Received odometry message");
      RCLCPP_INFO(
        this->get_logger(), "Position: x = %f, y = %f, z = %f",
        current_kinematics_->pose.pose.position.x, current_kinematics_->pose.pose.position.y,
        current_kinematics_->pose.pose.position.z);
    };

    rclcpp::CallbackGroup::SharedPtr cb_group_noexec = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive, false);
    auto subscription_options = rclcpp::SubscriptionOptions();
    subscription_options.callback_group = cb_group_noexec;


    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "~/odom", 1, no_executed, subscription_options);

    timer_ = this->create_wall_timer(
        1000ms, std::bind(&KinematicReader::timer_callback, this));
}

void KinematicReader::timer_callback()
{
    // Do something with the data
    nav_msgs::msg::Odometry odom_msg;
    rclcpp::MessageInfo msg_info;

    if (sub_->is_serialized()) {
        RCLCPP_INFO(this->get_logger(), "Message is serialized");
    }
    else {
        RCLCPP_INFO(this->get_logger(), "Message is not serialized");
    }

    if (sub_->take(odom_msg, msg_info)) {
        current_kinematics_ = std::make_shared<nav_msgs::msg::Odometry>(odom_msg);
        RCLCPP_INFO(this->get_logger(), "Received odometry message");
        RCLCPP_INFO(this->get_logger(), "Position: x = %f, y = %f, z = %f",
            current_kinematics_->pose.pose.position.x,
            current_kinematics_->pose.pose.position.y,
            current_kinematics_->pose.pose.position.z);
    }
    else {
        RCLCPP_INFO(this->get_logger(), "No odometry message received");
        current_kinematics_ = std::make_shared<nav_msgs::msg::Odometry>(odom_msg);
    }
}

} // namespace kinematic_reader

    
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(kinematic_reader::KinematicReader)
