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
    };

    rclcpp::CallbackGroup::SharedPtr cb_group_noexec = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive, false);
    auto subscription_options = rclcpp::SubscriptionOptions();
    subscription_options.callback_group = cb_group_noexec;


    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 1, no_executed);

    timer_ = this->create_wall_timer(
        500ms, std::bind(&KinematicReader::timer_callback, this));
}

void KinematicReader::timer_callback()
{
    // Do something with the data
    nav_msgs::msg::Odometry odom_msg;
    rclcpp::MessageInfo msg_info;

    if (sub_->take(odom_msg, msg_info)) {
        RCLCPP_INFO(this->get_logger(), "Received odometry message");
    }
    else {
        RCLCPP_INFO(this->get_logger(), "No odometry message received");
    }
}

} // namespace kinematic_reader

    
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(kinematic_reader::KinematicReader)
