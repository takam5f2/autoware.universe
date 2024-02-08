#include "kinematic_reader/kinematic_reader.hpp"

namespace kinematic_reader {

KinematicReader::KinematicReader(const rclcpp::NodeOptions & options)
: Node ("kinematic_reader", options)
{
    // Create a subscription to the odometry topic

    
    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&KinematicReader::odomCallback, this, std::placeholders::_1));
}

void KinematicReader::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Store the linear and angular velocities
    linear_velocity_ = msg->twist.twist.linear.x;
    angular_velocity_ = msg->twist.twist.angular.z;
}

} // namespace kinematic_reader

    
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(kinematic_reader::KinematicReader)
