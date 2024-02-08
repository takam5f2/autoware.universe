#ifndef KINEMATIC_READER__KINEMATIC_READER_HPP_
#define KINEMATIC_READER__KINEMATIC_READER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>


#include <memory>
#include <string>
#include <utility>

namespace kinematic_reader
{

using nav_msgs::msg::Odometry;


  class KinematicReader : public rclcpp::Node
  {
  public:
    explicit KinematicReader(const rclcpp::NodeOptions & options);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
    Odometry::ConstSharedPtr current_kinematics_;

  };
} // namespace kinematic_reader

#endif  