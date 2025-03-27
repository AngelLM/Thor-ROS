#ifndef THOR_JOYSTICK_CONTROLLER_HPP
#define THOR_JOYSTICK_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

class JoystickMoveItController : public rclcpp::Node {
public:
    JoystickMoveItController();
    
private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
    
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group_;
    geometry_msgs::msg::PoseStamped goal_pose_;
};

#endif // THOR_JOYSTICK_CONTROLLER_HPP