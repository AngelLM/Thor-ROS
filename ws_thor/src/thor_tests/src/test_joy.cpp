#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <thread>

class XboxControllerNode : public rclcpp::Node
{
public:
    XboxControllerNode()
    : Node("xbox_controller_node"),
      move_group_(
          std::make_shared<rclcpp::Node>(
              "moveit_arm_node", rclcpp::NodeOptions().parameter_overrides({{"use_sim_time", true}})),
          "arm_group"),
      step_size_(0.005),
      angle_step_(0.005)
    {
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&XboxControllerNode::joyCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Node started, listening to /joy.");
        pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("/current_pose", 10);
    }

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (msg->buttons[6]){
            geometry_msgs::msg::Pose current_pose = move_group_.getCurrentPose().pose;
            RCLCPP_INFO(this->get_logger(), "CURR POSE: %f, %f, %f", current_pose.position.x, current_pose.position.y, current_pose.position.z);
            pose_pub_->publish(current_pose);
        }    
    }

    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    moveit::planning_interface::MoveGroupInterface move_group_;

    // Incrementos en posición y orientación
    double step_size_;
    double angle_step_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto xbox_node = std::make_shared<XboxControllerNode>();

    // Usar un hilo separado para el Executor
    rclcpp::executors::SingleThreadedExecutor executor;
    std::thread executor_thread([&executor, &xbox_node]() {
        executor.add_node(xbox_node);
        executor.spin();
    });

    // Mantener el nodo activo hasta que el programa se cierre manualmente
    RCLCPP_INFO(rclcpp::get_logger("main"), "Press Ctrl+C to exit...");
    rclcpp::on_shutdown([&executor]() {
        RCLCPP_INFO(rclcpp::get_logger("main"), "Shutting down...");
        executor.cancel();
    });

    executor_thread.join();
    return 0;
}
