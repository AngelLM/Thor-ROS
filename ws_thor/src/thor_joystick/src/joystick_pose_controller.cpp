#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "thor_server/action/pose_task.hpp"

class JoystickPoseController : public rclcpp::Node
{
public:
    JoystickPoseController() : Node("joystick_pose_controller"), current_axes_(8, 0.0), current_pose_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    {
        // Subscriber to the /joy topic
        joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&JoystickPoseController::joyCallback, this, std::placeholders::_1));

        // Action client for PoseTask
        pose_task_client_ = rclcpp_action::create_client<thor_server::action::PoseTask>(this, "pose_task");

        // Timer to limit the sending frequency
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&JoystickPoseController::sendGoalIfNeeded, this));

        RCLCPP_INFO(this->get_logger(), "Joystick Pose Controller Node started");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
    rclcpp_action::Client<thor_server::action::PoseTask>::SharedPtr pose_task_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<double> current_axes_;
    std::array<double, 6> current_pose_; // Updated to store X, Y, Z, roll, pitch, yaw
    bool joystick_moved_ = false;

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        double deadzone = 0.4;
        joystick_moved_ = false;

        // Check if the axes are outside the dead zone
        for (size_t i = 0; i < msg->axes.size(); i++)
        {
            if (i == 2 || i == 5 || std::abs(msg->axes[i]) < deadzone)
            {
                current_axes_[i] = 0.0;
            }
            else
            {
                current_axes_[i] = msg->axes[i];
                joystick_moved_ = true;
            }
        }
    }

    void sendGoalIfNeeded()
    {
        if (!joystick_moved_)
        {
            return; // Do not send goal if the joystick did not move
        }

        if (!pose_task_client_->wait_for_action_server(std::chrono::seconds(1)))
        {
            RCLCPP_ERROR(this->get_logger(), "PoseTask action server not available");
            return;
        }

        double step = 0.1; // Increment for each joystick movement

        // Update the current pose
        current_pose_[0] += current_axes_[0] * step; // X
        current_pose_[1] += current_axes_[1] * step; // Y
        current_pose_[2] += current_axes_[3] * step; // Z
        current_pose_[3] += current_axes_[4] * step; // Roll
        current_pose_[4] += current_axes_[6] * step; // Pitch
        current_pose_[5] += current_axes_[7] * step; // Yaw

        // Create a goal for PoseTask
        auto goal_msg = thor_server::action::PoseTask::Goal();
        goal_msg.x = current_pose_[0];
        goal_msg.y = current_pose_[1];
        goal_msg.z = current_pose_[2];
        goal_msg.roll = current_pose_[3];
        goal_msg.pitch = current_pose_[4];
        goal_msg.yaw = current_pose_[5];

        // Print
        RCLCPP_INFO(this->get_logger(), "Sending PoseTask goal: %f, %f, %f, %f, %f, %f", goal_msg.x, goal_msg.y, goal_msg.z, goal_msg.roll, goal_msg.pitch, goal_msg.yaw);

        // Send the goal to the action server
        auto send_goal_options = rclcpp_action::Client<thor_server::action::PoseTask>::SendGoalOptions();
        send_goal_options.result_callback = [this](const rclcpp_action::ClientGoalHandle<thor_server::action::PoseTask>::WrappedResult &result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                RCLCPP_INFO(this->get_logger(), "PoseTask executed successfully");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to execute PoseTask");
            }
        };

        pose_task_client_->async_send_goal(goal_msg, send_goal_options);
        joystick_moved_ = false; // Reset the movement state
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoystickPoseController>());
    rclcpp::shutdown();
    return 0;
}