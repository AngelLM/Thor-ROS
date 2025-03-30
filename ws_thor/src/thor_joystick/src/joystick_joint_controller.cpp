#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "thor_server/action/joint_task.hpp"

class JoystickJointController : public rclcpp::Node
{
public:
    JoystickJointController() : Node("joystick_joint_controller"), current_axes_(8, 0.0), current_joint_positions_(7, 0.0)
    {
        // Subscriber to the /joy topic
        joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&JoystickJointController::joyCallback, this, std::placeholders::_1));

        // Action client for JointTask
        joint_task_client_ = rclcpp_action::create_client<thor_server::action::JointTask>(this, "joint_task");

        // Timer to limit the sending frequency
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&JoystickJointController::sendGoalIfNeeded, this));

        RCLCPP_INFO(this->get_logger(), "Joystick Joint Controller Node started");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
    rclcpp_action::Client<thor_server::action::JointTask>::SharedPtr joint_task_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<double> current_axes_;             // Current values of the axes
    std::vector<double> current_joint_positions_;  // Current positions of the joints
    bool joystick_moved_ = false;                  // Indicates if the joystick moved

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg){
        double deadzone = 0.4;
        joystick_moved_ = false;

        // Check if the axes are outside the dead zone
        for(size_t i = 0; i < msg->axes.size(); i++){
            if((i != 2 && i != 5) && std::abs(msg->axes[i]) < deadzone){
                current_axes_[i] = 0.0;                    
            }
            else if((i == 2 || i == 5) && std::abs(msg->axes[i] - 1) < deadzone){
                current_axes_[i] = 1.0;
            }
            else{
                current_axes_[i] = msg->axes[i];
                joystick_moved_ = true;
            }
        }
    }

    void sendGoalIfNeeded(){
        if(!joystick_moved_){
            return; // Do not send goal if the joystick did not move
        }

        if(!joint_task_client_->wait_for_action_server(std::chrono::seconds(1))){
            RCLCPP_ERROR(this->get_logger(), "JointTask action server not available");
            return;
        }

        double step = 5.0; // Increment for each joystick movement (in degrees)

        // Update the current joint positions
        current_joint_positions_[0] += current_axes_[0] * step;
        current_joint_positions_[1] += current_axes_[1] * step;
        current_joint_positions_[2] += current_axes_[3] * step;
        current_joint_positions_[3] += current_axes_[4] * step;
        current_joint_positions_[4] += current_axes_[6] * step;
        current_joint_positions_[5] += current_axes_[7] * step;
        current_joint_positions_[6] += (current_axes_[2] - 1) * step;
        current_joint_positions_[6] -= (current_axes_[5] - 1) * step;

        // Create a goal for JointTask
        auto goal_msg = thor_server::action::JointTask::Goal();
        goal_msg.joint1_deg = current_joint_positions_[0];
        goal_msg.joint2_deg = current_joint_positions_[1];
        goal_msg.joint3_deg = current_joint_positions_[2];
        goal_msg.joint4_deg = current_joint_positions_[3];
        goal_msg.joint5_deg = current_joint_positions_[4];
        goal_msg.joint6_deg = current_joint_positions_[5];
        goal_msg.gripper_joint_deg = current_joint_positions_[6];

        // Print
        RCLCPP_INFO(this->get_logger(), "Sending JointTask goal: %f, %f, %f, %f, %f, %f, %f", goal_msg.joint1_deg, goal_msg.joint2_deg, goal_msg.joint3_deg, goal_msg.joint4_deg, goal_msg.joint5_deg, goal_msg.joint6_deg, goal_msg.gripper_joint_deg);

        // Send the goal to the action server
        auto send_goal_options = rclcpp_action::Client<thor_server::action::JointTask>::SendGoalOptions();
        send_goal_options.result_callback = [this](const rclcpp_action::ClientGoalHandle<thor_server::action::JointTask>::WrappedResult &result) {
            if(result.code == rclcpp_action::ResultCode::SUCCEEDED){
                RCLCPP_INFO(this->get_logger(), "JointTask executed successfully");
            }
            else{
                RCLCPP_ERROR(this->get_logger(), "Failed to execute JointTask");
            }
        };

        joint_task_client_->async_send_goal(goal_msg, send_goal_options);
        joystick_moved_ = false; // Reset the movement state
    }
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoystickJointController>());
    rclcpp::shutdown();
    return 0;
}