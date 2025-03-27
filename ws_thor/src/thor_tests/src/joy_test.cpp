#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

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
}

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if(false){
            if (msg->buttons[0]){
                
                RCLCPP_INFO(this->get_logger(), "Checking current state...");
                if (move_group_.getCurrentState() == nullptr)
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to get current state of the robot.");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "Successfully retrieved current state.");

                geometry_msgs::msg::Pose current_pose = move_group_.getCurrentPose().pose;
                RCLCPP_INFO(this->get_logger(), "Current pose retrieved: [x: %f, y: %f, z: %f]", 
                    current_pose.position.x, current_pose.position.y, current_pose.position.z);
            }
        }


            
        // Recupera la pose actual del extremo del brazo
        // geometry_msgs::msg::Pose current_pose = move_group_.getCurrentPose().pose;
        // geometry_msgs::msg::Pose last_pose = current_pose;

        if (mode_ == 0){
            if (msg->buttons[0]){
                arm_joint_goal[0] += step_size_;
                action = true;
            }
            if (msg->buttons[1]){
                arm_joint_goal[0] -= step_size_;
                action = true;
            }
            if (msg->buttons[2]){
                arm_joint_goal[1] += step_size_;
                action = true;
            }
            if (msg->buttons[3]){
                arm_joint_goal[1] -= step_size_;
                action = true;
            }

            if (msg->buttons[4]){
                arm_joint_goal[2] += step_size_;
                action = true;
            }

            if (msg->buttons[5]){
                arm_joint_goal[2] -= step_size_;
                action = true;
            }
        }
        else{
            if (msg->buttons[0]){
                target_pose.position.x += step_size_;  // Botón A
                action = true;
            }
            if (msg->buttons[1]){
                target_pose.position.x -= step_size_;  // Botón B
                action = true;
            }
            if (msg->buttons[2]){
                target_pose.position.y += step_size_;  // Botón X
                action = true;
            }
            if (msg->buttons[3]){
                target_pose.position.y -= step_size_;  // Botón Y
                action = true;
            }

            if (msg->buttons[4]){
                target_pose.position.z += step_size_;  // Botón LB
                action = true;
            }

            if (msg->buttons[5]){
                target_pose.position.z -= step_size_;  // Botón RB
                action = true;
            }

            target_pose.orientation.x = 0.0;
            target_pose.orientation.y = 0.0;
            target_pose.orientation.z = 0.0;
            target_pose.orientation.w = 1.0;
        }


        // // Joystick izquierdo controla X e Y
        // current_pose.position.x += msg->axes[0] * step_size_;  // Eje X del joystick izquierdo
        // current_pose.position.y += msg->axes[1] * step_size_;  // Eje Y del joystick izquierdo

        // // Joystick derecho controla Z y Roll
        // current_pose.position.z += msg->axes[4] * step_size_;  // Eje Y del joystick derecho
        // roll_ += msg->axes[2] * angle_step_;                  // Eje X del joystick derecho

        // // Botones A/B controlan Pitch
        // if (msg->buttons[0]) pitch_ += angle_step_;  // Botón A
        // if (msg->buttons[1]) pitch_ -= angle_step_;  // Botón B

        // // Botones X/Y controlan Yaw
        // if (msg->buttons[2]) yaw_ += angle_step_;   // Botón X
        // if (msg->buttons[3]) yaw_ -= angle_step_;   // Botón Y

        // Configura orientación con Roll, Pitch, Yaw
        // tf2::Quaternion quaternion;
        // quaternion.setRPY(roll_, pitch_, yaw_);
        // current_pose.orientation = tf2::toMsg(quaternion);
        


        // if (current_pose.position.x != last_pose.position.x || current_pose.position.y != last_pose.position.y || current_pose.position.z != last_pose.position.z){
        //     RCLCPP_INFO(this->get_logger(), "--------");
        //     RCLCPP_INFO(this->get_logger(), "CURR POSE: %f, %f, %f", current_pose.position.x, current_pose.position.y, current_pose.position.z);
        //     RCLCPP_INFO(this->get_logger(), "LAST POSE: %f, %f, %f", last_pose.position.x, last_pose.position.y, last_pose.position.z);
        //     RCLCPP_INFO(this->get_logger(), "--------");
        // }



        // Verifica si el objetivo está dentro de los límites
        if(action){
            action = false;
            
            
            if (mode_ == 0){
                bool arm_within_bounds = move_group_.setJointValueTarget(arm_joint_goal);

                if (!arm_within_bounds)
                {
                    RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                                "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
                    return;
                }

                moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
                //moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
                bool arm_plan_success = (move_group_.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);
                //bool gripper_plan_success = (gripper_move_group.plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);

                //if(arm_plan_success && gripper_plan_success)
                if(arm_plan_success)
                {
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                                "Planner SUCCEED, moving the arm");
                    move_group_.move();
                    //gripper_move_group.move();
                }
                else
                {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                                "One or more planners failed!");
                    return;
                }
            }
            else{

                RCLCPP_INFO(this->get_logger(), "POSE: %f, %f, %f", target_pose.position.x, target_pose.position.y, target_pose.position.z);

                bool ik_success = move_group_.setApproximateJointValueTarget(target_pose);

                if (!ik_success)
                {
                    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), 
                                "Unable to find an IK solution for the given pose, attempting with approximation.");
                }

                // Planifica y ejecuta el movimiento
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                bool plan_success = (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

                if (plan_success)
                {
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Planner SUCCEED, moving the arm");
                    move_group_.move();
                }
                else
                {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Planner failed!");
                }

                // Limpia los objetivos después de ejecutar
                move_group_.clearPoseTargets();
            }


            // if (!move_group_.setPoseTarget(current_pose))
            // {
            //     RCLCPP_WARN(this->get_logger(), "Pose target is outside of joint limits or invalid.");
            //     return;
            // }

            // // Planifica y ejecuta si el objetivo es válido
            // moveit::planning_interface::MoveGroupInterface::Plan plan;
            // if (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
            // {
            //     RCLCPP_INFO(this->get_logger(), "Planner succeeded, moving to new pose...");
            //     move_group_.move();
            // }
            // else
            // {
            //     RCLCPP_WARN(this->get_logger(), "Planner failed to find a solution.");
            // }


            

        }       
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    moveit::planning_interface::MoveGroupInterface move_group_;

    // Incrementos en posición y orientación
    double step_size_;
    double angle_step_;
    double roll_ = 0.0, pitch_ = 0.0, yaw_ = 0.0;
    bool action = false;
    int mode_ = 1; // 0: ARTICULATIONS, 1: CARTESIAN

    std::vector<double> arm_joint_goal {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    geometry_msgs::msg::Pose target_pose;
    
    // target_pose.position.x = 0.0;
    // target_pose.position.y = 0.0;
    // target_pose.position.z = 0.65;

    // target_pose.orientation.x = 0.0;
    // target_pose.orientation.y = 0.0;
    // target_pose.orientation.z = 0.0;
    // target_pose.orientation.w = 0.0;

    

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<XboxControllerNode>());
    rclcpp::shutdown();
    return 0;
}
