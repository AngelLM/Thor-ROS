#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <memory>


void move_robot(const std::shared_ptr<rclcpp::Node> node)
{
    auto arm_move_group = moveit::planning_interface::MoveGroupInterface(node, "arm_group");
    //auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(node, "gripper");

    std::vector<double> arm_joint_goal {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    //std::vector<double> gripper_joint_goal {-0.7, 0.7};

    bool arm_within_bounds = arm_move_group.setJointValueTarget(arm_joint_goal);
    //bool gripper_within_bounds = gripper_move_group.setJointValueTarget(gripper_joint_goal);

    //if (!arm_within_bounds | !gripper_within_bounds)
    if (!arm_within_bounds)
    {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                    "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
        return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
    //moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
    bool arm_plan_success = (arm_move_group.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    //bool gripper_plan_success = (gripper_move_group.plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    //if(arm_plan_success && gripper_plan_success)
    if(arm_plan_success)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "Planner SUCCEED, moving the arm");
        arm_move_group.move();
        //gripper_move_group.move();
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "One or more planners failed!");
        return;
    }
}


void move_robot_to_approximate_pose(const std::shared_ptr<rclcpp::Node> node)
{
    // Inicializa el MoveGroupInterface para el grupo del brazo
    auto arm_move_group = moveit::planning_interface::MoveGroupInterface(node, "arm_group");

    // Define la pose objetivo
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.0;  // Posición en X
    target_pose.position.y = 0.255;  // Posición en Y
    target_pose.position.z = 0.45;  // Posición en Z

    // Define la orientación en forma de cuaterniones
    target_pose.orientation.x = -0.67;  // Sin rotación
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 0.75;

    for(int i=0; i<10; i++){

        target_pose.position.z -= 0.025;

        // Configura el objetivo utilizando una solución aproximada
        bool ik_success = arm_move_group.setApproximateJointValueTarget(target_pose);

        if (!ik_success)
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), 
                        "Unable to find an IK solution for the given pose, attempting with approximation.");
        }

        // Planifica y ejecuta el movimiento
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool plan_success = (arm_move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (plan_success)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Planner SUCCEED, moving the arm");
            arm_move_group.move();
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Planner failed!");
        }

        // Limpia los objetivos después de ejecutar
        arm_move_group.clearPoseTargets();
    }
}



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("simple_moveit_interface");
  //move_robot(node);
  move_robot_to_approximate_pose(node);

  rclcpp::spin(node);
  rclcpp::shutdown();
}