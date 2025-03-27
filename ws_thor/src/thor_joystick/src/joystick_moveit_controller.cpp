#include "thor_joystick/joystick_moveit_controller.hpp"

JoystickMoveItController::JoystickMoveItController()
    : Node("joystick_moveit_controller")
{
    // Suscripción al joystick
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&JoystickMoveItController::joyCallback, this, std::placeholders::_1)
    );

    // Publicación de la pose del goal
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/moveit_goal_pose", 10
    );

    // Inicializar la pose objetivo
    goal_pose_.header.frame_id = "world";
    goal_pose_.pose.orientation.w = 1.0;

    RCLCPP_INFO(this->get_logger(), "Nodo de joystick para MoveIt2 iniciado.");
}

void JoystickMoveItController::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    
    // Filtrar el minimo necesario para que se mueva
    if (std::abs(msg->axes[0]) < 0.4) msg->axes[0] = 0.0;
    if (std::abs(msg->axes[1]) < 0.4) msg->axes[1] = 0.0;
    if (std::abs(msg->axes[3]) < 0.4) msg->axes[3] = 0.0;
    if (std::abs(msg->axes[4]) < 0.4) msg->axes[4] = 0.0;
    if (std::abs(msg->axes[6]) < 0.4) msg->axes[6] = 0.0;
    if (std::abs(msg->axes[7]) < 0.4) msg->axes[7] = 0.0;

    // Mover el goal con el joystick
    double step = 0.01;
    double angle_step = 0.1;
    
    // Mover XYZ del goal
    if (msg->axes[0] != 0.0) goal_pose_.pose.position.x += msg->axes[0] * step;
    if (msg->axes[1] != 0.0) goal_pose_.pose.position.y += msg->axes[1] * step;
    if (msg->axes[3] != 0.0) goal_pose_.pose.position.z += msg->axes[3] * step;

    // Mover orientacion del goal
    if (msg->axes[4] != 0.0) goal_pose_.pose.orientation.x += msg->axes[4] * angle_step;
    if (msg->axes[6] != 0.0) goal_pose_.pose.orientation.y += msg->axes[6] * angle_step;
    if (msg->axes[7] != 0.0) goal_pose_.pose.orientation.z += msg->axes[7] * angle_step;

    goal_pose_.header.stamp = this->now();
    goal_pub_->publish(goal_pose_);

    // Si se presiona un botón, enviar la meta a MoveIt2
    if (msg->buttons[0] == 1) {  // Supongamos que el botón "A" (índice 0) es para ejecutar el movimiento
        if(!arm_move_group_){
            arm_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm_group");
        }
        arm_move_group_->setPoseTarget(goal_pose_.pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        
        if (arm_move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            arm_move_group_->execute(plan);
            RCLCPP_INFO(this->get_logger(), "Ejecutando movimiento.");
        } else {
            RCLCPP_WARN(this->get_logger(), "No se pudo planear el movimiento.");
        }
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoystickMoveItController>());
    rclcpp::shutdown();
    return 0;
}