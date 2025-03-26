#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include "thor_server/action/joint_task.hpp"
#include "thor_server/action/pose_task.hpp"
#include <moveit/move_group_interface/move_group_interface.h>

#include <memory>
#include <thread>


using namespace std::placeholders;

namespace thor_server
{
class TaskServer : public rclcpp::Node
{
public:
  explicit TaskServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("task_server", options)
  {
    RCLCPP_INFO(get_logger(), "Starting the Server");

    // Servidor para JointTask
    joint_task_server_ = rclcpp_action::create_server<thor_server::action::JointTask>(
        this, "joint_task",
        std::bind(&TaskServer::jointTaskGoalCallback, this, _1, _2),
        std::bind(&TaskServer::jointTaskCancelCallback, this, _1),
        std::bind(&TaskServer::jointTaskAcceptedCallback, this, _1));

    // Servidor para PoseTask
    pose_task_server_ = rclcpp_action::create_server<thor_server::action::PoseTask>(
        this, "pose_task",
        std::bind(&TaskServer::poseTaskGoalCallback, this, _1, _2),
        std::bind(&TaskServer::poseTaskCancelCallback, this, _1),
        std::bind(&TaskServer::poseTaskAcceptedCallback, this, _1));
  }

private:
  // Miembros de la clase
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group_;
  std::vector<double> arm_joint_goal_;

  // Servidores de acción
  rclcpp_action::Server<thor_server::action::JointTask>::SharedPtr joint_task_server_;
  rclcpp_action::Server<thor_server::action::PoseTask>::SharedPtr pose_task_server_;

  // Callbacks para JointTask
  rclcpp_action::GoalResponse jointTaskGoalCallback(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const thor_server::action::JointTask::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "Received JointTask goal request");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse jointTaskCancelCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<thor_server::action::JointTask>> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Received request to cancel JointTask");

    if(arm_move_group_){
      arm_move_group_->stop();
    }
    // if(gripper_move_group_){
    //   gripper_move_group_->stop();
    // }
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void jointTaskAcceptedCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<thor_server::action::JointTask>> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "JointTask goal accepted");
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{ std::bind(&TaskServer::jointTaskExecute, this, _1), goal_handle }.detach();
  }

  void jointTaskExecute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<thor_server::action::JointTask>> goal_handle){
    RCLCPP_INFO(get_logger(), "Executing goal");
    if(!arm_move_group_){
      arm_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm_group");
    }
    // if(!gripper_move_group_){
    //   gripper_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "gripper");
    // }

    auto result = std::make_shared<thor_server::action::JointTask::Result>();

    arm_joint_goal_ = {goal_handle->get_goal()->joint1_deg, goal_handle->get_goal()->joint2_deg, goal_handle->get_goal()->joint3_deg, goal_handle->get_goal()->joint4_deg, goal_handle->get_goal()->joint5_deg, goal_handle->get_goal()->joint6_deg};

    // convert joint angles to radians
    for(int i = 0; i < arm_joint_goal_.size(); i++){
      arm_joint_goal_[i] = arm_joint_goal_[i] * M_PI / 180.0;
    }

    arm_move_group_->setStartState(*arm_move_group_->getCurrentState());
    // gripper_move_group_->setStartState(*gripper_move_group_->getCurrentState());

    bool arm_within_bounds = arm_move_group_->setJointValueTarget(arm_joint_goal_);
    // bool gripper_within_bounds gripper_move_group_->setJointValueTarget(gripper_joint_goal_);

    if (!arm_within_bounds){ // || !gripper_within_bounds){
      RCLCPP_ERROR(get_logger(), "Arm goal is out of bounds");
      // result->success = false;
      // goal_handle->succeed(result);
      return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan arm_plan; //, gripper_plan;

    bool arm_plan_success = (arm_move_group_->plan(arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // bool gripper_plan_success = (gripper_move_group_->plan(gripper_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    arm_move_group_->setMaxVelocityScalingFactor(1.0);
    arm_move_group_->setMaxAccelerationScalingFactor(1.0);

    if(arm_plan_success){ // && gripper_plan_success){
      arm_move_group_->execute(arm_plan);
      // gripper_move_group_->execute(gripper_plan);
    }
    else{
      RCLCPP_ERROR(get_logger(), "Failed to plan arm movement");
      // result->success = false;
      // goal_handle->succeed(result);
      return;
    }

    result->success = true;
    goal_handle->succeed(result);
  }

  // Callbacks para PoseTask
  rclcpp_action::GoalResponse poseTaskGoalCallback(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const thor_server::action::PoseTask::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "Received PoseTask goal request");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse poseTaskCancelCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<thor_server::action::PoseTask>> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Received request to cancel PoseTask");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void poseTaskAcceptedCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<thor_server::action::PoseTask>> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "PoseTask goal accepted");
    // Ejecutar la tarea
  }
};
} // namespace thor_server

RCLCPP_COMPONENTS_REGISTER_NODE(thor_server::TaskServer) 