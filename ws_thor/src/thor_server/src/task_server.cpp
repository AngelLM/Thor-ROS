#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include "thor_server/action/thor_task.hpp"
#include <moveit/move_group_interface/move_group_interface.h>

#include <memory>
#include <thread>


using namespace std::placeholders;

namespace thor_server
{
class TaskServer : public rclcpp::Node
{
public:
  explicit TaskServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("task_server", options){
    RCLCPP_INFO(get_logger(), "Starting the Server");
    action_server_ = rclcpp_action::create_server<thor_server::action::ThorTask>(
        this, "task_server", std::bind(&TaskServer::goalCallback, this, _1, _2),
        std::bind(&TaskServer::cancelCallback, this, _1),
        std::bind(&TaskServer::acceptedCallback, this, _1));
  }

private:
  rclcpp_action::Server<thor_server::action::ThorTask>::SharedPtr action_server_;
  // std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group_, gripper_move_group_;
  // std::vector<double> arm_joint_goal_, gripper_joint_goal_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group_;
  std::vector<double> arm_joint_goal_;

  rclcpp_action::GoalResponse goalCallback(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const thor_server::action::ThorTask::Goal> goal){
    RCLCPP_INFO(get_logger(), "Received goal request with task_number %d", goal->task_number);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse cancelCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<thor_server::action::ThorTask>> goal_handle){
    RCLCPP_INFO(get_logger(), "Received request to cancel goal");
    if(arm_move_group_){
      arm_move_group_->stop();
    }
    // if(gripper_move_group_){
    //   gripper_move_group_->stop();
    // }
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void acceptedCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<thor_server::action::ThorTask>> goal_handle){
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{ std::bind(&TaskServer::execute, this, _1), goal_handle }.detach();
  }

  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<thor_server::action::ThorTask>> goal_handle){
    RCLCPP_INFO(get_logger(), "Executing goal");
    if(!arm_move_group_){
      arm_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm_group");
    }
    // if(!gripper_move_group_){
    //   gripper_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "gripper");
    // }

    auto result = std::make_shared<thor_server::action::ThorTask::Result>();

    if(goal_handle->get_goal()->task_number == 0){
      arm_joint_goal_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      // gripper_joint_goal_ = {0.0};
    }
    else if(goal_handle->get_goal()->task_number == 1){
      arm_joint_goal_ = {10.0, 20.0, -10.0, 5.0, 10.0, -5.0};
      // gripper_joint_goal_ = {0.0};
    }
    else if(goal_handle->get_goal()->task_number == 2){
      arm_joint_goal_ = {-40.0, -10.0, 25.0, 15.0, -20.0, 90.0};
      // gripper_joint_goal_ = {0.0};
    }
    else{
      RCLCPP_ERROR(get_logger(), "Unknown task number");
      // result->success = false;
      // goal_handle->succeed(result);
      return;
    }

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
};
}  // namespace thor_server

RCLCPP_COMPONENTS_REGISTER_NODE(thor_server::TaskServer)