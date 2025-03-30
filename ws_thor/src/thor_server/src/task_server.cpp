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

    // JointTask Server
    joint_task_server_ = rclcpp_action::create_server<thor_server::action::JointTask>(
        this, "joint_task",
        std::bind(&TaskServer::jointTaskGoalCallback, this, _1, _2),
        std::bind(&TaskServer::jointTaskCancelCallback, this, _1),
        std::bind(&TaskServer::jointTaskAcceptedCallback, this, _1));

    // PoseTask Server
    pose_task_server_ = rclcpp_action::create_server<thor_server::action::PoseTask>(
        this, "pose_task",
        std::bind(&TaskServer::poseTaskGoalCallback, this, _1, _2),
        std::bind(&TaskServer::poseTaskCancelCallback, this, _1),
        std::bind(&TaskServer::poseTaskAcceptedCallback, this, _1));
  }

private:
  // Class members
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group_, gripper_move_group_;
  std::vector<double> arm_joint_goal_, gripper_joint_goal_;

  // Action servers
  rclcpp_action::Server<thor_server::action::JointTask>::SharedPtr joint_task_server_;
  rclcpp_action::Server<thor_server::action::PoseTask>::SharedPtr pose_task_server_;

  // JointTask callbacks
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
    if(gripper_move_group_){
      gripper_move_group_->stop();
    }
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
    if(!gripper_move_group_){
      gripper_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "gripper_group");
    }

    auto result = std::make_shared<thor_server::action::JointTask::Result>();

    arm_joint_goal_ = {goal_handle->get_goal()->joint1_deg, goal_handle->get_goal()->joint2_deg, goal_handle->get_goal()->joint3_deg, goal_handle->get_goal()->joint4_deg, goal_handle->get_goal()->joint5_deg, goal_handle->get_goal()->joint6_deg};

    // convert joint angles to radians
    for(int i = 0; i < arm_joint_goal_.size(); i++){
      arm_joint_goal_[i] = arm_joint_goal_[i] * M_PI / 180.0;
    }

    double gripper_joint_rad = goal_handle->get_goal()->gripper_joint_deg * M_PI / 180.0;

    gripper_joint_goal_ = {gripper_joint_rad, gripper_joint_rad, -gripper_joint_rad, gripper_joint_rad, gripper_joint_rad, -gripper_joint_rad};

    arm_move_group_->setStartState(*arm_move_group_->getCurrentState());
    gripper_move_group_->setStartState(*gripper_move_group_->getCurrentState());

    bool arm_within_bounds = arm_move_group_->setJointValueTarget(arm_joint_goal_);
    bool gripper_within_bounds = gripper_move_group_->setJointValueTarget(gripper_joint_goal_);

    if (!arm_within_bounds || !gripper_within_bounds){
      if (!arm_within_bounds)
        RCLCPP_ERROR(get_logger(), "Arm goal is out of bounds");
      if (!gripper_within_bounds)
        RCLCPP_ERROR(get_logger(), "Gripper goal is out of bounds");
    }

    moveit::planning_interface::MoveGroupInterface::Plan arm_plan, gripper_plan;

    bool arm_plan_success = (arm_move_group_->plan(arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    bool gripper_plan_success = (gripper_move_group_->plan(gripper_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    arm_move_group_->setMaxVelocityScalingFactor(1.0);
    arm_move_group_->setMaxAccelerationScalingFactor(1.0);

    gripper_move_group_->setMaxVelocityScalingFactor(1.0);
    gripper_move_group_->setMaxAccelerationScalingFactor(1.0);

    if (arm_plan_success || gripper_plan_success){
      if(arm_plan_success)
        arm_move_group_->execute(arm_plan);
      else
        RCLCPP_ERROR(get_logger(), "Failed to plan arm movement");

      if(gripper_plan_success)
        gripper_move_group_->execute(gripper_plan);
      else
        RCLCPP_ERROR(get_logger(), "Failed to plan gripper movement");
    }
    
    else{
      RCLCPP_ERROR(get_logger(), "Failed to plan arm movement");
      return;
    }

    result->success = true;
    goal_handle->succeed(result);
  }

  // PoseTask callbacks
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
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{ std::bind(&TaskServer::poseTaskExecute, this, _1), goal_handle }.detach();
  }

  void poseTaskExecute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<thor_server::action::PoseTask>> goal_handle){
    RCLCPP_INFO(get_logger(), "Executing goal");
    if(!arm_move_group_){
      arm_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm_group");
    }

    auto result = std::make_shared<thor_server::action::PoseTask::Result>();

    arm_move_group_->setStartState(*arm_move_group_->getCurrentState());

    // Print current end effector name
    RCLCPP_INFO(get_logger(), "End effector link: %s", arm_move_group_->getEndEffectorLink().c_str());

    // Print current position and orientation
    RCLCPP_INFO(get_logger(), "Current position: %f, %f, %f", arm_move_group_->getCurrentPose().pose.position.x, arm_move_group_->getCurrentPose().pose.position.y, arm_move_group_->getCurrentPose().pose.position.z);
    RCLCPP_INFO(get_logger(), "Current orientation: %f, %f, %f", arm_move_group_->getCurrentRPY().at(0), arm_move_group_->getCurrentRPY().at(1), arm_move_group_->getCurrentRPY().at(2));
    
    // Ensure the reference frame is correct
    arm_move_group_->setPoseReferenceFrame("world");

    // Create a target pose
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = goal_handle->get_goal()->x;
    target_pose.position.y = goal_handle->get_goal()->y;
    target_pose.position.z = goal_handle->get_goal()->z;

    // Converr RPY to quaternion to set orientation correctly
    tf2::Quaternion q;
    q.setRPY(goal_handle->get_goal()->roll, goal_handle->get_goal()->pitch, goal_handle->get_goal()->yaw);
    target_pose.orientation.x = q.x();
    target_pose.orientation.y = q.y();
    target_pose.orientation.z = q.z();
    target_pose.orientation.w = q.w();


    arm_move_group_->setStartStateToCurrentState();
    // Establish the complete target pose
    arm_move_group_->setPoseTarget(target_pose);

    // Adjust speed and acceleration scaling (before planning)
    arm_move_group_->setMaxVelocityScalingFactor(1.0);
    arm_move_group_->setMaxAccelerationScalingFactor(1.0);

    // Favor similar final positions to the current position
    arm_move_group_->setGoalPositionTolerance(0.01);
    arm_move_group_->setGoalOrientationTolerance(0.01);

    // Plan the movement
    moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
    bool arm_plan_success = (arm_move_group_->plan(arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (arm_plan_success)
    {
        arm_move_group_->execute(arm_plan);
    }
      else{
        RCLCPP_ERROR(get_logger(), "Failed to plan arm movement");
        result->success = false;
        goal_handle->succeed(result);
        return;
      }

      result->success = true;
      goal_handle->succeed(result);
    }
  };
} // namespace thor_server

RCLCPP_COMPONENTS_REGISTER_NODE(thor_server::TaskServer) 