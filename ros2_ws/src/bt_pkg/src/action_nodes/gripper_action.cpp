#include "bt_pkg/action_nodes.hpp"

namespace bt_pkg {

GripperAction::GripperAction(const std::string& name,
                             const BT::NodeConfig& config,
                             const BT::RosNodeParams& params,
                             const GripperParams& gripper)
  : BT::RosActionNode<GripperCommand>(name, config, params)
  , gripper_(gripper)
{}

bool GripperAction::setGoal(Goal& goal)
{
  std::string cmd = "close";
  getInput("command", cmd);

  if (cmd == "close") {
    goal.command.position   = gripper_.close_position;
    goal.command.max_effort = gripper_.close_effort;
    RCLCPP_INFO(logger(), "GripperAction: CLOSE (contact detection active)");
  } else {
    goal.command.position   = gripper_.open_position;
    goal.command.max_effort = gripper_.open_effort;
    RCLCPP_INFO(logger(), "GripperAction: OPEN");
  }
  return true;
}

BT::NodeStatus GripperAction::onResultReceived(const WrappedResult& wr)
{
  std::string cmd = "close";
  getInput("command", cmd);

  const auto& res = *wr.result;

  if (cmd == "open") {
    // Open always succeeds regardless of result fields
    RCLCPP_INFO(logger(), "GripperAction OPEN: SUCCESS (pos=%.3frad)", res.position);
    return BT::NodeStatus::SUCCESS;
  }

  // CLOSE: GripperActionController (allow_stalling: true) reports stalled=true
  // when the finger stopped short of the goal — i.e. it closed on something.
  if (res.stalled) {
    RCLCPP_INFO(logger(),
      "GripperAction CLOSE: SUCCESS — contact at pos=%.3frad (object grasped)", res.position);
    return BT::NodeStatus::SUCCESS;
  }

  if (!gripper_.require_contact) {
    RCLCPP_WARN(logger(),
      "GripperAction CLOSE: fully closed, no contact — accepted anyway "
      "(gripper_require_contact=false)");
    return BT::NodeStatus::SUCCESS;
  }

  // Fingers fully closed (res.reached_goal=true) means nothing was there
  RCLCPP_WARN(logger(),
    "GripperAction CLOSE: FAILURE — fingers fully closed, no contact (missed)");
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus GripperAction::onFailure(BT::ActionNodeErrorCode err)
{
  RCLCPP_ERROR(logger(), "GripperAction action error: %s", BT::toStr(err));
  return BT::NodeStatus::FAILURE;
}

}  // namespace bt_pkg
