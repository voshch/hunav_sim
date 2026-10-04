#include "hunav_agent_manager/follow_velocity_field_node.hpp"
#include "hunav_agent_manager/agent_manager.hpp"
#include "hunav_agent_manager/bt_functions.hpp"
#include <cmath>

namespace hunav {

BT::NodeStatus FollowVelocityFieldNode::onStart() {
  // 1) Read required inputs
  if (!getInput<int>("agent_id", agent_id_))
    throw BT::RuntimeError("FollowVelocityFieldNode: missing [agent_id]");

  if (!getInput<int>("velocity_field_group_id", velocity_field_group_id_))
    throw BT::RuntimeError(
        "FollowVelocityFieldNode: missing [velocity_field_group_id]");

  if (g_btfunctions == nullptr) {
    throw BT::RuntimeError(
        "FollowVelocityFieldNode: BTfunctions singleton not initialized");
  }

  if (!getInput<double>("time_step", dt_))
    throw BT::RuntimeError("FollowVelocityFieldNode: missing [time_step]");
  if (!getInput<double>("tolerance", tolerance_))
    throw BT::RuntimeError("FollowVelocityFieldNode: missing [tolerance]");

  // 3) Fetch the global AgentManager pointer
  if (agent_manager_ == nullptr) {
    agent_manager_ = hunav::g_agent_manager;
    if (agent_manager_ == nullptr)
      throw BT::RuntimeError(
          "FollowVelocityFieldNode: global AgentManager pointer not set");
  }

  // 4) Snapshot any existing goals, so we can restore later
  original_goals_ = agent_manager_->getAgentGoals(agent_id_);

  // 5) Recompute goal
  recomputeGoal();

  return BT::NodeStatus::RUNNING;
}

void FollowVelocityFieldNode::recomputeGoal() {
  // 1) Compute current velocity
  utils::Vector2d pos = agent_manager_->getAgentPosition(agent_id_);
  auto [vx, vy] = g_btfunctions->getVelocityAt(velocity_field_group_id_,
                                               pos.getX(), pos.getY());

  // 2) Recompute the goal position according to velocity
  double x = pos.getX() + vx * dt_;
  double y = pos.getY() + vy * dt_;
  auto goal_pos = utils::Vector2d(x, y);

  // Scale the goal vector to make sure if the velocity is not zero,
  // the goal isn't within tolerance_
  if (vx != 0 || vy != 0) {
    auto direction = goal_pos - pos;

    // Ensure the distance between goal and current position is at least
    // 2 x tolerance_
    if (direction.norm() < 2 * tolerance_) {
      double direction_scale = 2 * tolerance_ / direction.norm();
      goal_pos = pos + direction * direction_scale;
    }
  }

  // 3) Set new goal for the agent
  sfm::Goal goal;
  goal.center.set(goal_pos.getX(), goal_pos.getY());
  goal.radius = 0.1;
  agent_manager_->clearAndSetAgentGoal(agent_id_, goal);
}

BT::NodeStatus FollowVelocityFieldNode::onRunning() {
  // 1) Re‐read dt in case it’s dynamic
  auto dt_msg = getInput<double>("time_step");
  if (!dt_msg)
    throw BT::RuntimeError(
        "FollowVelocityFieldNode: missing input [time_step] during onRunning",
        dt_msg.error());
  dt_ = dt_msg.value();

  // 2) Recompute the goal
  recomputeGoal();

  // 3) If within tolerance_, remove the temporary goal and succeed
  if (agent_manager_->goalReached(agent_id_)) //(distance <= tolerance_)
  {
    // Restore the original goals exactly as they were
    agent_manager_->restoreAgentGoals(agent_id_, original_goals_);
    return BT::NodeStatus::SUCCESS;
  }

  // 4) Otherwise, we haven’t reached it yet → keep moving
  agent_manager_->updatePosition(agent_id_, dt_);
  return BT::NodeStatus::RUNNING;
}

void FollowVelocityFieldNode::onHalted() {
  // If the tree is halted mid‐goal, restore original goals so we don’t leave
  // the agent stuck on a half‐completed temporary goal.
  agent_manager_->clearAndSetAgentGoals(agent_id_, original_goals_);
}

} // namespace hunav
