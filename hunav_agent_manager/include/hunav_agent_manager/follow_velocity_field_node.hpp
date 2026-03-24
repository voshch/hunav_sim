#ifndef HUNAV_FOLLOW_VELOCITY_FIELD_NODE_HPP_
#define HUNAV_FOLLOW_VELOCITY_FIELD_NODE_HPP_

#include "behaviortree_cpp/behavior_tree.h"
#include "hunav_agent_manager/agent_manager.hpp"
#include "hunav_agent_manager/bt_functions.hpp"
#include <list>

namespace hunav {
class FollowVelocityFieldNode : public BT::StatefulActionNode {
public:
  FollowVelocityFieldNode(const std::string &name, const BT::NodeConfig &config)
      : BT::StatefulActionNode(name, config), agent_manager_(nullptr) {}

  FollowVelocityFieldNode() = delete;

  static BT::PortsList providedPorts() {
    return {BT::InputPort<int>("agent_id"),
            BT::InputPort<int>("velocity_field_group_id"),
            BT::InputPort<double>("time_step"),
            BT::InputPort<double>("tolerance", 0.1,
                                  "Distance [m] to consider 'at goal'")};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
  void recomputeGoal();

private:
  int agent_id_;
  int velocity_field_group_id_;
  double dt_;
  double tolerance_;

  AgentManager *agent_manager_;
  std::list<sfm::Goal> original_goals_;
};

} // namespace hunav

#endif // HUNAV_FOLLOW_VELOCITY_FIELD_NODE_HPP_
