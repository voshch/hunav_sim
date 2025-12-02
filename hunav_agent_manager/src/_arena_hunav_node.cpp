#include "hunav_agent_manager/_arena_hunav_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace hunav
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  /**
   * @brief Construct a new ArenaHuNavNode object
   *
   */
  ArenaHuNavNode::ArenaHuNavNode() : BTnode()
  {
    RCLCPP_INFO(get_logger(), "ArenaHuNavNode initialized");
    bt_dir_base_ = ament_index_cpp::get_package_share_directory("arena_simulation_setup") + "/configs/hunav/behavior_trees";

    // Declare 1k dummy global goals for agents to set them later
    global_goals_.clear();
    for (int i = 0; i < 1000; i++)
    {
      geometry_msgs::msg::Point dummy_point;
      dummy_point.x = 15.0;
      dummy_point.y = 10.0;
      dummy_point.z = 0.0;
      global_goals_[i] = dummy_point;
    }
    btfunc_.setGlobalGoals(global_goals_);

    clear_srv_ = this->create_service<std_srvs::srv::Trigger>("clear_agents", std::bind(&ArenaHuNavNode::clearAgentsService, this, _1, _2));
  }

  void ArenaHuNavNode::clearAgentsService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                          std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    btfunc_.clear();
    initialized_ = false;
    response->success = true;
  }

  /**
   * @brief Destroy the ArenaHuNavNode object
   *
   */
  ArenaHuNavNode::~ArenaHuNavNode() = default;

}; // namespace hunav

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto bt_node = std::make_shared<hunav::ArenaHuNavNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(bt_node);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
