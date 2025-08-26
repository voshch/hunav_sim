#include "hunav_agent_manager/bt_node.hpp"
#include "hunav_agent_manager/agent_say_node.hpp"
#include <rclcpp/rclcpp.hpp>

namespace hunav
{

class ArenaHuNavNode : public BTnode
{
public:
  /**
   * @brief Construct a new ArenaHuNavNode object
   * 
   */
  ArenaHuNavNode() : BTnode()
  {
    RCLCPP_INFO(get_logger(), "ArenaHuNavNode initialized");
    bt_dir_base_ = ament_index_cpp::get_package_share_directory("arena_simulation_setup") + "/configs/hunav/behavior_trees";
    // Add any additional initialization logic here
  }

  /**
   * @brief Destroy the ArenaHuNavNode object
   * 
   */
  ~ArenaHuNavNode() = default;
};

} // namespace hunav

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  
  auto bt_node = std::make_shared<hunav::ArenaHuNavNode>();
  auto agent_say_node = std::make_shared<hunav::AgentSayNode>();
  hunav::AgentSayNode::setInstance(agent_say_node);
  
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(bt_node);
  executor.add_node(agent_say_node);
  
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}
