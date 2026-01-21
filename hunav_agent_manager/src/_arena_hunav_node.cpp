#include "hunav_agent_manager/_arena_hunav_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/detail/float32_multi_array__struct.hpp>

namespace hunav {
using std::placeholders::_1;
using std::placeholders::_2;

/**
 * @brief Construct a new ArenaHuNavNode object
 *
 */
ArenaHuNavNode::ArenaHuNavNode() : BTnode() {
  RCLCPP_INFO(get_logger(), "ArenaHuNavNode initialized");
  bt_dir_base_ =
      ament_index_cpp::get_package_share_directory("arena_simulation_setup") +
      "/configs/hunav/behavior_trees";

  // Declare 1k dummy global goals for agents to set them later
  RCLCPP_INFO(get_logger(), "Setting global goals...");
  global_goals_.clear();
  for (int i = 0; i < 1000; i++) {
    geometry_msgs::msg::Point dummy_point;
    dummy_point.x = 15.0;
    dummy_point.y = 10.0;
    dummy_point.z = 0.0;
    global_goals_[i] = dummy_point;
  }
  btfunc_.setGlobalGoals(global_goals_);

  clear_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "clear_agents",
      std::bind(&ArenaHuNavNode::clearAgentsService, this, _1, _2));

  set_velocity_field_srv_ =
      this->create_service<hunav_msgs::srv::SetVelocityField>(
          "set_velocity_field",
          std::bind(&ArenaHuNavNode::setVelocityFieldService, this,
                    std::placeholders::_1, std::placeholders::_2));

  set_arena_world_size_srv_ =
      this->create_service<hunav_msgs::srv::SetArenaWorldSize>(
          "set_arena_world_size",
          std::bind(&ArenaHuNavNode::setArenaWorldSizeService, this,
                    std::placeholders::_1, std::placeholders::_2));

}

void ArenaHuNavNode::clearAgentsService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  btfunc_.clear();
  initialized_ = false;
  response->success = true;
}

void ArenaHuNavNode::setArenaWorldSize(
    const std_msgs::msg::Float32MultiArray &msg) {
  std::lock_guard<std::mutex> lock(arena_world_size_mutex);
  if (msg.layout.dim.size() != 1) {
    throw std::runtime_error("Arena World Size must be 1D: (2,)");
  }

  arena_world_size.first = msg.data[0];
  arena_world_size.second = msg.data[1];

  btfunc_.setArenaWorldSize(arena_world_size);
}

void ArenaHuNavNode::setVelocityField(
    const std_msgs::msg::Float32MultiArray &msg) {
  std::lock_guard<std::mutex> lock(velocity_field_mutex);

  if (msg.layout.dim.size() != 4) {
    throw std::runtime_error("Velocity field must be 4D: (groups, 64, 64, 2)");
  }

  const size_t num_groups = msg.layout.dim[0].size;
  const size_t H = msg.layout.dim[1].size;
  const size_t W = msg.layout.dim[2].size;
  const size_t C = msg.layout.dim[3].size;

  if (H != VF_H || W != VF_W || C != VF_C) {
    throw std::runtime_error(
        "Velocity field dimensions must be (groups, 64, 64, 2)");
  }

  constexpr size_t single_field_size = VF_H * VF_W * VF_C;
  const size_t expected_size = num_groups * single_field_size;

  if (msg.data.size() != expected_size) {
    throw std::runtime_error("Velocity field data size mismatch. Expected " +
                             std::to_string(expected_size) + ", got " +
                             std::to_string(msg.data.size()));
  }

  VelocityField velocity_field;
  velocity_field.resize(num_groups);

  size_t idx = 0;
  for (size_t g = 0; g < num_groups; g++) {
    for (size_t i = 0; i < VF_H; i++) {
      for (size_t j = 0; j < VF_W; j++) {
        velocity_field[g][i][j][0] = msg.data[idx++];
        velocity_field[g][i][j][1] = msg.data[idx++];
      }
    }
  }

  btfunc_.setVelocityField(velocity_field);
}

void ArenaHuNavNode::setVelocityFieldService(
    const std::shared_ptr<hunav_msgs::srv::SetVelocityField::Request> request,
    std::shared_ptr<hunav_msgs::srv::SetVelocityField::Response> response) {
  try {
    setVelocityField(request->velocity_field);
    response->success = true;
    response->message = "Velocity field updated successfully";
  } catch (const std::exception &e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
  }
}

void ArenaHuNavNode::setArenaWorldSizeService(
    const std::shared_ptr<hunav_msgs::srv::SetArenaWorldSize::Request> request,
    std::shared_ptr<hunav_msgs::srv::SetArenaWorldSize::Response> response) {
  try {
    setArenaWorldSize(request->arena_world_size);
    response->success = true;
    response->message = "Arena world size updated successfully";
  } catch (const std::exception &e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
  }
}

/**
 * @brief Destroy the ArenaHuNavNode object
 *
 */
ArenaHuNavNode::~ArenaHuNavNode() = default;

}; // namespace hunav

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto bt_node = std::make_shared<hunav::ArenaHuNavNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(bt_node);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
