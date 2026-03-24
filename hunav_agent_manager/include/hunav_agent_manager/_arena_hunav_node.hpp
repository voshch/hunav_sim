#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <array>
#include <mutex>
#include <stdexcept>
#include <vector>

#include "hunav_agent_manager/bt_node.hpp"

#include "hunav_msgs/srv/set_velocity_field.hpp"
#include "hunav_msgs/srv/set_arena_world_bounds.hpp"
#include "std_srvs/srv/trigger.hpp"

#ifndef ARENA_HUNAV_NODE_HPP_
#define ARENA_HUNAV_NODE_HPP_

namespace hunav
{

    class ArenaHuNavNode : public BTnode
    {
    public:
        ArenaHuNavNode();

        ~ArenaHuNavNode();

        /* =====================
         * Arena world size
         * We need to scale between velocity field size and Arena World
         * because velocity field is just a grid map of the world
         * ===================== */
        std::vector<float> arena_world_bounds; // x_min, y_min, x_max, y_max
        void setArenaWorldBounds(const std_msgs::msg::Float32MultiArray &mgs);

        /* =====================
         * Velocity field
         * ===================== */
        static constexpr size_t VF_H = 64;
        static constexpr size_t VF_W = 64;
        static constexpr size_t VF_C = 2;

        // Each pedestrians group has its own velocity field
        using GroupVelocityField =
            std::array<std::array<std::array<float, VF_C>, VF_W>, VF_H>;
        using VelocityField = std::vector<GroupVelocityField>;

        void setVelocityField(const std_msgs::msg::Float32MultiArray &mgs);

    protected:
        /**
         * @brief ROS service to clear all agents
         * @param request empty
         * @param response contains a boolean to indicate success or failure
         */
        void clearAgentsService(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response);
        rclcpp::Service<hunav_msgs::srv::SetVelocityField>::SharedPtr
            set_velocity_field_srv_;
        void setVelocityFieldService(
            const std::shared_ptr<hunav_msgs::srv::SetVelocityField::Request> request,
            std::shared_ptr<hunav_msgs::srv::SetVelocityField::Response> response);
        mutable std::mutex velocity_field_mutex;

        rclcpp::Service<hunav_msgs::srv::SetArenaWorldBounds>::SharedPtr
            set_arena_world_bounds_srv_;
        void setArenaWorldBoundsService(
            const std::shared_ptr<hunav_msgs::srv::SetArenaWorldBounds::Request> request,
            std::shared_ptr<hunav_msgs::srv::SetArenaWorldBounds::Response> response);
        mutable std::mutex arena_world_bounds_mutex;
    };
}; // namespace hunav

#endif // ARENA_HUNAV_NODE_HPP_
