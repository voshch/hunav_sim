#include "rclcpp/rclcpp.hpp"

#include "hunav_agent_manager/bt_node.hpp"

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

    protected:
        /**
         * @brief ROS service to clear all agents
         * @param request empty
         * @param response contains a boolean to indicate success or failure
         */
        void clearAgentsService(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    };
};

#endif // ARENA_HUNAV_NODE_HPP_