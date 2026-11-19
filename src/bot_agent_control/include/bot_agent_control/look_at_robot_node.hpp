#ifndef BOT_LOOK_AT_ROBOT_NODE_HPP_
#define BOT_LOOK_AT_ROBOT_NODE_HPP_

#include "behaviortree_cpp/behavior_tree.h"
#include "bot_agent_control/agent_manager.hpp"

namespace bot
{

    class LookAtRobotNode : public BT::StatefulActionNode
    {
    public:
        LookAtRobotNode(const std::string &name, const BT::NodeConfig &config)
            : BT::StatefulActionNode(name, config),
              agent_manager_(nullptr)
        {
        }

        LookAtRobotNode() = delete;

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<int>("agent_id"),
                // How close (in radians) the agent must be pointed at the robot to be “done”
                BT::InputPort<double>(
                    "yaw_tolerance", 0.01,
                    "Angle tolerance [rad] to consider ‘aligned’ with the robot")};
        }

        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

    private:
        int agent_id_;
        double yaw_tolerance_;
        AgentManager *agent_manager_;
    };

} // namespace bot

#endif // BOT_LOOK_AT_ROBOT_NODE_HPP_
