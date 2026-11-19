#ifndef BOT_LOOK_AT_AGENT_NODE_HPP_
#define BOT_LOOK_AT_AGENT_NODE_HPP_

#include "behaviortree_cpp/behavior_tree.h"
#include "bot_agent_control/agent_manager.hpp"

namespace bot
{

    class LookAtAgentNode : public BT::StatefulActionNode
    {
    public:
        LookAtAgentNode(const std::string &name, const BT::NodeConfig &config)
            : BT::StatefulActionNode(name, config),
              agent_manager_(nullptr)
        {
        }

        LookAtAgentNode() = delete;

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<int>("observer_id"),
                BT::InputPort<int>("target_id"),
                // How close (in radians) the observer must be pointing at the target to be "done"
                BT::InputPort<double>("yaw_tolerance", 0.01,
                                      "Angle tolerance [rad] to consider 'aligned'")};
        }

        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

    private:
        int observer_id_;
        int target_id_;
        double yaw_tolerance_;
        AgentManager *agent_manager_;
    };

} // namespace bot

#endif // BOT_LOOK_AT_AGENT_NODE_HPP_
