#ifndef SIM_GOTO_NODE_HPP_
#define SIM_GOTO_NODE_HPP_

#include "behaviortree_cpp/behavior_tree.h"
#include "human_agents/agent_manager.hpp"
#include "human_agents/bt_functions.hpp"
#include <list>

namespace crowd
{

    class GoToNode : public BT::StatefulActionNode
    {
    public:
        GoToNode(const std::string &name, const BT::NodeConfig &config)
            : BT::StatefulActionNode(name, config),
              agent_manager_(nullptr)
        {
        }

        GoToNode() = delete;

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<int>("agent_id"),
                BT::InputPort<int>("goal_id", "Goal ID to look up from agent's ROS goals"),
                BT::InputPort<double>("time_step"),
                BT::InputPort<double>("tolerance", 0.1,
                                      "Distance [m] to consider 'at goal'")};
        }

        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

    private:
        int agent_id_;
        double target_x_;
        double target_y_;
        double dt_;
        double tolerance_;

        AgentManager *agent_manager_;
        std::list<sfm::Goal> original_goals_;
    };

} // namespace crowd

#endif // SIM_GOTO_NODE_HPP_
