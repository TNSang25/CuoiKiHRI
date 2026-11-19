#ifndef BOT_STOP_AND_WAIT_TIMER_ACTION_NODE_HPP_
#define BOT_STOP_AND_WAIT_TIMER_ACTION_NODE_HPP_

#include "behaviortree_cpp/behavior_tree.h"
#include "bot_agent_control/agent_manager.hpp"
#include <chrono>
#include <string>

namespace bot
{

  class StopAndWaitTimerActionNode : public BT::StatefulActionNode
  {
  public:
    StopAndWaitTimerActionNode(const std::string &name, const BT::NodeConfig &config)
        : BT::StatefulActionNode(name, config),
          agent_manager_(nullptr)
    {
    }

    StopAndWaitTimerActionNode() = delete;

    static BT::PortsList providedPorts()
    {
      return {BT::InputPort<int>("agent_id"),
              BT::InputPort<double>("wait_duration", 5.0, "Wait duration in seconds")};
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    int agent_id_;
    double wait_duration_;
    std::chrono::steady_clock::time_point start_time_;
    AgentManager *agent_manager_;
  };

} // namespace bot

#endif // BOT_STOP_AND_WAIT_TIMER_ACTION_NODE_HPP_
