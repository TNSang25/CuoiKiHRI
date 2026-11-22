#include "human_agents/bt_node.hpp"
#include "human_agents/agent_say_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto controller = std::make_shared<crowd::BTnode>();
    auto speech = std::make_shared<crowd::AgentSayNode>();
    crowd::AgentSayNode::setInstance(speech);
    {
      rclcpp::executors::MultiThreadedExecutor executor;
      executor.add_node(controller);
      executor.add_node(speech);
      executor.spin();
      executor.remove_node(controller);
      executor.remove_node(speech);
    }
    crowd::AgentSayNode::setInstance(nullptr);
    speech.reset();
    controller.reset();
  } catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("human_agents"), "%s", error.what());
    crowd::AgentSayNode::setInstance(nullptr);
    exit_code = 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return exit_code;
}
