#include "bot_agent_control/bt_node.hpp"
#include "bot_agent_control/agent_say_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto controller = std::make_shared<bot::BTnode>();
    auto speech = std::make_shared<bot::AgentSayNode>();
    bot::AgentSayNode::setInstance(speech);
    {
      rclcpp::executors::MultiThreadedExecutor executor;
      executor.add_node(controller);
      executor.add_node(speech);
      executor.spin();
      executor.remove_node(controller);
      executor.remove_node(speech);
    }
    bot::AgentSayNode::setInstance(nullptr);
    speech.reset();
    controller.reset();
  } catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("bot_agent_control"), "%s", error.what());
    bot::AgentSayNode::setInstance(nullptr);
    exit_code = 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return exit_code;
}
