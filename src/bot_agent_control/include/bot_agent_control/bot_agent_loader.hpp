
#include "bot_interfaces/msg/agents.hpp"
#include "bot_interfaces/srv/get_parameters.hpp"
#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace bot {

class BotLoader : public rclcpp::Node {
public:
  /**
   * @brief Construct a new Bot Loader object
   *
   */
  BotLoader();
  /**
   * @brief Destroy the Bot Loader object
   *
   */
  ~BotLoader();

private:
  /**
   * @brief Service callback to provide parameters to other nodes
   * 
   * @param request Empty request
   * @param response Parameters response
   */
  void getParametersService(const std::shared_ptr<bot_interfaces::srv::GetParameters::Request> request,
                           std::shared_ptr<bot_interfaces::srv::GetParameters::Response> response);

  // Service server
  rclcpp::Service<bot_interfaces::srv::GetParameters>::SharedPtr get_parameters_srv_;
};
} // namespace bot