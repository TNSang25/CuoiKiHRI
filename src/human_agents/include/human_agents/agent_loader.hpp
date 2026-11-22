
#include "social_nav_msgs/msg/agents.hpp"
#include "social_nav_msgs/srv/get_parameters.hpp"
#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace crowd {

class AgentLoader : public rclcpp::Node {
public:
  /**
   * @brief Construct a new Agent Loader object
   *
   */
  AgentLoader();
  /**
   * @brief Destroy the Agent Loader object
   *
   */
  ~AgentLoader();

private:
  /**
   * @brief Service callback to provide parameters to other nodes
   * 
   * @param request Empty request
   * @param response Parameters response
   */
  void getParametersService(const std::shared_ptr<social_nav_msgs::srv::GetParameters::Request> request,
                           std::shared_ptr<social_nav_msgs::srv::GetParameters::Response> response);

  // Service server
  rclcpp::Service<social_nav_msgs::srv::GetParameters>::SharedPtr get_parameters_srv_;
};
} // namespace crowd