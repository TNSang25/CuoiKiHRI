#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <bot_interfaces/msg/people.hpp>

#include "bot_agent_control/bt_functions.hpp"

#include "bot_interfaces/msg/agent.hpp"
#include "bot_interfaces/msg/agents.hpp"
//#include "human_nav_msgs/srv/is_robot_visible.hpp"
#include "bot_interfaces/srv/compute_agent.hpp"
#include "bot_interfaces/srv/compute_agents.hpp"
#include "bot_interfaces/srv/move_agent.hpp"
#include "bot_interfaces/srv/reset_agents.hpp"
#include "bot_interfaces/srv/get_parameters.hpp"

#include "bot_agent_control/time_expired_condition.hpp"
#include "bot_agent_control/stop_and_wait_timer_action_node.hpp"
#include "bot_agent_control/conversation_formation_node.hpp"
#include "bot_agent_control/go_to_node.hpp"
#include "bot_agent_control/approach_agent_node.hpp"
#include "bot_agent_control/follow_agent_node.hpp"
#include "bot_agent_control/approach_robot_node.hpp"
#include "bot_agent_control/block_robot_node.hpp"
#include "bot_agent_control/avoid_robot_node.hpp"
#include "bot_agent_control/block_agent_node.hpp"
#include "bot_agent_control/time_delay_decorator.hpp"
#include "bot_agent_control/group_walk_node.hpp"
#include "bot_agent_control/is_anyone_speaking_node.hpp"
#include "bot_agent_control/is_speaking_node.hpp"
#include "bot_agent_control/is_anyone_looking_at_me_node.hpp"
#include "bot_agent_control/is_looking_at_me_node.hpp"
#include "bot_agent_control/look_at_point_node.hpp"
#include "bot_agent_control/look_at_agent_node.hpp"
#include "bot_agent_control/look_at_robot_node.hpp"

#include "behaviortree_cpp/loggers/groot2_publisher.h"


//#include "nav2_behavior_tree/plugins/condition/time_expired_condition.hpp"

#include <functional>

// Social Force Model
#include <bot_social_force/sfm.hpp>

namespace bot {

class BTnode : public rclcpp::Node {
public:
  /**
   * @brief Construct a new BTnode object
   *
   */
  BTnode();
  /**
   * @brief Destroy the BTnode object
   *
   */
  ~BTnode();

  // void registerBTNodes(BT::BehaviorTreeFactory &factory);
  void registerBTNodes();

protected:
  void initializeBehaviorTrees(const bot_interfaces::msg::Agents& agents);
  void initializeBehaviorTree(const bot_interfaces::msg::Agent& agent);
  BT::NodeStatus tree_tick(double dt);

  BT::NodeStatus tree_tick(int id);
  BT::NodeStatus tree_tick(int id, double dt);
  /**
   * @brief ROS service to compute the new state of the agents
   *
   * @param request contains the current state of the agents
   * @param response contains the updated state of the agents
   */
  void computeAgentsService(
      const std::shared_ptr<bot_interfaces::srv::ComputeAgents::Request> request,
      std::shared_ptr<bot_interfaces::srv::ComputeAgents::Response> response);

  /**
   * @brief ROS service to reset the state of the agents
   *
   * @param request contains the desired state of the agents
   * @param response contains a boolean to indicate success or failure
   */
  void resetAgentsService(
      const std::shared_ptr<bot_interfaces::srv::ResetAgents::Request> request,
      std::shared_ptr<bot_interfaces::srv::ResetAgents::Response> response);

  /**
   * @brief Get parameters from bot_agent_loader node via service call
   * @return true if parameters were successfully retrieved
   */
  bool getParametersFromLoader();

  /**
   * @brief ROS service to compute the new state of one agent
   *
   * @param request contains the id of the agent
   * @param response contains the updated state of the agent
   */
  void computeAgentService(
      const std::shared_ptr<bot_interfaces::srv::ComputeAgent::Request> request,
      std::shared_ptr<bot_interfaces::srv::ComputeAgent::Response> response);

  /**
   * @brief ROS service to compute the new state of one agent
   *
   * @param request contains the current state of the agents and the id of the
   * agent to move
   * @param response contains the updated state of the agent
   */
  void moveAgentService(
      const std::shared_ptr<bot_interfaces::srv::MoveAgent::Request> request,
      std::shared_ptr<bot_interfaces::srv::MoveAgent::Response> response);

  /**
   * @brief Publish an arrow marker in Rviz representing a force
   * @param index id of the marker
   * @param name name of the agent to be used as namespace
   * @param frame string of the tf frame
   * @param t time stamp
   * @param p point of the marker
   * @param color RGB color of the marker
   * @param force force to be represented
   * @param markers markerArray in which the arrow will be added
   * @return none
   */
  void publishForceMarker(unsigned index, std::string name, std::string frame,
                          rclcpp::Time t, geometry_msgs::msg::Point p,
                          const std_msgs::msg::ColorRGBA &color,
                          const utils::Vector2d &force,
                          visualization_msgs::msg::MarkerArray &markers);

  /**
   * @brief fill a ColorRGBA message
   * @param r value of the red componet [0-1]
   * @param g value of the green component [0-1]
   * @param b value of the blue component [0-1]
   * @param a transparency of the color [0-1]
   * @return a ROS ColorRGBA message
   */
  std_msgs::msg::ColorRGBA getColor(double r, double g, double b, double a);

  /**
   * @brief Publish the TF of the robot and agents
   *
   * @param t time stamp
   * @param robot robot state msg
   * @param msg agent states msg
   */
  void publish_agents_tf(rclcpp::Time t,
                         const bot_interfaces::msg::Agent::SharedPtr robot,
                         const bot_interfaces::msg::Agents::SharedPtr msg);

  /**
   * @brief Publish the agents states
   *
   * @param t time stamp
   * @param msg agent states msg
   */
  void publish_agent_states(rclcpp::Time t,
                            const bot_interfaces::msg::Agents::SharedPtr msg);

  /**
   * @brief publish the robot state
   *
   * @param t time stamp
   * @param msg agent msg representing the robot state
   */
  void publish_robot_state(rclcpp::Time t,
                           const bot_interfaces::msg::Agent::SharedPtr msg);

  /**
   * @brief Publish the agents in the standard people msg
   *
   * @param t time stamp
   * @param msg agent states msg
   */
  void publish_people(rclcpp::Time t,
                      const bot_interfaces::msg::Agents::SharedPtr msg);

  /**
   * @brief Publish the set of SFM forces in RViz
   * @param t time stamp
   * @param msg agents msg
   * @return none
   */
  void publish_agents_forces(rclcpp::Time t,
                             const bot_interfaces::msg::Agents::SharedPtr msg);

  /**
   * @brief Convert share directory path to source directory path
   * @param share_path Path to package share directory
   * @return Path to source directory, or share_path if conversion fails
   */

  sfm::Forces getAgentForces(int id) { return btfunc_.getAgentForces(id); };

  // Services provided
  rclcpp::Service<bot_interfaces::srv::ComputeAgents>::SharedPtr agents_srv_;
  rclcpp::Service<bot_interfaces::srv::ComputeAgent>::SharedPtr agent_srv_;
  rclcpp::Service<bot_interfaces::srv::MoveAgent>::SharedPtr move_agent_srv_;

  rclcpp::Service<bot_interfaces::srv::ResetAgents>::SharedPtr reset_srv_;

  // Service client to get parameters from bot_agent_loader
  rclcpp::Client<bot_interfaces::srv::GetParameters>::SharedPtr get_parameters_client_;

  bool initialized_;
  BTfunctions btfunc_;
  std::string pkg_shared_tree_dir_;
  std::string bt_dir_base_;
  // std::vector<BT::Tree> trees_;
  std::unordered_map<int, BT::Tree> trees_;

  rclcpp::Time prev_time_;

  // node parameters
  bool pub_tf_;
  bool pub_forces_;
  bool pub_people_;
  std::string map_name_;
  std::string simulator_name_;
  std::string yaml_base_name_;
  std::map<int, geometry_msgs::msg::Point> global_goals_;

  // Groot 2 publisher
  std::unique_ptr<BT::Groot2Publisher> publisher_;

  // BT::Tree tree_;
  // std::unique_ptr<BT::ParallelNode> root_;
  // std::unique_ptr<BT::SequenceNode> root_;
  BT::BehaviorTreeFactory factory_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      forces_publisher_;
  rclcpp::Publisher<bot_interfaces::msg::Agents>::SharedPtr human_state_publisher_;
  rclcpp::Publisher<bot_interfaces::msg::Agent>::SharedPtr robot_state_publisher_;
  rclcpp::Publisher<bot_interfaces::msg::People>::SharedPtr people_publisher_;
};

} // namespace bot
