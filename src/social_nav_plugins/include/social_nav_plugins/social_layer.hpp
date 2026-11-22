// Nav2 costmap layer: asymmetric Gaussian personal space swept along each person's
// constant-velocity prediction, with a lethal core around the person's body.
#ifndef SOCIAL_NAV_PLUGINS__SOCIAL_LAYER_HPP_
#define SOCIAL_NAV_PLUGINS__SOCIAL_LAYER_HPP_

#include <string>
#include <vector>

#include "social_nav_msgs/msg/people.hpp"
#include "social_nav_plugins/people_model.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "rclcpp/rclcpp.hpp"

namespace social_nav_plugins
{

class SocialLayer : public nav2_costmap_2d::Layer
{
public:
  void onInitialize() override;
  void reset() override {}
  bool isClearable() override {return false;}
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j) override;

  /// Cost in [0, 254] that one person adds at world point (wx, wy); exposed for tests.
  unsigned char costAt(const Person & person, double wx, double wy) const;

  // Parameters (public so tests can configure the layer without a node).
  SocialShape shape;
  double prediction_horizon{2.0};   // s
  double prediction_step{0.25};     // s
  double lethal_radius{0.45};       // m around the current position
  bool lethal_on_prediction{false};
  double max_cost{252.0};           // peak of the Gaussian (below inscribed = 253)
  double cutoff{0.02};              // ignore Gaussian values below this
  double max_age{1.0};              // s; older /people messages are ignored
  // Cells this close to the robot are never written as lethal: a person who walks into the
  // robot must not put the robot "inside" an obstacle (planner/controller would then fail);
  // the high Gaussian cost there still drives the robot away.
  double robot_clear_radius{0.45};

private:
  struct Box {double min_x, min_y, max_x, max_y;};
  Box footprintOf(const Person & person) const;

  PeopleBuffer buffer_;
  rclcpp::Subscription<social_nav_msgs::msg::People>::SharedPtr sub_;
  std::vector<Person> people_;
  bool had_people_{false};
  double robot_x_{0.0}, robot_y_{0.0};
  Box last_box_{0, 0, 0, 0};
  rclcpp::Clock::SharedPtr clock_;
};

}  // namespace social_nav_plugins

#endif  // SOCIAL_NAV_PLUGINS__SOCIAL_LAYER_HPP_
