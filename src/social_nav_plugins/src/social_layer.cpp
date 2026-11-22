#include "social_nav_plugins/social_layer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace social_nav_plugins
{

void SocialLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("SocialLayer: failed to lock node");
  }
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("people_topic", rclcpp::ParameterValue(std::string("/people")));
  declareParameter("front_gain", rclcpp::ParameterValue(shape.front_gain));
  declareParameter("min_sigma", rclcpp::ParameterValue(shape.min_sigma));
  declareParameter("side_ratio", rclcpp::ParameterValue(shape.side_ratio));
  declareParameter("rear_ratio", rclcpp::ParameterValue(shape.rear_ratio));
  declareParameter("still_speed", rclcpp::ParameterValue(shape.still_speed));
  declareParameter("sigma_growth", rclcpp::ParameterValue(shape.growth));
  declareParameter("decay_time", rclcpp::ParameterValue(shape.decay_time));
  declareParameter("prediction_horizon", rclcpp::ParameterValue(prediction_horizon));
  declareParameter("prediction_step", rclcpp::ParameterValue(prediction_step));
  declareParameter("lethal_radius", rclcpp::ParameterValue(lethal_radius));
  declareParameter("lethal_on_prediction", rclcpp::ParameterValue(lethal_on_prediction));
  declareParameter("max_cost", rclcpp::ParameterValue(max_cost));
  declareParameter("cutoff", rclcpp::ParameterValue(cutoff));
  declareParameter("max_age", rclcpp::ParameterValue(max_age));
  declareParameter("robot_clear_radius", rclcpp::ParameterValue(robot_clear_radius));

  std::string topic;
  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".people_topic", topic);
  node->get_parameter(name_ + ".front_gain", shape.front_gain);
  node->get_parameter(name_ + ".min_sigma", shape.min_sigma);
  node->get_parameter(name_ + ".side_ratio", shape.side_ratio);
  node->get_parameter(name_ + ".rear_ratio", shape.rear_ratio);
  node->get_parameter(name_ + ".still_speed", shape.still_speed);
  node->get_parameter(name_ + ".sigma_growth", shape.growth);
  node->get_parameter(name_ + ".decay_time", shape.decay_time);
  node->get_parameter(name_ + ".prediction_horizon", prediction_horizon);
  node->get_parameter(name_ + ".prediction_step", prediction_step);
  node->get_parameter(name_ + ".lethal_radius", lethal_radius);
  node->get_parameter(name_ + ".lethal_on_prediction", lethal_on_prediction);
  node->get_parameter(name_ + ".max_cost", max_cost);
  node->get_parameter(name_ + ".cutoff", cutoff);
  node->get_parameter(name_ + ".max_age", max_age);
  node->get_parameter(name_ + ".robot_clear_radius", robot_clear_radius);
  prediction_step = std::max(prediction_step, 0.05);
  max_cost = std::clamp(max_cost, 1.0, 252.0);
  cutoff = std::clamp(cutoff, 1e-4, 0.5);

  clock_ = node->get_clock();
  sub_ = node->create_subscription<social_nav_msgs::msg::People>(
    topic, rclcpp::QoS(5),
    [this](const social_nav_msgs::msg::People::SharedPtr msg) {buffer_.store(msg);});
  current_ = true;
  RCLCPP_INFO(
    node->get_logger(), "SocialLayer '%s': people from %s, horizon %.1fs, lethal %.2fm",
    name_.c_str(), topic.c_str(), prediction_horizon, lethal_radius);
}

SocialLayer::Box SocialLayer::footprintOf(const Person & person) const
{
  // Widest Gaussian is the last prediction (sigmas grow with look-ahead time).
  const auto sig = shape.sigmasAt(person.speed(), prediction_horizon);
  const double r = std::max(SocialShape::support(sig, cutoff), lethal_radius);
  const double ex = person.vx * prediction_horizon, ey = person.vy * prediction_horizon;
  return {std::min(person.x, person.x + ex) - r, std::min(person.y, person.y + ey) - r,
    std::max(person.x, person.x + ex) + r, std::max(person.y, person.y + ey) + r};
}

void SocialLayer::updateBounds(
  double robot_x, double robot_y, double, double * min_x, double * min_y, double * max_x,
  double * max_y)
{
  if (!enabled_) {
    return;
  }
  robot_x_ = robot_x;
  robot_y_ = robot_y;
  std::string error;
  people_ = buffer_.snapshot(
    *tf_, layered_costmap_->getGlobalFrameID(), clock_->now(), max_age, &error);
  if (!error.empty()) {
    auto node = node_.lock();
    if (node) {
      RCLCPP_WARN_THROTTLE(node->get_logger(), *clock_, 5000, "SocialLayer: %s", error.c_str());
    }
  }
  // Include last cycle's area so costs of people who moved on are cleared.
  Box box{std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
    std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest()};
  for (const auto & person : people_) {
    const Box b = footprintOf(person);
    box = {std::min(box.min_x, b.min_x), std::min(box.min_y, b.min_y),
      std::max(box.max_x, b.max_x), std::max(box.max_y, b.max_y)};
  }
  for (const Box * b : {&box, had_people_ ? &last_box_ : nullptr}) {
    if (b && b->min_x <= b->max_x) {
      *min_x = std::min(*min_x, b->min_x);
      *min_y = std::min(*min_y, b->min_y);
      *max_x = std::max(*max_x, b->max_x);
      *max_y = std::max(*max_y, b->max_y);
    }
  }
  had_people_ = !people_.empty();
  if (had_people_) {
    last_box_ = box;
  }
}

unsigned char SocialLayer::costAt(const Person & person, double wx, double wy) const
{
  using nav2_costmap_2d::LETHAL_OBSTACLE;
  const double speed = person.speed();
  const double heading = person.heading();
  const double c = std::cos(heading), s = std::sin(heading);
  const int steps = std::max(0, static_cast<int>(std::floor(prediction_horizon / prediction_step + 1e-9)));
  double best = 0.0;
  for (int k = 0; k <= steps; ++k) {
    const double t = k * prediction_step;
    const double dx = wx - (person.x + person.vx * t);
    const double dy = wy - (person.y + person.vy * t);
    if ((k == 0 || lethal_on_prediction) && std::hypot(dx, dy) < lethal_radius) {
      return LETHAL_OBSTACLE;
    }
    best = std::max(best, shape.weightAt(t) * SocialShape::value(dx, dy, c, s, shape.sigmasAt(speed, t)));
  }
  if (best < cutoff) {
    return 0;
  }
  return static_cast<unsigned char>(std::lround(best * max_cost));
}

void SocialLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_ || people_.empty()) {
    return;
  }
  for (const auto & person : people_) {
    const Box b = footprintOf(person);
    int i0, j0, i1, j1;
    master_grid.worldToMapEnforceBounds(b.min_x, b.min_y, i0, j0);
    master_grid.worldToMapEnforceBounds(b.max_x, b.max_y, i1, j1);
    i0 = std::max(i0, min_i);
    j0 = std::max(j0, min_j);
    i1 = std::min(i1, max_i - 1);
    j1 = std::min(j1, max_j - 1);
    for (int j = j0; j <= j1; ++j) {
      for (int i = i0; i <= i1; ++i) {
        double wx, wy;
        master_grid.mapToWorld(i, j, wx, wy);
        unsigned char cost = costAt(person, wx, wy);
        if (cost == 0) {
          continue;
        }
        if (cost == nav2_costmap_2d::LETHAL_OBSTACLE &&
          std::hypot(wx - robot_x_, wy - robot_y_) < robot_clear_radius)
        {
          cost = static_cast<unsigned char>(max_cost);
        }
        const unsigned char old = master_grid.getCost(i, j);
        if (old == nav2_costmap_2d::NO_INFORMATION || cost > old) {
          master_grid.setCost(i, j, cost);
        }
      }
    }
  }
}

}  // namespace social_nav_plugins

PLUGINLIB_EXPORT_CLASS(social_nav_plugins::SocialLayer, nav2_costmap_2d::Layer)
