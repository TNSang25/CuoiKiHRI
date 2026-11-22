#include "social_nav_plugins/social_speed_critic.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include <xtensor/xbuilder.hpp>
#include <xtensor/xmath.hpp>
#include <xtensor/xoperation.hpp>
#include <xtensor/xview.hpp>

#include "pluginlib/class_list_macros.hpp"

namespace mppi::critics
{

void SocialSpeedCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 10.0f);
  getParam(slow_distance_, "slow_distance", 0.6f);
  getParam(free_distance_, "free_distance", 2.0f);
  getParam(slow_speed_, "slow_speed", 0.15f);
  getParam(max_speed_, "max_speed", 0.8f);
  getParam(max_distance_, "max_distance", 6.0f);
  getParam(max_age_, "max_age", 1.0f);
  getParam(time_offset_, "time_offset", 1.0f);
  std::string topic;
  getParam(topic, "people_topic", std::string("/people"));
  free_distance_ = std::max(free_distance_, slow_distance_ + 0.01f);

  auto node = parent_.lock();
  sub_ = node->create_subscription<social_nav_msgs::msg::People>(
    topic, rclcpp::QoS(5),
    [this](const social_nav_msgs::msg::People::SharedPtr msg) {buffer_.store(msg);});
  RCLCPP_INFO(
    logger_, "SocialSpeedCritic: %.2f m/s within %.2f m of a person, free beyond %.2f m, weight %.1f",
    slow_speed_, slow_distance_, free_distance_, weight_);
}

void SocialSpeedCritic::score(CriticData & data)
{
  if (!enabled_) {
    return;
  }
  auto node = parent_.lock();
  std::string error;
  const auto people = buffer_.snapshot(
    *costmap_ros_->getTfBuffer(), costmap_ros_->getGlobalFrameID(), node->now(), max_age_, &error);
  if (!error.empty()) {
    RCLCPP_WARN_THROTTLE(logger_, *node->get_clock(), 5000, "SocialSpeedCritic: %s", error.c_str());
  }
  const float rx = static_cast<float>(data.state.pose.pose.position.x);
  const float ry = static_cast<float>(data.state.pose.pose.position.y);
  const auto & tx = data.trajectories.x;   // [batch, time_steps]
  const auto & ty = data.trajectories.y;
  const std::size_t steps = tx.shape(1);
  const auto t = xt::eval((xt::arange<float>(static_cast<float>(steps)) + time_offset_) * data.model_dt);

  // Penalise only the speed component *towards* each predicted person: slowing down in every
  // direction kept the robot lingering next to people who ignore it; moving away or sideways
  // quickly must stay cheap. Plain loops (see DynamicHumanCritic) keep this real-time.
  const std::size_t batch = tx.shape(0);
  const float * xs = tx.data();
  const float * ys = ty.data();
  const float * vs = data.state.vx.data();
  const float * yaws = data.trajectories.yaws.data();
  const float free2 = free_distance_ * free_distance_;
  std::vector<float> cost(batch, 0.0f), px(steps), py(steps);
  bool any = false;
  for (const auto & person : people) {
    if (std::hypot(person.x - rx, person.y - ry) > max_distance_) {
      continue;
    }
    any = true;
    for (std::size_t k = 0; k < steps; ++k) {
      px[k] = static_cast<float>(person.x + person.vx * t(k));
      py[k] = static_cast<float>(person.y + person.vy * t(k));
    }
    for (std::size_t b = 0; b < batch; ++b) {
      const std::size_t row = b * steps;
      float sum = 0.0f;
      for (std::size_t k = 0; k < steps; ++k) {
        const float dx = px[k] - xs[row + k];
        const float dy = py[k] - ys[row + k];
        const float d2 = dx * dx + dy * dy;
        if (d2 >= free2) {
          continue;   // beyond free_distance the allowed speed is max_speed
        }
        const float dist = std::max(std::sqrt(d2), 1e-3f);
        const float closing = vs[row + k] * (std::cos(yaws[row + k]) * dx + std::sin(yaws[row + k]) * dy) / dist;
        const float ramp = std::clamp((dist - slow_distance_) / (free_distance_ - slow_distance_), 0.0f, 1.0f);
        const float excess = closing - (slow_speed_ + (max_speed_ - slow_speed_) * ramp);
        if (excess > 0.0f) {
          sum += excess * excess;
        }
      }
      cost[b] += sum;
    }
  }
  if (!any) {
    return;
  }
  for (std::size_t b = 0; b < batch; ++b) {
    const float c = cost[b] * weight_;
    data.costs(b) += power_ > 1u ? std::pow(c, static_cast<float>(power_)) : c;
  }
}

}  // namespace mppi::critics

PLUGINLIB_EXPORT_CLASS(mppi::critics::SocialSpeedCritic, mppi::critics::CriticFunction)
