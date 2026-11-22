#include "social_nav_plugins/dynamic_human_critic.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <xtensor/xmath.hpp>
#include <xtensor/xview.hpp>
#include <xtensor/xbuilder.hpp>
#include <xtensor/xoperation.hpp>

#include "pluginlib/class_list_macros.hpp"

namespace mppi::critics
{

void DynamicHumanCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 1.0f);
  getParam(max_distance_, "max_distance", 6.0f);
  getParam(max_age_, "max_age", 1.0f);
  getParam(time_offset_, "time_offset", 1.0f);
  getParam(shape_.front_gain, "front_gain", 2.0);
  getParam(shape_.min_sigma, "min_sigma", 0.5);
  getParam(shape_.side_ratio, "side_ratio", 2.0 / 3.0);
  getParam(shape_.rear_ratio, "rear_ratio", 0.5);
  getParam(shape_.still_speed, "still_speed", 0.1);
  getParam(collision_distance_, "collision_distance", 0.6f);
  getParam(collision_weight_, "collision_weight", 30.0f);
  getParam(collision_horizon_, "collision_horizon", 2.0f);
  getParam(shape_.growth, "sigma_growth", 0.0);
  getParam(shape_.decay_time, "decay_time", 0.0);
  std::string topic;
  getParam(topic, "people_topic", std::string("/people"));

  auto node = parent_.lock();
  sub_ = node->create_subscription<social_nav_msgs::msg::People>(
    topic, rclcpp::QoS(5),
    [this](const social_nav_msgs::msg::People::SharedPtr msg) {buffer_.store(msg);});
  RCLCPP_INFO(
    logger_, "DynamicHumanCritic: people from %s, weight %.2f, power %u", topic.c_str(),
    weight_, power_);
}

void DynamicHumanCritic::score(CriticData & data)
{
  if (!enabled_) {
    return;
  }
  auto node = parent_.lock();
  std::string error;
  const auto people = buffer_.snapshot(
    *costmap_ros_->getTfBuffer(), costmap_ros_->getGlobalFrameID(), node->now(), max_age_, &error);
  if (!error.empty()) {
    RCLCPP_WARN_THROTTLE(logger_, *node->get_clock(), 5000, "DynamicHumanCritic: %s", error.c_str());
  }
  if (people.empty()) {
    return;
  }
  const float rx = static_cast<float>(data.state.pose.pose.position.x);
  const float ry = static_cast<float>(data.state.pose.pose.position.y);
  // Plain loops over contiguous [batch, time_steps] storage: per-step quantities are computed
  // once, and every trajectory point is visited once per person. (Lazy xtensor expressions
  // re-evaluated dx/dy/along several times per element: ~100 ms per cycle with 8 people.)
  const auto & tx = data.trajectories.x;   // costmap global frame
  const auto & ty = data.trajectories.y;
  const std::size_t batch = tx.shape(0), steps = tx.shape(1);
  const float * xs = tx.data();
  const float * ys = ty.data();
  const float decay = static_cast<float>(shape_.decay_time);
  const float col_d2 = collision_distance_ * collision_distance_;

  std::vector<float> t(steps), weight_t(steps);
  std::size_t col_steps = 0;   // steps inside the collision horizon
  for (std::size_t k = 0; k < steps; ++k) {
    t[k] = (static_cast<float>(k) + time_offset_) * data.model_dt;
    weight_t[k] = decay > 0.0f ? std::exp(-t[k] / decay) : 1.0f;
    if (t[k] <= collision_horizon_) {
      col_steps = k + 1;
    }
  }
  std::vector<float> human_cost(batch, 0.0f), collision_cost(batch, 0.0f);
  std::vector<float> px(steps), py(steps), inv_front(steps), inv_rear(steps), inv_side(steps), reach2(steps);
  for (const auto & person : people) {
    if (std::hypot(person.x - rx, person.y - ry) > max_distance_) {
      continue;
    }
    const auto sig = shape_.sigmas(person.speed());
    const float growth = static_cast<float>(shape_.growthRate(person.speed()));
    const float c = static_cast<float>(std::cos(person.heading()));
    const float s = static_cast<float>(std::sin(person.heading()));
    for (std::size_t k = 0; k < steps; ++k) {
      px[k] = static_cast<float>(person.x + person.vx * t[k]);
      py[k] = static_cast<float>(person.y + person.vy * t[k]);
      // Sigmas widen with look-ahead time (prediction uncertainty).
      const float front = static_cast<float>(sig.front) + growth * t[k];
      const float rear = static_cast<float>(sig.rear) + growth * t[k];
      const float side = static_cast<float>(sig.side) + growth * t[k];
      inv_front[k] = 0.5f / (front * front);
      inv_rear[k] = 0.5f / (rear * rear);
      inv_side[k] = 0.5f / (side * side);
      // Beyond ~4.3 sigma the Gaussian is < 1e-4: skip the exp.
      const float widest = std::max({front, rear, side, collision_distance_});
      reach2[k] = 18.4f * widest * widest;
    }
    for (std::size_t b = 0; b < batch; ++b) {
      const float * xb = xs + b * steps;
      const float * yb = ys + b * steps;
      float gauss_sum = 0.0f, overlap_sum = 0.0f;
      for (std::size_t k = 0; k < steps; ++k) {
        const float dx = xb[k] - px[k];
        const float dy = yb[k] - py[k];
        const float d2 = dx * dx + dy * dy;
        if (d2 > reach2[k]) {
          continue;
        }
        const float along = c * dx + s * dy;
        const float across = -s * dx + c * dy;
        const float inv_along = along >= 0.0f ? inv_front[k] : inv_rear[k];
        gauss_sum += weight_t[k] * std::exp(-(along * along * inv_along + across * across * inv_side[k]));
        // Hinge on the predicted centre distance: 1 at contact, 0 at collision_distance.
        if (k < col_steps && d2 < col_d2) {
          overlap_sum += 1.0f - std::sqrt(d2) / collision_distance_;
        }
      }
      human_cost[b] += gauss_sum;
      collision_cost[b] += overlap_sum;
    }
  }
  for (std::size_t b = 0; b < batch; ++b) {
    const float social = human_cost[b] * weight_;
    data.costs(b) += (power_ > 1u ? std::pow(social, static_cast<float>(power_)) : social) +
      collision_cost[b] * collision_weight_;
  }
}

}  // namespace mppi::critics

PLUGINLIB_EXPORT_CLASS(mppi::critics::DynamicHumanCritic, mppi::critics::CriticFunction)
