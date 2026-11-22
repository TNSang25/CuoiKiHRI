#include "social_nav_plugins/passing_side_critic.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace mppi::critics
{

void PassingSideCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(weight_, "cost_weight", 15.0f);
  getParam(pass_distance_, "pass_distance", 2.0f);
  getParam(pass_offset_, "pass_offset", 0.9f);
  getParam(min_speed_, "min_speed", 0.2f);
  getParam(max_distance_, "max_distance", 6.0f);
  getParam(max_age_, "max_age", 1.0f);
  getParam(time_offset_, "time_offset", 1.0f);
  std::string topic, side;
  getParam(topic, "people_topic", std::string("/people"));
  getParam(side, "keep_side", std::string("right"));
  side_sign_ = side == "left" ? -1.0f : 1.0f;

  auto node = parent_.lock();
  sub_ = node->create_subscription<social_nav_msgs::msg::People>(
    topic, rclcpp::QoS(5),
    [this](const social_nav_msgs::msg::People::SharedPtr msg) {buffer_.store(msg);});
  RCLCPP_INFO(
    logger_, "PassingSideCritic: keep %s, wrong-side passes within %.1f m cost up to %.1f",
    side.c_str(), pass_distance_, weight_);
}

PassingSideCritic::Encounter PassingSideCritic::classify(
  double robot_yaw, double person_vx, double person_vy, double min_speed)
{
  if (std::hypot(person_vx, person_vy) < min_speed) {
    return Encounter::kNone;
  }
  const double rel = std::atan2(person_vy, person_vx) - robot_yaw;
  const double angle = std::abs(std::atan2(std::sin(rel), std::cos(rel)));   // [0, pi]
  if (angle > 0.75 * M_PI) {
    return Encounter::kOncoming;
  }
  if (angle < 0.25 * M_PI) {
    return Encounter::kSameWay;
  }
  return Encounter::kCrossing;
}

void PassingSideCritic::score(CriticData & data)
{
  if (!enabled_) {
    return;
  }
  auto node = parent_.lock();
  std::string error;
  const auto people = buffer_.snapshot(
    *costmap_ros_->getTfBuffer(), costmap_ros_->getGlobalFrameID(), node->now(), max_age_, &error);
  if (people.empty()) {
    return;
  }
  const auto & pose = data.state.pose.pose;
  const double robot_yaw = std::atan2(
    2.0 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y),
    1.0 - 2.0 * (pose.orientation.y * pose.orientation.y + pose.orientation.z * pose.orientation.z));

  const auto & tx = data.trajectories.x;
  const std::size_t batch = tx.shape(0), steps = tx.shape(1);
  const float * xs = tx.data();
  const float * ys = data.trajectories.y.data();
  const float * yaws = data.trajectories.yaws.data();
  std::vector<float> px(steps), py(steps), cost(batch, 0.0f);
  const float pass2 = pass_distance_ * pass_distance_;
  bool any = false;

  for (const auto & person : people) {
    if (std::hypot(person.x - pose.position.x, person.y - pose.position.y) > max_distance_) {
      continue;
    }
    const Encounter encounter = classify(robot_yaw, person.vx, person.vy, min_speed_);
    if (encounter == Encounter::kNone) {
      continue;
    }
    any = true;
    const float speed = static_cast<float>(person.speed());
    const float ux = static_cast<float>(person.vx) / speed, uy = static_cast<float>(person.vy) / speed;
    for (std::size_t k = 0; k < steps; ++k) {
      const float t = (static_cast<float>(k) + time_offset_) * data.model_dt;
      px[k] = static_cast<float>(person.x) + static_cast<float>(person.vx) * t;
      py[k] = static_cast<float>(person.y) + static_cast<float>(person.vy) * t;
    }
    for (std::size_t b = 0; b < batch; ++b) {
      const std::size_t row = b * steps;
      // Closest approach to the predicted person along this trajectory.
      float best2 = std::numeric_limits<float>::max();
      std::size_t kmin = 0;
      for (std::size_t k = 0; k < steps; ++k) {
        const float dx = px[k] - xs[row + k], dy = py[k] - ys[row + k];
        const float d2 = dx * dx + dy * dy;
        if (d2 < best2) {
          best2 = d2;
          kmin = k;
        }
      }
      if (best2 >= pass2) {
        continue;   // never close enough for the side to matter
      }
      const float dx = px[kmin] - xs[row + kmin], dy = py[kmin] - ys[row + kmin];
      // Signed clearance on the conventional side at the closest approach (> 0 = right side).
      float clearance;
      if (encounter == Encounter::kCrossing) {
        // How far behind the person (along its walking direction) the robot passes.
        clearance = ux * dx + uy * dy;
      } else {
        // > 0: person on the robot's left (relative to the trajectory's heading there).
        const float left = std::cos(yaws[row + kmin]) * dy - std::sin(yaws[row + kmin]) * dx;
        clearance = left * (encounter == Encounter::kOncoming ? side_sign_ : -side_sign_);
      }
      // Not just the side: a head-on line counts as 1, the wrong side up to 2. Straight-at-the-
      // person trajectories are therefore penalised as soon as the encounter enters the horizon,
      // which makes the robot drift to its side early instead of braking and swerving late.
      cost[b] += std::clamp((pass_offset_ - clearance) / pass_offset_, 0.0f, 2.0f);
    }
  }
  if (!any) {
    return;
  }
  for (std::size_t b = 0; b < batch; ++b) {
    data.costs(b) += cost[b] * weight_;
  }
}

}  // namespace mppi::critics

PLUGINLIB_EXPORT_CLASS(mppi::critics::PassingSideCritic, mppi::critics::CriticFunction)
