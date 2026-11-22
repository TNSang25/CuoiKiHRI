// Shared people model for the social costmap layer and the MPPI dynamic human critic.
#ifndef SOCIAL_NAV_PLUGINS__PEOPLE_MODEL_HPP_
#define SOCIAL_NAV_PLUGINS__PEOPLE_MODEL_HPP_

#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

#include "social_nav_msgs/msg/people.hpp"
#include "rclcpp/time.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"

namespace social_nav_plugins
{

/// A person in the consumer's frame: position [m] and velocity [m/s].
struct Person
{
  double x{0.0}, y{0.0}, vx{0.0}, vy{0.0};
  double speed() const {return std::hypot(vx, vy);}
  double heading() const {return std::atan2(vy, vx);}
};

/// Asymmetric Gaussian personal space whose long axis follows the walking direction.
///   sigma_front = max(front_gain * v, min_sigma)
///   sigma_side  = side_ratio * sigma_front,  sigma_rear = rear_ratio * sigma_front
/// Below `still_speed` the heading is undefined, so the space is isotropic (sigma_front).
struct SocialShape
{
  double front_gain{2.0};
  double min_sigma{0.5};
  double side_ratio{2.0 / 3.0};
  double rear_ratio{0.5};
  double still_speed{0.1};
  // Prediction uncertainty: every sigma grows by `growth` metres per second of look-ahead,
  // and the prediction at time t is trusted with weight exp(-t / decay_time).
  double growth{0.0};
  double decay_time{0.0};   // <= 0 disables the decay

  struct Sigmas
  {
    double front, side, rear;
    bool isotropic;
  };

  Sigmas sigmas(double speed) const
  {
    const double front = std::max(front_gain * speed, min_sigma);
    if (speed < still_speed) {
      return {front, front, front, true};
    }
    return {front, side_ratio * front, rear_ratio * front, false};
  }

  /// Speed-scaled growth rate: a standing person's future position is nearly certain, so its
  /// uncertainty must not swell (a flat 0.4 m/s grew a standing person to a 1.7 m blob in 3 s).
  double growthRate(double speed) const
  {
    return std::max(0.0, growth) * std::clamp(speed, 0.1, 1.0);
  }

  /// Sigmas of the prediction `t` seconds ahead (uncertainty grows with t and speed).
  Sigmas sigmasAt(double speed, double t) const
  {
    Sigmas s = sigmas(speed);
    const double extra = growthRate(speed) * std::max(0.0, t);
    s.front += extra;
    s.side += extra;
    s.rear += extra;
    return s;
  }

  /// Confidence in the prediction `t` seconds ahead.
  double weightAt(double t) const
  {
    return decay_time > 0.0 ? std::exp(-std::max(0.0, t) / decay_time) : 1.0;
  }

  /// Value in [0, 1] at offset (dx, dy) from the person's (predicted) centre.
  static double value(double dx, double dy, double cos_h, double sin_h, const Sigmas & s)
  {
    const double along = cos_h * dx + sin_h * dy;     // + in front of the person
    const double across = -sin_h * dx + cos_h * dy;
    const double s_along = along >= 0.0 ? s.front : s.rear;
    return std::exp(-(along * along / (2.0 * s_along * s_along) +
                      across * across / (2.0 * s.side * s.side)));
  }

  /// Radius beyond which value() < cutoff in every direction.
  static double support(const Sigmas & s, double cutoff)
  {
    return std::max({s.front, s.side, s.rear}) * std::sqrt(2.0 * std::log(1.0 / cutoff));
  }
};

/// Thread-safe holder of the latest /people message, transformed on demand.
class PeopleBuffer
{
public:
  void store(const social_nav_msgs::msg::People::SharedPtr & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_ = msg;
  }

  /// People expressed in `target_frame`; empty if the data is older than `max_age` seconds
  /// or the transform is unavailable (reason written to `error`).
  std::vector<Person> snapshot(
    tf2_ros::Buffer & tf, const std::string & target_frame, const rclcpp::Time & now,
    double max_age, std::string * error = nullptr) const
  {
    social_nav_msgs::msg::People::SharedPtr msg;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      msg = latest_;
    }
    std::vector<Person> people;
    if (!msg) {
      return people;
    }
    const rclcpp::Time stamp(msg->header.stamp, now.get_clock_type());
    if (max_age > 0.0 && stamp.nanoseconds() > 0 && (now - stamp).seconds() > max_age) {
      if (error) {*error = "people data is stale";}
      return people;
    }
    double tx = 0.0, ty = 0.0, yaw = 0.0;
    const std::string & source = msg->header.frame_id;
    if (!source.empty() && source != target_frame) {
      try {
        const auto t = tf.lookupTransform(target_frame, source, tf2::TimePointZero);
        tx = t.transform.translation.x;
        ty = t.transform.translation.y;
        const auto & q = t.transform.rotation;
        yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      } catch (const tf2::TransformException & ex) {
        if (error) {*error = ex.what();}
        return people;
      }
    }
    const double c = std::cos(yaw), s = std::sin(yaw);
    people.reserve(msg->people.size());
    for (const auto & p : msg->people) {
      Person q;
      q.x = c * p.position.x - s * p.position.y + tx;
      q.y = s * p.position.x + c * p.position.y + ty;
      q.vx = c * p.velocity.x - s * p.velocity.y;
      q.vy = s * p.velocity.x + c * p.velocity.y;
      people.push_back(q);
    }
    return people;
  }

private:
  mutable std::mutex mutex_;
  social_nav_msgs::msg::People::SharedPtr latest_;
};

}  // namespace social_nav_plugins

#endif  // SOCIAL_NAV_PLUGINS__PEOPLE_MODEL_HPP_
