// MPPI critic: commit early to a conventional passing side.
// Each nearby moving person is classified by the angle between the robot's heading and the
// person's walking direction:
//   oncoming   (> 135 deg)  keep right: the person must end up on the robot's left
//   crossing   (45-135 deg) pass behind: the robot must not cross in front of the person
//   same way   (< 45 deg)   overtake on the left: the person must end up on the robot's right
//   standing   (slow)       no preference
// For every sampled trajectory, at its closest approach to the person's predicted position
// p0 + v t (same t), if that approach is within `pass_distance` the signed clearance c on the
// conventional side (lateral offset, or distance behind a crossing person) is scored as
//   cost_weight * clamp((pass_offset - c) / pass_offset, 0, 2)
// i.e. 0 with enough room on the right side, 1 head-on, up to 2 on the wrong side.
// The MPPI horizon is ~4.5 s, so the side is chosen metres before the encounter instead of
// being renegotiated at close range (a cause of stop-and-go).
// `keep_side: left` mirrors the convention for left-hand traffic.
#ifndef SOCIAL_NAV_PLUGINS__PASSING_SIDE_CRITIC_HPP_
#define SOCIAL_NAV_PLUGINS__PASSING_SIDE_CRITIC_HPP_

#include <string>

#include "social_nav_msgs/msg/people.hpp"
#include "social_nav_plugins/people_model.hpp"
#include "nav2_mppi_controller/critic_function.hpp"

namespace mppi::critics
{

class PassingSideCritic : public CriticFunction
{
public:
  enum class Encounter {kNone, kOncoming, kCrossing, kSameWay};

  void initialize() override;
  void score(CriticData & data) override;

  /// Encounter type from the robot heading and the person's velocity (exposed for tests).
  static Encounter classify(double robot_yaw, double person_vx, double person_vy, double min_speed);

protected:
  social_nav_plugins::PeopleBuffer buffer_;
  rclcpp::Subscription<social_nav_msgs::msg::People>::SharedPtr sub_;
  float weight_{15.0f};
  float pass_distance_{2.0f};   // closest approach beyond which the side does not matter [m]
  float pass_offset_{0.9f};     // wanted clearance on the conventional side [m]
  float min_speed_{0.2f};       // slower people have no walking direction to respect [m/s]
  float max_distance_{6.0f};    // only people this close to the robot are considered [m]
  float max_age_{1.0f};
  float time_offset_{1.0f};
  float side_sign_{1.0f};       // +1 keep right (person on the robot's left when oncoming)
};

}  // namespace mppi::critics

#endif  // SOCIAL_NAV_PLUGINS__PASSING_SIDE_CRITIC_HPP_
