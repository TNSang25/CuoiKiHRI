// MPPI critic: slow down when approaching people.
// For every sampled trajectory, step t and person with predicted position p(t) = p0 + v t at
// distance d(t), the robot's speed towards p(t) may not exceed an allowed value that ramps
// linearly from `slow_speed` at d <= slow_distance to `max_speed` at d >= free_distance:
//   cost += cost_weight * sum_t sum_people max(0, v_towards(t) - v_allowed(d(t)))^2
// Moving away from or sideways past a person is not penalised.
#ifndef SOCIAL_NAV_PLUGINS__SOCIAL_SPEED_CRITIC_HPP_
#define SOCIAL_NAV_PLUGINS__SOCIAL_SPEED_CRITIC_HPP_

#include "social_nav_msgs/msg/people.hpp"
#include "social_nav_plugins/people_model.hpp"
#include "nav2_mppi_controller/critic_function.hpp"

namespace mppi::critics
{

class SocialSpeedCritic : public CriticFunction
{
public:
  void initialize() override;
  void score(CriticData & data) override;

protected:
  social_nav_plugins::PeopleBuffer buffer_;
  rclcpp::Subscription<social_nav_msgs::msg::People>::SharedPtr sub_;
  unsigned int power_{1};
  float weight_{10.0f};
  float slow_distance_{0.6f};   // centre distance at which only slow_speed is allowed [m]
  float free_distance_{2.0f};   // beyond this distance speed is not limited [m]
  float slow_speed_{0.15f};     // [m/s]
  float max_speed_{0.8f};       // allowed speed at free_distance [m/s]
  float max_distance_{6.0f};
  float max_age_{1.0f};
  float time_offset_{1.0f};
};

}  // namespace mppi::critics

#endif  // SOCIAL_NAV_PLUGINS__SOCIAL_SPEED_CRITIC_HPP_
