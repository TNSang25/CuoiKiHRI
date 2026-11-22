// MPPI critic: for every sampled trajectory and time step t, add
//   cost_weight * sum_people Gaussian(p_robot(t) - (p0 + v * t))
// using the same velocity-aligned asymmetric Gaussian as the social costmap layer.
#ifndef SOCIAL_NAV_PLUGINS__DYNAMIC_HUMAN_CRITIC_HPP_
#define SOCIAL_NAV_PLUGINS__DYNAMIC_HUMAN_CRITIC_HPP_

#include <string>

#include "social_nav_msgs/msg/people.hpp"
#include "social_nav_plugins/people_model.hpp"
#include "nav2_mppi_controller/critic_function.hpp"

// MPPI's critic manager loads critics as "mppi::critics::<name>", so the class lives there.
namespace mppi::critics
{

class DynamicHumanCritic : public CriticFunction
{
public:
  void initialize() override;
  void score(CriticData & data) override;

protected:
  social_nav_plugins::PeopleBuffer buffer_;
  social_nav_plugins::SocialShape shape_;
  rclcpp::Subscription<social_nav_msgs::msg::People>::SharedPtr sub_;
  unsigned int power_{1};
  float weight_{1.0f};
  float max_distance_{6.0f};   // ignore people farther than this from the robot
  float max_age_{1.0f};
  float time_offset_{1.0f};    // trajectory index k is predicted at (k + offset) * model_dt
  // Predicted collision: robot within collision_distance of a person's predicted position at the
  // same time t (t <= collision_horizon). Not discounted with t, so the controller yields
  // (slows down or passes behind) instead of cutting across in front of a walking person.
  float collision_distance_{0.6f};
  float collision_weight_{30.0f};
  float collision_horizon_{2.0f};
};

}  // namespace mppi::critics

#endif  // SOCIAL_NAV_PLUGINS__DYNAMIC_HUMAN_CRITIC_HPP_
