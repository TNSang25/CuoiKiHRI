#ifndef BOT_GAZEBO_ACTOR_MOTION_HPP_
#define BOT_GAZEBO_ACTOR_MOTION_HPP_

#include <cmath>

namespace bot {

constexpr double kActorRoll = 1.5707963267948966;
constexpr double kActorYawOffset = 1.5707963267948966;

inline double actorHeight(unsigned int skin)
{
  switch (skin) {
    case 0: return 0.96;
    case 1: return 0.97;
    case 2: case 3: return 0.93;
    case 4: return 0.97;
    default: return 1.05;
  }
}

inline double movementHeading(double dx, double dy, double stationary_yaw)
{
  return std::hypot(dx, dy) > 1e-6 ? std::atan2(dy, dx) : stationary_yaw;
}

}  // namespace bot
#endif
