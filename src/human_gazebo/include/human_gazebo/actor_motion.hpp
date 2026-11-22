#ifndef SIM_GAZEBO_ACTOR_MOTION_HPP_
#define SIM_GAZEBO_ACTOR_MOTION_HPP_

#include <algorithm>
#include <cmath>

namespace crowd {

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

// Simulated legs (lidar-visible collision of each person).
constexpr double kLegRadius = 0.065;      // calf radius [m]
constexpr double kLegLength = 0.8;        // from the floor up [m]
constexpr double kHipHalfWidth = 0.1;     // lateral offset of each leg [m]
constexpr double kGaitCycleLength = 1.2;  // distance walked per full left+right cycle [m]
constexpr double kMaxLegSwing = 0.22;     // max fore-aft foot offset at ~1.2 m/s [m]

/// Fore-aft offset of the left leg (the right leg gets the opposite) for a gait phase and speed.
inline double legSwing(double phase, double speed)
{
  const double amplitude = kMaxLegSwing * std::min(1.0, std::max(0.0, speed / 1.2));
  return amplitude * std::sin(phase);
}

inline double movementHeading(double dx, double dy, double stationary_yaw)
{
  return std::hypot(dx, dy) > 1e-6 ? std::atan2(dy, dx) : stationary_yaw;
}

}  // namespace crowd
#endif
