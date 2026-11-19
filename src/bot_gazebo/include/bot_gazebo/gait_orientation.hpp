#ifndef BOT_GAZEBO_GAIT_ORIENTATION_HPP_
#define BOT_GAZEBO_GAIT_ORIENTATION_HPP_

#include <ignition/math/Quaternion.hh>

namespace bot {

// The clip already contains the rig's rest-axis conversion and pelvis motion.
// Rotate that complete pose into the navigation frame so the spine's balancing
// motion still cancels the pelvis sway.
inline ignition::math::Quaterniond gaitOrientation(
  double navigation_heading, const ignition::math::Quaterniond &pelvis)
{
  return ignition::math::Quaterniond(0.0, 0.0, navigation_heading) * pelvis;
}

}  // namespace bot
#endif
