#include <gtest/gtest.h>
#include <gazebo/common/Mesh.hh>
#include <gazebo/common/MeshManager.hh>
#include <gazebo/common/Skeleton.hh>
#include <gazebo/common/SkeletonAnimation.hh>
#include <human_gazebo/actor_motion.hpp>
#include <human_gazebo/gait_orientation.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace {

// Reconstruct the head from the real clip's joint hierarchy. This measures the
// visual torso lean instead of just checking the actor's navigation yaw.
ignition::math::Vector3d headFromPelvis(
  gazebo::common::Skeleton *skeleton, gazebo::common::SkeletonAnimation *animation,
  double time)
{
  std::vector<gazebo::common::SkeletonNode *> chain;
  auto *root = skeleton->GetRootNode();
  for (auto *node = skeleton->GetNodeByName("Head"); node && node != root; node = node->GetParent())
    chain.push_back(node);
  std::reverse(chain.begin(), chain.end());
  auto pose = ignition::math::Matrix4d::Identity;
  const auto frame = animation->PoseAt(time, false);
  for (auto *node : chain)
  {
    const auto transform = frame.find(node->GetName());
    pose = pose * (transform == frame.end() ? node->Transform() : transform->second);
  }
  return pose.Translation();
}

double lateralLean(const ignition::math::Vector3d &up, double heading)
{
  const ignition::math::Vector3d lateral(-std::sin(heading), std::cos(heading), 0.0);
  return std::atan2(up.Dot(lateral), up.Z()) * 180.0 / std::acos(-1.0);
}

TEST(GaitBalance, WalkingKeepsTorsoBalancedThroughTheWholeClipAndAcrossHeadings)
{
  const std::string path = std::string(HUMAN_MODEL_DIRECTORY) + "/walk.dae";
  auto *manager = gazebo::common::MeshManager::Instance();
  manager->Load(path);
  const auto *mesh = manager->GetMesh(path);
  ASSERT_NE(mesh, nullptr);
  auto *skeleton = mesh->GetSkeleton();
  ASSERT_NE(skeleton, nullptr);
  ASSERT_NE(skeleton->GetNodeByName("Head"), nullptr);
  auto *animation = skeleton->GetAnimation(0);
  ASSERT_NE(animation, nullptr);
  const auto root = skeleton->GetRootNode()->GetName();

  for (double heading : {-3.0, -1.0, 0.0, 1.0, 3.0})
  {
    double low = std::numeric_limits<double>::infinity(), high = -low;
    double locked_low = low, locked_high = high;
    for (int frame = 0; frame <= 160; ++frame)
    {
      const double time = animation->GetLength() * frame / 160.0;
      const auto torso = headFromPelvis(skeleton, animation, time);
      const auto pelvis = animation->NodePoseAt(root, time, false).Rotation();
      const auto up = crowd::gaitOrientation(heading, pelvis).RotateVector(torso);
      const double lean = lateralLean(up, heading);
      low = std::min(low, lean); high = std::max(high, lean);

      const ignition::math::Quaterniond locked(crowd::kActorRoll, 0.0, heading + crowd::kActorYawOffset);
      const double old_lean = lateralLean(locked.RotateVector(torso), heading);
      locked_low = std::min(locked_low, old_lean);
      locked_high = std::max(locked_high, old_lean);
    }
    EXPECT_LT(high - low, 3.0) << "Torso rocks at heading " << heading;
    // Ensure the real clip still exercises the fixed-pelvis regression.
    EXPECT_GT(locked_high - locked_low, 8.0);
  }
}

TEST(GaitBalance, IdleWeightShiftStaysConstantAcrossNavigationHeadings)
{
  const std::string path = std::string(HUMAN_MODEL_DIRECTORY) + "/stand.dae";
  auto *manager = gazebo::common::MeshManager::Instance();
  manager->Load(path);
  const auto *mesh = manager->GetMesh(path);
  ASSERT_NE(mesh, nullptr);
  auto *skeleton = mesh->GetSkeleton();
  ASSERT_NE(skeleton, nullptr);
  auto *animation = skeleton->GetAnimation(0);
  ASSERT_NE(animation, nullptr);
  const auto pelvis = animation->NodePoseAt(skeleton->GetRootNode()->GetName(), 0.0, false).Rotation();
  const auto torso = headFromPelvis(skeleton, animation, 0.0);
  const double reference_lean = lateralLean(crowd::gaitOrientation(0.0, pelvis).RotateVector(torso), 0.0);
  for (double heading : {-3.0, 0.0, 3.0})
  {
    const auto up = crowd::gaitOrientation(heading, pelvis).RotateVector(torso);
    // This clip has a small, fixed weight shift. Turning must not turn that
    // natural standing pose into a new oscillation or excessive sideways lean.
    EXPECT_NEAR(lateralLean(up, heading), reference_lean, 1e-6);
    EXPECT_LT(std::abs(lateralLean(up, heading)), 5.0);
  }
}

}  // namespace
