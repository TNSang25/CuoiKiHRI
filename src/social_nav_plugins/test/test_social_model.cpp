#include <gtest/gtest.h>

#include "social_nav_plugins/people_model.hpp"
#include "social_nav_plugins/social_layer.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

using social_nav_plugins::Person;
using social_nav_plugins::SocialShape;

TEST(SocialShape, SigmasFollowTheMethod)
{
  SocialShape shape;
  auto s = shape.sigmas(1.0);   // sigma_front = max(2v, 0.5) = 2
  EXPECT_DOUBLE_EQ(s.front, 2.0);
  EXPECT_NEAR(s.side, 2.0 * 2.0 / 3.0, 1e-12);
  EXPECT_DOUBLE_EQ(s.rear, 1.0);
  s = shape.sigmas(0.1);        // floor at 0.5
  EXPECT_DOUBLE_EQ(s.front, 0.5);
}

TEST(SocialShape, FrontReachesFurtherThanRearAndSide)
{
  SocialShape shape;
  const auto s = shape.sigmas(1.0);
  // Walking along +x: 1 m ahead costs more than 1 m behind or 1 m to the side.
  const double front = SocialShape::value(1.0, 0.0, 1.0, 0.0, s);
  const double rear = SocialShape::value(-1.0, 0.0, 1.0, 0.0, s);
  const double side = SocialShape::value(0.0, 1.0, 1.0, 0.0, s);
  EXPECT_GT(front, side);
  EXPECT_GT(side, rear);
  EXPECT_DOUBLE_EQ(SocialShape::value(0.0, 0.0, 1.0, 0.0, s), 1.0);
}

TEST(SocialLayer, LethalCoreAndPredictionSweep)
{
  social_nav_plugins::SocialLayer layer;
  Person p;
  p.x = 0.0; p.y = 0.0; p.vx = 1.0; p.vy = 0.0;
  EXPECT_EQ(layer.costAt(p, 0.3, 0.0), nav2_costmap_2d::LETHAL_OBSTACLE);   // < 0.45 m
  // 1.5 m ahead lies on the predicted path (p0 + v t, t = 1.5 s): high but not lethal.
  const unsigned char ahead = layer.costAt(p, 1.5, 0.0);
  EXPECT_LT(ahead, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_GT(ahead, 240);
  // Same distance behind the person is much cheaper.
  EXPECT_LT(layer.costAt(p, -1.5, 0.0), ahead / 2);
  // Far away: nothing.
  EXPECT_EQ(layer.costAt(p, 0.0, 8.0), 0);
}

TEST(SocialLayer, StandingPersonIsIsotropic)
{
  social_nav_plugins::SocialLayer layer;
  Person p;
  EXPECT_EQ(layer.costAt(p, 0.8, 0.0), layer.costAt(p, 0.0, 0.8));
  EXPECT_EQ(layer.costAt(p, 0.8, 0.0), layer.costAt(p, -0.8, 0.0));
}

TEST(SocialShape, SigmasGrowWithLookAheadTime)
{
  SocialShape shape;
  shape.growth = 0.25;
  const auto now = shape.sigmasAt(1.0, 0.0);
  const auto later = shape.sigmasAt(1.0, 2.0);
  EXPECT_DOUBLE_EQ(later.front - now.front, 0.5);
  EXPECT_DOUBLE_EQ(later.side - now.side, 0.5);
  EXPECT_DOUBLE_EQ(later.rear - now.rear, 0.5);
  EXPECT_DOUBLE_EQ(shape.weightAt(1.0), 1.0);   // no decay by default
  shape.decay_time = 1.0;
  EXPECT_NEAR(shape.weightAt(1.0), std::exp(-1.0), 1e-12);
}

TEST(SocialLayer, DistantPredictionIsDiscounted)
{
  social_nav_plugins::SocialLayer plain, uncertain;
  uncertain.shape.decay_time = 1.0;
  uncertain.shape.growth = 0.25;
  Person p;
  p.vx = 1.0;
  // On the path 1.8 m ahead the plain model is near its peak; with decay it is much lower.
  EXPECT_GT(plain.costAt(p, 1.8, 0.0), 240);
  EXPECT_LT(uncertain.costAt(p, 1.8, 0.0), plain.costAt(p, 1.8, 0.0) * 0.7);
  // Next to the person (t ~ 0) both agree on the lethal core.
  EXPECT_EQ(uncertain.costAt(p, 0.2, 0.0), plain.costAt(p, 0.2, 0.0));
}
