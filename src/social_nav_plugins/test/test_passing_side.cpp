#include <gtest/gtest.h>

#include <cmath>

#include "social_nav_plugins/passing_side_critic.hpp"

using mppi::critics::PassingSideCritic;
using Encounter = PassingSideCritic::Encounter;

TEST(PassingSide, ClassifiesEncounters)
{
  // Robot heading +x.
  EXPECT_EQ(PassingSideCritic::classify(0.0, -1.0, 0.1, 0.2), Encounter::kOncoming);
  EXPECT_EQ(PassingSideCritic::classify(0.0, 0.1, -1.0, 0.2), Encounter::kCrossing);
  EXPECT_EQ(PassingSideCritic::classify(0.0, 1.0, 0.2, 0.2), Encounter::kSameWay);
  EXPECT_EQ(PassingSideCritic::classify(0.0, 0.05, 0.05, 0.2), Encounter::kNone);   // standing
}

TEST(PassingSide, ClassificationFollowsRobotHeading)
{
  // Robot heading +y: a person walking -y is oncoming, one walking +x is crossing.
  EXPECT_EQ(PassingSideCritic::classify(M_PI / 2, 0.0, -0.9, 0.2), Encounter::kOncoming);
  EXPECT_EQ(PassingSideCritic::classify(M_PI / 2, 0.9, 0.0, 0.2), Encounter::kCrossing);
}
