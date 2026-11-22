#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>

#include "people_tracker/tracking.hpp"

using people_tracker::Point;
using people_tracker::Points;

namespace
{
// Visible half of a leg cylinder as seen from the sensor at the origin.
Points legArc(double cx, double cy, int n = 6, double radius = 0.065)
{
  const double facing = std::atan2(-cy, -cx);
  Points pts;
  for (int i = 0; i < n; ++i) {
    const double a = facing - 1.0 + 2.0 * i / (n - 1);
    pts.emplace_back(cx + radius * std::cos(a), cy + radius * std::sin(a));
  }
  return pts;
}

Points sortedByBearing(Points pts)
{
  std::sort(pts.begin(), pts.end(), [](const Point & a, const Point & b) {
      return std::atan2(a.y(), a.x()) < std::atan2(b.y(), b.x());
    });
  return pts;
}
}  // namespace

TEST(LegDetection, TwoLegsBecomeOnePersonAtTheirMidpoint)
{
  Points pts = legArc(3.0, -0.1);
  const Points other = legArc(3.0, 0.1);
  pts.insert(pts.end(), other.begin(), other.end());
  const auto legs = people_tracker::detectLegs(sortedByBearing(pts), Point::Zero());
  ASSERT_EQ(legs.size(), 2u);
  const auto people = people_tracker::pairLegs(legs, 0.55);
  ASSERT_EQ(people.size(), 1u);
  EXPECT_NEAR(people[0].x(), 3.0, 0.03);
  EXPECT_NEAR(people[0].y(), 0.0, 0.03);
}

TEST(LegDetection, TouchingLegsAreSplit)
{
  Points pts = legArc(4.0, -0.07, 4);
  const Points other = legArc(4.0, 0.07, 4);
  pts.insert(pts.end(), other.begin(), other.end());
  EXPECT_EQ(people_tracker::detectLegs(sortedByBearing(pts), Point::Zero()).size(), 2u);
}

TEST(LegDetection, WallSizedClusterIsNotALeg)
{
  Points wall;
  for (int i = 0; i < 40; ++i) {
    wall.emplace_back(2.0, -1.0 + 2.0 * i / 39.0);
  }
  EXPECT_TRUE(people_tracker::detectLegs(wall, Point::Zero()).empty());
}

TEST(Tracker, ConfirmsAndEstimatesVelocity)
{
  people_tracker::PeopleTracker tracker;
  std::vector<people_tracker::Track> tracks;
  for (int k = 0; k < 30; ++k) {
    const double t = 0.1 * k;
    tracks = tracker.update({Point(1.0 + 1.2 * t, 2.0)}, t);
  }
  ASSERT_EQ(tracks.size(), 1u);
  EXPECT_NEAR(tracks[0].velocity().x(), 1.2, 0.1);
  EXPECT_NEAR(tracks[0].velocity().y(), 0.0, 0.1);
}

TEST(Tracker, DropsTracksAfterCoasting)
{
  people_tracker::PeopleTracker tracker;
  for (int k = 0; k < 5; ++k) {
    tracker.update({Point::Zero()}, 0.1 * k);
  }
  EXPECT_TRUE(tracker.update({}, 2.0).empty());
}

TEST(StaticMask, RemovesPointsNearTheMap)
{
  // 40 x 40 grid, resolution 0.05, wall in column 20 (x = 1.0..1.05).
  std::vector<std::uint8_t> occupied(40 * 40, 0);
  for (int r = 0; r < 40; ++r) {
    occupied[r * 40 + 20] = 1;
  }
  people_tracker::StaticMask mask;
  mask.build(occupied, 40, 40, 0.0, 0.0, 0.05, 0.15);
  const auto kept = mask.filter({Point(1.01, 0.5), Point(0.5, 0.5)});
  ASSERT_EQ(kept.size(), 1u);
  EXPECT_DOUBLE_EQ(kept[0].x(), 0.5);
}

TEST(Scan, InvalidRangesAreDropped)
{
  const std::vector<float> ranges{1.0f, std::numeric_limits<float>::infinity(), 0.05f, 2.0f};
  const auto pts = people_tracker::scanToPoints(ranges, 0.0, M_PI / 2, 0.1, 10.0, 0.0, 0.0, 0.0, 10.0);
  ASSERT_EQ(pts.size(), 2u);
  EXPECT_NEAR(pts[0].x(), 1.0, 1e-9);
  EXPECT_NEAR(pts[1].y(), -2.0, 1e-9);
}
