// 2-D lidar leg detection and people tracking (no ROS dependencies, unit-testable).
//
// Per scan (points already in the fixed frame):
//   1. drop points close to the static map (walls, furniture)
//   2. split consecutive points into clusters at gaps
//   3. keep leg-sized clusters (a double-width cluster is split at its widest gap)
//   4. pair legs into people (a lone leg also counts: the other may be occluded)
//   5. track people with constant-velocity Kalman filters and nearest-neighbour association
#ifndef PEOPLE_TRACKER__TRACKING_HPP_
#define PEOPLE_TRACKER__TRACKING_HPP_

#include <cstdint>
#include <vector>

#include <Eigen/Core>

namespace people_tracker
{

using Point = Eigen::Vector2d;
using Points = std::vector<Point>;

struct LegParams
{
  double cluster_gap{0.06};        // max distance between consecutive points of one cluster [m]
  int min_points{2};
  double min_width{0.02};          // cluster extent limits for one leg [m]
  double max_width{0.18};          // a leg is at most ~2 x 0.065 m plus noise
  double leg_radius{0.065};        // pushes the visible surface back to the leg axis [m]
  double max_pair_distance{0.55};  // two legs of the same person [m]
};

struct TrackerParams
{
  double gate{0.8};                // max association distance [m]
  double accel_noise{1.0};         // process noise (white acceleration) [m/s^2]
  double measurement_noise{0.08};  // detection position std [m]
  int confirm_hits{3};
  double max_coast{1.0};           // delete a track not updated for this long [s]
  double max_speed{2.5};           // clamp absurd velocity estimates [m/s]
};

struct Track
{
  int id{0};
  Eigen::Vector4d x{Eigen::Vector4d::Zero()};   // [px, py, vx, vy]
  Eigen::Matrix4d P{Eigen::Matrix4d::Identity()};
  double last_update{0.0};
  int hits{1};
  bool confirmed{false};
  Point position() const {return x.head<2>();}
  Point velocity() const {return x.tail<2>();}
};

/// Split ordered points into clusters wherever consecutive points are more than `gap` apart.
std::vector<Points> clusterPoints(const Points & points, double gap);

/// Leg centres from ordered scan points seen from `sensor`.
Points detectLegs(const Points & points, const Point & sensor, const LegParams & params = LegParams());

/// Greedy closest-first pairing of legs into people; unpaired legs are people too.
Points pairLegs(const Points & legs, double max_distance);

/// Valid scan returns (range_min < r < min(range_max, max_range)) as ordered fixed-frame points.
Points scanToPoints(
  const std::vector<float> & ranges, double angle_min, double angle_increment, double range_min,
  double range_max, double sensor_x, double sensor_y, double sensor_yaw, double max_range);

/// Occupancy grid dilated by a clearance radius, for O(1) "near a static obstacle" lookups.
class StaticMask
{
public:
  /// `occupied` is row-major, row 0 = min y (ROS OccupancyGrid layout).
  void build(
    const std::vector<std::uint8_t> & occupied, int width, int height, double origin_x,
    double origin_y, double resolution, double clearance);
  bool ready() const {return width_ > 0;}
  bool blocked(const Point & p) const;
  Points filter(const Points & points) const;

private:
  std::vector<std::uint8_t> mask_;
  int width_{0}, height_{0};
  double origin_x_{0.0}, origin_y_{0.0}, resolution_{0.05};
};

class PeopleTracker
{
public:
  explicit PeopleTracker(const TrackerParams & params = TrackerParams()) : params_(params) {}

  /// Advance all tracks to `stamp` [s], fuse detections and return the confirmed tracks.
  std::vector<Track> update(const Points & detections, double stamp);

  const std::vector<Track> & tracks() const {return tracks_;}

private:
  void predict(Track & track, double dt) const;
  void correct(Track & track, const Point & z, double stamp) const;

  TrackerParams params_;
  std::vector<Track> tracks_;
  int next_id_{1};
  bool has_time_{false};
  double last_time_{0.0};
};

}  // namespace people_tracker

#endif  // PEOPLE_TRACKER__TRACKING_HPP_
