#include "people_tracker/tracking.hpp"

#include <algorithm>
#include <cmath>
#include <tuple>

#include <Eigen/Dense>

namespace people_tracker
{

std::vector<Points> clusterPoints(const Points & points, double gap)
{
  std::vector<Points> clusters;
  for (const auto & p : points) {
    if (clusters.empty() || (p - clusters.back().back()).norm() > gap) {
      clusters.emplace_back();
    }
    clusters.back().push_back(p);
  }
  return clusters;
}

Points detectLegs(const Points & points, const Point & sensor, const LegParams & params)
{
  auto width = [](const Points & c) {return (c.front() - c.back()).norm();};
  auto leg_like = [&](const Points & c) {
      return static_cast<int>(c.size()) >= params.min_points &&
             width(c) >= params.min_width && width(c) <= params.max_width;
    };

  std::vector<Points> candidates;
  for (auto & cluster : clusterPoints(points, params.cluster_gap)) {
    if (leg_like(cluster)) {
      candidates.push_back(std::move(cluster));
    } else if (static_cast<int>(cluster.size()) >= 2 * params.min_points &&
      width(cluster) <= 2.0 * params.max_width)
    {
      // Two legs close together (or far away) merge into one cluster: split at its widest gap.
      std::size_t split = 1;
      double widest = -1.0;
      for (std::size_t i = 1; i < cluster.size(); ++i) {
        const double g = (cluster[i] - cluster[i - 1]).norm();
        if (g > widest) {
          widest = g;
          split = i;
        }
      }
      Points left(cluster.begin(), cluster.begin() + split);
      Points right(cluster.begin() + split, cluster.end());
      if (leg_like(left) && leg_like(right)) {
        candidates.push_back(std::move(left));
        candidates.push_back(std::move(right));
      }
    }
  }

  Points legs;
  legs.reserve(candidates.size());
  for (const auto & cluster : candidates) {
    Point centre = Point::Zero();
    for (const auto & p : cluster) {
      centre += p;
    }
    centre /= static_cast<double>(cluster.size());
    const Point ray = centre - sensor;
    if (ray.norm() > 1e-6) {
      centre += ray.normalized() * params.leg_radius;
    }
    legs.push_back(centre);
  }
  return legs;
}

Points pairLegs(const Points & legs, double max_distance)
{
  std::vector<std::tuple<double, std::size_t, std::size_t>> pairs;
  for (std::size_t i = 0; i < legs.size(); ++i) {
    for (std::size_t j = i + 1; j < legs.size(); ++j) {
      const double d = (legs[i] - legs[j]).norm();
      if (d <= max_distance) {
        pairs.emplace_back(d, i, j);
      }
    }
  }
  std::sort(pairs.begin(), pairs.end());
  std::vector<bool> used(legs.size(), false);
  Points people;
  for (const auto & [d, i, j] : pairs) {
    if (!used[i] && !used[j]) {
      used[i] = used[j] = true;
      people.push_back((legs[i] + legs[j]) / 2.0);
    }
  }
  for (std::size_t i = 0; i < legs.size(); ++i) {
    if (!used[i]) {
      people.push_back(legs[i]);
    }
  }
  return people;
}

Points scanToPoints(
  const std::vector<float> & ranges, double angle_min, double angle_increment, double range_min,
  double range_max, double sensor_x, double sensor_y, double sensor_yaw, double max_range)
{
  const double upper = std::min(range_max, max_range);
  Points points;
  points.reserve(ranges.size());
  for (std::size_t i = 0; i < ranges.size(); ++i) {
    const double r = ranges[i];
    if (!std::isfinite(r) || r <= range_min || r >= upper) {
      continue;
    }
    const double a = angle_min + angle_increment * static_cast<double>(i) + sensor_yaw;
    points.emplace_back(sensor_x + r * std::cos(a), sensor_y + r * std::sin(a));
  }
  return points;
}

void StaticMask::build(
  const std::vector<std::uint8_t> & occupied, int width, int height, double origin_x,
  double origin_y, double resolution, double clearance)
{
  width_ = width;
  height_ = height;
  origin_x_ = origin_x;
  origin_y_ = origin_y;
  resolution_ = resolution;
  mask_.assign(static_cast<std::size_t>(width) * height, 0);
  const int k = std::max(1, static_cast<int>(std::ceil(clearance / resolution)));
  for (int r = 0; r < height; ++r) {
    for (int c = 0; c < width; ++c) {
      if (!occupied[static_cast<std::size_t>(r) * width + c]) {
        continue;
      }
      for (int dr = -k; dr <= k; ++dr) {
        for (int dc = -k; dc <= k; ++dc) {
          const int rr = r + dr, cc = c + dc;
          if (dr * dr + dc * dc <= k * k && rr >= 0 && rr < height && cc >= 0 && cc < width) {
            mask_[static_cast<std::size_t>(rr) * width + cc] = 1;
          }
        }
      }
    }
  }
}

bool StaticMask::blocked(const Point & p) const
{
  const int c = static_cast<int>(std::floor((p.x() - origin_x_) / resolution_));
  const int r = static_cast<int>(std::floor((p.y() - origin_y_) / resolution_));
  if (c < 0 || c >= width_ || r < 0 || r >= height_) {
    return false;
  }
  return mask_[static_cast<std::size_t>(r) * width_ + c] != 0;
}

Points StaticMask::filter(const Points & points) const
{
  if (!ready()) {
    return points;
  }
  Points kept;
  kept.reserve(points.size());
  for (const auto & p : points) {
    if (!blocked(p)) {
      kept.push_back(p);
    }
  }
  return kept;
}

void PeopleTracker::predict(Track & track, double dt) const
{
  Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
  F(0, 2) = F(1, 3) = dt;
  Eigen::Matrix<double, 4, 2> G;
  G << 0.5 * dt * dt, 0.0, 0.0, 0.5 * dt * dt, dt, 0.0, 0.0, dt;
  const double q = params_.accel_noise * params_.accel_noise;
  track.x = F * track.x;
  track.P = F * track.P * F.transpose() + G * G.transpose() * q;
}

void PeopleTracker::correct(Track & track, const Point & z, double stamp) const
{
  Eigen::Matrix<double, 2, 4> H = Eigen::Matrix<double, 2, 4>::Zero();
  H(0, 0) = H(1, 1) = 1.0;
  const Eigen::Matrix2d R =
    Eigen::Matrix2d::Identity() * params_.measurement_noise * params_.measurement_noise;
  const Eigen::Vector2d y = z - H * track.x;
  const Eigen::Matrix2d S = H * track.P * H.transpose() + R;
  const Eigen::Matrix<double, 4, 2> K = track.P * H.transpose() * S.inverse();
  track.x += K * y;
  track.P = (Eigen::Matrix4d::Identity() - K * H) * track.P;
  const double speed = track.x.tail<2>().norm();
  if (speed > params_.max_speed) {
    track.x.tail<2>() *= params_.max_speed / speed;
  }
  track.last_update = stamp;
  track.hits += 1;
  if (track.hits >= params_.confirm_hits) {
    track.confirmed = true;
  }
}

std::vector<Track> PeopleTracker::update(const Points & detections, double stamp)
{
  const double dt = has_time_ ? std::max(0.0, stamp - last_time_) : 0.0;
  has_time_ = true;
  last_time_ = stamp;
  for (auto & track : tracks_) {
    predict(track, dt);
  }
  // Greedy nearest-neighbour association inside the gate.
  std::vector<std::tuple<double, std::size_t, std::size_t>> candidates;
  for (std::size_t ti = 0; ti < tracks_.size(); ++ti) {
    for (std::size_t di = 0; di < detections.size(); ++di) {
      const double d = (tracks_[ti].position() - detections[di]).norm();
      if (d <= params_.gate) {
        candidates.emplace_back(d, ti, di);
      }
    }
  }
  std::sort(candidates.begin(), candidates.end());
  std::vector<bool> track_taken(tracks_.size(), false), det_taken(detections.size(), false);
  for (const auto & [d, ti, di] : candidates) {
    if (track_taken[ti] || det_taken[di]) {
      continue;
    }
    track_taken[ti] = det_taken[di] = true;
    correct(tracks_[ti], detections[di], stamp);
  }
  for (std::size_t di = 0; di < detections.size(); ++di) {
    if (!det_taken[di]) {
      Track t;
      t.id = next_id_++;
      t.x << detections[di].x(), detections[di].y(), 0.0, 0.0;
      t.P = Eigen::Vector4d(0.1, 0.1, 1.0, 1.0).asDiagonal();
      t.last_update = stamp;
      tracks_.push_back(t);
    }
  }
  tracks_.erase(
    std::remove_if(
      tracks_.begin(), tracks_.end(),
      [&](const Track & t) {return stamp - t.last_update > params_.max_coast;}),
    tracks_.end());
  std::vector<Track> confirmed;
  for (const auto & t : tracks_) {
    if (t.confirmed) {
      confirmed.push_back(t);
    }
  }
  return confirmed;
}

}  // namespace people_tracker
