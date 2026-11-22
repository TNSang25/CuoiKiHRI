// LiDAR leg tracker: /scan (+ static /map) -> /people (social_nav_msgs/People).
//
// Drop-in replacement for the ground-truth people_adapter: same topic and message, so the social
// costmap layer and the MPPI critics run unchanged on tracked instead of ground-truth people.
// Also publishes
//   /scan_filtered         /scan without the returns of tracked people (for costmap obstacle
//                          layers, so people are moving agents, not static obstacles)
//   /leg_tracker/markers   detected legs and tracks for RViz
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "social_nav_msgs/msg/people.hpp"
#include "people_tracker/tracking.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace people_tracker
{

namespace
{
struct Pose2
{
  double x{0.0}, y{0.0}, yaw{0.0};
};

double yawOf(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

Pose2 compose(const Pose2 & a, const Pose2 & b)
{
  return {a.x + std::cos(a.yaw) * b.x - std::sin(a.yaw) * b.y,
    a.y + std::sin(a.yaw) * b.x + std::cos(a.yaw) * b.y, a.yaw + b.yaw};
}
}  // namespace

class LegTrackerNode : public rclcpp::Node
{
public:
  LegTrackerNode()
  : Node("people_tracker")
  {
    fixed_frame_ = declare_parameter<std::string>("fixed_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    max_range_ = declare_parameter<double>("max_range", 6.0);
    map_clearance_ = declare_parameter<double>("map_clearance", 0.15);
    person_clear_radius_ = declare_parameter<double>("person_clear_radius", 0.4);
    legs_.cluster_gap = declare_parameter<double>("cluster_gap", legs_.cluster_gap);
    legs_.min_points = declare_parameter<int>("min_points", legs_.min_points);
    legs_.min_width = declare_parameter<double>("min_leg_width", legs_.min_width);
    legs_.max_width = declare_parameter<double>("max_leg_width", legs_.max_width);
    legs_.leg_radius = declare_parameter<double>("leg_radius", legs_.leg_radius);
    legs_.max_pair_distance = declare_parameter<double>("max_pair_distance", legs_.max_pair_distance);
    TrackerParams tp;
    tp.gate = declare_parameter<double>("gate", tp.gate);
    tp.accel_noise = declare_parameter<double>("accel_noise", tp.accel_noise);
    tp.measurement_noise = declare_parameter<double>("measurement_noise", tp.measurement_noise);
    tp.confirm_hits = declare_parameter<int>("confirm_hits", tp.confirm_hits);
    tp.max_coast = declare_parameter<double>("max_coast", tp.max_coast);
    tracker_ = PeopleTracker(tp);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    people_pub_ = create_publisher<social_nav_msgs::msg::People>(
      declare_parameter<std::string>("people_topic", "people"), 5);
    filtered_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
      declare_parameter<std::string>("filtered_scan_topic", "scan_filtered"), rclcpp::SensorDataQoS());
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("leg_tracker/markers", 5);

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", rclcpp::QoS(1).transient_local(),
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {onMap(*msg);});
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 50, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {onOdom(*msg);});
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {onScan(*msg);});
  }

private:
  struct OdomSample
  {
    double t;
    Pose2 pose;
  };

  void onMap(const nav_msgs::msg::OccupancyGrid & msg)
  {
    std::vector<std::uint8_t> occupied(msg.data.size());
    for (std::size_t i = 0; i < msg.data.size(); ++i) {
      occupied[i] = msg.data[i] >= 65 ? 1 : 0;
    }
    mask_.build(
      occupied, static_cast<int>(msg.info.width), static_cast<int>(msg.info.height),
      msg.info.origin.position.x, msg.info.origin.position.y, msg.info.resolution, map_clearance_);
  }

  void onOdom(const nav_msgs::msg::Odometry & msg)
  {
    odom_frame_ = msg.header.frame_id.empty() ? "odom" : msg.header.frame_id;
    odom_.push_back({rclcpp::Time(msg.header.stamp).seconds(),
        {msg.pose.pose.position.x, msg.pose.pose.position.y, yawOf(msg.pose.pose.orientation)}});
    while (odom_.size() > 400) {
      odom_.pop_front();
    }
  }

  /// Robot pose in the odom frame at time t (linear interpolation of /odom).
  bool odomAt(double t, Pose2 & out) const
  {
    if (odom_.empty() || t < odom_.front().t - 0.05 || t > odom_.back().t + 0.1) {
      return false;
    }
    auto it = std::lower_bound(
      odom_.begin(), odom_.end(), t, [](const OdomSample & s, double v) {return s.t < v;});
    if (it == odom_.begin() || it == odom_.end()) {
      out = (it == odom_.end() ? odom_.back() : odom_.front()).pose;
      return true;
    }
    const auto & b = *it;
    const auto & a = *(it - 1);
    const double w = b.t > a.t ? (t - a.t) / (b.t - a.t) : 0.0;
    const double dyaw = std::atan2(std::sin(b.pose.yaw - a.pose.yaw), std::cos(b.pose.yaw - a.pose.yaw));
    out = {a.pose.x + w * (b.pose.x - a.pose.x), a.pose.y + w * (b.pose.y - a.pose.y),
      a.pose.yaw + w * dyaw};
    return true;
  }

  bool staticPose(const std::string & target, const std::string & source, Pose2 & out)
  {
    try {
      const auto t = tf_buffer_->lookupTransform(target, source, tf2::TimePointZero);
      out = {t.transform.translation.x, t.transform.translation.y, yawOf(t.transform.rotation)};
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Transform unavailable: %s", ex.what());
      return false;
    }
  }

  void onScan(const sensor_msgs::msg::LaserScan & scan)
  {
    // The sensor pose at the scan's stamp comes from /odom (interpolated) composed with the
    // static map->odom and base->lidar transforms, so it does not lag behind the scan.
    const double stamp = rclcpp::Time(scan.header.stamp).seconds();
    Pose2 robot, map_odom, base_lidar;
    if (!odomAt(stamp, robot)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No odometry at scan time");
      return;
    }
    if (!staticPose(fixed_frame_, odom_frame_, map_odom) ||
      !staticPose(base_frame_, scan.header.frame_id, base_lidar))
    {
      return;
    }
    const Pose2 sensor = compose(compose(map_odom, robot), base_lidar);

    const Points points = mask_.filter(scanToPoints(
        scan.ranges, scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max,
        sensor.x, sensor.y, sensor.yaw, max_range_));
    const Points legs = detectLegs(points, Point(sensor.x, sensor.y), legs_);
    const auto tracks = tracker_.update(pairLegs(legs, legs_.max_pair_distance), stamp);

    social_nav_msgs::msg::People out;
    out.header.stamp = scan.header.stamp;
    out.header.frame_id = fixed_frame_;
    for (const auto & track : tracks) {
      social_nav_msgs::msg::Person person;
      person.name = std::to_string(track.id);
      person.position.x = track.position().x();
      person.position.y = track.position().y();
      person.velocity.x = track.velocity().x();
      person.velocity.y = track.velocity().y();
      person.reliability = std::min(1.0, track.hits / 10.0);
      out.people.push_back(person);
    }
    people_pub_->publish(out);
    publishFilteredScan(scan, sensor, tracks);
    if (marker_pub_->get_subscription_count() > 0) {
      publishMarkers(scan.header.stamp, legs, tracks);
    }
  }

  void publishFilteredScan(
    const sensor_msgs::msg::LaserScan & scan, const Pose2 & sensor, const std::vector<Track> & tracks)
  {
    auto filtered = scan;
    const double r2 = person_clear_radius_ * person_clear_radius_;
    for (std::size_t i = 0; i < filtered.ranges.size() && !tracks.empty(); ++i) {
      const double r = filtered.ranges[i];
      if (!std::isfinite(r)) {
        continue;
      }
      const double a = scan.angle_min + scan.angle_increment * static_cast<double>(i) + sensor.yaw;
      const double x = sensor.x + r * std::cos(a), y = sensor.y + r * std::sin(a);
      for (const auto & track : tracks) {
        const double dx = x - track.position().x(), dy = y - track.position().y();
        if (dx * dx + dy * dy < r2) {
          // "No return": the obstacle layer (inf_is_valid) clears along this ray.
          filtered.ranges[i] = std::numeric_limits<float>::infinity();
          break;
        }
      }
    }
    filtered_pub_->publish(filtered);
  }

  void publishMarkers(
    const builtin_interfaces::msg::Time & stamp, const Points & legs, const std::vector<Track> & tracks)
  {
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);

    visualization_msgs::msg::Marker leg_marker;
    leg_marker.header.frame_id = fixed_frame_;
    leg_marker.header.stamp = stamp;
    leg_marker.ns = "legs";
    leg_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    leg_marker.scale.x = leg_marker.scale.y = leg_marker.scale.z = 0.13;
    leg_marker.color.r = 1.0;
    leg_marker.color.g = 0.85;
    leg_marker.color.b = 0.1;
    leg_marker.color.a = 0.9;
    leg_marker.pose.orientation.w = 1.0;
    for (const auto & leg : legs) {
      geometry_msgs::msg::Point p;
      p.x = leg.x();
      p.y = leg.y();
      p.z = 0.2;
      leg_marker.points.push_back(p);
    }
    array.markers.push_back(leg_marker);

    for (const auto & track : tracks) {
      visualization_msgs::msg::Marker arrow;
      arrow.header = leg_marker.header;
      arrow.ns = "tracks";
      arrow.id = track.id;
      arrow.type = visualization_msgs::msg::Marker::ARROW;
      geometry_msgs::msg::Point a, b;
      a.x = track.position().x();
      a.y = track.position().y();
      a.z = 0.3;
      b.x = a.x + track.velocity().x();
      b.y = a.y + track.velocity().y();
      b.z = 0.3;
      arrow.points = {a, b};
      arrow.scale.x = 0.05;
      arrow.scale.y = arrow.scale.z = 0.12;
      arrow.color.g = 0.9;
      arrow.color.b = 0.9;
      arrow.color.a = 1.0;
      array.markers.push_back(arrow);

      visualization_msgs::msg::Marker label;
      label.header = leg_marker.header;
      label.ns = "track_ids";
      label.id = track.id;
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.pose.position.x = a.x;
      label.pose.position.y = a.y;
      label.pose.position.z = 0.7;
      label.pose.orientation.w = 1.0;
      label.scale.z = 0.3;
      label.color.r = label.color.g = label.color.b = label.color.a = 1.0;
      label.text = "T" + std::to_string(track.id);
      array.markers.push_back(label);
    }
    marker_pub_->publish(array);
  }

  std::string fixed_frame_, base_frame_, odom_frame_{"odom"};
  double max_range_, map_clearance_, person_clear_radius_;
  LegParams legs_;
  PeopleTracker tracker_;
  StaticMask mask_;
  std::deque<OdomSample> odom_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<social_nav_msgs::msg::People>::SharedPtr people_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr filtered_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};

}  // namespace people_tracker

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<people_tracker::LegTrackerNode>());
  rclcpp::shutdown();
  return 0;
}
