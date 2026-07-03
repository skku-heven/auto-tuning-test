// [transferable-to-heven_ad]
#include "ad_tracker/gps_tracker.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace ad_tracker
{
namespace
{
double normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double headingToRad(double heading)
{
  if (std::abs(heading) > 2.0 * M_PI) {
    return heading * M_PI / 180.0;
  }
  return heading;
}

double clamp(double value, double low, double high)
{
  return std::max(low, std::min(value, high));
}
}  // namespace

struct GpsTracker::Impl
{
  std::vector<Waypoint> waypoints;
  nav_msgs::Path path;
  double lookahead_m = 3.0;
  double target_velocity_kph = 20.0;
  double stanley_gain = 0.5;
  double pid_kp = 0.3;
  double pid_ki = 0.0;
  double pid_kd = 0.01;
  double pid_integral = 0.0;
  double pid_prev_error = 0.0;
  ros::Time pid_prev_time;
};

GpsTracker::GpsTracker() : impl_(new Impl) {}
GpsTracker::~GpsTracker() = default;

nav_msgs::Path GpsTracker::Init(const std::string& pathfile,
                                double lookahead_m,
                                double target_velocity_kph,
                                double stanley_gain,
                                double pid_kp,
                                double pid_ki,
                                double pid_kd)
{
  impl_->lookahead_m = lookahead_m;
  impl_->target_velocity_kph = target_velocity_kph;
  impl_->stanley_gain = stanley_gain;
  impl_->pid_kp = pid_kp;
  impl_->pid_ki = pid_ki;
  impl_->pid_kd = pid_kd;
  impl_->pid_integral = 0.0;
  impl_->pid_prev_error = 0.0;
  impl_->pid_prev_time = ros::Time();

  std::ifstream file(pathfile);
  if (!file.is_open()) {
    throw std::runtime_error("failed to open path csv: " + pathfile);
  }

  impl_->waypoints.clear();
  impl_->path.poses.clear();
  impl_->path.header.frame_id = "map";

  std::string line;
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }

    std::stringstream ss(line);
    std::string item;
    std::vector<double> values;
    while (std::getline(ss, item, ',')) {
      if (!item.empty()) {
        values.push_back(std::stod(item));
      }
    }

    if (values.size() < 2) {
      continue;
    }

    Waypoint waypoint;
    waypoint.x = values[0];
    waypoint.y = values[1];
    if (values.size() >= 3) {
      waypoint.heading_rad = headingToRad(values[2]);
      waypoint.has_heading = true;
    }
    impl_->waypoints.push_back(waypoint);

    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = waypoint.x;
    pose.pose.position.y = waypoint.y;
    impl_->path.poses.push_back(pose);
  }

  if (impl_->waypoints.size() < 2) {
    throw std::runtime_error("path csv must contain at least two waypoints: " + pathfile);
  }

  ROS_INFO("ad_tracker loaded %zu waypoints from %s", impl_->waypoints.size(), pathfile.c_str());
  return impl_->path;
}

TargetInfo GpsTracker::FindTarget(const geometry_msgs::Pose2D& pose) const
{
  TargetInfo nearest;
  nearest.distance = std::numeric_limits<double>::infinity();

  for (std::size_t i = 0; i < impl_->waypoints.size(); ++i) {
    const auto& waypoint = impl_->waypoints[i];
    const double distance = std::hypot(waypoint.x - pose.x, waypoint.y - pose.y);
    if (distance < nearest.distance) {
      nearest.index = i;
      nearest.distance = distance;
    }
  }

  std::size_t target_index = nearest.index;
  double target_distance = nearest.distance;
  while (target_distance < impl_->lookahead_m) {
    target_index = (target_index + 1) % impl_->waypoints.size();
    const auto& waypoint = impl_->waypoints[target_index];
    target_distance = std::hypot(waypoint.x - pose.x, waypoint.y - pose.y);
    if (target_index == nearest.index) {
      break;
    }
  }

  const auto& target_waypoint = impl_->waypoints[target_index];
  TargetInfo target;
  target.index = target_index;
  target.distance = target_distance;
  target.pose.header.frame_id = "map";
  target.pose.pose.position.x = target_waypoint.x;
  target.pose.pose.position.y = target_waypoint.y;
  return target;
}

morai_msgs::CtrlCmd GpsTracker::Stanley(const geometry_msgs::Pose2D& pose,
                                        double current_velocity_kph,
                                        TargetInfo& target)
{
  target = FindTarget(pose);

  double path_theta = 0.0;
  const auto& target_waypoint = impl_->waypoints[target.index];
  if (target_waypoint.has_heading) {
    path_theta = target_waypoint.heading_rad;
  } else if (target.index + 1 < impl_->waypoints.size()) {
    const auto& curr = impl_->waypoints[target.index];
    const auto& next = impl_->waypoints[target.index + 1];
    path_theta = std::atan2(next.y - curr.y, next.x - curr.x);
  } else {
    const auto& prev = impl_->waypoints[target.index - 1];
    const auto& curr = impl_->waypoints[target.index];
    path_theta = std::atan2(curr.y - prev.y, curr.x - prev.x);
  }

  const double pose_theta = headingToRad(pose.theta);
  const double heading_error = normalizeAngle(path_theta - pose_theta);
  const double target_theta = std::atan2(target.pose.pose.position.y - pose.y,
                                         target.pose.pose.position.x - pose.x) -
                              path_theta;
  const double cross_track_error = target.distance * std::sin(target_theta);
  const double target_velocity_mps = std::max(impl_->target_velocity_kph / 3.6, 0.1);
  const double steering_angle =
      heading_error + std::atan2(impl_->stanley_gain * cross_track_error, target_velocity_mps);

  const ros::Time now = ros::Time::now();
  const double speed_error = impl_->target_velocity_kph - current_velocity_kph;
  double accel_cmd = impl_->pid_kp * speed_error;

  if (!impl_->pid_prev_time.isZero()) {
    const double dt = std::max((now - impl_->pid_prev_time).toSec(), 1e-3);
    impl_->pid_integral += speed_error * dt;
    impl_->pid_integral = clamp(impl_->pid_integral, -100.0, 100.0);
    const double derivative = (speed_error - impl_->pid_prev_error) / dt;
    accel_cmd += impl_->pid_ki * impl_->pid_integral + impl_->pid_kd * derivative;
  }

  impl_->pid_prev_error = speed_error;
  impl_->pid_prev_time = now;

  const double max_steer_rad = 40.0 * M_PI / 180.0;

  morai_msgs::CtrlCmd command;
  command.longlCmdType = 1;
  command.accel = clamp(accel_cmd, 0.0, 1.0);
  command.brake = speed_error < -1.0 ? clamp(-accel_cmd, 0.0, 1.0) : 0.0;
  const double front = clamp(steering_angle, -max_steer_rad, max_steer_rad);
  command.front_steer = front;
  command.steering = front / max_steer_rad;  // 정규화 [-1,1]. MORAI 25.S4는 steering 필드를 봄(front_steer 무시)
  command.rear_steer = 0.0;
  command.velocity = impl_->target_velocity_kph;
  command.acceleration = 0.0;

  ROS_INFO_THROTTLE(1.0,
                    "ad_tracker target=%zu dist=%.2f cte=%.2f heading_err=%.2fdeg steer=%.3frad accel=%.3f",
                    target.index,
                    target.distance,
                    cross_track_error,
                    heading_error * 180.0 / M_PI,
                    command.front_steer,
                    command.accel);
  return command;
}

const nav_msgs::Path& GpsTracker::path() const
{
  return impl_->path;
}

std::size_t GpsTracker::waypoint_count() const
{
  return impl_->waypoints.size();
}
}  // namespace ad_tracker
