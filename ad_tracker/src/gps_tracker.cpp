// [transferable-to-heven_ad]
// governed-Stanley: ad_autotune/scripts/ad_control.py(GovernedStanley+build_profile)와 동등.
//  - windowed monotonic nearest (global nearest는 경로 재접근 구간에서 오점착)
//  - signed cte at nearest segment (cte>0 = 경로 우측 → 좌조향 +)
//  - Stanley 분모 = k_soft + 측정속도[m/s]
//  - arc-length lookahead로 heading preview
//  - 곡률 기반 속도 프로파일(a_lat) + decel/accel 패스
//  - 조향 슬루 제한 120°/s
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
constexpr double kMaxSteerRad = 40.0 * M_PI / 180.0;
constexpr double kSteerRate = 120.0 * M_PI / 180.0;  // rad/s
constexpr double kADecel = 2.5;                      // m/s^2 (차량 물리 한계, 고정)
constexpr double kAAccel = 2.0;
constexpr double kVMinKph = 5.0;
constexpr double kVMaxKph = 60.0;
constexpr int kCurvSmooth = 5;
constexpr std::size_t kNearestWindow = 200;
constexpr double kJumpReseedDist = 20.0;  // m. 이보다 멀면 텔레포트로 보고 global 재시드
constexpr double kLaunchRampSec = 4.0;    // 발진 램프: 5kph→target을 4초에 걸쳐(풀스로틀 발진 이탈 방지)
constexpr double kLaunchRampFromKph = 5.0;

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
  std::vector<double> s;            // 누적 arc length [m]
  std::vector<double> profile_kph;  // 곡률 governed 속도 상한 [kph]
  double lookahead_m = 3.0;
  double target_velocity_kph = 20.0;
  double stanley_gain = 0.5;
  double k_soft = 1.0;
  double a_lat = 1.5;
  double pid_kp = 0.3;
  double pid_ki = 0.0;
  double pid_kd = 0.01;
  double pid_integral = 0.0;
  double pid_prev_error = 0.0;
  ros::Time prev_time;
  std::size_t cur = 0;   // monotonic nearest 인덱스
  bool seeded = false;   // 첫 pose에서 global nearest로 시드
  double prev_steer = 0.0;
  ros::Time first_cmd_time;  // 발진 램프 기준점

  void buildProfile();
  void seedNearest(const geometry_msgs::Pose2D& pose);
};

void GpsTracker::Impl::buildProfile()
{
  const std::size_t n = waypoints.size();
  s.assign(n, 0.0);
  for (std::size_t i = 1; i < n; ++i) {
    s[i] = s[i - 1] +
           std::hypot(waypoints[i].x - waypoints[i - 1].x, waypoints[i].y - waypoints[i - 1].y);
  }

  // 3점 외접원 곡률 (i-2, i, i+2)
  std::vector<double> kappa(n, 0.0);
  for (std::size_t i = 2; i + 2 < n; ++i) {
    const auto& a = waypoints[i - 2];
    const auto& b = waypoints[i];
    const auto& c = waypoints[i + 2];
    const double A = std::hypot(b.x - a.x, b.y - a.y);
    const double B = std::hypot(c.x - b.x, c.y - b.y);
    const double C = std::hypot(c.x - a.x, c.y - a.y);
    if (A * B * C < 1e-9) {
      continue;
    }
    const double area = std::abs((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y)) / 2.0;
    kappa[i] = 4.0 * area / (A * B * C);
  }

  // 보수적 스무딩(윈도우 max) 후 v = sqrt(a_lat/kappa)
  const double v_min = kVMinKph / 3.6;
  const double v_max = kVMaxKph / 3.6;
  std::vector<double> v(n, v_max);
  for (std::size_t i = 0; i < n; ++i) {
    const std::size_t lo = i > static_cast<std::size_t>(kCurvSmooth) ? i - kCurvSmooth : 0;
    const std::size_t hi = std::min(n - 1, i + kCurvSmooth);
    double k_worst = 0.0;
    for (std::size_t j = lo; j <= hi; ++j) {
      k_worst = std::max(k_worst, kappa[j]);
    }
    v[i] = clamp(std::sqrt(a_lat / std::max(k_worst, 1e-6)), v_min, v_max);
  }
  for (std::size_t i = n - 1; i-- > 0;) {  // backward: 감속 한계
    const double ds = s[i + 1] - s[i];
    v[i] = std::min(v[i], std::sqrt(v[i + 1] * v[i + 1] + 2.0 * kADecel * ds));
  }
  for (std::size_t i = 1; i < n; ++i) {    // forward: 가속 한계
    const double ds = s[i] - s[i - 1];
    v[i] = std::min(v[i], std::sqrt(v[i - 1] * v[i - 1] + 2.0 * kAAccel * ds));
  }

  profile_kph.assign(n, kVMaxKph);
  for (std::size_t i = 0; i < n; ++i) {
    profile_kph[i] = v[i] * 3.6;
  }
}

void GpsTracker::Impl::seedNearest(const geometry_msgs::Pose2D& pose)
{
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < waypoints.size(); ++i) {
    const double d = std::hypot(waypoints[i].x - pose.x, waypoints[i].y - pose.y);
    if (d < best) {
      best = d;
      cur = i;
    }
  }
  seeded = true;
}

GpsTracker::GpsTracker() : impl_(new Impl) {}
GpsTracker::~GpsTracker() = default;

nav_msgs::Path GpsTracker::Init(const std::string& pathfile,
                                double lookahead_m,
                                double target_velocity_kph,
                                double stanley_gain,
                                double k_soft,
                                double a_lat,
                                double pid_kp,
                                double pid_ki,
                                double pid_kd)
{
  impl_->lookahead_m = lookahead_m;
  impl_->target_velocity_kph = target_velocity_kph;
  impl_->stanley_gain = stanley_gain;
  impl_->k_soft = k_soft;
  impl_->a_lat = a_lat;
  impl_->pid_kp = pid_kp;
  impl_->pid_ki = pid_ki;
  impl_->pid_kd = pid_kd;
  impl_->pid_integral = 0.0;
  impl_->pid_prev_error = 0.0;
  impl_->prev_time = ros::Time();
  impl_->cur = 0;
  impl_->seeded = false;
  impl_->prev_steer = 0.0;
  impl_->first_cmd_time = ros::Time();

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
    bool bad_line = false;
    while (std::getline(ss, item, ',')) {
      if (item.empty()) {
        continue;
      }
      try {
        values.push_back(std::stod(item));
      } catch (const std::exception&) {
        bad_line = true;  // 헤더행(x,y,...) 등 숫자 아님 → 라인 스킵
        break;
      }
    }

    if (bad_line || values.size() < 2) {
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

  if (impl_->waypoints.size() < 5) {
    throw std::runtime_error("path csv must contain at least five waypoints: " + pathfile);
  }

  impl_->buildProfile();
  ROS_INFO("ad_tracker loaded %zu waypoints from %s (lookahead=%.2f v=%.1f k=%.2f k_soft=%.2f a_lat=%.2f)",
           impl_->waypoints.size(), pathfile.c_str(), lookahead_m, target_velocity_kph,
           stanley_gain, k_soft, a_lat);
  return impl_->path;
}

morai_msgs::CtrlCmd GpsTracker::Stanley(const geometry_msgs::Pose2D& pose,
                                        double current_velocity_kph,
                                        TargetInfo& target)
{
  const auto& wp = impl_->waypoints;
  const std::size_t n = wp.size();

  if (!impl_->seeded) {
    impl_->seedNearest(pose);
  }

  // windowed monotonic nearest
  std::size_t near = impl_->cur;
  double best_d2 = std::numeric_limits<double>::infinity();
  const std::size_t hi = std::min(n, impl_->cur + kNearestWindow);
  for (std::size_t i = impl_->cur; i < hi; ++i) {
    const double dx = wp[i].x - pose.x;
    const double dy = wp[i].y - pose.y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      near = i;
    }
  }
  if (best_d2 > kJumpReseedDist * kJumpReseedDist) {  // 텔레포트/리셋 감지 → 재시드
    impl_->seedNearest(pose);
    near = impl_->cur;
    impl_->prev_steer = 0.0;
    impl_->first_cmd_time = ros::Time();
    ROS_WARN("ad_tracker: pose jump detected, reseeded nearest=%zu", near);
  }
  impl_->cur = near;

  // signed cte at nearest segment (cte>0 = 경로 우측)
  std::size_t a_i = near, b_i = near + 1;
  if (b_i >= n) {
    a_i = near - 1;
    b_i = near;
  }
  const double dx = wp[b_i].x - wp[a_i].x;
  const double dy = wp[b_i].y - wp[a_i].y;
  const double seg_len = std::max(std::hypot(dx, dy), 1e-6);
  const double cross_track_error =
      (dy * (pose.x - wp[a_i].x) - dx * (pose.y - wp[a_i].y)) / seg_len;

  // heading preview: arc-length lookahead 앞 지점의 접선
  std::size_t pv = near;
  while (pv + 1 < n && impl_->s[pv] - impl_->s[near] < impl_->lookahead_m) {
    ++pv;
  }
  std::size_t p_i = pv, q_i = pv + 1;
  if (q_i >= n) {
    p_i = pv - 1;
    q_i = pv;
  }
  const double path_theta = std::atan2(wp[q_i].y - wp[p_i].y, wp[q_i].x - wp[p_i].x);

  const double pose_theta = headingToRad(pose.theta);
  const double heading_error = normalizeAngle(path_theta - pose_theta);
  const double v_mps = std::max(current_velocity_kph, 0.0) / 3.6;
  double steering_angle =
      heading_error + std::atan2(impl_->stanley_gain * cross_track_error, impl_->k_soft + v_mps);
  steering_angle = clamp(steering_angle, -kMaxSteerRad, kMaxSteerRad);

  const ros::Time now = ros::Time::now();
  double dt = 0.1;
  if (!impl_->prev_time.isZero()) {
    dt = std::max((now - impl_->prev_time).toSec(), 1e-3);
  }

  // 조향 슬루 제한
  const double ds_max = kSteerRate * dt;
  steering_angle = clamp(steering_angle, impl_->prev_steer - ds_max, impl_->prev_steer + ds_max);
  impl_->prev_steer = steering_angle;

  // 종방향: 프로파일 governed 목표속도에 PID (+ 발진 램프)
  if (impl_->first_cmd_time.isZero()) {
    impl_->first_cmd_time = now;
  }
  const double t_run = (now - impl_->first_cmd_time).toSec();
  const double ramp = std::min(t_run / kLaunchRampSec, 1.0);
  const double ramp_target =
      std::min(impl_->target_velocity_kph,
               kLaunchRampFromKph + (impl_->target_velocity_kph - kLaunchRampFromKph) * ramp);
  const double v_ref = std::min(ramp_target, impl_->profile_kph[near]);
  const double speed_error = v_ref - current_velocity_kph;
  double accel_cmd = impl_->pid_kp * speed_error;
  if (!impl_->prev_time.isZero()) {
    impl_->pid_integral = clamp(impl_->pid_integral + speed_error * dt, -100.0, 100.0);
    const double derivative = (speed_error - impl_->pid_prev_error) / dt;
    accel_cmd += impl_->pid_ki * impl_->pid_integral + impl_->pid_kd * derivative;
  }
  impl_->pid_prev_error = speed_error;
  impl_->prev_time = now;

  morai_msgs::CtrlCmd command;
  command.longlCmdType = 1;
  command.accel = clamp(accel_cmd, 0.0, 1.0);
  command.brake = speed_error < -1.0 ? clamp(-accel_cmd, 0.0, 1.0) : 0.0;
  command.front_steer = steering_angle;
  command.steering = steering_angle / kMaxSteerRad;  // 정규화 [-1,1]. MORAI 25.S4는 steering 필드를 봄
  command.rear_steer = 0.0;
  command.velocity = v_ref;
  command.acceleration = 0.0;

  target.index = pv;
  target.distance = std::hypot(wp[pv].x - pose.x, wp[pv].y - pose.y);
  target.pose.header.frame_id = "map";
  target.pose.pose.position.x = wp[pv].x;
  target.pose.pose.position.y = wp[pv].y;

  ROS_INFO_THROTTLE(1.0,
                    "ad_tracker near=%zu cte=%.2f head_err=%.1fdeg v_ref=%.1f steer=%.3frad accel=%.2f",
                    near,
                    cross_track_error,
                    heading_error * 180.0 / M_PI,
                    v_ref,
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
