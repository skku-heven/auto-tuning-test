// [transferable-to-heven_ad]
#pragma once

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <morai_msgs/CtrlCmd.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <memory>
#include <string>
#include <vector>

namespace ad_tracker
{
struct Waypoint
{
  double x = 0.0;
  double y = 0.0;
  double heading_rad = 0.0;
  bool has_heading = false;
};

struct TargetInfo
{
  geometry_msgs::PoseStamped pose;
  std::size_t index = 0;
  double distance = 0.0;
};

class GpsTracker
{
public:
  GpsTracker();
  ~GpsTracker();

  nav_msgs::Path Init(const std::string& pathfile,
                      double lookahead_m,
                      double target_velocity_kph,
                      double stanley_gain,
                      double k_soft,
                      double a_lat,
                      double pid_kp,
                      double pid_ki,
                      double pid_kd);

  morai_msgs::CtrlCmd Stanley(const geometry_msgs::Pose2D& pose,
                              double current_velocity_kph,
                              TargetInfo& target);

  const nav_msgs::Path& path() const;
  std::size_t waypoint_count() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace ad_tracker
