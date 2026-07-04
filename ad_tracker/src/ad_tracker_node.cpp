// [transferable-to-heven_ad]
#include "ad_tracker/gps_tracker.h"

#include <geometry_msgs/Pose2D.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <memory>
#include <string>

namespace ad_tracker
{
class TrackerNode
{
public:
  explicit TrackerNode(ros::NodeHandle& nh)
  {
    std::string csv_path;
    std::string pose_topic;
    std::string status_topic;
    double lookahead = 3.0;
    double target_velocity_kph = 20.0;
    double gain_k = 0.5;
    double k_soft = 1.0;
    double a_lat = 1.5;
    double pid_kp = 0.3;
    double pid_ki = 0.0;
    double pid_kd = 0.01;

    nh.param<std::string>("csv_path", csv_path, "");
    nh.param<std::string>("pose_topic", pose_topic, "/ad_pose_parser/pose");
    nh.param<std::string>("status_topic", status_topic, "/Competition_topic");
    nh.param("lookahead", lookahead, lookahead);
    nh.param("target_velocity_kph", target_velocity_kph, target_velocity_kph);
    nh.param("gain_k", gain_k, gain_k);
    nh.param("k_soft", k_soft, k_soft);
    nh.param("a_lat", a_lat, a_lat);
    nh.param("pid_kp", pid_kp, pid_kp);
    nh.param("pid_ki", pid_ki, pid_ki);
    nh.param("pid_kd", pid_kd, pid_kd);

    if (csv_path.empty()) {
      ROS_FATAL("csv_path parameter is required");
      throw std::runtime_error("csv_path parameter is required");
    }

    path_ = tracker_.Init(csv_path, lookahead, target_velocity_kph, gain_k, k_soft, a_lat,
                          pid_kp, pid_ki, pid_kd);
    command_pub_ = nh.advertise<morai_msgs::CtrlCmd>("command", 1);
    path_pub_ = nh.advertise<nav_msgs::Path>("/ad_tracker/path", 1, true);
    target_pub_ = nh.advertise<visualization_msgs::Marker>("/ad_tracker/target", 1);

    pose_sub_ = nh.subscribe(pose_topic, 1, &TrackerNode::poseCallback, this);
    status_sub_ = nh.subscribe(status_topic, 1, &TrackerNode::statusCallback, this);
    timer_ = nh.createTimer(ros::Duration(0.1), &TrackerNode::timerCallback, this);

    ROS_INFO("ad_tracker ready: %zu waypoints, pose=%s status=%s",
             tracker_.waypoint_count(),
             pose_topic.c_str(),
             status_topic.c_str());
  }

private:
  void poseCallback(const geometry_msgs::Pose2D& msg)
  {
    pose_ = msg;
    has_pose_ = true;
  }

  void statusCallback(const morai_msgs::EgoVehicleStatus& msg)
  {
    current_velocity_kph_ = msg.velocity.x * 3.6;
    has_status_ = true;
  }

  void timerCallback(const ros::TimerEvent&)
  {
    path_.header.stamp = ros::Time::now();
    path_pub_.publish(path_);

    if (!has_pose_) {
      ROS_WARN_THROTTLE(2.0, "ad_tracker waiting for pose");
      return;
    }

    TargetInfo target;
    const double current_velocity_kph = has_status_ ? current_velocity_kph_ : 0.0;
    const auto command = tracker_.Stanley(pose_, current_velocity_kph, target);
    command_pub_.publish(command);
    target_pub_.publish(targetMarker(target));
  }

  visualization_msgs::Marker targetMarker(const TargetInfo& target) const
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "ad_tracker";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = target.pose.pose.position;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.2;
    marker.color.b = 0.1;
    return marker;
  }

  GpsTracker tracker_;
  nav_msgs::Path path_;
  geometry_msgs::Pose2D pose_;
  double current_velocity_kph_ = 0.0;
  bool has_pose_ = false;
  bool has_status_ = false;
  ros::Subscriber pose_sub_;
  ros::Subscriber status_sub_;
  ros::Publisher command_pub_;
  ros::Publisher path_pub_;
  ros::Publisher target_pub_;
  ros::Timer timer_;
};
}  // namespace ad_tracker

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ad_tracker");
  ros::NodeHandle nh("~");

  try {
    ad_tracker::TrackerNode node(nh);
    ros::spin();
  } catch (const std::exception& e) {
    ROS_FATAL("ad_tracker failed: %s", e.what());
    return 1;
  }

  return 0;
}
