// [transferable-to-heven_ad]
#ifndef AD_POSE_PARSER_POSE_PARSER_H_
#define AD_POSE_PARSER_POSE_PARSER_H_

#include <morai_msgs/GPSMessage.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>
#include <geographic_msgs/GeoPoint.h>

#include <ros/ros.h>
#include <memory>


namespace ad_pose_parser
{
    class PoseParser
    {
    public:
        PoseParser();
        ~PoseParser();

        void Init(ros::NodeHandle &nh);
        void GPSCallback(const morai_msgs::GPSMessage &msg);
        void HeadingCallback(const sensor_msgs::Imu &msg);
        void MagCallback(const geometry_msgs::Vector3Stamped &msg);
        void Run();

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
        geometry_msgs::Pose2D ConvertGps2XYYaw(double lon, double lat, double h, double x0, double y0, float heading);
        float CalHeading(const geometry_msgs::Quaternion &msg);
        void SetOriginOrientation();
    };
}

#endif // AD_POSE_PARSER_POSE_PARSER_H_