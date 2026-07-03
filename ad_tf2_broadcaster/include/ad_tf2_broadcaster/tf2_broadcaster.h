// [transferable-to-heven_ad]
#ifndef AD_TF2_BROADCASTER_AD_TF2_BROADCASTER_TF2_BROADCASTER_H_
#define AD_TF2_BROADCASTER_AD_TF2_BROADCASTER_TF2_BROADCASTER_H_

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <cmath>

namespace ad_tf2_broadcaster
{
    struct tf_data{
        float t_x = 0;
        float t_y = 0;
        float t_z = 0;
        float r_x = 0;
        float r_y = 0;
        float r_z = 0;
        std::string parent_id;
        std::string frame_id;
    };

    class TF2Broadcaster
    {
    public:
        TF2Broadcaster();
        ~TF2Broadcaster();

        // set param, subscriber
        void Init(ros::NodeHandle &nh);
        // subscribe car position, heading data
        void CarPositionCallback(const geometry_msgs::Pose2D::Ptr &car);
        // subscribe tf time stamp
        void TFCallback(const tf2_msgs::TFMessage::Ptr &tf);
        // set each tf frame
        geometry_msgs::TransformStamped SetBroadcaster(tf_data data);
        // set time stamp and pub
        void Broadcasting(geometry_msgs::TransformStamped frame);

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
}
#endif // AD_TF2_BROADCASTER_AD_TF2_BROADCASTER_TF2_BROADCASTER_H_
