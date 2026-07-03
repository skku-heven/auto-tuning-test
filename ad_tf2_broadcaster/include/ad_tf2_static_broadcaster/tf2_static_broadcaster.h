// [transferable-to-heven_ad]
#ifndef AD_TF2_STATIC_BROADCASTER_TF2_STATIC_BROADCASTER_H_
#define AD_TF2_STATIC_BROADCASTER_TF2_STATIC_BROADCASTER_H_

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <cmath>

namespace ad_tf2_static_broadcaster
{
    struct tf_data{
        float t_x;
        float t_y;
        float t_z;
        float r_x;
        float r_y;
        float r_z;
        std::string parent_id;
        std::string frame_id;
    };

    class TF2StaticBroadcaster
    {
    public:
        TF2StaticBroadcaster();
        ~TF2StaticBroadcaster();

        // set param, subscriber
        void Init(ros::NodeHandle &nh);
        // Publishing tf_static
        void SpinOnce();
        // set each tf frame
        void SetBroadcaster(ros::NodeHandle &nh, std::string frame_id, geometry_msgs::TransformStamped &data);
        // set time stamp and pub
        void Broadcasting(geometry_msgs::TransformStamped &frame);
        void TFCallback(const tf2_msgs::TFMessage::Ptr &tf);

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
}
#endif // AD_TF2_STATIC_BROADCASTER_TF2_STATIC_BROADCASTER_H_
