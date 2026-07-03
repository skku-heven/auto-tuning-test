// [transferable-to-heven_ad]
#include "ad_tf2_broadcaster/tf2_broadcaster.h"

namespace ad_tf2_broadcaster
{
    struct TF2Broadcaster::Impl
    {           
        // tf2 broadcaster
        tf2_ros::TransformBroadcaster br;
        // subscriber
        // subscribe car position, heading data
        ros::Subscriber sub_state;
        // subscribe car steering
        ros::Subscriber sub_feedback;
        // subscribe tf time stamp
        ros::Subscriber sub_tf;

        ///
        //!@brief tf stamp
        ///
        geometry_msgs::TransformStamped odom;

        ///
        //!@brief parameter about frame position
        ///
        tf_data odom_data;

        bool is_synch;
        std::string synch_topic;
        ros::Time timestamp;
    };

    TF2Broadcaster::TF2Broadcaster() : impl_(new Impl) {}
    TF2Broadcaster::~TF2Broadcaster() {}

    void TF2Broadcaster::Init(ros::NodeHandle &nh)
    {
        // get param about frame_id of odom
        ROS_ASSERT(nh.getParam("odom/parent_id", impl_->odom_data.parent_id));
        ROS_ASSERT(nh.getParam("odom/frame_id", impl_->odom_data.frame_id));

        // get param wheter sync other tf stamp(time)
        ROS_ASSERT(nh.getParam("synch", impl_->is_synch));
        ROS_ASSERT(nh.getParam("synch_topic", impl_->synch_topic));

        // subscribe car position, heading data
        impl_->sub_state = nh.subscribe("StateEstimated", 3, &TF2Broadcaster::CarPositionCallback, this);

        if(impl_->is_synch) impl_->sub_tf = nh.subscribe("/tf", 50, &TF2Broadcaster::TFCallback, this);

        return;
    }

    void TF2Broadcaster::CarPositionCallback(const geometry_msgs::Pose2D::Ptr &car)
    {
        // set rotation of odom from camera by initial car heading (unit: rad)
        impl_->odom_data.r_x = 0;
        impl_->odom_data.r_y = 0;
        impl_->odom_data.r_z = (car->theta)* M_PI / 180;
        // set position of odom from camera by initial car position(unit: m)
        impl_->odom_data.t_x = car->x;
        impl_->odom_data.t_y = car->y;
        impl_->odom_data.t_z = 0;
        // set odom frame
        impl_->odom = SetBroadcaster(impl_->odom_data); 

        // broadcasting tf
        Broadcasting(impl_->odom);

        // finish callback function
        return;
    }

    geometry_msgs::TransformStamped TF2Broadcaster::SetBroadcaster(tf_data data){

        geometry_msgs::TransformStamped temp;
        tf2::Quaternion q;
        q.setRPY(data.r_x, data.r_y, data.r_z);
        // set camera
        // set camera frame name
        temp.header.frame_id = data.parent_id;
        temp.child_frame_id = data.frame_id;
        // set position of camera from world (unit: m)
        temp.transform.translation.x = data.t_x;
        temp.transform.translation.y = data.t_y;
        temp.transform.translation.z = data.t_z;
        // set rotation of camera from world (unit: rad)
        temp.transform.rotation.x = q.x();
        temp.transform.rotation.y = q.y();
        temp.transform.rotation.z = q.z();
        temp.transform.rotation.w = q.w();

        return temp;
    }
    void TF2Broadcaster::Broadcasting(geometry_msgs::TransformStamped frame){
        // set header time stamp
        frame.header.stamp = ros::Time::now();
        // broadcasting
        impl_->br.sendTransform(frame);
        return;
    }

    void TF2Broadcaster::TFCallback(const tf2_msgs::TFMessage::Ptr &tf){

        for(auto &it : tf->transforms){
            if(it.child_frame_id == impl_->synch_topic){
                impl_->timestamp = it.header.stamp;
                return;
            }else continue;
        }
        return;
    }

}
