// [transferable-to-heven_ad]
#include "ad_tf2_static_broadcaster/tf2_static_broadcaster.h"

namespace ad_tf2_static_broadcaster
{
    struct TF2StaticBroadcaster::Impl
    {   
        // tf2 broadcaster
        tf2_ros::TransformBroadcaster br;
        // subscriber
        ros::Subscriber sub_tf;

        ///
        //!@brief tf stamp
        ///
        geometry_msgs::TransformStamped map;
        geometry_msgs::TransformStamped base_link;
        geometry_msgs::TransformStamped velodyne1;
        geometry_msgs::TransformStamped velodyne2;
        geometry_msgs::TransformStamped carla;
        geometry_msgs::TransformStamped imu;


        // TransformStamped list
        std::vector<geometry_msgs::TransformStamped> stamp_list;

        bool is_synch;
        std::string synch_topic;
        ros::Time timestamp;
    };

    TF2StaticBroadcaster::TF2StaticBroadcaster() : impl_(new Impl) {}
    TF2StaticBroadcaster::~TF2StaticBroadcaster() {}

    void TF2StaticBroadcaster::Init(ros::NodeHandle &nh)
    {
        ROS_ASSERT(nh.getParam("synch", impl_->is_synch));
        ROS_ASSERT(nh.getParam("synch_topic", impl_->synch_topic));

        if(impl_->is_synch) impl_->sub_tf = nh.subscribe("/tf", 50, &TF2StaticBroadcaster::TFCallback, this);

        // get param about position of frame from parent_frame 
        SetBroadcaster(nh, "map", impl_->map);
        SetBroadcaster(nh, "base_link", impl_->base_link);
        SetBroadcaster(nh, "velodyne1", impl_->velodyne1);
        SetBroadcaster(nh, "velodyne2", impl_->velodyne2);
        SetBroadcaster(nh, "carla", impl_->carla);
        SetBroadcaster(nh, "imu", impl_->imu);


        return;
    }

    void TF2StaticBroadcaster::SetBroadcaster(ros::NodeHandle &nh, std::string frame_id, geometry_msgs::TransformStamped &data)
    {
        tf_data temp;
        ROS_ASSERT(nh.getParam(frame_id + "/t_x", temp.t_x));
        ROS_ASSERT(nh.getParam(frame_id + "/t_y", temp.t_y));
        ROS_ASSERT(nh.getParam(frame_id + "/t_z", temp.t_z));
        ROS_ASSERT(nh.getParam(frame_id + "/r_x", temp.r_x));
        ROS_ASSERT(nh.getParam(frame_id + "/r_y", temp.r_y));
        ROS_ASSERT(nh.getParam(frame_id + "/r_z", temp.r_z));
        ROS_ASSERT(nh.getParam(frame_id + "/parent_id", temp.parent_id));
        ROS_ASSERT(nh.getParam(frame_id + "/frame_id", temp.frame_id));

        // rotation data for map, velodyne tf that does not change its rotation
        tf2::Quaternion q;
        q.setRPY(temp.r_x, temp.r_y, temp.r_z);
        // set map
        // set map frame name
        data.header.frame_id = temp.parent_id;
        data.child_frame_id = temp.frame_id;
        // set position of map from world (unit: m)
        data.transform.translation.x = temp.t_x;
        data.transform.translation.y = temp.t_y;
        data.transform.translation.z = temp.t_z;
        // set rotation of map from world (unit: rad)
        data.transform.rotation.x = q.x();
        data.transform.rotation.y = q.y();
        data.transform.rotation.z = q.z();
        data.transform.rotation.w = q.w();
    
        impl_->stamp_list.push_back(data);
        return;
    }

    void TF2StaticBroadcaster::SpinOnce()
    {
        for(auto &it:impl_->stamp_list)
        {
            Broadcasting(it);
        }
        return;
    }

    void TF2StaticBroadcaster::Broadcasting(geometry_msgs::TransformStamped &frame){
        // set header time stamp
        if(!impl_->is_synch) frame.header.stamp = ros::Time::now();
        else frame.header.stamp = impl_->timestamp;
        // broadcasting
        impl_->br.sendTransform(frame);
        return;
    }

    void TF2StaticBroadcaster::TFCallback(const tf2_msgs::TFMessage::Ptr &tf){

        for(auto &it : tf->transforms){
            if(it.child_frame_id == impl_->synch_topic){
                impl_->timestamp = it.header.stamp;
                return;
            }else continue;
        }
        return;
    }
}
