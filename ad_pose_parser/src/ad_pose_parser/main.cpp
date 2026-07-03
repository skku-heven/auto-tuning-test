// [transferable-to-heven_ad]
#include "ad_pose_parser/pose_parser.h"
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ad_pose_parser");
    ros::NodeHandle nh("~");
    ad_pose_parser::PoseParser parser;
    parser.Init(nh);

    ros::spin();
}