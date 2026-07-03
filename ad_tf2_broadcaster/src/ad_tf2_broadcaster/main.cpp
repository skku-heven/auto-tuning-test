// [transferable-to-heven_ad]
#include "ad_tf2_broadcaster/tf2_broadcaster.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ad_tf2_broadcaster");
    ros::NodeHandle nh("~");
    ad_tf2_broadcaster::TF2Broadcaster tf2b;
    tf2b.Init(nh);
    ros::spin();
}
