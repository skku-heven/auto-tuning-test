#!/usr/bin/env python3
# [transferable-to-heven_ad]
"""ad_ego_pose_bridge — /Ego_topic(EgoVehicleStatus) → /ad_pose_parser/pose(Pose2D).

튜닝 하네스용 ground-truth pose 브리지. ad_tracker(C++)가 /ad_pose_parser/pose를 구독하는데,
튜닝 단계에선 GPS/IMU + 진짜 ad_pose_parser 대신 /Ego_topic(검증된 ground-truth)로 pose를 먹임.
실차 배포 땐 이 노드 대신 진짜 ad_pose_parser(GPS/IMU) 사용."""
import math
import rospy
from morai_msgs.msg import EgoVehicleStatus
from geometry_msgs.msg import Pose2D


def main():
    rospy.init_node("ad_ego_pose_bridge")
    pub = rospy.Publisher("/ad_pose_parser/pose", Pose2D, queue_size=1)

    def cb(m):
        p = Pose2D()
        p.x = m.position.x
        p.y = m.position.y
        p.theta = math.radians(m.heading)   # MORAI heading[deg] → rad
        pub.publish(p)

    rospy.Subscriber("/Ego_topic", EgoVehicleStatus, cb, queue_size=10)
    rospy.loginfo("ad_ego_pose_bridge: /Ego_topic -> /ad_pose_parser/pose")
    rospy.spin()


if __name__ == "__main__":
    main()
