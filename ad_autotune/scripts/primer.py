#!/usr/bin/env python3
# [transferable-to-heven_ad]
"""primer — MORAI가 advertise 없이 publish하는 토픽들의 타입을 rosbridge에 등록.
이거 없으면 rosbridge가 'Cannot infer topic type'. devel 소스 후 실행."""
import rospy
from morai_msgs.msg import EgoVehicleStatus, ObjectStatusList, CollisionData

rospy.init_node("morai_topic_primer", anonymous=True)
pubs = [
    rospy.Publisher("/Ego_topic", EgoVehicleStatus, queue_size=1),
    rospy.Publisher("/Competition_topic", EgoVehicleStatus, queue_size=1),
    rospy.Publisher("/Object_topic", ObjectStatusList, queue_size=1),
    rospy.Publisher("/CollisionData", CollisionData, queue_size=1),
]
rospy.loginfo("primed morai topic types for rosbridge")
rospy.spin()
