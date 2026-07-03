#!/usr/bin/env python3
# [transferable-to-heven_ad]
import sys
import signal
import time
import rospy
from pathlib import Path
sys.path.append(str(Path(__file__).resolve().parents[1]))

from lib.network.UDP import Receiver
from lib.define.EgoVehicleStatus import EgoVehicleStatus
from lib.define.CollisionData import CollisionData
from morai_msgs.msg import EgoVehicleStatus as EgoVehicleStatusMsg
from morai_msgs.msg import CollisionData as CollisionDataMsg
from morai_msgs.msg import ObjectStatus
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header

# IP = '127.0.0.1'
# VEHICLE_PORT = 1909
# COLLISION_PORT = 9092   # heven

IP = '127.0.0.1'
VEHICLE_PORT = 1911
COLLISION_PORT = 9092   # autonomous driving

# IP = '192.168.0.11'
# VEHICLE_PORT = 2211  
# COLLSION_PORT =       # morai

# IP = '192.168.0.11'
# VEHICLE_PORT = 2211  
# COLLSION_PORT =       # morai AI


def signal_handler(sig, frame):
    print("Ctrl+C pressed. Exiting...")
    sys.exit(0)  # force exit immediately

signal.signal(signal.SIGINT, signal_handler)

def main():
    rospy.init_node('udp_publisher', anonymous=False)

    # Publishers
    ego_pub = rospy.Publisher('/Competition_topic', EgoVehicleStatusMsg, queue_size=10)
    collision_pub = rospy.Publisher('/CollisionData', CollisionDataMsg, queue_size=10)

    # UDP Receivers
    ego_receiver = Receiver(IP, VEHICLE_PORT, EgoVehicleStatus())
    collision_receiver = Receiver(IP, COLLISION_PORT, CollisionData())

    rospy.loginfo("Publishers started on ports %d (EgoVehicle) and %d (CollisionData)", VEHICLE_PORT, COLLISION_PORT)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        status = ego_receiver.get_data()
        if status:

            msg = EgoVehicleStatusMsg()
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()

            msg.unique_id = 0  # Optional, depends on your use
            msg.acceleration = Vector3(status.accel_x, status.accel_y, status.accel_z)
            msg.position = Vector3(status.pos_x, status.pos_y, status.pos_z)
            msg.velocity = Vector3(status.vel_x/3.6, status.vel_y/3.6, status.vel_z/3.6)
            msg.heading = status.yaw

            msg.accel = status.accel
            msg.brake = status.brake
            msg.front_steer_angle = status.steer
            msg.rear_steer_angle = 0.0
            msg.lateral_offset = 0.0  # You can compute this if needed

            # Tire forces
            msg.tire_lateral_force_fl = status.tire_lateral_force_fl
            msg.tire_lateral_force_fr = status.tire_lateral_force_fr
            msg.tire_lateral_force_rl = status.tire_lateral_force_rl
            msg.tire_lateral_force_rr = status.tire_lateral_force_rr

            # Side slip angles
            msg.side_slip_angle_fl = status.side_slip_angle_fl
            msg.side_slip_angle_fr = status.side_slip_angle_fr
            msg.side_slip_angle_rl = status.side_slip_angle_rl
            msg.side_slip_angle_rr = status.side_slip_angle_rr

            # Cornering stiffness
            msg.tire_cornering_stiffness_fl = status.tire_cornering_stiffness_fl
            msg.tire_cornering_stiffness_fr = status.tire_cornering_stiffness_fr
            msg.tire_cornering_stiffness_rl = status.tire_cornering_stiffness_rl
            msg.tire_cornering_stiffness_rr = status.tire_cornering_stiffness_rr

            ego_pub.publish(msg)
        
        collision_data = collision_receiver.get_data()
        if collision_data:
            col_msg = CollisionDataMsg()
            col_msg.header = Header()
            col_msg.header.stamp = rospy.Time.now()
            col_msg.global_offset_x = collision_data._data[0].globalOffset_x
            col_msg.global_offset_y = collision_data._data[0].globalOffset_y
            col_msg.global_offset_z = collision_data._data[0].globalOffset_z

            # Convert each object
            col_msg.collision_object = []
            for obj in collision_data.data:
                if obj.objType == 0 or obj.objType == -1:
                    continue
                obj_msg = ObjectStatus()
                obj_msg.position.x = obj.pose_x
                obj_msg.position.y = obj.pose_y
                obj_msg.position.z = obj.pose_z
                obj_msg.type = obj.objType
                obj_msg.unique_id = obj.obj_id
                col_msg.collision_object.append(obj_msg)

            collision_pub.publish(col_msg)

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
