import rospy
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
from pyquaternion import Quaternion
import numpy as np
import numpy.matlib as npm
import json
import logging
import geometry_msgs
import tf2_msgs
from visualization_msgs.msg import Marker

pub = rospy.Publisher("/marker", Marker, queue_size=10)
joints = ["head", "neck", "torso", "left_shoulder", "left_elbow", "left_hand",
                  "right_shoulder", "right_elbow", "right_hand", "left_hip", "left_knee",
                  "left_foot", "right_hip", "right_knee", "right_foot"]
users = []

def callback(data):

    marker = Marker()
    marker.header.frame_id = "kinect1_depth_frame"
    marker.header.stamp    = rospy.get_rostime()

    marker.type = marker.CUBE
    marker.action = marker.ADD

    marker.ns = "my_namespace"

    t = rospy.Duration(0,100000000)
    marker.lifetime = t

    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.05


    for transform in data.transforms:
        marker.pose.position.x = transform.transform.translation.x
        marker.pose.position.y = transform.transform.translation.y
        marker.pose.position.z = transform.transform.translation.z

        marker.pose.orientation.x = transform.transform.rotation.x
        marker.pose.orientation.y = transform.transform.rotation.y
        marker.pose.orientation.z = transform.transform.rotation.z
        marker.pose.orientation.w = transform.transform.rotation.w

        for i, joint in enumerate(joints):
            if joint in str(transform.child_frame_id):
                marker.id = i

        if str(transform.child_frame_id).split("_")[-1] not in users:
            users.append(str(transform.child_frame_id).split("_")[-1])



        if str(transform.child_frame_id).split("_")[-1]=="Natalia":
            print(str(transform.child_frame_id).split("_")[-1])
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.id=marker.id+15*users.index(str(transform.child_frame_id).split("_")[-1])

        elif str(transform.child_frame_id).split("_")[-1]=="Karine":
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.id=marker.id+15*users.index(str(transform.child_frame_id).split("_")[-1])

        elif str(transform.child_frame_id).split("_")[-1]=="Renata":
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.id=marker.id+15*users.index(str(transform.child_frame_id).split("_")[-1])

        else:
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.id=marker.id+15*users.index(str(transform.child_frame_id).split("_")[-1])

    pub.publish(marker)

    

rospy.init_node('display', anonymous=True)
sub = rospy.Subscriber("/global_skeleton_tf", TFMessage, callback)
rospy.spin()
