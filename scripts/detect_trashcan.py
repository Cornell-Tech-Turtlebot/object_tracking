#!/usr/bin/env python
import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Float32, Float32MultiArray, Bool
import math
import tf
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('detect_trashcan')

    pose_publisher = rospy.Publisher('trashcan_pose',numpy_msg(Float32MultiArray),queue_size=1)
    done_publisher = rospy.Publisher('trashcan_detected',Bool,queue_size=1)

    tf_listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            #(translate, rotate) = tf_listener.lookupTransform('/map', '/tag_10', rospy.Time(0))
            (translate, rotate) = tf_listener.lookupTransform('/map', '/ar_marker_4', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        #message_array = [translate[0], translate[1], rotate[2], rotate[3]]
        message_array = translate + rotate
        message = Float32MultiArray(data = message_array)

        pose_publisher.publish(message)
        done_publisher.publish(True)
        
        print('Translation', translate)
        print('Rotate', rotate)

        rate.sleep()