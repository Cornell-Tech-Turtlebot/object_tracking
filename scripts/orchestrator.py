#!/usr/bin/env python
import rospy
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Float32, Float32MultiArray, Bool, String
import time

TRASH_DETECTED = False
#TRASHCAN_DETECTED = False

def state_callback(msg):
    global TRASH_DETECTED
    global TRASHCAN_DETECTED

    state_pub = rospy.Publisher('/state', String, queue_size=1)

    if msg.data == 'explore_done':
        state_pub.publish('patrol')

    if msg.data == 'trashcan_detected':
        TRASHCAN_DETECTED = True

    if msg.data == 'trash_detected' and TRASHCAN_DETECTED:
        state_pub.publish('approach_trash')

    if msg.data = 'approach_trash_done':
        state_pub.publish('pickup_trash')

    if msg.data = 'pickup_trash_done':
        state_pub.publish('approach_trashcan')

    if msg.data = 'approach_trashcan_done':
        state_pub.publish('drop_trash')

    if msg.data = 'drop_trash_done':
        #state_pub.publish('patrol')
        print('Done!')


def state_listener():
    rospy.init_node('orchestrator')
    rospy.Subscriber('/state', String, state_callback)
    rospy.spin()


if __name__ == '__main__':
    state_listener()