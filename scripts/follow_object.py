#!/usr/bin/env python
import rospy
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Float32, Float32MultiArray, Bool
from geometry_msgs.msg import Twist
msg=Twist()

OBJECT_FOUND = False

def pose_callback(data):
    global OBJECT_FOUND

    pub_velocity = rospy.Publisher('cmd_vel',Twist,queue_size=10)
    pub_done = rospy.Publisher('follow_object_done',Bool,queue_size=1)

    if OBJECT_FOUND:
        print('Aproaching object...')
        radius = data.data[0]
        center_x = data.data[1]
        center_y = data.data[2]

        if center_x < 300:
            msg.angular.z = 0.1
        elif center_x > 300:
            msg.angular.z = -0.1
        else:
            msg.angular.z = 0

        if(radius < 200):
            msg.linear.x = 0.3
        else:
            msg.linear.x = 0
            pub_done.publish(True)
            OBJECT_FOUND = False
            print('Done!')

        pub_velocity.publish(msg)


def done_callback(data):
    global OBJECT_FOUND

    print('find_object_done callback', data.data)
    if data.data:
        OBJECT_FOUND = True

def listener():
    rospy.init_node('follow_object', anonymous=True)

    rospy.Subscriber('find_object_done', Bool, done_callback)
    
    rospy.Subscriber('object_pose', numpy_msg(Float32MultiArray), pose_callback)

    rospy.spin()

if __name__ == '__main__':
    listener()