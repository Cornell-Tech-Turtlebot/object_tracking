#!/usr/bin/env python
import rospy
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import Twist
msg=Twist()

def callback(data):
    
    #rospy.loginfo(rospy.get_caller_id(), "I heard", data.data)
    pub=rospy.Publisher('cmd_vel',Twist,queue_size=10)
    #print('data',data.data[0])
    #print('radius',data.data[0])
    #print('center',data.data[1])
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
        msg.linear.x=0.5
    else:
        msg.linear.x=0

    pub.publish(msg)

def listener():

    rospy.init_node('follow_object', anonymous=True)

    rospy.Subscriber('object_position', numpy_msg(Float32MultiArray), callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()