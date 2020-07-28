#!/usr/bin/env python
import rospy
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Float32, Float32MultiArray, Bool
from nav_msgs.msg import Odometry
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist
from set_goal import move_to_goal
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import cos, sin, pi
import time

TRASHCAN_POSE = []
TRASHCAN_CENTER_POSE = None

def trashcan_callback(data):
    global TRASHCAN_POSE    
    TRASHCAN_POSE = data.data
    #print("Found trashcan!")
    #print('TRASHCAN_POSE',TRASHCAN_POSE)

def tag_callback(data):
    global TRASHCAN_CENTER_POSE
    detections = data.detections
    for tag in detections:
        if tag.id[0] == 10:
            TRASHCAN_CENTER_POSE = tag.pose.pose.pose.position.x
            print('TRASHCAN_CENTER_POSE',TRASHCAN_CENTER_POSE)

def find_trashcan():
    global TRASHCAN_POSE

    done_publisher = rospy.Publisher('find_trashcan_done',Bool,queue_size=1)

    #print('TRASHCAN_POSE',TRASHCAN_POSE)
    #print('TRASHCAN_POSE[0]', TRASHCAN_POSE[0])
    #print('TRASHCAN_POSE[1]', TRASHCAN_POSE[1])
    #print(TRASHCAN_POSE[2])
    #print('TRASHCAN_POSE[3]', TRASHCAN_POSE[3])
    #print('TRASHCAN_POSE[4]', TRASHCAN_POSE[4])
    #print('TRASHCAN_POSE[5]', TRASHCAN_POSE[5])
    #print('TRASHCAN_POSE[6]', TRASHCAN_POSE[6])


    (roll, pitch, yaw) = euler_from_quaternion([TRASHCAN_POSE[3], TRASHCAN_POSE[4], TRASHCAN_POSE[5], TRASHCAN_POSE[6]])
    #print('roll pitch yaw',roll, pitch, yaw)
    yaw = yaw + pi/2
    trashcan_q = quaternion_from_euler(roll, pitch, yaw)
    
    # set goal in front of trashcan, instead of at the trash can itself
    goal_x = TRASHCAN_POSE[0] - 0.4*cos(yaw)
    goal_y = TRASHCAN_POSE[1] - 0.4*sin(yaw)
    print('goal_x', goal_x)
    print('goal_y', goal_y)

    result = move_to_goal(goal_x, goal_y, trashcan_q[2], trashcan_q[3])
    if result:
        print("Reached trashcan!")
        # Tell other nodes that find_trashcan is done
        done_publisher.publish(True)
    else:
        print("Failed to reach near trashcan.")

    return result


def approach_trashcan():
    global TRASHCAN_CENTER_POSE

    velocity_publisher = rospy.Publisher('cmd_vel',Twist,queue_size=10)
    velocity_msg = Twist()

    reached = False
    centered_far = False
    centered_near = False

    while not centered_near:
        if not reached:
            rotate_speed = 0.05
        elif reached:
            rotate_speed = 0.05

        if TRASHCAN_CENTER_POSE < -0.01:
            velocity_msg.angular.z = rotate_speed
        elif TRASHCAN_CENTER_POSE > 0.01:
            velocity_msg.angular.z = -rotate_speed
        else:
            velocity_msg.angular.z = 0  
            centered_far = True
            centered_near = reached

        velocity_publisher.publish(velocity_msg)
        time.sleep(1)
        velocity_msg.angular.z = 0
        velocity_publisher.publish(velocity_msg)

        if centered_far and not reached:
            velocity_msg.linear.x = 0.1
            velocity_publisher.publish(velocity_msg)
            time.sleep(5)
            velocity_msg.linear.x = 0
            velocity_publisher.publish(velocity_msg)
            reached = True
            print('Reached trashcan!')
        #break

    print('Trashcan centered!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!') 

def pose_listener():

    rospy.init_node('find_trashcan')

    rospy.Subscriber('trashcan_pose', numpy_msg(Float32MultiArray), trashcan_callback)
    rospy.Subscriber('tag_detections', AprilTagDetectionArray, tag_callback)

    raw_input("Press Enter to start Finding Trashcan...")
    near_trashcan = find_trashcan()
    if near_trashcan:
        approach_trashcan()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    pose_listener()