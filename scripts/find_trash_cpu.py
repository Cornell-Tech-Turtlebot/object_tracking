#!/usr/bin/env python
import rospy
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Float32, Float32MultiArray, Bool
from nav_msgs.msg import Odometry
from darknet_ros_msgs.msg import BoundingBoxes
from set_goal import move_to_goal
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import cos, sin, atan
from geometry_msgs.msg import Twist
import time

ROBOT_POSES = []
TRASH_POSE = []

def odom_callback(data):
    global ROBOT_POSES
    odom_time = data.header.stamp.secs
    if ROBOT_POSES == [] or (odom_time - ROBOT_POSES[-1].header.stamp.secs) >=1:
        ROBOT_POSES.append(data)
        #print('robot_pose', data)
    while len(ROBOT_POSES) > 30:
        ROBOT_POSES.pop(0)

def trash_callback(data):
    global TRASH_POSE  

    #print('bounding boxes',data.bounding_boxes)
    image_time = data.image_header.stamp.secs
    boxes = data.bounding_boxes
    for box in boxes:
        if box.Class == "bottle":
            print('image_time',image_time)
            print("Found bottle!")
            bottle = box

            robot_pose = None
            for pose in ROBOT_POSES:
                #print('pose.header.stamp.secs',pose.header.stamp.secs)
                if abs(pose.header.stamp.secs - image_time) == 0:
                    robot_pose = pose.pose.pose
                    print('Robot pose time', pose.header.stamp.secs)
                    print('Trash image time', image_time)
                    break

            if robot_pose == None:
                print('Cant find a robot pose near the time of object detection.')
                robot_pose = ROBOT_POSES[-1].pose.pose

            TRASH_POSE = [image_time, bottle, robot_pose]

    #print('bottle', bottle)  
    #print(TRASH_POSE)


def find_trash():
    global TRASH_POSE
    global ROBOT_POSES

    velocity_publisher = rospy.Publisher('cmd_vel',Twist,queue_size=10)
    velocity_msg = Twist()
    done_publisher = rospy.Publisher('find_trash_done',Bool,queue_size=1)

    #camera conversion: 1m = 1650px
    #image resolution: 640px * 480px
    camera_meter_to_pixel= 1650
    image_heigth = 480
    image_width = 640
    image_center_x = image_width / 2

    trash = TRASH_POSE[1]
    trash_time = TRASH_POSE[0]
    print('trash_time',trash_time)
    print('trash',trash)
    trash_const = 99.0 #trash_distance_meter * trash_image_height_pixel
    trash_length_x = trash.xmax - trash.xmin
    trash_length_y = trash.ymax - trash.ymin
    trash_center_x = trash.xmin + trash_length_x / 2
    trash_off_center_x = trash_center_x - image_center_x   #in pixels
    print('trash_off_center_x',trash_off_center_x)

    trash_length = max([trash_length_x, trash_length_y])
    print('trash_length', trash_length)
    trash_distance_m = trash_const / trash_length   #in meters
    trash_distance_px = trash_distance_m * camera_meter_to_pixel   #in pixels
    print('trash_distance_px', trash_distance_px)

    trash_angle = atan(trash_off_center_x / trash_distance_px)  #with respect to robot frame
    print('trash_angle',trash_angle)

    robot_pose = TRASH_POSE[2]  #with respect to map
    robot_x = robot_pose.position.x
    robot_y = robot_pose.position.y
    (_, _, robot_yaw) = euler_from_quaternion([robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w])
    print('robot_yaw', robot_yaw)

    trash_yaw = robot_yaw + trash_angle
    trash_quarternion = quaternion_from_euler(0,0,trash_yaw)

    trash_x = robot_x + trash_distance_m * cos(trash_yaw)
    trash_y = robot_y + trash_distance_m * sin(trash_yaw)

    # set goal in front of trash, instead of at the trash itself
    goal_x = trash_x - 0.3*cos(trash_yaw)
    goal_y = trash_y - 0.3*sin(trash_yaw)
    print('goal_x', goal_x)
    print('goal_y', goal_y)

    result = move_to_goal(goal_x, goal_y, trash_quarternion[2], trash_quarternion[3])
    if result:
        print("Reached near trash!")

        see_trash = False
        reached_trash = False
        trash_centered_far = False
        trash_centered_near = False
        time_stop = rospy.Time.now().secs
        #while trash_distance_m > 0.1:
        while not trash_centered_near:
            time_late = 10
            while time_late > 0:
                try:
                    trash_detection = rospy.wait_for_message('/darknet_ros/bounding_boxes', BoundingBoxes, timeout=10)
                except:
                    #see_trash = False
                    print('Cannot detect any object')
                    break 
                image_time = trash_detection.image_header.stamp.secs
                time_late = time_stop - image_time
                print('time_late', time_late)

            print('Right time!')
            see_trash = False
            boxes = trash_detection.bounding_boxes
            for box in boxes:
                if box.Class in ["bottle", "cup"]:
                    see_trash = True
                    trash = box
                    trash_length_x = trash.xmax - trash.xmin
                    trash_length_y = trash.ymax - trash.ymin
                    trash_length = max([trash_length_x, trash_length_y])
                    trash_center_x = trash.xmin + trash_length_x / 2
                    print('trash_center_x', trash_center_x)
                    trash_off_center_x = trash_center_x - image_center_x
                    trash_distance_m = trash_const / trash_length

                    if not reached_trash:
                        rotate_speed = 0.05
                    elif reached_trash:
                        rotate_speed = 0.02

                    if trash_center_x < image_center_x-20:
                        velocity_msg.angular.z = rotate_speed
                    elif trash_center_x > image_center_x+20:
                        velocity_msg.angular.z = -rotate_speed
                    else:
                        velocity_msg.angular.z = 0  
                        trash_centered_far = True
                        trash_centered_near = reached_trash

                    velocity_publisher.publish(velocity_msg)
                    #time.sleep(trash_off_center_x / 43)
                    time.sleep(1)
                    velocity_msg.angular.z = 0
                    velocity_publisher.publish(velocity_msg)
                    time_stop = rospy.Time.now().secs

                    if trash_centered_far and not reached_trash:
                    #if trash_distance_m > 0.1:
                        velocity_msg.linear.x = 0.1
                        velocity_publisher.publish(velocity_msg)
                        time.sleep(2)
                        velocity_msg.linear.x = 0
                        velocity_publisher.publish(velocity_msg)
                        reached_trash = True
                        print('Reached trash!')

            if not see_trash: #don't see trash. Rotate to find it
                print('Do not see trash!')
                velocity_msg.angular.z = 0.1
                velocity_publisher.publish(velocity_msg)
                time.sleep(5)
                velocity_msg.angular.z = 0
                velocity_publisher.publish(velocity_msg)
                time.sleep(5)

        print('Trash centered!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')                       

    else:
        print("Failed to reach near trash.")


def pose_listener():

    rospy.init_node('find_trash', anonymous=True)

    rospy.Subscriber('/odom', Odometry, odom_callback)

    rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, trash_callback)

    raw_input("Press Enter to find the next Trash...")
    find_trash()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    pose_listener()