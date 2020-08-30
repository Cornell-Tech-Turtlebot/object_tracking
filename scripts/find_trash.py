#!/usr/bin/env python
import rospy
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Float32, Float32MultiArray, Bool, String
from nav_msgs.msg import Odometry
from darknet_ros_msgs.msg import BoundingBoxes
from set_goal import move_to_goal
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import cos, sin, atan, asin
from geometry_msgs.msg import Twist
import time

ROBOT_POSE = None
TRASH_POSE = []
LATEST_OBJECT = None
#detect_publisher = rospy.Publisher('trash_detected',Bool,queue_size=1)
detect_publisher = rospy.Publisher('state',String,queue_size=1)

#camera conversion: 1m = 1650px
#image resolution: 640px * 480px
CAMERA_METER_TO_PIXEL = 500 #2000
IMAGE_HEIGTH = 240 #480
IMAGE_WIDTH = 320 #640
IMAGE_CENTER_X = IMAGE_WIDTH / 2
TRASH_CONST = 50.0 #99.0 #trash_distance_meter * trash_image_height_pixel
TRASH_MAX_WIDTH = 100 #200


def odom_callback(data):
    global ROBOT_POSE
    ROBOT_POSE = data


def trash_callback(data):
    global TRASH_POSE  
    global LATEST_OBJECT
    global ROBOT_POSE
    global detect_publisher

    LATEST_OBJECT = data
    image_time = data.image_header.stamp.secs
    #print('image_time',image_time)
    boxes = data.bounding_boxes
    for box in boxes:
        if box.Class == "bottle":
            #print('image_time',image_time)
            #print("Found bottle!")
            bottle = box
            if ROBOT_POSE:
                robot_pose = ROBOT_POSE.pose.pose
                TRASH_POSE = [image_time, bottle, robot_pose]
                detect_publisher.publish('trash_detected')
            break


def find_trash():
    global TRASH_POSE
    global ROBOT_POSE

    trash = TRASH_POSE[1]
    trash_time = TRASH_POSE[0]
    print('trash_time',trash_time)
    print('trash',trash)
    #trash_const = 99.0 #trash_distance_meter * trash_image_height_pixel
    trash_length_x = trash.xmax - trash.xmin
    trash_length_y = trash.ymax - trash.ymin
    trash_center_x = trash.xmin + trash_length_x / 2
    trash_off_center_x = IMAGE_CENTER_X - trash_center_x   #in pixels
    print('trash_off_center_x',trash_off_center_x)

    trash_length = max([trash_length_x, trash_length_y])
    print('trash_length', trash_length)
    trash_distance_m = TRASH_CONST / trash_length   #in meters
    print('trash_distance_m', trash_distance_m)
    trash_distance_px = trash_distance_m * CAMERA_METER_TO_PIXEL   #in pixels
    print('trash_distance_px', trash_distance_px)

    trash_angle = asin(trash_off_center_x / trash_distance_px)  #with respect to robot frame
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
    goal_x = trash_x - 0.4*cos(trash_yaw)
    goal_y = trash_y - 0.4*sin(trash_yaw)
    print('goal_x', goal_x)
    print('goal_y', goal_y)

    result = move_to_goal(goal_x, goal_y, trash_quarternion[2], trash_quarternion[3])
    if result:
        print("Reached near trash!")
    else:
        print("Failed to reach near trash.")

    return result


def approach_trash():
    global LATEST_OBJECT
    global TRASH_MAX_WIDTH

    #done_publisher = rospy.Publisher('trash_approached',Bool,queue_size=1)
    done_publisher = rospy.Publisher('state',String,queue_size=1)
    velocity_publisher = rospy.Publisher('cmd_vel',Twist,queue_size=10)
    velocity_msg = Twist()

    see_trash = False
    reached_trash = False
    trash_centered_far = False
    trash_centered_near = False

    while not trash_centered_near:
        image_time = LATEST_OBJECT.image_header.stamp.secs
        time_now = rospy.Time.now().secs
        print('image_time',image_time)
        print('time_now',time_now)
        if time_now - image_time > 1:
            object_detected = False
        else:
            object_detected = True

        see_trash = False
        if object_detected:
            boxes = LATEST_OBJECT.bounding_boxes
            for box in boxes:
                if box.Class in ["bottle", "cup"]:
                    see_trash = True
                    trash = box
                    trash_length_x = trash.xmax - trash.xmin
                    trash_length_y = trash.ymax - trash.ymin
                    trash_length = max([trash_length_x, trash_length_y])
                    trash_center_x = trash.xmin + trash_length_x / 2
                    print('trash_center_x', trash_center_x)
                    trash_off_center_x = trash_center_x - IMAGE_CENTER_X
                    trash_distance_m = TRASH_CONST / trash_length

                    if not reached_trash:
                        rotate_speed = 0.1
                    elif reached_trash:
                        rotate_speed = 0.1

                    if trash_center_x < IMAGE_CENTER_X-20:
                        velocity_msg.angular.z = rotate_speed
                    elif trash_center_x > IMAGE_CENTER_X+20:
                        velocity_msg.angular.z = -rotate_speed
                    else:
                        velocity_msg.angular.z = 0  
                        trash_centered_far = True
                        trash_centered_near = reached_trash

                    velocity_publisher.publish(velocity_msg)
                    time.sleep(0.1)
                    velocity_msg.angular.z = 0
                    velocity_publisher.publish(velocity_msg)

                    if trash_centered_far and not reached_trash:
                        if trash_length_x < TRASH_MAX_WIDTH:
                            velocity_msg.linear.x = 0.05
                        else:
                            velocity_msg.linear.x = 0
                            reached_trash = True
                        velocity_publisher.publish(velocity_msg)
                        print('Reached trash!')
                    break

        if not see_trash: #don't see trash. Rotate to find it
            print('Do not see trash! Turning around to find...')
            velocity_msg.angular.z = 0.1
            velocity_publisher.publish(velocity_msg)
            time.sleep(0.1)
            velocity_msg.angular.z = 0
            velocity_publisher.publish(velocity_msg)
            #time.sleep(5)

    print('Trash centered!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')  

    # Tell other nodes that find_trash is done
    #done_publisher.publish(True)  
    done_publisher.publish('approach_trash_done')                    


def state_callback(data):
    if data.data == 'approach_trash':
        near_trash = find_trash()
        if near_trash:
            approach_trash()


def pose_listener():

    rospy.init_node('find_trash')#, anonymous=True)

    rospy.Subscriber('/odom', Odometry, odom_callback)

    rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, trash_callback)

    rospy.Subscriber('/state', String, state_callback)

    #raw_input("Press Enter to find the next Trash...")
    #near_trash = find_trash()
    #if near_trash:
    #    approach_trash()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    pose_listener()