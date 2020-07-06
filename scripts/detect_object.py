#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Float32, Float32MultiArray
from nav_msgs.msg import Odometry
from collections import deque
import argparse
import imutils
import sys
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CompressedImage
from datetime import datetime
import time

OBJECT_DETECTED = False
TIME_DETECTED = 0

def camera_callback(data):
	global OBJECT_DETECTED
	global TIME_DETECTED
	
	pub_object = rospy.Publisher('object_pose',numpy_msg(Float32MultiArray),queue_size=1)
	pub_robot = rospy.Publisher('robot_pose', Odometry ,queue_size=1)

	rate = rospy.Rate(20) # 10hz

	#ap = argparse.ArgumentParser()
	#ap.add_argument("-b", "--buffer", type=int, default=64,help="max buffer size")
	#args = vars(ap.parse_args())

	#green
	#colorLower = (20, 100, 100)
	#colorUpper = (64, 255, 255)

	#black
	#colorLower = (0, 0, 0)
	#colorUpper = (100, 100, 100)

	#Gazebo trash can
	#colorLower = (0, 0, 0)
	#colorUpper = (1, 27, 2)

	#Zwee trash can
	colorLower = (25, 25, 25)
	colorUpper = (40, 40, 40)

	#blue charger
	#colorLower = (50, 30, 0)
	#colorUpper = (130, 70, 30)

	pts = deque(maxlen=64)

	try:
		frame = CvBridge().imgmsg_to_cv2(data, desired_encoding="bgr8")
		#frame = CvBridge().compressed_imgmsg_to_cv2(data, desired_encoding="bgr8")
	except CvBridgeError as e:
		print(e)
	

	frame = imutils.resize(frame, width=600)
	mask = cv2.inRange(frame, colorLower, colorUpper)
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
	center = None
	if len(cnts) == 0:
		OBJECT_DETECTED = False
	elif len(cnts) > 0:
		time_diff = time.time() - TIME_DETECTED
		TIME_DETECTED = time.time()
		#print('time_diff', time_diff)

		if OBJECT_DETECTED == False and time_diff > 1:
			print('New object detected')
			odom_msg = rospy.wait_for_message('/odom', Odometry)
			pub_robot.publish(odom_msg)
		OBJECT_DETECTED = True

		c = max(cnts, key=cv2.contourArea)
		((x, y), radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
		#print('center',center)
		#print('radius',radius)

		message_array = [radius, center[0], center[1]]
		#print('message_array', message_array)

		message = Float32MultiArray(data = message_array)
		#print('message data', message.data)

		pub_object.publish(message)
		rate.sleep()

		if radius > 10:
			cv2.circle(frame, (int(x), int(y)), int(radius),(0, 255, 255), 2)
			cv2.circle(frame, center, 5, (0, 0, 255), -1)

	pts.appendleft(center)
	for i in xrange(1, len(pts)):
		if pts[i - 1] is None or pts[i] is None:
			continue
		thickness = int(np.sqrt(64 / float(i + 1)) * 2.5)
		cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)

	cv2.imshow("Camera", frame)
	cv2.waitKey(1)

	#rospy.spin()


def listener():	
	rospy.init_node('detect_object', anonymous=True)
	rospy.Subscriber('/camera/rgb/image_raw', Image, camera_callback)
	#rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, camera_callback)

	rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
		video_capture.release()
		cv2.destroyAllWindows()
		pass