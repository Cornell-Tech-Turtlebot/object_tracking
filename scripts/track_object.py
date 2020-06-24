#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Float32, Float32MultiArray
from collections import deque
import argparse
import imutils
import sys

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image


def camera_callback(data):
	pub=rospy.Publisher('object_position',numpy_msg(Float32MultiArray),queue_size=1)

	rate = rospy.Rate(20) # 10hz
	ap = argparse.ArgumentParser()
	ap.add_argument("-b", "--buffer", type=int, default=64,help="max buffer size")
	args = vars(ap.parse_args())

	#green
	#colorLower = (20, 100, 100)
	#colorUpper = (64, 255, 255)

	#black
	#colorLower = (0, 0, 0)
	#colorUpper = (100, 100, 100)

	#trash can
	colorLower = (0, 0, 0)
	colorUpper = (1, 27, 2)

	pts = deque(maxlen=64)

	try:
		frame = CvBridge().imgmsg_to_cv2(data, desired_encoding="bgr8")
	except CvBridgeError as e:
		print(e)
	

	frame = imutils.resize(frame, width=600)
	mask = cv2.inRange(frame, colorLower, colorUpper)
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
	center = None
	if len(cnts) > 0:
		c = max(cnts, key=cv2.contourArea)
		((x, y), radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
		#print('center',center)
		#print('radius',radius)

		message_array = [radius, center[0], center[1]]
		print('message_array', message_array)

		message = Float32MultiArray(data = message_array)
		print('message data', message.data)

		pub.publish(message)
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

	cv2.imshow("Frame", frame)
	cv2.waitKey(1)

	#rospy.spin()

def listener():
	rospy.init_node('track_object', anonymous=True)
	rospy.Subscriber('/camera/rgb/image_raw', Image, camera_callback)

	rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
	video_capture.release()
	cv2.destroyAllWindows()
	pass