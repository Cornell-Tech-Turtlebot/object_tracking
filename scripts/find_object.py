#!/usr/bin/env python
import rospy
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Float32, Float32MultiArray, Bool
from nav_msgs.msg import Odometry
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

ROBOT_POSES = []
POSE_ID = 0

def move_to_goal(robot_pose):

   # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
 
   # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

   # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

   # Linear position with respect to map
    goal.target_pose.pose.position.x = robot_pose.position.x
    goal.target_pose.pose.position.y = robot_pose.position.y

   # Rotation
    goal.target_pose.pose.orientation.z = robot_pose.orientation.z
    goal.target_pose.pose.orientation.w = robot_pose.orientation.w

   # Sends the goal to the action server.
    client.send_goal(goal)

   # Waits for the server to finish performing the action.
    wait = client.wait_for_result()

   # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Result of executing the action
        return client.get_result() 

def pose_callback(data):
	global ROBOT_POSES
	
	ROBOT_POSES.append(data.pose.pose)
	print('New object location', len(ROBOT_POSES)-1)
	

def find_pose():
	global ROBOT_POSES
	global POSE_ID

	pub_done = rospy.Publisher('find_object_done',Bool,queue_size=1)

	#check if exploration stoped or approached object, then set Goal to the next Robot_pose
	if POSE_ID < len(ROBOT_POSES):
		next_goal = ROBOT_POSES[POSE_ID]
		print('Next goal',POSE_ID,'- Total:',len(ROBOT_POSES))
		POSE_ID += 1

		prev_len = len(ROBOT_POSES)
		result = move_to_goal(next_goal)
		if result:
			print("Goal execution done!")
			# Tell other nodes that find_object is done
			pub_done.publish(True)
			print('Done published!')

		#remove duplicated new goals
		while len(ROBOT_POSES) > prev_len: 
			ROBOT_POSES.pop()
		print('Total goals:',len(ROBOT_POSES))
	else:
		print('Done, found all objects!')

def done_callback(data):
	if data.data:
		find_pose()

def pose_listener():

    rospy.init_node('find_object', anonymous=True)

    rospy.Subscriber('robot_pose', Odometry, pose_callback)

    raw_input("Press Enter to start Finding Object...")
    find_pose()

    rospy.Subscriber('follow_object_done', Bool, done_callback)
    #follow_object_done = rospy.wait_for_message('follow_object_done', Bool)
    #if follow_object_done:
    #	find_pose()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    pose_listener()
