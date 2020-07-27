#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def move_to_goal(x,y,z_orient,w_orient):
   # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
 
   # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

   # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

   # Linear position with respect to map
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y

   # Rotation
    #goal.target_pose.pose.orientation.x = x_orient
    #goal.target_pose.pose.orientation.y = y_orient
    goal.target_pose.pose.orientation.z = z_orient
    goal.target_pose.pose.orientation.w = w_orient

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


def listener():
    rospy.init_node('set_goal', anonymous=True)
    
    rospy.Subscriber('move_goal', numpy_msg(Float32MultiArray), move_to_goal)

    rospy.spin()

if __name__ == '__main__':
    listener()