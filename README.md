# object_tracking
Detect object &amp; move the robot toward object

## How to run:

1. Download & put this whole `object_tracking` folder inside the `src` folder in your catkin workspace

2. In your `catkin_ws`, run this:

        catkin_make

3. Then run these commands, each command in a separate Terminal window:

        roscore
    
        roslaunch turtlebot3_gazebo turtlebot3_house.launch
    
        roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
    
        roslaunch turtlebot3_navigation move_base.launch
    
        rosrun object_tracking track_object.py
    
        rosrun object_tracking follow_object
    
4. Use RViz to guide the robot to look toward the Trash Can in the House. If you don't know how to do this, follow this: https://www.oreilly.com/library/view/ros-programming-building/9781788627436/192de5c9-e5bd-40b3-a75a-2990bdfa7caf.xhtml

5. Once the robot sees the Trash Can, it will automatically move toward the Trash Can.
    
