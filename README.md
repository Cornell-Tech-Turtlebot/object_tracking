# object_tracking
Detect object &amp; move the robot toward object

## How to run:

1. Download & put this whole `object_tracking` folder inside the `src` folder in your catkin workspace

2. Go to `object_tracking/scripts/`, then run this:

        chmod a+x track_object.py
        chmod a+x follow_object.py

2. Go to your `catkin_ws`, then run this:

        catkin_make

3. Then run these commands, each command in a separate Terminal window:

        roscore
    
        roslaunch turtlebot3_gazebo turtlebot3_house.launch
    
        roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
    
        roslaunch turtlebot3_navigation move_base.launch
    
        rosrun object_tracking detect_object.py
        
        rosrun object_tracking find_object.py
    
        rosrun object_tracking follow_object.py
    
4. Use RViz to guide the robot to look toward the Trash Can in the House. If you don't know how to do this, follow this: https://www.oreilly.com/library/view/ros-programming-building/9781788627436/192de5c9-e5bd-40b3-a75a-2990bdfa7caf.xhtml

5. Once the robot sees the Trash Can, it will remember the location of the Trash Can.

6. Keep guiding the robot to explore other parts of the House. Once finished, open the Terminal window that is running `find_object.py`. Then press Enter. The robot will automatically revisit the Trash Can locations & approach the Trash Can.
    
