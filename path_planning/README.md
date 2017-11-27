# COMPGX01 - Path Planning

In this lab, we will be looking at trajectory planning using the messages `JointTrajectory` and `JointTrajectoryPoint`. This package contains a blank cpp/py file for you to fill-in your code to make youbot move.

##  Task 1: Write a code to load data from your bag from the previous lab
JointTrajectory and JointTrajectoryPoint directly take your joint command and publish them to the robot. Since you have already created your bag data from the last lab, this task is simply to write a code to load up those data.

##  Task 2: Create your trajectory using JointTrajectory message
`JointTrajectory` is a set of `JointTrajectoryPoint`. Consider one message of `JointTrajectory` as your whole trajectory containing a lot of "waypoint". You can see the documentation of `JointTrajectory` and `JointTrajectoryPoint` from these websites.

http://docs.ros.org/api/trajectory_msgs/html/msg/JointTrajectory.html
http://docs.ros.org/api/trajectory_msgs/html/msg/JointTrajectoryPoint.html

There are a few things that we would like you to take note. 
  * Your provided "waypoint" do not have to be close to each other. The trajectory controller in ROS uses cubic splines by default to interpolate between your waypoints. You can change to quintic splines too, if you fill in accelerations. You can use this interface as a sanity check whether your trajectory planner in your coursework correctly works or not.
  * You must not forget to fill-in the timestamp in the header of your `JointTrajectory` message. This value will tell the controller when your defined trajectory starts.
  * For each `JointTrajectoryPoint` in your `JointTrajectory`, you must fill-in time_from_start as well. This will tell the controller at what time counting from the start of the trajectory one specific waypoint should be reached.

## Optional: Visualising your path
You can visualise your path by running
`rosrun youbot_trail_rviz youbot_trail_rviz_node 1`
You can ignore the argument `1` for now because this will generate the waypoint you have to reach for the coursework.
