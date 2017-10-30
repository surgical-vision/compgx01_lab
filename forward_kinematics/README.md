# COMPGX01 - Forward Kinematics

## Introduction to TF2
TF2 is the transform library in ROS, it provides an interface to observe and modify a tree of transformations.

### Listening to TF
This section will look at how to get information from TF, a launch file has been provided that will start up the a tf broadcaster, youbot simulator and controller. 
  * Launch this using `roslaunch forward_kinematics listening_to_tf.launch`
    * You can use the same options as with the simulator launch eg `rviz:=true`
  * Checking the TF data
    * Firstly try `rostopic echo tf` a lot of messages get published to it so you probably won't be able to see much info, like most command line applications press ctrl and c to quit it.
    * A better way to check tf is to use `rosrun tf tf_echo arm_link_0 static_frame`
    * Alternatively you can use `rosrun tf2_tools view_frames.py`, this will make a pdf in the current directory showing all of the frames and their relationships to each other, open it in a pdf viewer, try `evince frames.pdf` on Ubuntu
    * Lastly in rviz try adding the tf tree, the add button will be in the toolbar on the left.

### Writing a TF listener
This section will look at writiing tf functionalities into your application starting with a listener.
  * Take a look through the tutorials:
    * [C++](http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28C%2B%2B%29)
    * [Python](http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28Python%29)
  * Write a listener to find the transform between `arm_link_0` and `static_frame`
    * Implement a service that allows you to change the child and parent frames
    * Publish the euclidean distance between the frames to /object_distance
  * This output of the above listener should be very boring as everything is static, set the parent frame as the end of the youbot (check out the frames pdf if you don't know what it's called)
  * Use your trajectory publisher from last week to start moving the youbot and see this distance change.
    * If you haven't got that working try `rostopic pub -1 /EffortJointInterface_J{joint_number}_controller/command std_msgs/Float64 '{commanded_position}'`
      Remember to change `{joint_number}` and `{commanded_position}`

## Forward Kinematics
This section will cover how to convert from joint space to cartesian space. 

### Robot Description
ROS has a format for describing robots, the Universal Robot Description Format (URDF), it is loaded everytime the youbot simulator is run. The URDF file contains information on the links and joints of the robot.
  * The URDF file is loaded at runtime to the parameter robot_description
  * This can be found using the commandline with `rosparam get robot_description`
  * Identify the joints in the description, you can either do this: 
    * Programatically by creating a node to parse robot description and find each joint (the file is an xml), this would allow you to use this for any robot_description
    * Manually
  * Make this into a function that accepts the joint values as a param then returns the pose of the end-effector
  
### Cartesian Pose
Combine this with the joint subscriber you made last week to make a node that:
  * Subscribes to the joint state of the robot
  * Computes the cartesian pose
  * Publishes the current pose as a PoseStamped message 
