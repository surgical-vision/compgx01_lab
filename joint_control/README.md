# COMPGX01 - Joint Control
Python and C++ files have been made in the scripts and src folder which you will need to implement. 
## Joint Subscriber
Task: Implement a ros node that: 
  * Subscribes to the topic `/joint_states` 
  * Prints to the terminal the current joint position every 2 seconds

Run: 
  * CPP: `rosrun joint_control joint_subscriber_node`
  * Python `rosrun joint_control joint_subscriber.py`
        
Hopefully Helpful Hints:
  * You can get info on the topic using `rostopic info {topic_name}`
  * If you want to check the name of the topics in use try `rostopic list`
    
## Joint Publisher
Task: Implement a ros node that:
  * Get the joint interface at startup (either using rosparams or args)
  * Apply a sin wave to the joint
  * Allow the joint interface and controller parameters to be modified by a ros service

Run: 
  * CPP: `rosrun joint_control joint_publisher_node`
  * Python `rosrun joint_control joint_publisher.py`
        
Hopefully Helpful Hints:
  * Use rostopic list to find the topics for the joint controllers
  * Look at the positition the robot starts in, it may be close to a joint limit 
		
## Extra (this will come in handy later):
  1. Launch the youbot simulator using:
        `roslaunch youbot_simulator youbot_sim.launch gui:=true trajectory_interface:=true`
  2. Look at the new topic for interfacing with the youbot
  3. Make a publisher that can control all the joints at once (apply a sin wave again just to test it out)
