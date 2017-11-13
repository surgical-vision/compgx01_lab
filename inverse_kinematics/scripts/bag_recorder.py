#!/usr/bin/env python

import rospkg

import rosbag
import rospy
import tf2_kdl
import tf2_ros

from inverse_kinematics.YoubotKDL import YoubotKDL

if __name__ == '__main__':
    rospy.init_node('youbot_bag_follower')
    youbot = YoubotKDL()

    rospack = rospkg.RosPack()

    # get the file path for inverse_kinematics
    path = rospack.get_path('inverse_kinematics')
    bag = rosbag.Bag(path + '/bags/test.bag')
    # Load the bag here

    while not rospy.is_shutdown():
	# Move to new position
	youbot.publish_joint_trajectory([0, 0, 0, 0, 0])

        # Find your current cartesian pose
        youbot.forward_kinematics(youbot.current_joint_position, youbot.current_pose)
        trans = youbot.broadcast_pose(youbot.current_pose)
	
	# Write to ROSbag
	rosbag.write('desired_pose', trans) 


        rospy.sleep(1)
