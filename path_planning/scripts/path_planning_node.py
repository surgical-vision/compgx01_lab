#!/usr/bin/env python

import rospkg
import rosbag
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


if __name__ == '__main__':
    rospy.init_node('path_planning_node')

    rospack = rospkg.RosPack()
    traj_publisher = rospy.Publisher('/EffortJointInterface_trajectory_controller/command', JointTrajectory, queue_size=3)

    ##TODO: Write a code to extract data from your bag.

    my_traj = JointTrajectory()
    my_waypoint = JointTrajectoryPoint()

    ##TODO: Write a code to create your trajectory from bag data.

    traj_publisher.publish(my_traj)
    rospy.sleep(1)
    raw_input('Press enter to rerun the trajectory\n')