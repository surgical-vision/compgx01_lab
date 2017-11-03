#! /usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import PyKDL


class Youbot:
    def __init__(self):
        # Setup the subscribers for the joint states
        self.subscriber_joint_state_ = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback, queue_size=5)

        self.current_joint_position = []

    def setup_kdl_chain(self):
        kine_chain = PyKDL.Chain()

        kine_chain.addSegment(PyKDL.Segment(PyKDL.Joint.RotZ(), PyKDL.Frame.DH(0.033)))


    def joint_state_callback(self, msg):
        msg = JointState()


if __name__ == '__main__':
    rospy.init_node('youbot')
    rospy.loginfo('\nWrite your forward kinematics here!\n')

