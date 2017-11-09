#! /usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from math import pi
import tf2_ros

class YoubotKine:
    def __init__(self):
        self.dh_params = [[0.033, pi / 2, 0.147, 0],
                          [0.155, 0, 0, pi / 2],
                          [0.135, 0, 0, 0],
                          [0.0, pi / 2, 0, pi / 2],
                          [0, 0, 0.218, pi]]

        # Setup the subscribers for the joint states
        self.subscriber_joint_state_ = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback,
                                                        queue_size=5)
        # TF2 broadcaster
        self.pose_broadcaster = tf2_ros.TransformBroadcaster()

    def joint_state_callback(self, msg):
        return 0

    def forward_kinematics(self, joint):

        return 0

    def get_jacobian(self, joint):

        return 0

    def inverse_kinematics_jac(self, desired_pose, current_joint):

        return 0

    def inverse_kinematics_closed(self, desired_pose):

        return 0
