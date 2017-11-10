#!/usr/bin/env python

import rospy
from math import pi
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion, Pose
import tf2_ros
import numpy as np
from inverse_kinematics.YoubotKinematics import YoubotKinematics


class YoubotTemplate(YoubotKinematics):
    def __init__(self):
        super(YoubotTemplate, self).__init__()

    def joint_state_callback(self, msg):
        
        for i in range(0, 5):
            print msg.position[i]

    def forward_kinematics(self, joint, pose):
        return Pose()

    def broadcast_pose(self, pose):
        trans = TransformStamped()

        # Fill in trans.transform with the pose you got from fkine
        
        trans.header.frame_id = "arm_link_0"
        trans.header.stamp = rospy.Time.now()
        trans.child_frame_id = "arm_end_effector"

        self.pose_broadcaster.sendTransform(trans)

    def get_jacobian(self, joint):
        jac = np.identity(5)
        return jac

    def inverse_kinematics_closed(self, desired_pose):

        return [0, 0, 0, 0, 0]

