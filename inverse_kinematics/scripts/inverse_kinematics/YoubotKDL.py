#! /usr/bin/env python

import rospy
from math import pi
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
import PyKDL
import tf2_ros

from YoubotKinematics import YoubotKinematics
from tf_conversions import posemath

import tf2_kdl


class YoubotKDL(YoubotKinematics):
    def __init__(self):
        super(YoubotKDL, self).__init__()

        # PyKDL
        self.kine_chain = PyKDL.Chain()
        self.setup_kdl_chain()
        self.current_joint_position = PyKDL.JntArray(self.kine_chain.getNrOfJoints())
        self.current_pose = PyKDL.Frame()

        self.fk_solver = PyKDL.ChainFkSolverPos_recursive(self.kine_chain)
        self.ik_solver = PyKDL.ChainIkSolverPos_LMA(self.kine_chain)
        self.jac_calc = PyKDL.ChainJntToJacSolver(self.kine_chain)

    def setup_kdl_chain(self):
        self.kine_chain.addSegment(PyKDL.Segment(PyKDL.Joint(PyKDL.Joint.RotZ), PyKDL.Frame().DH(0.033, pi / 2, 0.147, 0)))
        self.kine_chain.addSegment(PyKDL.Segment(PyKDL.Joint(PyKDL.Joint.RotZ), PyKDL.Frame().DH(0.155, 0, 0, pi / 2)))
        self.kine_chain.addSegment(PyKDL.Segment(PyKDL.Joint(PyKDL.Joint.RotZ), PyKDL.Frame().DH(0.135, 0, 0, 0)))
        self.kine_chain.addSegment(PyKDL.Segment(PyKDL.Joint(PyKDL.Joint.RotZ), PyKDL.Frame().DH(0.0, pi / 2, 0, pi / 2)))
        self.kine_chain.addSegment(PyKDL.Segment(PyKDL.Joint(PyKDL.Joint.RotZ), PyKDL.Frame().DH(0, 0, 0.218, pi)))

    def joint_state_callback(self, msg):
        # Copies joint position into KDL JntArray
        for i in range(0, 5):
            self.current_joint_position[i] = msg.position[i]

    def forward_kinematics(self, joint, pose):
        self.fk_solver.JntToCart(joint, pose)

    def broadcast_pose(self, pose):
        trans = TransformStamped()

        tf2_kdl.do_transform_frame(pose, trans)
        trans.header.frame_id = "arm_link_0"
        trans.header.stamp = rospy.Time.now()
        trans.child_frame_id = "arm_end_effector"

        self.pose_broadcaster.sendTransform(trans)

    def get_jacobian(self, joint):
        jac = PyKDL.Jacobian(self.kine_chain.getNrOfJoints())
        self.jac_calc.JntToJac(joint, jac)
        return jac

    def inverse_kinematics_closed(self, desired_pose):

        required_joint = PyKDL.JntArray(self.kine_chain.getNrOfJoints())
        # Current joint array, desired frame, required joint
        self.ik_solver.CartToJnt(self.current_joint_position, desired_pose, required_joint)

        return required_joint

    def run(self):
        rate = rospy.Rate(200)
        while not rospy.is_shutdown():
            # Find your current cartesian pose
            self.forward_kinematics(self.current_joint_position, self.current_pose)
            self.broadcast_pose(self.current_pose)

            # Ikine

            rate.sleep()



if __name__ == '__main__':
    rospy.init_node('youbot')
    youbot = YoubotKDL()

    youbot.run()
