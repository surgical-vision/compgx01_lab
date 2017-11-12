#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from math import pi
import tf2_ros
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class YoubotKinematics(object):
    def __init__(self):
        self.dh_params = [[0.033, pi / 2, 0.147, (170 * pi/ 180), -1],
                          [0.155, 0, 0, (65 * pi/ 180) + pi / 2, -1],
                          [0.135, 0, 0, (-146 * pi/ 180), -1],
                          [0.0, pi / 2, 0, (102.5 * pi/ 180) + pi / 2, -1],
                          [0, 0, 0.183, (167.5 * pi/ 180) + pi, -1]]

        # Setup the subscribers for the joint states
        self.subscriber_joint_state_ = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback,
                                                        queue_size=5)
        # TF2 broadcaster
        self.pose_broadcaster = tf2_ros.TransformBroadcaster()

        # Trajectory Publisher
        self.traj_publisher = rospy.Publisher('/EffortJointInterface_trajectory_controller/command', JointTrajectory,
                                         queue_size=3)
        self.joint_names = rospy.get_param('/EffortJointInterface_trajectory_controller/joints')

    def joint_state_callback(self, msg):
        raise NotImplementedError()

    def forward_kinematics(self, joint, pose):
        raise NotImplementedError()

    def broadcast_pose(self, pose):
        raise NotImplementedError()

    def get_jacobian(self, joint):
        raise NotImplementedError()

    def inverse_kinematics_jac(self, desired_pose, current_joint):
        raise NotImplementedError()

    def inverse_kinematics_closed(self, desired_pose):
        raise NotImplementedError()

    def publish_joint_trajectory(self, joint_trajectory, time_from_start=537230041):
        # Most simplistic implementation possible
        # Takes a single array of joint values for each joint

        # Recommend reimplementing this
        msg = JointTrajectory()
        msg.header.stamp = rospy.Time.now()

        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = joint_trajectory
        point.time_from_start.nsecs = time_from_start

        msg.points.append(point)

        self.traj_publisher.publish(msg)
