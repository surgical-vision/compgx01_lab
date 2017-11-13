#ifndef INVERSE_KINEMATICS_YOUBOTKINEMATICS_H
#define INVERSE_KINEMATICS_YOUBOTKINEMATICS_H

#include <ros/ros.h>
#include <kdl/chain.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <tf2_kdl/tf2_kdl.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>

using namespace Eigen;

class YoubotKinematics
{

protected:
    ros::NodeHandle n;
    ros::Publisher traj_publisher;
    ros::Subscriber subscriber_joint_state;
    tf2_ros::TransformBroadcaster pose_broadcaster;
    double DH_params[5][5];

public:
    virtual void init();
    void publish_joint_trajectory(trajectory_msgs::JointTrajectoryPoint joint_trajectory);
    virtual void joint_state_callback(const sensor_msgs::JointState::ConstPtr &q);
    virtual KDL::Frame forward_kinematics(KDL::JntArray current_joint_position, KDL::Frame current_pose);
    virtual void broadcast_pose(KDL::Frame current_pose);
    virtual KDL::Jacobian get_jacobian(KDL::JntArray current_joint_position);
    void inverse_kinematics_jac();
    virtual KDL::JntArray inverse_kinematics_closed(KDL::Frame desired_pose);

};

#endif //INVERSE_KINEMATICS_YOUBOTKINEMATICS_H
