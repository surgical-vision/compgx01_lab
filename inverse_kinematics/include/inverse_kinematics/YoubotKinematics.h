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
    geometry_msgs::TransformStamped tr_stamped;
    tf2::Quaternion q;
    double DH_params[5][5];

public:
    void init();
    void publish_joint_trajectory(trajectory_msgs::JointTrajectoryPoint joint_trajectory);
    virtual void joint_state_callback(const sensor_msgs::JointState::ConstPtr &q);
    void forward_kinematics();
    void broadcast_pose();
    void get_jacobian();
    void inverse_kinematics_jac();
    void inverse_kinematics_closed();

};

#endif //INVERSE_KINEMATICS_YOUBOTKINEMATICS_H
