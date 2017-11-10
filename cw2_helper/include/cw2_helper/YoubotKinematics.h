#ifndef CW2_HELPER_YOUBOTKINE_H
#define CW2_HELPER_YOUBOTKINE_H

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
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
    double DH_params[5][4];

public:
    void init();
    void publish_joint_trajectory(trajectory_msgs::JointTrajectoryPoint joint_trajectory);

};

#endif //CW2_HELPER_YOUBOTKINE_H
