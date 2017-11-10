#ifndef CW2_HELPER_YOUBOTKINE_H
#define CW2_HELPER_YOUBOTKINE_H

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>

using namespace Eigen;

class YoubotKinematics
{

protected:
    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped tr_stamped;
    tf2::Quaternion q;
    static double DH_params[4][5];

public:
    ros::NodeHandle n;
};

#endif //CW2_HELPER_YOUBOTKINE_H