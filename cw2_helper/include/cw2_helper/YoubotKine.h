#ifndef CW2_HELPER_YOUBOTKINE_H
#define CW2_HELPER_YOUBOTKINE_H

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_kdl/tf2_kdl.h>
#include <kdl/chain.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainjnttojacsolver.hpp>

class YoubotKine
{

protected:
    ros::NodeHandle n;
    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped tr_stamped;
    tf2::Quaternion q;
    static double DH_params[4][5];
};

#endif //CW2_HELPER_YOUBOTKINE_H