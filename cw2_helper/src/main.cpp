#include <ros/ros.h>
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

class YouBot
{

protected:
    ros::NodeHandle n;
    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped tr_stamped;
    tf2::Quaternion q;
    double DH_params[4][5] = {{0.033, 0.155, 0.135, 0.0, 0.0},
                              {M_PI_2, 0.0, 0.0, M_PI_2, 0.0},
                              {0.147, 0.0, 0.0, 0.0, 0.218},
                              {0.0, M_PI_2, 0.0, M_PI_2, M_PI}};

public:


};

class YouBotKDL : public YouBot
{

private:
    KDL::Frame current_pose;
    KDL::Chain kine_chain;
    KDL::JntArray current_joint_position;

public:

};

class YouBotKine : public YouBot
{

private:

public:

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "youbot");
    ros::NodeHandle n =
    return 0;
}