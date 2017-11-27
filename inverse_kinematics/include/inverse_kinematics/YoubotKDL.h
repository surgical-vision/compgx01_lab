#ifndef INVERSE_KINEMATICS_YOUBOTKDL_H
#define INVERSE_KINEMATICS_YOUBOTKDL_H

#include "inverse_kinematics/YoubotKinematics.h"

class YoubotKDL : public YoubotKinematics
{
public:
    KDL::Frame current_pose;
    KDL::Chain kine_chain;
    KDL::JntArray current_joint_position;

    void init();
    void broadcast_pose(KDL::Frame current_pose);
    void setup_kdl_chain();
    KDL::Jacobian get_jacobian(KDL::JntArray current_joint_position);
    KDL::JntArray inverse_kinematics_closed(KDL::Frame desired_pose);
    KDL::Frame forward_kinematics(KDL::JntArray current_joint_position, KDL::Frame current_pose);
    void joint_state_callback(const sensor_msgs::JointState::ConstPtr &q);

};

#endif //INVERSE_KINEMATICS_YOUBOTKDL_H
