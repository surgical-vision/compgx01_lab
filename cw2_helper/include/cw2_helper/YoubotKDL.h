#ifndef CW2_HELPER_YOUBOTKDL_H
#define CW2_HELPER_YOUBOTKDL_H

#include <cw2_helper/YoubotKine.h>

class YoubotKDL : public YoubotKine
{

private:
    KDL::Frame current_pose, desired_pose;
    KDL::Chain kine_chain;
    KDL::JntArray current_joint_position, desired_joint_position;

public:

    int init();
    void setup_kdl_chain();
    void joint_state_callback(const sensor_msgs::JointState::ConstPtr &q);
    KDL::Jacobian get_jacobian(KDL::ChainJntToJacSolver jac_solver);
    KDL::JntArray inverse_kinematics_jac(KDL::JntArray current_joint_position);
    KDL::JntArray inverse_kinematics_closed(KDL::Frame desired_pose);
    KDL::Frame forward_kinematics(KDL::Frame desired_pose, KDL::JntArray current_joint_position);

};

#endif //CW2_HELPER_YOUBOTKDL_H




