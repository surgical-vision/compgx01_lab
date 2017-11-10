#ifndef INVERSE_KINEMATICS_YOUBOTKDL_H
#define INVERSE_KINEMATICS_YOUBOTKDL_H

#include "inverse_kinematics/YoubotKinematics.h"

class YoubotKDL : public YoubotKinematics
{
private:

    KDL::Frame current_pose;
    KDL::Chain kine_chain;
    KDL::JntArray current_joint_position;


public:

    int init();
    void broadcast_pose();
    void setup_kdl_chain();
    KDL::Jacobian get_jacobian(KDL::ChainJntToJacSolver jac_solver);
    KDL::JntArray inverse_kinematics_closed(KDL::JntArray current_joint_position, KDL::Frame desired_pose, KDL::ChainIkSolverPos_LMA ik_solver);
    int forward_kinematics(KDL::JntArray current_joint_position);
    void joint_state_callback(const sensor_msgs::JointState::ConstPtr &q, KDL::ChainFkSolverPos_recursive fk_solver);

};

#endif //INVERSE_KINEMATICS_YOUBOTKDL_H
