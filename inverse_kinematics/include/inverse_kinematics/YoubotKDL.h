#ifndef INVERSE_KINEMATICS_YOUBOTKDL_H
#define INVERSE_KINEMATICS_YOUBOTKDL_H

#include "inverse_kinematics/YoubotKinematics.h"

class YoubotKDL : public YoubotKinematics
{
public:
    KDL::Frame current_pose;
    KDL::Chain kine_chain;
    KDL::JntArray current_joint_position;
    KDL::ChainFkSolverPos_recursive fk_solver;
    KDL::ChainIkSolverPos_LMA ik_solver;
    KDL::ChainJntToJacSolver jac_solver;

    int init();
    void broadcast_pose(KDL::Frame current_pose);
    void setup_kdl_chain();
    KDL::Jacobian get_jacobian(KDL::ChainJntToJacSolver jac_solver);
    KDL::JntArray inverse_kinematics_closed(KDL::Frame desired_pose, KDL::ChainIkSolverPos_LMA ik_solver);
    int forward_kinematics(KDL::ChainFkSolverPos_recursive fk_solver, KDL::JntArray current_joint_position, KDL::Frame current_pose);
    void joint_state_callback(const sensor_msgs::JointState::ConstPtr &q);

};

#endif //INVERSE_KINEMATICS_YOUBOTKDL_H
