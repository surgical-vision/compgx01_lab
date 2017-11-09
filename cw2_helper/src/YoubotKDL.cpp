#include <cw2_helper/YoubotKDL.h>

int YoubotKDL::init() {
    KDL::ChainFkSolverPos_recursive fk_solver = KDL::ChainFkSolverPos_recursive(kine_chain);
    KDL::ChainIkSolverPos_LMA ik_solver = KDL::ChainIkSolverPos_LMA(kine_chain);
    KDL::ChainJntToJacSolver jac_solver = KDL::ChainJntToJacSolver(kine_chain);

    ros::Subscriber joint_sub_KDL = n.subscribe<sensor_msgs::JointState>("/joint_states", 1,
                                                                         &YoubotKDL::joint_state_callback, this);
    setup_kdl_chain();
    current_joint_position = KDL::JntArray(kine_chain.getNrOfJoints());
}

void YoubotKDL::setup_kdl_chain() {
    //Add the code
}

void YoubotKDL::joint_state_callback(const sensor_msgs::JointState::ConstPtr &q) {
    current_joint_position.data(0) = 0.0;
    for (int i = 0; i < 5; i++)
        current_joint_position.data(i + 1) = q->position.at(i);
}

KDL::Jacobian YoubotKDL::get_jacobian(KDL::ChainJntToJacSolver jac_solver) {

    //Add KDL Jacobian code
}

KDL::JntArray YoubotKDL::inverse_kinematics_jac(KDL::JntArray current_joint_position)
{
    //Add KDL iterative inverse kinematics
}

KDL::JntArray YoubotKDL::inverse_kinematics_closed(KDL::Frame desired_pose)
{
    //Add KDL closed form inverse kinematics
}

KDL::Frame YoubotKDL::forward_kinematics(KDL::Frame desired_pose, KDL::JntArray current_joint_position)
{
    //Add Forward kinematics
}