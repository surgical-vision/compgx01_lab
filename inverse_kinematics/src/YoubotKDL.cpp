#include <inverse_kinematics/YoubotKDL.h>

int YoubotKDL::init()
{

    setup_kdl_chain();
    current_joint_position = KDL::JntArray(kine_chain.getNrOfJoints());

    KDL::ChainFkSolverPos_recursive fk_solver = KDL::ChainFkSolverPos_recursive(kine_chain);
    KDL::ChainIkSolverPos_LMA ik_solver = KDL::ChainIkSolverPos_LMA(kine_chain);
    KDL::ChainJntToJacSolver jac_solver = KDL::ChainJntToJacSolver(kine_chain);

    ros::Subscriber joint_sub = n.subscribe<sensor_msgs::JointState>("/joint_states", 1, &YoubotKDL::joint_state_callback,
                                                                                                     this);
}

KDL::Jacobian YoubotKDL::get_jacobian(KDL::ChainJntToJacSolver jac_solver)
{

    KDL::Jacobian jac = KDL::Jacobian(kine_chain.getNrOfJoints());
    jac_solver.JntToJac(current_joint_position, jac);
    return jac;
}

int YoubotKDL::forward_kinematics(KDL::JntArray current_joint_position, KDL::Frame current_pose, KDL::ChainFkSolverPos_recursive fk_solver)
{
    return fk_solver.JntToCart(current_joint_position, current_pose, 6);
}

void YoubotKDL::joint_state_callback(const sensor_msgs::JointState::ConstPtr &q)
{
    current_joint_position.data(0) = 0.0;
    for (int i = 0; i < 5; i++)
        current_joint_position.data(i + 1) = q->position.at(i);
}

void YoubotKDL::setup_kdl_chain()
{
    kine_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame::DH(-0.024,   M_PI,  0.096, 160*M_PI/180)));
    kine_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH( 0.033, M_PI_2, -0.019, M_PI + (169 * M_PI / 180.0))));
    kine_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(-0.155,      0,      0, M_PI_2 + (-65.0 * M_PI / 180.0))));
    kine_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(-0.135,      0,      0, (146 * M_PI / 180.0))));
    kine_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH( 0.002, M_PI_2,      0, M_PI_2 + (-102.5 * M_PI / 180.0))));
    kine_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(     0,   M_PI,  -0.21, -150*M_PI/180+(167.5 * M_PI / 180.0))));
}

KDL::JntArray YoubotKDL::inverse_kinematics_closed(KDL::Frame desired_pose, KDL::ChainIkSolverPos_LMA ik_solver)
{
    KDL::JntArray required_joint = KDL::JntArray(kine_chain.getNrOfJoints());
    ik_solver.CartToJnt(current_joint_position, desired_pose, required_joint);

    return required_joint;
}

void YoubotKDL::broadcast_pose(KDL::Frame current_pose)
{
    tr_stamped = tf2::kdlToTransform(current_pose);
    tr_stamped.header.stamp = ros::Time::now();
    tr_stamped.header.frame_id = "base_link";
    tr_stamped.child_frame_id = "arm_end_effector";

    pose_broadcaster.sendTransform(tr_stamped);

}