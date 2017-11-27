#include <inverse_kinematics/YoubotKDL.h>

void YoubotKDL::init()
{
    YoubotKinematics::init();
    setup_kdl_chain();
    this->current_joint_position = KDL::JntArray(this->kine_chain.getNrOfJoints());
}

KDL::Jacobian YoubotKDL::get_jacobian(KDL::JntArray current_joint_position)
{

    KDL::ChainJntToJacSolver jac_solver = KDL::ChainJntToJacSolver(this->kine_chain);
    KDL::Jacobian jac = KDL::Jacobian(this->kine_chain.getNrOfJoints());
    jac_solver.JntToJac(current_joint_position, jac);
    return jac;
}

KDL::Frame YoubotKDL::forward_kinematics(KDL::JntArray current_joint_position, KDL::Frame current_pose)
{
    KDL::ChainFkSolverPos_recursive fk_solver = KDL::ChainFkSolverPos_recursive(this->kine_chain);
    fk_solver.JntToCart(current_joint_position, current_pose, 5);
    return current_pose;
}

void YoubotKDL::joint_state_callback(const sensor_msgs::JointState::ConstPtr &q)
{
    for (int i = 0; i < 5; i++)
        this->current_joint_position.data(i) = q->position.at(i);
}

void YoubotKDL::setup_kdl_chain() {
    for (int i = 0; i < 5; i++)
        this->kine_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Vector(), KDL::Vector(0, 0, DH_params[i][4]), KDL::Joint::RotAxis),
                                                 KDL::Frame::DH(DH_params[i][0], DH_params[i][1], DH_params[i][2],
                                                                DH_params[i][3])));
}

KDL::JntArray YoubotKDL::inverse_kinematics_closed(KDL::Frame desired_pose)
{

    KDL::ChainIkSolverPos_LMA ik_solver = KDL::ChainIkSolverPos_LMA(this->kine_chain);
    KDL::JntArray required_joint = KDL::JntArray(this->kine_chain.getNrOfJoints());
    ik_solver.CartToJnt(this->current_joint_position, desired_pose, required_joint);

    return required_joint;
}

void YoubotKDL::broadcast_pose(KDL::Frame current_pose)
{
    geometry_msgs::TransformStamped trans;

    trans = tf2::kdlToTransform(current_pose);
    trans.header.stamp = ros::Time::now();
    trans.header.frame_id = "base_link";
    trans.child_frame_id = "arm_end_effector";

    this->pose_broadcaster.sendTransform(trans);

}
