#include <cw2_helper/YoubotIkine.h>

int YoubotIkine::init()
{
    YoubotKinematics::init();
    subscriber_joint_state = n.subscribe<sensor_msgs::JointState>("/joint_states", 5, &YoubotIkine::joint_state_callback,
                                                                  this);

    current_joint_position.resize(5);
    desired_joint_position.resize(5);
    jacobian.resize(6, 5);
}

void YoubotIkine::joint_state_callback(const sensor_msgs::JointState::ConstPtr &q) {
    for (int i = 0; i < 5; i++)
        current_joint_position(i) = q->position.at(i);
}

MatrixXd YoubotIkine::get_jacobian(VectorXd current_pose)
{
    //Add jacobian code. (without using KDL libraries)
}

VectorXd YoubotIkine::inverse_kinematics_closed(Matrix4d desired_pose)
{
    //Add closed-form inverse kinematics code. (without using KDL libraries)
}

VectorXd YoubotIkine::inverse_kinematics_jac(Matrix4d desired_pose, VectorXd current_joint_position)
{
    //Add iterative inverse kinematics code. (without using KDL libraries)
}

Matrix4d YoubotIkine::forward_kinematics(VectorXd current_joint_position)
{
    //Add forward kinematics code. (without using KDL libraries)
}

bool YoubotIkine::check_singularity(VectorXd joint_position)
{
    //Add singularity checker. (without using KDL libraries)
}