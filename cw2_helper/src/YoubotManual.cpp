#include <cw2_helper/YoubotManual.h>

int YoubotManual::init()
{
    ros::Subscriber joint_sub_kine = n.subscribe<sensor_msgs::JointState>("/joint_states", 1,
                                                                          &YoubotManual::joint_state_callback,
                                                                          this);
    std::cout << "TEST_MANUAL" << std::endl;
}

void YoubotManual::joint_state_callback(const sensor_msgs::JointState::ConstPtr &q) {
    for (int i = 0; i < 5; i++)
        current_joint_position(i) = q->position.at(i);
}

MatrixXd YoubotManual::get_jacobian(VectorXd current_pose)
{
    //Add jacobian code. (without using KDL libraries)
}

VectorXd YoubotManual::inverse_kinematics_closed(Matrix4d desired_pose)
{
    //Add closed-form inverse kinematics code. (without using KDL libraries)
}

VectorXd YoubotManual::inverse_kinematics_jac(Matrix4d desired_pose, VectorXd current_joint_position)
{
    //Add iterative inverse kinematics code. (without using KDL libraries)
}

Matrix4d YoubotManual::forward_kinematics(VectorXd current_joint_position)
{
    //Add forward kinematics code. (without using KDL libraries)
}
