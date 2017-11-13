#include <inverse_kinematics/YoubotKinematics.h>

void YoubotKinematics::init()
{
    DH_params[0][0] = 0.033;    DH_params[1][0] = 0.155;  DH_params[2][0] = 0.135;  DH_params[3][0] = 0.0;    DH_params[4][0] = 0.0;
    DH_params[0][1] = M_PI_2;   DH_params[1][1] = 0.0;    DH_params[2][1] = 0.0;    DH_params[3][1] = M_PI_2; DH_params[4][1] = 0.0;
    DH_params[0][2] = 0.147;    DH_params[1][2] = 0.0;    DH_params[2][2] = 0.0;    DH_params[3][2] = 0.0;    DH_params[4][2] = 0.183;
    DH_params[0][3] = 170*M_PI/180;
    DH_params[1][3] = 65*M_PI/180+M_PI_2;
    DH_params[2][3] = -146*M_PI/180;
    DH_params[3][3] = M_PI_2+102.5*M_PI/180;
    DH_params[4][3] = M_PI+167.5*M_PI/180;
    DH_params[0][4] = DH_params[1][4] = DH_params[2][4] = DH_params[3][4] = DH_params[4][4] = -1;

    this->subscriber_joint_state = this->n.subscribe<sensor_msgs::JointState>("/joint_states", 5, &YoubotKinematics::joint_state_callback,
    this);

    this->traj_publisher = this->n.advertise<trajectory_msgs::JointTrajectory>("/EffortJointInterface_trajectory_controller/command", 3);

}

void YoubotKinematics::joint_state_callback(const sensor_msgs::JointState::ConstPtr &q)
{
}

KDL::Frame YoubotKinematics::forward_kinematics(KDL::JntArray current_joint_position, KDL::Frame current_pose)
{
}

void YoubotKinematics::broadcast_pose(KDL::Frame current_pose)
{
}

KDL::Jacobian YoubotKinematics::get_jacobian(KDL::JntArray current_joint_position)
{
}

void YoubotKinematics::inverse_kinematics_jac()
{
    //Do something
}

KDL::JntArray YoubotKinematics::inverse_kinematics_closed(KDL::Frame desired_pose)
{
}

void YoubotKinematics::publish_joint_trajectory(trajectory_msgs::JointTrajectoryPoint joint_trajectory)
{

    int time_from_start = 537230041;

    trajectory_msgs::JointTrajectory msg;

    msg.header.stamp = ros::Time::now();
    msg.joint_names.push_back("arm_joint_1");
    msg.joint_names.push_back("arm_joint_2");
    msg.joint_names.push_back("arm_joint_3");
    msg.joint_names.push_back("arm_joint_4");
    msg.joint_names.push_back("arm_joint_5");

    joint_trajectory.time_from_start.nsec = time_from_start;

    msg.points.push_back(joint_trajectory);

    this->traj_publisher.publish(msg);
}