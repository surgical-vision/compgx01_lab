#include <ros/ros.h>
#include <cw2_helper/YoubotKDL.h>
#include <cw2_helper/YoubotManual.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

trajectory_msgs::JointTrajectoryPoint traj_pt;
trajectory_msgs::JointTrajectory my_traj;

////Uncomment this line if you are working on question 4a
#define n_data_Q4a 10
////Uncomment this line if you are working on question 4b
//#define n_data_Q4b 7
////Uncomment this line if you are working on question 4c,d or 6
//#define n_data_Q4cd 3

MatrixXd get_checkpoint()
{
    rosbag::Bag bag;

    ////Change the name of the file to the corresponding question.
    bag.open("/home/kpach/catkin_ws/src/youbot_stack/youbot_simulator/bags/data_q4a.bag", rosbag::bagmode::Read);

    std::vector<std::string> topics;
    std::vector<double> entries;

#ifdef n_data_Q4a
    topics.push_back(std::string("joint_data"));
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    foreach(rosbag::MessageInstance const m, view)
    {
        sensor_msgs::JointState::ConstPtr j = m.instantiate<sensor_msgs::JointState>();

        if (j != NULL)
        {
            for (int k = 0; k < 5; k++)
                entries.push_back(j->position.at(k));
            for (int k = 0; k < 5; k++)
                entries.push_back(j->velocity.at(k));
        }
    }

    int cc = 0;

    MatrixXd p(n_data_Q4a, entries.size()/n_data_Q4a);

    for (int i = 0; i < n_data_Q4a; i++)
    {
        for (int j = 0; j < entries.size()/n_data_Q4a; j++)
        {
            p(i, j) = entries.at(cc)*M_PI/180.0;
            cc++;
        }
    }

#elif n_data_Q4b
    topics.push_back(std::string("target_tf"));
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    foreach(rosbag::MessageInstance const m, view)
    {
        geometry_msgs::Transform::ConstPtr j = m.instantiate<geometry_msgs::Transform>();

        if (j != NULL)
        {
            entries.push_back(j->translation.x);
            entries.push_back(j->translation.y);
            entries.push_back(j->translation.z);

            entries.push_back(j->rotation.x);
            entries.push_back(j->rotation.y);
            entries.push_back(j->rotation.z);
            entries.push_back(j->rotation.w);
        }
    }

    int cc = 0;
    MatrixXd p(n_data_Q4b, entries.size()/n_data_Q4b);

    for (int i = 0; i < n_data_Q4b; i++)
    {
        for (int j = 0; j < entries.size()/n_data_Q4b; j++)
        {
            p(i, j) = entries.at(cc);
            cc++;
        }
    }

#elif n_data_Q4cd
    topics.push_back(std::string("target_position"));
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    foreach(rosbag::MessageInstance const m, view)
    {
        geometry_msgs::Point::ConstPtr j = m.instantiate<geometry_msgs::Point>();

        if (j != NULL)
        {
            entries.push_back(j->x);
            entries.push_back(j->y);
            entries.push_back(j->z);
        }
    }

    int cc = 0;
    MatrixXd p(n_data_Q4cd, entries.size()/n_data_Q4cd);

    for (int i = 0; i < n_data_Q4cd; i++)
    {
        for (int j = 0; j < entries.size()/n_data_Q4cd; j++)
        {
            p(i, j) = entries.at(cc);
            cc++;
        }
    }
#endif

    bag.close();

    return p;

}

void traj_q4a (MatrixXd checkpoint, int i)
{

    my_traj.points.resize(1);

    my_traj.points[0].positions.resize(5);
    my_traj.points[0].velocities.resize(5);

    my_traj.points[0].positions[0] = checkpoint(0, i) + 169*M_PI/180.0;
    my_traj.points[0].positions[1] = checkpoint(1, i) + 65.0*M_PI/180.0;
    my_traj.points[0].positions[2] = 146.0*M_PI/180.0 - checkpoint(2, i);
    my_traj.points[0].positions[3] = checkpoint(3, i) + 102.5*M_PI/180.0;
    my_traj.points[0].positions[4] = checkpoint(4, i) + 167.5*M_PI/180.0;

    if ((i == 0) || (i == checkpoint.cols() - 1))
        for (int j = 0; j < 5; j++)
            my_traj.points[0].velocities[j] = 0;
    else
        for (int j = 0; j < 5; j++)
            my_traj.points[0].velocities[j] = checkpoint(j + 5, i);

    my_traj.points[0].time_from_start = ros::Duration(10.0);

}

void traj_q4b (MatrixXd checkpoint)
{

}

void traj_q4c (MatrixXd checkpoint)
{

}

void traj_q4d (MatrixXd checkpoint)
{

}

void traj_qextra (MatrixXd checkpoint)
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Youbot");
    YoubotKDL youbot_kdl;
    YoubotManual youbot_kine;

    //youbot_kdl.init();
    //youbot_kine.init();

    ros::NodeHandle traj_nh;
    ros::Publisher traj_pub = traj_nh.advertise<trajectory_msgs::JointTrajectory>("/EffortJointInterface_trajectory_controller/command", 1);

    MatrixXd check_point_matrix = get_checkpoint();

    my_traj.joint_names.resize(5);

    my_traj.joint_names[0] = "arm_joint_1";
    my_traj.joint_names[1] = "arm_joint_2";
    my_traj.joint_names[2] = "arm_joint_3";
    my_traj.joint_names[3] = "arm_joint_4";
    my_traj.joint_names[4] = "arm_joint_5";

    int i = 0;
    while (ros::ok())
    {
        traj_q4a(check_point_matrix, i);
        traj_pub.publish(my_traj);
        ros::spinOnce();
        i++;

        if (i >= check_point_matrix.cols())
            break;

        sleep(3);
    }

    return 0;
}