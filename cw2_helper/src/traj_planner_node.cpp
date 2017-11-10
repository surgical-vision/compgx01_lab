#include <ros/ros.h>
#include <cw2_helper/YoubotIkine.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

trajectory_msgs::JointTrajectoryPoint traj_pt;

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
    bag.open(MY_BAG_PATH, rosbag::bagmode::Read);

    std::vector<std::string> topics;
    MatrixXd p;
#ifdef n_data_Q4a
    topics.push_back(std::string("joint_data"));
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    foreach(rosbag::MessageInstance const m, view)
    {
        sensor_msgs::JointState::ConstPtr j = m.instantiate<sensor_msgs::JointState>();

        if (j != NULL)
        {
            //Fill in the code to retrieve data from a bag
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
            //Fill in the code to retrieve data from a bag
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
           //Fill in the code to retrieve data from a bag
        }
    }

#endif

    bag.close();

    return p;

}

void traj_q4a (MatrixXd checkpoint)
{

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

    YoubotIkine youbot_kine;

    youbot_kine.init();

    MatrixXd check_point_matrix = get_checkpoint();

    int i = 0;
    while (ros::ok())
    {
        traj_q4a(check_point_matrix, i);

        youbot_kine.publish_joint_trajectory(traj_pt);

        ros::spinOnce();
        i++;

        if (i >= check_point_matrix.cols())
            i = 0;

        sleep(3);
    }

    return 0;
}
