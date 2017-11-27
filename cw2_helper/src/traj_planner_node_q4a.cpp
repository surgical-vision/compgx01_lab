#include <ros/ros.h>
#include <cw2_helper/YoubotIkine.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Point.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

trajectory_msgs::JointTrajectoryPoint traj_pt;

MatrixXd get_checkpoint()
{
    rosbag::Bag bag;

    std::vector<std::string> topics;
    MatrixXd p;


    bag.open(MY_BAG_PATH1, rosbag::bagmode::Read);
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
    
    bag.close();

    return p;

}

void traj_q4a (MatrixXd checkpoint)
{
    // Do something
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "youbot_traj_4a");

    YoubotIkine youbot_kine;

    youbot_kine.init();
    MatrixXd check_point_matrix = get_checkpoint();

    int i = 0;
    while (ros::ok())
    {
        youbot_kine.publish_joint_trajectory(traj_pt);

        ros::spinOnce();
        sleep(1);
    }

    return 0;
}
