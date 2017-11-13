#include <tf2_ros/transform_broadcaster.h>
#include <rosbag/bag.h>
#include <ros/ros.h>
#include <inverse_kinematics/YoubotKDL.h>

int main (int argc, char **argv)
{
    ros::init(argc, argv, "youbot_bag_follower");

    YoubotKDL youbot;

    youbot.init();

    rosbag::Bag bag;
    bag.open("test.bag", rosbag::bagmode::Write);

    while (ros::ok())
    {
        KDL::Frame current_pose = youbot.forward_kinematics(youbot.current_joint_position, youbot.current_pose);
        youbot.broadcast_pose(current_pose);

        ros::Duration(0.01).sleep();
    }

    return 1;
}