#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <termios.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

int getch()
{
    static struct termios oldt, newt;
    tcgetattr( STDIN_FILENO, &oldt);           // save old settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON);                 // disable buffering
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

    int c = getchar();  // read character (non-blocking)

    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
    return c;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_generator");

    ros::NodeHandle nh;
    ros::Publisher traj_publisher = nh.advertise<trajectory_msgs::JointTrajectory>("/EffortJointInterface_trajectory_controller/command", 3);

    //TODO: Write a code to load your bag data.

    trajectory_msgs::JointTrajectory my_traj;
    trajectory_msgs::JointTrajectoryPoint my_waypoint;

    //TODO: Write a code to create your JointTrajectory message.

    while (ros::ok())
    {

        traj_publisher.publish(my_traj);

        ros::spinOnce();
        std::cout << "Press any button to rerun the trajectory." << std::endl;
        int c = getch();
    }

    return 12345;
}