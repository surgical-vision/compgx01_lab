#include "ros/ros.h"
#include "compgx01_msgs/quat2Euler.h"

bool convert_quat2Euler(compgx01_msgs::quat2Euler::Request  &req,
         compgx01_msgs::quat2Euler::Response &res)
{
    //TO DO:
    //Put your rotation representations converter here..... (This is a template for only CW1-Q7a.)
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "quat2Euler_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("quat2Euler_converter", convert_quat2Euler);
    ROS_INFO("Ready to convert from a quaternion to Euler angle representation.");
    ros::spin();

    return 0;
}