#include "ros/ros.h"
#include "compgx01_msgs/quat2Euler.h"
#include "compgx01_msgs/quat2AngAxis.h"
#include "compgx01_msgs/rotmat2Quat.h"

bool convert_quat2Euler(compgx01_msgs::quat2Euler::Request  &req,
         compgx01_msgs::quat2Euler::Response &res)
{
    //TO DO:
    //Put your rotation representations converter here.....
    return true;
}

bool convert_quat2AngAxis(compgx01_msgs::quat2AngAxis::Request  &req,
                        compgx01_msgs::quat2AngAxis::Response &res)
{
    //TO DO:
    //Put your rotation representations converter here.....
    return true;
}


bool convert_rotmat2Quat(compgx01_msgs::rotmat2Quat::Request  &req,
                        compgx01_msgs::rotmat2Quat::Response &res)
{
    //TO DO:
    //Put your rotation representations converter here.....
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "quat2Euler_server");
    ros::NodeHandle n;

    ros::ServiceServer service_quat2Euler = n.advertiseService("quat2Euler_converter", convert_quat2Euler);
    ros::ServiceServer service_quat2AngleAxis = n.advertiseService("quat2AngAxis_converter", convert_quat2AngAxis);
    ros::ServiceServer service_rotMat2quat = n.advertiseService("rotmat2Quat_converter", convert_rotmat2Quat);

    ROS_INFO("Ready to convert from a quaternion to Euler angle representation.");
    ros::spin();

    return 0;
}
