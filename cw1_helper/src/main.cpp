#include "ros/ros.h"
#include "compgx01_msgs/quat2Euler.h"
// TO DO: Build
//#include "compgx01_msgs/quat2AngleAxis.h"
//#include "compgx01_msgs/rotMat2quat.h"

bool convert_quat2Euler(compgx01_msgs::quat2Euler::Request  &req,
         compgx01_msgs::quat2Euler::Response &res)
{
    //TO DO:
    //Put your rotation representations converter here.....
    return true;
}

//// Build your service first and uncomment these lines
//bool convert_quat2AngleAxis(compgx01_msgs::quat2AngleAxis::Request  &req,
//                        compgx01_msgs::quat2AngleAxis::Response &res)
//{
//    //TO DO:
//    //Put your rotation representations converter here.....
//    return true;
//}

//// Build your service first and uncomment these lines
//bool convert_rotMat2quat(compgx01_msgs::rotMat2quat::Request  &req,
//                        compgx01_msgs::rotMat2quat::Response &res)
//{
//    //TO DO:
//    //Put your rotation representations converter here.....
//    return true;
//}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "quat2Euler_server");
    ros::NodeHandle n;

    ros::ServiceServer service_quat2Euler = n.advertiseService("quat2Euler_converter", convert_quat2Euler);
    ros::ServiceServer service_quat2AngleAxis = n.advertiseService("quat2AngleAxis_converter", convert_quat2AngleAxis);
    ros::ServiceServer service_rotMat2quat = n.advertiseService("rotMat2quat_converter", convert_rotMat2quat);

    ROS_INFO("Ready to convert from a quaternion to Euler angle representation.");
    ros::spin();

    return 0;
}