#include "ros/ros.h"
#include "compgx01_msgs/cls_youbot_ikine.h"
#include "compgx01_msgs/ite_youbot_ikine.h"
#include "compgx01_msgs/youbot_jacob.h"

bool compute_closed_ikine(compgx01_msgs::cls_youbot_ikine::Request  &req,
                        compgx01_msgs::cls_youbot_ikine::Response &res)
{
    //TO DO:
    //Put your closed form inverse kinematics here.....
    return true;
}

bool compute_ite_ikine(compgx01_msgs::ite_youbot_ikine::Request  &req,
                          compgx01_msgs::ite_youbot_ikine::Response &res)
{
    //TO DO:
    //Put your iterative inverse kinematics here.....
    return true;
}


bool compute_jacobian(compgx01_msgs::youbot_jacob::Request  &req,
                         compgx01_msgs::youbot_jacob::Response &res)
{
    //TO DO:
    //Put your jacobian calculation here.....
    return true;
}

// TO DO:
//Put your singularity detection here....
//int detect_singularity()
//{
//
//}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ikine_jacob_server");
    ros::NodeHandle n;

    ros::ServiceServer service_quat2Euler = n.advertiseService("ikine_closed_form", compute_closed_ikine);
    ros::ServiceServer service_quat2AngleAxis = n.advertiseService("ikine_ite", compute_ite_ikine);
    ros::ServiceServer service_rotMat2quat = n.advertiseService("jacobian_youbot", compute_jacobian);

    ROS_INFO("Ready to compute inverse kinematics and jacobian matrix.");
    ros::spin();

    return 0;
}