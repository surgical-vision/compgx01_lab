#include <inverse_kinematics/YoubotKinematics.h>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "youbot");

    YoubotKinematics y;
    return y.init();
}