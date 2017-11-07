#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <tf2_kdl/tf2_kdl.h>
#include <kdl/chain.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainjnttojacsolver.hpp>

class Youbot
{
private:

    ros::NodeHandle n;
    ros::Subscriber joint_sub;
    tf::TransformBroadcaster br;
    KDL::Frame current_pose;
    KDL::Chain kine_chain;
    KDL::JntArray current_joint_position;

public:

    Youbot(): n() {}

    int main(int argc, char **argv) {


        setup_kdl_chain();
        current_joint_position = KDL::JntArray(kine_chain.getNrOfJoints());

        KDL::ChainFkSolverPos_recursive fk_solver = KDL::ChainFkSolverPos_recursive(kine_chain);
        KDL::ChainIkSolverPos_LMA ik_solver = KDL::ChainIkSolverPos_LMA(kine_chain);
        KDL::ChainJntToJacSolver jac_solver = KDL::ChainJntToJacSolver(kine_chain);

        joint_sub = n.subscribe<sensor_msgs::JointState>("/joint_states", 1, boost::bind(&Youbot::joint_state_callback,
                                                                                         this, _1, fk_solver));

        ros::Rate r(200);
        tf::Transform trans;
        geometry_msgs::TransformStamped tr_stamped;
        tf::Quaternion q;

        while (n.ok())
        {

            tr_stamped = tf2::kdlToTransform(current_pose);

            trans.setOrigin(tf::Vector3(tr_stamped.transform.translation.x,
                                        tr_stamped.transform.translation.y,
                                        tr_stamped.transform.translation.z));

            q.setW(tr_stamped.transform.rotation.w);
            q.setX(tr_stamped.transform.rotation.x);
            q.setY(tr_stamped.transform.rotation.y);
            q.setZ(tr_stamped.transform.rotation.z);

            trans.setRotation(q);

            br.sendTransform(tf::StampedTransform(trans, ros::Time::now(), "base_link", "arm_end_effector"));
            r.sleep();

            ros::spinOnce();
        }

        return 0;
    }

    void setup_kdl_chain()
    {
        kine_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(-0.024,   M_PI,  0.096, 160*M_PI/180)));
        kine_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH( 0.033, M_PI_2, -0.019, M_PI + (169 * M_PI / 180.0))));
        kine_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(-0.155,      0,      0, M_PI_2 + (-65.0 * M_PI / 180.0))));
        kine_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(-0.135,      0,      0, (146 * M_PI / 180.0))));
        kine_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH( 0.002, M_PI_2,      0, M_PI_2 + (-102.5 * M_PI / 180.0))));
        kine_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(     0,   M_PI,  -0.21, -150*M_PI/180+(167.5 * M_PI / 180.0))));
    }

    KDL::Jacobian get_jacobian(KDL::ChainJntToJacSolver jac_solver)
    {

        KDL::Jacobian jac = KDL::Jacobian(kine_chain.getNrOfJoints());
        jac_solver.JntToJac(current_joint_position, jac);
        return jac;
    }

    void joint_state_callback(const sensor_msgs::JointState::ConstPtr &q, KDL::ChainFkSolverPos_recursive fk_solver)
    {
        current_joint_position.data(0) = 0.0;
        for (int i = 0; i < 5; i++)
            current_joint_position.data(i + 1) = q->position.at(i);

        fk_solver.JntToCart(current_joint_position, current_pose, 6);
    }
};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "youbot");

    Youbot y;
    return y.main(argc, argv);
}