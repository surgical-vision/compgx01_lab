import rospy
import rosbag
from inverse_kinematics.YoubotKDL import YoubotKDL
import PyKDL
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion
import tf2_kdl
import tf2_ros
import rospkg


if __name__ == '__main__':
    rospy.init_node('youbot_bag_follower')
    youbot = YoubotKDL()
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    rospack = rospkg.RosPack()

    # get the file path for rospy_tutorials
    path = rospack.get_path('inverse_kinematics')
    bag = rosbag.Bag(path + '/bags/test.bag')
    trans_list = []

    for topic, msg, t in bag.read_messages(topics=['desired_pose']):
        trans_list.append(msg)

    # trans = TransformStamped()
    # trans.child_frame_id = 'desired_pos'
    # trans.header.frame_id = 'arm_link_0'
    # trans.transform.translation = Vector3(-0.034, -0.197, 0.041)
    # trans.transform.rotation = Quaternion(0.724, 0.676, -0.120, 0.071)

    for trans in trans_list:
        # Publish the desired tf
        trans.header.stamp = rospy.Time.now()
        broadcaster.sendTransform(trans)

        # Find your current cartesian pose
        youbot.forward_kinematics(youbot.current_joint_position, youbot.current_pose)
        youbot.broadcast_pose(youbot.current_pose)

        # Get next desired pose from rosbag
        frame = tf2_kdl.transform_to_kdl(trans)

        # Ikine
        jointkdl = youbot.inverse_kinematics_closed(frame)
        joint_array = []

        for pos in jointkdl:
            joint_array.append(pos)

        youbot.publish_joint_trajectory(joint_array)

        rospy.sleep(1)
        #raw_input('Press enter for next pose\n')