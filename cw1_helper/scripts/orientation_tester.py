import rospy
from compgx01_msgs.srv import *
from compgx01_msgs.msg import *
from geometry_msgs.msg import Quaternion

from PyKDL.Rotation import Rotation

if __name__ == "__main__":
    rospy.init_node("orientation_tester")

    quat2euler = rospy.ServiceProxy('quat2Euler_converter', quat2Euler)
    quat2angaxis = rospy.ServiceProxy('quat2AngAxis_converter', quat2AngAxis)
    rotmat2quat = rospy.ServiceProxy('rotmat2Quat_converter', rotmat2Quat)

    #
