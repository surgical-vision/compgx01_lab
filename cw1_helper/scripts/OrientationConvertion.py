import rospy
from compgx01_msgs.srv import *
from compgx01_msgs.msg import *
from geometry_msgs.msg import Quaternion


def quat2euler_func(req):
    # This is the quaternion msg
    quat = req.input

    # Fill in your euler angles here
    euler = EulerAngle()
    euler.order = 'zyx'
    euler.angle = [0, 0, 0]

    return quat2EulerResponse(euler)


def quat2angaxis_func(req):
    # This is the quaternion msg
    quat = req.input

    # Fill in your euler angles here
    angAxis = AngleAxis()
    angAxis.angle = 0.0
    angAxis.axis.x = 0.0
    angAxis.axis.y = 0.0
    angAxis.axis.z = 1.0

    return quat2AngAxisResponse(angAxis)


def rotmat2quat_func(req):
    rotmat = req.input

    quat = Quaternion()
    quat.x = 0.0
    quat.y = 0.0
    quat.z = 0.0
    quat.w = 1.0
    return rotmat2QuatResponse(quat)

if __name__ == "__main__":
    rospy.init_node('orientation_conversion')
    ServiceEuler = rospy.Service('quat2Euler_converter', quat2Euler, quat2euler_func)
    ServiceAngAxis = rospy.Service('quat2AngAxis_converter', quat2AngAxis, quat2angaxis_func)
    ServiceAngAxis = rospy.Service('rotmat2Quat_converter', rotmat2Quat, rotmat2quat_func)
    rospy.spin()
