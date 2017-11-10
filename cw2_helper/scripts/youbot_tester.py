#!/usr/bin/env python

import rospy
from cw2_helper.YoubotTemplate import YoubotTemplate

if __name__ == "__main__":
    rospy.init_node('youbot_test')
    youbot = YoubotTemplate()
    print youbot.dh_params
