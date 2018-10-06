#!/usr/bin/env python
import rospy

import numpy as np
import matplotlib.pyplot as plt
import array
import copy
from Task2.dora_main import predict

from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

current_pose = None


class ThreeChannelImage:

    def __init__(self):
        self.listener()
        self.data = None
        self.dim = (0, 0)
        self.channels = 3
        self.bridge = CvBridge()
        self.image = None

    def callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        # cv2.imshow('image', self.image)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

    def listener(self):
        rospy.Subscriber("/drone_front_camera/image_raw", Image, self.callback)


def positionCallback(msg):
    current_pose = msg



def computer_vision():
    def shouldguessCallback(msg):
        should_guess = True

    guess = rospy.Publisher("/guess", Int8, queue_size=1)

    rospy.init_node("cv_node", anonymous=True)
    rospy.Subscriber("mavros/local_position/pose", PoseStamped, positionCallback)
    rospy.Subscriber("/shouldguess", Int8, shouldguessCallback)

    three_channel_image = ThreeChannelImage()

    rate = rospy.Rate(1)

    should_guess = False
    while not rospy.is_shutdown():
        """
        Useful variables in scope:
            position.x
            position.y
            image
        """
        if should_guess:
            prediction = predict(three_channel_image.data)
            guess.publish(prediction)
            should_guess = False

        rate.sleep()


if __name__ == '__main__':
    try:
        computer_vision()
    except rospy.ROSInterruptException:
        pass
