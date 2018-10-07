#!/usr/bin/env python
import rospy


from dora_main import predict

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

    should_guess = False

    def shouldguessCallback(msg):
        print("callback")
        global should_guess
        should_guess = True

    guess = rospy.Publisher("/guess", Int8, queue_size=1)

    rospy.init_node("cv_node", anonymous=True)
    rospy.Subscriber("mavros/local_position/pose", PoseStamped, positionCallback)
    rospy.Subscriber("/should_guess", Int8, shouldguessCallback)

    three_channel_image = ThreeChannelImage()

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        """
        Useful variables in scope:
            position.x
            position.y
            image
        """
        print('should guess:', should_guess)
        if should_guess:
            print("should guess now")
            prediction = predict(three_channel_image.data)
            guess.publish(prediction)
            global should_guess
            should_guess = False

        rate.sleep()


if __name__ == '__main__':
    try:
        print("helloe from main")
        computer_vision()
    except rospy.ROSInterruptException:
        pass
