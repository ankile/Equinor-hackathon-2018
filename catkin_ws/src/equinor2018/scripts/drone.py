#!/usr/bin/env python

import rospy
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import tf

msg = PositionTarget()
last_pos = PoseStamped()
pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=1)

DEFAULT = 2552
TAKEOFF = 0x1000

def pos_cb(msg):
    global last_pos
    last_pos = msg

rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pos_cb)

def init():
    rospy.Timer(rospy.Duration(1.0 / 30.0), publish_setpoint)


def set_target(x, y, yaw):
    global msg
    msg.position.x = x
    msg.position.y = y
    msg.yaw = yaw
    msg.type_mask = DEFAULT


def takeoff():
    global msg
    msg.position.x = last_pos.pose.position.x
    msg.position.y = last_pos.pose.position.y
    msg.position.z = 2.0
    msg.type_mask = DEFAULT | TAKEOFF

def publish_setpoint(event):
    pub.publish(msg)

last_state = State()
def state_cb(msg):
    global last_state
    last_state = msg

def block_until_armed_and_offboard():
    arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
    offboard = rospy.ServiceProxy('mavros/set_mode', SetMode)
    rospy.Subscriber('mavros/state', State, state_cb)

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        arm_resp = arming(True)
        offboard_resp = offboard(0, "OFFBOARD")
        if last_state.mode == "OFFBOARD" and last_state.armed:
            break
        rate.sleep()
