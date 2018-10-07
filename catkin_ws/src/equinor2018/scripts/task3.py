#!/usr/bin/env python

import rospy
import drone
from dijkstra import *
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped, PoseArray
from ascend_msgs.srv import GlobalMap

"""
current_pose = PoseStamped()  # geometry_msgs::PoseStamped

# Contains .poses
goals = PoseArray()  # geometry_msgs::PoseArray
goals_updated = False
"""


class AutoPilot:
    def __init__(self, graph):
        self.initial_goals = PoseArray()
        self.remaining_goals = PoseArray()
        self.graph = graph
        self.is_waiting_for_guess = False

        self.path = None
        self.velocity = None
        self.prev_pos = None
        self.pos = None

    def drone_pose_callback(self, pose_stamped):
        self.prev_pos = self.pos
        self.pos = pose_stamped

        # We'll only update the velocity if we have enough samples to do so
        if self.prev_pos is not None:
            (x0, y0) = (self.prev_pos.x, self.prev_pos.y)
            (x1, y1) = (self.pos.x, self.pos.y)
            self.velocity = (x1 - x0, y1 - y0)

    def goals_callback(self, pose_array):
        if pose_array != self.initial_goals:
            self.initial_goals = pose_array.positions[:]
            self.remaining_goals = pose_array.positions[:]
            self.path = shortest_path(self.graph, self.pos, self.initial_goals[0])
        pass

    def is_at_subgoal(self):
        # We're at a goal iff path == []
        if self.path is not None and not self.path:
            print("Currently at a subgoal")
            return True

        return False

    def has_guessed_callback(self, response):
        print("Got response: " +  str(response))
        self.is_waiting_for_guess = False
        self.remaining_goals = self.remaining_goals[1:]

        if self.remaining_goals:
            self.path = shortest_path(self.graph, self.pos, self.remaining_goals[0])

    def tick(self):
        pass

"""
def dronePoseCallback(msg):
    global current_pose
    current_pose = msg


def goalCallback(msg):
    global goals
    global goals_updated

    if goals != msg:
        goals = msg
        goals_updated = True
"""

def parseMap(msg):
    num_rows = msg.dim[0].size
    num_col = msg.dim[1].size

    world_map = [[0]]
    world_map = [[msg.data[row * num_col + col] for col in range(num_col)] for row in range(num_rows)]
    return world_map


def main():
    # Init ROS node
    rospy.init_node('task1', anonymous=True)

    # Create a topic which the we'll publish to when we're in position.
    should_guess = rospy.Publisher('/should_guess', Int8, queue_size = 1)

    # Create map service client
    getMap = rospy.ServiceProxy('/GlobalMap', GlobalMap)
    rospy.wait_for_service('/GlobalMap')

    try:
        raw_map = getMap()
    except rospy.ServiceException as e:
        print("Map service error: " + str(e))
        return

    # Get map as 2D list
    world_map = parseMap(raw_map)

    # Construct a graph from the map
    graph = ManhattanGraph(world_map)
    # Initialize the auto-pilot
    auto_pilot = AutoPilot(graph)

    # Create subscriber for guess, pose and goals
    rospy.Subscriber('/guess', Int8, auto_pilot.has_guessed_callback)
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, auto_pilot.drone_pose_callback)
    rospy.Subscriber('/goals', PoseArray, auto_pilot.goals_callback)
    print("Created AutoPilot and subscribed it for /guess, .../pose and /goals")

    # Initialize drone
    drone.init()

    # Arm and set offboard
    drone.block_until_armed_and_offboard()

    # Takeoff
    drone.takeoff()

    # Create rate limiter
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        rate.sleep()

        # If we're waiting from a response from the CV system, we'll simply wait
        if auto_pilot.is_waiting_for_guess:
            continue

        # If we're at a sub_coal, we'll initialize the CV system, and block until we get a response
        if auto_pilot.is_at_subgoal():
            print("Currently at sub-goal, waiting for response...")
            auto_pilot.is_waiting_for_guess = True
            should_guess.publish(1)
            continue

        # Otherwise, we'll set the waypoints as specified by the auto-pilot
        auto_pilot.tick()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass