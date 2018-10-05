#!/usr/bin/env python

import rospy
import drone
from target_updater import *
from dijkstra import *
from geometry_msgs.msg import PoseStamped, Point, Pose
from ascend_msgs.srv import GlobalMap

current_pose = PoseStamped()  # geometry_msgs::PoseStamped
goal = Point()  # geometry_msgs::Point
goal_updated = False


def distance(p0, p1):
    (x0, y0) = (p0.x, p0.y)
    (x1, y1) = (p1.x, p1.y)
    return ((x0 - x1)**2 + (y0 - y1)**2) ** 0.5



def dronePoseCallback(msg):
    global current_pose
    current_pose = msg


def goalCallback(msg):
    global goal
    global goal_updated

    goal = msg.position
    goal_updated = true


def parseMap(msg):
    num_rows = msg.dim[0].size
    num_col = msg.dim[1].size

    world_map = [[0]]
    world_map = [[msg.data[row * num_col + col] for col in range(num_col)] for row in range(num_rows)]
    return world_map


def main():
    global goal
    global goal_updated
    # Init ROS node
    rospy.init_node('task1', anonymous=True)

    # Create subscriber for position and goal
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, dronePoseCallback)
    rospy.Subscriber('/goal', Pose, goalCallback)

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
    # Construct a graph from the world map
    graph = ManhattanGraph(world_map)
    # The current path
    path = []

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
        # Do stuff

        pos = (current_pose.pose.position.x, current_pose.pose.y)

        if goal_updated:
            print ("Goal updated")
            src = (int(pos[0]), int(pos[1]))
            dst = (int(goal.x), int(goal.y))

            path = shortest_path(graph, src, dst)
            path[0] = pos
            path[-1] = (goal.x, goal.y)

            (x, y) = path[0]
            path = path[1:]
            drone.set_target(x, y, 0)
            goal_updated = False
            print("Target set to " + str(x) + ", " + str(y))

        if not path or not goal:
            continue

        g = (goal.x, goal.y)

        if distance(pos, g) < 0.4:
            (x, y) = path[0]
            path = path[1:]
            dron.set_target(x, y, 0)
            print("Target set to ", x, y)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
