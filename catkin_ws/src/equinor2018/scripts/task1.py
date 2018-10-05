#!/usr/bin/env python

import rospy
import drone
from target_updater import *
from dijkstra import *
from geometry_msgs.msg import PoseStamped, Point, Pose
from ascend_msgs.srv import GlobalMap

current_pose = PoseStamped()  # geometry_msgs::PoseStamped
goal = Point()  # geometry_msgs::Point

world_graph = None
target_updater = TargetUpdater([])



def dronePoseCallback(msg):
    global current_pose
    current_pose = msg


def goalCallback(msg):
    global goal
    global target_updater
    global world_graph
    print("Goal callback called with", goal)
    if msg.position != goal and world_graph is not None:
        goal = msg.position
        pos0 = (int(current_pose.pose.position.x), int(current_pose.pose.position.y))
        pos1 = (int(goal.x), int(goal.y))
        print("Computing shortest path...")
        target_updater = TargetUpdater(shift_reference_point(shortest_path(world_graph, pos0, pos1)[0]))
        drone.set_target(pos0[0], pos0[1], 0)
        print("Computed shortest path! =", target_updater.path)


def parseMap(msg):
    num_rows = msg.dim[0].size
    num_col = msg.dim[1].size

    world_map = [[0]]
    world_map = [[msg.data[row * num_col + col] for col in range(num_col)] for row in range(num_rows)]
    return world_map


def main():
    global world_graph
    global target_updater
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
    print(world_map)
    # Set the global map_graph
    world_graph = ManhattanGraph(world_map)

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

        pos = current_pose.pose.position

        if not target_updater.should_move_to_next_target(pos.x, pos.y):
            continue


        (x, y) = target_updater.next_target()
        print('Updating target to (' + x + ', ' + y + ')')
        drone.set_target(x, y, 0)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
