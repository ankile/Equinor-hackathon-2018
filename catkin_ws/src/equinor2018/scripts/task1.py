#!/usr/bin/env python

import rospy

import drone
from dijkstra import *


from geometry_msgs.msg import PoseStamped, Point, Pose
from ascend_msgs.srv import GlobalMap

current_pose = PoseStamped()  # geometry_msgs::PoseStamped
goal = Point()  # geometry_msgs::Point
goal_updated = False

def endurance_track(graph):
    return shortest_path(graph, (6, 19), (3, 3)) + shortest_path(graph, (3, 3), (22, 35)) + shortest_path(graph, (22, 35), (6, 19))


def distance(p0, p1):
    (x0, y0) = (p0[0], p0[1])
    (x1, y1) = (p1[0], p1[1])
    return ((x0 - x1)**2 + (y0 - y1)**2) ** 0.5



def dronePoseCallback(msg):
    global current_pose
    current_pose = msg


def goalCallback(msg):
    global goal
    global goal_updated

    goal_updated = goal_updated or goal != msg.position
    goal = msg.position


def parseMap(msg):
    num_rows = msg.dim[0].size
    num_col = msg.dim[1].size

    world_map = [[0]]
    world_map = [[msg.data[row * num_col + col] for col in range(num_col)] for row in range(num_rows)]
    return world_map


def main():
    global goal
    global goal_updated
    global current_pose
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
    path = []#endurance_track(graph)

    # Initialize drone
    drone.init()

    # Arm and set offboard
    drone.block_until_armed_and_offboard()
    print("Hello!")
    # Takeoff
    drone.takeoff()
    prev_pos = (current_pose.pose.position.x, current_pose.pose.position.y)

    # Create rate limiter
    rate = rospy.Rate(30)

    drag_back_point = False
    last_point = (1,1)

    while not rospy.is_shutdown():
        rate.sleep()
        # Do stuff

        pos = current_pose.pose.position

        cur_pos = (pos.x, pos.y)
        speed = distance(prev_pos, cur_pos) * 30
        speed_in_x = abs(prev_pos[0] - cur_pos[0]) * 30
        speed_in_y = abs(prev_pos[1] - cur_pos[1]) * 30

        velocity = (cur_pos[0] - prev_pos[0], cur_pos[1] - prev_pos[1])
        prev_pos = cur_pos



        if goal_updated:
            print ("Goal updated")
            src = (int(pos.x), int(pos.y))
            dst = (int(goal.x), int(goal.y))

            path = shortest_path(graph, src, dst)
            last_point = src
            # path = insert_break_points(path)
            path[-1] = (goal.x, goal.y)
            print("Path: " + str(path))
            (x, y) = path[0]
            drone.set_target(x, y, 0)
            goal_updated = False
            print("Target set to " + str(x) + ", " + str(y))

        if not path or not goal:
            continue




        waypoint = path[0]
        print("pos = " + str((pos.x, pos.y)))
        print("distance = " + str(distance((pos.x, pos.y), waypoint)))

        dist = distance((pos.x, pos.y), waypoint)
        s1 = 0.18
        if dist > 3:
            x = dist
            if x > 9:
                x = 9
            s1 += (x - 3)*0.03





        if (not drag_back_point) and speed > s1 and (speed**2.0)*0.067 + 1.0 > distance((pos.x, pos.y), waypoint) and distance((pos.x, pos.y), waypoint) > 0.8:
            path.insert(0,last_point)

            (x, y) = path[0]
            drone.set_target(x, y, 0)
            print("Target set to ", x, y)
            drag_back_point = True


        if drag_back_point and speed < s1 - 0.01:
            path = path[1:]
            (x, y) = path[0]
            drone.set_target(x, y, 0)
            print("Target set to ", x, y)
            drag_back_point = False


        speed_max = speed

        distance_to_next = distance(waypoint,path[1])
        speed_var = 0.11
        dist_var = 0.09
        if distance_to_next >= 4:
            speed_var = 0.08
            dist_var = 0.08

        if (dist < dist_var and speed_max < speed_var):
            last_point = path[0]
            path = path[1:]
            (x, y) = path[0]
            drone.set_target(x, y, 0)
            print("Target set to ", x, y)
            drag_back_point = False


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
