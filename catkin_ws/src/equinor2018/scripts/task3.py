#!/usr/bin/env python

import rospy
import drone
from dijkstra import *
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped, PoseArray
from ascend_msgs.srv import GlobalMap

current_pose = PoseStamped()  # geometry_msgs::PoseStamped
goals = []  # geometry_msgs::PoseArray()
path = []
goals_initialized = False
waiting_for_guess = False
graph = None
last_point = (1,1)

def distance(p0, p1):
    (x0, y0) = (p0[0], p0[1])
    (x1, y1) = (p1[0], p1[1])
    return ((x0 - x1)**2 + (y0 - y1)**2) ** 0.5



def dronePoseCallback(msg):
    global current_pose
    current_pose = msg


def goalCallback(msg):
    global goals
    global goals_initialized
    global graph
    global path

    print("goalCallback...")
    if not goals_initialized and graph is not None:
        print("Set goals to: " + str(goals))
        goals = msg.poses

        print ("Goal updated")
        src = (int(current_pose.pose.position.x), int(current_pose.pose.position.y))
        dst = (int(goals[0].position.x), int(goals[0].position.y))

        path = shortest_path(graph, src, dst)
        # path = insert_break_points(path)
        path[-1] = (goals[0].position.x, goals[0].position.y)
        print("Path: " + str(path))
        (x, y) = path[0]
        drone.set_target(x, y, 0)
        #goal_updated = False
        print("Target set to " + str(x) + ", " + str(y))
        goals_initialized = True

def guessed_callback(msg):
    global goals
    global waiting_for_guess
    global current_pose
    global last_point

    print("Guessed: " + str(msg))
    # Remove the current goal from the list of goals
    goals = goals[1:]

    if not goals:
        print("No more goals")
        return

    goal = goals[0]
    print ("Goal updated")
    src = (int(current_pose.pose.position.x), int(current_pose.pose.position.y))
    dst = (int(goal.position.x), int(goal.position.y))

    path = shortest_path(graph, src, dst)

    last_point = src

    path[-1] = (goal.position.x, goal.position.y)
    print("Path: " + str(path))
    (x, y) = path[0]
    drone.set_target(x, y, 0)
    #goal_updated = False
    print("Target set to " + str(x) + ", " + str(y))

    waiting_for_guess = False




def distance(p0, p1):
    (x0, y0) = (p0[0], p0[1])
    (x1, y1) = (p1[0], p1[1])
    return ((x0 - x1)**2 + (y0 - y1)**2) ** 0.5


def parseMap(msg):
    num_rows = msg.dim[0].size
    num_col = msg.dim[1].size

    world_map = [[0]]
    world_map = [[msg.data[row * num_col + col] for col in range(num_col)] for row in range(num_rows)]
    return world_map

def is_at_goal(pos, thresh = 0.1):
    global goals

    if not goals:
        return False

    return ((goals[0].position.x - pos.x)**2 + (goals[0].position.y - pos.y)**2) ** 0.5 < thresh


def main():
    global goals
    global goals_initialized
    global graph
    global waiting_for_guess
    global current_pose
    global path
    # Init ROS node
    rospy.init_node('task1', anonymous=True)

    # Create subscriber for position and goal
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, dronePoseCallback)
    rospy.Subscriber('/goals', PoseArray, goalCallback)
    rospy.Subscriber('/guess', Int8, guessed_callback)
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
    # Construct a graph from the world map
    graph = ManhattanGraph(world_map)
    # The current path
    #path = []#endurance_track(graph)

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
        #print("A")
        if waiting_for_guess:
            print("Waiting for guess")
            continue

        pos = current_pose.pose.position

        cur_pos = (pos.x, pos.y)
        speed = distance(prev_pos, cur_pos) * 30
        speed_in_x = abs(prev_pos[0] - cur_pos[0]) * 30
        speed_in_y = abs(prev_pos[1] - cur_pos[1]) * 30

        velocity = (cur_pos[0] - prev_pos[0], cur_pos[1] - prev_pos[1])
        prev_pos = cur_pos

        #print("B")
        if is_at_goal(pos):
            print("Is at goal!")
            waiting_for_guess = True
            should_guess.publish(1)

        #print("C")
        if not path or not goals:
            continue


        waypoint = path[0]
        print("pos = " + str((pos.x, pos.y)))
        print("distance = " + str(distance((pos.x, pos.y), waypoint)))



        if (not drag_back_point) and speed > 0.32 and (speed**2)*0.063 > distance((pos.x, pos.y), waypoint) and distance((pos.x, pos.y), waypoint) > 1.5:
            print("A")
            path.insert(0,last_point)
            (x, y) = path[0]
            drone.set_target(x, y, 0)
            print("Target set to ", x, y)
            drag_back_point = True


        if drag_back_point and speed < 0.31:
            print("B")
            path = path[1:]
            (x, y) = path[0]
            drone.set_target(x, y, 0)
            print("Target set to ", x, y)
            drag_back_point = False


        if (dist < 0.06 and speed_max < 0.06):
            print("C")
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