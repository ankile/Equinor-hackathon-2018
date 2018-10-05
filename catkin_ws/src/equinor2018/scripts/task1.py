#!/usr/bin/env python

import rospy
import drone
from geometry_msgs.msg import PoseStamped, Point, Pose
from ascend_msgs.srv import GlobalMap

current_pose = PoseStamped() #geometry_msgs::PoseStamped
goal = Point() #geometry_msgs::Point

def dronePoseCallback(msg):
    global current_pose
    current_pose = msg
    
def goalCallback(msg):
    global goal
    goal = msg.position
    
def parseMap(msg):
    num_rows = msg.dim[0].size
    num_col = msg.dim[1].size
    
    world_map = [[0]]
    world_map = [[ msg.data[row * num_col + col] for col in range(num_col)] for row in range(num_rows)]
    return world_map
    
    

def main():
    #Init ROS node
    rospy.init_node('task1', anonymous=True)
    
    #Create subscriber for position and goal
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, dronePoseCallback)
    rospy.Subscriber('/goal', Pose, goalCallback)
    
    #Create map service client
    getMap = rospy.ServiceProxy('/GlobalMap', GlobalMap)
    rospy.wait_for_service('/GlobalMap')
    
    try:
        raw_map = getMap()
    except rospy.ServiceException as e:
        print("Map service error: " + str(e))
        return
    
    #Get map as 2D list
    world_map = parseMap(raw_map)
    
    #Initialize drone
    drone.init()
    
    #Arm and set offboard
    drone.block_until_armed_and_offboard()
    
    #Takeoff
    drone.takeoff()
    
    #Create rate limiter
    rate = rospy.Rate(30)
    target_set = False
    while not rospy.is_shutdown():
        rate.sleep()
        #Do stuff
        pos = current_pose.pose.position
        
        if not target_set and pos.z > 0.5:
            drone.set_target(pos.x + 2.0, pos.y, 0.0)
            target_set = True


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
