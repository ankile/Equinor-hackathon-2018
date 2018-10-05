# Simple Python Example

This is a short example on how to fly the drone around in the arena using the provided python template. 

# Step 1: Get the python template for task 1:

The python templates are located in `equinor_hackathon_2018/catkin_ws/src/equinor2018/scripts`, and the template for task 1 should look like this:

```python
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
    num_rows = len(msg.dim[0])
    num_col = len(msg.dim[1])
    
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
    while not rospy.is_shutdown():
        rate.sleep()
        #Do stuff


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
```

## Step 2: Set position target after takeoff

```python
while not rospy.is_shutdown():
    rate.sleep()
    #Do stuff
```
This will loop will run at a specified rate (avoid wasting CPU cycles), and the loop will run "forever". Modify the main loop and set the position target once


```python
target_set = False
while not rospy.is_shutdown():
    rate.sleep()
    #Avoid long variable names..
    pos = current_pose.pose.position
    #Do stuff
    if not target_set and pos.z > 0.5:
        drone.set_target(pos.x + 2.0, pos.y, 0.0)
        target_set = True

```
This code will set the target once after takeoff is completed

## Step 3: Run the code on the server

### Step 3.1: Start simulator 
Follow [this guide](simulator.md) to start the simulator for task 1 in a dedicated terminal, and leave the terminal open.

### Step 3.2: Run your code
Open a new terminal and login to the team user using `source start.sh` from the home directory. The scripts can be launched using rosrun.

For task 1: 
```bash
rosrun equinor2018 task1.py
```

Alternativly, they can be launched directly from the `equinor_hackathon_2018_team/catkin_ws/src/equinor2018/scripts` folder. 

For task 1:
```bash
./task1.py
```

If ubuntu or rosrun can't find the python executable, run (for task 1)
```bash
chmod +x task1.py
```

The drone should perform a takeoff and move 2 meters in the X direction after reaching a certian altitude.
