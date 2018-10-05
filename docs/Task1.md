# Task 1: Path planning

The goal of this task is to autonomously maneuver a drone through a maze. A map of the maze is provided in the beginning of the run as a 2D array (see below). The drone can be maneuvered around with fixed altitude.

A template ROS node has been provided in `catkin_ws/src/equinor_2018/src/navigation.cpp`.

## Scoring

Teams are scored based on the time taken to reach the goal. During the competition the leaderboard will be updated based on teams results on the given maps. For the final the node will be tested on new maps.

## Useful variables

`global_map` is a double indexed array with matrix indexing. In other words to access coordinate (x,y) call global_map[y][x].

`current_pose` is a ROS message type that contains information about the drones position and orientation. [The data structure can be found here](http://docs.ros.org/lunar/api/geometry_msgs/html/msg/PoseStamped.html). Note that the orientation is stored as a Quaternion, so some math is required to find the yaw.

`goal` is a ROS message type that defines a point in space defined by 3 coordinates: x,y and z. In this case only x and y will be defined. This defines the position the drone should navigate to.

## Provided map
The map provided is a 2-dimensional array of zeroes and ones, where 1 means that there is a wall, and 0 means no wall. 
Note that if the map says `global_map[1][0] = 1`, meaning y=1 and x=0 is a wall, then there is a wall on the grid from (x,y)=(0,1) to
(x,y)=(1,2), since the wall is atleast 1x1 meter in size!

If the map says that `global_map[5][9] = 0`, then there is no wall at y=5, x=9. This means that a safe position is x=9.5 and y=5.5!

## How to control the drone using the API

To simplify communication with the drone, a small drone API is provided. It gives high-level access to all functions needed to position and orient the drone. 

### Simple Drone API

#### Available functions
```C++ 
/*  Takeoff at current position to an altitude of 2 meters */
void Drone::takeoff();

/*  Set a position target in xy-plane with height 2 meters. Set yaw orientation to and angle between -pi and pi.
    The drone will blindly go to this target, and hold it's position when it gets there. */
void Drone::setTarget(float x, float y, float yaw);

/*  Land at the current position */
void Drone::land();
```
#### Potential pitfalls
The API uses ROS functionallity, so it is dependent on calls to ros::spinOnce(). You *do not* need to call ros::spinOnce yourself after using these functions as the privided framework will take care of it, but it means you need to make sure that the algorithms and solutions you create don't get stuck in loops. 

### Note about movement precision
TLDR; Expect some position offset when arriving at a target! 

The drone lives in a physics simulator which simulates environment noise. It is controlled by an emulated flight controller which runs state of the art flight controller software. Because of these factors, the drone does not always fly perfectly and generelly doesn't perfectly hit the target setpoint. 


