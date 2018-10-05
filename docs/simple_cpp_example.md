# Simple C++ example using template

This is a simple example using the C++ template provided. Our goal is to fly the drone two meters in the X direction after takeoff and hold the target position. 

## Step 1: Open the template for task 1
The template is located in `equinor_hackathon_2018_team/catkin_ws/src/equinor_2018/src/task1.cpp`, and should look like this:
```cpp
#include "ros/ros.h"
#include "equinor2018/navigation.h"
#include "drone.hpp"
#include <ascend_msgs/GlobalMap.h>
#include <geometry_msgs/Point.h>
#include <vector>


geometry_msgs::PoseStamped current_pose;
geometry_msgs::Point goal;

void dronePoseCallback(const geometry_msgs::PoseStamped& msg) {
    //Msg contains the last received drone position and orientation
    //Position = msg.pose.position.x/y/z
    //Orientation = msg.pose.orientation.x/y/z/w
    current_pose = msg;
}

//The map is parsed into a 2D vector
std::vector<std::vector<char> > parseMap(const ascend_msgs::GlobalMap::Response& msg){

	int num_rows = msg.dim[0].size;
	int num_cols = msg.dim[1].size;

	std::vector<std::vector<char> > world_map;
    world_map.reserve(num_rows);
	for(int row=0; row<num_rows; row++){
        std::vector<char> temp;
        temp.reserve(num_cols);
		for(int col=0; col<num_cols; col++){
			temp.push_back(msg.data[num_cols*row + col]);
		}
        world_map.push_back(temp);
	}

    return world_map;
}


void goalCallback(const geometry_msgs::Pose& msg) {
    goal = msg.position;
}


int main(int argc, char **argv){

    ros::init(argc, argv, "navigation");
    ros::NodeHandle node;
    ros::Rate rate(30);

    // Sets up drone position listener
    ros::Subscriber drone_sub = node.subscribe("/mavros/local_position/pose", 1, dronePoseCallback);
    ros::Subscriber goal_sub = node.subscribe("/goal", 1, goalCallback);

    //Get map from service
    ascend_msgs::GlobalMap service;
    while(ros::ok() && !ros::service::call("/GlobalMap", service)){
        ros::spinOnce();
        rate.sleep();
    }

    //Parse map received from map service
    std::vector<std::vector<char> > map = parseMap(service.response);
    
    //Initialize drone controller
    Drone drone;
    //Wait for armed and offboard
    drone.blockUntilArmedAndOffboard();    
    //Takeoff
    drone.takeoff();
    while (ros::ok()) {
        //Sleep the remaning time to achieve desired rate
        rate.sleep();
        //Fetches new ROS messages and triggers callbacks
        ros::spinOnce();
        

        //Implement your code here

    }
    return 0;
}

```

## Step 2: Modify the main loop
```cpp
//Takeoff
drone.takeoff();
while (ros::ok()) {
    //Sleep the remaning time to achieve desired rate
    rate.sleep();
    //Fetches new ROS messages and triggers callbacks
    ros::spinOnce();

    //Implement your code here
}
```

When initialization is completed, the drone will perform a takeoff (`drone.takeoff()`) at the current position. After takeoff it will enter an infinite loop where we can control the drone. The `ros::spinOnce()` has to run frequently for ROS to function properly, so avoid blocking code. The loop will make sure the `current_pose` and `goal` objects always contains the most recent information.

### Step 2.1: Set position and yaw target
```cpp
//Only set target once
bool target_set = false;
while (ros::ok()) {
    //Sleep the remaning time to achieve desired rate
    rate.sleep();
    //Fetches new ROS messages and triggers callbacks
    ros::spinOnce();

    //Implement your code here

    //Avoid long variable names...
    auto& pos = current_pose.pose.position;
    //Are takeoff completed?
    if (!target_set && pos.z > 0.5) {
        //Set X, Y and yaw setpoint
        drone.setTarget(pos.x + 2.0, pos.y, 0.0);
        target_set = true;
    }
}
```

We only want to set the position target once, and hold that position. Yaw is in this example set to zero for simplisity.  

## Step 3: Compile your code
Go to your `catkin_ws`, located in `equinor_hackathon_2018/catkin_ws` and run `catkin_make` to compile your code. 

## Step 4: Run your code

### Step 4.1: Start simulator
Follow [this guide](simulator.md) to start the simulator for task 1 in a dedicated terminal, and leave the terminal open.

### Step 4.2: Run your code 
Open a new terminal and login to the team account (using `source start.sh` in your home directory). All templates are located in the `equinor2018` ROS package, and can be launched using rosrun after compilation. 

For task 1:
```bash
rosrun equinor2018 task1
```

The code should make the drone perform a takeoff, and move 2 meters in the X direction after reaching a certian altitude. 


