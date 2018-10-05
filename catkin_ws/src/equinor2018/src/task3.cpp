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

