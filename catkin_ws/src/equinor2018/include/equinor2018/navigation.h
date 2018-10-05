#pragma once
#include <ascend_msgs/Map.h>
#include <geometry_msgs/PoseStamped.h>

// Internal functionality. Implementation in _navigation.cpp
void setupDrone();

void dronePositionCallback(geometry_msgs::PoseStamped::ConstPtr msg);
void mapCallback(ascend_msgs::Map::ConstPtr msg);
