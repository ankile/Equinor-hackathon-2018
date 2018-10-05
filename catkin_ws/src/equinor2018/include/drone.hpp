#ifndef DRONE_HPP
#define DRONE_HPP
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
class Drone {
private:
    mavros_msgs::PositionTarget targetPos_;
    geometry_msgs::PoseStamped currPos_;
    ros::Publisher posPub_;
    ros::Subscriber posSub_;
    ros::NodeHandle nh_;
    ros::Timer timer_;

    void posCB(const geometry_msgs::PoseStamped& msg);
    void timerCB(const ros::TimerEvent& e);
public:
    Drone();
    void takeoff();
    void setTarget(float x, float y, float yaw);
    void land();
    void blockUntilArmedAndOffboard();
};

#endif
