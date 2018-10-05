#include "drone.hpp"
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <cmath>

namespace {
#define IGNORE_PX (1 << 0)  // Position ignore flags
#define IGNORE_PY (1 << 1)
#define IGNORE_PZ (1 << 2)
#define IGNORE_VX (1 << 3)  // Velocity vector ignore flags
#define IGNORE_VY (1 << 4)
#define IGNORE_VZ (1 << 5)
#define IGNORE_AFX (1 << 6) // Acceleration/Force vector ignore flags
#define IGNORE_AFY (1 << 7)
#define IGNORE_AFZ (1 << 8)
#define FORCE (1 << 9)  // Force in af vector flag
#define IGNORE_YAW (1 << 10)
#define IGNORE_YAW_RATE (1 << 11)
#define SETPOINT_TYPE_TAKEOFF 0x1000
#define SETPOINT_TYPE_LAND 0x2000
#define SETPOINT_TYPE_LOITER 0x3000
#define SETPOINT_TYPE_IDLE 0x4000

constexpr uint16_t default_mask = IGNORE_VX | IGNORE_VY | IGNORE_VZ | IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ | IGNORE_YAW_RATE;
};


mavros_msgs::State last_state;

Drone::Drone() {
    assert(ros::isInitialized());
    posPub_ = nh_.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
    posSub_ = nh_.subscribe("mavros/local_position/pose", 10, &Drone::posCB, this);  
    timer_ = nh_.createTimer(ros::Duration(0.03), &Drone::timerCB, this);
    targetPos_.type_mask = default_mask;
}

void Drone::posCB(const geometry_msgs::PoseStamped& msg) {
    currPos_ = msg; 
}

void Drone::timerCB(const ros::TimerEvent& e) {
    targetPos_.header.stamp = ros::Time::now();
    posPub_.publish(targetPos_);    
}

void Drone::takeoff() {
    targetPos_.type_mask = default_mask | SETPOINT_TYPE_TAKEOFF;
    targetPos_.position.x = currPos_.pose.position.x;
    targetPos_.position.y = currPos_.pose.position.y;
    targetPos_.position.z = 2.0; //Meters
}

void Drone::setTarget(float x, float y, float yaw) {
    targetPos_.type_mask = default_mask;
    targetPos_.position.x = x;
    targetPos_.position.y = y;
    targetPos_.yaw = yaw;
}

void Drone::land() {
    targetPos_.type_mask = default_mask | SETPOINT_TYPE_LAND;
    targetPos_.position.x = currPos_.pose.position.x;
    targetPos_.position.y = currPos_.pose.position.y;
    targetPos_.position.z = -1.0; //Not used
}

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    last_state = *msg;
}

void Drone::blockUntilArmedAndOffboard() {
    ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    ros::Time last_request = ros::Time::now();

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Rate rate(1);
    while(ros::ok()) {
        ros::spinOnce();
        arming_client.call(arm_cmd);
        set_mode_client.call(offb_set_mode);
        if(last_state.mode == "OFFBOARD" && (bool) last_state.armed) {
            break;
        }
        rate.sleep();
    }
}


