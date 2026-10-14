#ifndef AREA_SCANNING_DRONE_CONTROLLER_HPP
#define AREA_SCANNING_DRONE_CONTROLLER_HPP

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/BatteryState.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandTOL.h>
#include <string>

class DroneController {
public:
    DroneController(ros::NodeHandle& nh, const std::string& drone_ns = "");

    bool armDrone();
    bool takeoff(float altitude);
    bool setMode(std::string mode);
    bool checkSystemHealth();

private:
    void stateCallback(const mavros_msgs::State::ConstPtr& msg);
    void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg);

    ros::NodeHandle nh_;
    std::string ns_;

    ros::Subscriber state_sub_, battery_sub_;
    ros::ServiceClient arming_client_, set_mode_client_, takeoff_client_;

    mavros_msgs::State current_state_;
    sensor_msgs::BatteryState current_battery_;
};

#endif
