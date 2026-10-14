#ifndef DRONE_CONTROLLER_HPP
#define DRONE_CONTROLLER_HPP

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/BatteryState.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandTOL.h>
#include <geometry_msgs/Twist.h>
#include <human_tracking/PersonBBox.h>
#include <string>

class DroneController {
public:
    DroneController(ros::NodeHandle& nh, const std::string& drone_ns = "");

    bool armDrone();
    bool takeoff(float altitude);
    bool setMode(std::string mode);
    bool checkSystemHealth();

    void publishVelocity(float forward_vel, float lateral_vel, float yaw_rate);
    void runFollowLogic(const human_tracking::PersonBBox& msg, float current_range);

private:
    void stateCallback(const mavros_msgs::State::ConstPtr& msg);
    void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg);

    ros::Time last_seen_time_;
    int last_locked_id_ = -1;
    bool target_acquired_ = false;
    enum class ForwardAxis { X_AXIS, Y_AXIS };
    ForwardAxis forward_dir_;

    float prev_lat_err_ = 0, lat_integral_ = 0;

    ros::NodeHandle nh_;
    std::string ns_;
    ros::Subscriber state_sub_, battery_sub_;
    ros::Publisher vel_pub_;
    ros::ServiceClient arming_client_, set_mode_client_, takeoff_client_;

    mavros_msgs::State current_state_;
    sensor_msgs::BatteryState current_battery_;

    float prev_dist_err_ = 0, dist_integral_ = 0;
    float prev_yaw_err_ = 0, yaw_integral_ = 0;
};

#endif
