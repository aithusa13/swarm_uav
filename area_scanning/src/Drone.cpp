#include "area_scanning/Drone.hpp"

DroneController::DroneController(ros::NodeHandle& nh, const std::string& drone_ns) : nh_(nh), ns_(drone_ns) {

    std::string prefix = "";
    if (!ns_.empty()) {
        prefix = (ns_[0] == '/') ? ns_ : "/" + ns_;
    }

    state_sub_   = nh_.subscribe<mavros_msgs::State>(prefix + "/mavros/state", 10, &DroneController::stateCallback, this);
    battery_sub_ = nh_.subscribe<sensor_msgs::BatteryState>(prefix + "/mavros/battery", 10, &DroneController::batteryCallback, this);

    arming_client_   = nh_.serviceClient<mavros_msgs::CommandBool>(prefix + "/mavros/cmd/arming");
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>(prefix + "/mavros/set_mode");
    takeoff_client_  = nh_.serviceClient<mavros_msgs::CommandTOL>(prefix + "/mavros/cmd/takeoff");
}

void DroneController::stateCallback(const mavros_msgs::State::ConstPtr& msg)          { current_state_   = *msg; }
void DroneController::batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg) { current_battery_ = *msg; }

bool DroneController::checkSystemHealth() {
    if (!current_state_.connected) {
        ROS_WARN_THROTTLE(10, "[%s_PRECHECK] Waiting for FCU connection...", ns_.c_str());
        return false;
    }
    if (current_battery_.percentage > 0.0 && current_battery_.percentage < 0.1) {
        ROS_ERROR_THROTTLE(10, "[%s_PRECHECK] Critical battery level!", ns_.c_str());
        return false;
    }
    if (current_state_.system_status < 3) return false;

    return true;
}

bool DroneController::setMode(std::string mode) {
    mavros_msgs::SetMode sm;
    sm.request.custom_mode = mode;
    return set_mode_client_.call(sm) && sm.response.mode_sent;
}

bool DroneController::armDrone() {
    if (!checkSystemHealth()) return false;

    if (current_state_.mode != "GUIDED") {
        setMode("GUIDED");
        ros::Duration(1.0).sleep();
        return false;
    }

    if (current_state_.armed) return true;

    static ros::Time last_arm_time = ros::Time(0);
    if (ros::Time::now() - last_arm_time < ros::Duration(2.0)) return false;

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    last_arm_time = ros::Time::now();

    return arming_client_.call(arm_cmd) && arm_cmd.response.success;
}

bool DroneController::takeoff(float altitude) {
    mavros_msgs::CommandTOL to;
    to.request.altitude = altitude;
    return takeoff_client_.call(to) && to.response.success;
}
