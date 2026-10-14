#include "human_tracking/Drone.hpp"

DroneController::DroneController(ros::NodeHandle& nh, const std::string& drone_ns) : nh_(nh), ns_(drone_ns) {

    std::string prefix = "";
    if (!ns_.empty()) {
        prefix = (ns_[0] == '/') ? ns_ : "/" + ns_;
    }

    state_sub_   = nh_.subscribe<mavros_msgs::State>(prefix + "/mavros/state", 10, &DroneController::stateCallback, this);
    battery_sub_ = nh_.subscribe<sensor_msgs::BatteryState>(prefix + "/mavros/battery", 10, &DroneController::batteryCallback, this);
    vel_pub_     = nh_.advertise<geometry_msgs::Twist>(prefix + "/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

    arming_client_   = nh_.serviceClient<mavros_msgs::CommandBool>(prefix + "/mavros/cmd/arming");
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>(prefix + "/mavros/set_mode");
    takeoff_client_  = nh_.serviceClient<mavros_msgs::CommandTOL>(prefix + "/mavros/cmd/takeoff");

    forward_dir_ = ForwardAxis::Y_AXIS;
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

void DroneController::publishVelocity(float forward_vel, float lateral_vel, float yaw_rate) {
    geometry_msgs::Twist cmd;

    cmd.linear.y  = forward_vel;
    cmd.linear.x  = lateral_vel;
    cmd.angular.z = yaw_rate;

    if (std::abs(forward_vel) < 0.01 && std::abs(lateral_vel) < 0.01 && std::abs(yaw_rate) < 0.01) {
        dist_integral_ = 0; lat_integral_ = 0; yaw_integral_ = 0;
        prev_dist_err_ = 0; prev_lat_err_ = 0; prev_yaw_err_ = 0;
    }

    vel_pub_.publish(cmd);
}

void DroneController::runFollowLogic(const human_tracking::PersonBBox& msg, float current_range) {
    if (current_range <= 0.1 || current_range > 20.0) {
        ROS_WARN_THROTTLE(1.0, "[%s_FOLLOW] Invalid distance data: %.2f -- stopping drone.", ns_.c_str(), current_range);
        publishVelocity(0.0, 0.0, 0.0);
        return;
    }

    float target_dist = 3.0;
    float dist_err = current_range - target_dist;

    dist_integral_ += dist_err;
    if (std::abs(dist_integral_) > 0.2)
        dist_integral_ = (dist_integral_ > 0) ? 0.2 : -0.2;

    float dist_deriv = dist_err - prev_dist_err_;
    prev_dist_err_ = dist_err;

    float move_val = (0.40 * dist_err) + (0.02 * dist_integral_) + (0.25 * dist_deriv);

    if (current_range < 2.2) {
        float escape_diff = 2.2 - current_range;
        move_val = -0.40 - (escape_diff * 0.8);
    }

    if (std::abs(dist_err) < 0.30) {
        move_val       = 0.0;
        dist_integral_ = 0;
        prev_dist_err_ = 0;
    }

    float lat_err = msg.cx - msg.img_width / 2.0;

    lat_integral_ += lat_err;
    if (std::abs(lat_integral_) > 1.0)
        lat_integral_ = (lat_integral_ > 0) ? 1.0 : -1.0;

    float lat_deriv = lat_err - prev_lat_err_;
    prev_lat_err_ = lat_err;

    float lat_vel = (0.0020 * lat_err) + (0.00005 * lat_integral_) + (0.0004 * lat_deriv);

    if (std::abs(lat_err) < 20.0) lat_vel = 0.0;

    float yaw_rate = 0.0;
    if (std::abs(lat_err) > 70.0) {
        yaw_integral_ += lat_err;
        float yaw_deriv = lat_err - prev_yaw_err_;
        prev_yaw_err_ = lat_err;
        yaw_rate = -((0.0008 * lat_err) + (0.00005 * yaw_integral_) + (0.0004 * yaw_deriv));
    } else {
        yaw_integral_ = 0;
    }

    move_val = std::max(-0.80f, std::min(0.45f, move_val));
    lat_vel  = std::max(-0.50f, std::min(0.50f, lat_vel));
    yaw_rate = std::max(-0.50f, std::min(0.50f, yaw_rate));

    publishVelocity(move_val, lat_vel, yaw_rate);

    ROS_INFO_THROTTLE(0.5, "[%s_FOLLOW] Distance: %.2fm | Fwd: %.2f | Lat: %.2f | Yaw: %.2f",
                      ns_.c_str(), current_range, move_val, lat_vel, yaw_rate);
}
