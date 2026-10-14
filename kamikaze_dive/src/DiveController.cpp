#include "kamikaze_dive/DiveController.hpp"
#include <cmath>
#include <algorithm>

DiveController::DiveController(ros::NodeHandle& nh) : nh_(nh)
{
    state_sub_ = nh_.subscribe<mavros_msgs::State>(
        "/mavros/state", 10,
        &DiveController::stateCallback, this);

    pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
        "/mavros/local_position/pose", 10,
        &DiveController::poseCallback, this);

    vel_pub_ = nh_.advertise<geometry_msgs::Twist>(
        "/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

    arming_client_      = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    set_mode_client_    = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    takeoff_client_     = nh_.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    stream_rate_client_ = nh_.serviceClient<mavros_msgs::StreamRate>("/mavros/set_stream_rate");
}

void DiveController::stateCallback(const mavros_msgs::State::ConstPtr& msg)
{
    current_state_ = *msg;
}

void DiveController::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    float z = static_cast<float>(msg->pose.position.z);

    if (!telem_.valid) {
        origin_alt_ = z;
        ROS_INFO("[POSE] Origin altitude locked: %.2f m (ENU z)", origin_alt_);
    }

    telem_.altitude_m = z - origin_alt_;

    float y = static_cast<float>(msg->pose.position.y);
    telem_.forward_dist = y - cruise_origin_;

    telem_.valid = true;
}

float DiveController::computePid(float error, PidState& st,
                                  float kp, float ki, float kd,
                                  float integral_clamp,
                                  float dt)
{
    st.integral += error * dt;
    st.integral  = std::max(-integral_clamp,
                   std::min( integral_clamp, st.integral));

    float derivative = (error - st.prev_error) / dt;
    st.prev_error    = error;

    return (kp * error) + (ki * st.integral) + (kd * derivative);
}

bool DiveController::checkSystemHealth()
{
    if (!current_state_.connected) {
        ROS_WARN_THROTTLE(5, "[PRECHECK] No FCU connection...");
        return false;
    }
    if (current_state_.system_status < 3) {
        ROS_WARN_THROTTLE(5, "[PRECHECK] Pre-arm checks not passed. Status: %d",
                          current_state_.system_status);
        return false;
    }
    return true;
}

void DiveController::configureStreamRate()
{
    if (!stream_rate_client_.waitForExistence(ros::Duration(5.0))) {
        ROS_WARN("[STREAM] set_stream_rate service not found.");
        return;
    }

    mavros_msgs::StreamRate req;
    req.request.stream_id    = 0;
    req.request.message_rate = 20;
    req.request.on_off       = true;

    if (stream_rate_client_.call(req)) {
        ROS_INFO("[STREAM] Stream rate set to 20 Hz.");
        stream_rate_configured_ = true;
    } else {
        ROS_WARN("[STREAM] Failed to set stream rate.");
    }
}

bool DiveController::setMode(const std::string& mode)
{
    mavros_msgs::SetMode sm;
    sm.request.custom_mode = mode;

    if (set_mode_client_.call(sm) && sm.response.mode_sent) {
        ROS_INFO("[MODE] Mode changed: %s", mode.c_str());
        return true;
    }
    ROS_WARN("[MODE] Failed to change mode: %s", mode.c_str());
    return false;
}

bool DiveController::armDrone()
{
    if (!checkSystemHealth()) return false;

    if (current_state_.mode != "GUIDED") {
        setMode("GUIDED");
        ros::Duration(1.0).sleep();
        return false;
    }

    if (current_state_.armed) return true;

    static ros::Time last_attempt = ros::Time(0);
    if (ros::Time::now() - last_attempt < ros::Duration(2.0)) return false;
    last_attempt = ros::Time::now();

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    if (arming_client_.call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("[ARM] Arm command accepted. Waiting for confirmation...");
        ros::Time start = ros::Time::now();
        while (!current_state_.armed && (ros::Time::now() - start).toSec() < 5.0) {
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }
        if (current_state_.armed) {
            ROS_INFO("[ARM] Arm confirmed.");
            return true;
        }
        ROS_WARN("[ARM] Arm confirmation timeout.");
        return false;
    }

    ROS_WARN("[ARM] FCU rejected arm command.");
    return false;
}

bool DiveController::takeoff(float altitude_m)
{
    mavros_msgs::CommandTOL to;
    to.request.altitude = altitude_m;

    if (takeoff_client_.call(to) && to.response.success) {
        ROS_INFO("[TAKEOFF] Command sent. Target: %.1f m", altitude_m);
        return true;
    }
    ROS_WARN("[TAKEOFF] Command failed.");
    return false;
}

void DiveController::land()
{
    setMode("LAND");
    ROS_INFO("[LAND] Land mode active.");
}

void DiveController::publishVelocity(float fwd, float lat,
                                      float vert, float yaw_rate)
{
    geometry_msgs::Twist cmd;
    cmd.linear.x  = lat;
    cmd.linear.y  = fwd;
    cmd.linear.z  = vert;
    cmd.angular.z = yaw_rate;

    vel_pub_.publish(cmd);
}

void DiveController::stop()
{
    publishVelocity(0.0f, 0.0f, 0.0f, 0.0f);
    alt_pid_  = PidState{};
    fwd_pid_  = PidState{};
    dive_pid_ = PidState{};
}

void DiveController::resetCruiseOrigin()
{
    cruise_origin_ += telem_.forward_dist;
    telem_.forward_dist = 0.0f;
    ROS_INFO("[CRUISE] Progress reset. Start point locked.");
}

bool DiveController::holdAltitude(float target_alt,
                                   float tolerance,
                                   float hold_time_s)
{
    if (!telem_.valid) return false;

    static ros::Time last_time = ros::Time(0);
    ros::Time now = ros::Time::now();
    float dt = (last_time == ros::Time(0)) ? 0.05f
                                           : static_cast<float>((now - last_time).toSec());
    last_time = now;
    dt = std::max(0.005f, std::min(0.5f, dt));

    float err = target_alt - telem_.altitude_m;

    float vert_vel = computePid(err, alt_pid_,
                                 1.2f,
                                 0.05f,
                                 0.40f,
                                 1.0f,
                                 dt);

    vert_vel = std::max(-2.0f, std::min(2.0f, vert_vel));

    publishVelocity(0.0f, 0.0f, vert_vel);

    ROS_INFO_THROTTLE(0.5,
        "[ALT HOLD] Alt: %.2f m | Target: %.2f m | Err: %.2f | Vel: %.2f",
        telem_.altitude_m, target_alt, err, vert_vel);

    bool in_tol = (std::abs(err) < tolerance);
    if (in_tol) {
        if (!within_tol_active_) {
            within_tol_start_  = now;
            within_tol_active_ = true;
        } else if ((now - within_tol_start_).toSec() >= hold_time_s) {
            within_tol_active_ = false;
            return true;
        }
    } else {
        within_tol_active_ = false;
    }
    return false;
}

bool DiveController::cruiseForward(float target_fwd_m, float cruise_alt)
{
    if (!telem_.valid) return false;

    static ros::Time last_time = ros::Time(0);
    ros::Time now = ros::Time::now();
    float dt = (last_time == ros::Time(0)) ? 0.05f
                                           : static_cast<float>((now - last_time).toSec());
    last_time = now;
    dt = std::max(0.005f, std::min(0.5f, dt));

    float fwd_err = target_fwd_m - telem_.forward_dist;
    float fwd_vel = computePid(fwd_err, fwd_pid_,
                                0.8f,
                                0.02f,
                                0.30f,
                                2.0f,
                                dt);
    fwd_vel = std::max(-1.5f, std::min(1.5f, fwd_vel));

    float alt_err  = cruise_alt - telem_.altitude_m;
    float vert_vel = computePid(alt_err, alt_pid_,
                                 1.2f,
                                 0.05f,
                                 0.40f,
                                 1.0f,
                                 dt);
    vert_vel = std::max(-1.5f, std::min(1.5f, vert_vel));

    publishVelocity(fwd_vel, 0.0f, vert_vel);

    ROS_INFO_THROTTLE(0.5,
        "[CRUISE] Fwd: %.2f/%.2f m | Alt: %.2f/%.2f m | Vfwd: %.2f | Vz: %.2f",
        telem_.forward_dist, target_fwd_m,
        telem_.altitude_m,   cruise_alt,
        fwd_vel, vert_vel);

    return (telem_.forward_dist >= target_fwd_m - 0.20f);
}

bool DiveController::kamikazeDive(float target_alt,
                                   float dive_fwd_vel,
                                   float dive_dn_vel)
{
    if (!telem_.valid) return false;

    if (!dive_time_set_) {
        dive_enter_time_ = ros::Time::now();
        dive_time_set_   = true;
        ROS_WARN("[DIVE] *** KAMIKAZE DIVE STARTED *** Target: %.1f m", target_alt);
    }

    float elapsed = static_cast<float>((ros::Time::now() - dive_enter_time_).toSec());

    static ros::Time last_time = ros::Time(0);
    ros::Time now = ros::Time::now();
    float dt = (last_time == ros::Time(0)) ? 0.05f
                                           : static_cast<float>((now - last_time).toSec());
    last_time = now;
    dt = std::max(0.005f, std::min(0.5f, dt));

    float alt_err  = telem_.altitude_m - target_alt;
    float dive_pid_out = computePid(alt_err, dive_pid_,
                                     1.5f,
                                     0.0f,
                                     0.30f,
                                     1.0f,
                                     dt);

    float vert_vel = -(dive_dn_vel + std::max(0.0f, dive_pid_out));

    vert_vel = std::max(-5.0f, std::min(0.0f, vert_vel));

    float ramp   = std::min(1.0f, elapsed / 0.5f);
    float fwd_vel = dive_fwd_vel * ramp;
    fwd_vel = std::max(0.0f, std::min(4.0f, fwd_vel));

    publishVelocity(fwd_vel, 0.0f, vert_vel);

    ROS_WARN_THROTTLE(0.2,
        "[DIVE] Alt: %.2f m | Target: %.2f m | Vfwd: %.2f | Vdn: %.2f | t: %.1fs",
        telem_.altitude_m, target_alt, fwd_vel, vert_vel, elapsed);

    if (telem_.altitude_m <= target_alt + 0.3f) {
        dive_time_set_ = false;
        ROS_WARN("[DIVE] Target altitude reached! Pullout starting...");
        return true;
    }

    if (elapsed > 15.0f) {
        ROS_ERROR("[DIVE] Timeout! Emergency -- dive aborted.");
        stop();
        dive_time_set_ = false;
        return true;
    }

    return false;
}

bool DiveController::pullOut(float hold_alt, float hold_time_s)
{
    if (!telem_.valid) return false;

    static ros::Time last_time = ros::Time(0);
    ros::Time now = ros::Time::now();
    float dt = (last_time == ros::Time(0)) ? 0.05f
                                           : static_cast<float>((now - last_time).toSec());
    last_time = now;
    dt = std::max(0.005f, std::min(0.5f, dt));

    float err = hold_alt - telem_.altitude_m;

    float vert_vel = computePid(err, alt_pid_,
                                 3.0f,
                                 0.10f,
                                 0.80f,
                                 2.0f,
                                 dt);

    vert_vel = std::max(-0.5f, std::min(4.0f, vert_vel));

    publishVelocity(0.0f, 0.0f, vert_vel);

    ROS_INFO_THROTTLE(0.3,
        "[PULLOUT] Alt: %.2f m | Hold: %.2f m | Err: %.2f | Vz: %.2f",
        telem_.altitude_m, hold_alt, err, vert_vel);

    bool in_tol = (std::abs(err) < 0.20f);
    if (in_tol) {
        if (!within_tol_active_) {
            within_tol_start_  = now;
            within_tol_active_ = true;
        } else if ((now - within_tol_start_).toSec() >= hold_time_s) {
            within_tol_active_ = false;
            ROS_INFO("[PULLOUT] Stabilization complete.");
            return true;
        }
    } else {
        within_tol_active_ = false;
    }

    return false;
}
