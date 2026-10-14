#ifndef DIVE_CONTROLLER_HPP
#define DIVE_CONTROLLER_HPP

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/StreamRate.h>
#include <string>

struct PidState {
    float integral   = 0.0f;
    float prev_error = 0.0f;
};

struct TelemetryData {
    float altitude_m    = 0.0f;
    float forward_dist  = 0.0f;
    bool  valid         = false;
};

class DiveController {
public:
    explicit DiveController(ros::NodeHandle& nh);

    bool checkSystemHealth();
    void configureStreamRate();
    bool isStreamRateConfigured() const { return stream_rate_configured_; }
    bool isArmed()                const { return current_state_.armed; }

    bool armDrone();
    bool setMode(const std::string& mode);
    bool takeoff(float altitude_m);
    void land();

    void publishVelocity(float fwd, float lat, float vert, float yaw_rate = 0.0f);
    void stop();

    bool holdAltitude(float target_alt,
                      float tolerance   = 0.15f,
                      float hold_time_s = 1.5f);

    bool cruiseForward(float target_fwd_m, float cruise_alt = 7.0f);

    bool kamikazeDive(float target_alt  = 1.0f,
                      float dive_fwd_vel = 3.0f,
                      float dive_dn_vel  = 4.0f);

    bool pullOut(float hold_alt = 1.0f, float hold_time_s = 2.0f);

    const TelemetryData& telemetry() const { return telem_; }

    void resetCruiseOrigin();

private:
    void stateCallback(const mavros_msgs::State::ConstPtr& msg);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    float computePid(float error, PidState& st,
                     float kp, float ki, float kd,
                     float integral_clamp,
                     float dt);

    ros::NodeHandle    nh_;
    ros::Subscriber    state_sub_;
    ros::Subscriber    pose_sub_;
    ros::Publisher     vel_pub_;
    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;
    ros::ServiceClient takeoff_client_;
    ros::ServiceClient stream_rate_client_;

    mavros_msgs::State current_state_;
    bool               stream_rate_configured_ = false;

    TelemetryData telem_;
    float         origin_alt_    = 0.0f;
    float         cruise_origin_ = 0.0f;

    PidState alt_pid_;
    PidState fwd_pid_;
    PidState dive_pid_;

    ros::Time phase_enter_time_;
    bool      phase_time_set_ = false;

    ros::Time within_tol_start_;
    bool      within_tol_active_ = false;

    ros::Time dive_enter_time_;
    bool      dive_time_set_ = false;
};

#endif
