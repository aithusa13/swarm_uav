#include <ros/ros.h>
#include "kamikaze_dive/DiveController.hpp"

static float P_CRUISE_ALT   = 7.0f;
static float P_CRUISE_DIST  = 5.0f;
static float P_DIVE_ALT     = 1.0f;
static float P_DIVE_FWD_VEL = 3.0f;
static float P_DIVE_DN_VEL  = 4.0f;
static float P_PULLOUT_ALT  = 1.0f;

enum class State {
    PRECHECK,
    TAKEOFF,
    CLIMB,
    CRUISE,
    DIVE,
    PULLOUT,
    LAND
};

static const char* stateName(State s)
{
    switch (s) {
        case State::PRECHECK:  return "PRECHECK";
        case State::TAKEOFF:   return "TAKEOFF";
        case State::CLIMB:     return "CLIMB";
        case State::CRUISE:    return "CRUISE";
        case State::DIVE:      return "DIVE";
        case State::PULLOUT:   return "PULLOUT";
        case State::LAND:      return "LAND";
    }
    return "UNKNOWN";
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "kamikaze_dive_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    nh_priv.param("cruise_alt",   P_CRUISE_ALT,   P_CRUISE_ALT);
    nh_priv.param("cruise_dist",  P_CRUISE_DIST,  P_CRUISE_DIST);
    nh_priv.param("dive_alt",     P_DIVE_ALT,     P_DIVE_ALT);
    nh_priv.param("dive_fwd_vel", P_DIVE_FWD_VEL, P_DIVE_FWD_VEL);
    nh_priv.param("dive_dn_vel",  P_DIVE_DN_VEL,  P_DIVE_DN_VEL);
    nh_priv.param("pullout_alt",  P_PULLOUT_ALT,  P_PULLOUT_ALT);

    DiveController drone(nh);

    ros::Rate rate(20);

    State current_state  = State::PRECHECK;
    State last_state_log = static_cast<State>(-1);

    ROS_INFO("============================================================");
    ROS_INFO("  Kamikaze Dive Node");
    ROS_INFO("  Cruise alt : %.1f m", P_CRUISE_ALT);
    ROS_INFO("  Cruise dist: %.1f m", P_CRUISE_DIST);
    ROS_INFO("  Dive target: %.1f m", P_DIVE_ALT);
    ROS_INFO("  Dive fwd   : %.1f m/s", P_DIVE_FWD_VEL);
    ROS_INFO("  Dive down  : %.1f m/s", P_DIVE_DN_VEL);
    ROS_INFO("============================================================");

    while (ros::ok())
    {
        ros::spinOnce();

        if (current_state != last_state_log) {
            ROS_INFO("[STATE] %-10s -> %s",
                     stateName(last_state_log), stateName(current_state));
            last_state_log = current_state;
        }

        switch (current_state)
        {
            case State::PRECHECK:
            {
                if (drone.checkSystemHealth()) {
                    if (!drone.isStreamRateConfigured()) {
                        ROS_INFO("[PRECHECK] FCU connected. Configuring stream rate...");
                        drone.configureStreamRate();
                    } else {
                        current_state = State::TAKEOFF;
                    }
                } else {
                    ROS_INFO_THROTTLE(5, "[PRECHECK] Waiting for FCU and pre-arm...");
                }
                break;
            }

            case State::TAKEOFF:
            {
                if (drone.armDrone()) {
                    ROS_INFO("[TAKEOFF] Waiting for motor stabilization (3s)...");
                    ros::Duration(3.0).sleep();

                    if (drone.takeoff(3.0f)) {
                        ROS_INFO("[TAKEOFF] Command sent. CLIMB phase starting...");
                        ros::Duration(3.0).sleep();
                        current_state = State::CLIMB;
                    }
                }
                break;
            }

            case State::CLIMB:
            {
                bool reached = drone.holdAltitude(P_CRUISE_ALT,
                                                   0.20f,
                                                   2.0f);
                if (reached) {
                    ROS_INFO("[CLIMB] Reached %.1f m. CRUISE starting...", P_CRUISE_ALT);
                    drone.resetCruiseOrigin();
                    current_state = State::CRUISE;
                }
                break;
            }

            case State::CRUISE:
            {
                bool reached = drone.cruiseForward(P_CRUISE_DIST, P_CRUISE_ALT);
                if (reached) {
                    ROS_WARN("[CRUISE] Advanced %.1f m. DIVE starting!", P_CRUISE_DIST);
                    drone.stop();
                    ros::Duration(0.5).sleep();
                    current_state = State::DIVE;
                }
                break;
            }

            case State::DIVE:
            {
                bool hit = drone.kamikazeDive(P_DIVE_ALT,
                                               P_DIVE_FWD_VEL,
                                               P_DIVE_DN_VEL);
                if (hit) {
                    current_state = State::PULLOUT;
                }
                break;
            }

            case State::PULLOUT:
            {
                bool stable = drone.pullOut(P_PULLOUT_ALT, 2.0f);
                if (stable) {
                    ROS_INFO("[PULLOUT] Stabilization OK. Landing...");
                    current_state = State::LAND;
                }
                break;
            }

            case State::LAND:
            {
                drone.land();
                ROS_INFO("[LAND] Mission complete.");
                ros::Duration(10.0).sleep();
                return 0;
            }
        }

        rate.sleep();
    }

    return 0;
}
