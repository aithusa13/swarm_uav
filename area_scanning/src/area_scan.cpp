#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <mavros_msgs/CommandTOL.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <cstdlib>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>
#include <csignal>
#include <string>

#include "area_scanning/Drone.hpp"

static ros::ServiceClient* g_land_client = nullptr;
static DroneController*    g_drone_ptr   = nullptr;

void sigintHandler(int)
{
    ROS_WARN("[SYSTEM] SIGINT! Landing...");
    if (g_land_client) {
        mavros_msgs::CommandTOL land;
        land.request.altitude = 0.0f;
        g_land_client->call(land);
    }
    ros::shutdown();
}

static bool jsonGetFloat(const std::string& j, const std::string& key, float& out)
{
    std::string s = "\"" + key + "\"";
    size_t p = j.find(s);
    if (p == std::string::npos) return false;
    p = j.find(":", p); if (p == std::string::npos) return false;
    ++p;
    while (p < j.size() && (j[p] == ' ' || j[p] == '\t')) ++p;
    out = static_cast<float>(std::atof(j.c_str() + p));
    return true;
}

static bool jsonGetBool(const std::string& j, const std::string& key, bool& out)
{
    std::string s = "\"" + key + "\"";
    size_t p = j.find(s);
    if (p == std::string::npos) return false;
    p = j.find(":", p); if (p == std::string::npos) return false;
    ++p;
    while (p < j.size() && (j[p] == ' ' || j[p] == '\t')) ++p;
    if (j.substr(p, 4) == "true")  { out = true;  return true; }
    if (j.substr(p, 5) == "false") { out = false; return true; }
    return false;
}

static bool  g_person_detected  = false;
static float g_person_cx        = 0.0f;
static float g_person_cy        = 0.0f;
static float g_pos_x            = 0.0f;
static float g_pos_y            = 0.0f;
static float g_pos_z            = 0.0f;
static float g_yaw_rad          = 0.0f;
static float g_lidar_floor_dist = -1.0f;

void bboxCallback(const std_msgs::String::ConstPtr& msg)
{
    bool detected = false;
    jsonGetBool(msg->data, "detected", detected);
    g_person_detected = detected;
    if (detected) {
        jsonGetFloat(msg->data, "cx_err", g_person_cx);
        jsonGetFloat(msg->data, "cy_err", g_person_cy);
    }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    g_pos_x = static_cast<float>(msg->pose.pose.position.x);
    g_pos_y = static_cast<float>(msg->pose.pose.position.y);
    g_pos_z = static_cast<float>(msg->pose.pose.position.z);

    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    g_yaw_rad = static_cast<float>(yaw);
}

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    if (!msg->ranges.empty()) {
        float r = msg->ranges[msg->ranges.size() / 2];
        if (std::isfinite(r) && r > 0.1f)
            g_lidar_floor_dist = r;
    }
}

struct PID {
    float kp, ki, kd, i_limit;
    float integral   = 0.0f;
    float prev_error = 0.0f;

    PID(float p, float i, float d, float ilim = 2.0f)
        : kp(p), ki(i), kd(d), i_limit(ilim) {}

    float update(float error, float dt)
    {
        integral += error * dt;
        integral  = std::max(-i_limit, std::min(i_limit, integral));
        float deriv = (error - prev_error) / dt;
        prev_error  = error;
        return kp * error + ki * integral + kd * deriv;
    }
    void reset() { integral = 0.0f; prev_error = 0.0f; }
};

enum class ScanPattern { S_SHAPE, ZIGZAG };

struct ScanParams {
    float       a       = 2.0f;
    int         k       = 2;
    ScanPattern pattern = ScanPattern::S_SHAPE;
};

static ScanParams readParams()
{
    ScanParams p;
    std::cout << "\n=== AREA SCAN PARAMETERS ===\n";
    while (true) {
        std::cout << "  Step distance  a  [1-10 m]: ";
        if (std::cin >> p.a && p.a >= 1.0f && p.a <= 10.0f) break;
        std::cout << "  ! Must be between 1 and 10.\n";
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
    while (true) {
        std::cout << "  Strip count    k  [1-10]: ";
        if (std::cin >> p.k && p.k >= 1 && p.k <= 10) break;
        std::cout << "  ! Must be between 1 and 10.\n";
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
    int choice = 0;
    while (true) {
        std::cout << "  Pattern: [1] S-Shape  [2] Zigzag : ";
        if (std::cin >> choice && (choice == 1 || choice == 2)) break;
        std::cout << "  ! Enter 1 or 2.\n";
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
    p.pattern = (choice == 2) ? ScanPattern::ZIGZAG : ScanPattern::S_SHAPE;
    std::cout << "  a=" << p.a << " m  |  k=" << p.k
              << "  |  Pattern: " << (p.pattern == ScanPattern::S_SHAPE ? "S-Shape" : "Zigzag")
              << "\n============================\n\n";
    return p;
}

struct WP { float x; float y; };

static std::vector<WP> buildWaypoints(const ScanParams& p, float ox, float oy)
{
    std::vector<WP> wps;
    const float a = p.a;

    if (p.pattern == ScanPattern::S_SHAPE) {
        for (int i = 0; i < p.k; ++i) {
            float y_top = oy - static_cast<float>(i * 2) * a;
            float y_bot = y_top - a;

            wps.push_back({ox - a, y_top    });
            wps.push_back({ox - a, y_bot    });
            wps.push_back({ox,     y_bot    });
            wps.push_back({ox,     y_bot - a});
        }
    } else {
        for (int i = 0; i < p.k; ++i) {
            float y = oy - static_cast<float>(i) * a;

            wps.push_back({ox - a, y    });
            wps.push_back({ox,     y - a});
        }
    }

    return wps;
}

enum class State { PRECHECK, TAKEOFF, SCAN, HOVER_WAIT, BACK_MOVE, DESCEND_CHECK, RTL };

static const char* stateName(State s)
{
    switch (s) {
        case State::PRECHECK:      return "PRECHECK";
        case State::TAKEOFF:       return "TAKEOFF";
        case State::SCAN:          return "SCAN";
        case State::HOVER_WAIT:    return "HOVER_WAIT";
        case State::BACK_MOVE:     return "BACK_MOVE";
        case State::DESCEND_CHECK: return "DESCEND_CHECK";
        case State::RTL:           return "RTL";
    }
    return "UNKNOWN";
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "area_scan_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    ScanParams params = readParams();

    ros::ServiceClient land_client =
        nh.serviceClient<mavros_msgs::CommandTOL>("/drone2/mavros/cmd/land");
    g_land_client = &land_client;

    DroneController drone(nh, "drone2");
    g_drone_ptr = &drone;

    std::signal(SIGINT, sigintHandler);

    ros::Publisher vel_pub =
        nh.advertise<geometry_msgs::Twist>(
            "/drone2/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

    ros::Subscriber bbox_sub  = nh.subscribe("/person_bbox",                                 10, bboxCallback);
    ros::Subscriber odom_sub  = nh.subscribe("/drone2/mavros/local_position/odom",          10, odomCallback);
    ros::Subscriber lidar_sub = nh.subscribe("/drone2/laser/scan", 10, lidarCallback);

    ros::Rate rate(10);
    const float DT = 0.1f;

    PID pid_x(0.60f, 0.01f, 0.25f, 1.0f);
    PID pid_y(0.60f, 0.01f, 0.25f, 1.0f);
    PID pid_yaw(0.90f, 0.005f, 0.30f, 0.5f);

    const float MAX_VEL       = 0.45f;
    const float MAX_YAW       = 0.40f;
    const float ARRIVE_THRESH = 0.15f;

    State current_state  = State::PRECHECK;
    State last_state_log = State::PRECHECK;

    std::vector<WP> waypoints;
    size_t wp_idx    = 0;
    bool   wps_built = false;

    float locked_yaw   = 0.0f;
    const float CRUISE_ALT = 4.0f;

    int hover_ticks = 0;
    const int HOVER_TARGET_TICKS = 35;
    int back_ticks = 0;
    const int BACK_TARGET_TICKS = 15;

    ROS_INFO("[SYSTEM] Area scan node active.");

    while (ros::ok())
    {
        ros::spinOnce();

        if (current_state != last_state_log) {
            ROS_INFO("[STATE] %s -> %s", stateName(last_state_log), stateName(current_state));
            last_state_log = current_state;
        }

        switch (current_state)
        {
            case State::PRECHECK:
            {
                if (drone.checkSystemHealth())
                    current_state = State::TAKEOFF;
                else
                    ROS_INFO_THROTTLE(5, "[PRECHECK] Waiting for FCU...");
                break;
            }

            case State::TAKEOFF:
            {
                if (drone.armDrone()) {
                    ros::Duration(2.0).sleep();
                    if (drone.takeoff(CRUISE_ALT)) {
                        ros::Duration(6.0).sleep();
                        locked_yaw = g_yaw_rad;
                        ROS_INFO("[TAKEOFF] Locked yaw: %.4f rad", locked_yaw);
                        current_state = State::SCAN;
                    }
                }
                break;
            }

            case State::SCAN:
            {
                if (!wps_built) {
                    waypoints = buildWaypoints(params, g_pos_x, g_pos_y);
                    wps_built = true;
                    wp_idx    = 0;
                    pid_x.reset(); pid_y.reset(); pid_yaw.reset();
                    ROS_INFO("[SCAN] %zu waypoints loaded. Start: (%.2f, %.2f)",
                             waypoints.size(), g_pos_x, g_pos_y);
                    for (size_t i = 0; i < waypoints.size(); ++i)
                        ROS_INFO("[SCAN]  WP[%zu]: (%.2f, %.2f)", i, waypoints[i].x, waypoints[i].y);
                }

                if (g_person_detected) {
                    ROS_INFO("[SCAN] Person detected! HOVER_WAIT.");
                    geometry_msgs::Twist stop;
                    vel_pub.publish(stop);
                    hover_ticks = 0;
                    current_state = State::HOVER_WAIT;
                    break;
                }

                if (wp_idx >= waypoints.size()) {
                    ROS_INFO("[SCAN] Scan complete. RTL.");
                    current_state = State::RTL;
                    break;
                }

                const WP& tgt = waypoints[wp_idx];
                float dx   = tgt.x - g_pos_x;
                float dy   = tgt.y - g_pos_y;
                float dist = std::sqrt(dx*dx + dy*dy);

                if (dist < ARRIVE_THRESH) {
                    ROS_INFO("[SCAN] WP[%zu] reached.", wp_idx);
                    ++wp_idx;
                    pid_x.reset(); pid_y.reset();
                    geometry_msgs::Twist stop;
                    vel_pub.publish(stop);
                    break;
                }

                float vx = pid_x.update(dx, DT);
                float vy = pid_y.update(dy, DT);
                vx = std::max(-MAX_VEL, std::min(MAX_VEL, vx));
                vy = std::max(-MAX_VEL, std::min(MAX_VEL, vy));

                float yaw_err = locked_yaw - g_yaw_rad;
                while (yaw_err >  M_PI) yaw_err -= 2.0f * M_PI;
                while (yaw_err < -M_PI) yaw_err += 2.0f * M_PI;
                float vyaw = pid_yaw.update(yaw_err, DT);
                vyaw = std::max(-MAX_YAW, std::min(MAX_YAW, vyaw));

                geometry_msgs::Twist cmd;
                cmd.linear.x  = vx;
                cmd.linear.y  = vy;
                cmd.angular.z = vyaw;
                vel_pub.publish(cmd);

                ROS_INFO_THROTTLE(0.5,
                    "[SCAN] WP[%zu] -> (%.2f,%.2f) | dist=%.2f | vx=%.2f vy=%.2f yaw=%.2f",
                    wp_idx, tgt.x, tgt.y, dist, vx, vy, vyaw);
                break;
            }

            case State::HOVER_WAIT:
            {
                geometry_msgs::Twist stop;
                vel_pub.publish(stop);
                if (++hover_ticks >= HOVER_TARGET_TICKS) {
                    back_ticks = 0;
                    current_state = State::BACK_MOVE;
                }
                ROS_INFO_THROTTLE(1.0, "[HOVER] %d/%d", hover_ticks, HOVER_TARGET_TICKS);
                break;
            }

            case State::BACK_MOVE:
            {
                geometry_msgs::Twist cmd;
                cmd.linear.y = 0.35f;
                vel_pub.publish(cmd);
                if (++back_ticks >= BACK_TARGET_TICKS) {
                    geometry_msgs::Twist stop;
                    vel_pub.publish(stop);
                    current_state = State::DESCEND_CHECK;
                }
                break;
            }

            case State::DESCEND_CHECK:
            {
                if (g_lidar_floor_dist > 0.1f && g_lidar_floor_dist < 2.0f) {
                    ROS_WARN("[DESCEND_CHECK] Risk! Lidar: %.2f m -> RTL.", g_lidar_floor_dist);
                    current_state = State::RTL;
                    break;
                }
                if (g_pos_z <= 1.10f) {
                    ROS_INFO("[DESCEND_CHECK] Descended to 1 meter. RTL.");
                    current_state = State::RTL;
                    break;
                }
                geometry_msgs::Twist cmd;
                cmd.linear.z = -0.3f;
                vel_pub.publish(cmd);
                ROS_INFO_THROTTLE(1.0, "[DESCEND_CHECK] z=%.2f | lidar=%.2f", g_pos_z, g_lidar_floor_dist);
                break;
            }

            case State::RTL:
            {
                static bool rtl_sent = false;
                if (!rtl_sent) {
                    drone.setMode("RTL");
                    rtl_sent = true;
                    ROS_INFO("[RTL] Return mode active.");
                }
                break;
            }
        }

        rate.sleep();
    }

    return 0;
}
