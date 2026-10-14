#include <ros/ros.h>
#include "human_tracking/Drone.hpp"
#include <human_tracking/PersonBBox.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>

enum class State { PRECHECK, TAKEOFF, SCAN, FOLLOW };

float g_target_distance = -1.0;
human_tracking::PersonBBox g_last_bbox;
bool g_person_detected = false;
ros::Time g_last_seen_time;

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    int center = msg->ranges.size() / 2;
    float min_dist = msg->range_max;

    for (int i = center - 30; i <= center + 30; i++) {
        if (i >= 0 && i < msg->ranges.size()) {
            float r = msg->ranges[i];
            if (std::isfinite(r) && r > 0.3 && r < min_dist)
                min_dist = r;
        }
    }
    g_target_distance = (min_dist == msg->range_max) ? -1.0 : min_dist;
}

void bboxCallback(const human_tracking::PersonBBox::ConstPtr& msg) {
    if (msg->authorized) {
        g_last_bbox = *msg;
        g_person_detected = true;
        g_last_seen_time = ros::Time::now();
    } else {
        g_person_detected = false;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "follow_person_node");
    ros::NodeHandle nh;

    DroneController drone(nh, "drone1");

    ros::Subscriber lidar_sub = nh.subscribe("/drone1/laser/scan", 10, lidarCallback);
    ros::Subscriber bbox_sub  = nh.subscribe("/drone1/person_bbox", 10, bboxCallback);

    ros::Rate rate(10);
    State current_state = State::PRECHECK;

    ROS_INFO("[FOLLOW_NODE] Mission started. Waiting for precheck...");

    while (ros::ok()) {
        ros::spinOnce();

        switch (current_state) {
            case State::PRECHECK:
                if (drone.checkSystemHealth()) {
                    ROS_INFO("[FOLLOW_NODE] System healthy, switching to takeoff.");
                    current_state = State::TAKEOFF;
                }
                break;

            case State::TAKEOFF:
                if (drone.armDrone()) {
                    if (drone.takeoff(1.0)) {
                        ros::Duration(5.0).sleep();
                        current_state = State::SCAN;
                    }
                }
                break;

            case State::SCAN:
                drone.publishVelocity(0.0, 0.0, 0.35);
                ROS_INFO_THROTTLE(2, "[DRONE1_SCAN] Searching for authorized person...");

                if (g_person_detected) {
                    ROS_INFO("[DRONE1_SCAN] Target locked. ID: %d", g_last_bbox.id);
                    current_state = State::FOLLOW;
                }
                break;

            case State::FOLLOW:
                double time_since_last_seen = (ros::Time::now() - g_last_seen_time).toSec();

                if (g_person_detected) {
                    drone.runFollowLogic(g_last_bbox, g_target_distance);
                }
                else if (time_since_last_seen < 10.0) {
                    ROS_WARN_THROTTLE(1.0, "[DRONE1_FOLLOW] Target lost, waiting... (%.1fs)", 10.0 - time_since_last_seen);
                    drone.publishVelocity(0.0, 0.0, 0.0);
                }
                else {
                    ROS_ERROR("[DRONE1_FOLLOW] Wait timeout. Returning to SCAN.");
                    g_person_detected = false;
                    current_state = State::SCAN;
                }
                break;
        }

        rate.sleep();
    }

    return 0;
}
