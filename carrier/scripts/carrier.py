#!/usr/bin/env python3
import rospy
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from mavros_msgs.msg import State
from mavros_msgs.msg import GlobalPositionTarget


current_state = None

def state_cb(msg):
    global current_state
    current_state = msg

def goto_position_and_wait(latitude, longitude, altitude, wait_seconds=45, label=""):
    pub = rospy.Publisher('/mavros/setpoint_raw/global', GlobalPositionTarget, queue_size=10)
    rate = rospy.Rate(10)

    goal = GlobalPositionTarget()
    goal.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
    goal.type_mask = (
        GlobalPositionTarget.IGNORE_VX |
        GlobalPositionTarget.IGNORE_VY |
        GlobalPositionTarget.IGNORE_VZ |
        GlobalPositionTarget.IGNORE_AFX |
        GlobalPositionTarget.IGNORE_AFY |
        GlobalPositionTarget.IGNORE_AFZ |
        GlobalPositionTarget.IGNORE_YAW |
        GlobalPositionTarget.IGNORE_YAW_RATE
    )
    goal.latitude  = latitude
    goal.longitude = longitude
    goal.altitude  = altitude

    rospy.loginfo(f"[{label}] Going to target: lat={latitude}, lon={longitude}, alt={altitude}")

    for _ in range(200):
        goal.header.stamp = rospy.Time.now()
        pub.publish(goal)
        rate.sleep()

    rospy.loginfo(f"[{label}] Target reached (assumed). Waiting {wait_seconds}s...")

    wait_iterations = int(wait_seconds * 10)
    for _ in range(wait_iterations):
        goal.header.stamp = rospy.Time.now()
        pub.publish(goal)
        rate.sleep()

    rospy.loginfo(f"[{label}] Wait complete.")


def return_to_launch():
    rospy.wait_for_service('mavros/set_mode')
    try:
        set_mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)
        resp = set_mode_srv(custom_mode="RTL")
        if resp.mode_sent:
            rospy.loginfo("RTL mode active: Drone returning to launch point")
        else:
            rospy.logwarn("Failed to set RTL mode")
    except rospy.ServiceException as e:
        rospy.logerr("Service error: %s" % e)


def main():
    global current_state

    rospy.init_node('guided_arm_takeoff_node')
    rospy.Subscriber('mavros/state', State, state_cb)

    rospy.wait_for_service('mavros/cmd/arming')
    rospy.wait_for_service('mavros/set_mode')
    rospy.wait_for_service('/mavros/cmd/takeoff')

    arm_srv      = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
    set_mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)
    takeoff_srv  = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown() and (current_state is None or not current_state.connected):
        rospy.loginfo("Waiting for FCU connection...")
        rate.sleep()

    rospy.loginfo("FCU connected")

    last_request = rospy.Time.now()
    while not rospy.is_shutdown():
        if current_state.mode != "GUIDED":
            if (rospy.Time.now() - last_request) > rospy.Duration(5.0):
                resp = set_mode_srv(custom_mode="GUIDED")
                if resp.mode_sent:
                    rospy.loginfo("GUIDED mode active")
                last_request = rospy.Time.now()
        else:
            break
        rate.sleep()

    while not rospy.is_shutdown() and not current_state.armed:
        if (rospy.Time.now() - last_request) > rospy.Duration(5.0):
            arm_srv(True)
            rospy.loginfo("Arming...")
            last_request = rospy.Time.now()
        rate.sleep()

    rospy.loginfo("Drone armed!")

    altitude = 3.0
    try:
        resp = takeoff_srv(altitude=altitude, latitude=0, longitude=0, min_pitch=0, yaw=0)
        if resp.success:
            rospy.loginfo("Takeoff started!")
        else:
            rospy.logwarn("Takeoff command failed!")
    except rospy.ServiceException as e:
        rospy.logerr("Takeoff service error: %s" % e)

    rospy.sleep(10)

    waypoints = [
        {"lat": -35.3632421, "lon": 149.1651000, "alt": 3.0, "label": "Point 1"},
        {"lat": -35.3635000, "lon": 149.1655000, "alt": 3.0, "label": "Point 2"},
        {"lat": -35.3638000, "lon": 149.1650000, "alt": 3.0, "label": "Point 3"},
        {"lat": -35.3640000, "lon": 149.1645000, "alt": 3.0, "label": "Point 4"},
    ]

    for wp in waypoints:
        goto_position_and_wait(
            latitude=wp["lat"],
            longitude=wp["lon"],
            altitude=wp["alt"],
            wait_seconds=45,
            label=wp["label"]
        )

    rospy.loginfo("All points complete. Starting RTL.")
    return_to_launch()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
