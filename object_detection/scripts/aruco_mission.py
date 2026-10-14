#!/usr/bin/env python3

import rospy
import time
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, NavSatFix
from geometry_msgs.msg import Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL

class UAVMissionNode:
    def __init__(self):
        rospy.init_node('uav_mission_node', anonymous=True)

        self.current_state = State()
        self.current_gps = NavSatFix()
        self.target_aruco_id = 0
        self.target_detected = False
        self.aruco_x_offset = 0.0
        self.aruco_y_offset = 0.0

        self.bridge = CvBridge()

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        rospy.Subscriber('/mavros/state', State, self.state_cb)
        rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.gps_cb)

        rospy.Subscriber('/webcam/image_raw', Image, self.image_cb)

        self.velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
        self.target_gps_pub = rospy.Publisher('/uav/detected_target_global', NavSatFix, queue_size=10)

        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/set_mode')
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        self.rate = rospy.Rate(20)
        self.Kp = 0.005

    def state_cb(self, msg):
        self.current_state = msg

    def gps_cb(self, msg):
        self.current_gps = msg

    def image_cb(self, msg):
            try:
                frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                h, w, _ = frame.shape
                cam_cx, cam_cy = w // 2, h // 2

                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                corners, ids, rejected = self.detector.detectMarkers(gray)

                if ids is not None:
                    rospy.loginfo(f"Markers in frame. Detected IDs: {ids.flatten()}")
                else:
                    pass

                if ids is not None:
                    for i in range(len(ids)):
                        if ids[i][0] == self.target_aruco_id:
                            if not self.target_detected:
                                rospy.logwarn(f"!!! ASYNC CATCH !!! ArUco (ID: {self.target_aruco_id}) DETECTED during scan!")

                            self.target_detected = True

                            c = corners[i][0]
                            aruco_cx = int((c[0][0] + c[1][0] + c[2][0] + c[3][0]) / 4)
                            aruco_cy = int((c[0][1] + c[1][1] + c[2][1] + c[3][1]) / 4)

                            self.aruco_x_offset = float(aruco_cx - cam_cx)
                            self.aruco_y_offset = float(aruco_cy - cam_cy)

                            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                            cv2.circle(frame, (aruco_cx, aruco_cy), 5, (0, 0, 255), -1)
                            cv2.putText(frame, "LOCKED", (aruco_cx, aruco_cy - 10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                cv2.drawMarker(frame, (cam_cx, cam_cy), (255, 0, 0), cv2.MARKER_CROSS, 20, 2)

                cv2.imshow("Drone Camera View (ArUco Tracking)", frame)
                cv2.waitKey(1)

            except Exception as e:
                rospy.logerr(f"Image processing error: {e}")

    def send_velocity(self, vx, vy, duration):
        vel_msg = Twist()
        start_time = rospy.Time.now()

        while (rospy.Time.now() - start_time).to_sec() < duration:
            if self.target_detected:
                return False

            vel_msg.linear.x = vx
            vel_msg.linear.y = vy
            self.velocity_pub.publish(vel_msg)
            self.rate.sleep()

        stop_msg = Twist()
        for _ in range(5):
            if self.target_detected:
                return False
            self.velocity_pub.publish(stop_msg)
            self.rate.sleep()

        return True

    def execute_mission(self):
        rospy.loginfo("Waiting for EKF and GPS lock...")
        while not rospy.is_shutdown() and not self.current_state.connected:
            self.rate.sleep()

        rospy.loginfo("Setting mode to GUIDED...")
        self.set_mode_client(custom_mode="GUIDED")

        rospy.loginfo("Sending ARM command...")
        while not rospy.is_shutdown() and not self.current_state.armed:
            self.arming_client(True)
            time.sleep(1)

        rospy.loginfo("Taking off to 5 meters...")
        takeoff_client = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        takeoff_client(altitude=1.0)
        time.sleep(6)

        rospy.loginfo("Starting area scan steps...")

        steps = [
            {"name": "Right 1.5m",   "vx": 0.5,  "vy": 0.0, "t": 3.0},
            {"name": "Forward 1.5m", "vx": 0.0,  "vy": 0.5, "t": 3.0},
            {"name": "Left 3.0m",    "vx": -0.5, "vy": 0.0, "t": 6.0},
            {"name": "Forward 1.5m", "vx": 0.0,  "vy": 0.5, "t": 3.0},
            {"name": "Right 1.5m",   "vx": 0.5,  "vy": 0.0, "t": 3.0},
            {"name": "Forward 1.5m", "vx": 0.0,  "vy": 0.5, "t": 3.0}
        ]

        route_complete = True
        for step in steps:
            if self.target_detected:
                route_complete = False
                break

            rospy.loginfo(f"Route step: {step['name']}")
            if not self.send_velocity(step["vx"], step["vy"], step["t"]):
                rospy.logwarn("Target seen during scan -- route steps INTERRUPTED!")
                route_complete = False
                break

        if route_complete:
            rospy.loginfo("Route complete but no ArUco found.")

        rospy.loginfo("Switching to lock and centering mode...")

        while not rospy.is_shutdown():
            if self.target_detected:
                if abs(self.aruco_x_offset) > 20 or abs(self.aruco_y_offset) > 20:
                    vel_msg = Twist()

                    vel_msg.linear.x = self.aruco_x_offset * self.Kp
                    vel_msg.linear.y = -self.aruco_y_offset * self.Kp

                    vel_msg.linear.x = max(-0.35, min(0.35, vel_msg.linear.x))
                    vel_msg.linear.y = max(-0.35, min(0.35, vel_msg.linear.y))

                    self.velocity_pub.publish(vel_msg)
                else:
                    rospy.logerr("DIRECTLY OVER TARGET! PUBLISHING COORDINATES...")
                    self.target_gps_pub.publish(self.current_gps)

                    self.set_mode_client(custom_mode="RTL")
                    break
            else:
                vel_msg = Twist()
                self.velocity_pub.publish(vel_msg)

            self.rate.sleep()

        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        node = UAVMissionNode()
        node.execute_mission()
    except rospy.ROSInterruptException:
        pass
