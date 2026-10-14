#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import json
import rospkg

class GeneralPersonDetectorNode:
    def __init__(self):
        rospy.init_node('person_detector_node', anonymous=True)

        self.blob_pub = rospy.Publisher('/person_bbox', String, queue_size=10)

        self.image_sub = rospy.Subscriber('/drone2/usb_camera/image_raw', Image, self.image_callback)

        self.bridge = CvBridge()
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path("area_scanning")

        PROTOTXT = pkg_path + "/scripts/models/MobileNetSSD_deploy.prototxt.txt"
        MODEL    = pkg_path + "/scripts/models/MobileNetSSD_deploy.caffemodel"

        rospy.loginfo("Loading MobileNet-SSD model...")
        self.net = cv2.dnn.readNetFromCaffe(PROTOTXT, MODEL)

        self.image_sub = rospy.Subscriber(
            "/drone2/webcam/image_raw",
            Image,
            self.image_callback,
            queue_size=1
        )

        self.CLASSES = [
            "background", "aeroplane", "bicycle", "bird", "boat",
            "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
            "dog", "horse", "motorbike", "person", "pottedplant",
            "sheep", "sofa", "train", "tvmonitor"
        ]


    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"[VISION] CvBridge error: {e}")
            return

        (h, w) = cv_image.shape[:2]
        blob = cv2.dnn.blobFromImage(cv2.resize(cv_image, (300, 300)), 0.007843, (300, 300), 127.5)
        self.net.setInput(blob)
        detections = self.net.forward()

        person_found = False
        cx_err = 0.0
        cy_err = 0.0

        for i in range(0, detections.shape[2]):
            confidence = detections[0, 0, i, 2]

            if confidence > 0.5:
                class_id = int(detections[0, 0, i, 1])

                if self.CLASSES[class_id] == "person":
                    person_found = True

                    box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                    (startX, startY, endX, endY) = box.astype("int")

                    person_cx = (startX + endX) / 2.0
                    person_cy = (startY + endY) / 2.0

                    cx_err = person_cx - (w / 2.0)
                    cy_err = person_cy - (h / 2.0)

                    cv2.rectangle(cv_image, (startX, startY), (endX, endY), (255, 0, 0), 2)
                    label = f"Person: {confidence * 100:.1f}%"
                    cv2.putText(cv_image, label, (startX, startY - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                    break

        msg_data = {
            "detected": person_found,
            "cx_err": float(cx_err),
            "cy_err": float(cy_err)
        }

        self.blob_pub.publish(json.dumps(msg_data))

        cv2.imshow("Drone Camera", cv_image)
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        node = GeneralPersonDetectorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
