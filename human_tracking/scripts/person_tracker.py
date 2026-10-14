#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import rospkg
import numpy as np
from human_tracking.msg import PersonBBox
from sort import Sort

class PersonTrackerNode:
    def __init__(self):
        rospy.init_node('person_tracker', anonymous=True)
        self.bridge = CvBridge()

        self.pub = rospy.Publisher("/drone1/person_bbox", PersonBBox, queue_size=10)

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path("human_tracking")

        self.tracker = Sort(max_age=120, min_hits=1, iou_threshold=0.05)

        PROTOTXT = pkg_path + "/scripts/models/MobileNetSSD_deploy.prototxt.txt"
        MODEL    = pkg_path + "/scripts/models/MobileNetSSD_deploy.caffemodel"

        rospy.loginfo("Loading MobileNet-SSD model...")
        self.net = cv2.dnn.readNetFromCaffe(PROTOTXT, MODEL)

        self.image_sub = rospy.Subscriber(
            "/drone1/webcam/image_raw",
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

        self.LOW_RED_1  = np.array([0,   70, 50])
        self.HIGH_RED_1 = np.array([10, 255, 255])
        self.LOW_RED_2  = np.array([170,  70, 50])
        self.HIGH_RED_2 = np.array([180, 255, 255])

        self.id_memory = {}
        self.id_remap  = {}
        self.MAX_MEMORY_AGE = 90
        self.REMAP_DIST_PX  = 150

        self.BLUR_THRESHOLD = 80.0
        self.CONF_NORMAL    = 0.50
        self.CONF_BLURRY    = 0.30

    def _remap_id(self, raw_id, bbox):
        if raw_id in self.id_remap:
            return self.id_remap[raw_id]

        cx = (bbox[0] + bbox[2]) / 2.0
        cy = (bbox[1] + bbox[3]) / 2.0

        best_old_id = None
        best_dist   = self.REMAP_DIST_PX

        for old_id, mem in self.id_memory.items():
            if old_id in self.id_remap.values():
                continue
            ox, oy = mem['center']
            dist = ((cx - ox) ** 2 + (cy - oy) ** 2) ** 0.5
            if dist < best_dist:
                best_dist   = dist
                best_old_id = old_id

        if best_old_id is not None:
            self.id_remap[raw_id] = best_old_id
            rospy.logdebug(f"ID remap: {raw_id} -> {best_old_id} (dist={best_dist:.1f}px)")
            return best_old_id

        return raw_id

    def _is_blurry(self, gray_frame):
        return cv2.Laplacian(gray_frame, cv2.CV_64F).var() < self.BLUR_THRESHOLD

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        (h, w) = frame.shape[:2]

        gray   = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurry = self._is_blurry(gray)

        if blurry:
            proc_frame = cv2.GaussianBlur(frame, (5, 5), 0)
            conf_thresh = self.CONF_BLURRY
            rospy.logdebug("Blurry frame detected -- using lower confidence threshold")
        else:
            proc_frame  = frame
            conf_thresh = self.CONF_NORMAL

        blob = cv2.dnn.blobFromImage(
            cv2.resize(proc_frame, (300, 300)),
            0.007843,
            (300, 300),
            127.5
        )
        self.net.setInput(blob)
        detections = self.net.forward()

        detections_for_sort = []

        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            if confidence < conf_thresh:
                continue
            idx = int(detections[0, 0, i, 1])
            if self.CLASSES[idx] != "person":
                continue
            box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
            startX, startY, endX, endY = box.astype("int")
            detections_for_sort.append([startX, startY, endX, endY, confidence])

        if len(detections_for_sort) > 0:
            tracked_objects = self.tracker.update(np.array(detections_for_sort))
        else:
            tracked_objects = self.tracker.update(np.empty((0, 5)))

        active_ids = set()
        for t in tracked_objects:
            _, _, _, _, rid = t.astype(int)
            active_ids.add(int(rid))

        for tid in list(self.id_memory.keys()):
            if tid not in active_ids:
                self.id_memory[tid]['age'] += 1
                if self.id_memory[tid]['age'] > self.MAX_MEMORY_AGE:
                    del self.id_memory[tid]
                    self.id_remap = {k: v for k, v in self.id_remap.items() if v != tid}

        best_candidate_msg = None
        max_red_ratio = -1.0

        for t in tracked_objects:
            startX, startY, endX, endY, raw_id = t.astype(int)
            raw_id = int(raw_id)

            startX = max(0, startX); startY = max(0, startY)
            endX   = min(w, endX);   endY   = min(h, endY)

            track_id = self._remap_id(raw_id, (startX, startY, endX, endY))

            self.id_memory[track_id] = {
                'center': ((startX + endX) / 2.0, (startY + endY) / 2.0),
                'age': 0
            }

            roi_y1 = startY + int(0.25 * (endY - startY))
            roi_y2 = startY + int(0.55 * (endY - startY))
            roi_x1 = startX + int(0.25 * (endX - startX))
            roi_x2 = startX + int(0.75 * (endX - startX))

            roi = frame[roi_y1:roi_y2, roi_x1:roi_x2]

            is_authorized     = False
            current_red_ratio = 0.0

            if roi.size > 0:
                hsv    = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                mask1  = cv2.inRange(hsv, self.LOW_RED_1, self.HIGH_RED_1)
                mask2  = cv2.inRange(hsv, self.LOW_RED_2, self.HIGH_RED_2)
                red_mask = cv2.bitwise_or(mask1, mask2)
                current_red_ratio = cv2.countNonZero(red_mask) / float(roi.shape[0] * roi.shape[1])
                is_authorized     = current_red_ratio > 0.12

                self.id_memory[track_id]['red'] = current_red_ratio

            color = (0, 255, 0) if is_authorized else (0, 0, 255)

            thickness = 1 if blurry else 2
            cv2.rectangle(frame, (startX, startY), (endX, endY), color, thickness)

            auth_text = "AUTH" if is_authorized else "NO AUTH"
            label = f"ID:{track_id} {auth_text}"
            if blurry:
                label += " ~"
            cv2.putText(frame, label,
                        (startX, startY - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

            if is_authorized and current_red_ratio > max_red_ratio:
                max_red_ratio = current_red_ratio

                msg_out            = PersonBBox()
                msg_out.id         = track_id
                msg_out.authorized = True
                msg_out.cx         = int((startX + endX) / 2)
                msg_out.cy         = int(startY + (endY - startY) * 0.35)
                msg_out.img_width  = w
                msg_out.img_height = h
                msg_out.confidence = float(current_red_ratio)
                best_candidate_msg = msg_out

        if best_candidate_msg is not None:
            self.pub.publish(best_candidate_msg)
            cv2.circle(frame, (best_candidate_msg.cx, best_candidate_msg.cy),
                       8, (255, 0, 0), -1)

        cv2.imshow("Person Tracking", frame)
        cv2.waitKey(1)

if __name__ == "__main__":
    try:
        PersonTrackerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
