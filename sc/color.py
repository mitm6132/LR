Хьюстон, у нас проблема, [07.02.2026 18:09]
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import math
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from clover import srv
from std_srvs.srv import Trigger

# ================= НАСТРОЙКИ =================
FLIGHT_HEIGHT = 1.2
GRID_SIZE = 3
CELL_SIZE = 0.6
MIN_MARKER_DIST = 0.5
ROUND_STEP = 0.5

COLOR_RANGES = {
    "yellow": [(21, 120, 120), (45, 255, 255)],
    "blue":   [(90, 200, 130), (120, 255, 190)],
}
# ============================================

rospy.init_node('color_polygon_search')
bridge = CvBridge()

rospy.loginfo("Waiting for Clover services...")
rospy.wait_for_service('get_telemetry')
rospy.wait_for_service('navigate')
rospy.wait_for_service('land')
rospy.loginfo("Services connected")

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land_srv = rospy.ServiceProxy('land', Trigger)

image_pub = rospy.Publisher('/color_debug', Image, queue_size=1)

found_markers = []
search_started = False
home_x = 0
home_y = 0

# ---------- ВСПОМОГАТЕЛЬНЫЕ ----------
def round_to_step(value):
    if math.isnan(value):
        return None
    return round(value / ROUND_STEP) * ROUND_STEP

def far_enough(x, y):
    for fx, fy, _, _ in found_markers:
        if math.hypot(fx - x, fy - y) < MIN_MARKER_DIST:
            return False
    return True

def navigate_wait(x=0, y=0, z=0, frame_id='aruco_map', speed=0.5, tolerance=0.15, auto_arm=False):
    try:
        navigate(x=x, y=y, z=z, frame_id=frame_id, speed=speed, auto_arm=auto_arm)
    except rospy.ServiceException as e:
        rospy.logerr(f"Navigate failed: {e}")
        return

    while not rospy.is_shutdown():
        try:
            telem = get_telemetry(frame_id='navigate_target')
            dist = math.sqrt(telem.x**2 + telem.y**2 + telem.z**2)
            if dist < tolerance:
                break
        except:
            rospy.logwarn_throttle(2, "Telemetry lost...")
        rospy.sleep(0.2)

# ---------- ДЕТЕКТОР ----------
class Detector:

    def init(self):
        rospy.Subscriber('/main_camera/image_raw', Image, self.image_callback)

    def image_callback(self, msg):
        frame = bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        try:
            telem = get_telemetry(frame_id='aruco_map')
            drone_x = telem.x
            drone_y = telem.y
        except:
            return

        for color_name, (low, high) in COLOR_RANGES.items():
            mask = cv2.inRange(hsv, np.array(low), np.array(high))
            mask = cv2.medianBlur(mask, 5)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < 500:
                    continue

                x, y, w, h = cv2.boundingRect(cnt)
                obj_x = round_to_step(drone_x)
                obj_y = round_to_step(drone_y)

                if obj_x is None or obj_y is None:
                    continue

                if not far_enough(obj_x, obj_y):
                    continue

                approx = cv2.approxPolyDP(cnt, 0.04 * cv2.arcLength(cnt, True), True)
                shape = "square" if len(approx) == 4 else "circle"

                found_markers.append((obj_x, obj_y, color_name, shape))
                rospy.loginfo(f"FOUND {color_name} {shape} at ({obj_x}, {obj_y})")

                cv2.drawContours(frame, [cnt], -1, (0,255,0), 3)
                cv2.putText(frame, f"{color_name} {obj_x},{obj_y}",
                            (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

        image_pub.publish(bridge.cv2_to_imgmsg(frame, 'bgr8'))

detector = Detector()

# ---------- ЖДЁМ КОМАНДУ ----------
rospy.loginfo("Detection running. Press ENTER to start flight...")
input()

# ---------- ВЗЛЁТ ----------
rospy.loginfo("Taking off")
navigate_wait(z=FLIGHT_HEIGHT, frame_id='body', auto_arm=True)

Хьюстон, у нас проблема, [07.02.2026 18:09]
home = get_telemetry(frame_id='aruco_map')
home_x, home_y = home.x, home.y

rospy.sleep(2)
rospy.loginfo("Start scanning")

# ---------- ОБЛЁТ ПОЛИГОНА ----------
for i in range(GRID_SIZE):
    for j in range(GRID_SIZE):
        x = i * CELL_SIZE
        y = j * CELL_SIZE
        navigate_wait(x=x, y=y, z=FLIGHT_HEIGHT)

# ---------- ВОЗВРАТ ----------
rospy.loginfo("Returning home")
navigate_wait(x=home_x, y=home_y, z=FLIGHT_HEIGHT)

rospy.loginfo("Landing")
land_srv()

# ---------- СОХРАНЕНИЕ ----------
with open('found_markers.txt', 'w') as f:
    for x, y, color, shape in found_markers:
        f.write(f"{color} {shape} {x} {y}\n")

rospy.loginfo("Saved to found_markers.txt")