#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import math
from clover import srv
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# ================= НАСТРОЙКИ ПОЛИГОНА =================
GRID_SIZE_X = 5
GRID_SIZE_Y = 4
MARKER_STEP = 1.0

FLIGHT_HEIGHT = 1.2
SPEED = 0.5
SEARCH_DELAY = 2.0

# ===== НАСТРОЙКИ ДЕТЕКЦИИ =====
MIN_AREA = 1200

CAMERA_FX = 920.0
CAMERA_FY = 920.0
MIN_MARKER_DIST = 2.0
# =======================================================

COLOR_RANGES = {
    "red":    [(0, 120, 120), (10, 255, 255)],
    "blue":   [(105, 150, 80), (130, 255, 255)],
    "green":  [(45, 120, 80), (75, 255, 255)],
    "yellow": [(22, 150, 150), (32, 255, 255)],
    "orange": [(10, 150, 150), (20, 255, 255)]
}

DRAW_COLORS = {
    "red": (0, 0, 255),
    "blue": (255, 0, 0),
    "green": (0, 255, 0),
    "yellow": (0, 255, 255),
    "orange": (0, 165, 255)
}


class PolygonSearcher:

    def __init__(self):
        rospy.init_node('polygon_search')

        self.bridge = CvBridge()
        self.found_positions = []
        self.home_position = None

        self.get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
        self.navigate = rospy.ServiceProxy('navigate', srv.Navigate)
        self.land_srv = rospy.ServiceProxy('land', Trigger)

        self.image_pub = rospy.Publisher('/color_detection/debug', Image, queue_size=1)
        self.file = open("detections.txt", "w")

        rospy.Subscriber('/main_camera/image_raw', Image, self.image_callback)

    # ---------------------------------------------------
    def navigate_wait(self, x=0, y=0, z=0, yaw=float('nan'),
                      speed=0.5, frame_id='aruco_map', auto_arm=False, tolerance=0.2):

        self.navigate(x=x, y=y, z=z, yaw=yaw,
                      speed=speed, frame_id=frame_id, auto_arm=auto_arm)

        while not rospy.is_shutdown():
            telem = self.get_telemetry(frame_id='navigate_target')
            if math.sqrt(telem.x**2 + telem.y**2 + telem.z**2) < tolerance:
                break
            rospy.sleep(0.2)

    # ---------------------------------------------------

    def takeoff(self):
        rospy.loginfo("Taking off...")
        self.navigate_wait(z=FLIGHT_HEIGHT, frame_id='body', auto_arm=True)

        telem = self.get_telemetry(frame_id='aruco_map')
        self.home_position = (telem.x, telem.y)
        rospy.loginfo(f"Home position saved: {self.home_position}")

    def go_to(self, x, y, z):
        self.navigate_wait(x=x, y=y, z=z, frame_id='aruco_map', speed=SPEED)
        rospy.sleep(SEARCH_DELAY)

    def return_home(self):
        rospy.loginfo("Returning home...")
        x, y = self.home_position
        self.navigate_wait(x=x, y=y, z=FLIGHT_HEIGHT, frame_id='aruco_map')

    def round_to_half(self, value):
        return round(value * 2) / 2

    # ---------------------------------------------------
    def pixel_to_map(self, cx, cy, img_w, img_h, drone_x, drone_y, drone_z):
        dx = (cx - img_w / 2) * drone_z / CAMERA_FX
        dy = (cy - img_h / 2) * drone_z / CAMERA_FY
        return drone_x + dx, drone_y + dy

    def is_far_enough(self, x, y):
        for px, py in self.found_positions:
            if math.hypot(x - px, y - py) < MIN_MARKER_DIST:
                return False
        return True

    # ---------------------------------------------------

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h, w, _ = frame.shape

        telem = self.get_telemetry(frame_id='aruco_map')

        for color, (lower, upper) in COLOR_RANGES.items():
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))

            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < MIN_AREA:
                    continue

                perimeter = cv2.arcLength(cnt, True)
                if perimeter == 0:
                    continue

                circularity = 4 * math.pi * area / (perimeter * perimeter)
                shape = "circle" if circularity > 0.8 else "square"

                M = cv2.moments(cnt)
                if M["m00"] == 0:
                    continue

                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                obj_x, obj_y = self.pixel_to_map(cx, cy, w, h, telem.x, telem.y, telem.z)
                obj_x = self.round_to_half(obj_x)
                obj_y = self.round_to_half(obj_y)

                if self.is_far_enough(obj_x, obj_y):
                    self.found_positions.append((obj_x, obj_y))
                    self.file.write(f"{color} {shape} at ({obj_x}, {obj_y})\n")
                    self.file.flush()
                    rospy.loginfo(f"Found {color} {shape} at ({obj_x}, {obj_y})")

                draw_color = DRAW_COLORS[color]
                cv2.drawContours(frame, [cnt], -1, draw_color, 3)
                cv2.circle(frame, (cx, cy), 5, draw_color, -1)
                cv2.putText(frame, f"{color} {shape} ({obj_x}, {obj_y})",
                            (cx + 10, cy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, draw_color, 2)

        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))

    # ---------------------------------------------------

    def scan_polygon(self):
        rospy.loginfo("Starting scan...")
        for i in range(GRID_SIZE_X):
            y_range = range(GRID_SIZE_Y) if i % 2 == 0 else reversed(range(GRID_SIZE_Y))
            for j in y_range:
                self.go_to(i * MARKER_STEP, j * MARKER_STEP, FLIGHT_HEIGHT)

    def land(self):
        rospy.loginfo("Landing...")
        self.land_srv()
        self.file.close()


# ================= ЗАПУСК =================

if __name__ == "__main__":
    searcher = PolygonSearcher()
    rospy.sleep(2)

    searcher.takeoff()
    searcher.scan_polygon()
    searcher.return_home()
    searcher.land()
