#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

rospy.init_node('shape_detector_test')

bridge = CvBridge()
image_pub = rospy.Publisher('/shape_detection/debug', Image, queue_size=1)

# Минимальная площадь объекта (фильтр шума)
MIN_AREA = 1200

# HSV диапазоны цветов
COLOR_RANGES = {
    "red":    [(0, 120, 120), (10, 255, 255)],
    "blue":   [(90, 200, 130), (120, 255, 190)],
    "green":  [(45, 120, 80), (75, 255, 255)],
    "yellow": [(21, 120, 120), (45, 255, 255)],
    "orange": [(10, 150, 150), (20, 255, 255)]
}

DRAW_COLORS = {
    "red": (0, 0, 255),
    "blue": (255, 0, 0),
    "green": (0, 255, 0),
    "yellow": (0, 255, 255),
    "orange": (0, 165, 255)
}


def image_callback(msg):
    frame = bridge.imgmsg_to_cv2(msg, 'bgr8')
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    for color, (lower, upper) in COLOR_RANGES.items():
        mask = cv2.inRange(hsv, np.array(lower), np.array(upper))

        # Убираем шум
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

            # Определяем форму через округлость
            circularity = 4 * math.pi * area / (perimeter * perimeter)
            shape = "circle" if circularity > 0.8 else "square"

            M = cv2.moments(cnt)
            if M["m00"] == 0:
                continue

            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            draw_color = DRAW_COLORS[color]

            cv2.drawContours(frame, [cnt], -1, draw_color, 3)
            cv2.circle(frame, (cx, cy), 5, draw_color, -1)
            cv2.putText(frame, f"{color} {shape}",
                        (cx + 10, cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, draw_color, 2)

    image_pub.publish(bridge.cv2_to_imgmsg(frame, 'bgr8'))


rospy.Subscriber('/main_camera/image_raw', Image, image_callback)

rospy.loginfo("Shape detector test node started")
rospy.spin()