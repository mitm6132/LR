import rospy
from clover.srv import SetLEDEffect
from clover import srv
from std_srvs.srv import Trigger
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import math

start_recognition = False
land_point_x = 1
land_point_y = 1

rospy.init_node('flight')
bridge = CvBridge()

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)
set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect, persistent=True)
image_pub = rospy.Publisher('/Romanov_Pavel_debug', Image, queue_size=1)

def navigate_wait(x=0, y=0, z=0, speed=0.5, frame_id='body', auto_arm=False):
    res = navigate(x=x, y=y, z=z, yaw=float('nan'), speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    if not res.success:
        raise Exception(res.message)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < 0.2:
            return
        rospy.sleep(0.2)

def land_wait():
    land()
    while get_telemetry().armed:
        rospy.sleep(0.2)

def draw_contour(img, moments, color):
    if start_recognition == True:

        if moments["m00"] > 0:
            cx = int(moments["m10"] / moments["m00"])
            cy = int(moments["m01"] / moments["m00"])

            size = 50
            cv2.rectangle(img, (cx - size, cy - size), (cx + size, cy + size), color, 2)

            if color == (255, 0, 0):
                cv2.putText(img, 'Blue object', (cx - 40, cy - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            elif color == (0, 0, 255):
                cv2.putText(img, 'Red object', (cx - 40, cy - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            elif color == (0, 255, 0):
                cv2.putText(img, 'Green object', (cx - 40, cy - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            elif color == (255, 255, 0):
                cv2.putText(img, 'Yellow object', (cx - 40, cy - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            elif color == (255, 127, 0):
                cv2.putText(img, 'Orange object', (cx - 40, cy - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

            return True
        return False

def color_detect(msg):
    if start_recognition == True:
        img = bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        mask_red1 = cv2.inRange(hsv, (0, 120, 70), (10, 255, 255))
        mask_red2 = cv2.inRange(hsv, (170, 120, 70), (180, 255, 255))
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        mask_blue = cv2.inRange(hsv, (100,150,50), (140,255,255))
        mask_green = cv2.inRange(hsv, (40, 100, 100), (80, 255, 255))
        mask_yellow = cv2.inRange(hsv, (60,132,255), (65, 255, 255))
        mask_orange= cv2.inRange(hsv, (40, 120, 220), (50, 255, 255))


        red = cv2.moments(mask_red)
        blue = cv2.moments(mask_blue)
        green = cv2.moments(mask_green)
        yellow = cv2.moments(mask_yellow)
        orange = cv2.moments(mask_orange)


        detected = False

        if draw_contour(img, red, (0, 0, 255)):
            print('object 1: red', ('square'))
            set_effect(effect='fill', r=255, g=0, b=0)
            detected = True
        elif draw_contour(img, blue, (255, 0, 0)):
            print('object 2: blue', ('square'))
            set_effect(effect='fill', r=0, g=0, b=255)
            detected = True
        elif draw_contour(img, green, (0, 255, 0)):
            print('object 3 or 4: green', ('square or circle')) 
            set_effect(effect='fill', r=0, g=255, b=0)
            detected = True
        elif draw_contour(img, yellow, (255, 255, 0)):
            print('object 5: yellow', ('circle')) 
            set_effect(effect='fill', r=255, g=255, b=0)
            detected = True
        elif draw_contour(img, orange, (255, 127, 0)):
            print('object 6: orange', ('circle')) 
            set_effect(effect='fill', r=255, g=127, b=0)
            detected = True

        image_pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))


image_sub = rospy.Subscriber('main_camera/image_raw', Image, color_detect, queue_size=1)

print('Takeoff')
set_effect(effect='fill', r=102, g=0, b=204)
navigate_wait(z=1, frame_id='body', auto_arm=True)

rospy.sleep(0.5)
print('Fly to start point')
set_effect(effect='fill', r=51, g=255, b=255)
navigate_wait(x=0, y=0, z=1, frame_id='aruco_map', speed=0.5)

rospy.sleep(0.5)
print("I'm going down")
navigate_wait(x=0, y=0, z=0.5, frame_id='aruco_map', speed=0.5)

rospy.sleep(0.5)
print('Start Recognition')
start_recognition = True

navigate_wait(x=7, y=0, z=0.5, frame_id='aruco_map', speed=0.5)
rospy.sleep(0.5)
navigate_wait(x=7, y=1, z=0.5, frame_id='aruco_map', speed=0.5)
rospy.sleep(0.5)
navigate_wait(x=0, y=1, z=0.5, frame_id='aruco_map', speed=0.5)
rospy.sleep(0.5)
navigate_wait(x=0, y=2, z=0.5, frame_id='aruco_map', speed=0.5)
rospy.sleep(0.5)
navigate_wait(x=7, y=2, z=0.5, frame_id='aruco_map', speed=0.5)
rospy.sleep(0.5)
navigate_wait(x=7, y=3, z=0.5, frame_id='aruco_map', speed=0.5)
rospy.sleep(0.5)
navigate_wait(x=0, y=3, z=0.5, frame_id='aruco_map', speed=0.5)
rospy.sleep(0.5)
navigate_wait(x=0, y=4, z=0.5, frame_id='aruco_map', speed=0.5)
rospy.sleep(0.5)
navigate_wait(x=7, y=4, z=0.5, frame_id='aruco_map', speed=0.5)
rospy.sleep(0.5)
navigate_wait(x=7, y=5, z=0.5, frame_id='aruco_map', speed=0.5)
rospy.sleep(0.5)
navigate_wait(x=0, y=5, z=0.5, frame_id='aruco_map', speed=0.5)
rospy.sleep(0.5)
navigate_wait(x=0, y=6, z=0.5, frame_id='aruco_map', speed=0.5)
rospy.sleep(0.5)
navigate_wait(x=7, y=6, z=0.5, frame_id='aruco_map', speed=0.5)
rospy.sleep(0.5)
navigate_wait(x=7, y=7, z=0.5, frame_id='aruco_map', speed=0.5)
rospy.sleep(0.5)
navigate_wait(x=0, y=7, z=0.5, frame_id='aruco_map', speed=0.5)

start_recognition = False
print('Stop Recognition')
set_effect(effect='fill', r=255, g=255, b=255)
rospy.sleep(0.5)

print('Fly to land point')
navigate_wait(x=land_point_x, y=land_point_y, z=0.5, frame_id='aruco_map', speed=0.5)

rospy.sleep(0.5)

print('Landing')
land_wait()

rospy.spin()