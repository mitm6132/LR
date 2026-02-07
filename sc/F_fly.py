from clover import srv
from std_srvs.srv import Trigger
import rospy
import math
from clover.srv import SetLEDEffect

Aruco_ID = None
Speed_Takeoff = None
Speed = None
Wait = None

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)
set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect, persistent=True)

def navigate_wait(x=0, y=0, z=0, speed=0.5, frame_id='body', auto_arm=False):
    res = navigate(x=x, y=y, z=z, yaw=float('nan'), speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    if not res.success:
        raise Exception(res.message)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < 0.2:
            return
        rospy.sleep(0.2)


Aruco_ID = 130
Speed_Takeoff = 0.8
Speed = 0.35
Wait = 0.3
navigate_wait(z=0.5, frame_id='body', auto_arm=True)
navigate_wait(x=0.5, y=0.5, z=0.5, frame_id='aruco_map', speed=Speed_Takeoff)
print('Takeoff')
navigate_wait(x=2, y=2, z=0.5, frame_id='aruco_map', speed=Speed_Takeoff)
print('Navigate to start point')
navigate_wait(x=2, y=2, z=1.85, frame_id='aruco_map', speed=Speed_Takeoff)
print('Seach Platform')
rospy.sleep(1)
set_effect(effect='fill', r=51, g=51, b=255)
navigate_wait(x=0, y=0, z=1, frame_id='aruco_' + str(int(Aruco_ID)), speed=Speed)
rospy.sleep(Wait)
print('Navigate to platform')
navigate_wait(x=0, y=0, z=0.5, frame_id='aruco_' + str(int(Aruco_ID)), speed=Speed)
rospy.sleep(Wait)
print('Lowering the altitude')
rospy.sleep(Wait)
navigate_wait(x=0, y=0, z=0.4, frame_id='aruco_' + str(int(Aruco_ID)), speed=Speed)
rospy.sleep(Wait)
navigate_wait(x=0, y=0, z=0.3, frame_id='aruco_' + str(int(Aruco_ID)), speed=Speed)
rospy.sleep(Wait)
navigate(x=0, y=0, z=0.25, frame_id='aruco_' + str(int(Aruco_ID)), speed=Speed, yaw=float('nan'))
rospy.sleep(Wait)
print('Landing')
land()
set_effect(effect='fill', r=51, g=204, b=0)