import math
import rospy
from clover import srv
from mavros_msgs.srv import CommandBool
from std_srvs.srv import Trigger

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land_srv = rospy.ServiceProxy('land', Trigger)
arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)


def navigate_wait(x=0, y=0, z=0, yaw=math.nan, speed=0.5, frame_id='body', tolerance=0.2, auto_arm=False):
    res = navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)
    if not res.success:
        rospy.logwarn(f"Navigate failed: {res.message}")
        return res
    while not rospy.is_shutdown():
        try:
            telem = get_telemetry(frame_id='navigate_target')
            dist = math.sqrt(telem.x**2 + telem.y**2 + telem.z**2)
            if dist < tolerance:
                return res
        except rospy.ServiceException:
            rospy.logwarn_throttle(2, "Telemetry unavailable, retrying...")
        rospy.sleep(0.2)

def wait_for_aruco(id, timeout=60):
    rospy.loginfo(f"Waiting for ArUco {id} to appear...")
    start = rospy.Time.now()
    while (rospy.Time.now() - start).to_sec() < timeout and not rospy.is_shutdown():
        try:
            get_telemetry(frame_id=f'aruco_{id}')
            rospy.loginfo(f"ArUco {id} detected!")
            return True
        except rospy.ServiceException:
            rospy.sleep(0.2)
    rospy.logwarn(f"ArUco {id} not detected within timeout!")
   rospy.logwarn(f"ArUco {id} not detected within timeout!")
    return False

# ---------------- ПОЛЁТ ----------------

rospy.loginfo("Takeoff 1 m")
navigate_wait(z=1, frame_id='body', auto_arm=True)

rospy.loginfo("Fly forward 1 m")
navigate_wait(x=2, z=1, frame_id='aruco_map')
navigate_wait(y=1, z=1, frame_id='aruco_map')

# Ждём появления маркера ID 100
if not wait_for_aruco(100, timeout=120):
    rospy.logerr("Marker not found! Landing aborted.")
    land_srv()
    exit()

# Плавное снижение по маркеру
Z_LEVELS = [1.0, 0.8, 0.6, 0.4]  # можно добавить больше уровней для плавности
for z in Z_LEVELS:
    navigate_wait(x=0, y=0, z=z, frame_id='aruco_100', speed=0.2)

# Финальная посадка — отключаем моторы
arming(False)
rospy.loginfo("Landed successfully on moving platform!")