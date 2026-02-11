from clover import srv
from std_srvs.srv import Trigger
import rospy
import math
from clover.srv import SetLEDEffect

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

# ---------- ЦВЕТОВЫЕ ДИАПАЗОНЫ (такие же, как в детекторе) ----------
COLOR_RANGES = {
    "yellow": [(20, 100, 100), (35, 255, 255)],
    "orange": [(5, 100, 100), (18, 255, 255)],
    "blue":   [(90, 150, 100), (120, 255, 255)],
    "green":  [(40, 100, 100), (80, 255, 255)],
    "red1":   [(0, 100, 100), (10, 255, 255)],
    "red2":   [(160, 100, 100), (180, 255, 255)]
}
# --------------------------------------------------------------

bridge = CvBridge()

def detect_current_markers(timeout=1.0):
    result = []

    # Получаем одно изображение (блокируем, пока не придёт)
    try:
        img_msg = rospy.wait_for_message('/main_camera/image_raw', Image, timeout=timeout)
    except rospy.ROSException:
        rospy.logerr("Timeout: не получено изображение с камеры")
        return result

    frame = bridge.imgmsg_to_cv2(img_msg, 'bgr8')
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Обрабатываем красный (два диапазона)
    for color_name in ["red", "yellow", "orange", "blue", "green"]:
        if color_name == "red":
            ranges = [COLOR_RANGES["red1"], COLOR_RANGES["red2"]]
        else:
            ranges = [COLOR_RANGES[color_name]]

        # Создаём маску для этого цвета
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for low, high in ranges:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, np.array(low), np.array(high)))

        mask = cv2.medianBlur(mask, 5)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 500:
                continue

            # Определение формы
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)
            shape = "square" if len(approx) == 4 else "circle"

            result.append((color_name, shape))

    return result


# Где-то в основном коде, после инициализации ноды:
found = detect_current_markers(timeout=2.0)
for color, shape in found:
    print(f"Обнаружен: {color} {shape}")