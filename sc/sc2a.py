import rospy
from clover.srv import SetLEDEffect
from clover import srv
from std_srvs.srv import Trigger
import csv
import math
import time

rospy.init_node('csv_animation_player')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)
set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect, persistent=True)

def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='body', auto_arm=False):
    res = navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)
    if not res.success:
        raise Exception(res.message)
    
    # Не ждем полного достижения цели, так как у нас непрерывная анимация
    # Ждем немного, чтобы дрон начал движение
    rospy.sleep(0.05)

def read_csv_animation(file_path):
    """Чтение анимации из CSV файла"""
    frames = []
    with open(file_path, 'r') as file:
        reader = csv.reader(file)
        next(reader)  # Пропускаем заголовок
        
        for row in reader:
            if len(row) >= 8:
                try:
                    frame = {
                        'frame': int(row[0]),
                        'x': float(row[1]),      # X координата
                        'y': float(row[2]),      # Y координата
                        'z': float(row[3]),      # Z координата (высота)
                        'yaw': float(row[4]),    # Рыскание
                        'mode': int(row[5]),     # Режим цвета
                        'r': int(row[6]),        # Красная компонента
                        'g': int(row[7]),        # Зеленая компонента
                        'b': 0                   # Синяя компонента (будем вычислять)
                    }
                    
                    # Определяем синий компонент на основе mode
                    if frame['mode'] == 0 and frame['r'] == 0 and frame['g'] == 0:
                        frame['b'] = 255  # Чистый синий
                    elif frame['mode'] == 255:
                        frame['b'] = 0    # Без синего (красный/зеленый)
                    elif frame['mode'] == 50:
                        frame['b'] = 100  # Смешанный
                    elif frame['mode'] == 49:
                        frame['b'] = 150  # Другой оттенок
                    elif frame['mode'] == 84:
                        frame['b'] = 200  # Еще один оттенок
                    else:
                        frame['b'] = 50   # По умолчанию
                    
                    # Корректируем yaw (если -π, то оставляем текущую ориентацию)
                    if abs(frame['yaw'] + 3.14159) < 0.001:
                        frame['yaw'] = float('nan')
                    
                    frames.append(frame)
                except (ValueError, IndexError):
                    continue
    
    return frames

def play_animation(frames, frame_rate=10.0, scale=1.0, height_offset=0.0):
    """Воспроизведение анимации с заданной частотой кадров"""
    frame_delay = 1.0 / frame_rate
    total_frames = len(frames)
    
    print(f"Starting animation: {total_frames} frames")
    print(f"Frame rate: {frame_rate} Hz")
    print(f"Total time: {total_frames / frame_rate:.1f} seconds")
    
    start_time = time.time()
    
    for i, frame in enumerate(frames):
        # Пропускаем первые несколько кадров для стабилизации
        if i < 10:
            continue
            
        # Вычисляем время для этого кадра
        expected_time = start_time + (i * frame_delay)
        current_time = time.time()
        
        # Если мы отстаем, пропускаем кадры
        if current_time > expected_time + 0.2:
            continue
            
        # Ожидаем следующего кадра
        sleep_time = max(0, expected_time - current_time)
        rospy.sleep(sleep_time)
        
        # Управляем светодиодами
        set_effect(effect='fill', r=frame['r'], g=frame['g'], b=frame['b'])
        
        # Управляем дроном (масштабируем координаты)
        x = frame['x'] * scale
        y = frame['y'] * scale
        z = frame['z'] * scale + height_offset
        
        # Отправляем команду навигации
        try:
            if i == 10:  # Первая команда после пропуска
                navigate(x=x, y=y, z=z, yaw=frame['yaw'], speed=0.5, 
                        frame_id='aruco_map', auto_arm=True)
            else:
                navigate(x=x, y=y, z=z, yaw=frame['yaw'], speed=0.5, 
                        frame_id='aruco_map')
        except Exception as e:
            print(f"Error at frame {frame['frame']}: {e}")
        
        # Прогресс
        if i % 50 == 0:
            progress = (i / total_frames) * 100
            print(f"Progress: {progress:.1f}% ({i}/{total_frames})")

def main():
    # Параметры анимации
    CSV_FILE = 'clover-blue.csv'
    FRAME_RATE = 10.0  # 10 кадров в секунду (как в файле)
    SCALE = 1.0        # Масштаб координат (уменьшите, если полигон маленький)
    HEIGHT_OFFSET = 1.0  # Базовая высота (добавляется к Z из файла)
    
    print("=== Clover CSV Animation Player ===")
    print("Reading animation from CSV...")
    
    # Чтение анимации
    frames = read_csv_animation(CSV_FILE)
    
    if not frames:
        print("ERROR: No frames loaded from CSV!")
        return
    
    print(f"Loaded {len(frames)} animation frames")
    
    # Анализируем диапазон координат
    x_vals = [f['x'] for f in frames]
    y_vals = [f['y'] for f in frames]
    z_vals = [f['z'] for f in frames]
    
    print(f"X range: {min(x_vals):.2f} to {max(x_vals):.2f}")
    print(f"Y range: {min(y_vals):.2f} to {max(y_vals):.2f}")
    print(f"Z range: {min(z_vals):.2f} to {max(z_vals):.2f}")
    
    # Проверяем, поместится ли анимация в полигон
    max_x = max(x_vals) * SCALE
    max_y = max(y_vals) * SCALE
    
    if max_x > 3.0 or max_y > 3.0:
        print(f"WARNING: Animation may exceed 3x3m area (X: {max_x:.1f}m, Y: {max_y:.1f}m)")
        print("Consider reducing SCALE parameter")
    
    # Подтверждение запуска
    print("\nAnimation parameters:")
    print(f"  Frame rate: {FRAME_RATE} Hz")
    print(f"  Scale: {SCALE}")
    print(f"  Height offset: {HEIGHT_OFFSET}m")
    print(f"  Estimated duration: {len(frames) / FRAME_RATE:.1f}s")
    
    input("\nPress Enter to start animation (or Ctrl+C to cancel)...")
    
    try:
        # Запускаем анимацию
        play_animation(frames, FRAME_RATE, SCALE, HEIGHT_OFFSET)
        
        # После завершения анимации
        print("\nAnimation completed!")
        print("Returning to start position...")
        
        # Возвращаемся в начало
        set_effect(effect='rainbow')
        navigate(x=0, y=0, z=HEIGHT_OFFSET, yaw=float('nan'), speed=0.5, 
                frame_id='aruco_map')
        rospy.sleep(3)
        
        # Посадка
        print("Landing...")
        set_effect(effect='fill', r=255, g=0, b=0)
        land()
        rospy.sleep(2)
        
        print("Mission complete!")
        
    except KeyboardInterrupt:
        print("\nAnimation interrupted!")
        # Аварийная посадка
        set_effect(effect='flash', r=255, g=255, b=0)
        land()
        rospy.sleep(2)
        
    except Exception as e:
        print(f"\nERROR: {e}")
        set_effect(effect='flash', r=255, g=0, b=0)
        land()
        rospy.sleep(2)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass