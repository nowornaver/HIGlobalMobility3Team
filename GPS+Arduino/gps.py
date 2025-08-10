import sys
import math
import time
import serial
import csv
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import threading
import os
import collections   # 이동평균용
ser = serial.Serial(GPS_PORT, GPS_BAUD, timeout=0.1)

GPS_PORT = 'COM11'
GPS_BAUD = 115200
WAYPOINT_CSV = 'waypoints.csv'
OUTPUT_CSV = 'heading_vectors.csv'
PATH_SIZE = 1000
EARTH_RADIUS_METERS = 6378137.0
MIN_WAYPOINT_DISTANCE = 2.8
TARGET_RADIUS = 2.8
STEER_DIST_THRESHOLD = 0.1
GPS_JUMP_THRESHOLD = 0.5

# 저역필터 설정
f_c = 2.0
f_s = 10.0
alpha = (2 * math.pi * f_c) / (2 * math.pi * f_c + f_s)
filtered_steering_angle = 0.0

# 웨이포인트 파일 로드
if not os.path.exists(WAYPOINT_CSV):
    print(f'웨이포인트 파일 {WAYPOINT_CSV}가 없습니다.')
    sys.exit(1)

def latlon_to_meters(lat, lon):
    x = EARTH_RADIUS_METERS * math.radians(lon)
    y = EARTH_RADIUS_METERS * math.log(math.tan((90 + lat) * math.pi / 360.0))
    return x, y

df = pd.read_csv(WAYPOINT_CSV)
if not ('Lat' in df.columns and 'Lon' in df.columns):
    raise ValueError('웨이포인트 파일에 "Lat","Lon" 헤더가 있어야 합니다.')

coords = [latlon_to_meters(row['Lat'], row['Lon']) for _, row in df.iterrows()]
filtered_x = [coords[0][0]]
filtered_y = [coords[0][1]]
for x, y in coords[1:]:
    if np.hypot(x - filtered_x[-1], y - filtered_y[-1]) >= MIN_WAYPOINT_DISTANCE:
        filtered_x.append(x)
        filtered_y.append(y)
waypoints = np.array([filtered_x, filtered_y])

waypoint_index = 0

# 위치 이력
current_x, current_y = [], []
heading_vectors = []
filtered_steering_angle = 0.0

# 최근 N개 위치 이동평균용
N = 5
x_hist = collections.deque(maxlen=N)
y_hist = collections.deque(maxlen=N)

def smoothed_pos(x_list, y_list):
    if len(x_list) == 0:
        return None, None
    return np.mean(x_list), np.mean(y_list)

# 로그 CSV 준비
if not os.path.exists(OUTPUT_CSV):
    with open(OUTPUT_CSV, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'current_x','current_y','prev_x','prev_y',
            'target_vector_x','target_vector_y','waypoint_x','waypoint_y','raw_angle','filtered_angle'
        ])

def dm_to_dec(dm, direction):
    try:
        d = int(float(dm) / 100)
        m = float(dm) - d * 100
        dec = d + m / 60
        return -dec if direction in ['S', 'W'] else dec
    except:
        return None

def calculate_steering_angle(v1, v2, prev_angle):
    norm1 = np.linalg.norm(v1)
    norm2 = np.linalg.norm(v2)
    min_mag = 0.02
    if norm1 < min_mag or norm2 < min_mag:
        return prev_angle
    dot = np.dot(v1, v2)
    cos_theta = max(min(dot / (norm1 * norm2), 1.0), -1.0)
    angle = math.degrees(math.acos(cos_theta))
    cross = v1[0] * v2[1] - v1[1] * v2[0]
    if cross < 0:
        angle = -angle
    angle = angle / 1.3
    angle = np.clip(angle, -25, 25)
    return -angle

def apply_low_pass_filter(prev, current):
    global alpha
    return (1 - alpha) * prev + alpha * current

def gps_thread():
    global waypoint_index, filtered_steering_angle
    prev_sx = None
    prev_sy = None
    prev_angle = 0.0

    while True:
        try:
            line = ser.readline().decode('ascii', errors='ignore').strip()
            if not line:
                time.sleep(0.01)
                continue
            if line.startswith('$GPGGA') or line.startswith('$GNGGA'):
                parts = line.split(',')
                if len(parts) > 5 and parts[2] and parts[3] and parts[4] and parts[5]:
                    lat = dm_to_dec(parts[2], parts[3])
                    lon = dm_to_dec(parts[4], parts[5])
                    if lat is None or lon is None:
                        continue
                    x, y = latlon_to_meters(lat, lon)
                    x_hist.append(x)
                    y_hist.append(y)
                    sx, sy = smoothed_pos(x_hist, y_hist)
                    if len(current_x) > 0:
                        jump = np.hypot(sx - current_x[-1], sy - current_y[-1])
                        if jump > GPS_JUMP_THRESHOLD:
                            print(f"Jump detected! {jump:.2f} m")
                            continue
                    if (len(current_x) == 0 or np.hypot(sx - current_x[-1], sy - current_y[-1]) >= 0.001):
                        current_x.append(sx)
                        current_y.append(sy)
                        # steering 각
                        if prev_sx is not None and prev_sy is not None:
                            v1 = np.array([sx - prev_sx, sy - prev_sy])
                            tx, ty = waypoints[0][waypoint_index], waypoints[1][waypoint_index]
                            v2 = np.array([tx - sx, ty - sy])
                            angle = calculate_steering_angle(v1, v2, prev_angle)
                            prev_angle = angle
                            filtered_steering_angle = apply_low_pass_filter(filtered_steering_angle, angle)
                            print(f"Raw steering: {angle:.2f}, Filtered steering: {filtered_steering_angle:.2f}")
                            with open(OUTPUT_CSV, 'a', newline='') as f:
                                writer = csv.writer(f)
                                writer.writerow([
                                    sx, sy, prev_sx, prev_sy, v2[0], v2[1], tx, ty, angle, filtered_steering_angle
                                ])
                            if np.hypot(sx - tx, sy - ty) < TARGET_RADIUS and waypoint_index < waypoints.shape[1] - 1:
                                waypoint_index += 1
                        prev_sx = sx
                        prev_sy = sy
            time.sleep(0.01)
        except Exception as e:
            print(f"GPS error: {e}")
            time.sleep(0.1)
            continue
import sys
import math
import time
import serial
import csv
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import threading
import os
import collections   # 이동평균용


GPS_PORT = 'COM11'
GPS_BAUD = 115200
WAYPOINT_CSV = 'waypoints.csv'
OUTPUT_CSV = 'heading_vectors.csv'
PATH_SIZE = 1000
EARTH_RADIUS_METERS = 6378137.0
MIN_WAYPOINT_DISTANCE = 2.8
TARGET_RADIUS = 2.8
STEER_DIST_THRESHOLD = 0.1
GPS_JUMP_THRESHOLD = 0.5

# 저역필터 설정
f_c = 2.0
f_s = 10.0
alpha = (2 * math.pi * f_c) / (2 * math.pi * f_c + f_s)
filtered_steering_angle = 0.0

# 웨이포인트 파일 로드
if not os.path.exists(WAYPOINT_CSV):
    print(f'웨이포인트 파일 {WAYPOINT_CSV}가 없습니다.')
    sys.exit(1)

def latlon_to_meters(lat, lon):
    x = EARTH_RADIUS_METERS * math.radians(lon)
    y = EARTH_RADIUS_METERS * math.log(math.tan((90 + lat) * math.pi / 360.0))
    return x, y

df = pd.read_csv(WAYPOINT_CSV)
if not ('Lat' in df.columns and 'Lon' in df.columns):
    raise ValueError('웨이포인트 파일에 "Lat","Lon" 헤더가 있어야 합니다.')

coords = [latlon_to_meters(row['Lat'], row['Lon']) for _, row in df.iterrows()]
filtered_x = [coords[0][0]]
filtered_y = [coords[0][1]]
for x, y in coords[1:]:
    if np.hypot(x - filtered_x[-1], y - filtered_y[-1]) >= MIN_WAYPOINT_DISTANCE:
        filtered_x.append(x)
        filtered_y.append(y)
waypoints = np.array([filtered_x, filtered_y])

waypoint_index = 0

# 위치 이력
current_x, current_y = [], []
heading_vectors = []
filtered_steering_angle = 0.0

# 최근 N개 위치 이동평균용
N = 5
x_hist = collections.deque(maxlen=N)
y_hist = collections.deque(maxlen=N)

def smoothed_pos(x_list, y_list):
    if len(x_list) == 0:
        return None, None
    return np.mean(x_list), np.mean(y_list)

# 로그 CSV 준비
if not os.path.exists(OUTPUT_CSV):
    with open(OUTPUT_CSV, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'current_x','current_y','prev_x','prev_y',
            'target_vector_x','target_vector_y','waypoint_x','waypoint_y','raw_angle','filtered_angle'
        ])

def dm_to_dec(dm, direction):
    try:
        d = int(float(dm) / 100)
        m = float(dm) - d * 100
        dec = d + m / 60
        return -dec if direction in ['S', 'W'] else dec
    except:
        return None

def calculate_steering_angle(v1, v2, prev_angle):
    norm1 = np.linalg.norm(v1)
    norm2 = np.linalg.norm(v2)
    min_mag = 0.02
    if norm1 < min_mag or norm2 < min_mag:
        return prev_angle
    dot = np.dot(v1, v2)
    cos_theta = max(min(dot / (norm1 * norm2), 1.0), -1.0)
    angle = math.degrees(math.acos(cos_theta))
    cross = v1[0] * v2[1] - v1[1] * v2[0]
    if cross < 0:
        angle = -angle
    angle = angle / 1.3
    angle = np.clip(angle, -25, 25)
    return -angle

def apply_low_pass_filter(prev, current):
    global alpha
    return (1 - alpha) * prev + alpha * current

def gps_thread():
    global waypoint_index, filtered_steering_angle
    ser = serial.Serial(GPS_PORT, GPS_BAUD, timeout=0.1)
    prev_sx = None
    prev_sy = None
    prev_angle = 0.0
    ser.write(b'2')  # GPS 모드 전환 명령 (모드 문자)

    while True:
        try:
            line = ser.readline().decode('ascii', errors='ignore').strip()
            if not line:
                time.sleep(0.01)
                continue
            if line.startswith('$GPGGA') or line.startswith('$GNGGA'):
                parts = line.split(',')
                if len(parts) > 5 and parts[2] and parts[3] and parts[4] and parts[5]:
                    lat = dm_to_dec(parts[2], parts[3])
                    lon = dm_to_dec(parts[4], parts[5])
                    if lat is None or lon is None:
                        continue
                    x, y = latlon_to_meters(lat, lon)
                    x_hist.append(x)
                    y_hist.append(y)
                    sx, sy = smoothed_pos(x_hist, y_hist)
                    if len(current_x) > 0:
                        jump = np.hypot(sx - current_x[-1], sy - current_y[-1])
                        if jump > GPS_JUMP_THRESHOLD:
                            print(f"Jump detected! {jump:.2f} m")
                            continue
                    if (len(current_x) == 0 or np.hypot(sx - current_x[-1], sy - current_y[-1]) >= 0.001):
                        current_x.append(sx)
                        current_y.append(sy)
                        # steering 각
                        if prev_sx is not None and prev_sy is not None:
                            v1 = np.array([sx - prev_sx, sy - prev_sy])
                            tx, ty = waypoints[0][waypoint_index], waypoints[1][waypoint_index]
                            v2 = np.array([tx - sx, ty - sy])
                            angle = calculate_steering_angle(v1, v2, prev_angle)
                            prev_angle = angle
                            filtered_steering_angle = apply_low_pass_filter(filtered_steering_angle, angle)
                            print(f"Raw steering: {angle:.2f}, Filtered steering: {filtered_steering_angle:.2f}")
                            int_angle = int(round(filtered_steering_angle))

                            ser.write(int_angle .to_bytes(1, byteorder='little', signed=True))  # 조향각 1바이트 전송
                            with open(OUTPUT_CSV, 'a', newline='') as f:
                                writer = csv.writer(f)
                                writer.writerow([
                                    sx, sy, prev_sx, prev_sy, v2[0], v2[1], tx, ty, angle, filtered_steering_angle
                                ])
                            if np.hypot(sx - tx, sy - ty) < TARGET_RADIUS and waypoint_index < waypoints.shape[1] - 1:
                                waypoint_index += 1
                        prev_sx = sx
                        prev_sy = sy
            time.sleep(0.01)
        except Exception as e:
            print(f"GPS error: {e}")
            time.sleep(0.1)
            continue

fig = plt.figure(figsize=(7,7))
def update_plot(_):
    global waypoint_index
    ax = plt.gca()
    ax.clear()

    # 지난 웨이포인트(빨간색)와 미도달(파란색) 분리 표시
    past_idx = max(0, waypoint_index) # 0 이상

fig = plt.figure(figsize=(7,7))
def update_plot(_):
    global waypoint_index
    ax = plt.gca()
    ax.clear()

    # 지난 웨이포인트(빨간색)와 미도달(파란색) 분리 표시
    past_idx = max(0, waypoint_index) # 0 이상
