import time
import cv2
import numpy as np
import keyboard
from control.drone_control import Drone

# ===== CONFIG =====
TARGET_HEIGHT = 150
TOLERANCE = 10
MAX_HEIGHT = 180

MIN_DIST = 200
MAX_DIST = 250

DIST_FACTOR = 2.181
MARKER_SIZE = 0.20

# ===== INIT =====
drone = Drone()
drone.connect()

tello = drone.tello
frame_read = tello.get_frame_read()

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

# Cámara
frame = frame_read.frame
h, w = frame.shape[:2]

focal_length = w
camera_matrix = np.array([
    [focal_length, 0, w / 2],
    [0, focal_length, h / 2],
    [0, 0, 1]
])

dist_coeffs = np.zeros((4, 1))

half_size = MARKER_SIZE / 2
obj_points = np.array([
    [-half_size,  half_size, 0],
    [ half_size,  half_size, 0],
    [ half_size, -half_size, 0],
    [-half_size, -half_size, 0]
], dtype=np.float32)

# ===== TAKEOFF =====
print("Despegando")
drone.takeoff()
time.sleep(2)

# ===== ALTURA =====
while True:
    height = tello.get_height()
    print(f"Altura: {height}")

    if keyboard.is_pressed('q'):
        drone.land()
        exit()

    if height > MAX_HEIGHT:
        drone.send_control(0, 0, -20, 0)
        continue

    if height < TARGET_HEIGHT - TOLERANCE:
        drone.send_control(0, 0, 25, 0)
    else:
        drone.send_control(0, 0, 0, 0)
        break

    time.sleep(0.1)

print("Altura alcanzada, iniciando visión...")

# ===== LOOP PRINCIPAL =====
while True:

    frame = frame_read.frame
    if frame is None:
        continue

    display = frame.copy()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, ids, _ = detector.detectMarkers(gray)

    if keyboard.is_pressed('q'):
        drone.land()
        break

    # Mantener altura
    height = tello.get_height()
    if height < TARGET_HEIGHT - TOLERANCE:
        drone.send_control(0, 0, 15, 0)
        continue
    elif height > TARGET_HEIGHT + TOLERANCE:
        drone.send_control(0, 0, -15, 0)
        continue

    if ids is not None:

        for corner in corners:

            img_points = corner[0].astype(np.float32)

            success, rvec, tvec = cv2.solvePnP(
                obj_points,
                img_points,
                camera_matrix,
                dist_coeffs
            )

            if success:
                distance = tvec[2][0] * 100 * DIST_FACTOR

                # Centro
                cx = int(img_points[:, 0].mean())
                cy = int(img_points[:, 1].mean())

                # Dibujos
                cv2.polylines(display, [img_points.astype(int)], True, (0,255,0), 2)
                cv2.circle(display, (cx, cy), 5, (0,0,255), -1)

                cv2.putText(display, f"{distance:.1f} cm",
                            (cx - 40, cy - 30),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            (0,255,0), 2)

                print(f"Distancia: {distance:.2f} cm")

                # Control
                if distance > MAX_DIST:
                    drone.send_control(0, 20, 0, 0)
                elif distance < MIN_DIST:
                    drone.send_control(0, -20, 0, 0)
                else:
                    drone.send_control(0, 0, 0, 0)
                    print("Distancia correcta")
                    time.sleep(2)
                    drone.land()
                    exit()

    else:
        cv2.putText(display, "NO ARUCO",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0,0,255), 2)

        drone.send_control(0, 0, 0, 20)

    # Mostrar cámara
    cv2.imshow("Tello Vision", display)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        drone.land()
        break

    time.sleep(0.05)

cv2.destroyAllWindows()
tello.streamoff()
