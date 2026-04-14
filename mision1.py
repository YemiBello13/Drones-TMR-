import time
import cv2
import numpy as np
import keyboard
from control.drone_control import Drone

# ===== CONFIG =====
TARGET_DIST    = 100   # Se detiene a 1 metro para centrarse
TOL_PX_PRECISO = 20    # Margen de error de solo 20 píxeles (muy centrado)
ARCO_LADO_CM   = 52.0  # Tamaño real del marco de PVC

# ===== INIT =====
drone = Drone()
drone.connect()
tello = drone.tello
tello.streamon()
time.sleep(2)
frame_read = tello.get_frame_read()

# Dimensiones de cámara Tello (960x720)
W, H = 960, 720
centro_x_img = W // 2
centro_y_img = H // 2

def check_q():
    """Paro de emergencia con la tecla Q"""
    if keyboard.is_pressed('q'):
        print("!!! EMERGENCIA - ATERRIZANDO !!!")
        drone.send_control(0, 0, 0, 0)
        tello.land()
        exit()

def detectar_arco(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # Rango para el azul del PVC (ajustar si es necesario)
    lower = np.array([90, 80, 50])
    upper = np.array([130, 255, 255])
    mask = cv2.inRange(hsv, lower, upper)
    
    kernel = np.ones((7, 7), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours: return None
    
    c = max(contours, key=cv2.contourArea)
    if cv2.contourArea(c) < 1500: return None

    x, y, bw, bh = cv2.boundingRect(c)
    cx, cy = x + bw // 2, y + bh // 2
    distancia = (ARCO_LADO_CM * W) / bw # Estimación de distancia
    
    return (cx, cy, distancia, x, y, bw, bh)

# ===== VUELO =====
drone.takeoff()
time.sleep(2)

fase = 1
dist_al_cruzar = 0
last_err_x = 0

print("Buscando arco para centrado de precisión...")

while True:
    check_q()
    frame = frame_read.frame
    if frame is None: continue
    
    display = frame.copy()
    res = detectar_arco(frame)

    if res:
        cx, cy, dist, x, y, bw, bh = res
        err_x = cx - centro_x_img
        err_y = cy - centro_y_img # Si es + el dron debe bajar, si es - debe subir
        last_err_x = err_x

        # Dibujar guías
        cv2.rectangle(display, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
        cv2.circle(display, (cx, cy), 5, (0, 0, 255), -1)
        cv2.putText(display, f"Dist: {int(dist)}cm", (x, y-10), 1, 1.5, (0,255,0), 2)

        # CÁLCULO DE MOVIMIENTOS
        # Centrado Lateral (Roll)
        lr = int(np.clip(0.25 * err_x, -25, 25)) if abs(err_x) > TOL_PX_PRECISO else 0
        # Centrado Vertical (Up/Down)
        ud = int(np.clip(-0.25 * err_y, -25, 25)) if abs(err_y) > TOL_PX_PRECISO else 0
        # Distancia (Forward)
        err_dist = dist - TARGET_DIST
        fb = int(np.clip(0.3 * err_dist, -20, 20)) if abs(err_dist) > 10 else 0

        drone.send_control(lr, fb, ud, 0)

        # Si está perfectamente centrado en los 25cm (X e Y) y a la distancia
        if lr == 0 and ud == 0 and fb == 0:
            print("!!! CENTRADO 25cm x 25cm ALCANZADO !!!")
            dist_al_cruzar = dist
            drone.send_control(0, 0, 0, 0)
            time.sleep(1)
            break
    else:
        # Si no ve el arco, gira para buscarlo
        yaw = 20 if last_err_x >= 0 else -20
        drone.send_control(0, 0, 0, yaw)

    # ESTO DEBE ESTAR AQUÍ: Fuera del IF pero dentro del WHILE
    cv2.imshow("Arco Vision", display)
    if cv2.waitKey(1) & 0xFF == ord('q'): break

# ===== CRUCE FINAL =====
print("Cruzando...")
t0 = time.time()
# Avanza la distancia medida + 80cm extra para asegurar que pase
tiempo_cruce = (dist_al_cruzar + 80) / 30 
while time.time() - t0 < tiempo_cruce:
    check_q()
    drone.send_control(0, 30, 0, 0)
    time.sleep(0.1)

drone.send_control(0, 0, 0, 0)
tello.land()
cv2.destroyAllWindows()
