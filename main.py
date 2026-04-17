import time
import cv2
import numpy as np
import keyboard
import os
from control.drone_control import Drone

# ===== CONFIG =====
TARGET_HEIGHT  = 200
MAX_HEIGHT     = 300
TOLERANCE      = 8
DIST_FACTOR    = 1.0
MARKER_SIZE    = 0.125
TARGET_DIST    = 150

TOL_X_CM  = 5
TOL_Y_PX  = 300

# Altura durante centrado
TARGET_HEIGHT_CENTRADO = 190
MAX_HEIGHT_CENTRADO    = 230

# Desplazamiento post-centrado
DESP_LATERAL_CM  = 50
DESP_VERTICAL_CM = 40
VEL_DESP         = 15

# Aproximación al pizarrón
VEL_APROX    = 10
MARGEN_APROX = 15

# Trazo
VEL_LATERAL   = 25
FB_CONTACTO   = 8
LONG_LINEA_CM = 250

# Búsqueda lateral
VEL_BUSQUEDA     = 20
TIMEOUT_BUSQUEDA = 10.0

# ===== INIT =====
drone = Drone()
drone.connect()
tello = drone.tello

# ===== MEDIR SENSORES ANTES DE ENCENDER CAMARA =====
print("Midiendo sensores iniciales...")
time.sleep(2.0)
lecturas_altura = []
t0 = time.time()
while len(lecturas_altura) < 10 and time.time() - t0 < 5.0:
    h_raw = tello.get_height()
    if 0 <= h_raw <= 30:
        lecturas_altura.append(h_raw)
    time.sleep(0.2)
altura_inicial = int(np.median(lecturas_altura)) if lecturas_altura else 0
yaw_inicial    = tello.get_yaw()
print(f"Altura base: {altura_inicial}cm | Yaw base: {yaw_inicial}°")

# ===== CAMARA =====
tello.streamon()
time.sleep(2)
frame_read = tello.get_frame_read()
time.sleep(1)

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
parameters = cv2.aruco.DetectorParameters()
detector   = cv2.aruco.ArucoDetector(aruco_dict, parameters)

while True:
    frame = frame_read.frame
    if frame is not None:
        break
    time.sleep(0.1)

h, w  = frame.shape[:2]
print(f"Resolución: {w}x{h}")

script_dir = os.path.dirname(os.path.abspath(__file__))
video_path = os.path.join(script_dir, f"vuelo_{int(time.time())}.mp4")
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter(video_path, fourcc, 30, (w, h))

focal_length  = w
camera_matrix = np.array([
    [focal_length, 0, w / 2],
    [0, focal_length, h / 2],
    [0, 0, 1]
])
dist_coeffs   = np.zeros((4, 1))
half_size     = MARKER_SIZE / 2
obj_points    = np.array([
    [-half_size,  half_size, 0],
    [ half_size,  half_size, 0],
    [ half_size, -half_size, 0],
    [-half_size, -half_size, 0]
], dtype=np.float32)

# ===== ESTADO =====
last_err_x      = 0
last_err_y      = 0
fase            = 0
dist_al_centrar = 0.0
ang_al_centrar  = 0.0
lat_al_centrar  = 0.0
centrado_listo  = False

_ultimo_print = 0.0
def print_throttle(msg):
    global _ultimo_print
    ahora = time.time()
    if ahora - _ultimo_print >= 0.2:
        print(msg)
        _ultimo_print = ahora


# ===== HELPERS =====

def cleanup():
    try:
        out.release()
    except:
        pass
    cv2.destroyAllWindows()
    try:
        tello.streamoff()
    except:
        pass

def safe_land():
    drone.send_control(0, 0, 0, 0)
    time.sleep(0.2)
    try:
        drone.land()
    except Exception as e:
        print(f"Land ignorado: {e}")

def emergency_stop():
    print("EMERGENCIA — motores cortados")
    try:
        tello.emergency()
    except:
        pass
    cleanup()
    exit()

def check_keys():
    if keyboard.is_pressed('space'):
        emergency_stop()
    if keyboard.is_pressed('q'):
        print("Q presionada — aterrizando")
        drone.send_control(0, 0, 0, 0)
        time.sleep(0.2)
        if fase >= 8:
            print("Retrocediendo antes de aterrizar")
            t0 = time.time()
            while time.time() - t0 < 3.5:
                if keyboard.is_pressed('space'):
                    emergency_stop()
                drone.send_control(0, -20, ud_seguro(), 0)
                time.sleep(0.1)
            drone.send_control(0, 0, 0, 0)
        safe_land()
        cleanup()
        exit()

def ud_seguro():
    alt = tello.get_height()
    if alt >= MAX_HEIGHT:
        return -20
    if alt < TARGET_HEIGHT - TOLERANCE:
        return 15
    return 0

def ud_centrado():
    alt = tello.get_height()
    if alt >= MAX_HEIGHT_CENTRADO:
        return -20
    if alt < TARGET_HEIGHT_CENTRADO - TOLERANCE:
        return 15
    return 0

def sleep_con_altura(segundos, paso=0.1):
    t0 = time.time()
    while time.time() - t0 < segundos:
        check_keys()
        drone.send_control(0, 0, ud_centrado(), 0)
        time.sleep(paso)

def rotar_grados(grados, velocidad=30, timeout=5.0):
    yaw_antes = tello.get_yaw()
    objetivo  = yaw_antes + grados
    if objetivo > 180:  objetivo -= 360
    if objetivo < -180: objetivo += 360
    t0 = time.time()
    print(f"Rotando {grados}° | yaw_antes:{yaw_antes}° objetivo:{objetivo}°")
    while time.time() - t0 < timeout:
        check_keys()
        yaw_actual = tello.get_yaw()
        err = objetivo - yaw_actual
        if err > 180:  err -= 360
        if err < -180: err += 360
        if abs(err) < 5:
            break
        corr = int(np.clip(err * 0.6, -velocidad, velocidad))
        drone.send_control(0, 0, ud_seguro(), corr)
        time.sleep(0.1)
    drone.send_control(0, 0, 0, 0)
    time.sleep(0.5)
    print(f"Rotación completada | yaw_final:{tello.get_yaw()}°")

def dibujar_flecha(display, direccion, color=(0, 200, 255)):
    cx, cy = w // 2, h // 2
    tam    = 80
    grosor = 6
    if '←' in direccion:
        cv2.arrowedLine(display, (cx + tam, cy), (cx - tam, cy), color, grosor, tipLength=0.4)
    if '→' in direccion:
        cv2.arrowedLine(display, (cx - tam, cy), (cx + tam, cy), color, grosor, tipLength=0.4)
    if '↑' in direccion:
        cv2.arrowedLine(display, (cx, cy + tam), (cx, cy - tam), color, grosor, tipLength=0.4)
    if '↓' in direccion:
        cv2.arrowedLine(display, (cx, cy - tam), (cx, cy + tam), color, grosor, tipLength=0.4)
    if '↻' in direccion:
        cv2.putText(display, "GIRAR DER >>", (cx - 100, cy + 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 3)
    if '↺' in direccion:
        cv2.putText(display, "<< GIRAR IZQ", (cx - 100, cy + 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 3)


# ===== TAKEOFF =====
fase = 0
print("Despegando — avanzando lentamente para estabilizar...")
drone.takeoff()
time.sleep(1.5)

yaw_despegue = tello.get_yaw()
print(f"Yaw post-despegue: {yaw_despegue}°")

while True:
    check_keys()
    height = tello.get_height()
    print(f"Altura: {height}")
    if height >= MAX_HEIGHT:
        drone.send_control(0, 0, -20, 0)
        time.sleep(0.1)
        continue
    if height < TARGET_HEIGHT - TOLERANCE:
        drone.send_control(0, 15, 20, 0)
    else:
        drone.send_control(0, 0, 0, 0)
        break
    time.sleep(0.1)

print("Altura alcanzada — rotando 90° a la derecha...")

# ===== ROTAR 90° A LA DERECHA =====
fase = 1
rotar_grados(90, velocidad=30)
yaw_tras_rotar = tello.get_yaw()
print(f"Yaw tras rotación: {yaw_tras_rotar}°")
time.sleep(0.5)

# ===== BUSCAR ARUCO AVANZANDO A LA IZQUIERDA =====
fase = 2
print(f"Buscando aruco | timeout:{TIMEOUT_BUSQUEDA}s...")

cv2.namedWindow("Tello Vision", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Tello Vision", 960, 720)

t_busqueda = time.time()

while True:
    check_keys()

    frame = frame_read.frame
    if frame is None:
        continue

    display = frame.copy()
    gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)

    ud_loop   = ud_seguro()
    yaw_actual = tello.get_yaw()
    err_yaw    = yaw_tras_rotar - yaw_actual
    if err_yaw > 180:  err_yaw -= 360
    if err_yaw < -180: err_yaw += 360
    corr_yaw_busqueda = int(np.clip(err_yaw * 0.4, -15, 15))

    tiempo_buscando = time.time() - t_busqueda

    if ids is not None and len(corners) > 0:
        drone.send_control(0, 0, 0, 0)
        print(f"Aruco detectado tras {tiempo_buscando:.1f}s")
        time.sleep(0.3)
        break

    if tiempo_buscando > TIMEOUT_BUSQUEDA:
        print(f"Timeout búsqueda ({TIMEOUT_BUSQUEDA}s) — aterrizando")
        safe_land()
        cleanup()
        exit()

    seg_restantes = int(TIMEOUT_BUSQUEDA - tiempo_buscando)
    cv2.putText(display, f"Buscando izquierda | {seg_restantes}s",
                (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)
    drone.send_control(-VEL_BUSQUEDA, 0, ud_loop, corr_yaw_busqueda)

    out.write(display)
    cv2.imshow("Tello Vision", display)
    cv2.waitKey(1)
    time.sleep(0.05)


# ===== ACERCARSE A 150CM Y HACER LECTURA UNICA =====
fase = 3
print("Acercándose a 150cm para lectura única...")
t_sin_aruco    = time.time()
lectura_hecha  = False

while not lectura_hecha:
    check_keys()

    frame = frame_read.frame
    if frame is None:
        continue

    display = frame.copy()
    gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)

    height  = tello.get_height()
    if height >= MAX_HEIGHT_CENTRADO:
        drone.send_control(0, 0, -20, 0)
        cv2.imshow("Tello Vision", display)
        cv2.waitKey(1)
        time.sleep(0.05)
        continue

    ud_loop = ud_centrado()

    cv2.line(display, (w//2 - 25, h//2), (w//2 + 25, h//2), (255, 0, 0), 1)
    cv2.line(display, (w//2, h//2 - 25), (w//2, h//2 + 25), (255, 0, 0), 1)

    if ids is not None and len(corners) > 0:
        t_sin_aruco = time.time()

        for i, corner in enumerate(corners):
            img_points = corner[0].astype(np.float32)
            success, rvec, tvec = cv2.solvePnP(
                obj_points, img_points, camera_matrix, dist_coeffs
            )
            if not success:
                continue

            distance        = tvec[2][0] * 100 * DIST_FACTOR
            lateral_real_cm = float(tvec[0][0] * 100)

            cx    = int(img_points[:, 0].mean())
            cy    = int(img_points[:, 1].mean())
            err_x = cx - (w // 2)

            last_err_x = err_x

            rmat, _ = cv2.Rodrigues(rvec)
            angle_y = np.degrees(np.arctan2(rmat[0][2], rmat[2][2]))
            if angle_y > 90:  angle_y -= 180
            if angle_y < -90: angle_y += 180

            cv2.polylines(display, [img_points.astype(int)], True, (0, 255, 0), 2)
            cv2.circle(display, (cx, cy), 5, (0, 0, 255), -1)
            cv2.putText(display, f"{distance:.1f}cm lat:{lateral_real_cm:.1f} ang:{angle_y:.1f}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

            err_dist = distance - TARGET_DIST

            if abs(err_dist) < 15:
                # Llegó a 150cm — guardar lectura y hacer corrección única
                dist_al_centrar = distance
                lat_al_centrar  = lateral_real_cm
                ang_al_centrar  = angle_y
                print(f"Lectura única — dist:{dist_al_centrar:.1f}cm lat:{lat_al_centrar:.1f}cm ang:{ang_al_centrar:.1f}°")

                drone.send_control(0, 0, 0, 0)
                time.sleep(0.5)

                # Corrección de lateral en un solo movimiento
                if abs(lat_al_centrar) > TOL_X_CM:
                    vel_lat   = int(np.clip(lat_al_centrar * 0.5, -20, 20))
                    t_corr_lat = abs(lat_al_centrar) / max(abs(vel_lat), 1) * 0.6
                    print(f"Corrigiendo lateral {lat_al_centrar:.1f}cm vel:{vel_lat} t:{t_corr_lat:.1f}s")
                    t0 = time.time()
                    while time.time() - t0 < t_corr_lat:
                        check_keys()
                        drone.send_control(vel_lat, 0, ud_centrado(), 0)
                        time.sleep(0.1)
                    drone.send_control(0, 0, 0, 0)
                    time.sleep(0.3)

                # Corrección de ángulo en un solo movimiento
                if abs(ang_al_centrar) > 5:
                    vel_yaw   = int(np.clip(ang_al_centrar * 0.6, -20, 20))
                    t_corr_ang = abs(ang_al_centrar) / max(abs(vel_yaw), 1) * 0.5
                    print(f"Corrigiendo ángulo {ang_al_centrar:.1f}° vel:{vel_yaw} t:{t_corr_ang:.1f}s")
                    t0 = time.time()
                    while time.time() - t0 < t_corr_ang:
                        check_keys()
                        drone.send_control(0, 0, ud_centrado(), vel_yaw)
                        time.sleep(0.1)
                    drone.send_control(0, 0, 0, 0)
                    time.sleep(0.3)

                print("Corrección única completada")
                lectura_hecha  = True
                centrado_listo = True
                break
            else:
                fb = int(np.clip(0.25 * err_dist, -20, 20))
                drone.send_control(0, fb, ud_loop, 0)

    else:
        tiempo_sin = time.time() - t_sin_aruco
        if tiempo_sin > 15.0:
            print("Sin aruco 15s — aterrizando")
            safe_land()
            cleanup()
            exit()
        yaw_recovery = 20 if last_err_x >= 0 else -20
        drone.send_control(0, 0, ud_loop, yaw_recovery)
        print_throttle("NO ARUCO buscando...")

    out.write(display)
    cv2.imshow("Tello Vision", display)
    cv2.waitKey(1)
    time.sleep(0.05)


# ===== SECUENCIA DE DIBUJO =====
cv2.destroyAllWindows()
fase = 7

print("Iniciando secuencia de dibujo...")

# ── FASE 7: moverse 50cm derecha y 40cm abajo ────────────────────────────
print(f"Desplazando {DESP_LATERAL_CM}cm derecha y {DESP_VERTICAL_CM}cm abajo...")
altura_al_centrar = int(np.median([tello.get_height() for _ in range(5)]))
time.sleep(0.3)
TARGET_HEIGHT = max(altura_al_centrar - DESP_VERTICAL_CM, 50)

TIEMPO_DESP = max(DESP_LATERAL_CM, DESP_VERTICAL_CM) / VEL_DESP
t0 = time.time()
while time.time() - t0 < TIEMPO_DESP:
    check_keys()
    alt     = tello.get_height()
    ud_desp = -15 if alt > TARGET_HEIGHT + 5 else (15 if alt < TARGET_HEIGHT - 5 else 0)
    drone.send_control(VEL_DESP, 0, ud_desp, 0)
    time.sleep(0.1)
drone.send_control(0, 0, 0, 0)
time.sleep(1.0)
print(f"Desplazado — altura actual: {tello.get_height()}cm")

# ── FASE 8: avanzar al pizarrón ──────────────────────────────────────────
fase = 8
dist_avance  = dist_al_centrar + MARGEN_APROX
TIEMPO_APROX = dist_avance / VEL_APROX
print(f"Avanzando {dist_avance:.1f}cm a vel {VEL_APROX} ({TIEMPO_APROX:.1f}s)...")

t0 = time.time()
while time.time() - t0 < TIEMPO_APROX:
    check_keys()
    drone.send_control(0, VEL_APROX, ud_seguro(), 0)
    time.sleep(0.1)
drone.send_control(0, 0, 0, 0)
time.sleep(0.5)
print("En contacto con pizarron")

# ── FASE 9: trazar línea con corrección de yaw ────────────────────────────
fase = 9
TIEMPO_LINEA = LONG_LINEA_CM / VEL_LATERAL
yaw_linea    = tello.get_yaw()
print(f"Trazando {LONG_LINEA_CM}cm | vel:{VEL_LATERAL} fb:{FB_CONTACTO} ({TIEMPO_LINEA:.1f}s)...")

t0 = time.time()
while time.time() - t0 < TIEMPO_LINEA:
    check_keys()
    yaw_actual = tello.get_yaw()
    err_yaw    = yaw_linea - yaw_actual
    if err_yaw > 180:  err_yaw -= 360
    if err_yaw < -180: err_yaw += 360
    corr_yaw = int(np.clip(err_yaw * 0.5, -10, 10))
    drone.send_control(VEL_LATERAL, FB_CONTACTO, ud_seguro(), corr_yaw)
    time.sleep(0.1)
drone.send_control(0, 0, 0, 0)
time.sleep(0.5)
print("Linea trazada")

# ── FASE 10: retroceder ───────────────────────────────────────────────────
fase = 10
TARGET_HEIGHT = altura_al_centrar
print("Retrocediendo...")

t0 = time.time()
while time.time() - t0 < 6.0:
    check_keys()
    drone.send_control(0, -20, ud_seguro(), 0)
    time.sleep(0.1)
drone.send_control(0, 0, 0, 0)
time.sleep(0.5)

# ── FASE 11: aterrizar ────────────────────────────────────────────────────
fase = 11
print("Aterrizando...")
safe_land()
cleanup()