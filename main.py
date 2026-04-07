import time
import cv2
import numpy as np
import keyboard
import os
from control.drone_control import Drone

# ===== CONFIG =====
TARGET_HEIGHT  = 190
MAX_HEIGHT     = 200
TOLERANCE      = 8
DIST_FACTOR    = 1.0
MARKER_SIZE    = 0.20
TARGET_DIST    = 150

TOL_X_CM  = 5
TOL_Y_PX  = 60

# ===== INIT =====
drone = Drone()
drone.connect()
tello = drone.tello

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
last_err_x  = 0
last_err_y  = 0
fase        = 1
medidas_x   = []
medidas_y   = []
medidas_ang = []
medidas_lat = []
avg_x = avg_y = avg_ang = avg_lat = 0.0

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

def check_q():
    if keyboard.is_pressed('q'):
        print("Q presionada — aterrizando")
        drone.send_control(0, 0, 0, 0)
        time.sleep(0.2)
        drone.land()
        cleanup()
        exit()

def ud_seguro():
    alt = tello.get_height()
    if alt >= MAX_HEIGHT:
        return -20
    if alt < TARGET_HEIGHT - TOLERANCE:
        return 15
    return 0

def sleep_con_altura(segundos, paso=0.1):
    t0 = time.time()
    while time.time() - t0 < segundos:
        check_q()
        drone.send_control(0, 0, ud_seguro(), 0)
        time.sleep(paso)

def mover_con_altura(lr, fb, yaw, segundos, paso=0.1):
    t0 = time.time()
    while time.time() - t0 < segundos:
        check_q()
        drone.send_control(lr, fb, ud_seguro(), yaw)
        time.sleep(paso)
    drone.send_control(0, 0, 0, 0)
    sleep_con_altura(0.5)

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
print("Despegando")
drone.takeoff()
time.sleep(2)

# ===== ALTURA INICIAL =====
while True:
    check_q()
    height = tello.get_height()
    print(f"Altura: {height}")
    if height >= MAX_HEIGHT:
        drone.send_control(0, 0, -20, 0)
        time.sleep(0.1)
        continue
    if height < TARGET_HEIGHT - TOLERANCE:
        drone.send_control(0, 0, 20, 0)
    else:
        drone.send_control(0, 0, 0, 0)
        break
    time.sleep(0.1)

print("Altura alcanzada, iniciando visión...")
cv2.namedWindow("Tello Vision", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Tello Vision", 960, 720)

# ===== LOOP PRINCIPAL =====
while True:
    check_q()

    frame = frame_read.frame
    if frame is None:
        continue

    display = frame.copy()
    gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)

    # ── Techo de emergencia ──────────────────────────────────────────────
    height = tello.get_height()
    if height >= MAX_HEIGHT:
        drone.send_control(0, 0, -20, 0)
        cv2.putText(display, f"!! ALTURA MAX {height} cm !!",
                    (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 3)
        out.write(display)
        cv2.imshow("Tello Vision", display)
        cv2.waitKey(1)
        time.sleep(0.05)
        continue

    ud_loop = ud_seguro()

    # Cruz de referencia
    cv2.line(display, (w//2 - 25, h//2), (w//2 + 25, h//2), (255, 0, 0), 1)
    cv2.line(display, (w//2, h//2 - 25), (w//2, h//2 + 25), (255, 0, 0), 1)

    if ids is not None and len(corners) > 0:
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
            err_y = cy - (h // 2)

            last_err_x = err_x
            last_err_y = err_y

            rmat, _  = cv2.Rodrigues(rvec)
            angle_y  = np.degrees(np.arctan2(rmat[0][2], rmat[2][2]))

            # Normalizar ángulo
            if angle_y > 90:
                angle_y = angle_y - 180
            elif angle_y < -90:
                angle_y = angle_y + 180

            # Dibujos
            cv2.polylines(display, [img_points.astype(int)], True, (0, 255, 0), 2)
            cv2.circle(display, (cx, cy), 5, (0, 0, 255), -1)
            cv2.putText(display, f"{distance:.1f} cm",
                        (cx - 40, max(20, cy - 50)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(display, f"lat:{lateral_real_cm:.1f}cm ang:{angle_y:.1f}",
                        (cx - 40, max(40, cy - 30)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

            # Flechas informativas
            acciones = []
            flechas  = []
            err_dist = distance - TARGET_DIST

            if err_dist > 15:
                acciones.append(f"Acercar {err_dist:.0f}cm")
                flechas.append('↑')
            elif err_dist < -15:
                acciones.append(f"Alejar {abs(err_dist):.0f}cm")
                flechas.append('↓')
            if lateral_real_cm > TOL_X_CM:
                acciones.append(f"Der {lateral_real_cm:.1f}cm")
                flechas.append('→')
            elif lateral_real_cm < -TOL_X_CM:
                acciones.append(f"Izq {abs(lateral_real_cm):.1f}cm")
                flechas.append('←')
            if err_y > TOL_Y_PX:
                acciones.append(f"Bajar {err_y}px")
                flechas.append('↓')
            elif err_y < -TOL_Y_PX:
                acciones.append(f"Subir {abs(err_y)}px")
                flechas.append('↑')
            if angle_y > 8:
                acciones.append(f"Girar der {angle_y:.1f}")
                flechas.append('↻')
            elif angle_y < -8:
                acciones.append(f"Girar izq {abs(angle_y):.1f}")
                flechas.append('↺')

            for f in flechas:
                dibujar_flecha(display, f)

            estado       = "CENTRADO OK" if not acciones else "  ".join(acciones)
            color_estado = (0, 255, 0)   if not acciones else (0, 200, 255)
            cv2.putText(display, estado, (10, h - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_estado, 2)
            cv2.putText(display,
                        f"F{fase} | Alt:{height} | d:{distance:.0f} | lat:{lateral_real_cm:.1f} | ey:{err_y} | ang:{angle_y:.1f}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 255, 0), 2)

            print_throttle(f"Dist:{distance:.1f} Lat:{lateral_real_cm:.1f} Fase:{fase} "
                           f"Alt:{height} ey:{err_y} ang:{angle_y:.1f} | "
                           f"{'  '.join(flechas) if flechas else 'OK'}")

            # ── FASE 1: acercarse corrigiendo lateral y ángulo ──────────
            if fase == 1:
                err_dist = distance - TARGET_DIST
                DIST_PREVIA = 250

                if distance > DIST_PREVIA:
                    # Lejos — avanzar directo
                    fb = int(np.clip(0.25 * err_dist, -20, 20))
                    drone.send_control(0, fb, ud_loop, 0)

                else:
                    # Cerca — corregir todo simultáneamente y con más fuerza
                    fb       = int(np.clip(0.25 * err_dist, -20, 20))
                    yaw_fix  = int(np.clip(0.3  * angle_y,       -20, 20))  # más agresivo
                    lr_fix   = int(np.clip(0.3  * lateral_real_cm, -20, 20))  # lateral físico

                    if abs(err_dist) < 15:
                        drone.send_control(0, 0, 0, 0)
                        print("Distancia alcanzada — estabilizando...")
                        fase = 2
                    else:
                        drone.send_control(lr_fix, fb, ud_loop, yaw_fix)

            # ── FASE 2: estabilizar ──────────────────────────────────────
            elif fase == 2:
                cv2.putText(display, "Estabilizando...",
                            (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 165, 0), 2)
                out.write(display)
                cv2.imshow("Tello Vision", display)
                cv2.waitKey(1)
                sleep_con_altura(1.5)
                medidas_x   = []
                medidas_y   = []
                medidas_ang = []
                medidas_lat = []
                fase = 3

            # ── FASE 3: acumular lecturas ────────────────────────────────
            elif fase == 3:
                if len(medidas_x) == 0 or abs(lateral_real_cm - np.mean(medidas_lat)) < 50:
                    medidas_x.append(err_x)
                    medidas_y.append(err_y)
                    medidas_ang.append(angle_y)
                    medidas_lat.append(lateral_real_cm)
                else:
                    print_throttle(f"[RUIDO] lat={lateral_real_cm:.1f} descartado")

                cv2.putText(display, f"Midiendo... {len(medidas_x)}/20",
                            (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 165, 0), 2)
                drone.send_control(0, 0, ud_loop, 0)

                if len(medidas_x) >= 20:
                    avg_x   = float(np.median(medidas_x))
                    avg_y   = float(np.median(medidas_y))
                    avg_ang = float(np.median(medidas_ang))
                    avg_lat = float(np.median(medidas_lat))
                    print(f"Mediana -> lat:{avg_lat:.1f}cm ey:{avg_y:.1f}px ang:{avg_ang:.1f}°")
                    fase = 4

            # ── FASE 4: verificar centrado ───────────────────────────────
            elif fase == 4:
                out.write(display)
                cv2.imshow("Tello Vision", display)
                cv2.waitKey(1)
                if abs(avg_lat) <= TOL_X_CM and abs(avg_ang) <= 15.0:
                    print("Centrado correcto — retrocediendo...")
                    fase = 5
                else:
                    print(f"No centrado (lat:{avg_lat:.1f} ang:{avg_ang:.1f}) — reposicionando...")
                    medidas_x   = []
                    medidas_y   = []
                    medidas_ang = []
                    medidas_lat = []
                    fase = 1

            # ── FASE 5: retroceder ───────────────────────────────────────
            elif fase == 5:
                cv2.putText(display, "Retrocediendo...",
                            (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 165, 0), 2)
                out.write(display)
                cv2.imshow("Tello Vision", display)
                cv2.waitKey(1)
                mover_con_altura(0, -20, 0, 4.0)
                fase = 6

            # ── FASE 6: aterrizar ────────────────────────────────────────
            elif fase == 6:
                print("Aterrizando...")
                drone.land()
                cleanup()
                exit()

    # ── Sin marcador: búsqueda dirigida ─────────────────────────────────
    else:
        yaw_recovery = 20 if last_err_x >= 0 else -20
        cv2.putText(display,
                    f"NO ARUCO | {'der' if last_err_x >= 0 else 'izq'}",
                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        drone.send_control(0, 0, ud_loop, yaw_recovery)
        print_throttle("NO ARUCO")

    out.write(display)
    cv2.imshow("Tello Vision", display)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        check_q()
    time.sleep(0.05)

cleanup()

