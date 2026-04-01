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
DIST_FACTOR    = 2.181
MARKER_SIZE    = 0.20
TARGET_DIST    = 120

TOL_X_PX  = 40   # tolerancia lateral en píxeles para considerar centrado
TOL_ANG   = 6.0  # tolerancia angular en grados
MAX_INTENTOS_CENTRADO = 4  # máximo de vueltas del bucle de centrado

# ===== INIT =====
drone = Drone()
drone.connect()
tello = drone.tello
frame_read = tello.get_frame_read()

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
parameters = cv2.aruco.DetectorParameters()
detector   = cv2.aruco.ArucoDetector(aruco_dict, parameters)

frame = frame_read.frame
h, w  = frame.shape[:2]

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
last_err_x   = 0
last_err_y   = 0
last_lado    = 0   # último lado conocido: +1 derecha, -1 izquierda, 0 desconocido

# Fases principales:
# 1 = acercarse
# 2 = estabilizar antes de medir
# 3 = medir (acumular)
# 4 = corregir lateral (lr)
# 5 = corregir angular (yaw)
# 6 = verificar — si ok → 7, si no → volver a 2
# 7 = retroceder
# 8 = aterrizar
fase           = 1
intento        = 0   # contador de intentos del bucle de centrado

medidas_x      = []
medidas_y      = []
medidas_ang    = []
medidas_lat    = []

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
    try: out.release()
    except: pass
    cv2.destroyAllWindows()
    try: tello.streamoff()
    except: pass

def check_q():
    if keyboard.is_pressed('q'):
        print("Q — aterrizando")
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
    """Ejecuta movimiento manteniendo altura y respondiendo a q."""
    dur = min(segundos, 5.0)   # límite de seguridad: nunca más de 5 s de un tirón
    t0  = time.time()
    while time.time() - t0 < dur:
        check_q()
        drone.send_control(lr, fb, ud_seguro(), yaw)
        time.sleep(paso)
    drone.send_control(0, 0, 0, 0)
    sleep_con_altura(0.5)

def medir_frames(n=20):
    """
    Acumula n frames válidos con el marcador visible.
    Devuelve (med_err_x, med_err_y, med_ang, med_lat) usando mediana,
    o None si no se consiguieron suficientes lecturas en el tiempo límite.
    """
    xs, ys, angs, lats = [], [], [], []
    t_limite = time.time() + 8.0   # máximo 8 s para acumular
    while len(xs) < n:
        check_q()
        if time.time() > t_limite:
            print("Tiempo límite de medición agotado")
            return None

        frm = frame_read.frame
        if frm is None:
            time.sleep(0.05)
            continue

        gris = cv2.cvtColor(frm, cv2.COLOR_BGR2GRAY)
        crns, iids, _ = detector.detectMarkers(gris)

        if iids is None or len(crns) == 0:
            time.sleep(0.05)
            continue

        ipts = crns[0][0].astype(np.float32)
        ok, rvec, tvec = cv2.solvePnP(obj_points, ipts, camera_matrix, dist_coeffs)
        if not ok:
            time.sleep(0.05)
            continue

        dist = tvec[2][0] * 100 * DIST_FACTOR
        lat  = float(tvec[0][0] * 100)

        # Filtro básico de outliers
        if dist < 20 or dist > 800 or abs(lat) > 300:
            continue

        cx  = int(ipts[:, 0].mean())
        cy  = int(ipts[:, 1].mean())
        ex  = cx - w // 2
        ey  = cy - h // 2

        rmat, _ = cv2.Rodrigues(rvec)
        ang     = np.degrees(np.arctan2(rmat[0][2], rmat[2][2]))

        # Filtro coherencia respecto a lo acumulado
        if len(xs) > 0 and abs(ex - np.median(xs)) > 150:
            continue

        xs.append(ex)
        ys.append(ey)
        angs.append(ang)
        lats.append(lat)

        drone.send_control(0, 0, ud_seguro(), 0)
        time.sleep(0.05)

    return (float(np.median(xs)),
            float(np.median(ys)),
            float(np.median(angs)),
            float(np.median(lats)))


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

# ===== LOOP PRINCIPAL =====
while True:
    check_q()

    frame = frame_read.frame
    if frame is None:
        continue

    display = frame.copy()
    gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)

    # ── Seguro de altura siempre primero ────────────────────────────────
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

    # ── Marcador en frame ────────────────────────────────────────────────
    if ids is not None and len(corners) > 0:

        img_points = corners[0][0].astype(np.float32)
        ok, rvec, tvec = cv2.solvePnP(obj_points, img_points, camera_matrix, dist_coeffs)

        if ok:
            distance        = tvec[2][0] * 100 * DIST_FACTOR
            lateral_real_cm = float(tvec[0][0] * 100)

            cx    = int(img_points[:, 0].mean())
            cy    = int(img_points[:, 1].mean())
            err_x = cx - w // 2
            err_y = cy - h // 2

            last_err_x = err_x
            last_err_y = err_y
            # Actualizar último lado conocido según dónde esté el centroide
            if err_x > 30:
                last_lado = 1    # marcador a la derecha
            elif err_x < -30:
                last_lado = -1   # marcador a la izquierda

            rmat, _ = cv2.Rodrigues(rvec)
            angle_y  = np.degrees(np.arctan2(rmat[0][2], rmat[2][2]))

            # Dibujos
            cv2.polylines(display, [img_points.astype(int)], True, (0, 255, 0), 2)
            cv2.circle(display, (cx, cy), 5, (0, 0, 255), -1)
            cv2.line(display, (w//2-20, h//2), (w//2+20, h//2), (255, 0, 0), 1)
            cv2.line(display, (w//2, h//2-20), (w//2, h//2+20), (255, 0, 0), 1)
            cv2.putText(display, f"{distance:.1f}cm",
                        (cx-40, max(20, cy-30)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
            cv2.putText(display,
                        f"F{fase}({intento}) | Alt:{height} | d:{distance:.0f} | ex:{err_x} ey:{err_y} ang:{angle_y:.1f}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255,255,0), 2)

            print_throttle(f"Dist:{distance:.1f} Lat:{lateral_real_cm:.1f} F:{fase} "
                           f"Alt:{height} ex:{err_x} ey:{err_y} ang:{angle_y:.1f}")

            # ── FASE 1: acercarse a TARGET_DIST ─────────────────────────
            if fase == 1:
                err_dist = distance - TARGET_DIST
                fb       = int(np.clip(0.25 * err_dist, -20, 20))
                yaw_f1   = int(np.clip(0.10 * err_x, -15, 15))
                if abs(err_dist) < 15:
                    drone.send_control(0, 0, 0, 0)
                    print("Distancia alcanzada — iniciando centrado")
                    intento = 0
                    fase = 2
                else:
                    drone.send_control(0, fb, ud_loop, yaw_f1)

            # ── FASE 2: estabilizar antes de medir ──────────────────────
            elif fase == 2:
                cv2.putText(display, f"Estabilizando... (intento {intento+1})",
                            (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,165,0), 2)
                out.write(display)
                cv2.imshow("Tello Vision", display)
                cv2.waitKey(1)
                drone.send_control(0, 0, 0, 0)
                sleep_con_altura(1.5)
                fase = 3

            # ── FASE 3: medir con mediana ────────────────────────────────
            elif fase == 3:
                cv2.putText(display, "Midiendo...",
                            (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,165,0), 2)
                out.write(display)
                cv2.imshow("Tello Vision", display)
                cv2.waitKey(1)

                resultado = medir_frames(20)
                if resultado is None:
                    # No pudo medir — buscar marcador
                    print("No se pudo medir — buscando marcador")
                    fase = 1
                else:
                    avg_x, avg_y, avg_ang, avg_lat = resultado
                    print(f"Medición [{intento+1}] -> ex:{avg_x:.1f}px ey:{avg_y:.1f}px "
                          f"ang:{avg_ang:.1f}° lat:{avg_lat:.1f}cm")
                    fase = 4

            # ── FASE 4: corregir lateral (desplazamiento físico lr) ──────
            elif fase == 4:
                cv2.putText(display, f"Corr. lateral lat:{avg_lat:.1f}cm ex:{avg_x:.1f}px",
                            (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0,200,255), 2)
                out.write(display)
                cv2.imshow("Tello Vision", display)
                cv2.waitKey(1)

                # Usar avg_lat (tvec X) como fuente principal de desplazamiento real.
                # avg_lat > 0 → marcador a la derecha → lr positivo (dron va derecha)
                # avg_lat < 0 → marcador a la izquierda → lr negativo (dron va izquierda)
                # Si avg_lat es pequeño pero err_x es grande, usar err_x como respaldo.
                corr_lat = avg_lat
                if abs(corr_lat) < 10 and abs(avg_x) > TOL_X_PX:
                    corr_lat = avg_x * 0.10   # px → cm aproximado

                if abs(corr_lat) > 8:
                    lr_fix = int(np.clip(corr_lat * 0.45, -30, 30))
                    dur_lr = min(abs(corr_lat) / 22.0, 4.0)
                    print(f"  lr={lr_fix} dur={dur_lr:.2f}s (corr_lat={corr_lat:.1f}cm)")
                    mover_con_altura(lr_fix, 0, 0, dur_lr)
                else:
                    print(f"  Lateral ok ({corr_lat:.1f}cm)")

                fase = 5

            # ── FASE 5: corregir angular (yaw sobre sí mismo) ────────────
            elif fase == 5:
                cv2.putText(display, f"Corr. yaw ang:{avg_ang:.1f}° ex:{avg_x:.1f}px",
                            (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0,200,255), 2)
                out.write(display)
                cv2.imshow("Tello Vision", display)
                cv2.waitKey(1)

                # Corrección angular: usa avg_ang como referencia principal.
                # Si avg_ang es pequeño pero err_x sigue siendo grande,
                # corregir también con err_x.
                corr_yaw = avg_ang
                if abs(corr_yaw) < TOL_ANG and abs(avg_x) > TOL_X_PX:
                    corr_yaw = avg_x * 0.08

                if abs(corr_yaw) > TOL_ANG:
                    yaw_fix = int(np.clip(corr_yaw * 0.35, -22, 22))
                    dur_yaw = min(abs(corr_yaw) / 150.0, 2.5)
                    print(f"  yaw={yaw_fix} dur={dur_yaw:.2f}s (corr_yaw={corr_yaw:.1f}°)")
                    mover_con_altura(0, 0, yaw_fix, dur_yaw)
                else:
                    print(f"  Yaw ok ({corr_yaw:.1f}°)")

                fase = 6

            # ── FASE 6: verificar — medir de nuevo y decidir ─────────────
            elif fase == 6:
                cv2.putText(display, "Verificando...",
                            (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,128), 2)
                out.write(display)
                cv2.imshow("Tello Vision", display)
                cv2.waitKey(1)

                resultado = medir_frames(15)
                if resultado is None:
                    print("Verificación sin marcador — reintentando desde fase 1")
                    intento += 1
                    fase = 1
                else:
                    vx, vy, vang, vlat = resultado
                    print(f"Verificación [{intento+1}] -> ex:{vx:.1f} ang:{vang:.1f}°")

                    centrado_x   = abs(vx)   <= TOL_X_PX
                    centrado_ang = abs(vang) <= TOL_ANG

                    if centrado_x and centrado_ang:
                        print("✓ Centrado correcto — retrocediendo")
                        fase = 7
                    elif intento + 1 >= MAX_INTENTOS_CENTRADO:
                        print("Máximo de intentos alcanzado — retrocediendo igualmente")
                        fase = 7
                    else:
                        print(f"No centrado (ex:{vx:.1f} ang:{vang:.1f}) — reintentando")
                        # Actualizar promedios con la nueva medición para corregir mejor
                        avg_x, avg_y, avg_ang, avg_lat = vx, vy, vang, vlat
                        intento += 1
                        fase = 4   # volver directo a corrección (no hace falta estabilizar de nuevo)

            # ── FASE 7: retroceder ───────────────────────────────────────
            elif fase == 7:
                cv2.putText(display, "Retrocediendo...",
                            (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,165,0), 2)
                out.write(display)
                cv2.imshow("Tello Vision", display)
                cv2.waitKey(1)
                mover_con_altura(0, -20, 0, 4.0)
                fase = 8

            # ── FASE 8: aterrizar ────────────────────────────────────────
            elif fase == 8:
                print("Aterrizando...")
                drone.land()
                cleanup()
                exit()

    # ── Sin marcador en frame ────────────────────────────────────────────
    else:
        # Determinar dirección de búsqueda según el último lado conocido.
        # last_lado +1 = el marcador estaba a la derecha → girar derecha para encontrarlo
        # last_lado -1 = el marcador estaba a la izquierda → girar izquierda
        # last_lado  0 = nunca visto → girar derecha por defecto
        if last_lado == -1:
            yaw_recovery = -20
            lado_txt = "izq (último lado conocido)"
        else:
            yaw_recovery = 20
            lado_txt = "der (último lado conocido)"

        cv2.putText(display, f"NO ARUCO | buscando {lado_txt}",
                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0,0,255), 2)
        drone.send_control(0, 0, ud_loop, yaw_recovery)
        print_throttle(f"NO ARUCO | buscando {lado_txt}")

    out.write(display)
    cv2.imshow("Tello Vision", display)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        check_q()
    time.sleep(0.05)

cleanup()
