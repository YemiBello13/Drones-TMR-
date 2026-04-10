import cv2
import numpy as np
from djitellopy import Tello
import time

# ===== CONFIGURACIÓN TÚNEL PEQUEÑO (0.5m x 0.5m, base 1.5m) =====
ALTURA_ROJO    = 175   # cm — base 150cm + mitad ventana 25cm
TOLERANCIA_ALT = 5
TOLERANCIA_X   = 15
TOLERANCIA_Y   = 15
AREA_MINIMA    = 800
AREA_MAXIMA    = 8000

# ===== TECHO DE EMERGENCIA AUTOMÁTICO =====
EMERGENCY_CEIL = 200   # Si supera 200cm → aterrizaje automático

# ===== RATIO CUADRADO =====
RATIO_MIN = 0.65
RATIO_MAX = 1.35

# ===== RANGOS HSV — ROJO PURO =====
rojo_bajo1 = np.array([0,   100, 100])
rojo_alto1 = np.array([10,  255, 255])
rojo_bajo2 = np.array([160, 100, 100])
rojo_alto2 = np.array([180, 255, 255])

# ===== VARIABLES DE BÚSQUEDA =====
FRAMES_SIN_TUNEL_MAX = 30
frames_sin_tunel     = 0
sentido_giro         = 1
giros_realizados     = 0
MAX_GIROS            = 12

def detectar_tunel_cuadrado_rojo(frame):
    if frame is None:
        return None, 0, None

    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
    mask = cv2.add(
        cv2.inRange(hsv, rojo_bajo1, rojo_alto1),
        cv2.inRange(hsv, rojo_bajo2, rojo_alto2)
    )
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask,  kernel, iterations=2)
    mask = cv2.dilate(mask, kernel, iterations=3)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    mejor_contorno = None
    mejor_area     = 0
    mejor_centro   = None

    for c in contours:
        area = cv2.contourArea(c)
        if area < AREA_MINIMA or area > AREA_MAXIMA:
            continue

        x, y, bw, bh = cv2.boundingRect(c)
        if bh == 0:
            continue
        ratio = bw / bh
        if not (RATIO_MIN <= ratio <= RATIO_MAX):
            continue

        hull      = cv2.convexHull(c)
        hull_area = cv2.contourArea(hull)
        if hull_area == 0:
            continue
        solidez = area / hull_area
        if solidez < 0.6:
            continue

        perimetro = cv2.arcLength(c, True)
        approx    = cv2.approxPolyDP(c, 0.04 * perimetro, True)
        if not (4 <= len(approx) <= 6):
            continue

        if area > mejor_area:
            M = cv2.moments(c)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                mejor_area     = area
                mejor_centro   = (cx, cy)
                mejor_contorno = approx

    return mejor_centro, mejor_area, mejor_contorno


# ===== CONEXIÓN Y STREAM =====
tello = Tello()
tello.connect()
print(f"BATERÍA: {tello.get_battery()}%")
tello.streamon()
frame_read = tello.get_frame_read()

print("Esperando cámara...")
while frame_read.frame is None:
    time.sleep(0.1)

primer_frame = frame_read.frame
h, w, _ = primer_frame.shape
centro_x = w // 2
centro_y = h // 2

# ===== GRABACIÓN =====
video_name = f"tunel_pequeno_rojo_{int(time.time())}.avi"
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out    = cv2.VideoWriter(video_name, fourcc, 20.0, (w, h))

fase = 1

try:
    print("FASE 1: Despegue")
    print("─────────────────────────────────────────")
    print("  CONTROLES:")
    print("  Q        → Emergencia manual (corta motores)")
    print("  ESPACIO  → Salida limpia (aterrizaje suave)")
    print(f"  AUTO     → Techo {EMERGENCY_CEIL}cm (aterrizaje automático)")
    print("─────────────────────────────────────────")
    tello.takeoff()
    time.sleep(2)
    fase = 2
    print("FASE 2: Ajustando altura (175cm)")

    while True:
        img = frame_read.frame
        if img is None:
            continue

        frame_display = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        key = cv2.waitKey(1) & 0xFF

        # ── NIVEL 1: Emergencia manual con Q (corta motores al instante) ──
        if key == ord('q'):
            print("🛑 EMERGENCIA MANUAL (Q) — Motores cortados al instante")
            tello.emergency()
            break

        # ── NIVEL 2: Salida limpia con ESPACIO (aterrizaje suave) ──
        if key == ord(' '):
            print("🔽 SALIDA LIMPIA (ESPACIO) — Aterrizando suavemente")
            tello.send_rc_control(0, 0, 0, 0)
            time.sleep(0.2)
            tello.land()
            break

        height = tello.get_height()

        # ── NIVEL 3: Techo de emergencia automático ──
        if height >= EMERGENCY_CEIL:
            print(f"⚠ TECHO DE EMERGENCIA: {height}cm >= {EMERGENCY_CEIL}cm — Aterrizando!")
            tello.send_rc_control(0, 0, 0, 0)
            time.sleep(0.2)
            tello.land()
            break

        pos_tunel, area_tunel, contorno_tunel = detectar_tunel_cuadrado_rojo(img)

        # --------------------------------------------------
        # FASE 2: ALCANZAR ALTURA 175cm
        # --------------------------------------------------
        if fase == 2:
            if height < ALTURA_ROJO - TOLERANCIA_ALT:
                tello.send_rc_control(0, 0, 25, 0)
            elif height > ALTURA_ROJO + TOLERANCIA_ALT:
                tello.send_rc_control(0, 0, -20, 0)
            else:
                tello.send_rc_control(0, 0, 0, 0)
                print("✓ Altura lista. FASE 3: Buscando túnel cuadrado rojo")
                fase = 3

        # --------------------------------------------------
        # FASE 3: BÚSQUEDA GIRATORIA + CENTRADO X e Y
        # --------------------------------------------------
        elif fase == 3:
            if pos_tunel:
                frames_sin_tunel = 0
                cx, cy = pos_tunel
                error_x = cx - centro_x
                error_y = cy - centro_y

                cv2.line(frame_display, (centro_x, 0), (centro_x, h), (255, 255, 0), 1)
                cv2.line(frame_display, (0, centro_y), (w, centro_y), (255, 255, 0), 1)

                if abs(error_x) > TOLERANCIA_X:
                    vel_lat = 18 if error_x > 0 else -18
                    tello.send_rc_control(vel_lat, 0, 0, 0)
                    cv2.putText(frame_display,
                                f"Centrando X... err={error_x}px",
                                (20, 70), 2, 0.65, (0, 165, 255), 2)
                elif abs(error_y) > TOLERANCIA_Y:
                    vel_vert = -15 if error_y > 0 else 15
                    tello.send_rc_control(0, 0, vel_vert, 0)
                    cv2.putText(frame_display,
                                f"Ajustando Y... err={error_y}px",
                                (20, 70), 2, 0.65, (255, 165, 0), 2)
                else:
                    tello.send_rc_control(0, 0, 0, 0)
                    print("✓ CENTRADO COMPLETO (X e Y) — FASE 4: Cruzar túnel")
                    fase = 4

            else:
                frames_sin_tunel += 1
                tello.send_rc_control(0, 0, 0, 20 * sentido_giro)

                if frames_sin_tunel % FRAMES_SIN_TUNEL_MAX == 0:
                    giros_realizados += 1
                    print(f"  Buscando túnel... giro {giros_realizados}/{MAX_GIROS}")
                    if giros_realizados == MAX_GIROS // 2:
                        sentido_giro *= -1
                        print("  Cambiando sentido de búsqueda")
                    if giros_realizados >= MAX_GIROS:
                        print("⚠ Túnel no encontrado. Aterrizando.")
                        fase = 5

                cv2.putText(frame_display, "Buscando cuadrado rojo...",
                            (20, 70), 2, 0.65, (0, 0, 255), 2)

        # --------------------------------------------------
        # FASE 4: CRUZAR TÚNEL
        # --------------------------------------------------
        elif fase == 4:
            print("FASE 4: Cruzando túnel pequeño (300cm)...")
            tello.send_rc_control(0, 0, 0, 0)
            time.sleep(0.3)
            tello.move_forward(300)
            print("✓ Túnel cruzado. FASE 5: Aterrizaje")
            fase = 5

        # --------------------------------------------------
        # FASE 5: ATERRIZAJE
        # --------------------------------------------------
        elif fase == 5:
            print("FASE 5: Aterrizando...")
            while tello.get_height() > 40:
                tello.send_rc_control(0, 0, -20, 0)
                time.sleep(0.2)
            tello.land()
            break

        # --------------------------------------------------
        # HUD + DIBUJO + GRABACIÓN
        # --------------------------------------------------
        if frame_display is not None:
            if pos_tunel and contorno_tunel is not None:
                cv2.drawContours(frame_display, [contorno_tunel], -1, (0, 0, 255), 3)
                cv2.circle(frame_display, pos_tunel, 8, (0, 255, 255), -1)
                cv2.putText(frame_display, f"Area:{int(area_tunel)}",
                            (pos_tunel[0]+15, pos_tunel[1]),
                            2, 0.55, (0, 0, 255), 1)
                cv2.putText(frame_display, "TUNEL ROJO OK",
                            (pos_tunel[0]-40, pos_tunel[1]-20),
                            2, 0.6, (0, 255, 0), 2)

            # Retícula central
            cv2.drawMarker(frame_display, (centro_x, centro_y),
                           (255, 255, 0), cv2.MARKER_CROSS, 25, 2)

            # HUD principal
            cv2.putText(frame_display,
                        f"FASE:{fase} | ALT:{height}cm | BAT:{tello.get_battery()}%",
                        (20, 40), 2, 0.7, (0, 255, 0), 2)

            # HUD de controles (esquina inferior izquierda)
            cv2.putText(frame_display, "Q=EMERGENCIA | SPACE=ATERIZAR",
                        (20, h - 20), 2, 0.5, (0, 255, 255), 1)

            # Advertencia si se acerca al techo
            if height >= EMERGENCY_CEIL - 20:
                cv2.putText(frame_display,
                            f"⚠ CERCA DEL TECHO: {height}cm/{EMERGENCY_CEIL}cm",
                            (20, h - 50), 2, 0.6, (0, 0, 255), 2)

            out.write(frame_display)
            cv2.imshow("TMR - TUNEL PEQUENO ROJO", frame_display)

except Exception as e:
    print(f"ERROR CRÍTICO: {e}")
    tello.land()

finally:
    print(f"Guardando: {video_name}")
    out.release()
    tello.streamoff()
    cv2.destroyAllWindows()
