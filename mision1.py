import cv2
import numpy as np
from djitellopy import Tello
import time

# ===== CONFIGURACIÓN TÚNEL PEQUEÑO (0.5m x 0.5m, base 1.5m) =====
ALTURA_ROJO    = 175   # cm — centro exacto de la ventana pequeña
EMERGENCY_CEIL = 200
TOLERANCIA_ALT = 5     # ±5cm
TOLERANCIA_X   = 15    # Ventana es solo 50cm de ancho
AREA_MINIMA    = 3000  

# ===== RANGOS HSV — SOLO ROJO PURO (sin naranja, sin café) =====
# Rojo "frío" (lado alto del espectro HSV — más fiable para gates rojos)
rojo_bajo1 = np.array([0,   100, 100])  # H=0,  S y V altos → rojo puro
rojo_alto1 = np.array([10,  255, 255])  # H=10  máximo

# Rojo "cálido" (lado bajo del espectro HSV)
rojo_bajo2 = np.array([160, 100, 100])  # H=160 → rojo intenso
rojo_alto2 = np.array([180, 255, 255])  # H=180 máximo

# ===== VARIABLES DE BÚSQUEDA =====
FRAMES_SIN_ROJO_MAX = 30
frames_sin_rojo     = 0
sentido_giro        = 1   # 1=derecha, -1=izquierda
giros_realizados    = 0
MAX_GIROS           = 12

def obtener_centro_rojo(frame):
    """Detecta SOLO el color rojo puro y retorna centroide y área."""
    if frame is None:
        return None, 0
    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

    # Combinar ambas máscaras de rojo
    mask = cv2.add(
        cv2.inRange(hsv, rojo_bajo1, rojo_alto1),
        cv2.inRange(hsv, rojo_bajo2, rojo_alto2)
    )

    # Morfología para eliminar ruido
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask,  kernel, iterations=2)
    mask = cv2.dilate(mask, kernel, iterations=3)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        c = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(c)
        if area > AREA_MINIMA:
            M = cv2.moments(c)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                return (cx, cy), area
    return None, 0

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
video_name = f"mision_tunel_pequeno_{int(time.time())}.avi"
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter(video_name, fourcc, 20.0, (w, h))

fase = 1

try:
    print("FASE 1: Despegue")
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
        if key == ord(' '):
            tello.emergency()
            break
        if key == ord('q'):
            break

        height             = tello.get_height()
        pos_rojo, area_roja = obtener_centro_rojo(img)

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
                print("✓ Altura lista. FASE 3: Búsqueda túnel rojo")
                fase = 3

        # --------------------------------------------------
        # FASE 3: BÚSQUEDA GIRATORIA + CENTRADO
        # --------------------------------------------------
        elif fase == 3:
            if pos_rojo:
                frames_sin_rojo = 0
                cx, cy = pos_rojo
                error_x = cx - centro_x

                cv2.line(frame_display, (centro_x, 0), (centro_x, h), (255, 255, 0), 1)
                cv2.line(frame_display, (cx, cy-20), (cx, cy+20), (0, 0, 255), 2)

                if abs(error_x) > TOLERANCIA_X:
                    vel_lat = 18 if error_x > 0 else -18
                    tello.send_rc_control(vel_lat, 0, 0, 0)
                    cv2.putText(frame_display, f"Centrando... err={error_x}px",
                                (20, 70), 2, 0.65, (0, 165, 255), 2)
                else:
                    tello.send_rc_control(0, 0, 0, 0)
                    print("✓ CENTRADO — FASE 4: Cruzar túnel")
                    fase = 4

            else:
                # ── No ve rojo → girar y buscar ──
                frames_sin_rojo += 1
                tello.send_rc_control(0, 0, 0, 20 * sentido_giro)

                if frames_sin_rojo % FRAMES_SIN_ROJO_MAX == 0:
                    giros_realizados += 1
                    print(f"  Buscando... giro {giros_realizados}/{MAX_GIROS}")
                    if giros_realizados == MAX_GIROS // 2:
                        sentido_giro *= -1
                        print("  Cambiando sentido de búsqueda")
                    if giros_realizados >= MAX_GIROS:
                        print("⚠ No encontrado. Aterrizando.")
                        fase = 5

                cv2.putText(frame_display, "Buscando tunel rojo...",
                            (20, 70), 2, 0.65, (0, 0, 255), 2)

        # --------------------------------------------------
        # FASE 4: CRUZAR TÚNEL
        # --------------------------------------------------
        elif fase == 4:
            print("FASE 4: Cruzando (300cm)...")
            tello.send_rc_control(0, 0, 0, 0)
            time.sleep(0.3)
            tello.move_forward(300)
            print("✓ Cruzado. FASE 5: Aterrizaje")
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
        # HUD + GRABACIÓN
        # --------------------------------------------------
        if frame_display is not None:
            if pos_rojo:
                cv2.circle(frame_display, pos_rojo, 12, (0, 0, 255), -1)
                cv2.putText(frame_display, f"Area:{int(area_roja)}",
                            (pos_rojo[0]+15, pos_rojo[1]), 2, 0.55, (0, 0, 255), 1)

            cv2.drawMarker(frame_display, (centro_x, centro_y),
                           (255, 255, 0), cv2.MARKER_CROSS, 20, 1)
            cv2.putText(frame_display,
                        f"FASE:{fase} | ALT:{height}cm | BAT:{tello.get_battery()}%",
                        (20, 40), 2, 0.7, (0, 255, 0), 2)

            out.write(frame_display)
            cv2.imshow("TMR - TUNEL PEQUENO ROJO", frame_display)

except Exception as e:
    print(f"ERROR CRÍTICO: {e}")
    tello.land()

finally:
    print(f"Guardando video: {video_name}")
    out.release()
    tello.streamoff()
    cv2.destroyAllWindows()
