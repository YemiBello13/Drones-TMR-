import cv2
import numpy as np
from djitellopy import Tello
import time

# ===== CONFIGURACIÓN =====
ALTURA_ROJO    = 175
TOLERANCIA_ALT = 5
TOLERANCIA_X   = 25
TOLERANCIA_Y   = 20
EMERGENCY_CEIL = 200

# ===== ÁREA =====
AREA_MINIMA = 500
AREA_MAXIMA = 80000

# ===== RATIO =====
RATIO_MIN = 0.55
RATIO_MAX = 1.45

# ===== HSV ROJO — PVC rojo bajo luz artificial =====
rojo_bajo1 = np.array([0,   120, 80])
rojo_alto1 = np.array([15,  255, 255])
rojo_bajo2 = np.array([155, 120, 80])
rojo_alto2 = np.array([180, 255, 255])

# ===== VARIABLES DE BÚSQUEDA =====
FRAMES_SIN_TUNEL_MAX = 30
frames_sin_tunel     = 0
sentido_giro         = 1
giros_realizados     = 0
MAX_GIROS            = 12


def detectar_gate_rojo(frame):
    """
    Detección por color rojo + forma cuadrada.
    No exige vértices exactos — funciona con PVC redondeado.
    """
    if frame is None:
        return None, 0, None

    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

    mask = cv2.add(
        cv2.inRange(hsv, rojo_bajo1, rojo_alto1),
        cv2.inRange(hsv, rojo_bajo2, rojo_alto2)
    )

    kernel = np.ones((7, 7), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=3)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel, iterations=1)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    mejor_area   = 0
    mejor_centro = None
    mejor_bbox   = None

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

        if area > mejor_area:
            M = cv2.moments(c)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                mejor_area   = area
                mejor_centro = (cx, cy)
                mejor_bbox   = (x, y, bw, bh)

    return mejor_centro, mejor_area, mejor_bbox


def aterrizaje_suave(tello):
    """Descenso gradual en 3 velocidades según altura."""
    print("Iniciando descenso suave...")
    while True:
        h = tello.get_height()
        if h > 120:
            tello.send_rc_control(0, 0, -30, 0)
        elif h > 60:
            tello.send_rc_control(0, 0, -20, 0)
        elif h > 30:
            tello.send_rc_control(0, 0, -10, 0)
        else:
            tello.send_rc_control(0, 0, 0, 0)
            time.sleep(0.3)
            tello.land()
            print("✓ Aterrizaje suave completado")
            break
        time.sleep(0.15)


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
video_name = f"tunel_rojo_{int(time.time())}.avi"
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out    = cv2.VideoWriter(video_name, fourcc, 20.0, (w, h))

fase   = 1
height = 0

try:
    print("FASE 1: Despegue")
    print("─────────────────────────────────────────")
    print("  Q       → Emergencia (corta motores)")
    print("  ESPACIO → Aterrizaje suave manual")
    print(f"  AUTO    → Techo {EMERGENCY_CEIL}cm")
    print("─────────────────────────────────────────")
    tello.takeoff()
    time.sleep(2)
    fase = 2
    print("FASE 2: Ajustando altura (175cm)")

    while True:

        # ── 1. LEER FRAME ──────────────────────────────
        img = frame_read.frame
        if img is None:
            cv2.waitKey(1)
            continue

        frame_display = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        # ── 2. MOSTRAR IMAGEN (mantiene buffer activo) ─
        cv2.imshow("TMR - GATE ROJO", frame_display)
        key = cv2.waitKey(1) & 0xFF

        # ── 3. CONTROLES DE TECLADO ────────────────────
        if key == ord('q'):
            print("EMERGENCIA (Q) — Motores cortados")
            tello.emergency()
            break

        if key == ord(' '):
            print("Aterrizaje manual (ESPACIO)")
            tello.send_rc_control(0, 0, 0, 0)
            time.sleep(0.2)
            aterrizaje_suave(tello)
            break

        # ── 4. TECHO DE EMERGENCIA ─────────────────────
        height = tello.get_height()
        if height >= EMERGENCY_CEIL:
            print(f"TECHO DE EMERGENCIA: {height}cm — Aterrizando")
            tello.send_rc_control(0, 0, 0, 0)
            time.sleep(0.2)
            aterrizaje_suave(tello)
            break

        # ── 5. DETECCIÓN ───────────────────────────────
        pos_gate, area_gate, bbox_gate = detectar_gate_rojo(img)

        # ── 6. MÁQUINA DE FASES ────────────────────────

        # FASE 2: Alcanzar 175cm
        if fase == 2:
            if height < ALTURA_ROJO - TOLERANCIA_ALT:
                tello.send_rc_control(0, 0, 30, 0)
            elif height > ALTURA_ROJO + TOLERANCIA_ALT:
                tello.send_rc_control(0, 0, -20, 0)
            else:
                tello.send_rc_control(0, 0, 0, 0)
                print("✓ Altura lista. FASE 3: Buscando gate rojo")
                fase = 3

        # FASE 3: Búsqueda giratoria + centrado
        elif fase == 3:
            if pos_gate:
                frames_sin_tunel = 0
                cx, cy = pos_gate
                error_x = cx - centro_x
                error_y = cy - centro_y

                cv2.line(frame_display, (centro_x, 0), (centro_x, h), (255, 255, 0), 1)
                cv2.line(frame_display, (0, centro_y), (w, centro_y), (255, 255, 0), 1)

                if abs(error_x) > TOLERANCIA_X:
                    vel_lat = 15 if error_x > 0 else -15
                    tello.send_rc_control(vel_lat, 0, 0, 0)
                    cv2.putText(frame_display,
                                f"Centrando X err={error_x}px",
                                (20, 70), 2, 0.65, (0, 165, 255), 2)

                elif abs(error_y) > TOLERANCIA_Y:
                    vel_v = -15 if error_y > 0 else 15
                    tello.send_rc_control(0, 0, vel_v, 0)
                    cv2.putText(frame_display,
                                f"Ajustando Y err={error_y}px",
                                (20, 70), 2, 0.65, (255, 165, 0), 2)

                else:
                    tello.send_rc_control(0, 0, 0, 0)
                    print("✓ CENTRADO COMPLETO — FASE 4: Cruzar")
                    fase = 4

            else:
                # No detecta → girar buscando
                frames_sin_tunel += 1
                tello.send_rc_control(0, 0, 0, 30 * sentido_giro)

                if frames_sin_tunel % FRAMES_SIN_TUNEL_MAX == 0:
                    giros_realizados += 1
                    print(f"  Buscando... giro {giros_realizados}/{MAX_GIROS}")
                    if giros_realizados == MAX_GIROS // 2:
                        sentido_giro *= -1
                        print("  Cambiando sentido de búsqueda")
                    if giros_realizados >= MAX_GIROS:
                        print("No encontrado tras búsqueda completa. Aterrizando.")
                        fase = 5

                cv2.putText(frame_display, "Buscando gate rojo...",
                            (20, 70), 2, 0.65, (0, 0, 255), 2)

        # FASE 4: Cruzar el túnel
        elif fase == 4:
            print("FASE 4: Cruzando túnel (300cm)...")
            tello.send_rc_control(0, 0, 0, 0)
            time.sleep(0.3)
            tello.move_forward(300)
            print("✓ Túnel cruzado. FASE 5: Aterrizaje")
            fase = 5

        # FASE 5: Aterrizaje suave
        elif fase == 5:
            tello.send_rc_control(0, 0, 0, 0)
            time.sleep(0.3)
            aterrizaje_suave(tello)
            break

        # ── 7. HUD SOBRE FRAME ─────────────────────────
        if bbox_gate is not None:
            bx, by, bw, bh = bbox_gate
            cv2.rectangle(frame_display,
                          (bx, by), (bx + bw, by + bh),
                          (0, 0, 255), 3)
            cv2.circle(frame_display, pos_gate, 8, (0, 255, 255), -1)
            cv2.putText(frame_display,
                        f"GATE OK | Area:{int(area_gate)}",
                        (bx, by - 10), 2, 0.6, (0, 255, 0), 2)

        cv2.drawMarker(frame_display, (centro_x, centro_y),
                       (255, 255, 0), cv2.MARKER_CROSS, 25, 2)

        cv2.putText(frame_display,
                    f"FASE:{fase} | ALT:{height}cm | BAT:{tello.get_battery()}%",
                    (20, 40), 2, 0.7, (0, 255, 0), 2)

        cv2.putText(frame_display, "Q=EMERGENCIA | SPACE=ATERRIZAR",
                    (20, h - 20), 2, 0.5, (0, 255, 255), 1)

        if height >= EMERGENCY_CEIL - 20:
            cv2.putText(frame_display,
                        f"CERCA TECHO: {height}/{EMERGENCY_CEIL}cm",
                        (20, h - 50), 2, 0.6, (0, 0, 255), 2)

        # ── 8. ACTUALIZAR VENTANA + GRABAR ────────────
        cv2.imshow("TMR - GATE ROJO", frame_display)
        out.write(frame_display)

except Exception as e:
    print(f"ERROR CRITICO: {e}")
    try:
        aterrizaje_suave(tello)
    except Exception:
        pass

finally:
    print(f"Guardando: {video_name}")
    out.release()
    tello.streamoff()
    cv2.destroyAllWindows()
