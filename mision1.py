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

# ===== ÁREA — ajustada para PVC rojo desde distancia =====
AREA_MINIMA = 500    # ← Muy reducida, el gate se ve pequeño de lejos
AREA_MAXIMA = 80000  # ← Amplia para cuando esté cerca

# ===== RATIO — más tolerante para PVC con esquinas redondeadas =====
RATIO_MIN = 0.55
RATIO_MAX = 1.45

# ===== HSV ROJO — calibrado para PVC rojo brillante bajo luz artificial =====
# El PVC rojo en interiores con luz fluorescente tiende a ser rojo-naranja
rojo_bajo1 = np.array([0,   120, 80])   # H=0  rojo puro
rojo_alto1 = np.array([15,  255, 255])  # H=15 incluye rojo-naranja del PVC

rojo_bajo2 = np.array([155, 120, 80])   # H=155 rojo frío
rojo_alto2 = np.array([180, 255, 255])  # H=180

# ===== VARIABLES DE BÚSQUEDA =====
FRAMES_SIN_TUNEL_MAX = 30
frames_sin_tunel     = 0
sentido_giro         = 1
giros_realizados     = 0
MAX_GIROS            = 12

def detectar_gate_rojo(frame):
    """
    Detección simplificada — NO exige vértices exactos.
    Solo requiere: color rojo + forma aproximadamente cuadrada.
    Funciona con PVC de esquinas redondeadas.
    """
    if frame is None:
        return None, 0, None

    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

    mask = cv2.add(
        cv2.inRange(hsv, rojo_bajo1, rojo_alto1),
        cv2.inRange(hsv, rojo_bajo2, rojo_alto2)
    )

    # Morfología más agresiva para cerrar las esquinas del PVC
    kernel = np.ones((7, 7), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=3)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel, iterations=1)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    mejor_contorno = None
    mejor_area     = 0
    mejor_centro   = None
    mejor_bbox     = None

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

        # ── Sin exigir vértices: solo color + ratio cuadrado ──
        if area > mejor_area:
            M = cv2.moments(c)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                mejor_area     = area
                mejor_centro   = (cx, cy)
                mejor_contorno = c
                mejor_bbox     = (x, y, bw, bh)

    return mejor_centro, mejor_area, mejor_bbox


def aterrizaje_suave(tello):
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


# ===== CONEXIÓN =====
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

fase = 1

try:
    print("FASE 1: Despegue")
    print("─────────────────────────────────────────")
    print("  Q       → Emergencia (corta motores)")
    print("  ESPACIO → Aterrizaje suave")
    print(f"  AUTO    → Techo {EMERGENCY_CEIL}cm")
    print("─────────────────────────────────────────")
    tello.takeoff()
    time.sleep(2)
    fase = 2
    print("FASE 2: Ajustando altura (175cm)")

    while True:
        # ── Siempre leer frame primero ──
        img = frame_read.frame
        if img is None:
            cv2.waitKey(1)
            continue

        frame_display = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        # ── Mostrar imagen SIEMPRE (mantiene el buffer activo) ──
        cv2.imshow("TMR - GATE ROJO", frame_display)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            print("🛑 EMERGENCIA (Q)")
            tello.emergency()
            break

        if key == ord(' '):
            print("🔽 Aterrizaje manual")
            tello.send_rc_control(0, 0, 0, 0)
            time.sleep(0.2)
            aterrizaje_suave(tello)
            break

        height = tello.get_height()

        if height >= EMERGENCY_CEIL:
            print(f"⚠ TECHO: {height}cm — Aterrizando")
            tello.send_rc_control(0, 0, 0, 0)
            time.sleep(0.2)
            aterrizaje_suave(tello)
            break

        pos_gate, area_gate, bbox_gate = detectar_gate_rojo(img)

        # --------------------------------------------------
        # FASE 2: ALTURA 175cm
        # --------------------------------------------------
        if fase == 2:
            if height < ALTURA_ROJO - TOLERANCIA_ALT:
                tello.send_rc_control(0, 0, 30, 0)
            elif height > ALTURA_ROJO + TOLERANCIA_ALT:
                tello.send_rc_control(0, 0, -20, 0)
            else:
                tello.send_rc_control(0, 0, 0, 0)
                print("✓ Altura lista. FASE 3: Buscando gate rojo")
                fase = 3

        # --------------------------------------------------
        # FASE 3: BÚSQUEDA + CENTRADO
        # --------------------------------------------------
        elif fase == 3:
            if pos_gate:
                frames_sin_tunel = 0
                cx, cy = pos_gate
                error_x = cx - centro_x
                error_y = cy - centro_y

                cv2.line(frame_display, (centro_x, 0), (centro_x, h), (255,255,0), 1)
                cv2.line(frame_display, (0, centro_y), (w, centro_y), (255,255,0), 1)

                if abs(error_x) > TOLERANCIA_X:
                    vel_lat = 15 if error_x > 0 else -
