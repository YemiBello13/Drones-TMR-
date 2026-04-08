import cv2
import numpy as np
from djitellopy import Tello
import time

# ===== CONFIGURACIÓN DE MISIÓN (3 PUNTOS) =====
ALTURA_ROJO    = 175  # 1.5m base + 0.25m centro ventana
EMERGENCY_CEIL = 200 
TOLERANCIA_ALT = 5   
TOLERANCIA_X   = 25  
AREA_MINIMA    = 6000 # Para no confundirse con ruidos lejanos

# ===== RANGOS HSV (MÁS TOLERANTES AL NARANJA/CAFÉ) =====
rojo_bajo1 = np.array([0, 40, 40]);   rojo_alto1 = np.array([25, 255, 255])
rojo_bajo2 = np.array([155, 40, 40]); rojo_alto2 = np.array([180, 255, 255])

def obtener_centro_color(frame):
    if frame is None: return None, 0
    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
    mask = cv2.add(cv2.inRange(hsv, rojo_bajo1, rojo_alto1), 
                    cv2.inRange(hsv, rojo_bajo2, rojo_alto2))
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
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

tello = Tello()
tello.connect()
print(f"BATERÍA: {tello.get_battery()}%")
tello.streamon()
frame_read = tello.get_frame_read()

# --- ESPERAR EL PRIMER FRAME PARA AJUSTAR EL VIDEO ---
print("Esperando cámara...")
while frame_read.frame is None: time.sleep(0.1)
primer_frame = frame_read.frame
h, w, _ = primer_frame.shape

# --- CONFIGURACIÓN DE VIDEO DINÁMICA ---
video_name = f"mision_05m_{int(time.time())}.avi"
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter(video_name, fourcc, 20.0, (w, h))

fase = 1 
centro_x = w // 2

try:
    print("FASE 1: Despegue")
    tello.takeoff()
    time.sleep(2)
    fase = 2

    while True:
        img = frame_read.frame
        if img is None: continue
        
        # Copia para procesar y mostrar (Colores corregidos BGR)
        frame_display = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord(' '): tello.emergency(); break
        if key == ord('q'): break

        height = tello.get_height()
        pos_rojo, area_roja = obtener_centro_color(img)

        # FASE 2: Alcanzar altura (175cm)
        if fase == 2:
            if height < ALTURA_ROJO - TOLERANCIA_ALT:
                tello.send_rc_control(0, 0, 30, 0)
            elif height > ALTURA_ROJO + TOLERANCIA_ALT:
                tello.send_rc_control(0, 0, -20, 0)
            else:
                tello.send_rc_control(0, 0, 0, 0)
                print("Altura lista. Iniciando FASE 3: Búsqueda y Centrado")
                fase = 3

        # FASE 3: Búsqueda (Girar) y Centrado (Lateral)
        elif fase == 3:
            if pos_rojo:
                cx, cy = pos_rojo
                error_x = cx - centro_x
                if abs(error_x) > TOLERANCIA_X:
                    vel_lat = 15 if error_x > 0 else -15
                    tello.send_rc_control(vel_lat, 0, 0, 0)
                else:
                    tello.send_rc_control(0, 0, 0, 0)
                    print("!!! CENTRADO COMPLETO !!!")
                    fase = 4
            else:
                # SI NO ENCUENTRA EL COLOR ROJO -> Gira para buscarlo
                tello.send_rc_control(0, 0, 0, 30)

        # FASE 4: Cruzar túnel
        elif fase == 4:
            tello.move_forward(300)
            fase = 5

        # FASE 5: Aterrizaje suave
        elif fase == 5:
            print("Bajando...")
            while tello.get_height() > 40:
                tello.send_rc_control(0, 0, -20, 0)
                time.sleep(0.2)
            tello.land()
            break

        # --- DIBUJO Y GRABACIÓN (Solo si frame_display es válido) ---
        if frame_display is not None:
            if pos_rojo:
                cv2.circle(frame_display, pos_rojo, 15, (0, 0, 255), -1)
            
            cv2.putText(frame_display, f"FASE: {fase} | ALT: {height}cm", (20, 40), 2, 0.7, (0, 255, 0), 2)
            
            # Grabamos el frame tal cual se ve en pantalla
            out.write(frame_display)
            cv2.imshow("TMR - MISION 1 ROJO", frame_display)

except Exception as e:
    print(f"ERROR CRÍTICO: {e}")
    tello.land()

finally:
    print(f"Guardando video: {video_name}")
    out.release()
    tello.streamoff()
    cv2.destroyAllWindows()
