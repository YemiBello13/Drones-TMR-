import cv2
import numpy as np
from djitellopy import Tello
import time
import os

# ===== CONFIGURACIÓN DE MISIÓN =====
ALTURA_ROJO  = 175  
EMERGENCY_CEIL = 200 
TOLERANCIA_ALT = 5   
TOLERANCIA_X   = 20  

# ===== RANGOS HSV PARA ROJO/NARANJA/CAFÉ =====
rojo_bajo1 = np.array([0, 40, 40])
rojo_alto1 = np.array([25, 255, 255])
rojo_bajo2 = np.array([155, 40, 40])
rojo_alto2 = np.array([180, 255, 255])

def obtener_centro_color(frame):
    # El Tello manda RGB, OpenCV usa BGR para procesar
    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
    
    mask = cv2.add(cv2.inRange(hsv, rojo_bajo1, rojo_alto1), 
                    cv2.inRange(hsv, rojo_bajo2, rojo_alto2))
    
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(c) > 5000:
            M = cv2.moments(c)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                return (cx, cy), mask
    return None, mask

tello = Tello()

def conectar_dron():
    intentos = 0
    while intentos < 3:
        try:
            tello.connect()
            print(f"Conectado! Batería: {tello.get_battery()}%")
            return True
        except Exception as e:
            print(f"Intento {intentos+1} fallido. Reintentando...")
            intentos += 1
            time.sleep(2)
    return False

if not conectar_dron():
    print("Error de conexión.")
    exit()

tello.streamon()
frame_read = tello.get_frame_read()

# --- CONFIGURACIÓN DEL GRABADOR DE VIDEO ---
# Obtenemos el ancho y alto del frame del Tello (960x720 por defecto)
width, height_frame = 960, 720
video_name = f"vuelo_mision1_{int(time.time())}.avi"
# Definimos el codec y creamos el objeto VideoWriter
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter(video_name, fourcc, 20.0, (width, height_frame))

fase = 1 
centro_pantalla_x = 480 

try:
    print("FASE 1: Despegue")
    tello.takeoff()
    time.sleep(2)
    fase = 2

    while True:
        img = frame_read.frame
        if img is None: continue
        
        # Convertimos RGB a BGR para que el video guardado y la pantalla tengan colores reales
        frame_display = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord(' '): tello.emergency(); break
        if key == ord('q'): tello.land(); break

        height = tello.get_height()
        pos_rojo, mascara = obtener_centro_color(img)

        # FASE 2: Alcanzar altura objetivo
        if fase == 2:
            if height < ALTURA_ROJO - TOLERANCIA_ALT:
                tello.send_rc_control(0, 0, 30, 0) 
            elif height > ALTURA_ROJO + TOLERANCIA_ALT:
                tello.send_rc_control(0, 0, -20, 0) 
            else:
                tello.send_rc_control(0, 0, 0, 0)
                print("FASE 3: Centrando...")
                fase = 3

        # FASE 3: Centrado Horizontal Visual
        elif fase == 3:
            if pos_rojo:
                cx, cy = pos_rojo
                error_x = cx - centro_pantalla_x
                
                if abs(error_x) > TOLERANCIA_X:
                    velocidad_lat = 15 if error_x > 0 else -15
                    tello.send_rc_control(velocidad_lat, 0, 0, 0)
                else:
                    tello.send_rc_control(0, 0, 0, 0)
                    print("Centrado listo. FASE 4: Cruzando...")
                    fase = 4
            else:
                tello.send_rc_control(0, 0, 0, 15) # Buscar rotando

        # FASE 4: Cruzar el túnel
        elif fase == 4:
            tello.move_forward(250)
            fase = 5

        # FASE 5: Aterrizar
        elif fase == 5:
            print("Misión terminada.")
            tello.land()
            break

        # Telemetría visual en el frame
        if pos_rojo:
            cv2.circle(frame_display, pos_rojo, 10, (0, 0, 255), -1)
        
        cv2.putText(frame_display, f"Fase: {fase} | Alt: {height}cm", (20, 50), 2, 0.8, (0, 255, 0), 2)
        
        # --- GUARDAR FRAME EN EL VIDEO ---
        out.write(frame_display)
        
        # Mostrar en pantalla
        cv2.imshow("TMR - MISION 1 ROJO", frame_display)

except Exception as e:
    print(f"Error detectado: {e}")
    tello.land()
finally:
    # --- CERRAR RECURSOS ---
    print(f"Video guardado como: {video_name}")
    out.release() # Muy importante para que el video no se corrompa
    tello.streamoff()
    cv2.destroyAllWindows()
