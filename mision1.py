import cv2
import numpy as np
from djitellopy import Tello
import time

# ===== CONFIGURACIÓN DE MISIÓN =====
ALTURA_ROJO  = 175  # Centro de la ventana pequeña (1.5m base + 0.25m ventana)
EMERGENCY_CEIL = 200 # Altura máxima de seguridad (3 metros)
TOLERANCIA_ALT = 5   # Tolerancia de altura en cm
TOLERANCIA_X   = 20  # Tolerancia de centrado horizontal en píxeles

# ===== RANGOS HSV PARA ROJO (Ajustado para mayor sensibilidad) =====
rojo_bajo1 = np.array([0, 100, 100]);  rojo_alto1 = np.array([10, 255, 255])
rojo_bajo2 = np.array([160, 100, 100]); rojo_alto2 = np.array([180, 255, 255])

def obtener_centro_color(frame):
    """Devuelve las coordenadas (x, y) del centro de la masa roja más grande."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.add(cv2.inRange(hsv, rojo_bajo1, rojo_alto1), 
                    cv2.inRange(hsv, rojo_bajo2, rojo_alto2))
    
    # Limpiar ruido
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(c) > 5000: # Solo si el objeto es suficientemente grande
            M = cv2.moments(c)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                return (cx, cy), mask
    return None, mask

tello = Tello()
tello.connect()
print(f"Batería: {tello.get_battery()}%")
tello.streamon()
frame_read = tello.get_frame_read()

fase = 1 # 1:Takeoff, 2:Buscar/Subir, 3:Centrar, 4:Cruzar, 5:Land
centro_pantalla_x = 480 # La resolución del Tello es 960x720

try:
    print("FASE 1: Despegue")
    tello.takeoff()
    time.sleep(2)
    fase = 2

    while True:
        img = frame_read.frame
        if img is None: continue
        
        # Seguridad Manual (Espacio para emergencia, Q para aterrizar)
        key = cv2.waitKey(1) & 0xFF
        if key == ord(' '): tello.emergency(); break
        if key == ord('q'): tello.land(); break

        height = tello.get_height()
        pos_rojo, mascara = obtener_centro_color(img)

        # FASE 2: Alcanzar altura objetivo
        if fase == 2:
            if height < ALTURA_ROJO - TOLERANCIA_ALT:
                tello.send_rc_control(0, 0, 30, 0) # Subir
            elif height > ALTURA_ROJO + TOLERANCIA_ALT:
                tello.send_rc_control(0, 0, -20, 0) # Bajar
            else:
                tello.send_rc_control(0, 0, 0, 0)
                print("FASE 3: Centrando dron con el túnel rojo...")
                fase = 3

        # FASE 3: Centrado Horizontal (Visual Servoing)
        # Esto es vital para el túnel de 0.5m
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
                # Si pierde el color, rotar un poco para buscarlo
                tello.send_rc_control(0, 0, 0, 15)

        # FASE 4: Avance de precisión
        elif fase == 4:
            # Los 5 aros forman un túnel de 2m. Avanzamos 2.5m para salir limpio.
            tello.move_forward(250)
            fase = 5

        # FASE 5: Aterrizaje
        elif fase == 5:
            print("Misión terminada exitosamente.")
            tello.land()
            break

        # Dibujar UI para telemetría
        cv2.circle(img, (centro_pantalla_x, 360), 10, (255, 255, 255), 2)
        if pos_rojo:
            cv2.circle(img, pos_rojo[0], 10, (0, 0, 255), -1)
        
        cv2.putText(img, f"Fase: {fase} | Alt: {height}cm", (20, 50), 2, 0.8, (0, 255, 0), 2)
        cv2.imshow("TMR - MISION 1 ROJO", img)

except Exception as e:
    print(f"Error: {e}")
    tello.land()
finally:
    tello.streamoff()
    cv2.destroyAllWindows()
