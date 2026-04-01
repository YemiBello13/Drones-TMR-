import cv2
import numpy as np
from djitellopy import Tello
import time

# ===== CONFIGURACIÓN DE MISIÓN =====
# Alturas de los centros de las ventanas según las reglas
ALTURA_AZUL  = 125  # Ventana 1.5m, base 0.5m -> Centro 1.25m
ALTURA_VERDE = 150  # Ventana 1.0m, base 1.0m -> Centro 1.50m
ALTURA_ROJO  = 175  # Ventana 0.5m, base 1.5m -> Centro 1.75m

EMERGENCY_CEIL = 380 # Regla 13: 4 metros
TOLERANCIA_ALT = 10

# ===== RANGOS HSV (Corregidos para tus fotos) =====
azul_bajo = np.array([90, 45, 45]);   azul_alto = np.array([130, 255, 255])
verde_bajo = np.array([35, 40, 40]);  verde_alto = np.array([90, 255, 255])
# Rojo usa dos rangos por el círculo de color HSV
rojo_bajo1 = np.array([0, 70, 50]);   rojo_alto1 = np.array([15, 255, 255])
rojo_bajo2 = np.array([160, 70, 50]); rojo_alto2 = np.array([180, 255, 255])

def detectar_color(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask_azul = cv2.inRange(hsv, azul_bajo, azul_alto)
    mask_verde = cv2.inRange(hsv, verde_bajo, verde_alto)
    mask_rojo = cv2.add(cv2.inRange(hsv, rojo_bajo1, rojo_alto1), 
                        cv2.inRange(hsv, rojo_bajo2, rojo_alto2))

    pixels = {
        "AZUL": cv2.countNonZero(mask_azul),
        "VERDE": cv2.countNonZero(mask_verde),
        "ROJO": cv2.countNonZero(mask_rojo)
    }
    max_c = max(pixels, key=pixels.get)
    if pixels[max_c] > 12000: return max_c
    return None

# ===== INICIALIZACIÓN ROBUSTA =====
tello = Tello()

def conectar_dron():
    intentos = 0
    while intentos < 3:
        try:
            tello.connect()
            # Forzamos una consulta para asegurar que el socket de estado responda
            print(f"Conectado! Batería: {tello.get_battery()}%")
            return True
        except Exception as e:
            print(f"Intento {intentos+1} fallido: {e}. Reintentando...")
            intentos += 1
            time.sleep(2)
    return False

if not conectar_dron():
    print("No se pudo establecer comunicación con el Tello. Revisa el WiFi.")
    exit()

tello.streamon()
frame_read = tello.get_frame_read()

# ===== LÓGICA DE FASES (Como el código de tu compañero) =====
# Fase 1: Despegue | Fase 2: Buscar Color | Fase 3: Ajustar Altura | Fase 4: Cruzar | Fase 5: Land
fase = 1
color_detectado = None

try:
    print("FASE 1: Despegando...")
    tello.takeoff()
    time.sleep(2)
    fase = 2

    while True:
        img = frame_read.frame
        if img is None: continue
        
        # Seguridad: Paro de Emergencia (Regla 15)
        key = cv2.waitKey(1) & 0xFF
        if key == ord(' '): # Espacio
            tello.emergency()
            break
        if key == ord('q'):
            break

        height = tello.get_height()

        # Fase 2: Buscar color de túnel
        if fase == 2:
            color_detectado = detectar_color(img)
            if color_detectado:
                print(f"FASE 3: Túnel {color_detectado} detectado. Ajustando altura...")
                fase = 3
            else:
                # Si no ve nada, rotar suavemente para buscar
                tello.send_rc_control(0, 0, 0, 20) 

        # Fase 3: Ajuste de altura específico según el color
        elif fase == 3:
            target = 0
            if color_detectado == "AZUL":  target = ALTURA_AZUL
            if color_detectado == "VERDE": target = ALTURA_VERDE
            if color_detectado == "ROJO":  target = ALTURA_ROJO

            # Lógica de subida/bajada precisa
            if height < target - TOLERANCIA_ALT:
                tello.send_rc_control(0, 0, 20, 0)
            elif height > target + TOLERANCIA_ALT:
                tello.send_rc_control(0, 0, -20, 0)
            else:
                print("Altura correcta. FASE 4: Cruzando túnel...")
                tello.send_rc_control(0, 0, 0, 0)
                fase = 4

        # Fase 4: Avance a través del túnel (2 metros)
        elif fase == 4:
            # Avanzamos 3 metros para asegurar salida
            tello.move_forward(300)
            print("Túnel cruzado. FASE 5: Aterrizando...")
            fase = 5

        # Fase 5: Aterrizaje suave
        elif fase == 5:
            while tello.get_height() > 40:
                tello.send_rc_control(0, 0, -20, 0)
                time.sleep(0.2)
            tello.land()
            break

        # Dibujar info en pantalla
        cv2.putText(img, f"Fase: {fase} | Alt: {height}cm", (20, 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        if color_detectado:
            cv2.putText(img, f"Tunel: {color_detectado}", (20, 90), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        
        cv2.imshow("Mision 1 - Tello", img)

except Exception as e:
    print(f"Error en vuelo: {e}")
    tello.land()

finally:
    tello.streamoff()
    cv2.destroyAllWindows()
