import cv2 # type: ignore
import mediapipe as mp # type: ignore
import numpy as np # type: ignore
import math
import serial # type: ignore
import time
import pygame # type: ignore

# -------------------- CONFIGURACIÓN DE AUDIO --------------------
pygame.mixer.init()
sonido_cuadrado = pygame.mixer.Sound("cuadrado.mp3")
sonido_triangulo = pygame.mixer.Sound("triangulo.mp3")
sonido_circulo = pygame.mixer.Sound("circulo.mp3")

# -------------------- CONFIGURACIÓN ARDUINO --------------------
arduino = serial.Serial('COM3', 9600)  # Cambia 'COM3' por tu puerto correcto
time.sleep(2)  # Espera para establecer conexión

# Definición de comandos
COMANDO_SERVO = b'S'    # Cuadrado - Servo
COMANDO_LEDS = b'L'     # Triángulo - LEDs
COMANDO_BOMBA = b'B'    # Círculo - Bomba
COMANDO_APAGAR = b'A'   # Apagar todo

# -------------------- CONFIGURACIÓN MEDIAPIPE --------------------
mp_manos = mp.solutions.hands
mp_dibujo = mp.solutions.drawing_utils
manos = mp_manos.Hands(
    max_num_hands=2,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.7
)

# -------------------- INICIALIZACIÓN CÁMARA --------------------
camara = cv2.VideoCapture(0)

# -------------------- VARIABLES DE ESTADO --------------------
cuadrado_activo = False
cuadrado_tiempo = None

triangulo_activo = False
triangulo_tiempo = None

circulo_lateral_dibujado = False
circulo_lateral_posicion = None
circulo_tiempo = None

estado_circulo = 0
manos_juntas_anterior = None
posicion_mano_anterior_y = None
angulo_anterior_dedos = None

# -------------------- FUNCIONES AUXILIARES --------------------
def distancia(p1, p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

def calcular_angulo_vector(p1, p2):
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    return math.degrees(math.atan2(dy, dx))

def es_palma_abierta(lm):
    dedos_punta = [8, 12, 16, 20]
    dedos_base = [5, 9, 13, 17]
    return all(lm[p].y < lm[b].y for p, b in zip(dedos_punta, dedos_base))

def es_puno(lm):
    dedos_punta = [8, 12, 16, 20]
    dedos_base = [5, 9, 13, 17]
    return all(lm[p].y > lm[b].y for p, b in zip(dedos_punta, dedos_base))

# -------------------- BUCLE PRINCIPAL --------------------
while camara.isOpened():
    exito, fotograma = camara.read()
    if not exito:
        break

    fotograma = cv2.flip(fotograma, 1)
    alto, ancho, _ = fotograma.shape
    imagen_rgb = cv2.cvtColor(fotograma, cv2.COLOR_BGR2RGB)
    resultado = manos.process(imagen_rgb)

    cantidad_manos = len(resultado.multi_hand_landmarks) if resultado.multi_hand_landmarks else 0
    manos_juntas = None
    tiempo_actual = time.time()

    if resultado.multi_hand_landmarks and resultado.multi_handedness:
        lista_manos = []
        info_manos = []
        for i, puntos_mano in enumerate(resultado.multi_hand_landmarks):
            mp_dibujo.draw_landmarks(fotograma, puntos_mano, mp_manos.HAND_CONNECTIONS)
            lm = puntos_mano.landmark
            info_manos.append(lm)
            punto9 = lm[9]
            lista_manos.append((punto9.x * ancho, punto9.y * alto))

        # -------------------- CUADRADO (SERVO) --------------------
        if cantidad_manos == 1 and not cuadrado_activo:
            lm = info_manos[0]
            if es_palma_abierta(lm):
                mano_y = lm[0].y * alto
                if posicion_mano_anterior_y is not None:
                    if posicion_mano_anterior_y - mano_y > 50:
                        cuadrado_activo = True
                        cuadrado_tiempo = tiempo_actual
                        sonido_cuadrado.play()
                        arduino.write(COMANDO_SERVO)
                        print("SERVO ACTIVADO - Gestos: Mano levantada (Cuadrado)")
                posicion_mano_anterior_y = mano_y
            else:
                posicion_mano_anterior_y = None

        # -------------------- CÍRCULO (BOMBA) --------------------
        if cantidad_manos == 1 and not circulo_lateral_dibujado:
            lm = info_manos[0]

            if estado_circulo == 0 and es_palma_abierta(lm):
                estado_circulo = 1

            elif estado_circulo == 1:
                p5 = (lm[5].x * ancho, lm[5].y * alto)
                p8 = (lm[8].x * ancho, lm[8].y * alto)
                angulo = calcular_angulo_vector(p5, p8)

                if -30 < angulo < 30 or 150 < abs(angulo) < 210:
                    angulo_anterior_dedos = angulo
                    estado_circulo = 2

            elif estado_circulo == 2:
                p5 = (lm[5].x * ancho, lm[5].y * alto)
                p8 = (lm[8].x * ancho, lm[8].y * alto)
                angulo_actual = calcular_angulo_vector(p5, p8)

                if angulo_anterior_dedos is not None and abs(angulo_actual - angulo_anterior_dedos) > 30:
                    if angulo_actual > angulo_anterior_dedos:
                        circulo_lateral_posicion = (50, alto // 2)
                    else:
                        circulo_lateral_posicion = (ancho - 50, alto // 2)

                    circulo_lateral_dibujado = True
                    circulo_tiempo = tiempo_actual
                    sonido_circulo.play()
                    arduino.write(COMANDO_BOMBA)
                    print("BOMBA ACTIVADA - Gestos: Inclinación de dedos (Círculo)")
                    estado_circulo = 0

            elif not es_palma_abierta(lm):
                estado_circulo = 0
                angulo_anterior_dedos = None

        # -------------------- TRIÁNGULO (LEDS) --------------------
        if cantidad_manos == 2 and not triangulo_activo:
            if es_palma_abierta(info_manos[0]) and es_palma_abierta(info_manos[1]):
                dist = distancia(lista_manos[0], lista_manos[1])
                if dist < 100:
                    manos_juntas = True
                elif dist > 150:
                    manos_juntas = False

                if manos_juntas_anterior is not None and manos_juntas_anterior == True and manos_juntas == False:
                    triangulo_activo = True
                    triangulo_tiempo = tiempo_actual
                    sonido_triangulo.play()
                    arduino.write(COMANDO_LEDS)
                    print("LEDS ACTIVADOS - Gestos: Separar manos (Triángulo)")
            else:
                manos_juntas = None
        else:
            manos_juntas = None

    if manos_juntas is not None:
        manos_juntas_anterior = manos_juntas

    # -------------------- LIMPIEZA AUTOMÁTICA --------------------
    if cuadrado_activo and tiempo_actual - cuadrado_tiempo > 10:
        cuadrado_activo = False
        cuadrado_tiempo = None
        arduino.write(COMANDO_APAGAR)
        print("Componentes apagados - Tiempo agotado")

    if triangulo_activo and tiempo_actual - triangulo_tiempo > 10:
        triangulo_activo = False
        triangulo_tiempo = None
        arduino.write(COMANDO_APAGAR)
        print("Componentes apagados - Tiempo agotado")

    if circulo_lateral_dibujado and tiempo_actual - circulo_tiempo > 10:
        circulo_lateral_dibujado = False
        circulo_lateral_posicion = None
        circulo_tiempo = None
        arduino.write(COMANDO_APAGAR)
        print("Componentes apagados - Tiempo agotado")

    # -------------------- DIBUJAR FORMAS --------------------
    if cuadrado_activo:
        tamaño = 60
        centro = (ancho // 3, alto // 2)
        esquina1 = (centro[0] - tamaño // 2, centro[1] - tamaño // 2)
        esquina2 = (centro[0] + tamaño // 2, centro[1] + tamaño // 2)
        cv2.rectangle(fotograma, esquina1, esquina2, (0, 255, 255), -1)
        cv2.putText(fotograma, 'SERVO', (esquina1[0], esquina1[1]-10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

    if triangulo_activo:
        centro_x, centro_y = int(ancho * 2 / 3), alto // 2
        tamaño = 60
        p1 = (centro_x, centro_y - tamaño)
        p2 = (centro_x - tamaño, centro_y + tamaño)
        p3 = (centro_x + tamaño, centro_y + tamaño)
        puntos = [p1, p2, p3]
        cv2.polylines(fotograma, [np.array(puntos, np.int32)], isClosed=True, color=(0, 128, 255), thickness=4)
        cv2.putText(fotograma, 'LEDS', (p2[0], p2[1]+30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 128, 255), 2)

    if circulo_lateral_dibujado and circulo_lateral_posicion:
        cv2.circle(fotograma, circulo_lateral_posicion, 30, (255, 0, 0), -1)
        cv2.putText(fotograma, 'BOMBA', (circulo_lateral_posicion[0]-40, circulo_lateral_posicion[1]-40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

    cv2.imshow("Control Arduino por Gestos", fotograma)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        arduino.write(COMANDO_APAGAR)
        break

# -------------------- LIMPIEZA FINAL --------------------
camara.release()
cv2.destroyAllWindows()
arduino.close()