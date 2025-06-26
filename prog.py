import cv2 
import mediapipe as mp 
import numpy as np 
import math
import serial
import time

#arduino = serial.Serial('COM3', 9600)
#time.sleep(2)

mp_manos = mp.solutions.hands
mp_dibujo = mp.solutions.drawing_utils
manos = mp_manos.Hands(
    max_num_hands=2,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.7
)

camara = cv2.VideoCapture(0)

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

angulo_anterior_dedos = None

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

        # -------------------- CUADRADO --------------------
        if cantidad_manos == 1 and not cuadrado_activo:
            lm = info_manos[0]
            if es_palma_abierta(lm):
                mano_y = lm[0].y * alto
                if posicion_mano_anterior_y is not None:
                    if posicion_mano_anterior_y - mano_y > 20:
                        cuadrado_activo = True
                        cuadrado_tiempo = tiempo_actual
                        #arduino.write(b'C')
                posicion_mano_anterior_y = mano_y
            else:
                posicion_mano_anterior_y = None

        # -------------------- CiRCULO --------------------
        if cantidad_manos == 1 and not circulo_lateral_dibujado:
            lm = info_manos[0]

            # Paso 1: mano abierta
            if estado_circulo == 0 and es_palma_abierta(lm):
                estado_circulo = 1

            # Paso 2: dedos rectos apuntan hacia X o -X (ángulo ~0 o ~180 grados)
            elif estado_circulo == 1:
                p5 = (lm[5].x * ancho, lm[5].y * alto)  # base índice
                p8 = (lm[8].x * ancho, lm[8].y * alto)  # punta índice
                angulo = calcular_angulo_vector(p5, p8)

                if -30 < angulo < 30 or 150 < abs(angulo) < 210:  # apuntando a la derecha o izquierda
                    angulo_anterior_dedos = angulo
                    estado_circulo = 2

            # Paso 3: inclinación del dedo (ángulo cambia notablemente en Y)
            elif estado_circulo == 2:
                p5 = (lm[5].x * ancho, lm[5].y * alto)
                p8 = (lm[8].x * ancho, lm[8].y * alto)
                angulo_actual = calcular_angulo_vector(p5, p8)

                if angulo_anterior_dedos is not None and abs(angulo_actual - angulo_anterior_dedos) > 30:
                    # Inclinación detectada
                    if angulo_actual > angulo_anterior_dedos:
                        circulo_lateral_posicion = (50, alto // 2)
                    else:
                        circulo_lateral_posicion = (ancho - 50, alto // 2)

                    circulo_lateral_dibujado = True
                    circulo_tiempo = tiempo_actual
                    estado_circulo = 0
                    #arduino.write(b'O')

            # Reinicio si se pierde el gesto
            elif not es_palma_abierta(lm):
                estado_circulo = 0
                angulo_anterior_dedos = None

        # -------------------- TRIANGULO --------------------
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
                    #arduino.write(b'T')
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

    if triangulo_activo and tiempo_actual - triangulo_tiempo > 10:
        triangulo_activo = False
        triangulo_tiempo = None

    if circulo_lateral_dibujado and tiempo_actual - circulo_tiempo > 10:
        circulo_lateral_dibujado = False
        circulo_lateral_posicion = None
        circulo_tiempo = None

    # -------------------- DIBUJAR FORMAS --------------------
    if cuadrado_activo:
        tamaño = 60
        centro = (ancho // 3, alto // 2)
        esquina1 = (centro[0] - tamaño // 2, centro[1] - tamaño // 2)
        esquina2 = (centro[0] + tamaño // 2, centro[1] + tamaño // 2)
        cv2.rectangle(fotograma, esquina1, esquina2, (0, 255, 255), -1)
        cv2.putText(fotograma, 'Gesto: levantar mano (cuadrado)', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    if triangulo_activo:
        centro_x, centro_y = int(ancho * 2 / 3), alto // 2
        tamaño = 60
        p1 = (centro_x, centro_y - tamaño)
        p2 = (centro_x - tamaño, centro_y + tamaño)
        p3 = (centro_x + tamaño, centro_y + tamaño)
        puntos = [p1, p2, p3]
        cv2.polylines(fotograma, [np.array(puntos, np.int32)], isClosed=True, color=(0, 128, 255), thickness=4)
        cv2.putText(fotograma, 'Gesto: separar manos (triangulo)', (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 128, 255), 2)

    if circulo_lateral_dibujado and circulo_lateral_posicion:
        cv2.circle(fotograma, circulo_lateral_posicion, 30, (255, 0, 0), -1)
        cv2.putText(fotograma, 'Gesto: dedos inclinados (circulo)', (10, alto - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

    cv2.imshow("Gestos", fotograma)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

camara.release()
cv2.destroyAllWindows()
#arduino.close()
