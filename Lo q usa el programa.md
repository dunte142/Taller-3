Primero tenes que isntalar python 3.11

Para que funque tenes que poner en la consola 

python -m venv env               # Crear entorno virtual 
env\Scripts\activate            # Activar entorno en Windows

pip install opencv-python mediapipe numpy pyserial


Principalmente se usa 
https://ai.google.dev/edge/mediapipe/solutions/vision/hand_landmarker?hl=es-419 #Ia de google q reconoce la mano
https://colab.research.google.com/github/googlesamples/mediapipe/blob/main/examples/hand_landmarker/python/hand_landmarker.ipynb?hl=es-419 #Reconoce imagenes (supuestamente para la camara)
