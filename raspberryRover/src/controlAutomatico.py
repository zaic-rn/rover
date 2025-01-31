from ultralytics import YOLO
import cv2
import os
import sys

# Cargar el modelo entrenado
model = YOLO("/home/zaic/Desktop/raspberryRover/models/best.pt")

# Iniciar la captura de video (cámara web)
cap = cv2.VideoCapture(0)  # Usar cámara web

# Verificar si hay acceso a una pantalla
try:
    # Intentar usar 'cv2.imshow' y ver si puede funcionar en el entorno
    cv2.imshow('Prueba', cv2.imread("test_image.jpg"))
    cv2.destroyWindow('Prueba')
    display_available = True
except cv2.error:
    display_available = False

# Iniciar el procesamiento de los frames
frame_count = 0
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Realizar la predicción usando el modelo YOLO
    results = model.predict(frame, conf=0.5)  # Detectar línea

    # Dibujar las detecciones en la imagen
    for r in results:
        for box in r.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

    if display_available:
        # Si hay acceso a la pantalla, mostrar la imagen
        cv2.imshow("Detección de Línea", frame)

        # Asegurarse de que la ventana sea actualizada correctamente
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
    else:
        # Si no hay acceso a la pantalla (SSH), mostrar en consola
        print("Detección de línea: ")
        for r in results:
            for box in r.boxes:
                print(
                    f"Detección en [x1: {box.xyxy[0][0]}, y1: {box.xyxy[0][1]}, x2: {box.xyxy[0][2]}, y2: {box.xyxy[0][3]}]")

# Liberar la captura de video y cerrar
cap.release()
if display_available:
    cv2.destroyAllWindows()
print("Captura de video finalizada.")