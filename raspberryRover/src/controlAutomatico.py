from ultralytics import YOLO
import cv2

model = YOLO("../models/best.pt")  # Cargar el modelo entrenado

cap = cv2.VideoCapture(0)  # Usar cmara web

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    results = model.predict(frame, conf=0.5)  # Detectar linea

    # Dibujar las detecciones en la imagen
    for r in results:
        for box in r.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

    cv2.imshow("Deteccion de Linea", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
