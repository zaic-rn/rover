import serial
import time
import cv2
import os
import subprocess
import numpy as np

class RoverController:
    def __init__(self, mac_address, baudrate=9600):
        self.mac_address = mac_address
        self.rfcomm_port = "/dev/rfcomm0"
        self.serial_connection = None

        self.configurar_bluetooth()

        try:
            self.serial_connection = serial.Serial(self.rfcomm_port, baudrate, timeout=1)
            time.sleep(2)  # Esperar a que se establezca la conexión
            print("Conexión establecida con el Arduino.")
        except serial.SerialException as e:
            print(f"Error al conectar con el puerto {self.rfcomm_port}: {e}")
            self.serial_connection = None

    def configurar_bluetooth(self):
        print("Configurando conexión Bluetooth...")
        self.limpiar_rfcomm()
        subprocess.run(["sudo", "rfcomm", "bind", "0", self.mac_address], check=True)
        print("Bluetooth conectado correctamente.")

    def limpiar_rfcomm(self):
        print("Liberando cualquier conexión previa...")
        subprocess.run(["sudo", "rfcomm", "release", "0"], check=False)
        time.sleep(1)

    def enviar_comando(self, comando):
        if self.serial_connection and self.serial_connection.is_open:
            try:
                self.serial_connection.write(comando.encode('utf-8'))
                print(f"Comando enviado: {comando}")
            except serial.SerialException as e:
                print(f"Error al enviar el comando: {e}")
        else:
            print("Conexión serial no disponible.")

    def cerrar_conexion(self):
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
            print("Conexión serial cerrada.")
        self.limpiar_rfcomm()


def mostrar_menu():
    print("\n--- Control del Rover ---")
    print("w: Avanzar")
    print("s: Retroceder")
    print("a: Dirección izquierda")
    print("d: Dirección derecha")
    print("x: Detener")
    print("+: Aumentar velocidad")
    print("-: Reducir velocidad")
    print("v: Grabar video")
    print("p: Tomar foto")
    print("e: Extraer imágenes del video")
    print("q: Salir")


# Cargar la red YOLO
net = cv2.dnn.readNet('yolov3.weights', 'yolov3.cfg')
layer_names = net.getLayerNames()
output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]

def detectar_linea(frame):
    # Preprocesamiento de la imagen
    blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
    net.setInput(blob)
    outs = net.forward(output_layers)

    # Listas para almacenar los resultados de las detecciones
    boxes = []
    confidences = []
    class_ids = []

    height, width, channels = frame.shape

    # Procesar las detecciones
    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > 0.5:  # Puedes ajustar el umbral de confianza
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)

                # Coordenadas de las cajas delimitadoras
                x = int(center_x - w / 2)
                y = int(center_y - h / 2)

                boxes.append([x, y, w, h])
                confidences.append(float(confidence))
                class_ids.append(class_id)

    return boxes, confidences, class_ids


def grabar_video(output_dir, duration=10):
    cap = cv2.VideoCapture(0)  # Inicializar la cámara (0 para la cámara principal)
    if not cap.isOpened():
        print("Error: No se pudo acceder a la cámara.")
        return

    os.makedirs(output_dir, exist_ok=True)
    video_path = os.path.join(output_dir, "video.avi")
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(video_path, fourcc, 20.0, (640, 480))

    print("Grabando video...")
    start_time = time.time()

    while int(time.time() - start_time) < duration:
        ret, frame = cap.read()
        if not ret:
            print("Error: No se pudo leer el frame.")
            break

        boxes, confidences, class_ids = detectar_linea(frame)

        # Filtrar las detecciones de la línea y mostrar los resultados
        for i in range(len(boxes)):
            x, y, w, h = boxes[i]
            label = str(class_ids[i])
            confidence = confidences[i]

            # Dibuja la caja delimitadora
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Imprimir información sobre la detección
            print(f"Detectado: {label} - Confianza: {confidence:.2f}")

        # Guardar la imagen con las detecciones
        out.write(frame)

        # Si no deseas mostrar la imagen, solo puedes guardar la imagen aquí sin usar imshow.
        cv2.imwrite(os.path.join(output_dir, f"frame_{int(time.time())}.jpg"), frame)

    cap.release()
    out.release()
    cv2.destroyAllWindows()
    print(f"Video guardado en: {video_path}")


def tomar_foto(output_dir):
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: No se pudo acceder a la cámara.")
        return

    os.makedirs(output_dir, exist_ok=True)
    time.sleep(2)

    for _ in range(5):
        ret, frame = cap.read()

    if ret:
        foto_path = os.path.join(output_dir, "foto.jpg")
        cv2.imwrite(foto_path, frame)
        print(f"Foto guardada en: {foto_path}")
    else:
        print("Error: No se pudo capturar la foto.")

    cap.release()
    cv2.destroyAllWindows()


def extraer_imagenes(video_path, output_dir, frame_interval=10):
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print("Error: No se pudo abrir el video.")
        return

    os.makedirs(output_dir, exist_ok=True)
    frame_count = 0
    saved_count = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        if frame_count % frame_interval == 0:
            image_path = os.path.join(output_dir, f"frame_{saved_count:05d}.jpg")
            cv2.imwrite(image_path, frame)
            print(f"Imagen guardada: {image_path}")
            saved_count += 1

        frame_count += 1

    cap.release()
    print(f"Se extrajeron {saved_count} imágenes del video.")


if __name__ == "__main__":
    mac_address = "98:D3:32:70:D8:75"  # MAC del HC-05
    controlador = RoverController(mac_address)
    output_dir = "../media"

    try:
        while True:
            mostrar_menu()
            comando = input("Ingresa un comando: ").strip()

            if comando == 'q':
                print("Saliendo del programa...")
                break
            elif comando == 'v':
                duracion = int(input("Duración del video en segundos: "))
                grabar_video(output_dir, duracion)
            elif comando == 'p':
                tomar_foto(output_dir)
            elif comando == 'e':
                video_path = os.path.join(output_dir, "video.avi")
                images_dir = os.path.join(output_dir, "frames")
                intervalo = int(input("Ingresa el intervalo de frames (por ejemplo, 10): "))
                extraer_imagenes(video_path, images_dir, intervalo)
            else:
                controlador.enviar_comando(comando)
    except KeyboardInterrupt:
        print("\nPrograma interrumpido por el usuario.")
    finally:
        controlador.cerrar_conexion()
        print("Programa finalizado.")