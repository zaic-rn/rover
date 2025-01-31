import serial
import time
import cv2
import os
import subprocess


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
        out.write(frame)
        cv2.imshow('Grabando...', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

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
