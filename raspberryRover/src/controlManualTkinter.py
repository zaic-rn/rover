import serial
import time
import cv2
import os
import subprocess
import tkinter as tk
from PIL import Image, ImageTk
import threading


class RoverController:
    def __init__(self, mac_address, baudrate=9600):
        self.mac_address = mac_address
        self.rfcomm_port = "/dev/rfcomm0"
        self.serial_connection = None

        self.configurar_bluetooth()

        try:
            self.serial_connection = serial.Serial(self.rfcomm_port, baudrate, timeout=1)
            time.sleep(2)
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


class RoverApp:
    def __init__(self, root, rover):
        self.root = root
        self.rover = rover
        self.cap = None
        self.video_label = None
        self.recording = False
        self.output_dir = "../media"

        root.title("Control del Rover")

        # Crear botones de control
        self.btn_forward = tk.Button(root, text="Avanzar", command=lambda: self.send_command("w"))
        self.btn_forward.grid(row=0, column=1)

        self.btn_left = tk.Button(root, text="Izquierda", command=lambda: self.send_command("a"))
        self.btn_left.grid(row=1, column=0)

        self.btn_stop = tk.Button(root, text="Detener", command=lambda: self.send_command("x"))
        self.btn_stop.grid(row=1, column=1)

        self.btn_right = tk.Button(root, text="Derecha", command=lambda: self.send_command("d"))
        self.btn_right.grid(row=1, column=2)

        self.btn_backward = tk.Button(root, text="Retroceder", command=lambda: self.send_command("s"))
        self.btn_backward.grid(row=2, column=1)

        self.btn_record = tk.Button(root, text="Grabar Video", command=self.start_recording)
        self.btn_record.grid(row=3, column=0)

        self.btn_photo = tk.Button(root, text="Tomar Foto", command=self.take_photo)
        self.btn_photo.grid(row=3, column=1)

        self.btn_extract = tk.Button(root, text="Extraer Imágenes", command=self.extract_images)
        self.btn_extract.grid(row=3, column=2)

        # Video Label
        self.video_label = tk.Label(root)
        self.video_label.grid(row=4, column=0, columnspan=3)

        # Botón para salir
        self.btn_exit = tk.Button(root, text="Salir", command=self.exit_app)
        self.btn_exit.grid(row=5, column=1)

        # Iniciar captura de video
        self.start_video_stream()

    def send_command(self, command):
        self.rover.enviar_comando(command)

    def start_video_stream(self):
        self.cap = cv2.VideoCapture(0)
        self.update_video()

    def update_video(self):
        if self.cap is not None:
            ret, frame = self.cap.read()
            if ret:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                image = Image.fromarray(frame)
                photo = ImageTk.PhotoImage(image)

                self.video_label.config(image=photo)
                self.video_label.image = photo

            self.root.after(10, self.update_video)

    def start_recording(self):
        if not self.recording:
            self.recording = True
            threading.Thread(target=self.record_video, daemon=True).start()

    def record_video(self, duration=10):
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("Error: No se pudo acceder a la cámara.")
            return

        os.makedirs(self.output_dir, exist_ok=True)
        video_path = os.path.join(self.output_dir, "video.avi")
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        out = cv2.VideoWriter(video_path, fourcc, 20.0, (640, 480))

        print("Grabando video...")
        start_time = time.time()

        while int(time.time() - start_time) < duration and self.recording:
            ret, frame = cap.read()
            if not ret:
                break
            out.write(frame)

        cap.release()
        out.release()
        self.recording = False
        print(f"Video guardado en: {video_path}")

    def take_photo(self):
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("Error: No se pudo acceder a la cámara.")
            return

        os.makedirs(self.output_dir, exist_ok=True)
        time.sleep(2)
        ret, frame = cap.read()

        if ret:
            photo_path = os.path.join(self.output_dir, "foto.jpg")
            cv2.imwrite(photo_path, frame)
            print(f"Foto guardada en: {photo_path}")

        cap.release()

    def extract_images(self):
        video_path = os.path.join(self.output_dir, "video.avi")
        output_dir = os.path.join(self.output_dir, "frames")
        os.makedirs(output_dir, exist_ok=True)

        cap = cv2.VideoCapture(video_path)
        if not cap.isOpened():
            print("Error: No se pudo abrir el video.")
            return

        frame_count = 0
        saved_count = 0
        frame_interval = 10

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

    def exit_app(self):
        if self.cap:
            self.cap.release()
        self.rover.cerrar_conexion()
        self.root.quit()


if __name__ == "__main__":
    mac_address = "98:D3:32:70:D8:75"
    controlador = RoverController(mac_address)

    root = tk.Tk()
    app = RoverApp(root, controlador)
    root.mainloop()