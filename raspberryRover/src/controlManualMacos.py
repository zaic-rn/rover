import serial
import time

# Configuración del puerto Bluetooth
bluetooth_port = "/dev/tty.Rover"  # Cambia esto al puerto correcto
baud_rate = 9600

# Inicializar la conexión Bluetooth
try:
    bluetooth = serial.Serial(bluetooth_port, baud_rate, timeout=1)
    print(f"Conectado al puerto Bluetooth {bluetooth_port}")
except Exception as e:
    print(f"Error al conectar al puerto Bluetooth: {e}")
    exit()

# Función para enviar comandos al Arduino
def enviar_comando(comando):
    bluetooth.write(comando.encode())
    print(f"Comando enviado: {comando}")

# Menú de comandos
def mostrar_menu():
    print("\n--- Comandos del Rover ---")
    print("w: Avanzar")
    print("s: Retroceder")
    print("a: Giro en pivote a la izquierda")
    print("d: Giro en pivote a la derecha")
    print("q: Curva a la izquierda")
    print("e: Curva a la derecha")
    print("x: Detener")
    print("+: Aumentar velocidad")
    print("-: Disminuir velocidad")
    print("r: Aumentar factor de reducción en curvas")
    print("f: Disminuir factor de reducción en curvas")
    print("q: Salir")

# Bucle principal
def main():
    while True:
        mostrar_menu()
        comando = input("Ingresa un comando: ").strip().lower()

        if comando == "q":
            print("Saliendo...")
            break

        # Enviar el comando al Arduino
        enviar_comando(comando)

        # Pequeña pausa para evitar saturación
        time.sleep(0.1)

    # Cerrar la conexión Bluetooth al salir
    bluetooth.close()
    print("Conexión Bluetooth cerrada.")

if __name__ == "__main__":
    main()
