import serial

# Configura el puerto serial (ajusta el puerto COM y la velocidad en baudios según tu configuración)
bluetooth_port = '/dev/tty.Rover'  # Cambia esto al puerto correcto
baud_rate = 9600

try:
    # Intenta abrir la conexión serial
    bluetooth = serial.Serial(bluetooth_port, baud_rate, timeout=1)
    print(f"Conectado a {bluetooth_port} a {baud_rate} baudios.")

    while True:
        # Envía un dato al Arduino
        dato = input("Escribe un carácter para enviar al Arduino: ")
        bluetooth.write(dato.encode())  # Envía el dato como bytes
        print(f"Enviado: {dato}")

except serial.SerialException as e:
    print(f"Error al conectar con el dispositivo Bluetooth: {e}")
finally:
    if 'bluetooth' in locals() and bluetooth.is_open:
        bluetooth.close()
        print("Conexión cerrada.")