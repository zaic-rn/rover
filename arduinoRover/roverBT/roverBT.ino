#include <Arduino.h>
#include <SoftwareSerial.h>

// Declaraciones de funciones
void procesarEntrada(char comando);
void enviarMensaje(String mensaje);
void actualizarMovimiento();

// Velocidad global, se puede modificar en tiempo real
int velocidadActual = 100;  // Valor inicial (ajustable)
// Factor de reducción para curvas: determina la velocidad reducida (por defecto 2)
int factorReduccion = 2;     

// Estado actual del movimiento:
// 'w' = avanzando, 's' = retrocediendo,
// 'd' = pivote giro derecha, 'a' = pivote giro izquierda,
// 'e' = curva derecha, 'q' = curva izquierda,
// 'x' = detenido
char estadoMovimiento = 'x'; 

// Clase para el control de motores DC
class MotorDC {
private:
    int pinIn1, pinIn2, pinPWM;

public:
    MotorDC(int in1, int in2, int pwm) {
        pinIn1 = in1;
        pinIn2 = in2;
        pinPWM = pwm;
        pinMode(pinIn1, OUTPUT);
        pinMode(pinIn2, OUTPUT);
        pinMode(pinPWM, OUTPUT);
    }

    void avanzar(int velocidad) {
        digitalWrite(pinIn1, HIGH);
        digitalWrite(pinIn2, LOW);
        analogWrite(pinPWM, velocidad);
    }

    void retroceder(int velocidad) {
        digitalWrite(pinIn1, LOW);
        digitalWrite(pinIn2, HIGH);
        analogWrite(pinPWM, velocidad);
    }

    void detener() {
        digitalWrite(pinIn1, LOW);
        digitalWrite(pinIn2, LOW);
        analogWrite(pinPWM, 0);
    }
};

// Clase para el Rover
class Rover {
private:
    MotorDC motorIzquierdoDelantero, motorIzquierdoMedio, motorIzquierdoTrasero;
    MotorDC motorDerechoDelantero, motorDerechoMedio, motorDerechoTrasero;

public:
    Rover(int in1, int in2, int pwm1, int in3, int in4, int pwm2, int in5, int in6, int pwm3,
          int in7, int in8, int pwm4, int in9, int in10, int pwm5, int in11, int in12, int pwm6)
            : motorIzquierdoDelantero(in1, in2, pwm1),
              motorIzquierdoMedio(in3, in4, pwm2),
              motorIzquierdoTrasero(in5, in6, pwm3),
              motorDerechoDelantero(in7, in8, pwm4),
              motorDerechoMedio(in9, in10, pwm5),
              motorDerechoTrasero(in11, in12, pwm6) {}

    void avanzar(int velocidad) {
        motorIzquierdoDelantero.avanzar(velocidad);
        motorIzquierdoMedio.avanzar(velocidad);
        motorIzquierdoTrasero.avanzar(velocidad);
        motorDerechoDelantero.avanzar(velocidad);
        motorDerechoMedio.avanzar(velocidad);
        motorDerechoTrasero.avanzar(velocidad);
    }

    void retroceder(int velocidad) {
        motorIzquierdoDelantero.retroceder(velocidad);
        motorIzquierdoMedio.retroceder(velocidad);
        motorIzquierdoTrasero.retroceder(velocidad);
        motorDerechoDelantero.retroceder(velocidad);
        motorDerechoMedio.retroceder(velocidad);
        motorDerechoTrasero.retroceder(velocidad);
    }

    void detener() {
        motorIzquierdoDelantero.detener();
        motorIzquierdoMedio.detener();
        motorIzquierdoTrasero.detener();
        motorDerechoDelantero.detener();
        motorDerechoMedio.detener();
        motorDerechoTrasero.detener();
    }
    
    // Giro en pivote a la derecha (gira en su propio eje)
    void girarDerecha(int velocidad) {
        motorIzquierdoDelantero.avanzar(velocidad);
        motorIzquierdoMedio.avanzar(velocidad);
        motorIzquierdoTrasero.avanzar(velocidad);
        motorDerechoDelantero.retroceder(velocidad);
        motorDerechoMedio.retroceder(velocidad);
        motorDerechoTrasero.retroceder(velocidad);
    }
    
    // Giro en pivote a la izquierda (gira en su propio eje)
    void girarIzquierda(int velocidad) {
        motorIzquierdoDelantero.retroceder(velocidad);
        motorIzquierdoMedio.retroceder(velocidad);
        motorIzquierdoTrasero.retroceder(velocidad);
        motorDerechoDelantero.avanzar(velocidad);
        motorDerechoMedio.avanzar(velocidad);
        motorDerechoTrasero.avanzar(velocidad);
    }
    
    // Giro curvo a la derecha: todos avanzan, pero los motores derechos a velocidad reducida
    void curvaDerecha(int velocidad) {
        int velocidadReducida = velocidad / factorReduccion;
        motorIzquierdoDelantero.avanzar(velocidad);
        motorIzquierdoMedio.avanzar(velocidad);
        motorIzquierdoTrasero.avanzar(velocidad);
        motorDerechoDelantero.avanzar(velocidadReducida);
        motorDerechoMedio.avanzar(velocidadReducida);
        motorDerechoTrasero.avanzar(velocidadReducida);
    }
    
    // Giro curvo a la izquierda: todos avanzan, pero los motores izquierdos a velocidad reducida
    void curvaIzquierda(int velocidad) {
        int velocidadReducida = velocidad / factorReduccion;
        motorIzquierdoDelantero.avanzar(velocidadReducida);
        motorIzquierdoMedio.avanzar(velocidadReducida);
        motorIzquierdoTrasero.avanzar(velocidadReducida);
        motorDerechoDelantero.avanzar(velocidad);
        motorDerechoMedio.avanzar(velocidad);
        motorDerechoTrasero.avanzar(velocidad);
    }
};

// Variables globales para los encoders
volatile int pulsos[6] = {0, 0, 0, 0, 0, 0}; // Contadores para cada motor
unsigned long tiempoAnterior = 0;

// Constantes del encoder
const int PPR = 100; // Pulsos por revolución
const float radioRueda = 0.035; // Radio de la rueda en metros

// Pines de los encoders
const int pinesEncoders[] = {2, 3, 18, 19, 20, 21}; // Pines digitales de interrupción

// Rover con los pines de cada motor
Rover rover(22, 23, 4, 24, 25, 5, 26, 27, 6, 28, 29, 7, 30, 31, 8, 32, 33, 9);

// SoftwareSerial para el Bluetooth
SoftwareSerial bluetooth(52, 53); // RX, TX

// ISR para cada encoder
void isrEncoder0() { pulsos[0]++; }
void isrEncoder1() { pulsos[1]++; }
void isrEncoder2() { pulsos[2]++; }
void isrEncoder3() { pulsos[3]++; }
void isrEncoder4() { pulsos[4]++; }
void isrEncoder5() { pulsos[5]++; }

// Array de funciones ISR
void (*isrEncoders[6])() = {isrEncoder0, isrEncoder1, isrEncoder2, isrEncoder3, isrEncoder4, isrEncoder5};

void setup() {
    Serial.begin(9600); // Comunicación Serial (USB)
    bluetooth.begin(9600); // Comunicación Bluetooth

    // Configurar los pines de los encoders como entrada y habilitar interrupciones
    for (int i = 0; i < 6; i++) {
        pinMode(pinesEncoders[i], INPUT);
        attachInterrupt(digitalPinToInterrupt(pinesEncoders[i]), isrEncoders[i], RISING);
    }
}

void loop() {
    unsigned long tiempoActual = millis();
    if (tiempoActual - tiempoAnterior >= 1000) { // Cada segundo
        noInterrupts();
        int pulsosMedidos[6];
        for (int i = 0; i < 6; i++) {
            pulsosMedidos[i] = pulsos[i];
            pulsos[i] = 0;
        }
        interrupts();

        // Calcular y enviar velocidades por Serial y Bluetooth
        for (int i = 0; i < 6; i++) {
            float velocidadRPM = (pulsosMedidos[i] * 60.0) / PPR;
            float velocidadLineal = (velocidadRPM * 2 * PI * radioRueda) / 60.0;

            String mensaje = "Motor " + String(i + 1) + " - Velocidad (RPM): " + String(velocidadRPM) +
                             " - Velocidad lineal (m/s): " + String(velocidadLineal);
            enviarMensaje(mensaje);
        }
        tiempoAnterior = tiempoActual;
    }

    // Procesar comandos desde Serial o Bluetooth
    if (Serial.available()) {
        char comando = Serial.read();
        procesarEntrada(comando);
    }
    if (bluetooth.available()) {
        char comando = bluetooth.read();
        procesarEntrada(comando);
    }

    // Refrescar continuamente el movimiento actual para actualizar la velocidad
    actualizarMovimiento();
}

void procesarEntrada(char comando) {
    switch (comando) {
        case 'w':
            estadoMovimiento = 'w';
            rover.avanzar(velocidadActual);
            enviarMensaje("Avanzando a velocidad " + String(velocidadActual));
            break;
        case 's':
            estadoMovimiento = 's';
            rover.retroceder(velocidadActual);
            enviarMensaje("Retrocediendo a velocidad " + String(velocidadActual));
            break;
        case 'd': // Giro en pivote a la derecha
            estadoMovimiento = 'd';
            rover.girarDerecha(velocidadActual);
            enviarMensaje("Giro en pivote derecha a velocidad " + String(velocidadActual));
            break;
        case 'a': // Giro en pivote a la izquierda
            estadoMovimiento = 'a';
            rover.girarIzquierda(velocidadActual);
            enviarMensaje("Giro en pivote izquierda a velocidad " + String(velocidadActual));
            break;
        case 'e': // Curva a la derecha (giro curvo)
            estadoMovimiento = 'e';
            rover.curvaDerecha(velocidadActual);
            enviarMensaje("Curva derecha a velocidad " + String(velocidadActual));
            break;
        case 'q': // Curva a la izquierda (giro curvo)
            estadoMovimiento = 'q';
            rover.curvaIzquierda(velocidadActual);
            enviarMensaje("Curva izquierda a velocidad " + String(velocidadActual));
            break;
        case 'x':
            estadoMovimiento = 'x';
            rover.detener();
            enviarMensaje("Detenido");
            break;
        case '+': // Incrementa la velocidad en 10 unidades (máximo 255)
            velocidadActual = min(velocidadActual + 10, 255);
            enviarMensaje("Velocidad aumentada a " + String(velocidadActual));
            break;
        case '-': // Disminuye la velocidad en 10 unidades (mínimo 0)
            velocidadActual = max(velocidadActual - 10, 0);
            enviarMensaje("Velocidad disminuida a " + String(velocidadActual));
            break;
        // Comandos para ajustar el factor de reducción en curvas
        case 'r': // Aumenta el factor de reducción (más reducción, curva más pronunciada)
            factorReduccion = min(factorReduccion + 1, 5);
            enviarMensaje("Factor de reducción aumentado a " + String(factorReduccion));
            break;
        case 'f': // Disminuye el factor de reducción (menos reducción)
            factorReduccion = max(factorReduccion - 1, 1);
            enviarMensaje("Factor de reducción disminuido a " + String(factorReduccion));
            break;
        default:
            enviarMensaje("Comando no reconocido");
            break;
    }
}

void actualizarMovimiento() {
    // Esta función refresca el movimiento según el estado actual
    switch (estadoMovimiento) {
        case 'w':
            rover.avanzar(velocidadActual);
            break;
        case 's':
            rover.retroceder(velocidadActual);
            break;
        case 'd':
            rover.girarDerecha(velocidadActual);
            break;
        case 'a':
            rover.girarIzquierda(velocidadActual);
            break;
        case 'e':
            rover.curvaDerecha(velocidadActual);
            break;
        case 'q':
            rover.curvaIzquierda(velocidadActual);
            break;
        case 'x':
        default:
            rover.detener();
            break;
    }
}

void enviarMensaje(String mensaje) {
    Serial.println(mensaje);
    bluetooth.println(mensaje);
}
