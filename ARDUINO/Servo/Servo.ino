#include <Servo.h> // Incluye la librería para controlar servos

Servo miServo; // Crea un objeto Servo
int pinServo = 9; // Define el pin donde está conectado el servo

void setup() {
    miServo.attach(pinServo); // Conectar el servo al pin 9
    
}

void loop() {
    miServo.write(0);  // Mueve el servo a 0 grados
    delay(1000);       // Espera 1 segundoºº

    miServo.write(180); // Mueve el servo a 180 grados
    delay(1000);        // Espera 1 segundo
}