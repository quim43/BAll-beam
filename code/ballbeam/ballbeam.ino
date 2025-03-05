#include <Servo.h>  

/////////////////////// Parámetros del sistema ///////////////////////
const float m = 0.046;   // Masa de la bola (kg)
const float R = 0.02125; // Radio de la bola (m)
const float g = 9.81;    // Aceleración de la gravedad (m/s^2)
const float L = 0.30;    // Longitud de la viga (m)
const float I = (2.0/5.0) * m * R * R; // Momento de inercia

const float dt = 0.05;  
const float K = (-m * g * R) / (L * (m + I / (R * R)));  

/////////////////////// PID (Ajustado) //////////////////////
float Kp = 6.8; //6.8;    // Aumentado para mejor respuesta
float Ki = 2.9;    // Incrementado para eliminar error en estado estacionario
float Kd = 0.90;

/////////////////////// Sensores y actuadores ///////////////////////
const int pinSensor = A0;
const int pinServo = 9;
Servo miServo;

/////////////////////// Variables de estado ///////////////////////
float distancia;  
float distancia_setpoint = 15.0;  // **Centro de la viga en 20 cm**
float distancia_error, distancia_error_anterior, distancia_integral = 0;
float PID_p, PID_i, PID_d, PID_total;
float angulo_servo;
float tiempo, tiempoPrevio;

/////////////////////// Variables de la dinámica del sistema ///////////////////////
float x;  // **Posición inicial de la bola**
float v = 0.0;  // Velocidad inicial
float a = 0.0;  // Aceleración inicial
float theta;    // Ángulo inicial del servo en radianes

// **Umbral de tolerancia para detener el PID**
const float umbral_error = 0.8;  // La bola se considera estabilizada si el error es menor a 0.5 cm

void setup() {
    Serial.begin(9600);
    miServo.attach(pinServo);
    
    // **Forzar la posición inicial del servo en 90°**
    miServo.write(90);
    delay(1000); // **Esperar a que el servo se estabilice**

    // **Medir la posición inicial de la bola**
    int adcValor = analogRead(pinSensor);
    x = 19661.88 * pow(adcValor, -1.282);  // **Posición inicial de la pelota**

    tiempoPrevio = millis();
}

void loop() {
    // 1. **Leer el sensor con filtro de promedio**
    distancia = leerSensorPromediado(3);  

    // 2. **Calcular error**
    distancia_error = distancia_setpoint - distancia;

    // 3. **Si la distancia está dentro del umbral, detenemos el control**
    if (abs(distancia_error) < umbral_error) {
        Serial.println("🎯 Pilota estable!");
        return;  // **No ajustamos el servo si la pelota ya está en su sitio**
    }

    // 4. **Calcular PID**
    PID_p = Kp * distancia_error;
    distancia_integral += distancia_error * dt;
    distancia_integral = constrain(distancia_integral, -30, 30);  
    PID_i = Ki * distancia_integral;
    PID_d = Kd * ((distancia_error - distancia_error_anterior) / dt);
    
    PID_total = (PID_p + PID_i + PID_d);  

    // 5. **Convertir PID a ángulo del servo**
    angulo_servo = map(PID_total, -200, 200, 160, 20);  
    angulo_servo = constrain(angulo_servo, 20, 160);
    miServo.write(angulo_servo);

    // 6. **Corrección de la función de transferencia**
    theta = (angulo_servo - 90) * (PI / 180.0);  
    a = K * theta;  

    v += a * dt;  
    v = constrain(v, -8, 8);  
    x += v * dt + 0.5 * a * dt * dt;  
    x = constrain(x, 5, 25);  

    // 7. **Guardar el error previo**
    distancia_error_anterior = distancia_error;

    // 8. **Mostrar valores en el puerto serie**
    Serial.print("Distancia Sensor: ");
    Serial.print(distancia);
    Serial.print(" cm | Posición Modelo: ");
    Serial.print(x);
    Serial.print(" cm | Servo: ");
    Serial.println(angulo_servo);

    // 9. **Esperar el tiempo de muestreo**
    delay(dt * 1000);
}

// **Función para filtrar el ruido del sensor**
float leerSensorPromediado(int num_muestras) {
    float suma = 0;
    for (int i = 0; i < num_muestras; i++) {
        suma += analogRead(pinSensor);
        delay(2);  
    }
    return 19661.88 * pow(suma / num_muestras, -1.282);  
}