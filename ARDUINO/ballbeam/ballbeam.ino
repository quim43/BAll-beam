#include <Servo.h>  

/////////////////////// Calibración del sensor ///////////////////////
const float A = 10988.20; 
const float b = -1.147;  

/////////////////////// Parámetros del sistema ///////////////////////
const float m = 0.02;
const float R = 0.02;
const float g = 9.81;
const float L = 0.30;
const float I = (2.0 / 5.0) * m * R * R;
const float dt = 0.01;
const float K = (-m * g * R) / (L * (m + I / (R * R)));

/////////////////////// PID ///////////////////////
float Kp = 114;
float Ki = 3.2; //3.6;
float Kd = 0.051;
float Q = 2250;

/////////////////////// Sensores y actuadores ///////////////////////
const int pinSensor = A0;
const int pinServo = 9;
Servo miServo;

/////////////////////// Estado del sistema ///////////////////////
float distancia;
float distancia_setpoint = 17.20;
float distancia_error, distancia_error_anterior, distancia_integral = 0;
float PID_p, PID_i, PID_d, PID_total;
float angulo_servo;
float distancia_filtrada = 0; // Nueva variable para filtro

/////////////////////// Modelo dinámico ///////////////////////
float x;
float v = 0.0;
float a = 0.0;
float theta;

const float umbral_error = 0.3;

void setup() {
  Serial.begin(9600);
  miServo.attach(pinServo);
  miServo.write(90); // Posición inicial

  delay(1000); // Espera a que el servo se estabilice

  // Medir posición inicial de la bola
  int adcValor = analogRead(pinSensor);
  distancia_filtrada = A * pow(adcValor, b);
  x = distancia_filtrada;
}

void loop() {
  // 1. Leer sensor con filtro exponencial suavizado
  distancia = leerSensorFiltrado(0.150);  // Alpha entre 0.2 y 0.4 0.25

  // 2. Calcular error
  distancia_error = distancia_setpoint - distancia;

  // 3. Si está en el umbral, no actuamos
  if (abs(distancia_error) < umbral_error) {
    print(distancia);
    return;
  }

  // 4. PID
  PID_p = Kp * distancia_error;
  distancia_integral += distancia_error * dt;
  distancia_integral = constrain(distancia_integral, -33, 33);
  PID_i = Ki * distancia_integral;
  PID_d = Kd * ((distancia_error - distancia_error_anterior) / dt);
  PID_total = PID_p + PID_i + PID_d;

  // 5. Calcular ángulo del servo
  angulo_servo = map(PID_total, -Q, Q, 180, 10);
  angulo_servo = constrain(angulo_servo, 10, 180);
  miServo.write(angulo_servo);

  // 6. Modelo físico
  theta = (angulo_servo - 90) * (PI / 180.0);
  a = K * theta;
  v += a * dt;
  v = constrain(v, -8, 8);
  x += v * dt + 0.5 * a * dt * dt;
  x = constrain(x, 5, 25);

  // 7. Guardar error previo
  distancia_error_anterior = distancia_error;

  // 8. Imprimir datos
  print(distancia);

  // 9. Delay
  delay(dt * 1000);
}

// Filtro exponencial suavizado
float leerSensorFiltrado(float alpha) {
  float adc = 0;
  for (int i = 0; i < 10; i++) {
    adc += analogRead(pinSensor);
    delay(1);
  }
  adc /= 10;

  float lectura_actual = A * pow(adc, b);
  distancia_filtrada = alpha * lectura_actual + (1 - alpha) * distancia_filtrada;
  return distancia_filtrada;
}

// 📊 Imprimir en formato para plotter
float print(float valor) {
  Serial.print(0); Serial.print(" ");
  Serial.print(35); Serial.print(" ");
  Serial.print(valor); Serial.print(" ");
  Serial.println(distancia_setpoint);
}