const int pinSensor = A0; // Pin donde está conectado el sensor
float distancia;
const float a = 10988.20; 
const float b = -1.147;  

void setup() {
    Serial.begin(9600); // Inicia comunicación serial
}

void loop() {
    int adcValor = analogRead(pinSensor); // Leer el valor del ADC
    distancia = a * pow(adcValor, b); // Calcular la distancia en cm

    Serial.print("Distancia: ");
    Serial.print(distancia);
    Serial.println(" cm");

    delay(500); // Pequeño retardo para evitar ruido en la lectura
}