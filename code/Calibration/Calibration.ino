void setup() {
  // // Comunicación seria a 9600 baudios
  Serial.begin(9600);
}

void loop() {
  // Leemos el promedio de la entrada analógica 0 
  int ADC_SHARP=ADC0_promedio(20);
  Serial.print(0);
  Serial.print(" ");
  Serial.print(700);
  Serial.print(" ");
  Serial.print(ADC_SHARP);
  Serial.println(" ");
  delay(10);
}

int ADC0_promedio(int n)
{
  long suma=0;
  for(int i=0;i<n;i++)
  {
    suma=suma+analogRead(A0);
  }  
  return(suma/n);
}
