#include <Arduino.h>
#include "SensorCorriente.cpp"

#define AMP_PIN  10   
SensorCorriente sensor1;
void setup()
{
Serial.begin(115200);
pinMode(AMP_PIN,INPUT);
}


void loop()
{
 int adcCurrentValue = analogRead(AMP_PIN);


  //** PROCESS DATA */
 double  stsCorrienteCarga = sensor1.calcularCorrienteS1(adcCurrentValue);
  // 0.85V lectura 1.73V real a
  // 1.73V a 5V   R=2.89

 // stsAMPOK = (sensor1.getVoltaje(adcCurrentValue) > 0.5);

 Serial.println((String)" Analog: "+adcCurrentValue );
  Serial.println((String)" Corriente: "+stsCorrienteCarga );
}
