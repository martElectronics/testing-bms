#include <Arduino.h>

class SensorCorriente{
    
    private:
    int valorADC;
    float voltage;
    const double VOLTAJE_REPOSO = 0.847590;
    const double S1_SENSITIVITY = 9.238754325; //26.7 mv/A la de 75A  26.7mv/2.89
    const float S2_SENSITIVITY = 0.0057;
    const float DEFAULT_VOLTAJE = 3.36; //Valor máximo de la tensión en la salida 
    const float MAX_ADC_VALUE = 4095.0; //Valor entero máximo que puede leerse en la salida

    public:
    //Constructor
    SensorCorriente(){
    }

    //Obtener el entero (representa el voltaje) de la entrada
    int getValorADC(){
        return valorADC;
    }

    //Calcular el voltaje de la salida
    float getVoltaje(int adc){
        voltage= (adc*1.0) * (DEFAULT_VOLTAJE/ MAX_ADC_VALUE);
        return voltage;
    }
    
    //Calcular la corriente de la salida 1
    double calcularCorrienteS1(int adc){
        double sum=0;
        for(int i=0;i<10;i++)
        {
        sum+=getVoltaje(adc);
        delay(1);
        }
        return (sum/10 - VOLTAJE_REPOSO)/ (S1_SENSITIVITY/1000);
    }

    //Calcular la corriente de la salida 2
    float calcularCorrienteS2(){
        return (voltage - VOLTAJE_REPOSO) / S2_SENSITIVITY;
    }
};
