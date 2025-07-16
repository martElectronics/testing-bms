#include <Arduino.h>

class SensorCorriente{
    
    private:
    int valorADC;
    float voltage;
    const float VOLTAJE_REPOSO = 2.46;
    const float S1_SENSITIVITY = 0.0667;
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
    float calcularCorrienteS1(int adc){
        getVoltaje(adc);
        return (voltage - VOLTAJE_REPOSO) / S1_SENSITIVITY;
    }

    //Calcular la corriente de la salida 2
    float calcularCorrienteS2(){
        return (voltage - VOLTAJE_REPOSO) / S2_SENSITIVITY;
    }
};
