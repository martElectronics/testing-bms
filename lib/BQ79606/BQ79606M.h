#include "BQ79606.h"
#include <stdlib.h>
#include <Arduino.h>

class BQ79606M : public BQ79606 {
    private:
        //* Estructura que define las lecturas de tensiones y temperatura *//
        struct Mensure {  
            uint16_t voltage;       // Voltaje de la celda
            uint16_t temperature;   // Temperatura de la celda
        };
        //* Estructura que define la estructura de fallo*//
        struct Fault {
            uint16_t faultID;       // ID del fallo
            uint16_t faultValue;    // Estado del fallo
        };
        //* Estructura que define el circuito integrado*//
        struct IcBQ {
            byte idCell;            // ID de la celda
            Mensure measure[6];     // Estructura para almacenar medidas de la celda(SIEMPRE SON 6 FALLOS)
            Fault* faults;          // Estructura para almacenar fallos de la celda
            bool  isBalanced;       // Estado de balanceo de la celda
        };
        // Función para leer registros de tensión
        uint16_t readVoltages(uint8_t cell,byte id);
        uint16_t readTemperatures(uint8_t cellID,byte id);
        
        void setTimeCells(float timeOut, float timeCells,bool inSeconds);

        void resetfaultBQ();      // Resetea todos los registros de fallo
        void getFault(); // Lee los registros de fallo y los guarda en la estructura
 
    public:
        // Constructor
        BQ79606M(uint8_t totalBoards, uint32_t baudRate, uint8_t wakePin, uint8_t faultPin, uint8_t bmsOkPin, uint8_t bmsRxPin, uint8_t bmsTxPin);

        IcBQ icBq[24];                          //Array de estructuras donde se almacenan los datos de los BQ

        int  readFault(uint64_t faultID, byte id);       
        void initializeBQs();
        void setThresh(int volt);
        void printData();

        void getMeansure();                     //Lee valores de temeraturas en los GPIO

        // Funciónes rutina de balanceo
        void setBalance(byte timeOut, byte timeCells,bool inSeconds,byte typeBalancing);
        void startBalancing();
        void stopBalancing();
};