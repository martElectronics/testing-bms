#include "BQ79606M.h"
#include "BQ79606.h"
#include <HardwareSerial.h>
#include <Arduino.h>


#define MAXBYTES 8*2 

BQ79606M::BQ79606M(uint8_t totalBoards, uint32_t baudRate, uint8_t wakePin, uint8_t faultPin, uint8_t bmsOkPin, uint8_t bmsRxPin, uint8_t bmsTxPin) : BQ79606(totalBoards, baudRate, wakePin, faultPin, bmsOkPin, bmsRxPin, bmsTxPin) {
    if(totalBoards<2){
        totalBoards=2;
    }
}
    
void BQ79606M::initializeBQs() 
{
    Ini_ESP();  //Inicia micro
    Wake79606();//Despierta el BQ79606M
    CommReset(250000);//Configutacion velocidad de comunicacion
    AutoAddress();//Se realiza autoadressing
    InitDevices();//Configuramos los dispositivos escritura de registros
    //resetfaultBQ();//Reseteamos registos de fallos
    delay(10);
}

int BQ79606M::readFault(uint64_t faultID, byte id) //Lee valores de fallos en el BQ79606
{
    byte response[MAXBYTES + 6];
    int bytesRead = ReadReg(id,faultID, response, 2, 0, FRMWRT_SGL_R);

    if (bytesRead > 0) {
        uint16_t rawData = (response[4]);
        return rawData;
    } else {
        Serial.println("Error: No se pudo leer el registro de fallo");
        return 0;
    }

}
void BQ79606M::getMeansure()
{
    for(int i = 0; i < totalBoards; i++){
        for(int j = 0; j < 6; j++) {
            icBq[i].measure[j].voltage = readVoltages(i,icBq[i].idCell); // Lee el voltaje de la celda
            icBq[i].measure[j].temperature = readTemperatures(i,icBq[i].idCell); // Lee la temperatura de la celda
        }
    }
}

uint16_t BQ79606M::readVoltages(uint8_t cellID,byte id) 
{
    byte response[MAXBYTES + 6];
    if (cellID >= 6) {
        Serial.println("Error: cellID fuera de los límites");
        return 0;
    }

    uint16_t cellVoltageAddr = VCELL1H + (cellID * 2); // Calcula la dirección de memoria donde se va a leer
    int bytesRead = ReadReg(id,cellVoltageAddr, response, 2, 0, FRMWRT_SGL_R);

    if (bytesRead > 0) {
        uint16_t rawData = (response[4] << 8) | response[5];//Adjunta datos ya que la lectura es de 2 bytes
        return 1000*Complement(rawData,0.00019073);

    } 
    return 0;
}

uint16_t BQ79606M::readTemperatures(uint8_t cellID,byte id) //Lee valores de temeraturas en los GPIO
{
    byte response[MAXBYTES + 6];
    uint16_t cellTemperatureAddr = AUX_GPIO1H + (cellID * 2); 
    int bytesRead = ReadReg(id,cellTemperatureAddr, response, 2, 0, FRMWRT_SGL_R);

    if (bytesRead > 0) {
        uint16_t rawData = (response[4] << 8) | response[5];
        uint16_t v = Complement(rawData,0.00019073);
        return (25.03-37.81*(v-1.23));          //Curva que aproxima el ntc 
    }
    return 0;
}




void BQ79606M::resetfaultBQ() {//Resetea todos los registros de fallo

    byte reset = 0x3F;

    WriteReg(0,SYSFLT1_FLT_RST, reset, 0, FRMWRT_ALL_NR);
    WriteReg(0,SYSFLT2_FLT_RST, reset, 0, FRMWRT_ALL_NR);
    WriteReg(0,SYSFLT3_FLT_RST, reset, 0, FRMWRT_ALL_NR);
    WriteReg(0,COMM_COMH_FLT_RST, reset, 0, FRMWRT_ALL_NR);
    WriteReg(0,COMM_COMH_RC_FLT_RST, reset, 0, FRMWRT_ALL_NR);
    WriteReg(0,COMM_COMH_RR_FLT_RST, reset, 0, FRMWRT_ALL_NR);
    WriteReg(0,COMM_COMH_TR_FLT_RST, reset, 0, FRMWRT_ALL_NR);
    WriteReg(0,COMM_COML_FLT_RST, reset, 0, FRMWRT_ALL_NR);
    WriteReg(0,COMM_COML_RC_FLT_RST, reset, 0, FRMWRT_ALL_NR);
    WriteReg(0,COMM_COML_RR_FLT_RST, reset, 0, FRMWRT_ALL_NR);
    WriteReg(0,COMM_COML_TR_FLT_RST, reset, 0, FRMWRT_ALL_NR);

    WriteReg(0,OTP_FLT_RST, reset, 0, FRMWRT_ALL_NR);
    WriteReg(0,RAIL_FLT_RST, reset, 0, FRMWRT_ALL_NR);
    WriteReg(0,OVUV_BIST_FLT_RST, reset, 0, FRMWRT_ALL_NR);
    WriteReg(0,OTUT_BIST_FLT_RST, reset, 0, FRMWRT_ALL_NR);
    WriteReg(0,COMM_UART_FLT_RST, reset, 0, FRMWRT_ALL_NR);
    WriteReg(0,COMM_UART_RC_FLT_RST, reset, 0, FRMWRT_ALL_NR);
    WriteReg(0,COMM_UART_RR_FLT_RST, reset, 0, FRMWRT_ALL_NR);
    WriteReg(0,COMM_UART_TR_FLT_RST, reset, 0, FRMWRT_ALL_NR);
}



void BQ79606M::setThresh(int volt) {

	// Validar el rango de tension permitido (2.8V a 4.3V)
	if (volt < 2.8) volt = 2.8;
	if (volt > 4.3) volt = 4.3;

	// Calcular el valor del campo THRESH[5:0]
	byte threshValue = (byte)((volt - 2.8) / 0.025);  // Paso de 25mV

	// Asegurar que el valor no exceda el límite permitido (4.3V)
	if (threshValue > 0b111100) threshValue = 0b111100;

	// Construir el byte de datos (Habilitar ENABLE y configurar THRESH[5:0])
	byte regValue = 0b01000000 | (threshValue & 0b00111111);

	// Escribir en el registro
	WriteReg(0, CB_DONE_THRESHOLD, regValue, 1, FRMWRT_ALL_NR);
}
void BQ79606M::setBalance(byte timeOut, byte timeCells,bool inSeconds,byte typeBalancing) {

	// Validar el rango de tiempo permitido (0 a 127)
	if (timeOut< 0) timeOut = 0;
	if (timeOut> 127) timeOut = 127;

	// Convertir tiempo a un valor entero dentro del rango (0 - 127)
	uint8_t timeValue = (uint8_t)timeOut & 0b01111111;  // Asegurar que es un valor de 7 bits

	// Configurar TIME_UNIT (bit 7): 0 para minutos, 1 para segundos
	uint8_t unitTime = (inSeconds ? 0b10000000 : 0b00000000);
    uint8_t typeBal=0x00; // Inicializar tipo de balanceo

    switch (typeBalancing){//Se configura el tipo de balanceo
        case 0x00: // Balancea impares solo
            typeBal=0x00; // Balancea impares solo
            break;
        case 0x01: // Balancea pares solo
            typeBal=0x01; // Balancea pares solo
            break;
        case 0x02: // Balancea pares y despues impares
            typeBal=0x02; // Balancea pares y despues impares
            break;
        case 0x03: // Balancea impares y despues pares
            typeBal=0x03; // Balancea impares y despues pares
            break;
        default:
            Serial.println("Error: Tipo de balanceo no válido.Se configura al tipo 2");
            typeBal=0x03;
             // Salir de la función si el tipo de balanceo no es válido
        break;
    }

    uint8_t cbConfigDutyCicle = 0b01111000; // Inicializar el registro de configuración de balanceo
    uint8_t cbcellBalancing = 0b01111111; // Tiempo de balanceo de celdas

    //* Se construiran los registros para escribirlos *//

    byte regCBDONE = typeBal | unitTime | cbConfigDutyCicle; // Configuración de tiempo de balanceo total
    byte regCBBAL  = typeBal | unitTime | cbcellBalancing; // Configuración de tiempo de balanceo de celdas
    



    WriteReg(0, CB_CONFIG, regCBDONE, 1, FRMWRT_ALL_NR); // 2 minutes duty cycle, continue on fault, odds then even
    //configure cell balancing  timers
    WriteReg(0, CB_CELL1_CTRL, regCBBAL, 1, FRMWRT_ALL_NR); // Configuracion de tiempos para las celdas
    WriteReg(0, CB_CELL2_CTRL, regCBBAL, 1, FRMWRT_ALL_NR); // 
    WriteReg(0, CB_CELL3_CTRL, regCBBAL, 1, FRMWRT_ALL_NR); // 
    WriteReg(0, CB_CELL4_CTRL, regCBBAL, 1, FRMWRT_ALL_NR); // 
    WriteReg(0, CB_CELL5_CTRL, regCBBAL, 1, FRMWRT_ALL_NR); // 
    WriteReg(0, CB_CELL6_CTRL, regCBBAL, 1, FRMWRT_ALL_NR); // 
}
void BQ79606M::getFault() {//Guarda lo fallos dentro de la estructura cell de forma dinamuca
    byte value=0;//Variable que cuenta la cantidad de fallos que contiene el sistema 
    for(int i = SYS_FAULT1; i < 3;i++) {
        readFault(i,value); // Lee el registro de fallos y guarda el valor en la estructura
    }
}
void BQ79606M::printData()
{
    Serial.println("Datos de las celdas:");
    for (int i = 0; i < totalBoards; i++) {
        for (int j = 0; j < 6; j++) {
            Serial.print("Celda ");
            Serial.print(j);
            Serial.print(": Voltaje: ");
            Serial.print(icBq[i].measure[j].voltage);
            Serial.print(" mV, Temperatura: ");
            Serial.print(icBq[i].measure[j].temperature);
            Serial.println(" °C");
        }

    }
}
void BQ79606M::startBalancing() {
   WriteReg(0, CONTROL2, CONTROL2_INIT_BALUE | 0b00100000, 1, FRMWRT_ALL_NR);
}
void BQ79606M::stopBalancing() {
   WriteReg(0, CONTROL2, CONTROL2_INIT_BALUE & 0b11011111, 1, FRMWRT_ALL_NR);
}