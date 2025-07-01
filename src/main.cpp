#include <Arduino.h>
//#include <BQ79606.h>
#include "BQ79606.h"
#include <MART_CAN.h>
#include <set>

CAN_BUS CAN(HardwareType::Transciever, MCP_SPEED_500, 2,10);

//BMS
float stsVoltCells[12][11];
float stsTempCells[12][9];

void readVoltages(bool &ok);
void printVoltages();
float voltToTemp(float GPIOVoltage);
// Estructura para definir un punto de exclusión.
struct ExclusionPoint {
    int idModule;
    int contVoltNTC;

    // Sobrecarga del operador '<' para que std::set pueda ordenar los elementos.
    bool operator<(const ExclusionPoint& other) const {
        if (idModule != other.idModule) {
            return idModule < other.idModule;
        }
        return contVoltNTC < other.contVoltNTC;
    }
};
// Declaramos la lista de exclusión como una variable global.
std::set<ExclusionPoint> exclusionList;

//******CHARGE    */
unsigned long idStsCharger=0x18FF50E5;
unsigned long idCmdCharger=0x1806E5F4;
byte stsChargerByte[8];
byte cmdChargerByte[8];
void controlCharge(float maxVolt,float maxCurrent, bool start);
void mostrarDatosDetallados();
void showChargeData();
void procesarComandoSerial(float &valorFloatRef, bool &valorBoolRef, bool &reset, bool &fail);



void setup() {

 bool ok=false;
  Ini_ESP();
  // while(!ok)
  // {
  //   Wake79606();
  //   CommReset(BAUDRATE);
  //   ok= AutoAddress();
  // }

  Serial.print("Addres: ");
	delay(10);

  int nCurrentBoard = 0;
  byte response_frame[(MAXBYTES+6)];
  byte response_frame2[(MAXBYTES+6)];
    
	for (nCurrentBoard = 0; nCurrentBoard < TOTALBOARDS; nCurrentBoard++) {
    memset(response_frame2, 0, sizeof(response_frame2));
    ReadReg(nCurrentBoard, DEVADD_USR, response_frame2, 1, 0, FRMWRT_SGL_R);
		Serial.print((String)"Board "+nCurrentBoard+"= ");

    Serial.print(response_frame2[4]);

		Serial.println(".");

		delay(10);

	}

  InitDevices();

  WriteReg(0, SYSFLT1_FLT_RST, 0xFFFFFF, 3, FRMWRT_ALL_NR);   //reset system faults
  WriteReg(0, SYSFLT1_FLT_MSK, 0xFFFFFF, 3, FRMWRT_ALL_NR);
  WriteReg(0, CONTROL2, 0x10, 1, FRMWRT_ALL_NR);          //tsref activo
  
  //SET UP MAIN ADC
  WriteReg(0, CELL_ADC_CTRL, 0x3F, 1, FRMWRT_ALL_NR);     //enable conversions for all cells
  WriteReg(0, CELL_ADC_CONF2, 0x08, 1, FRMWRT_ALL_NR);    //set continuous ADC conversions, and set minimum conversion interval

  WriteReg(0, GPIO1_CONF, 0x20, 1, FRMWRT_ALL_NR);       //GPIO is an input
  WriteReg(0, GPIO2_CONF, 0x20, 1, FRMWRT_ALL_NR);       //GPIO is an input
  WriteReg(0, GPIO3_CONF, 0x20, 1, FRMWRT_ALL_NR);       //GPIO is an input
  WriteReg(0, GPIO4_CONF, 0x20, 1, FRMWRT_ALL_NR);       //GPIO is an input
  WriteReg(0, GPIO5_CONF, 0x20, 1, FRMWRT_ALL_NR);       //GPIO is an input
  WriteReg(0, GPIO6_CONF, 0x20, 1, FRMWRT_ALL_NR);       //GPIO is an input

  WriteReg(0, AUX_ADC_CTRL1, 0xF0, 1, FRMWRT_ALL_NR);       //GPIO is an input
  WriteReg(0, AUX_ADC_CTRL2, 0x03, 1, FRMWRT_ALL_NR);       //GPIO is an input


  WriteReg(0, CONTROL2, 0x13, 1, FRMWRT_ALL_NR);          //CELL_ADC_GO = 1 Y tsref y AUX_ADC_GO = 1


  delay(3*TOTALBOARDS+901);                             //3us of re-clocking delay per board + 901us waiting for first ADC conversion to complete
  
     while(CAN.error == 1){
      Serial.println("Error Initializing EScP32Can...");
   }
   Serial.println("CAN OK");

 
mostrarDatosDetallados();


}

void loop() {

  static bool cmdCharge = false;
  static bool stsVoltagesOK=false;
  static bool cmdResetFail = false;
  static bool stsFail = false;
  static float corrienteCarga=0;

  CAN.receive();
  CAN.getPacket(idStsCharger,stsChargerByte,8,false);
  


  
// printVoltages();
  procesarComandoSerial(corrienteCarga,cmdCharge,cmdResetFail,stsVoltagesOK);

  if(!stsVoltagesOK)
  {
    stsFail=1;
  }
  else if(stsVoltagesOK && cmdResetFail)
  {
    cmdResetFail=false;
    stsFail=0;
  }
  
  if(stsFail)
  {
    cmdCharge=0;
    corrienteCarga=0;
  }
  

   controlCharge(24,corrienteCarga,cmdCharge);
   CAN.send();
   //printVoltages();
   //Serial.println((String)"I= "+corrienteCarga+" cmdCharge= "+cmdCharge +" reset= "+cmdResetFail+ " ok= "+stsVoltagesOK);

   static int t=millis();
   if((millis()-t)>=1000)
   {
      //Serial.println((String)"I= "+corrienteCarga+" cmdCharge= "+cmdCharge +" reset= "+cmdResetFail+ " ok= "+stsVoltagesOK);
      //showChargeData();
     // readVoltages(stsVoltagesOK);
     CAN.printByteArray(stsChargerByte,8);
   }
}

void debug(){
CAN.printByteArray(stsChargerByte,8);
//Serial.println((String)"I= "+corrienteCarga+" cmdCharge= "+cmdCharge +" reset= "+cmdResetFail);
}


/**
 * @brief Espera y procesa comandos desde el puerto serie.
 * @param valorFloatRef Referencia a la variable float que se modificará con el comando 'c'.
 * @param valorBoolRef Referencia a la variable bool que se modificará con el comando 'c'.
 */
void procesarComandoSerial(float &valorFloatRef, bool &valorBoolRef, bool &reset, bool &fail) {
  // Solo procesa si hay datos disponibles en el buffer del puerto serie
  if (Serial.available() > 0) {
    // Lee la cadena completa hasta que encuentra un salto de línea
    String comando = Serial.readStringUntil('\n');
    comando.trim(); // Elimina espacios en blanco o carácteres invisibles al inicio/final

    // --- Comando 'c': configurar valores ---
    // Formato esperado: "c,valor_float,valor_bool" (ej: "c,123.45,1")
    if (comando.startsWith("c,")) {
      // Busca la posición de la primera y segunda coma
      int primeraComa = comando.indexOf(',');
      int segundaComa = comando.indexOf(',', primeraComa + 1);

      // Si encontramos ambas comas, el formato es potencialmente correcto
      if (segundaComa > primeraComa) {
        // Extrae la subcadena para el float
        String floatStr = comando.substring(primeraComa + 1, segundaComa);
        
        // Extrae la subcadena para el bool (0 o 1)
        String boolStr = comando.substring(segundaComa + 1);

        // Convierte las cadenas a sus tipos de dato y actualiza las variables por referencia
        valorFloatRef = floatStr.toFloat();
        valorBoolRef = (boolStr.toInt() == 1); // Convierte a bool (1=true, 0=false)

        Serial.print(F("-> OK: Comando 'c' recibido. Nuevos valores: "));
        Serial.print(F("float = "));
        Serial.print(valorFloatRef);
        Serial.print(F(", bool = "));
        Serial.println(valorBoolRef ? "true" : "false");

      } else {
        // Error si el formato no es el esperado
        Serial.println(F("-> ERROR: Formato de comando 'c' incorrecto. Use: c,float,bool"));
      }
    
    // --- Comando 'i': ejecutar función I ---
    } else if (comando == "i") {
     mostrarDatosDetallados();

    // --- Comando 'r': ejecutar función R ---
    } else if (comando == "r") {
      reset=1;

      } else if (comando == "f") {
      fail=0;
      }
    else if (comando == "p") {
     printVoltages();
      
    
    // --- Comando desconocido ---
    } else if (comando.length() > 0) {
      Serial.print(F("-> ERROR: Comando desconocido: '"));
      Serial.print(comando);
      Serial.println(F("'"));
    }
  }
}


void mostrarDatosDetallados() {

//randomSeed(analogRead(4));
  // for (int i = 0; i < TOTALBOARDS/2; i++) {
  //   for (int j = 0; j < 11; j++) 
  //   stsVoltCells[i][j] = 3.9;//random(320, 415) / 100.0;
  // }
  // for (int i = 0; i < TOTALBOARDS/2; i++) {
  //   for (int j = 0; j < 9; j++) 
  //   stsTempCells[i][j] = 1.08;//random(250, 450) / 10.0;
  // }
  // stsVoltCells[2][5] = 3.15;
  // stsVoltCells[8][1] = 4.21;
  // stsTempCells[10][3] = 22.5;
  // stsTempCells[4][7] = 51.2;


  Serial.println(F("\n--- Vista Detallada por Modulo ---")); // F() macro ahorra RAM
  Serial.println(F("Modulo | Voltajes (V)                  | Temperaturas (C)"));
  Serial.println(F("------------------------------------------------------------------"));

  // Bucle a través de cada módulo
  for (int i = 0; i < TOTALBOARDS/2; i++) {
    // Imprime el identificador del módulo (ej: M00, M01, ... M11)
    Serial.print("M");
    if (i < 10) {
      Serial.print("0"); // Añade un cero para alinear M0 a M9 con M10 y M11
    }
    Serial.print(i);
    Serial.print("   | V: ");

    // Imprime los 11 voltajes de celda para el módulo actual
    for (int j = 0; j < 11; j++) {
      Serial.print(stsVoltCells[i][j], 2); // Imprime con 2 decimales
      Serial.print(" ");
    }

    Serial.print("| T: ");

    // Imprime las 9 temperaturas para el módulo actual
    for (int j = 0; j < 9; j++) {
      Serial.print(stsTempCells[i][j], 1); // Imprime con 1 decimal
      Serial.print(" ");
    }

    Serial.println(); // Salto de línea para el siguiente módulo
  }
  Serial.println(F("------------------------------------------------------------------"));
}



void showChargeData()
{
  // 3. PROCESAMIENTO Y CÁLCULO
  // --- Inicialización de variables de resultados ---
  float minVolt = 5.0;      // Iniciar con un valor alto
  float maxVolt = 0.0;      // Iniciar con un valor bajo
  int   moduloMinVolt = 0;
  int   moduloMaxVolt = 0;
  float sumaTotalVolt = 0.0;

  float minTemp = 100.0;    // Iniciar con un valor alto
  float maxTemp = -20.0;    // Iniciar con un valor bajo
  int   moduloMinTemp = 0;
  int   moduloMaxTemp = 0;
  float sumaTotalTemp = 0.0;

  // --- Procesamiento del array de Voltajes ---
  for (int i = 0; i < TOTALBOARDS/2; i++) { // Bucle por cada Módulo
    for (int j = 0; j < 11; j++) { // Bucle por cada Celda de voltaje
      float v = stsVoltCells[i][j];
      if (v < minVolt) {
        minVolt = v;
        moduloMinVolt = i;
      }
      if (v > maxVolt) {
        maxVolt = v;
        moduloMaxVolt = i;
      }
      sumaTotalVolt += v;
    }
  }

  // --- Procesamiento del array de Temperaturas ---
  for (int i = 0; i < TOTALBOARDS/2; i++) { // Bucle por cada Módulo
    for (int j = 0; j < 9; j++) { // Bucle por cada Sensor de temperatura
      float t = stsTempCells[i][j];
      if (t < minTemp) {
        minTemp = t;
        moduloMinTemp = i;
      }
      if (t > maxTemp) {
        maxTemp = t;
        moduloMaxTemp = i;
      }
      sumaTotalTemp += t;
    }
  }
  
  // --- Cálculo final de la temperatura media ---
  float tempMedia = sumaTotalTemp / ((TOTALBOARDS/2.0) * 9.0);

  // 4. IMPRESIÓN DEL MENSAJE COMPACTO
  Serial.print("V:{");
  Serial.print(minVolt, 2);
  Serial.print("V@M");
  Serial.print(moduloMinVolt);
  Serial.print(" ~ ");
  Serial.print(maxVolt, 2);
  Serial.print("V@M");
  Serial.print(moduloMaxVolt);
  Serial.print("} T:{");
  Serial.print(minTemp, 1);
  Serial.print("C@M");
  Serial.print(moduloMinTemp);
  Serial.print(" ~ ");
  Serial.print(maxTemp, 1);
  Serial.print("C@M");
  Serial.print(moduloMaxTemp);
  Serial.print("} | V_total:");
  Serial.print(sumaTotalVolt, 2);
  Serial.print("V | T_media:");
  Serial.print(tempMedia, 2);
  Serial.println("C");

}


float voltToTemp(float GPIOVoltage)
{
  float R_NTC = (1.952e9 * GPIOVoltage) / (200000.0 * (2.5 - GPIOVoltage) - 9760.0 * GPIOVoltage);
  float R_NTC_2 = R_NTC / 1000.0f; // Convert to kOhms
  float TEMP = -7.388512707e-02f * R_NTC_2 * R_NTC_2 * R_NTC_2 + 1.987401122e+00f * R_NTC_2 * R_NTC_2 + -1.975941021e+01f * R_NTC_2 + 9.894631155e+01f;
  return TEMP;
}


void controlCharge(float maxVolt, float maxCurrent, bool start) {
  // These will hold the final two-byte values for voltage and current.
    uint8_t volt_high_byte, volt_low_byte;
    uint8_t curr_high_byte, curr_low_byte;

    if (!start) {
        // If start is false, we stop the charging process by setting
        // voltage and current to 0.
        volt_high_byte = 0x00;
        volt_low_byte = 0x00;
        curr_high_byte = 0x00;
        curr_low_byte = 0x00;
        return;
    }

    const float VOLTAGE_SCALING_FACTOR = 10.0f;
    uint16_t scaled_voltage = static_cast<uint16_t>(maxVolt * VOLTAGE_SCALING_FACTOR);

    
    volt_high_byte = (scaled_voltage >> 8) & 0xFF; // Right shift by 8 bits to get the MSB
    volt_low_byte = scaled_voltage & 0xFF;        // AND with 0xFF to get the LSB

    const float CURRENT_SCALING_FACTOR = 10.0f;
    uint16_t scaled_current = static_cast<uint16_t>(maxCurrent * CURRENT_SCALING_FACTOR);

    // Extract the high and low bytes from the 16-bit scaled current.
    curr_high_byte = (scaled_current >> 8) & 0xFF;
    curr_low_byte = scaled_current & 0xFF;


     byte cmdChargerByte[8];
  for(int i=0;i<8;i++)
  {
   cmdChargerByte[i]=0x00;
  }
  cmdChargerByte[0]=volt_high_byte;
  cmdChargerByte[1]=volt_low_byte;
  cmdChargerByte[2]=curr_high_byte;
  cmdChargerByte[3]=curr_low_byte;
  cmdChargerByte[4]=(byte)!start;

   CAN.setPacket(idCmdCharger,cmdChargerByte,8,false);

}



void readVoltages(bool &ok)
{
  ok=true;
    //VARIABLES
    byte response_frame[(MAXBYTES+6)];
    byte response_frame2[(MAXBYTES+6)];
    int currentBoard = 0;
    int res1 = 0,res2=0;
    int i = 0;
    int contVoltCells=0, contVoltNTC=0, idModule=0;
    //reset variables
        memset(response_frame, 0, sizeof(response_frame));
        i = 0;
        currentBoard=0;
        WriteReg(0, CONTROL2, 0x13, 1, FRMWRT_ALL_NR);
        delay(200);
        
          //PARSE, FORMAT, AND PRINT THE DATA
          for(currentBoard = 0; currentBoard<TOTALBOARDS; currentBoard++)
          { 
              idModule=currentBoard/2;  
              memset(response_frame, 0, sizeof(response_frame));
              memset(response_frame2, 0, sizeof(response_frame));
              //read back data (6 cells and 2 bytes each cell)
              res1 = ReadReg(currentBoard, VCELL1H, response_frame, MAXBYTES, 0, FRMWRT_SGL_R);
              res2 = ReadReg(currentBoard, AUX_GPIO1H, response_frame2, MAXBYTES, 0, FRMWRT_SGL_R);
              if((res1<=0) || (res2<=0))
              {
                ok=false;
                Serial.println("Error de lectura numBytes=0");
              }

              //Cambio al módulo siguiente, reset de contadores
              if((currentBoard%2)==0)
              {
                contVoltCells=0;
                contVoltNTC=0;
              }
              //response frame actually starts with top of stack, so currentBoard is actually inverted from what it should be
              //go through each byte in the current board (12 bytes = 6 cells * 2 bytes each)
              for(i=0; i<12; i+=2)
              {
                uint16_t rawData = (response_frame[i+4] << 8) | response_frame[i+5];
                float cellVoltage = Complement(rawData,0.00019073);
                //cellVoltage=4;
                
                if(cellVoltage >= 4.2 || cellVoltage<=2.5){
                  ok=false;
                }

                if(contVoltCells<11)
                {
                stsVoltCells[idModule][contVoltCells]=cellVoltage;
                contVoltCells++;
                }
    
                
              }


              //go through each byte in the current board (12 bytes = 6 GPIO * 2 bytes each)
              for(i=0; i<12; i+=2)
              {
                //each board responds with 32 data bytes + 6 header bytes

                //convert the two individual bytes of each cell into a single 16 bit data item (by bit shifting)
                uint16_t rawData = (response_frame2[i+4] << 8) | response_frame2[i+5];

                //do the two's complement of the resultant 16 bit data item, and multiply by 190.73uV to get an actual voltage
                float GPIOVoltage = Complement(rawData,0.00019073);
               // GPIOVoltage=1.08;
              // Serial.println((String)"GPIO " +(i/2)+" Voltage= " +GPIOVoltage);
                float temp= voltToTemp(GPIOVoltage);
                if(temp >= 60 ){
                  ok=false;
                }
                if(contVoltNTC<9)
                {
                  
                //Serial.println(contVoltNTC);
                stsTempCells[idModule][contVoltNTC]=temp;
                contVoltNTC++;
                }
                //print the voltages - it is i/2 because cells start from 1 up to 6
                //and there are 2 bytes per cell (i value is twice the cell number),
                //and it's +1 because cell names start with "Cell1"
              }
          }
      
}


void printVoltages()
{

   delay(10);
    //VARIABLES
    byte response_frame[(MAXBYTES+6)];
    byte response_frame2[(MAXBYTES+6)];
    int currentBoard = 0;
    int Bytesleidos = 0;
    int i = 0;
    //reset variables
        memset(response_frame, 0, sizeof(response_frame));
        i = 0;
        currentBoard=0;
        WriteReg(0, CONTROL2, 0x13, 1, FRMWRT_ALL_NR);
        delay(2000);
        /*
         * ***********************************************
         * NOTE: SOME COMPUTERS HAVE ISSUES TRANSMITTING
         * A LARGE AMOUNT OF DATA VIA PRINTF STATEMENTS.
         * THE FOLLOWING PRINTOUT OF THE RESPONSE DATA
         * IS NOT GUARANTEED TO WORK ON ALL SYSTEMS.
         * ***********************************************
        */
        
        if(Bytesleidos == -1){
          Serial.println("No se ha podido leer los datos, se ha excedido el tiempo");
          //delay(1000);
        }
        else{

        
          //PARSE, FORMAT, AND PRINT THE DATA
          for(currentBoard = 0; currentBoard<TOTALBOARDS; currentBoard++)
          {   
              memset(response_frame, 0, sizeof(response_frame));
              memset(response_frame2, 0, sizeof(response_frame));
              //read back data (6 cells and 2 bytes each cell)
              Bytesleidos = ReadReg(currentBoard, VCELL1H, response_frame, MAXBYTES, 0, FRMWRT_SGL_R);
              Bytesleidos = ReadReg(currentBoard, AUX_GPIO1H, response_frame2, MAXBYTES, 0, FRMWRT_SGL_R);
              //response frame actually starts with top of stack, so currentBoard is actually inverted from what it should be
              Serial.println((String)"Num board= "+currentBoard);

              //go through each byte in the current board (12 bytes = 6 cells * 2 bytes each)
              for(i=0; i<12; i+=2)
              {
                //each board responds with 32 data bytes + 6 header bytes
                //so need to find the start of each board by doing that * currentBoard


                //convert the two individual bytes of each cell into a single 16 bit data item (by bit shifting)
                uint16_t rawData = (response_frame[i+4] << 8) | response_frame[i+5];

                //do the two's complement of the resultant 16 bit data item, and multiply by 190.73uV to get an actual voltage
                float cellVoltage = Complement(rawData,0.00019073);
                
                if(cellVoltage >= 4.2){
                  digitalWrite(BMS_OK, LOW);
                  Serial.println("Fallo de tensión");
                }
                //print the voltages - it is i/2 because cells start from 1 up to 6
                //and there are 2 bytes per cell (i value is twice the cell number),
                //and it's +1 because cell names start with "Cell1"
                Serial.println((String)"Cell " +(i/2)+" voltage= " +cellVoltage);
              }


              //go through each byte in the current board (12 bytes = 6 GPIO * 2 bytes each)
              for(i=0; i<12; i+=2)
              {
                //each board responds with 32 data bytes + 6 header bytes

                //convert the two individual bytes of each cell into a single 16 bit data item (by bit shifting)
                uint16_t rawData = (response_frame2[i+4] << 8) | response_frame2[i+5];

                //do the two's complement of the resultant 16 bit data item, and multiply by 190.73uV to get an actual voltage
                float GPIOVoltage = Complement(rawData,0.00019073);

                //print the voltages - it is i/2 because cells start from 1 up to 6
                //and there are 2 bytes per cell (i value is twice the cell number),
                //and it's +1 because cell names start with "Cell1"
                Serial.println((String)"GPIO " +(i/2)+" Voltage= " +GPIOVoltage);
              }
          }
      }

}