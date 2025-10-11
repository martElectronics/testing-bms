#include <Arduino.h>
// #include <BQ79606.h>
#include "BQ79606.h"
#include <MART_CAN.h>
#include <set>
#include <Adafruit_NeoPixel.h>
#include "SensorCorriente.cpp"


// LED
#define LED_PIN 48
#define LED_COUNT 1
Adafruit_NeoPixel pixels(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

CAN_BUS CAN(HardwareType::Transciever, MCP_SPEED_500, 2, 10);

// BMS
//  --- Configuración de Dimensiones de Arrays ---
const int MAX_MODULES = TOTALBOARDS / 2;
const int SENSORS_PER_MODULE_VOLT = 11;
const int SENSORS_PER_MODULE_TEMP = 9;

//En estos arrays se guardan las temperaturas y los voltajes de los módulos
float stsVoltCells[12][11];
float stsTempCells[12][9];

unsigned int numCRCFails = 0;
 bool holdBMSOK=true;
byte stsNumAutoadressedDevices=0;

// --- Arrays de Resultados (Fallas) ---
bool stsVoltCellsFail[MAX_MODULES][SENSORS_PER_MODULE_VOLT];
bool stsTempCellsFail[MAX_MODULES][SENSORS_PER_MODULE_TEMP];

// --- Márgenes Parametrizables ---
const float MIN_VALID_VOLTAGE = 2.8f;
const float MAX_VALID_VOLTAGE = 4.2f;
const float MIN_VALID_TEMP = 5.0f;
const float MAX_VALID_TEMP = 60.0f;



/**
Lee las tensiones de las celdas y los voltajes de los NTC y las guarda en: "stsVoltCells" y "stsTempCells". 
Devuelve:
-> 0 si no hay errores en la comunicación
-> -1 si hay errores en la comunicación (fallo de lectura)
-> 1 si hay error en la comunicación (CRC incorrecto)
*/
int readVoltages2(bool &ok);


/**
Función auxiliar que convierte el voltaje de los NTC a temperatura en grados celsius
*/
float voltToTemp(float GPIOVoltage);

/**
Analiza las tensiones y temperaturas están guardadas en memoria: "stsVoltCells" y "stsTempCells"  
y pone "ok"=0 si alguna está fuera del rango seguro (ver Márgenes Parametrizables).
También actualiza los arrays: "stsVoltCellsFail" y "stsTempCellsFail" (Indican fallos (0 o 1) de cada elemento de forma resumida)
*/
void checkFails(bool &ok);

/**
Permite configurar alguna excepción (de tensión o temperatura) para que no se detecte como fallo al realizar la lectura.
La función checkFails() la ignora a la hora de notificar si hay algún fallo o no (tampoco lo indica en los arrays de fallos)
*/
void setupExclusions();


/**
Envía el mensaje por CAN necesario al cargador para iniciar y detener la carga
*/
void controlCharge(float maxVolt, float maxCurrent, bool start);

/**
Muestra las tensiones y temperaturas detalladas de cada módulo
*/
void mostrarDatosDetallados();

/**
Muestra los datos de la carga
*/
void showChargeData();

/**
Lee el carácter introducido por el usuario por teclado para mostrar información relevante de los módulos
*/
void procesarComandoSerial(float &valorFloatRef, bool &valorBoolRef, bool &reset, bool &fail);


/**
Muestra las temperaturas detalladas de un módulo
*/
void mostrarDatosDetalladosTemperaturas(int i);

/**


//**** FUNCIONES DE DEPURACIÓN */

/**
Función para depurar que llena los arrays donde se guardan los datos de las tensiones y temperaturas con datos no reales
para comprobar que todo funciona bien
*/
void populateTestData();

/**
Imprime por el monitor serial los fallos de cada módulo (indicado por 0 o 1)
*/
void printFailResults();


/**
Imprime por el monitor serial la lista de exclusión configurada
*/
void printExclusionLists();

/**
Función que encapsula el código que se usa en la rama /main para hacer las lecturas de las tensiones de las celdas y los voltajes
Está poco optimizada y no se usa en este código
*/
void printVoltages();

/**
No usada
*/
void readVoltages();


// Define the starting address and length for the read block
#define START_ADDR_INFO 0x0200 // Starting with PARTID register
#define LEN_READ_INFO   81     // Reading up to AUX_AVAO_REFL (0x0250)
#define MAX_BUFFER_SIZE (LEN_READ_INFO + 6) 

/**
 * Reads a contiguous block of status/ADC registers from a single BQ79606A-Q1 IC
 * and prints the results to the Serial Monitor.
 *
 * @param deviceID The address of the target BQ79606A-Q1 device (0 to TOTALBOARDS-1).
 */
void readAndDisplaySingleICRegisters(uint8_t deviceID) {
    uint8_t readBuffer[MAX_BUFFER_SIZE] = {0};
    int bytesRead = 0;
    uint16_t currentAddress = START_ADDR_INFO;

    Serial.println("\n--- Reading Status and ADC Registers (0x0200 to 0x0250) ---");
    Serial.printf("Target Device ID: %u\n", deviceID);

    // 1. Execute Single Device Read for 81 bytes
    bytesRead = ReadReg(
        deviceID,
        START_ADDR_INFO,
        readBuffer,
        LEN_READ_INFO,
        0, // Timeout parameter ignored for this example
        FRMWRT_SGL_R // Single Device Read
    );

    if (bytesRead <= 0) {
        Serial.printf("ERROR: Failed to read registers. ReadReg returned %d bytes.\n", bytesRead);
        if (bytesRead == -1) {
            Serial.println("Possible: Timeout Error or No device found.");
        }
        return;
    }

    // 2. Check CRC to ensure data integrity
    if (!CheckCRC(readBuffer, bytesRead)) {
        Serial.println("ERROR: CRC check failed on received data. Data may be corrupted.");
        return;
    }

    // 3. Process and print the received data
    // The actual data starts at index 4 (after Init Byte, Device ID, and 2x Reg Address)
    Serial.println("Address | Hex Value | Register");
    Serial.println("-------------------------------------");

    for (int i = 0; i < LEN_READ_INFO; ++i) {
        uint8_t dataByte = readBuffer[4 + i]; // Data starts at buffer index 4

        // Print Address and Value
        Serial.printf("0x%04X  |  0x%02X   | ", currentAddress + i, dataByte);

        // Print Register Name (using a partial switch for illustration)
        switch(currentAddress + i) {
            case PARTID: Serial.println("PARTID (Device Revision)"); break;
            case SYS_FAULT1: Serial.println("SYS_FAULT1 (System Fault 1)"); break;
            case SYS_FAULT2: Serial.println("SYS_FAULT2 (System Fault 2)"); break;
            case DEV_STAT: Serial.println("DEV_STAT (Device Status)"); break;
            case FAULT_SUM: Serial.println("FAULT_SUM (Fault Summary)"); break;
            case VCELL1H: Serial.println("VCELL1H (Cell 1 Voltage High)"); break;
            case VCELL1L: Serial.println("VCELL1L (Cell 1 Voltage Low)"); break;
            case AUX_BATH: Serial.println("AUX_BATH (Stack Voltage High)"); break;
            case AUX_BATL: Serial.println("AUX_BATL (Stack Voltage Low)"); break;
            case AUX_AVDDH: Serial.println("AUX_AVDDH (AVDD LDO Voltage High)"); break;
            case AUX_CVDDH: Serial.println("AUX_CVDDH (CVDD LDO Voltage High)"); break;
            case AUX_AVAOH: Serial.println("AUX_AVAOH (AVAO_REF Voltage High)"); break;
            default: Serial.println(""); break;
        }
    }
    Serial.println("-------------------------------------");
    Serial.println("--- Register Read Complete ---");
}



/**
 * @brief Reads the content of a single, one-byte register from the BQ79606A-Q1
 * and prints the result to the Serial Monitor.
 *
 * @param deviceID The address of the target BQ79606A-Q1 device (0 to TOTALBOARDS-1).
 * @param regAddress The 16-bit address of the register to read (e.g., 0x0201 for SYS_FAULT1).
 */
void readAndPrintSingleRegister(uint8_t deviceID, uint16_t regAddress) {
    uint8_t readBuffer[7] = {0};
    const uint8_t dataLength = 1; // We are reading exactly one byte
    int bytesRead = 0;

    Serial.printf("--- Reading Register 0x%04X (Device ID: %u) ---\n", regAddress, deviceID);

    // 1. Execute Single Device Read (Requesting 1 byte)
    bytesRead = ReadReg(
        deviceID,
        regAddress,
        readBuffer,
        dataLength,
        0, // Timeout ignored for this example
        FRMWRT_SGL_R
    );

    if (bytesRead <= 0) {
        Serial.printf("ERROR: Failed to read register. ReadReg returned %d bytes.\n", bytesRead);
        if (bytesRead == -1) {
            Serial.println("CAUSE: Communication timeout or device non-responsive.");
        }
        return;
    }
    
    // Check for the minimum expected response size (1 data byte + 6 overhead = 7 bytes)
    if (bytesRead < 7) {
        Serial.printf("ERROR: Incomplete response. Expected at least 7 bytes, received %d.\n", bytesRead);
        return;
    }

    // 2. Check CRC to ensure data integrity
    if (!CheckCRC(readBuffer, bytesRead)) {
        Serial.println("ERROR: CRC check failed on received data. Data corrupted.");
        return;
    }

    // 3. Extract and print the data
    // The single byte of data is always at index 4 of the successful response frame.
    uint8_t registerValue = readBuffer[4]; 
    
    Serial.printf("  0x%04X: Value = 0x%02X (Binary: 0b%s)\n", 
                  regAddress, registerValue, String(registerValue, 2).c_str());
    Serial.println("--- Read Complete ---");
}




// Estructura para definir un punto de exclusión.
// Estructura y lista para excluir sensores de VOLTAJE
struct VoltExclusionPoint
{
  int idModule;
  int idVolt; // Índice del sensor de voltaje

  bool operator<(const VoltExclusionPoint &other) const
  {
    if (idModule != other.idModule)
      return idModule < other.idModule;
    return idVolt < other.idVolt;
  }
};
std::set<VoltExclusionPoint> voltExclusionList;

// Estructura y lista para excluir sensores de TEMPERATURA
struct TempExclusionPoint
{
  int idModule;
  int idNTC; // Índice del sensor de temperatura (NTC)

  bool operator<(const TempExclusionPoint &other) const
  {
    if (idModule != other.idModule)
      return idModule < other.idModule;
    return idNTC < other.idNTC;
  }
};
std::set<TempExclusionPoint> tempExclusionList;

//******CHARGE    */
unsigned long idStsCharger = 0x18FF50E5;
unsigned long idCmdCharger = 0x1806E5F4;
unsigned long idPrueba = 0x30;
byte stsChargerByte[8];
byte cmdChargerByte[8];
byte cmdPrueba[8];
const int PIN_CURRENT_1 = 14;
SensorCorriente sensor1;
double stsCorrienteCarga=0;





void setup()
{

  bool ok = false;
  
  configBMS();
  digitalWrite(BMS_OK, true);
  while (CAN.error == 1)
  {
    Serial.println("Error Initializing EScP32Can...");
  }
  Serial.println("CAN OK");
  bool okk = false;
  readVoltages2(okk);
  checkFails(okk);
  printFailResults();
  //setupExclusions();
  //printExclusionLists();

  pixels.clear(); // Set all pixel colors to 'off'

  Serial.println((String)"INSTRUCCIONES DE USO DEL PROGRAMA:  - Pulsar 'i' para mostrar los voltajes y temperaturas de todos los módulos. Pulsar 'l' para mostrar fallos de tensión y temperatura" );
}

void loop()
{
  unsigned long int tTotal=millis();
  static bool cmdCharge = false;
  static bool stsNumBytesOK = false;
  static bool stsVoltagesOK = false;
  static bool stsStartCharge = false;
  static bool stsAMPOK = false;
  static bool cmdResetFail = false;
  static bool stsFail = false;
  static float corrienteCarga = 0;
  static bool flagShow = false;
  bool stsChargerOK = false;
  


  CAN.receive();
  CAN.getPacket(idStsCharger, stsChargerByte, 8, false);
  stsChargerOK = (stsChargerByte[4] == 0);
  stsStartCharge=!digitalRead(START_PIN);


 //0.85V lectura 1.73V real a 
 //1.73V a 5V   R=2.89
  int adcCurrentValue=analogRead(AMP_PIN);
  stsCorrienteCarga=sensor1.calcularCorrienteS1(adcCurrentValue);
  stsAMPOK=(sensor1.getVoltaje(adcCurrentValue)>0.5);
 // stsAMPOK=true;

 static int contFails=0;
   int resReadVoltages=readVoltages2(stsNumBytesOK);
  if(resReadVoltages==0)
  {
    checkFails(stsVoltagesOK);
  }
  else if(resReadVoltages==1)
  {
    numCRCFails++;
    Serial.println("CRC error");
  }
  else if(resReadVoltages==-1)
  {
    contFails++;
    Serial.print("*********CONT = ");
    Serial.println(contFails);
    if(contFails>5)
    {
      //configBMS();
      contFails=0;
    }
  }

  procesarComandoSerial(corrienteCarga, cmdCharge, cmdResetFail, stsNumBytesOK);

  bool failCondition = !(stsVoltagesOK && stsNumBytesOK && stsAMPOK);
  //failCondition=false;
  // failCondition = false;

  // if (failCondition)
  // {
  //   stsFail = 1;
  // }
  // else if (!failCondition && cmdResetFail)
  // {
  //   cmdResetFail = false;
  //   stsFail = 0;
  // }
  stsFail=failCondition;
  //stsFail=0;

  if(!holdBMSOK)
  {

 
  if (stsFail)
  {
    cmdCharge = 0;
    corrienteCarga = 0;
   digitalWrite(BMS_OK, false);
  }
  else
  {
    digitalWrite(BMS_OK, true);
    flagShow = 0;
  }
}
else
{
  digitalWrite(BMS_OK, true);
}

  controlCharge(350, corrienteCarga, cmdCharge);

  CAN.send();

  // MOSTRAR UNA SOLA VEZ POR SERIAL CADA VEZ QUE FALLE
  if (stsFail && !flagShow)
  {
    printFailResults();
    flagShow = 1;
  }

  static int t = millis();
  if ((millis() - t) >= 1000)
  {
    // Serial.println((String)"I= "+corrienteCarga+" cmdCharge= "+cmdCharge +" reset= "+cmdResetFail+ " ok= "+stsVoltagesOK);
    // showChargeData();
    // readVoltages(stsVoltagesOK);
    // CAN.printByteArray(stsChargerByte,8);
    // printVoltages();
    // Serial.println((String)"I= "+corrienteCarga+" cmdCharge= "+cmdCharge +" reset= "+cmdResetFail+ " ok= "+stsVoltagesOK);
    t = millis();
    //bool failCondition = !(stsVoltagesOK && stsNumBytesOK && stsAMPOK);
    //  Serial.println((String)"stsVoltagesOK= "+stsVoltagesOK+" stsNumBytesOK= "+stsNumBytesOK +" stsAMPOK= "+stsAMPOK);
    //  Serial.println((String)"Current= "+stsCorrienteCarga+" adc voltage = "+sensor1.getVoltaje(adcCurrentValue));
    //  Serial.println(sensor1.getVoltaje(adcCurrentValue),6);
    // Serial.println((String)"start_charge"+stsStartCharge);
     //Serial.println((String)"FAIL: "+ stsFail);
    // CAN.printByteArray(stsChargerByte,8);
   // Serial.println(t);
  }

  if (((millis() - t) >= 1000) && !stsFail)
  {
    // showChargeData();
  }

  if (!stsChargerOK)
  {
    pixels.setPixelColor(0, pixels.Color(255, 0, 0));
    pixels.show(); // Send the updated pixel colors to the hardware.
  }
  else if (stsChargerOK && !cmdCharge)
  {
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
    pixels.show(); // Send the updated pixel colors to the hardware.
  }
  else if (stsChargerOK && cmdCharge)
  {
    pixels.setPixelColor(0, pixels.Color(0, 255, 0));
    pixels.show(); // Send the updated pixel colors to the hardware.
  }
 // CAN.printReceivedIds();
  //CAN.printByteArray(stsChargerByte,8);
 // Serial.println(millis()-tTotal);

 //Serial.println(stsTempCells[0][1]);
 mostrarDatosDetalladosTemperaturas(0);
}

void debug()
{
  CAN.printByteArray(stsChargerByte, 8);
  // Serial.println((String)"I= "+corrienteCarga+" cmdCharge= "+cmdCharge +" reset= "+cmdResetFail);
}

/**
 * @brief Espera y procesa comandos desde el puerto serie.
 * @param valorFloatRef Referencia a la variable float que se modificará con el comando 'c'.
 * @param valorBoolRef Referencia a la variable bool que se modificará con el comando 'c'.
 */
void procesarComandoSerial(float &valorFloatRef, bool &valorBoolRef, bool &reset, bool &fail)
{
  // Solo procesa si hay datos disponibles en el buffer del puerto serie
  if (Serial.available() > 0)
  {
    // Lee la cadena completa hasta que encuentra un salto de línea
    String comando = Serial.readStringUntil('\n');
    comando.trim(); // Elimina espacios en blanco o carácteres invisibles al inicio/final

    // --- Comando 'c': configurar valores ---
    // Formato esperado: "c,valor_float,valor_bool" (ej: "c,123.45,1")
    if (comando.startsWith("c,"))
    {
      // Busca la posición de la primera y segunda coma
      int primeraComa = comando.indexOf(',');
      int segundaComa = comando.indexOf(',', primeraComa + 1);

      // Si encontramos ambas comas, el formato es potencialmente correcto
      if (segundaComa > primeraComa)
      {
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
      }
      else
      {
        // Error si el formato no es el esperado
        Serial.println(F("-> ERROR: Formato de comando 'c' incorrecto. Use: c,float,bool"));
      }

      // --- Comando 'i': ejecutar función I ---
    }
    else if (comando == "i")
    {
      bool stsNumBytesOK;
      bool stsVoltagesOK;
      int resReadVoltages = readVoltages2(stsNumBytesOK);
      if (resReadVoltages == 0)
      {
        checkFails(stsVoltagesOK);
         mostrarDatosDetallados();
      }
      else if (resReadVoltages == 1)
      {
        numCRCFails++;
        Serial.println("CRC error");
      }
      else
      {
        Serial.println("Read error");
      }

     
    }
    else if (comando == "l")
    {

      printFailResults();

      // --- Comando 'r': ejecutar función R ---
    }
    else if (comando == "r")
    {
      reset = 1;
    }
    else if (comando == "f")
    {
      fail = 0;
    }
    else if (comando == "p")
    {
      printVoltages();

      // --- Comando desconocido ---
    }
    else if (comando == "s")
    {
      // bool oko;
      // readVoltages(oko);
      // checkFails(oko);
      // setupExclusions();
      // printExclusionLists();
      holdBMSOK=!holdBMSOK;
      Serial.println((String)"hold bms = "+holdBMSOK);

      // --- Comando desconocido ---
    }
    else if (comando == "d")
    {
      showChargeData();
      // --- Comando desconocido ---
    }

    else if (comando == "a")
    {
      Serial.println("AAAAAAAAAAAA");
      configBMS();
      // --- Comando desconocido ---
    }
    else if (comando == "b")
    {
      Serial.println("BBBBBBBBBBBBBBBBBBBBBBBBB");
      CommSleepToWake();
      // --- Comando desconocido ---
    }

    else if (comando == "v")
    {
      readAndDisplaySingleICRegisters(0);
      // --- Comando desconocido ---
    }

    else if (comando == "w")
    {
      readAndPrintSingleRegister(2,0x0023);
      // --- Comando desconocido ---
    }
    

    else if (comando.length() > 0)
    {
      Serial.print(F("-> ERROR: Comando desconocido: '"));
      Serial.print(comando);
      Serial.println(F("'"));
    }
    

    Serial.println("i = mostrar datos módulos, f= forzar fallo, r=reset fallo,l= lista de fallos, s= crear lista de exclusión");
  }
}

void mostrarDatosDetalladosTemperaturas(int i)
{
  for (int j = 0; j < 9; j++)
    {
      Serial.print((String) (j+1) + "-> "+stsTempCells[i][j]); // Imprime con 1 decimal
      Serial.print("   ");
    }
    Serial.println();
}


void mostrarDatosDetallados()
{


  Serial.println(F("\n--- Vista Detallada por Modulo ---")); // F() macro ahorra RAM
  Serial.println(F("Modulo | Voltajes (V)                  | Temperaturas (C)"));
  Serial.println(F("            1    2    3    4    5    6    7    8    9   10    11        1    2    3    4    5    6    7    8    9  "));
  Serial.println(F("-----------------------------------------------------------------------------------------------------------------"));

  // Bucle a través de cada módulo
  for (int i = 0; i < TOTALBOARDS / 2; i++)
  {
    // Imprime el identificador del módulo (ej: M00, M01, ... M11)
    Serial.print("M");
    if (i < 9)
    {
      Serial.print("0"); // Añade un cero para alinear M0 a M9 con M10 y M11
    }
    Serial.print(i+1);
    Serial.print("   | V: ");

    // Imprime los 11 voltajes de celda para el módulo actual
    for (int j = 0; j < 11; j++)
    {
      Serial.print(stsVoltCells[i][j], 2); // Imprime con 2 decimales
      Serial.print(" ");
    }

    Serial.print("| T: ");

    // Imprime las 9 temperaturas para el módulo actual
    for (int j = 0; j < 9; j++)
    {
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
  float minVolt = 5.0; // Iniciar con un valor alto
  float maxVolt = 0.0; // Iniciar con un valor bajo
  int moduloMinVolt = 0;
  int moduloMaxVolt = 0;
  float sumaTotalVolt = 0.0;

  float minTemp = 100.0; // Iniciar con un valor alto
  float maxTemp = -20.0; // Iniciar con un valor bajo
  int moduloMinTemp = 0;
  int moduloMaxTemp = 0;
  float sumaTotalTemp = 0.0;

  // --- Procesamiento del array de Voltajes ---
  for (int i = 0; i < TOTALBOARDS / 2; i++)
  { // Bucle por cada Módulo
    for (int j = 0; j < 11; j++)
    { // Bucle por cada Celda de voltaje
      float v = stsVoltCells[i][j];
      if (v < minVolt)
      {
        minVolt = v;
        moduloMinVolt = i;
      }
      if (v > maxVolt)
      {
        maxVolt = v;
        moduloMaxVolt = i;
      }
      sumaTotalVolt += v;
    }
  }

  // --- Procesamiento del array de Temperaturas ---
  for (int i = 0; i < TOTALBOARDS / 2; i++)
  { // Bucle por cada Módulo
    for (int j = 0; j < 9; j++)
    { // Bucle por cada Sensor de temperatura
      float t = stsTempCells[i][j];
      if (t < minTemp)
      {
        minTemp = t;
        moduloMinTemp = i;
      }
      if (t > maxTemp)
      {
        maxTemp = t;
        moduloMaxTemp = i;
      }
      sumaTotalTemp += t;
    }
  }

  // --- Cálculo final de la temperatura media ---
  float tempMedia = sumaTotalTemp / ((TOTALBOARDS / 2.0) * 9.0);

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

  Serial.println((String)"Charge Current :" +stsCorrienteCarga);
}

float voltToTemp(float GPIOVoltage)
{
  float R_NTC = (1.952e9 * GPIOVoltage) / (200000.0 * (2.5 - GPIOVoltage) - 9760.0 * GPIOVoltage);
  float R_NTC_2 = R_NTC / 1000.0f; // Convert to kOhms
  float TEMP = -7.388512707e-02f * R_NTC_2 * R_NTC_2 * R_NTC_2 + 1.987401122e+00f * R_NTC_2 * R_NTC_2 + -1.975941021e+01f * R_NTC_2 + 9.894631155e+01f;
  return TEMP;
}

void controlCharge(float maxVolt, float maxCurrent, bool start)
{
  // These will hold the final two-byte values for voltage and current.
  uint8_t volt_high_byte, volt_low_byte;
  uint8_t curr_high_byte, curr_low_byte;

  if (!start)
  {
    // If start is false, we stop the charging process by setting
    // voltage and current to 0.
    volt_high_byte = 0x00;
    volt_low_byte = 0x00;
    curr_high_byte = 0x00;
    curr_low_byte = 0x00;
  }
  else
  {
    const float VOLTAGE_SCALING_FACTOR = 10.0f;
    uint16_t scaled_voltage = static_cast<uint16_t>(maxVolt * VOLTAGE_SCALING_FACTOR);

    volt_high_byte = (scaled_voltage >> 8) & 0xFF; // Right shift by 8 bits to get the MSB
    volt_low_byte = scaled_voltage & 0xFF;         // AND with 0xFF to get the LSB

    const float CURRENT_SCALING_FACTOR = 10.0f;
    uint16_t scaled_current = static_cast<uint16_t>(maxCurrent * CURRENT_SCALING_FACTOR);

    // Extract the high and low bytes from the 16-bit scaled current.
    curr_high_byte = (scaled_current >> 8) & 0xFF;
    curr_low_byte = scaled_current & 0xFF;

    byte cmdChargerByte[8];
  }

  for (int i = 0; i < 8; i++)
  {
    cmdChargerByte[i] = 0x00;
  }
  cmdChargerByte[0] = volt_high_byte;
  cmdChargerByte[1] = volt_low_byte;
  cmdChargerByte[2] = curr_high_byte;
  cmdChargerByte[3] = curr_low_byte;
  cmdChargerByte[4] = (byte)!start;

  if (maxVolt > 500)
    maxVolt = 500;
  CAN.setPacket(idCmdCharger, cmdChargerByte, 8, false);
}

/**
 * @brief Recorre los arrays de lecturas, las valida contra márgenes
 * y actualiza los arrays de fallas, respetando las listas de exclusión.
 */
void checkFails(bool &ok)
{
  ok = true;
  // --- 1. Comprobación de Voltajes ---
  for (int i = 0; i < MAX_MODULES; ++i)
  {
    for (int j = 0; j < SENSORS_PER_MODULE_VOLT; ++j)
    {

      VoltExclusionPoint pointToCheck = {.idModule = i, .idVolt = j};

      // Verificamos si el punto actual está en la lista de exclusión de voltajes
      if (voltExclusionList.find(pointToCheck) == voltExclusionList.end())
      {
        // NO está excluido: Procedemos a la validación
        float currentVoltage = stsVoltCells[i][j];

        if (currentVoltage < MIN_VALID_VOLTAGE || currentVoltage > MAX_VALID_VOLTAGE)
        {
          stsVoltCellsFail[i][j] = true; // El valor está fuera de rango -> FALLA
          ok = false;
        }
        else
        {
          stsVoltCellsFail[i][j] = false; // El valor está en rango -> OK
        }
      }
      else
      {
        // SÍ está excluido: Lo marcamos como "no fallido" por defecto.
        // No queremos falsas alarmas de sensores que hemos decidido ignorar.
        stsVoltCellsFail[i][j] = false;
      }
    }
  }

  // --- 2. Comprobación de Temperaturas ---
  for (int i = 0; i < MAX_MODULES; ++i)
  {
    for (int j = 0; j < SENSORS_PER_MODULE_TEMP; ++j)
    {

      TempExclusionPoint pointToCheck = {.idModule = i, .idNTC = j};

      // Verificamos si el punto actual está en la lista de exclusión de temperaturas
      if (tempExclusionList.find(pointToCheck) == tempExclusionList.end())
      {
        // NO está excluido: Procedemos a la validación
        float currentTemp = stsTempCells[i][j];

        if (currentTemp < MIN_VALID_TEMP || currentTemp > MAX_VALID_TEMP)
        {
          stsTempCellsFail[i][j] = true; // El valor está fuera de rango -> FALLA
          ok = false;
        }
        else
        {
          stsTempCellsFail[i][j] = false; // El valor está en rango -> OK
        }
      }
      else
      {
        // SÍ está excluido: Lo marcamos como "no fallido" por defecto.
        stsTempCellsFail[i][j] = false;
      }
    }
  }
}

// Returns 0 if ok, -1 if crc ok and voltage fail, 1 if crc fail
int readVoltages2(bool &ok)
{
  int t = millis();
  ok = true;
  int returnValue = 0;
  // VARIABLES
  byte response_frame[(MAXBYTES + 6)];
  byte response_frame2[(MAXBYTES + 6)];
  int currentBoard = 0;
  int res1 = 0, res2 = 0;
  int i = 0;
  int contVoltCells = 0, contVoltNTC = 0, idModule = 0;
  // reset variables
  memset(response_frame, 0, sizeof(response_frame));
  i = 0;
  currentBoard = 0;
  WriteReg(0, CONTROL2, 0x13, 1, FRMWRT_ALL_NR);
 delay(10);

  // PARSE, FORMAT, AND PRINT THE DATA
  for (currentBoard = 0; currentBoard < TOTALBOARDS; currentBoard++)
  {
    idModule = currentBoard / 2;
    memset(response_frame, 0, sizeof(response_frame));
    memset(response_frame2, 0, sizeof(response_frame));

    // read back data (6 cells and 2 bytes each cell)
    res1 = ReadReg(currentBoard, VCELL1H, response_frame, MAXBYTES, 0, FRMWRT_SGL_R);
    res2 = ReadReg(currentBoard, AUX_GPIO1H, response_frame2, MAXBYTES, 0, FRMWRT_SGL_R);
    bool okCRC = false;
    bool okCRC2 = false;

    if ((res1 <= 0) || (res2 <= 0))
    { // Fallo de lectura
      ok = false;
      returnValue = -1;
    }
    else
    {
      okCRC = CheckCRC(response_frame, sizeof(response_frame));
      okCRC2 = CheckCRC(response_frame2, sizeof(response_frame2));
      if (!okCRC || !okCRC2)
      {
        // CRC incorrecto
        ok = false;
        returnValue = 1;
      }
      else
      {
        // *********************** CRC correcto
        //  Cambio al módulo siguiente, reset de contadores
        if ((currentBoard % 2) == 0)
        {
          contVoltCells = 0;
          contVoltNTC = 0;
        }
        // response frame actually starts with top of stack, so currentBoard is actually inverted from what it should be
        // go through each byte in the current board (12 bytes = 6 cells * 2 bytes each)
        for (i = 0; i < 12; i += 2)
        {
          uint16_t rawData = (response_frame[i + 4] << 8) | response_frame[i + 5];
          float cellVoltage = Complement(rawData, 0.00019073);
          if (contVoltCells < 11)
          {
            stsVoltCells[idModule][contVoltCells] = cellVoltage;
            contVoltCells++;
          }
        }

        // go through each byte in the current board (12 bytes = 6 GPIO * 2 bytes each)
        for (i = 0; i < 12; i += 2)
        {
          // each board responds with 32 data bytes + 6 header bytes

          // convert the two individual bytes of each cell into a single 16 bit data item (by bit shifting)
          uint16_t rawData = (response_frame2[i + 4] << 8) | response_frame2[i + 5];

          // do the two's complement of the resultant 16 bit data item, and multiply by 190.73uV to get an actual voltage
          float GPIOVoltage = Complement(rawData, 0.00019073);
          // GPIOVoltage=1.08;
          // Serial.println((String)"GPIO " +(i/2)+" Voltage= " +GPIOVoltage);
          float temp = voltToTemp(GPIOVoltage);
          // if(temp >= 60 ){
          //   ok=false;
          // }
          if (contVoltNTC < 9)
          {

            // Serial.println(contVoltNTC);
            stsTempCells[idModule][contVoltNTC] = temp;
            contVoltNTC++;
          }
        }
      }
    }
  }
  int tTotal = millis() - t;
  return returnValue;
}

void readVoltages(bool &ok)

{
  int t = millis();
  ok = true;
  // VARIABLES
  byte response_frame[(MAXBYTES + 6)];
  byte response_frame2[(MAXBYTES + 6)];
  int currentBoard = 0;
  int res1 = 0, res2 = 0;
  int i = 0;
  int contVoltCells = 0, contVoltNTC = 0, idModule = 0;
  // reset variables
  memset(response_frame, 0, sizeof(response_frame));
  i = 0;
  currentBoard = 0;
  WriteReg(0, CONTROL2, 0x13, 1, FRMWRT_ALL_NR);
  delay(100);

  // PARSE, FORMAT, AND PRINT THE DATA
  for (currentBoard = 0; currentBoard < TOTALBOARDS; currentBoard++)
  {
    idModule = currentBoard / 2;
    memset(response_frame, 0, sizeof(response_frame));
    memset(response_frame2, 0, sizeof(response_frame));
    // read back data (6 cells and 2 bytes each cell)
    res1 = ReadReg(currentBoard, VCELL1H, response_frame, MAXBYTES, 0, FRMWRT_SGL_R);
    res2 = ReadReg(currentBoard, AUX_GPIO1H, response_frame2, MAXBYTES, 0, FRMWRT_SGL_R);
    if ((res1 <= 0) || (res2 <= 0))
    {
      ok = false;
      Serial.println("Error de lectura numBytes=0");
    }

    // Cambio al módulo siguiente, reset de contadores
    if ((currentBoard % 2) == 0)
    {
      contVoltCells = 0;
      contVoltNTC = 0;
    }
    // response frame actually starts with top of stack, so currentBoard is actually inverted from what it should be
    // go through each byte in the current board (12 bytes = 6 cells * 2 bytes each)
    for (i = 0; i < 12; i += 2)
    {
      uint16_t rawData = (response_frame[i + 4] << 8) | response_frame[i + 5];
      float cellVoltage = Complement(rawData, 0.00019073);
      // cellVoltage=4;

      // if(cellVoltage >= 4.2 || cellVoltage<=2.5){
      //   ok=false;
      // }

      if (contVoltCells < 11)
      {
        stsVoltCells[idModule][contVoltCells] = cellVoltage;
        contVoltCells++;
      }
    }

    // go through each byte in the current board (12 bytes = 6 GPIO * 2 bytes each)
    for (i = 0; i < 12; i += 2)
    {
      // each board responds with 32 data bytes + 6 header bytes

      // convert the two individual bytes of each cell into a single 16 bit data item (by bit shifting)
      uint16_t rawData = (response_frame2[i + 4] << 8) | response_frame2[i + 5];

      // do the two's complement of the resultant 16 bit data item, and multiply by 190.73uV to get an actual voltage
      float GPIOVoltage = Complement(rawData, 0.00019073);
      // GPIOVoltage=1.08;
      // Serial.println((String)"GPIO " +(i/2)+" Voltage= " +GPIOVoltage);
      float temp = voltToTemp(GPIOVoltage);
      // if(temp >= 60 ){
      //   ok=false;
      // }
      if (contVoltNTC < 9)
      {

        // Serial.println(contVoltNTC);
        stsTempCells[idModule][contVoltNTC] = temp;
        contVoltNTC++;
      }
      // print the voltages - it is i/2 because cells start from 1 up to 6
      // and there are 2 bytes per cell (i value is twice the cell number),
      // and it's +1 because cell names start with "Cell1"
    }
  }

  int tTotal = millis() - t;
  // Serial.println(tTotal);
}

void printVoltages()
{

  delay(10);
  // VARIABLES
  byte response_frame[(MAXBYTES + 6)];
  byte response_frame2[(MAXBYTES + 6)];
  int currentBoard = 0;
  int Bytesleidos = 0;
  int i = 0;
  // reset variables
  memset(response_frame, 0, sizeof(response_frame));
  i = 0;
  currentBoard = 0;
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

  if (Bytesleidos == -1)
  {
    Serial.println("No se ha podido leer los datos, se ha excedido el tiempo");
    // delay(1000);
  }
  else
  {

    // PARSE, FORMAT, AND PRINT THE DATA
    for (currentBoard = 0; currentBoard < TOTALBOARDS; currentBoard++)
    {
      memset(response_frame, 0, sizeof(response_frame));
      memset(response_frame2, 0, sizeof(response_frame));
      // read back data (6 cells and 2 bytes each cell)
      Bytesleidos = ReadReg(currentBoard, VCELL1H, response_frame, MAXBYTES, 0, FRMWRT_SGL_R);
      Bytesleidos = ReadReg(currentBoard, AUX_GPIO1H, response_frame2, MAXBYTES, 0, FRMWRT_SGL_R);
      // response frame actually starts with top of stack, so currentBoard is actually inverted from what it should be
      Serial.println((String) "Num board= " + currentBoard);

      // go through each byte in the current board (12 bytes = 6 cells * 2 bytes each)
      for (i = 0; i < 12; i += 2)
      {
        // each board responds with 32 data bytes + 6 header bytes
        // so need to find the start of each board by doing that * currentBoard

        // convert the two individual bytes of each cell into a single 16 bit data item (by bit shifting)
        uint16_t rawData = (response_frame[i + 4] << 8) | response_frame[i + 5];

        // do the two's complement of the resultant 16 bit data item, and multiply by 190.73uV to get an actual voltage
        float cellVoltage = Complement(rawData, 0.00019073);

        if (cellVoltage >= 4.2)
        {
          digitalWrite(BMS_OK, LOW);
          Serial.println("Fallo de tensión");
        }
        // print the voltages - it is i/2 because cells start from 1 up to 6
        // and there are 2 bytes per cell (i value is twice the cell number),
        // and it's +1 because cell names start with "Cell1"
        Serial.println((String) "Cell " + (i / 2) + " voltage= " + cellVoltage);
      }

      // go through each byte in the current board (12 bytes = 6 GPIO * 2 bytes each)
      for (i = 0; i < 12; i += 2)
      {
        // each board responds with 32 data bytes + 6 header bytes

        // convert the two individual bytes of each cell into a single 16 bit data item (by bit shifting)
        uint16_t rawData = (response_frame2[i + 4] << 8) | response_frame2[i + 5];

        // do the two's complement of the resultant 16 bit data item, and multiply by 190.73uV to get an actual voltage
        float GPIOVoltage = Complement(rawData, 0.00019073);

        // print the voltages - it is i/2 because cells start from 1 up to 6
        // and there are 2 bytes per cell (i value is twice the cell number),
        // and it's +1 because cell names start with "Cell1"
        Serial.println((String) "GPIO " + (i / 2) + " Voltage= " + GPIOVoltage);
      }
    }
  }
}

void configBMS()
{
  bool ok = false;
  Ini_ESP();
  Serial.println(" START AUTOADRESSING");
  while (!ok)
  {
    Wake79606();
    delay(200);
    CommReset(BAUDRATE);
    delay(200);
    ok = AutoAddress();
  }


  int nCurrentBoard = 0;
  byte response_frame[(MAXBYTES + 6)];
  byte response_frame2[(MAXBYTES + 6)];

  InitDevices();

  //WriteReg(0, SYSFLT1_FLT_RST, 0xFFFFFF, 3, FRMWRT_ALL_NR); // reset system faults
  //WriteReg(0, SYSFLT1_FLT_MSK, 0xFFFFFF, 3, FRMWRT_ALL_NR);
  WriteReg(0, CONTROL2, 0x10, 1, FRMWRT_ALL_NR); // tsref activo

  // SET UP MAIN ADC
  WriteReg(0, CELL_ADC_CTRL, 0x3F, 1, FRMWRT_ALL_NR);  // enable conversions for all cells
  WriteReg(0, CELL_ADC_CONF2, 0x08, 1, FRMWRT_ALL_NR); // set continuous ADC conversions, and set minimum conversion interval

  WriteReg(0, GPIO1_CONF, 0x20, 1, FRMWRT_ALL_NR); // GPIO is an input
  WriteReg(0, GPIO2_CONF, 0x20, 1, FRMWRT_ALL_NR); // GPIO is an input
  WriteReg(0, GPIO3_CONF, 0x20, 1, FRMWRT_ALL_NR); // GPIO is an input
  WriteReg(0, GPIO4_CONF, 0x20, 1, FRMWRT_ALL_NR); // GPIO is an input
  WriteReg(0, GPIO5_CONF, 0x20, 1, FRMWRT_ALL_NR); // GPIO is an input
  WriteReg(0, GPIO6_CONF, 0x20, 1, FRMWRT_ALL_NR); // GPIO is an input

  WriteReg(0, AUX_ADC_CTRL1, 0xF0, 1, FRMWRT_ALL_NR); // GPIO is an input
  WriteReg(0, AUX_ADC_CTRL2, 0x03, 1, FRMWRT_ALL_NR); // GPIO is an input

  WriteReg(0, CONTROL2, 0x13, 1, FRMWRT_ALL_NR); // CELL_ADC_GO = 1 Y tsref y AUX_ADC_GO = 1

  delay(3 * TOTALBOARDS + 901); // 3us of re-clocking delay per board + 901us waiting for first ADC conversion to complete
}

void setupExclusions()
{
  Serial.println("Configurando listas de exclusión...");
  // Excluir sensor de voltaje del módulo 0, sensor 5
  voltExclusionList.insert({.idModule = 0, .idVolt = 5});
  // Excluir sensor de voltaje del módulo 2, sensor 10
  voltExclusionList.insert({.idModule = 2, .idVolt = 10});

  // Excluir sensor de temperatura del módulo 1, sensor 1
  tempExclusionList.insert({.idModule = 1, .idNTC = 1});
  Serial.printf("Exclusiones configuradas: %u de voltaje, %u de temperatura.\n", voltExclusionList.size(), tempExclusionList.size());

  if (true)
  {
    // --- 1. Comprobación de Voltajes ---
    for (int i = 0; i < MAX_MODULES; ++i)
    {
      for (int j = 0; j < SENSORS_PER_MODULE_VOLT; ++j)
      {
        if (stsVoltCellsFail[i][j])
        {
          voltExclusionList.insert({.idModule = i, .idVolt = j});
        }
      }
    }
    // --- 2. Comprobación de Temperaturas ---
    for (int i = 0; i < MAX_MODULES; ++i)
    {
      for (int j = 0; j < SENSORS_PER_MODULE_TEMP; ++j)
      {
        if (stsTempCellsFail[i][j])
        {
          tempExclusionList.insert({.idModule = i, .idNTC = j});
        }
      }
    }
  }
}

void populateTestData()
{
  Serial.println("Poblando arrays con datos de prueba...");
  // Llenar todo con valores válidos por defecto
  for (int i = 0; i < MAX_MODULES; ++i)
    for (int j = 0; j < SENSORS_PER_MODULE_VOLT; ++j)
      stsVoltCells[i][j] = 3.8f;
  for (int i = 0; i < MAX_MODULES; ++i)
    for (int j = 0; j < SENSORS_PER_MODULE_TEMP; ++j)
      stsTempCells[i][j] = 25.0f;

  // Insertar algunos valores que deberían fallar
  stsVoltCells[0][2] = 2.9f;  // Falla (bajo voltaje)
  stsVoltCells[1][8] = 4.5f;  // Falla (alto voltaje)
  stsTempCells[0][0] = 70.0f; // Falla (alta temperatura)

  // Insertar un valor fuera de rango en una posición EXCLUIDA
  // Este NO debería aparecer como una falla.
  stsVoltCells[0][5] = 1.5f;  // Excluido, no debe fallar
  stsTempCells[1][1] = 99.0f; // Excluido, no debe fallar
}

void printFailResults()
{
  int contFallosTension = 0;
  int contFallosTemp = 0;
  Serial.println("\n--- Resultados de Fallas de Voltaje (1=FAIL, 0=OK) ---");
  for (int i = 0; i < MAX_MODULES; ++i)
  {
    Serial.printf(" Modulo %02d: ", i);
    for (int j = 0; j < SENSORS_PER_MODULE_VOLT; ++j)
    {
      if (stsVoltCellsFail[i][j])
        contFallosTension++;
      Serial.print(stsVoltCellsFail[i][j]);
    }
    Serial.println();
  }

  Serial.println("\n--- Resultados de Fallas de Temperatura (1=FAIL, 0=OK) ---");
  for (int i = 0; i < MAX_MODULES; ++i)
  {
    Serial.printf(" Modulo %02d: ", i);
    for (int j = 0; j < SENSORS_PER_MODULE_TEMP; ++j)
    {
      if (stsTempCellsFail[i][j])
        contFallosTemp++;
      Serial.print(stsTempCellsFail[i][j]);
    }
    Serial.println();
  }
  Serial.println((String) "Total fallos tension = " + contFallosTension);
  Serial.println((String) "Total fallos temperatura = " + contFallosTemp);
  Serial.println((String) "Total fallos CRC = " + numCRCFails);
}

void printExclusionLists()
{
  Serial.println("\n--- Contenido de la Lista de Exclusión de Voltaje ---");

  if (voltExclusionList.empty())
  {
    Serial.println("-> La lista está vacía.");
  }
  else
  {
    // Iteramos sobre cada elemento del set usando un bucle for-each
    for (const auto &point : voltExclusionList)
    {
      Serial.printf("-> Excluido: Módulo %d, Sensor de Voltaje %d\n", point.idModule, point.idVolt);
    }
  }

  Serial.println("\n--- Contenido de la Lista de Exclusión de Temperatura ---");

  if (tempExclusionList.empty())
  {
    Serial.println("-> La lista está vacía.");
  }
  else
  {
    // Iteramos sobre cada elemento del set
    for (const auto &point : tempExclusionList)
    {
      Serial.printf("-> Excluido: Módulo %d, Sensor de Temperatura %d\n", point.idModule, point.idNTC);
    }
  }
}