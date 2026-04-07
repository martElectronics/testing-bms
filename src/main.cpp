#include <Arduino.h>
// #include <BQ79606.h>
#include "BQ79606.h"
#include <MART_CAN.h>
#include <global.h>
#include "SensorCorriente.cpp"
#include <esp_task_wdt.h>

// ============================================================
// PINES (AÑADIDOS DESDE LA LÓGICA NUEVA DE AIRs)
// ============================================================
constexpr uint8_t PIN_AIR_PLUS_CTRL   = 38;
constexpr uint8_t PIN_AIR_MINUS_CTRL  = 35;

constexpr uint8_t PIN_SDC_END_PRECH   = 3;
constexpr uint8_t PIN_AIRS_NO_STATE   = 5;
constexpr uint8_t PIN_SDC_BMS_IMD     = 1;

constexpr uint8_t PIN_TSON            = 2;   // Pulsador TSON (PULLDOWN)

constexpr int N_MUESTRAS = 20;


// LED
#define LED_PIN 48
#define LED_COUNT 1

CAN_BUS CAN(HardwareType::Transciever, 500, 2);

//******BMS
//  --- Configuración de Dimensiones de Arrays ---
const int MAX_MODULES = TOTALBOARDS / 2;
const int SENSORS_PER_MODULE_VOLT = 11;
const int SENSORS_PER_MODULE_TEMP = 9;

// En estos arrays se guardan las temperaturas y los voltajes de los módulos
float stsVoltCells[12][11];
float stsVoltCellsMin[12];
float stsVoltCellsMax[12];
float stsTempCells[12][9];
float stsTempCellsMin[12];
float stsTempCellsMax[12];
float minVolt = 5.0; // Iniciar con un valor alto
float maxVolt = 0.0; // Iniciar con un valor bajo
float minTemp = 100.0; // Iniciar con un valor alto
float maxTemp = -20.0; // Iniciar con un valor bajo

// Estado de la comunicación
unsigned int numCRCFails = 0;
unsigned int numCOMMFails = 0;
unsigned int numResets = 0;
bool holdBMSOK = false; //!!!!!! CUIDADO !!! Si esta variable está a true, se ignoran los fallos del BMS
byte stsNumAutoadressedDevices = 0;
byte stsNumAutoAdressingAttempts = 0;
bool stsAutoadressingOK = false;
uint16_t stsNumTriesToResetComm = false;
#define NUM_TRIES_TO_RESET_COMM 5

// Estas variables guardan el estado de fallo aunque la causa ya no esté presente, para ponerlos a '0', pulsar la 'r'
byte stsFailCommLatch = 0, stsFailVoltLatch = 0, stsFailAmpLatch = 0;

//Variables de comprobación
uint16_t stsLastTotalFailTime=0;

// --- Márgenes Parametrizables ---
const float MIN_VALID_VOLTAGE = 2.8f;
const float MAX_VALID_VOLTAGE = 4.2f;
const float MIN_VALID_TEMP = -300.0f;
const float MAX_VALID_TEMP = 40.0f;

// --- Arrays de Resultados (Fallas) ---
bool stsVoltCellsFail[MAX_MODULES][SENSORS_PER_MODULE_VOLT];
bool stsTempCellsFail[MAX_MODULES][SENSORS_PER_MODULE_TEMP];

//****** CARGADOR    */
unsigned long idStsCharger = 0x18FF50E5; // 419385573
unsigned long idCmdCharger = 0x1806E5F4;
unsigned long idPrueba = 0x30;
byte stsChargerByte[8];
byte cmdChargerByte[8];

//****** AMPERIMETRO    */
SensorCorriente sensor1;
double stsCorrienteCarga = 0;

//****** PWM    */
const int pwmChannel = 0;
const int pwmPin = PWM_FANS;
const int pwmFrequency = 1000;
const int pwmResolution = 8;
int pwmPorcentaje = 0;

// ============================================================
// NUEVA LÓGICA AUX / TSON / PRECARGA / COHERENCIA
// ============================================================

float filtered_airs = 0.0f;
int stable_aux_state = 0;
int last_aux_state = 0;
unsigned long aux_change_time = 0;
float last_v_airs = 0.0f;

bool tson_armed = false;
bool last_tson_state = false;
bool last_sdc_end = false;

unsigned long precharge_start_time = 0;
bool precharge_started = false;
bool precharge_timeout_error = false;

bool air_coherence_error = false;
bool initial_check_done = false;

// ============================================================
// PROTOTIPOS
// ============================================================

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
Si se pone cmdFillWithInitialErrors a true, se añaden automáticamente a la lista de exclusión las temperaturas y voltajes erróneos al iniciar el programa
Usar tempExclusionList.insert({.idModule = 1, .idNTC = 7}); para añadir exclusiones personalizadas (módulo 2 NTC 8)
*/
void setupExclusions(bool cmdFillWithInitialErrors);

/**
Envía el mensaje por CAN necesario al cargador para iniciar y detener la carga
*/
void controlCharge(float maxVolt, float maxCurrent, bool start);

/**
Muestra las tensiones y temperaturas detalladas de cada módulo
*/
void mostrarDatosDetallados();

/**
Muestra las tensiones y temperaturas detalladas de cada módulo sin usar floats (ahorra RAM)
*/
void mostrarDatosDetalladosOptimizado();

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

void imprimirDatosCSV(float stsVoltCells[12][11], float stsTempCells[12][9]);

void sendBatteryInfoCan(float stsVoltCells[12][11], float stsTempCells[12][9], uint32_t baseIDVolt, uint32_t baseIDTemp);

void mostrarEstadoBMS(bool sdc_end,
                      bool precharge_done,
                      bool tson_armed,
                      bool air_plus_cmd,
                      bool air_minus_cmd,
                      bool aux_plus,
                      bool aux_minus,
                      bool precharge_timeout_error,
                      bool air_coherence_error,
                      bool initial_check_done,
                      bool stsFail);


#define TARGET_CHAR_TO_SDC 's'
#define TARGET_CHAR_TO_LOOP l

bool prev_sdc_end = false;
bool prev_precharge_done = false;
bool prev_tson_armed = false;
bool prev_air_plus_cmd = false;
bool prev_air_minus_cmd = false;
bool prev_aux_plus = false;
bool prev_aux_minus = false;
bool prev_precharge_timeout_error = false;
bool prev_air_coherence_error = false;
bool prev_initial_check_done = false;
bool prev_stsFail = false;

bool force_show_state = false;

// ============================================================
// SETUP
// ============================================================

void setup()
{
    bool ok = false;
    pinMode(BMS_OK, OUTPUT);
    digitalWrite(BMS_OK, false);
    pinMode(OE_TXS_PIN, OUTPUT);
    digitalWrite(OE_TXS_PIN, HIGH);
    
    // Configuración de pines AIRs
    pinMode(PIN_AIR_PLUS_CTRL, OUTPUT);
    pinMode(PIN_AIR_MINUS_CTRL, OUTPUT);
    digitalWrite(PIN_AIR_PLUS_CTRL, HIGH);   // Abierto por defecto (lógica invertida)
    digitalWrite(PIN_AIR_MINUS_CTRL, HIGH);  // Abierto por defecto (lógica invertida)
    
    // Configuración de pin TSON (pulsador con PULLDOWN externo)
    pinMode(PIN_TSON, INPUT);
  
    //delay(10000);
    Serial.begin(115200);
    static int t = millis();

    configBMS();

    // Inicialización CAN con timeout
    int can_init_attempts = 0;
    const int MAX_CAN_ATTEMPTS = 10;
    
    while (CAN.error == 1 && can_init_attempts < MAX_CAN_ATTEMPTS)
    {
      Serial.print(F("Er.CAN (intento "));
      Serial.print(can_init_attempts + 1);
      Serial.println(F("/10)"));
      delay(100);
      can_init_attempts++;
    }
    
    if (CAN.error == 1) {
      Serial.println(F("⚠️  WARNING: CAN no inicializado después de 10 intentos"));
      Serial.println(F("Sistema continuará sin CAN"));
    } else {
      Serial.println(F("CAN OK"));
    }
    bool okk = false;
    readVoltages2(okk);
    checkFails(okk);
    printFailResults();

    setupExclusions(false);

    ledcSetup(pwmChannel, pwmFrequency, pwmResolution);
    ledcAttachPin(pwmPin, pwmChannel);
    ledcWrite(pwmChannel, 0);

    Serial.println("Reset reason: ");
    Serial.println(esp_reset_reason());

    Serial.println("---==[ RUNTIME MEMORY REPORT ]==---");

    // This is the total free memory you have for ALL dynamic 
    // allocations (new, malloc, other tasks, etc.)
    Serial.printf("Free Heap (at setup): %u bytes\n", ESP.getFreeHeap());

    // This is the "low water mark" for the heap. It tells you
    // the smallest amount of free heap you've had so far.
    Serial.printf("Min Free Heap (at setup): %u bytes\n", ESP.getMinFreeHeap());

    // This is your check. It shows the minimum free space
    // your 16,000-byte loop stack has had so far.
    UBaseType_t stackHighWater = uxTaskGetStackHighWaterMark(NULL);
    Serial.printf("Loop Stack HWM (minimum free): %u bytes\n", stackHighWater);

    // You can calculate the "used" part from this
    Serial.printf("Loop Stack Used (approx): %u bytes\n", 
                  CONFIG_ARDUINO_LOOP_STACK_SIZE - stackHighWater);
    
    Serial.println("---=============================---");


      //CAN SETUP
    
    CAN.setPacketTimer(10,1000);
    CAN.setPacketTimer(11,1000);
    CAN.setPacketTimer(14,200);
    
    // Configuración de Watchdog para detectar bucles infinitos
    esp_task_wdt_init(10, true);  // Timeout 10 segundos, panic on timeout
    esp_task_wdt_add(NULL);       // Añadir task actual al watchdog
    Serial.println(F("Watchdog activado (10s)"));
}

void loop()
{
  // Reset watchdog al inicio de cada ciclo
  esp_task_wdt_reset();
  
  unsigned long int tTotal = millis();
  static bool cmdCharge = false;
  static bool stsCommOK = false;
  static bool stsVoltagesOK = false;
  static bool stsStartCharge = false;
  static bool stsAMPOK = false;
  static bool cmdResetFail = false;
  static bool stsFail = false;
  static float corrienteCargaTarget = 0;
  static bool flagShow = false;
  bool stsChargerOK = false;
  static int contFails = 0;

  //** CAN RECEIVE */

  CAN.receive();
  CAN.getPacket(idStsCharger, stsChargerByte, 8, false);

  //** READ GPIO */

  // ============================================================
  // LECTURA ADC (NUEVA LÓGICA)
  // ============================================================

  auto readADC = [](int pin) -> float {
    long acc = 0;
    for (int i = 0; i < N_MUESTRAS; i++) {
      acc += analogRead(pin);
      delayMicroseconds(100);
    }
    return (acc / (float)N_MUESTRAS) * 3.3f / 4095.0f;
  };

  float v_sdc    = readADC(PIN_SDC_END_PRECH);
  float v_airs   = readADC(PIN_AIRS_NO_STATE);
  float v_bmsimd = readADC(PIN_SDC_BMS_IMD);

  bool sdc_end       = (v_sdc > 1.0f);
  bool precharge_done = (v_sdc > 2.5f);

  // AUX ESTABLE (NUEVA LÓGICA)
  auto decode_aux_stable = [](float v) -> int {
    // 1) Filtro digital
    filtered_airs = 0.8f * filtered_airs + 0.2f * v;

    int new_state;
    if (filtered_airs < 0.15f)      new_state = 0; // ambos abiertos
    else if (filtered_airs < 0.70f) new_state = 2; // AUX- cerrado
    else if (filtered_airs < 1.15f) new_state = 1; // AUX+ cerrado
    else                            new_state = 3; // ambos cerrados

    // Histeresis simple
    if (new_state != last_aux_state) {
      if (fabs(filtered_airs - last_v_airs) < 0.05f) {
        return stable_aux_state;
      }
    }

    // Debounce temporal
    if (new_state != stable_aux_state) {
      if (millis() - aux_change_time > 40) {
        stable_aux_state = new_state;
      }
    } else {
      aux_change_time = millis();
    }

    last_aux_state = new_state;
    last_v_airs = filtered_airs;
    return stable_aux_state;
  };

  int aux_idx = decode_aux_stable(v_airs);
  bool aux_plus  = (aux_idx == 1 || aux_idx == 3);
  bool aux_minus = (aux_idx == 2 || aux_idx == 3);

  // ============================================================
  // LÓGICA TSON / PRECARGA / AIRs / COHERENCIA 
  // ============================================================

  // --- TSON ---
  bool tson_state = digitalRead(PIN_TSON);
  bool tson_rising_edge = (tson_state && !last_tson_state);
  last_tson_state = tson_state;

  if (tson_rising_edge && sdc_end) {
      tson_armed = true;
      Serial.println("🔘 TSON pulsado → AIR- ARMADO");
  }

  if (!sdc_end && last_sdc_end) {
      tson_armed = false;
      Serial.println("⚠️  SDC_end perdido → TSON DESARMADO");
  }

  last_sdc_end = sdc_end;

  // --- PRECARGA ---
  constexpr unsigned long PRECHARGE_TIMEOUT_MS = 5000;

  if (!precharge_started && sdc_end && tson_armed && !precharge_done) {
      precharge_start_time = millis();
      precharge_started = true;
      Serial.println("⚡ PRECARGA INICIADA");
  }

  if (precharge_started && !precharge_done) {
      unsigned long elapsed = millis() - precharge_start_time;
      if (elapsed > PRECHARGE_TIMEOUT_MS && !precharge_timeout_error) {
          precharge_timeout_error = true;
          Serial.println("🔴 ERROR: TIMEOUT DE PRECARGA");
      }
  }

  if (precharge_done && precharge_started) {
      if (!precharge_timeout_error)
          Serial.println("✅ PRECARGA COMPLETADA");
      precharge_started = false;
  }

  if (!sdc_end) {
      if (precharge_started)
          Serial.println("⚠️  Precarga abortada: SDC abierto");
      precharge_started = false;
      precharge_timeout_error = false;
  }

  // ============================================================
  // CÁLCULO DE COMANDOS AIRs 
  // ============================================================

  bool air_plus_cmd = false;
  bool air_minus_cmd = false;

  // Solo permitir cerrar AIRs si NO hay fallo global
  if (!stsFail)
  {
      air_minus_cmd = (sdc_end && tson_armed);
      air_plus_cmd  = (precharge_done && !precharge_timeout_error);
  }

  // ============================================================
  // COHERENCIA AIRs 
  // ============================================================

  air_coherence_error = false;

  if (sdc_end)
  {
      if (air_plus_cmd  != aux_plus)  air_coherence_error = true;
      if (air_minus_cmd != aux_minus) air_coherence_error = true;
  }

  // ============================================================
  // VERIFICACIÓN INICIAL 
  // ============================================================

  // Solo marcar como completada si:
  // 1. No hay timeout de precarga
  // 2. No hay error de coherencia
  // 3. Los AIRs están ABIERTOS (estado seguro inicial)
  if (!initial_check_done &&
      !precharge_timeout_error &&
      !air_coherence_error &&
      !aux_plus &&   // Verificar que AIR+ esté abierto
      !aux_minus)    // Verificar que AIR- esté abierto
  {
      initial_check_done = true;
      Serial.println("✅ Verificación inicial completada - AIRs abiertos correctamente");
  }

  // ============================================================
  // CAPA DE SEGURIDAD 
  // ============================================================

  bool newSafetyFail = false;

  if (precharge_timeout_error) newSafetyFail = true;
  if (!initial_check_done)     newSafetyFail = true;
  if (air_coherence_error)     newSafetyFail = true;

  // Aplicar fallo de seguridad
  if (newSafetyFail)
      stsFail = 1;

  // ============================================================
  // APLICAR COMANDOS A HARDWARE
  // ============================================================

  // VERIFICACIÓN DE SEGURIDAD: NO cerrar AIR+ si AIR- no está cerrado
  if (air_plus_cmd && !aux_minus) {
      Serial.println("🔴 SEGURIDAD: Intento de cerrar AIR+ sin AIR- cerrado");
      air_plus_cmd = false;
      air_coherence_error = true;
  }

  digitalWrite(PIN_AIR_PLUS_CTRL,  air_plus_cmd  ? LOW : HIGH);
  digitalWrite(PIN_AIR_MINUS_CTRL, air_minus_cmd ? LOW : HIGH);


  // ============================================================
  // LECTURA SENSOR HALL DHAB S/118 (Dual Range 30A / 350A)
  // ============================================================

  // Pines del sensor (ajusta si hace falta)
  const int PIN_HALL_30A  = AMP_PIN;      // salida fina
  const int PIN_HALL_350A = AMP_PIN_2;    // salida gruesa, recordar que en la placa estan cambiadas

  // Leer ADC bruto
  int adc30 = analogRead(PIN_HALL_30A);
  int adc350 = analogRead(PIN_HALL_350A);

  // ADC → Voltios
  auto readADCvolts = [](int pin) -> float {
      long acc = 0;
      for (int i = 0; i < 50; i++) {
          acc += analogRead(pin);
          delayMicroseconds(100);
      }
      float adc = acc / 50.0f;
      return adc * 3.3f / 4095.0f;
  };

  // Leer tensiones
  float V30  = readADCvolts(PIN_HALL_30A);
  float V350 = readADCvolts(PIN_HALL_350A);

  // ============================================================
  // CALIBRACIÓN (VALORES PROVISIONALES, AJUSTAR CON DATOS REALES)
  // ============================================================

  float V0_30   = 2.50f;
  float V0_350  = 2.50f;

  float S30     = 0.060f;   // V/A canal fino ±30A
  float S350    = 0.008f;   // V/A canal grueso ±350A

  float I30  = (V30  - V0_30)  / S30;
  float I350 = (V350 - V0_350) / S350;

  // ============================================================
  // FUSIÓN INTELIGENTE DE RANGOS (BLENDING)
  // ============================================================

  float I_SW_LOW  = 20.0f;
  float I_SW_HIGH = 28.0f;

  float Iabs = fabs(I30);
  float Ifinal = 0.0f;

  if (Iabs <= I_SW_LOW) {
      Ifinal = I30;
  }
  else if (Iabs >= I_SW_HIGH) {
      Ifinal = I350;
  }
  else {
      float w = (Iabs - I_SW_LOW) / (I_SW_HIGH - I_SW_LOW);
      Ifinal = (1.0f - w) * I30 + w * I350;
  }

  // ============================================================
  // RESULTADO FINAL
  // ============================================================
  //** PROCESS DATA */ 
  stsChargerOK = (stsChargerByte[4] == 0);
  stsCorrienteCarga = Ifinal;
  stsAMPOK = (fabs(stsCorrienteCarga) <= 100.0f);


  // ============================================================
  // LECTURA DE CELDAS Y FALLOS 
  // ============================================================
  
  int resReadVoltages = readVoltages2(stsCommOK);
  if (resReadVoltages == 0)
  {
    checkFails(stsVoltagesOK);
  }
  else if (resReadVoltages == 1)
  {
    numCRCFails++;
    Serial.println(F("CRC error"));
  }
  else if (resReadVoltages == -1)
  {
    contFails++;
    numCOMMFails++;
    Serial.println(F("Comm Error"));
    //Serial.println(contFails);

  }


  bool a; // basurilla
  procesarComandoSerial(corrienteCargaTarget, cmdCharge, cmdResetFail, a);

  //Para que no haya fallo, tienen que estar todas a 1
  bool failCondition = !(stsVoltagesOK && stsCommOK && stsAMPOK && stsAutoadressingOK);
  // Se ponen a true las variables de estado de fallo para en caso de que el fallo deje de estar presente, estas sigan activas hasta
  // que el ususario introduzca 'r'
  if (!stsVoltagesOK)
    stsFailVoltLatch = true;
  if (!stsCommOK)
    stsFailCommLatch = true;
  if (!stsAMPOK)
    stsFailAmpLatch = true;

  static unsigned long int timeFail = millis();
  const unsigned int timeToFail = 3000;
  static long int contFail = 0;

  //** Rutina de activación de fallo */
  /*
    Si se da la condición de fallo durante más de "timeToFail" milisegundos:
      Situación 1 - Si el fallo se debe a problema de comunicación entre el esp32 y los esclavos y este está presente durante más del tiempo establecido "timeToFail", se vuelve a hacer el AutoAdressing
        para intentar reestablecer la comunicación pasado el tiem, si no se consigue (stsAutoAdressingOK=false), se abre el SDC
      Situación 2 - Si es por fallo de voltaje, amperímetro o fallo del autoadressing, se abre el SDC inmediatamente
  */

  // ============================================================
  // LÓGICA DE FALLO ANTIGUA 
  // ============================================================

  if (failCondition)
  {
    if(!(stsVoltagesOK && stsAMPOK && stsAutoadressingOK))
    {
      stsFail=1;
    }
    else if ((millis() - timeFail) > timeToFail)
    {
      if (!stsCommOK) // Situación 1
      {
        if(stsNumTriesToResetComm<5)
        {
        configBMS();
        stsNumTriesToResetComm++;
        }
        else
        {
          stsFail = 1;
        }
      }
    }
    // Serial.println(millis()-timeFail);
  }

  else
  {
    timeFail = millis();

    //***COMENTAR ESTA LÍNEA PARA QUE SE TENGA QUE REINICIAR EL LV PARA QUE EL BMS PUEDA ESTAR OK DESPUES DE FALLO
    // Solo resetear stsFail si NO hay fallos de seguridad activos
    if (!newSafetyFail)
        stsFail=0;
  }
    
  // ============================================================
  // BMS_OK FINAL 
  // ============================================================
  if (stsFail)
  {
    cmdCharge = 0;
    corrienteCargaTarget = 0;
    digitalWrite(BMS_OK, false);
  }
  else
  {
    if(stsCommOK) stsNumTriesToResetComm=0;
    digitalWrite(BMS_OK, true);
    flagShow = 0;
  }

  // ============================================================
  // CONTROL CARGA / PWM / CAN / DEBUG (TU LÓGICA ANTIGUA)
  // ============================================================

  //controlCharge(456, corrienteCargaTarget, cmdCharge);
  //** PWM CONTROL*/
  if (pwmPorcentaje > 100)
    pwmPorcentaje = 100;
  else if (pwmPorcentaje < 0)
    pwmPorcentaje = 0;
  float pwmDutyCycleFloat = (pwmPorcentaje * 1000.0) / (2550.0);
  uint32_t pwmDutyCycle = static_cast<uint32_t>(pwmDutyCycleFloat);
  ledcWrite(pwmChannel, pwmDutyCycle);


  //Time error count
  stsLastTotalFailTime= (millis() - timeFail);
  static uint16_t array1[4];
  array1[0]=stsLastTotalFailTime;
  array1[1]=numCOMMFails;
  array1[2]=numCRCFails;
  CAN.setPacket(14,array1,4);

  //BMS Status CAN SEND
  static uint8_t array2[8];
  array2[0]=stsFail;
  array2[1]=0;
  array2[2]=0;
  array2[3]=0;
  array2[4]=stsFailCommLatch;
  array2[5]=stsFailVoltLatch;
  array2[6]=stsFailAmpLatch;
  array2[7]=stsAutoadressingOK;
  CAN.setPacket(10,array2,8);

  static int16_t array3[4];
  array3[0]=(int16_t)maxTemp;
  array3[1]=(int16_t)(maxVolt*1000);
  array3[2]=(int16_t)(minVolt*1000);
  array3[3]=(int16_t)minTemp;
  CAN.setPacket(11,array3,4);

  //** CAN SEND */
  CAN.send();

// ============================================================
// MOSTRAR ESTADO BMS SOLO SI HAY CAMBIOS O SI EL USUARIO LO PIDE
// ============================================================

bool changed =
    (sdc_end != prev_sdc_end) ||
    (precharge_done != prev_precharge_done) ||
    (tson_armed != prev_tson_armed) ||
    (air_plus_cmd != prev_air_plus_cmd) ||
    (air_minus_cmd != prev_air_minus_cmd) ||
    (aux_plus != prev_aux_plus) ||
    (aux_minus != prev_aux_minus) ||
    (precharge_timeout_error != prev_precharge_timeout_error) ||
    (air_coherence_error != prev_air_coherence_error) ||
    (initial_check_done != prev_initial_check_done) ||
    (stsFail != prev_stsFail);

// Mostrar si hay cambios o si el usuario lo pide
if (changed || force_show_state)
{
    mostrarEstadoBMS(
        sdc_end,
        precharge_done,
        tson_armed,
        air_plus_cmd,
        air_minus_cmd,
        aux_plus,
        aux_minus,
        precharge_timeout_error,
        air_coherence_error,
        initial_check_done,
        stsFail
    );

    force_show_state = false; // reset manual
}

// Actualizar snapshot
prev_sdc_end = sdc_end;
prev_precharge_done = precharge_done;
prev_tson_armed = tson_armed;
prev_air_plus_cmd = air_plus_cmd;
prev_air_minus_cmd = air_minus_cmd;
prev_aux_plus = aux_plus;
prev_aux_minus = aux_minus;
prev_precharge_timeout_error = precharge_timeout_error;
prev_air_coherence_error = air_coherence_error;
prev_initial_check_done = initial_check_done;
prev_stsFail = stsFail;


  //** DEBUG */
  static int t = millis();
  if ((millis() - t) >= 2000)
  {
    // Serial.println((String)"I= "+corrienteCargaTarget+" cmdCharge= "+cmdCharge +" reset= "+cmdResetFail+ " ok= "+stsVoltagesOK);

    // showChargeData();
    // readVoltages(stsVoltagesOK);
    // CAN.printByteArray(stsChargerByte,8);
    // printVoltages();
    // ✅ GOOD (direct printing, no temporary objects)
Serial.print(F("Fail(Comm,Volt,Amp,AutoAdressing)= "));
Serial.print(stsFailCommLatch);
Serial.print(F(", "));
Serial.print(stsFailVoltLatch);
Serial.print(F(", "));
Serial.print(stsFailAmpLatch);
Serial.print(F(", "));
Serial.println(!stsAutoadressingOK);

Serial.print(F("Fail(FailCond,StsF)"));
Serial.print(failCondition);
Serial.print(F(", "));
Serial.println(stsFail);

Serial.println("=== DHAB DEBUG ===");
Serial.print("ADC30 = ");  Serial.println(adc30);
Serial.print("ADC350 = "); Serial.println(adc350);
Serial.print("V30 = ");  Serial.println(V30, 5);
Serial.print("V350 = "); Serial.println(V350, 5);
Serial.print("I30 = ");  Serial.println(I30, 3);
Serial.print("I350 = "); Serial.println(I350, 3);
Serial.print("Ifinal = "); Serial.println(stsCorrienteCarga, 3);


Serial.print(F("Tried reset: "));
Serial.print(stsNumTriesToResetComm);




    Serial.println(contFail);
    Serial.println(millis());
    Serial.println();
    Serial.println(ESP.getFreeHeap()); // 361180 libres
    Serial.println();
  
    t = millis();

    CAN.printByteArray(stsChargerByte, 8);
     UBaseType_t stackHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    Serial.printf("Free stack: %d bytes\n", stackHighWaterMark);
    
    if (stackHighWaterMark < 500) {
        Serial.println("WARNING: Stack running low!");
    }
  }
}

void debug()
{

}

/**
 * @brief Espera y procesa comandos desde el puerto serie.
 * @param valorFloatRef Referencia a la variable float que se modificará con el comando 'c'.
 * @param valorBoolRef Referencia a la variable bool que se modificará con el comando 'c'.
 */
void procesarComandoSerial(float &valorFloatRef, bool &valorBoolRef, bool &reset, bool &fail)
{
  // Solo procesa si hay datos disponibles
  if (Serial.available() > 0)
  {
    // 1. SAFE BUFFER: Define a rigid limit for the command length. 
    // 32 bytes is plenty for "c,123.45,1" or single chars.
    const size_t MAX_CMD_SIZE = 32;
    char cmdBuffer[MAX_CMD_SIZE + 1]; // +1 for null terminator

    // 2. SAFE READ: Read bytes until newline OR until buffer is full.
    // This prevents the "infinite read" crash caused by noise.
    size_t readLen = Serial.readBytesUntil('\n', cmdBuffer, MAX_CMD_SIZE);
    

    // 3. Null-terminate the string so C functions know where it ends
    cmdBuffer[readLen] = '\0';
    Serial.println(cmdBuffer);
    // Cleanup: Remove carriage return '\r' if present (common from Serial Monitors)
    if (readLen > 0 && cmdBuffer[readLen - 1] == '\r')
    {
      cmdBuffer[readLen - 1] = '\0';
      readLen--;
    }

    // If buffer is empty (just a newline received), exit
    if (readLen == 0) return;

    // --- PARSING ---

    // Check for command 'c' (starts with "c,")
    if (cmdBuffer[0] == 'c' && cmdBuffer[1] == ',')
    {
      float tempFloat = 0.0;
      int tempInt = 0;
      
      // Use sscanf to safely parse the numbers
      // Returns the number of items successfully matched
      int parsed = sscanf(cmdBuffer, "c,%f,%d", &tempFloat, &tempInt);

      if (parsed == 2)
      {
        valorFloatRef = tempFloat;
        valorBoolRef = (tempInt == 1);

        Serial.print(F("-> OK: Coma 'c' rec: "));
        Serial.print(F("float = "));
        Serial.print(valorFloatRef);
        Serial.print(F(", bool = "));
        Serial.println(valorBoolRef ? "true" : "false");
      }
      else
      {
        Serial.println(F("-> ERROR: Format 'c,float,bool'"));
      }
    }
    // Handle Single Character Commands
    else if (readLen == 1)
    {
      char cmd = cmdBuffer[0];

      switch (cmd)
      {
      case 'i':
      {
        bool stsCommOK_loc;
        bool stsVoltagesOK_loc;
        // Ensure readVoltages2 uses local buffers (remove 'static' from readVoltages2 vars first!)
        int resReadVoltages = readVoltages2(stsCommOK_loc);
        
        if (resReadVoltages == 0)
        {
          checkFails(stsVoltagesOK_loc);
          mostrarDatosDetallados();
          showChargeData();
        }
        else if (resReadVoltages == 1)
        {
          numCRCFails++;
          Serial.println(F("CRC error"));
        }
        else
        {
          Serial.println(F("Read error"));
        }
        break;
      }

      case 'l':
        printFailResults();
        break;

      case 'r':
        Serial.println(F("Restarting..."));
        delay(100);
        ESP.restart();
        break;

      case 'f':
        fail = 0;
        stsFailAmpLatch = 0;
        stsFailCommLatch = 0;
        stsFailVoltLatch = 0;
        Serial.println(F("Fails Reset"));
        break;

      case 'd':
        showChargeData();
        break;

      case 'a':
        configBMS();
        break;

      case 'b':
        CommSleepToWake();
        break;

      case 'e':
        imprimirDatosCSV(stsVoltCells, stsTempCells);
        break;

      case 't':
        sendBatteryInfoCan(stsVoltCells, stsTempCells, 1168, 1205);
        break;
      
      // Debug/Unused commands kept for compatibility
      
      case 's':
      BqShutdownAllDevices(); 
      break;
     
      case 'x':
      force_show_state = true;
      Serial.println("Mostrando estado AIRs...");
      break;

      default:
        // It's a single char, but not one we know. Likely noise. 
        // Print nothing to avoid flooding logs.
        break;
      }
    }
    else
    {
      // If we get here, it means we received a string that isn't "c,..." and isn't 1 char.
      // This is almost certainly NOISE. We ignore it completely.
      // Serial.print(F("Ignored noise: ")); Serial.println(cmdBuffer);
    }
  }
}

void mostrarDatosDetalladosTemperaturas(int i)
{
  for (int j = 0; j < 9; j++)
  {
    Serial.print((String)(j + 1) + "-> " + stsTempCells[i][j]); // Imprime con 1 decimal
    Serial.print("   ");
  }
  Serial.println();
}

void mostrarDatosDetallados()
{

  // Serial.println(F("\n--- Vista Detallada por Modulo ---")); // F() macro ahorra RAM
  // Serial.println(F("Modulo | Voltajes (V)                  | Temperaturas (C)"));
  Serial.println(F("            1    2    3    4    5    6    7    8    9   10    11    V-    V+        1    2    3    4    5    6    7    8    9    T-   T+"));
  Serial.println(F("-----------------------------------------------------------------------------------------------------------------"));


const int NUM_MODULES = TOTALBOARDS / 2;

  // 3. PROCESAMIENTO Y CÁLCULO
  
  // --- Inicialización de ARRAYS estáticos ---
  // Usamos static para que la memoria se reserve una sola vez, 
  // pero debemos limpiar/reiniciar los valores dentro del bucle.




  // --- Procesamiento del array de Voltajes ---
  for (int i = 0; i < NUM_MODULES; i++)
  { // Bucle por cada Módulo
    
    // REINICIAR valores para este módulo específico antes de mirar sus celdas
    stsVoltCellsMin[i] = 5.0; // Iniciar con valor alto
    stsVoltCellsMax[i] = 0.0; // Iniciar con valor bajo

    for (int j = 0; j < 11; j++)
    { // Bucle por cada Celda de voltaje
      float v = stsVoltCells[i][j];
      
      // Comprobar Mínimo para el módulo 'i'
      if (v < stsVoltCellsMin[i])
      {
        stsVoltCellsMin[i] = v;
      }
      
      // Comprobar Máximo para el módulo 'i'
      if (v > stsVoltCellsMax[i])
      {
        stsVoltCellsMax[i] = v;
      }
      
    }
  }

  // --- Procesamiento del array de Temperaturas ---
  for (int i = 0; i < NUM_MODULES; i++)
  { // Bucle por cada Módulo

    // REINICIAR valores para este módulo específico
    stsTempCellsMin[i] = 100.0; // Iniciar con valor alto
    stsTempCellsMax[i] = -20.0; // Iniciar con valor bajo

    for (int j = 0; j < 9; j++)
    { // Bucle por cada Sensor de temperatura
      float t = stsTempCells[i][j];
      
      // Comprobar Mínimo para el módulo 'i'
      if (t < stsTempCellsMin[i])
      {
        stsTempCellsMin[i] = t;
      }
      
      // Comprobar Máximo para el módulo 'i'
      if (t > stsTempCellsMax[i])
      {
        stsTempCellsMax[i] = t;
      }
    }
  }



  // Bucle a través de cada módulo
  for (int i = 0; i < TOTALBOARDS / 2; i++)
  {
    // Imprime el identificador del módulo (ej: M00, M01, ... M11)
    Serial.print("M");
    if (i < 9)
    {
      Serial.print("0"); // Añade un cero para alinear M0 a M9 con M10 y M11
    }
    Serial.print(i + 1);
    Serial.print("   | V: ");

    // Imprime los 11 voltajes de celda para el módulo actual
    for (int j = 0; j < 11; j++)
    {
      Serial.print(stsVoltCells[i][j], 2); // Imprime con 2 decimales
      Serial.print(" ");
    }
    Serial.print("  ");
    Serial.print(stsVoltCellsMin[i], 2); // Imprime con 2 decimales
      Serial.print(" ");
Serial.print(stsVoltCellsMax[i], 2); // Imprime con 2 decimales
      Serial.print(" ");
    Serial.print("| T: ");

    // Imprime las 9 temperaturas para el módulo actual
    for (int j = 0; j < 9; j++)
    {
      Serial.print(stsTempCells[i][j], 1); // Imprime con 1 decimal
      Serial.print(" ");
    }
    Serial.print("  ");
    Serial.print(stsTempCellsMin[i], 1); // Imprime con 1 decimal
      Serial.print(" ");
      Serial.print(stsTempCellsMax[i], 1); // Imprime con 1 decimal
      Serial.print(" ");
    Serial.println(); // Salto de línea para el siguiente módulo
  }
  Serial.println(F("------------------------------------------------------------------"));
}

void mostrarDatosDetalladosOptimizado()
{
  // Serial.println(F("\n--- Vista Detallada por Modulo ---"));
  // Serial.println(F("Modulo | Voltajes (V)                  | Temperaturas (C)"));
  Serial.println(F("            1    2    3    4    5    6    7    8    9   10    11        1    2    3    4    5    6    7    8    9  "));
  Serial.println(F("-----------------------------------------------------------------------------------------------------------------"));

  for (int i = 0; i < TOTALBOARDS / 2; i++)
  {
    Serial.print("M");
    if (i < 9)
    {
      Serial.print("0");
    }
    Serial.print(i + 1);
    Serial.print("   | V: ");

    // --- OPTIMIZED VOLTAGE PRINTING ---
    for (int j = 0; j < 11; j++)
    {
      // 1. Get the float value
      float v = stsVoltCells[i][j];

      // 2. Handle negative sign if necessary
      if (v < 0)
      {
        Serial.print("-");
        v = -v;
      }

      // 3. Convert to integer with rounding (e.g., 3.1415 -> 314)
      long v_int = (long)(v * 100.0 + 0.5);

      // 4. Print the integer part (e.g., 314 / 100 = 3)
      Serial.print(v_int / 100);
      Serial.print(".");

      // 5. Print the decimal part (e.g., 314 % 100 = 14)
      long v_dec = v_int % 100;
      if (v_dec < 10)
        Serial.print("0"); // Add leading zero for values like 3.01
      Serial.print(v_dec);
      Serial.print(" ");
    }

    Serial.print("| T: ");

    // --- OPTIMIZED TEMPERATURE PRINTING ---
    for (int j = 0; j < 9; j++)
    {
      // 1. Get the float value
      float t = stsTempCells[i][j];

      // 2. Handle negative sign
      if (t < 0)
      {
        Serial.print("-");
        t = -t;
      }

      // 3. Convert to integer with rounding (e.g., 25.78 -> 258)
      long t_int = (long)(t * 10.0 + 0.5);

      // 4. Print the integer part (e.g., 258 / 10 = 25)
      Serial.print(t_int / 10);
      Serial.print(".");

      // 5. Print the decimal part (e.g., 258 % 10 = 8)
      Serial.print(t_int % 10);
      Serial.print(" ");
    }

    Serial.println(); // Salto de línea
  }
  Serial.println(F("------------------------------------------------------------------"));
}



void showChargeData()
{
  // 3. PROCESAMIENTO Y CÁLCULO
  // --- Inicialización de variables de resultados ---

  int moduloMinVolt = 0;
  int moduloMaxVolt = 0;
  float sumaTotalVolt = 0.0;


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
  Serial.print(moduloMinVolt+1);
  Serial.print(" ~ ");
  Serial.print(maxVolt, 2);
  Serial.print("V@M");
  Serial.print(moduloMaxVolt+1);
  Serial.print("} T:{");
  Serial.print(minTemp, 1);
  Serial.print("C@M");
  Serial.print(moduloMinTemp+1);
  Serial.print(" ~ ");
  Serial.print(maxTemp, 1);
  Serial.print("C@M");
  Serial.print(moduloMaxTemp+1);
  Serial.print("} | V_total:");
  Serial.print(sumaTotalVolt, 2);
  Serial.print("V | T_media:");
  Serial.print(tempMedia, 2);
  Serial.println("C");

  Serial.print(F("Charge Current :"));
  Serial.println(stsCorrienteCarga);
}

void mostrarEstadoBMS(bool sdc_end,
                      bool precharge_done,
                      bool tson_armed,
                      bool air_plus_cmd,
                      bool air_minus_cmd,
                      bool aux_plus,
                      bool aux_minus,
                      bool precharge_timeout_error,
                      bool air_coherence_error,
                      bool initial_check_done,
                      bool stsFail)
{
    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║          ESTADO DEL SISTEMA BMS        ║");
    Serial.println("╚════════════════════════════════════════╝");

    // --- SDC ---
    Serial.print("SDC_end: ");
    Serial.println(sdc_end ? "CERRADO ✓" : "ABIERTO ✗");

    // --- Precarga ---
    Serial.print("Precarga completada: ");
    Serial.println(precharge_done ? "SI ✓" : "NO ✗");

    Serial.print("Timeout precarga: ");
    Serial.println(precharge_timeout_error ? "SI 🔴" : "NO ✓");

    // --- TSON ---
    Serial.print("TSON armado: ");
    Serial.println(tson_armed ? "SI ✓" : "NO ✗");

    // --- AIRs comandados ---
    Serial.println("\n--- AIRs (comando) ---");
    Serial.print("AIR-: ");
    Serial.println(air_minus_cmd ? "CERRADO 🔒" : "ABIERTO 🔓");

    Serial.print("AIR+: ");
    Serial.println(air_plus_cmd ? "CERRADO 🔒" : "ABIERTO 🔓");

    // --- AUX reales ---
    Serial.println("\n--- AUX (realimentación) ---");
    Serial.print("AUX-: ");
    Serial.println(aux_minus ? "CERRADO 🔒" : "ABIERTO 🔓");

    Serial.print("AUX+: ");
    Serial.println(aux_plus ? "CERRADO 🔒" : "ABIERTO 🔓");

    // --- Coherencia ---
    Serial.print("\nCoherencia AIRs: ");
    Serial.println(air_coherence_error ? "INCOHERENTE 🔴" : "OK ✓");

    // --- Verificación inicial ---
    Serial.print("Verificación inicial: ");
    Serial.println(initial_check_done ? "COMPLETADA ✓" : "PENDIENTE ✗");

    // --- BMS_OK ---
    Serial.print("\nBMS_OK: ");
    Serial.println(stsFail ? "FALSE ❌" : "TRUE ✓");

    Serial.println("════════════════════════════════════════");
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

  if (maxVolt > 555)
    maxVolt = 500;
  // Serial.println()
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

        if (currentVoltage <= MIN_VALID_VOLTAGE || currentVoltage >= MAX_VALID_VOLTAGE)
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
  static byte response_frame[(MAXBYTES + 6)];
  static byte response_frame2[(MAXBYTES + 6)];
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
    delay(1);
  }
  int tTotal = millis() - t;
  return returnValue;
}

void configBMS()
{
  bool ok = false;
  Ini_ESP();
  Serial.println(F("STRT.ADRESS"));
  stsNumAutoAdressingAttempts = 0;
  stsAutoadressingOK = true;
  while (!ok && stsNumAutoAdressingAttempts < NUM_MAX_AUTOADRESSING_ATTEMPTS)
  {
    static int tWait=200;
    Serial.print("Attempt: ");
    Serial.println(stsNumAutoAdressingAttempts);
    Wake79606();
    delay(tWait);
    CommReset(BAUDRATE);
    delay(tWait);
    ok = AutoAddress();
    delay(tWait);
    stsNumAutoAdressingAttempts++;

  }
  if (stsNumAutoAdressingAttempts >= NUM_MAX_AUTOADRESSING_ATTEMPTS)
  {
    stsAutoadressingOK = false;
  }

  int nCurrentBoard = 0;
  static byte response_frame[(MAXBYTES + 6)];
  static byte response_frame2[(MAXBYTES + 6)];

  InitDevices();

  // WriteReg(0, SYSFLT1_FLT_RST, 0xFFFFFF, 3, FRMWRT_ALL_NR); // reset system faults
  // WriteReg(0, SYSFLT1_FLT_MSK, 0xFFFFFF, 3, FRMWRT_ALL_NR);
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

void setupExclusions(bool cmdFillWithInitialErrors)
{
  // Serial.println("Configurando listas de exclusión...");
  //  Excluir sensor de voltaje del módulo 0, sensor 5
  // voltExclusionList.insert({.idModule = 0, .idVolt = 5});
  // Excluir sensor de voltaje del módulo 2, sensor 10
  // voltExclusionList.insert({.idModule = 2, .idVolt = 10});

  // Excluir sensor de temperatura del módulo 1, sensor 1
  // tempExclusionList.insert({.idModule = 1, .idNTC = 7});
  // Serial.printf("Exclusiones configuradas: %u de voltaje, %u de temperatura.\n", voltExclusionList.size(), tempExclusionList.size());

  // Añade automáticamente a la lista de exclusión las temperaturas y voltajes erróneos al iniciar el programa

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
  // Serial.println("Poblando arrays con datos de prueba...");
  // // Llenar todo con valores válidos por defecto
  // for (int i = 0; i < MAX_MODULES; ++i)
  //   for (int j = 0; j < SENSORS_PER_MODULE_VOLT; ++j)
  //     stsVoltCells[i][j] = 3.8f;
  // for (int i = 0; i < MAX_MODULES; ++i)
  //   for (int j = 0; j < SENSORS_PER_MODULE_TEMP; ++j)
  //     stsTempCells[i][j] = 25.0f;

  // // Insertar algunos valores que deberían fallar
  // stsVoltCells[0][2] = 2.9f;  // Falla (bajo voltaje)
  // stsVoltCells[1][8] = 4.5f;  // Falla (alto voltaje)
  // stsTempCells[0][0] = 70.0f; // Falla (alta temperatura)

  // // Insertar un valor fuera de rango en una posición EXCLUIDA
  // // Este NO debería aparecer como una falla.
  // stsVoltCells[0][5] = 1.5f;  // Excluido, no debe fallar
  // stsTempCells[1][1] = 99.0f; // Excluido, no debe fallar
}

void printFailResults()
{
  int contFallosTension = 0;
  int contFallosTemp = 0;
  Serial.println(F("\n-F.Volt (1=FAIL, 0=OK) ---"));
  for (int i = 0; i < MAX_MODULES; ++i)
  {
    Serial.printf(" Modulo %02d: ", i+1);
    for (int j = 0; j < SENSORS_PER_MODULE_VOLT; ++j)
    {
      if (stsVoltCellsFail[i][j])
        contFallosTension++;
      Serial.print(stsVoltCellsFail[i][j]);
    }
    Serial.println();
  }

  Serial.println(F("\n--- F.TEMP ---"));
  for (int i = 0; i < MAX_MODULES; ++i)
  {
    Serial.printf(" Modulo %02d: ", i+1);
    for (int j = 0; j < SENSORS_PER_MODULE_TEMP; ++j)
    {
      if (stsTempCellsFail[i][j])
        contFallosTemp++;
      Serial.print(stsTempCellsFail[i][j]);
    }
    Serial.println();
  }

  Serial.print(F("F.Volt ="));
  Serial.println(contFallosTension);

  Serial.print(F("F.TEMP = "));
  Serial.println(contFallosTemp);

  Serial.print(F("F.CRC = "));
  Serial.println(numCRCFails);
}

void printExclusionLists()
{
  // Serial.println("\n--- Contenido de la Lista de Exclusión de Voltaje ---");

  if (voltExclusionList.empty())
  {
    // Serial.println("-> EMPTY.");
  }
  else
  {
    // Iteramos sobre cada elemento del set usando un bucle for-each
    for (const auto &point : voltExclusionList)
    {
      Serial.printf("-> Exc: Mod %d, Temp %d\n %d\n", point.idModule, point.idVolt);
    }
  }

  // Serial.println("\n--- Contenido de la Lista de Exclusión de Temperatura ---");

  if (tempExclusionList.empty())
  {
    // Serial.println("-> La lista está vacía.");
  }
  else
  {
    // Iteramos sobre cada elemento del set
    for (const auto &point : tempExclusionList)
    {
      Serial.printf("-> Exc: Mod %d, Temp %d\n", point.idModule, point.idNTC);
    }
  }
}

/**
 * @brief Imprime los datos de voltaje y temperatura en formato CSV al monitor serie.
 * * Utiliza punto y coma (;) como delimitador de columnas y formatea los nombres
 * de módulo (M01-M12) y los encabezados (V1-V11, T1-T9) según lo especificado.
 *
 * @param stsVoltCells Array de 12x11 con los datos de voltaje de las celdas.
 * @param stsTempCells Array de 12x9 con los datos de temperatura de las celdas.
 */
void imprimirDatosCSV(float stsVoltCells[12][11], float stsTempCells[12][9])
{

  // --- 1. Imprimir la fila de encabezado ---

  Serial.print("Modulo");

  // Imprimir encabezados de Voltaje (V1 a V11)
  for (int j = 0; j < 11; j++)
  {
    Serial.print(";V");
    Serial.print(j + 1);
  }

  // Imprimir encabezados de Temperatura (T1 a T9)
  for (int j = 0; j < 9; j++)
  {
    Serial.print(";T");
    Serial.print(j + 1);
  }

  // Terminar la línea del encabezado
  Serial.println();

  // --- 2. Imprimir las filas de datos (una por módulo) ---

  for (int i = 0; i < 12; i++)
  { // Iterar sobre cada módulo (filas 0-11)

    // Imprimir el nombre del módulo (M01, M02, ..., M12)
    Serial.print("M");
    int moduloNum = i + 1;
    if (moduloNum < 10)
    {
      Serial.print("0"); // Añadir cero inicial para M01-M09
    }
    Serial.print(moduloNum);

    // Imprimir los 11 datos de voltaje para este módulo
    for (int j = 0; j < 11; j++)
    {
      Serial.print(";");
      Serial.print(stsVoltCells[i][j]);
    }

    // Imprimir los 9 datos de temperatura para este módulo
    for (int j = 0; j < 9; j++)
    {
      Serial.print(";");
      Serial.print(stsTempCells[i][j]);
    }

    // Terminar la línea de datos para este módulo
    Serial.println();
  }
}

/**
 * @brief Envía por CAN string con los datos de voltaje y temperatura de la batería.
 * 
 * @param stsVoltCells Array de 12x11 con los datos de voltaje.
 * @param stsTempCells Array de 12x9 con los datos de temperatura.
 * @param baseIDVolt ID inicial para los voltajes (ej. 0x100)
 * @param baseIDTemp ID inicial para las temperaturas (ej. 0x200)
 */
void sendBatteryInfoCan(float stsVoltCells[12][11], float stsTempCells[12][9], uint32_t baseIDVolt, uint32_t baseIDTemp) {
  
  for (int i = 0; i < 12; i++) {
    uint32_t moduloID = i + 1;

    // --- 1. ENVIAR VOLTAJES ---
    for (int vIdx = 0; vIdx < 11; vIdx += 4) {
      // El ID se construye sumando la base + el desplazamiento del módulo
      uint32_t canID = baseIDVolt + (moduloID << 4) + (vIdx / 4);
      uint16_t buffer[4] = {0, 0, 0, 0};

      for (int k = 0; k < 4; k++) {
        if (vIdx + k < 11) {
          buffer[k] = (uint16_t)(stsVoltCells[i][vIdx + k] * 1000);
        }
      }
      CAN.setPacket(canID, (byte*)buffer, 8);
      CAN.send();
    }

    // --- 2. ENVIAR TEMPERATURAS ---
    for (int tIdx = 0; tIdx < 9; tIdx += 4) {
      uint32_t canID = baseIDTemp + (moduloID << 4) + (tIdx / 4);
      uint16_t buffer[4] = {0, 0, 0, 0};

      for (int k = 0; k < 4; k++) {
        if (tIdx + k < 9) {
          buffer[k] = (uint16_t)(stsTempCells[i][tIdx + k] * 100);
        }
      }
      CAN.setPacket(canID, (byte*)buffer, 8);
      CAN.send();
    }
  }
}

