#ifndef COMMON_H
#define COMMON_H

#include <Arduino.h>

// Uncomment the following line to enable debugging
// #define DEBUG
// #define ERROR_LOGGING

//#define DEBUG

#ifdef DEBUG
#define DEBUG_PRINT(x)  Serial.print(x)
#define DEBUG_PRINTLN(x)  Serial.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

// Uncomment the following line to enable error logging

//#define ERROR_LOGGING

#ifdef ERROR_LOGGING
#define ERROR_PRINT(x)  Serial.print(x)
#define ERROR_PRINTLN(x)  Serial.println(x)
#else
#define ERROR_PRINT(x)
#define ERROR_PRINTLN(x)
#endif


struct CanPacketRawData {
    unsigned long id;
    byte size;
    byte bytes[8];
    bool rrf;
    byte typeExtendedId;
    bool WaitForRRF;
    unsigned long nextSendTime;
};

#define STATUS_START_MASTER_ID 2030
#define STATUS_NUM_PAQUETS 3
#define STATUS_NUM_IDS 15
#define STATUS_DATA_TIME_CALC 3000


#endif // COMMON_H