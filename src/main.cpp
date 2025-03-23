#include <Arduino.h>
#include "BQ79606.h"

// Function prototypes
void setupBMS();
void readVoltages();
void processCommand(char* command);
void printParams(byte bID, uint16_t wAddr, byte bLen, uint32_t dwTimeOut, byte bWriteType);
void printReadResult(byte* pData, byte bLen);
uint64_t parseHexUint64(const char* str);

// Buffer for incoming serial data
const int MAX_INPUT_LENGTH = 64;
char inputBuffer[MAX_INPUT_LENGTH];
int inputIndex = 0;

// Buffer for read data
const int MAX_DATA_LENGTH = 32;
byte dataBuffer[MAX_DATA_LENGTH];

void setup() {
  //Serial.begin(115200);  // Start serial communication at 115200 baud
  setupBMS();
  Serial.println("BQ79606 Register Reader Ready");
  Serial.println("Format: READ,ID,ADDRESS,LENGTH,TIMEOUT,TYPE");
  Serial.println("Example: READ,0x01,0x2000,8,1000,1");
}

void loop() {
  // Check if data is available to read from serial
  if (Serial.available() > 0) {
    char inChar = Serial.read();
    
    // Add character to buffer if not end of line
    if (inChar != '\n' && inputIndex < MAX_INPUT_LENGTH - 1) {
      inputBuffer[inputIndex] = inChar;
      inputIndex++;
    }
    // Process the command when line end is received
    else {
      inputBuffer[inputIndex] = '\0';  // Null-terminate the string
      processCommand(inputBuffer);
      inputIndex = 0;  // Reset buffer index
    }
  }

}

// Custom function to parse 64-bit hex values
uint64_t parseHexUint64(const char* str) {
  uint64_t result = 0;
  
  // Skip "0x" prefix if present
  if (str[0] == '0' && (str[1] == 'x' || str[1] == 'X')) {
    str += 2;
  }
  
  // Parse hex digits
  while (*str) {
    char c = *str++;
    result <<= 4; // Shift left by 4 bits (multiply by 16)
    
    if (c >= '0' && c <= '9') {
      result |= (c - '0');
    } else if (c >= 'a' && c <= 'f') {
      result |= (c - 'a' + 10);
    } else if (c >= 'A' && c <= 'F') {
      result |= (c - 'A' + 10);
    } else {
      // Invalid character
      break;
    }
  }
  
  return result;
}

void processCommand(char* command) {
  // Check if this is a READ command
  if (strncmp(command, "READ,", 5) == 0) {
    // Process read command
    char* readCommand = command + 5; // Skip "READ,"
    
    byte bID;
    uint16_t wAddr;
    byte bLen;
    uint32_t dwTimeOut;
    byte bWriteType;
    
    // Parse command string to extract parameters
    // Format expected: READ,ID,ADDRESS,LENGTH,TIMEOUT,TYPE
    
    char* token = strtok(readCommand, ",");
    int paramCount = 0;
    
    while (token != NULL && paramCount < 5) {
      switch (paramCount) {
        case 0: // ID
          bID = (byte)strtoul(token, NULL, 0);
          break;
        case 1: // Address
          wAddr = (uint16_t)strtoul(token, NULL, 0);
          break;
        case 2: // Length
          bLen = (byte)strtoul(token, NULL, 0);
          if (bLen > MAX_DATA_LENGTH) {
            Serial.print("ERROR: Length too large. Maximum is ");
            Serial.println(MAX_DATA_LENGTH);
            return;
          }
          break;
        case 3: // Timeout
          dwTimeOut = (uint32_t)strtoul(token, NULL, 0);
          break;
        case 4: // Write Type
          bWriteType = (byte)strtoul(token, NULL, 0);
          break;
      }
      
      token = strtok(NULL, ",");
      paramCount++;
    }
    
    // Check if we got all 5 parameters
    if (paramCount != 5) {
      Serial.println("ERROR: Invalid number of parameters");
      Serial.println("Format: READ,ID,ADDRESS,LENGTH,TIMEOUT,TYPE");
      return;
    }
    
    // Debug print the parsed values
    printParams(bID, wAddr, bLen, dwTimeOut, bWriteType);
    
    // Clear the data buffer first
    memset(dataBuffer, 0, MAX_DATA_LENGTH);
    
    // Call ReadReg with the parsed parameters
    int result = ReadReg(bID, wAddr, dataBuffer, bLen, dwTimeOut, bWriteType);
    Serial.print("ReadReg Result: ");
    Serial.println(result);
    
    if (result > 0) {  // Assuming 0 means success
      printReadResult(dataBuffer, bLen);
    }
    else
    {
      Serial.println("NO INFO READ");
    }
  }
  else {
    Serial.println("Unknown command. Use READ,ID,ADDRESS,LENGTH,TIMEOUT,TYPE");
  }
}

void printParams(byte bID, uint16_t wAddr, byte bLen, uint32_t dwTimeOut, byte bWriteType) {
  Serial.println("Parsed Parameters:");
  
  Serial.print("ID (hex): 0x");
  if (bID < 0x10) Serial.print("0");
  Serial.println(bID, HEX);
  
  Serial.print("Address (hex): 0x");
  if (wAddr < 0x1000) Serial.print("0");
  if (wAddr < 0x100) Serial.print("0");
  if (wAddr < 0x10) Serial.print("0");
  Serial.println(wAddr, HEX);
  
  Serial.print("Length: ");
  Serial.println(bLen);
  
  Serial.print("Timeout (ms): ");
  Serial.println(dwTimeOut);
  
  Serial.print("Write Type: ");
  Serial.println(bWriteType);
  Serial.println();
}

void printReadResult(byte* pData, byte bLen) {
  Serial.println("Read Data (hex):");
  
  for (int i = 0; i < bLen; i++) {
    if (pData[i] < 0x10) Serial.print("0");
    Serial.print(pData[i], HEX);
    Serial.print(" ");
    
    // Add a newline every 8 bytes for readability
    if ((i + 1) % 8 == 0 || i == bLen - 1) {
      Serial.println();
    }
  }
  
  // Show as a single hex value if length is 2, 4, or 8 bytes
  if (bLen == 2 || bLen == 4 || bLen == 8) {
    uint64_t value = 0;
    for (int i = 0; i < bLen; i++) {
      value = (value << 8) | pData[i];
    }
    
    Serial.print("Value as ");
    Serial.print(bLen * 8);
    Serial.print("-bit hex: 0x");
    
    // Print in groups of 4 hex digits (16 bits)
    for (int i = bLen / 2 - 1; i >= 0; i--) {
      uint16_t chunk = (value >> (i * 16)) & 0xFFFF;
      if (chunk < 0x1000) Serial.print("0");
      if (chunk < 0x100) Serial.print("0");
      if (chunk < 0x10) Serial.print("0");
      Serial.print(chunk, HEX);
    }
    Serial.println();
  }
}

void setupBMS()
{
  Ini_ESP();
  Wake79606();
  CommReset(BAUDRATE);
  AutoAddress();
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
  
  
//Serial2.println("OK");*/
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
            Bytesleidos = ReadReg(currentBoard, VCELL1H, response_frame, MAXBYTES, 0, FRMWRT_SGL_R); //0,0x0215,12,0,0
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