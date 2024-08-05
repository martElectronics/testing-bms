#include <Arduino.h>
#include "MART_CAN.h"
#include "BQ79606.h"
#include "edgeDetector.h"
#include "timer.h"

//Objects
CAN_BUS CAN(5,1,500);

//pins
int pinSDC=13,pinDetectCharger=25;

//control
bool v;
bool cmdOpenSDC,cmdStartCharge;
int a;

//***cfg ACU
float cfgMAXVoltageCell=4.1;
float cfgMAXTempCell=60;
float cfgMINVoltCell=2.5;
float cfgMINVoltageDiffBalancing=0.1;
int cfgChargerVoltageMAX=500,cfgChargerCurrentMAX=1;

//*** data ACU
float dataACUVolt[12][12];



//CAN
unsigned long canIdCMDCarroBMS=3,canIdCMDCharger=0x1806E5F4;;
unsigned long canIdSTSBMS=1,canIdSTSCharger=0x18FF50E5;

byte BMSCmdBatCharge;
byte canDataCMDCarroBMS[8],canDataSTSBMS[8],canDataCMDBMSCharger[8];

EdgeDetector stsDetectChargerEdge(100);



enum Status
{
  statusCharging,
  statusBalancing,
  statusDischarging,
  statusFailure,
  statusDone
};

enum Event
{
  eventButton,
  eventChargerPlug,
  eventTempMax,
  eventVoltMax,
  eventVoltMin,
  eventNoEvent
};

Status currentSatus = statusDone;

//Functions
void configBMS();
void readBMS();
void handleStatus(Status &status, bool stsButton, bool stsChargerPlug, float stsTempMaxCell, float stsVoltMaxCell, float stsVoltMinCell);

void configCharger(int maxVolt,int maxCurrent)
{
  byte v0, v1, i0, i1;
  v0=maxVolt/256;
  v1=maxVolt%256;
  i0=maxCurrent/256;
  i1=maxCurrent%256;
  canDataCMDBMSCharger[0]=v0;
  canDataCMDBMSCharger[1]=v1;
  canDataCMDBMSCharger[2]=i0;
  canDataCMDBMSCharger[3]=i1;
}

void setup()
{
  
  pinMode(pinSDC, OUTPUT);
  pinMode(pinDetectCharger,INPUT_PULLDOWN);

  //configBMS();
  //configCharger(cfgChargerVoltageMAX,cfgChargerCurrentMAX);

 
}

void loop()
{
  byte dataPR[8];
  bool stsDetectCharger=!digitalRead(pinDetectCharger);
  stsDetectChargerEdge.detectRisingEdge(stsDetectCharger);

  CAN.receive();
  CAN.getPacket(canIdCMDCarroBMS,canDataCMDCarroBMS);
   CAN.getPacket(10,dataPR);
  bool stsBot=(canDataCMDCarroBMS[0]==1);

  if(currentSatus==statusCharging)   canDataSTSBMS[0]=1;
  else canDataSTSBMS[0]=0;
  canDataSTSBMS[1]=currentSatus;

  if(cmdStartCharge)   canDataCMDBMSCharger[4]=0;
  else canDataCMDBMSCharger[4]=1;

  //handleStatus(currentSatus,stsBot,stsBot,20,3.5,3);

  CAN.setPacket(canIdSTSBMS,canDataSTSBMS);
  CAN.setPacket(canIdCMDCharger,canDataCMDBMSCharger);

  CAN.printByteArray(dataPR,8);

  CAN.send();
  
}

void handleStatus(Status &status, bool stsButton, bool stsChargerPlug, float stsTempMaxCell, float stsVoltMaxCell, float stsVoltMinCell)
{
  switch (status)
  {
  case statusCharging:
    cmdOpenSDC=false;

    if(stsTempMaxCell>=cfgMAXTempCell)  status=statusFailure;
    else if(!stsButton || !stsChargerPlug)  status=statusDone;
    else if(stsVoltMaxCell>cfgMAXVoltageCell)
    {
      if((stsVoltMaxCell-stsVoltMinCell)>=cfgMINVoltageDiffBalancing)
      {
        status=statusBalancing;
      }
      else
      {
        status=statusDone;
      }
    }
    break;

  case statusBalancing:
    cmdOpenSDC=false;
    if(stsTempMaxCell>=cfgMAXTempCell)  status=statusFailure;
    else if(!stsButton || !stsChargerPlug)  status=statusDone;
    else if((stsVoltMaxCell-stsVoltMinCell)<cfgMINVoltageDiffBalancing)
    {
      if(stsVoltMaxCell<cfgMAXVoltageCell)
      {
        status=statusCharging;
      }
      else
      {
        status=statusDone;
      }
    }
    break;

  case statusDischarging:
  cmdOpenSDC=false;
    if(stsTempMaxCell>=cfgMAXTempCell)  status=statusFailure;
    else if(stsVoltMinCell<=cfgMINVoltCell)
    {
      status=statusFailure; //FAIL????
    }
    else if(stsChargerPlug)  status=statusDone;
    break;

  case statusFailure:
    cmdOpenSDC=true;
    if(stsTempMaxCell<cfgMAXTempCell)
    {
      if(stsButton) status=statusDone;
    }
    break;
  case statusDone:
    cmdOpenSDC=false;
    if(stsTempMaxCell>=cfgMAXTempCell)  status=statusFailure;
    else if(stsChargerPlug && stsButton) 
    {
      CAN_BUS CAN(5,1,500);
      status=statusCharging;
    }
    else if(!stsChargerPlug)  
    {
      CAN_BUS CAN(5,1,1000);
      status=statusDischarging;
    }
    break;

  
  default:
    break;
  }
  
}


void configBMS()
{
   Ini_ESP();

  Wake79606();

  CommReset(500000);

  AutoAddress();

  Serial.print("Addres: ");
  delay(10);

  int nCurrentBoard = 0;
  byte response_frame[(MAXBYTES + 6)];
  byte response_frame2[(MAXBYTES + 6)];

  for (nCurrentBoard = 0; nCurrentBoard < 24; nCurrentBoard++)
  {
    memset(response_frame2, 0, sizeof(response_frame2));
    ReadReg(nCurrentBoard, DEVADD_USR, response_frame2, 1, 0, FRMWRT_SGL_R);
    Serial.print((String) "Board " + nCurrentBoard + "= ");

    Serial.print(response_frame2[4]);

    Serial.println(".");

    delay(10);
  }

  InitDevices();

  WriteReg(0, SYSFLT1_FLT_RST, 0xFFFFFF, 3, FRMWRT_ALL_NR); // reset system faults
  WriteReg(0, SYSFLT1_FLT_MSK, 0xFFFFFF, 3, FRMWRT_ALL_NR);
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

  // Serial2.println("OK");*/
}

void readBMS()
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