#include "usb.h"
#include "usb_host_msd.h"
#include "usb_host_msd_scsi.h"
#include "FSIO.h"

//Program info
//Motec is on CAN2 - 1 mbps
//PCS Accel on CAN1 500 kbps

// *****************************************************************************
// *****************************************************************************
// Configuration Bits
// *****************************************************************************
// *****************************************************************************

#ifndef OVERRIDE_CONFIG_BITS
        
    #pragma config UPLLEN   = ON            // USB PLL Enabled
    #pragma config FPLLMUL  = MUL_20        // PLL Multiplier
    #pragma config UPLLIDIV = DIV_2         // USB PLL Input Divider
    #pragma config ICESEL       = ICS_PGx1  // ICE/ICD Comm Channel Select
    #pragma config DEBUG        = OFF       // Debugger Disabled for Starter Kit

    #pragma config FNOSC        = PRIPLL	// Oscillator selection
    #pragma config POSCMOD      = XT	// Primary oscillator mode
    #pragma config FPLLIDIV     = DIV_2	// PLL input divider
    #pragma config FPLLODIV     = DIV_1	// PLL output divider
    #pragma config FPBDIV       = DIV_1	// Peripheral bus clock divider
    #pragma config FSOSCEN      = OFF	// Secondary oscillator enable

    #pragma config FCANIO = OFF //required for CAN see mx7ck rm

    #pragma config FWDTEN = OFF // Watchdog Timer enable
    #pragma config WDTPS = PS1 // Watchdog Timer Postscale

#endif // OVERRIDE_CONFIG_BITS

#define getPBusClock() 80000000
#define UART_BAUD 9600
#define SYS_FREQ (80000000L)

//Defines for GPS command strings
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29" //Only output GPRMC from GPS module
#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F" //GPS update rate
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C"
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F"
#define PMTK_SET_BAUD_57600 "$PMTK251,57600*2C" //Baud rate adjustment
#define PMTK_SET_BAUD_9600 "$PMTK251,9600*17"
#define PMTK_HOT_RESTART "$PMTK101*32\r\n"   //Hot Restart
//GPS Variables
char GPSData[500];
char GPSLogData[500] = "1,2,3,4,5,6\n";
int CommaCount = 0;
int GPSIndex = 0;
int currentLine = 1;
int newData = 0;

//Check Engine light thresholds
int ECTThreshold = 240;
int EGTThreshold = 1900;
int OilTempThreshold = 300;
//RPM of Engine to begin logging
int LogRPM = 2000;
int stopDelayCounter = 0;
int engineOff = 1;

// Global variables
BYTE CAN1MessageFifoArea[2 * 8 * 16];
BYTE CAN2MessageFifoArea[2 * 8 * 16];
int prevButton1;
int prevButton2;
int counter = 0;

int stillPressed = 0;
int buttonCount = 0;
int buttonCountLow = 0;

FSFILE * myFile;
BYTE myData[512];
size_t numBytes;
volatile BOOL deviceAttached;
BOOL shouldLog;
BOOL shouldStop;
//int count;
BYTE angularRateInfo[8];
BOOL angularRateInfoRec;
BYTE accelerationSensor[8];
BOOL accelerationSensorRec;
BYTE HRaccelerationSensor[8];
BOOL HRaccelerationSensorRec;
static unsigned int analogIn1;
static unsigned int analogIn2;
BOOL analogRead;


int PSOC_volts[12]; //A place to store the voltage data from the PSOC
BOOL PSOCConnected = TRUE; //Set to TRUE when PSOC is attached for testing, without the PSOC set to FALSE
char nameString[20];

unsigned short rpm;
unsigned short tp;
unsigned short map;
unsigned short at;
unsigned short et;
unsigned short la1;
unsigned short fp;
unsigned short op;
unsigned short egt1;
unsigned short launch;
unsigned short ntrl;
unsigned short bp;
unsigned short bpf1;
unsigned short batV;
unsigned short ldSpd;
unsigned short lgSpd;
unsigned short rdSpd;
unsigned short rgSpd;
unsigned short runTime;
unsigned short fuel;
unsigned short oilTemp;
unsigned short ignTime;
BOOL motec0Read, motec1Read, motec2Read, motec3Read, motec4Read, motec5Read;

unsigned int millisec;
BIT dataFlag = 0;

//enum used for the finite state machine implemented in this logger
typedef enum{
    init, startLog, log, stopLog, wait

}STATE;

int logNum;
STATE state;

void initUART1(){
    UARTConfigure(UART1, UART_ENABLE_PINS_TX_RX_ONLY);//configure uart for rx and tx
    UARTSetFifoMode(UART1, UART_INTERRUPT_ON_TX_NOT_FULL | UART_INTERRUPT_ON_RX_NOT_EMPTY);//configure for interrupts
    UARTSetLineControl(UART1,  UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);//set uart line options
    UARTSetDataRate(UART1, getPBusClock(), UART_BAUD);//data rate 9600 baud and pbus 10MHz
    UARTEnable(UART1, UART_PERIPHERAL | UART_RX | UART_TX | UART_ENABLE); //enable the uart for tx rx
    while(!UARTTransmitterIsReady(UART1));
}

void initUART2(){
    UARTConfigure(UART2, UART_ENABLE_PINS_TX_RX_ONLY);//configure uart for rx and tx
    //UARTSetFifoMode(UART2, UART_INTERRUPT_ON_TX_NOT_FULL | UART_INTERRUPT_ON_RX_NOT_EMPTY);//configure for interrupts
    UARTSetLineControl(UART2,  UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);//set uart line options
    UARTSetDataRate(UART2, getPBusClock(), UART_BAUD);//data rate 9600 baud and pbus 10MHz
    UARTEnable(UART2, UART_PERIPHERAL | UART_RX | UART_TX | UART_ENABLE); //enable the uart for tx rx
    while(!UARTTransmitterIsReady(UART2));
}

void sendString(char* string){
    int i = 0;
    while(string[i]){
        while(!UARTTransmitterIsReady(UART1));
        UARTSendDataByte(UART1, string[i]);
        i++;
    }
}

void UARTSendString(UART_MODULE uart, const char *string)
{
    while(*string)
    {
        if (UARTTransmitterIsReady(uart))//Case where it doesn't matter whether if or while because location is only incremented when the byte is sent
        {
            UARTSendDataByte(uart, *string);
            while(!UARTTransmissionHasCompleted(uart)){}
            string++; //increment through the string
        }
    }
}

//Return 0 For a "do nothing"
//Return 1 For a transition from wait to log
//Return 2 For a transition from log to wait
int CheckLogStateChange(){
    int countThreshold = 100; //Threshold for a "high" to be registered from the switch
    //look for rising edge in the switch
    if(buttonCount > 500){  //Reset count to 200 if it reaches 500, gives an upper bound to the count
        buttonCount  = 200;
    }
    if(buttonCountLow > 500){  //Reset count to 200 if it reaches 500, gives an upper bound to the count
        buttonCountLow  = 200;
    }
    if(PORTA & 0x02){
        buttonCount++;
        buttonCountLow = 0;

    }
    else if(!(PORTA & 0x02))
    {     
        buttonCountLow ++;
        stillPressed = 0;
        buttonCount = 0;
    }
    //Here begins the logic of figuring out what the next move in the state machine is
    //FIXME  This may prevent the manual stop of a log while the engine is running
    //Manual button only for track side test with engine off? maybe for zeroing sensors
    if((buttonCount >= countThreshold)&& state == wait && stillPressed == 0){ //Should prevent oscillations caused by holding down the button
        if(rpm > LogRPM){
        engineOff = 0; //Engine is currently on
        }
        buttonCount = 0;
        buttonCountLow = 0;
        stillPressed = 1;
        return 1;//transition from wait to log
    }
//    else if((buttonCount >= countThreshold ||rpm < LogRPM) && state == log && buttonCountLow > countThreshold){
//        //Start Delay of 5s
//        engineOff = 1; //Engine is now off
//        stopDelayCounter = 0;
//        buttonCount = 0;
//        buttonCountLow = 0;
//        return 0;
//    }
//    else if(engineOff == 1 && stopDelayCounter >= 5000 ){//switch to a wait state, 5 seconds have elapsed
//        stopDelayCounter = 0;
//        return 2; //transition from log to wait
//    }
    else if(buttonCount >= countThreshold && stillPressed == 0 && state == log)
    {
        buttonCount = 0;
        buttonCountLow = 0;
        stillPressed = 1;
        return 2;
    }
    else{ //Catchall waiting state
        return 0;
    }
}
BOOL checkForButton2(){ //Probably can get rid of this, I don't think it is used anymore
    //look for rising edge in button1
  //  if((PORTG & 0x80) != 0x80){
    if((PORTG & 0x8000) == 0x8000){
        prevButton2 = 0;
        return FALSE;
    }else if( prevButton2 == 0){
        DelayMs(20);
  //      if(PORTG & 0x80){
        if((PORTG & 0x8000) != 0x800){
            prevButton2 = 1;
            return TRUE;
        }
        return FALSE;
    }else{
        prevButton2 = 1;
        return FALSE;
    }
    return FALSE;
}
void WriteAccelData(CANRxMessageBuffer *message)
{
//Accel
    char string[20];
    int eidE = message->msgEID.IDE;
    int eidD = message->msgEID.DLC;
    int eidEid = ((message->msgEID.EID) & 0xFFFF);

    BYTE data[8];
    int i;
    for(i = 0; i < 8; i++){
        data[i] = message->data[i];
    }
    int messageWord[4];
    for(i = 0; i < 4; i++){
        messageWord[i] = message->messageWord[i];
    }
    if(eidEid == 0x2d30){
        int i;
        for(i = 0; i < 8; i++){
            accelerationSensor[i] = data[i];
        }
        accelerationSensorRec = TRUE;
    }else if(eidEid == 0x2a30){
        int i;
        for(i = 0; i < 8; i++){
            angularRateInfo[i] = data[i];
        }
        angularRateInfoRec = TRUE;
    }else if(eidEid == 0x0030){
        int i;
        for(i = 0; i < 8; i++){
            HRaccelerationSensor[i] = data[i];
        }
        HRaccelerationSensorRec = TRUE;
    }
}

void writeCan2Msg(CANRxMessageBuffer *message){
    BYTE data[8];
    int i;
    for(i = 0; i < 8; i++){
        data[i] = message->data[i];
    }
    unsigned int sid = message->msgSID.SID;
    if(sid == 0x6f0){
        rpm = data[0] << 8 | data[1];
        tp = data[2] << 8 | data[3];
        map = data[4] << 8 | data[5];
        at = data[6] << 8 | data[7];
        motec0Read = TRUE;
    }else if(sid == 0x6f1){
        et = data[0] << 8 | data[1];
        la1 = data[2] << 8 | data[3];
        fp = data[4] << 8 | data[5];
        op = data[6] << 8 | data[7];
        motec1Read = TRUE;
    }else if(sid == 0x6f2){
        egt1 = data[0] << 8 | data[1];
        launch = data[2] << 8 | data[3];
        ntrl = data[4] << 8 | data[5];
        bp = data[6] << 8 | data[7];
        motec2Read = TRUE;
    }else if(sid == 0x6f3){
        bpf1 = data[0] << 8 | data[1];
        batV = data[2] << 8 | data[3];
        ldSpd = data[4] << 8 | data[5];
        lgSpd = data[6] << 8 | data[7];
        motec3Read = TRUE;
    }else if(sid == 0x6f4){
        rdSpd = data[0] << 8 | data[1];
        rgSpd = data[2] << 8 | data[3];
        runTime = data[4] << 8 | data[5];
        motec4Read = TRUE;
        fuel = data[6] << 8| data[7];
    }
        else if(sid == 0x6f5){
        oilTemp = data[0] << 8 | data[1];
        ignTime = data[2] << 8 | data[3];
        motec5Read = TRUE;
    }

}
//the next few functions are general for i2c
//all these functions were stolen from Kralick_Lab4
void initI2CBus(I2C_MODULE id, int pBusFrq, int i2cFrq){
    I2CConfigure(id, 0);
    I2CSetFrequency(id, pBusFrq, i2cFrq);
    I2CEnable(id, TRUE);
}
void i2cStart(I2C_MODULE id){
    while( !I2CBusIsIdle(id) );
    I2CStart(id);
    while ( !(I2CGetStatus(id) & I2C_START) );
}void i2cSendByte(I2C_MODULE id, BYTE data){
    while(!I2CTransmitterIsReady(id));//maybe comment this out
    I2CSendByte(id, data);
    //I2C_RESULT temp = I2CSendByte(id, data);
    //temp;
    while(!I2CTransmissionHasCompleted(id));
    while(!I2CByteWasAcknowledged(id));
}
void i2cRepeatedStart(I2C_MODULE id){
    I2CRepeatStart(id);
    while ( !(I2CGetStatus(id) & I2C_START) );
}
void i2cStop(I2C_MODULE id){
    I2CStop(id);
    while ( !(I2CGetStatus(id) & I2C_STOP) );
}
BYTE i2cRecieveByte(I2C_MODULE id, BOOL ack){
    I2CReceiverEnable(id, TRUE);
    while(!I2CReceivedDataIsAvailable(id));
    I2CAcknowledgeByte(id, ack);
    while(!I2CAcknowledgeHasCompleted(id)) ;
    BYTE result = I2CGetByte(id);
    return result;
}
//setup I2C to talk to the eeprom
void initI2CEEPROM(){
    initI2CBus(I2C2, getPBusClock(), 400000);
}
void initI2CPSoC(){
    initI2CBus(I2C1, 80000000, 100000); //100kbs
}
//Need to implement a check here so that it doesn't get stuck if the PSOC isn't attached
void PSOC_Read(){
    //First 12 bytes are current sensor data, next 8 bytes are shock pots
    BYTE PSOC_Data[24];
    int i;
    i2cStart(I2C1);
    i2cSendByte(I2C1, (0x08 << 1) + 0) ;//send addresss and write
    i2cSendByte(I2C1, 0x00);//send data pointer
    i2cRepeatedStart(I2C1); //Repeat start to change data direction

  //  i2cStart(I2C1);
    i2cSendByte(I2C1, (0x08 << 1) + 1) ;//send addresss and read
    for(i = 0 ; i < 23; i++){
        PSOC_Data[i] = i2cRecieveByte(I2C1, TRUE);
    }
    PSOC_Data[23] = i2cRecieveByte(I2C1, FALSE);
    i2cStop(I2C1);//Stop the bus
    //Combine the bytes
    for(i = 0; i<12; i++){
    PSOC_volts[i] = (PSOC_Data[2*i+1] <<8  | PSOC_Data[2*i]);
    }
 }

void WriteToUSB(){
if (state == log){
            unsigned millisec_USE = millisec;
            double pitch, roll, yaw;
            double latAcc, longAcc, vertAcc;
            double HRlatAcc, HRlongAcc, HRvertAcc;
            char angString[40];
            char accString[40];
            char HRaccString[40];
            char motec0String[40];
            char motec1String[40];
            char motec2String[40];
            char motec3String[40];
            char motec4String[40];
            char motec5String[40];
            char PSOCstring0[40];
            char PSOCstring1[40];
            char PSOCstring2[40];
            char dataFlagString[40];
            char timeString[40];

            sprintf(timeString,"%d,", millisec_USE);
            if(angularRateInfoRec){
                pitch = (((double)(angularRateInfo[1] << 8 | angularRateInfo[0])) /128) - 250;
                roll = (((double)(angularRateInfo[3] << 8 | angularRateInfo[2])) /128) - 250;
                yaw = (((double)(angularRateInfo[5] << 8 | angularRateInfo[4])) /128) - 250;
                angularRateInfoRec = FALSE;
                sprintf(angString, "%.6f,%.6f,%.6f,", pitch, roll, yaw);
            }else{
                sprintf(angString, " , , ,");
            }
            if(accelerationSensorRec){
                latAcc = (((double)(accelerationSensor[1] << 8 | accelerationSensor[0])) * .01) - 320;
                longAcc = (((double)(accelerationSensor[3] << 8 | accelerationSensor[2])) * .01) - 320;
                vertAcc = (((double)(accelerationSensor[5] << 8 | accelerationSensor[4])) * .01) - 320;
                accelerationSensorRec = FALSE;
                sprintf(accString, "%.6f,%.6f,%.6f,", latAcc, longAcc, vertAcc);
            }else{
                sprintf(accString, " , , ,");
            }
            if(HRaccelerationSensorRec){
                HRlatAcc = (((double)(HRaccelerationSensor[1] << 8 | HRaccelerationSensor[0])) * .000599) - 19.62;
                HRlongAcc = (((double)(HRaccelerationSensor[3] << 8 | HRaccelerationSensor[2])) * .000599) - 19.62;
                HRvertAcc = (((double)(HRaccelerationSensor[5] << 8 | HRaccelerationSensor[4])) * .000599) - 19.62;
                HRaccelerationSensorRec = FALSE;
                sprintf(HRaccString, "%.6f,%.6f,%.6f,", HRlatAcc, HRlongAcc, HRvertAcc);
            }else{
                sprintf(HRaccString, " , , ,");
            }
            if(motec0Read){
                int t_rpm = rpm;
                double t_tps = (double) tp * .1;
                double t_map = (double) map *0.1;
                double t_at = (double) at*0.1;
                sprintf(motec0String, "%d,%.6f,%.6f,%.6f,", t_rpm, t_tps, t_map, t_at );
                motec0Read = FALSE;
            }else{
                sprintf(motec0String, " , ,");
            }
            if(motec1Read){
                double t_engineTemp = (double) et * .1;
                double t_lambda1 = (double) la1 * .001;
                int t_fuelPress = fp;
                sprintf(motec1String, "%.6f,%.6f,%d,", t_engineTemp, t_lambda1, t_fuelPress);
                motec1Read = FALSE;
            }else{
                sprintf(motec1String, " , , ,");
            }
            if(motec2Read){
                double t_egt1 = (double)egt1;
                int t_userLaunch = launch;//not sure scaling
                int t_userNtrl = ntrl;//not sure scaling
                int t_brakePres = bp;//not sure scaling
                sprintf(motec2String, "%.6f,%d,%d,%d,", t_egt1, t_userLaunch, t_userNtrl, t_brakePres);
                motec2Read = FALSE;
            }else{
                sprintf(motec2String, " , , , ,");
            }
            if(motec3Read){
                int t_brakePresFil = bpf1;//not sure scaling
                double t_batV = (double)batV * .01;
                double t_ldspd = (double)ldSpd * .1;
                double t_lgspd = (double)lgSpd * .1;
                sprintf(motec3String, "%d,%.6f,%.6f,%.6f,", t_brakePresFil, t_batV, t_ldspd, t_lgspd);
                motec3Read = FALSE;
            }else{
                sprintf(motec3String, " , , , ,");
            }
            if(motec4Read){
                double t_rdspd = (double)rdSpd * .1;
                double t_rgspd = (double)rgSpd * .1;
                float t_runTime = runTime * .1;//not sure scaling
                sprintf(motec4String, "%.6f,%.6f,%.6f, %d,", t_rdspd, t_rgspd, t_runTime, fuel);
                motec4Read = FALSE;
            }else{
                sprintf(motec4String, " , , , ,");
            }
            if(motec5Read){
                double oilTempDegF = (double)oilTemp * .1;
                double ignTimeL = (double)ignTime * 0.1;
                sprintf(motec5String, "%.6f, %.6f,", oilTempDegF, ignTimeL);
                motec5Read = FALSE;
            }else{
                sprintf(motec5String, ", ,");
            }
            //PSOC
            if(PSOCConnected){ //This allows for the program to be tested without the PSOC connected.  PSOC read is a global variable defined at the top of the file
            PSOC_Read();
            sprintf(PSOCstring0,"%d,%d,%d,%d,%d,%d,",PSOC_volts[0],PSOC_volts[1],PSOC_volts[2],PSOC_volts[3],PSOC_volts[4],PSOC_volts[5]); //Current Sensors
            sprintf(PSOCstring1,"%d,%d,%d,%d,",PSOC_volts[6],PSOC_volts[7],PSOC_volts[8],PSOC_volts[9]); //Shock Pots
            sprintf(PSOCstring2,"%d,%d,",PSOC_volts[10],PSOC_volts[11]); //Steering Angle And Brake temp and millisec counter
            }
            else{
                sprintf(PSOCstring0," , , , , , , ");
                sprintf(PSOCstring1," , , , ,");
                sprintf(PSOCstring2,", ,");
            }
            //dataFlag
            sprintf(dataFlagString,"%d,",dataFlag);
            //Write all strings to the CSV file
            FSfwrite(timeString,1,strlen(timeString),myFile);
            FSfwrite(angString,1, strlen(angString),myFile);
            FSfwrite(accString,1, strlen(accString),myFile);
            FSfwrite(HRaccString,1, strlen(HRaccString),myFile);
            FSfwrite(motec0String,1, strlen(motec0String),myFile);
            FSfwrite(motec1String,1, strlen(motec1String),myFile);
            FSfwrite(motec2String,1, strlen(motec2String),myFile);
            FSfwrite(motec3String,1, strlen(motec3String),myFile);
            FSfwrite(motec4String,1, strlen(motec4String),myFile);
            FSfwrite(motec5String,1, strlen(motec5String),myFile);
            FSfwrite(PSOCstring0,1, strlen(PSOCstring0),myFile);
            FSfwrite(PSOCstring1,1, strlen(PSOCstring1),myFile);
            FSfwrite(PSOCstring2,1, strlen(PSOCstring2),myFile);
            FSfwrite(dataFlagString,1,strlen(dataFlagString),myFile);
            FSfwrite(GPSLogData,1,strlen(GPSLogData),myFile); //The NMEA sentence had a \n character at the end, no need to manually include it
            //FSfwrite(",123\n",1,5,myFile);
        }
}

//write a byte to eeprom
void writeEEPROM(short address, BYTE data){
    i2cStart(I2C2);
    i2cSendByte(I2C2, 0x50 << 1);
    i2cSendByte(I2C2, (address & 0xFF00)>> 8);
    i2cSendByte(I2C2, address & 0xFF);
    i2cSendByte(I2C2, data);
    i2cStop(I2C2);
}
//write several bytes to the eeprom
void writeMultiEEPROM(short address, BYTE *data, int length){
    i2cStart(I2C2);
    i2cSendByte(I2C2, 0x50 << 1);
    BYTE high = (address & 0xFF00) >> 8;
    BYTE low = address & 0xFF;
    i2cSendByte(I2C2, high);
    i2cSendByte(I2C2, low);
    int i;
    for(i = 0; i < length; i++){
        i2cSendByte(I2C2, data[i]);
    }
    i2cStop(I2C2);
}
//read a byte from the eeprom
BYTE readEEPROM(BYTE address){
    i2cStart(I2C2);
    i2cSendByte(I2C2, 0x50 << 1);
    i2cSendByte(I2C2, (address & 0xFF00)>> 8);
    i2cSendByte(I2C2, address & 0xFF);
    i2cRepeatedStart(I2C2);
    i2cSendByte(I2C2, 0x50 << 1 | 0x01);
    BYTE temp = i2cRecieveByte(I2C2, FALSE);
    i2cStop(I2C2);
    return temp;
}
//read multiple bytes from the eeprom
BOOL readMultiEEPROM(BYTE address, BYTE *data, int length){
    BOOL keepReading = TRUE;
    i2cStart(I2C2);
    i2cSendByte(I2C2, 0x50 << 1);
    i2cSendByte(I2C2, (address & 0xFF00)>> 8);
    i2cSendByte(I2C2, address & 0xFF);
    i2cRepeatedStart(I2C2);
    i2cSendByte(I2C2, 0x50 << 1 | 0x01);
    int i;
    for(i = 0; i < length-1; i++){
        *(data + i) = i2cRecieveByte(I2C2, TRUE);
        if(*(data+i)  == 0xFF){
            keepReading = FALSE;
        }
        if(keepReading == FALSE){
            *(data +i) = 0xFF;
        }
    }
    *(data + length - 1) = i2cRecieveByte(I2C2, FALSE);
    i2cStop(I2C2);
    return keepReading;
}

BYTE UARTReceiveByte(UART_MODULE uart)
{
      return UARTGetDataByte(uart); //This function is only called when receive data is ready, therefore no check is needed
}

void GPSDataRead(){
    BYTE receive;
    if(UARTReceivedDataIsAvailable(UART2))
    {
    receive = UARTReceiveByte(UART2);
    GPSIndex++;
    newData = 1;
    }
    if(newData == 1){
        if(!(receive=='.'||receive=='0'||receive=='1'||receive=='2'||receive=='3'||receive=='4'||receive=='5'||receive=='6'||receive=='7'||receive=='8'||receive=='9'||receive==','||receive==0x0A||receive==0x0D))
           receive = '3'; //Make all characters that are not important a 3
       // UARTSendByte(UART1,receive);
        newData = 0;
        GPSData[GPSIndex] = receive;
        GPSIndex++;
        if(receive == 0xA){
           strncpy(GPSLogData, GPSData, strlen(GPSData));
            GPSIndex = 0;
          //  LineDone = 1;
           // UARTSendByte(UART1,0xA);
        }
    }
    return 0;
}

void ClutchHold(){
    double lgspeeduse = (double)lgSpd * .1;
    if(rpm > 6000 && lgspeeduse < 5){
        LATDCLR = 0x4000; //Vent clutch through flow regulator
    }
    else
        LATDSET = 0x4000; //Vent clutch to atmosphere
}

void DataFlagFunc(){
    if(PORTA & 0x20)
        dataFlag = 1;
    else
        dataFlag = 0;
}

int main(void)
{
    int  value;
    int junk;
    millisec = 0;
    value = SYSTEMConfigWaitStatesAndPB( GetSystemClock() );

    // Enable the cache for the best performance
    CheKseg0CacheOn();

    //Setupt input for inteface button JF8 (RA01) (0x02)
    TRISASET = 0x02;
    //RED LED - JF9 (RA04)  (0x10)
    TRISACLR = 0x10;
    ODCACLR = 0x10;
    LATASET = 0x10;
    //Green LED -JF7 (RE9)  (0x200)
    TRISECLR = 0x200;
    ODCECLR = 0x200;
    LATESET = 0x200;
    //Setupt Input for DataFlag Button - JF10 - RA5 0x20
    TRISASET = 0x20;
    //Setup Output for Clutch Hold (Launch) JE1 RD14 0x4000
    //This function is active low, driving the FET on the PDU
    TRISDCLR = 0x4000;
    ODCDCLR = 0x4000;
    LATDSET = 0x4000; //Default state is high (off)

    CAN1Init();//CAN1 ACCL 500kbs
    CAN2Init();//Motec 1mbs
    DelayInit();

    initUART1();
    initUART2(); // GPS UART
    prevButton1 = 0;
    prevButton2 = 0;
    millisec = 0;

   // Configure Timer 2 to request a real-time interrupt once per millisecond.
   // The period of Timer 2 is (16 * 5000)/(80 MHz) = 1 ms.
   OpenTimer2(T2_ON | T2_IDLE_CON | T2_SOURCE_INT | T2_PS_1_16 | T2_GATE_OFF, 5000);

   // Configure the CPU to respond to Timer 2's interrupt requests.
   INTEnableSystemMultiVectoredInt();
   INTSetVectorPriority(INT_TIMER_2_VECTOR, INT_PRIORITY_LEVEL_1);
   INTClearFlag(INT_T2);
   INTEnable(INT_T2, INT_ENABLED);

    value = OSCCON;
    while (!(value & 0x00000020))
    {
        value = OSCCON;    // Wait for PLL lock to stabilize
    }

    deviceAttached = FALSE;

    //Initialize the stack
    USBInitialize(0);

    shouldLog = FALSE;
    shouldStop = FALSE;
    //count = 0;
    angularRateInfoRec = FALSE;
    accelerationSensorRec = FALSE;
    HRaccelerationSensorRec = FALSE;
    analogRead = FALSE;
    analogIn1 = 0;
    analogIn2 = 0;

    INTSetVectorPriority(INT_ADC_VECTOR, INT_PRIORITY_LEVEL_1);
    INTEnable(INT_AD1, INT_ENABLED);
    INTClearFlag(INT_AD1);  //clear flag to avoid spurious interrupt

    //current statys of ADC is that sampling 0-3.3V from JA-03 and JA-04 with some weird things happing in the data, it could just be a bad joystick or weird data things happing

    //init tim er 3 to convert adc at 100hz
    OpenTimer3(T3_ON|T3_PS_1_256|T3_SOURCE_INT, 1562);

    //set ADC to sample analog pin2
    SetChanADC10( ADC_CH0_NEG_SAMPLEA_NVREF | ADC_CH0_POS_SAMPLEA_AN4 | ADC_CH0_NEG_SAMPLEB_NVREF | ADC_CH0_POS_SAMPLEB_AN6);

    //open ADC to give unsigned integers and start conversions on timer3
    //use 0v and 3.3V take one sample per interrupt, use analog pin2
    OpenADC10(ADC_MODULE_ON|ADC_FORMAT_INTG32|ADC_CLK_TMR|ADC_AUTO_SAMPLING_ON ,
        ADC_VREF_AVDD_AVSS |ADC_SAMPLES_PER_INT_2 | ADC_ALT_INPUT_ON,
        ADC_CONV_CLK_PB|ADC_CONV_CLK_Tcy,
        ENABLE_AN4_ANA | ENABLE_AN6_ANA,
        SKIP_SCAN_AN0 | SKIP_SCAN_AN1 | SKIP_SCAN_AN2 | SKIP_SCAN_AN3 | SKIP_SCAN_AN5 | SKIP_SCAN_AN7 | SKIP_SCAN_AN8 | SKIP_SCAN_AN9 | SKIP_SCAN_AN10 | SKIP_SCAN_AN11 | SKIP_SCAN_AN12 | SKIP_SCAN_AN13 | SKIP_SCAN_AN14 | SKIP_SCAN_AN15);

    EnableADC10();
    //AD1CON1SET = 0x4;
    //TRISBCLR = 5 << 7;//for adc this was also screwing up the adc making things be randomly 1ff
    //PORTBSET = 1 << 6; //this will make adc respoint with binary haha

    //initialize i2c for the psoc
    initI2CPSoC();
    
    state = wait;
    logNum = 0;

    initI2CEEPROM();
    short addy = 0x0000;
    BYTE num = 0x00;
    logNum = readEEPROM(addy);
    if(logNum >= 0xEF)  //Address stored in EEPROM  if greater than 0xEF reset to zero, limited to a single byte with current code configuration
    {
        writeEEPROM(addy, 0x00);
    }
    char GroupString[550];//Group Names (Line1)
    char UnitString[550];//Units (line2)
    char ParamString[650];//Paramater Names (line3)
    sprintf(GroupString,"Time,Accelerometer,Accelerometer,Accelerometer,Accelerometer,Accelerometer,Accelerometer,Accelerometer,Accelerometer,Accelerometer,Engine,Engine,Engine,Engine,Engine,Engine,Engine,Engine,Engine,Engine,Drivetrain,Drivetrain,Electrical,Drivetrain,Drivetrain,Drivetrain,Drivetrain,Engine,Engine,Engine,Engine,Electrical,Electrical,Electrical,Electrical,Electrical,Electrical,Suspension,Suspension,Suspension,Suspension,Suspension,Drivetrain,Driver\n");
    sprintf(UnitString,"ms,deg/s,deg/s,deg/s,m/s^2,m/s^2,m/s^2,m/s^2,m/s^2,m/s^2,rpm,%,kpa,degF,degF,lambda,psi,degF,na,na,psi,psi,V,mph,mph,mph,mph,s,gal,degF,degBTDC,mV,mV,mV,mV,mV,mV,mV,mV,mV,mV,mV,mV,\n");
    sprintf(ParamString, "Millisec,pitch(deg/sec),roll(deg/sec),yaw(deg/sec),lat(m/s^2),long(m/s^2),vert(m/s^2),latHR(m/s^2),longHR(m/s^2),vertHR(m/s^2),rpm,tps(percent),MAP(kpa),AT(degF),ect(degF),lambda,fuel pres,egt(degF),launch,neutral,brake pres,brake pres filtered,BattVolt(V),ld speed(mph), lg speed(mph),rd speed(mph),rg speed(mph),run time(s),fuel used,Oil Temp (deg F), Ignition Adv (degBTDC),Overall Consumption(mV),Overall Production(mV),Fuel Pump(mV),Fuel Injector(mV),Ignition(mV),Vref(mV),Back Left(mV),Back Right(mV),Front Left(mV),Front Right(mV),Steering Angle(mV),Brake Temp(mV),Data Flag,GPRMC,Time,Valid,Lat,N/S,Long,E/W,Speed,Course,Date,Variation,E/W\n");


    LATACLR = 0x10; //Turn on Red LED
   // LATECLR = 0x200;

    UARTSendString(UART2,PMTK_HOT_RESTART);
    int i = 0;
    while(!UARTTransmissionHasCompleted(UART2)){
        i++;
    }

//    char test;
//    while(test != 0xA){
//           if(UARTReceivedDataIsAvailable(UART2))
//        {
//        test = UARTReceiveByte(UART2);
//        }
//    }

    while(1)
    {
        //GPS Handler
        //FIXME -  need to figure out if a few lines need to be removed from the RX buffer before log begins.
        GPSDataRead();
      //  ClutchHold(); //This function handles the venting direction of the clutch actuator
       // DataFlagFunc(); //This function handles the updates of the data flag variable
        //USB stack process function
        USBTasks();

        switch(state){
            case wait:
                USBTasks();
                millisec = 0;
                if(CheckLogStateChange() == 1){ //start the transition from wait to log
                    state = startLog;
                }
                break;
                //Try to remove this state, just bounce between log and wait
//            case init:
//                if(CheckLogStateChange() == 1){
//                    state = startLog;
//                }
//                break;
            case startLog:
                //if thumbdrive is plugged in
                if(USBHostMSDSCSIMediaDetect())
                {
                    deviceAttached = TRUE;
                    //now a device is attached
                    //See if the device is attached and in the right format
                    if(FSInit())
                    {
                        //Opening a file in mode "w" will create the file if it doesn't
                        //  exist.  If the file does exist it will delete the old file
                        //  and create a new one that is blank.
                        logNum = readEEPROM(addy);
                        sprintf(nameString, "test%d.csv", logNum);
                        myFile = FSfopen(nameString,"w");
                        FSfwrite(GroupString,1,strlen(GroupString),myFile);
                        FSfwrite(UnitString,1,strlen(UnitString),myFile);
                        FSfwrite(ParamString,1, strlen(ParamString),myFile);
                        millisec = 0;
                        //LATDSET = 0x4000; //Send sync pulse (aeroprobe)
                       // while(millisec < 1000){} //Wait 1s then move to log, the aeroprobe ADC waits 1s.
                            state = log;
                        LATECLR = 0x200; //Turn on Green
                        LATASET = 0x10; //Turn off Red
                    }
                }
                break;
            case log:
                //This uses MOTEC as the master timer.  Data is only written to the USB after all the motec Data is received
                if(motec0Read && motec1Read && motec2Read && motec3Read && motec4Read && motec5Read){
                    WriteToUSB();
                }
                else{}//Wait for motec data to write the next row
                if(CheckLogStateChange() == 2){ //Start the transition from log to wait
                    state = stopLog;
                }
                if(millisec > 2000){
                    LATDCLR = 0x4000; //After 2 seconds pass no need to keep output high
                }
                //Add a function to check for a flag button and set a variable
                break;
            case stopLog:
                //Always make sure to close the file so that the data gets written to the drive.
                FSfwrite("endFile", 1, 7, myFile);
                FSfclose(myFile);
                state = wait;
                logNum++;
                writeEEPROM(addy, logNum);
                LATACLR = 0x10; //Turn on Red
                LATESET = 0x200; //Turn off Green
                break;
            default:
                state = wait;
                break;
        }


        //CAN Handlers
        CANRxMessageBuffer* CAN1RxMessage = CAN1RxMsgProcess();
        if(CAN1RxMessage){
            WriteAccelData(CAN1RxMessage); //Accel is on CAN 1
        }
        CANRxMessageBuffer* CAN2RxMessage = CAN2RxMsgProcess();
        if(CAN2RxMessage){
            writeCan2Msg(CAN2RxMessage); //Motec is on CAN 2
        }
    }
    return 0;
}

/****************************************************************************
  Function:
    BOOL USB_ApplicationEventHandler( BYTE address, USB_EVENT event,
                void *data, DWORD size )

  Summary:
    This is the application event handler.  It is called when the stack has
    an event that needs to be handled by the application layer rather than
    by the client driver.

  Description:
    This is the application event handler.  It is called when the stack has
    an event that needs to be handled by the application layer rather than
    by the client driver.  If the application is able to handle the event, it
    returns TRUE.  Otherwise, it returns FALSE.

  Precondition:
    None

  Parameters:
    BYTE address    - Address of device where event occurred
    USB_EVENT event - Identifies the event that occured
    void *data      - Pointer to event-specific data
    DWORD size      - Size of the event-specific data

  Return Values:
    TRUE    - The event was handled
    FALSE   - The event was not handled

  Remarks:
    The application may also implement an event handling routine if it
    requires knowledge of events.  To do so, it must implement a routine that
    matches this function signature and define the USB_HOST_APP_EVENT_HANDLER
    macro as the name of that function.
  ***************************************************************************/

BOOL USB_ApplicationEventHandler( BYTE address, USB_EVENT event, void *data, DWORD size )
{
    switch( event )
    {
        case EVENT_VBUS_REQUEST_POWER:
            // The data pointer points to a byte that represents the amount of power
            // requested in mA, divided by two.  If the device wants too much power,
            // we reject it.
            return TRUE;

        case EVENT_VBUS_RELEASE_POWER:
            // Turn off Vbus power.
            // The PIC24F with the Explorer 16 cannot turn off Vbus through software.

            //This means that the device was removed
            deviceAttached = FALSE;
            return TRUE;

        case EVENT_HUB_ATTACH:
            return TRUE;

        case EVENT_UNSUPPORTED_DEVICE:
            return TRUE;

        case EVENT_CANNOT_ENUMERATE:
            return TRUE;

        case EVENT_CLIENT_INIT_ERROR:
            return TRUE;

        case EVENT_OUT_OF_MEMORY:
            return TRUE;

        case EVENT_UNSPECIFIED_ERROR:   // This should never be generated.
            return TRUE;

        default:
            break;
    }

    return FALSE;
}


void __ISR(_TIMER_2_VECTOR, ipl1) Timer2_ISR(void) {
    if (INTGetFlag(INT_T2)){
         millisec++;
         stopDelayCounter++;
         INTClearFlag(INT_T2);   // Acknowledge the interrupt source by clearing its flag.
    }
}

//adding the following stopped the error
//undefined reference to `_p32mxsk_putc
//that was caused by using sprintf
void _mon_putc(char c) {}


void __attribute__((vector(_ADC_VECTOR), interrupt(ipl4), nomips16)) ADC_ISR(void)
{
    if (INTGetFlag(INT_AD1)){
        //when conversion is complete write ADC word to the variable so it can be displayed
        analogIn1 = ReadADC10(0);
        analogIn2 = ReadADC10(1);
        analogRead = TRUE;
        INTClearFlag(INT_AD1);
    }
}