/*
  VTM17e
  Data Logging Code
  Danny Lloyd
  December 4, 2016
 */

#define _PLIB_DISABLE_LEGACY
#include "usb.h"
#include "usb_config.h"
#include "usb_host_msd.h"
#include "usb_host_msd_scsi.h"
#include "FSIO.h"

#include "PmodOLED.h"
#include "OledChar.h"
#include "OledGrph.h"
#include "delay.h"
#include "i2c.h"
#include "i2cLCD.h"
#include "CANFunctions.h"
#include "stdbool.h"

// Digilent board configuration
#ifndef OVERRIDE_CONFIG_BITS
#pragma config UPLLEN       = ON        // USB PLL Enabled
#pragma config FPLLMUL      = MUL_20	// PLL multiplier
#pragma config UPLLIDIV     = DIV_2     // USB PLL Input Divider
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
#endif

#define getPBusClock() 80000000
#define UART_BAUD 9600
#define SYS_FREQ (80000000L)

BYTE CAN1MessageFifoArea[2 * 8 * 16];
BYTE CAN2MessageFifoArea[2 * 8 * 16];

// Parameters received off CAN bus and corresponding Node IDs
signed int motorSpeed;
int nodeStatus, rmsMotorCurrent; // Node ID 0x187 Variables (from Inverter)
float nodeDCVoltage;
float dcLowThreshold=75;
int heatSinkTemp, motorTemp, dcCurrent, voltageAngle; // Node ID 0x287 Varaibles (also from Inverter)
int maxMotorTemp = 120;
int vmc30State, steeringFiltered, throttleFiltered, errorBits; // Node ID 0x205 Variables (from VMC30)
int commandWord, commandCurrent, accelCurrentLimit, decelCurrentLimit; // Node ID 0x201 Variables (from VMC30)
int bmsCCL, bmsDCL, bmsHighTemp, bmsPackCurrent; // Node ID 0x183 (from BMS)
int cellid, instVoltage, openVoltage, checkSum; // Node ID 0x100 (from BMS)
BOOL inRegen1; // Node ID 0x305 (from VMC30)

int prevButton1;
int prevButton2;
int counter = 0;
int fix = 0;

FSFILE * myFile;
BYTE myData[512];
size_t numBytes;
volatile BOOL deviceAttached;
BOOL shouldLog;
BOOL shouldStop;

int stillPressed = 0;
int buttonCount = 0;
int buttonCountLow = 0;

BOOL dataRead187, dataRead201, dataRead205, dataRead287, dataRead305, dataRead183, dataRead100; // Number is Node ID from VMC30

typedef enum{
    init, startLog, log, stopLog, wait

}STATE;
int logNum;
STATE state;

typedef struct{
    int id;
    int length;
    BYTE data[8];

}CAN_MSG;
CAN_MSG canRecent[4];

void initUART1(){
    UARTConfigure(UART1, UART_ENABLE_PINS_TX_RX_ONLY);//configure uart for rx and tx
    UARTSetFifoMode(UART1, UART_INTERRUPT_ON_TX_NOT_FULL | UART_INTERRUPT_ON_RX_NOT_EMPTY);//configure for interrupts
    UARTSetLineControl(UART1,  UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);//set uart line options
    UARTSetDataRate(UART1, getPBusClock(), UART_BAUD);//data rate 9600 baud and pbus 10MHz
    UARTEnable(UART1, UART_PERIPHERAL | UART_RX | UART_TX | UART_ENABLE); //enable the uart for tx rx
    while(!UARTTransmitterIsReady(UART1));
}

BOOL checkForButton1(){
    //look for rising edge in button1
    if((PORTG & 0x40) != 0x40){
        prevButton1 = 0;
        return FALSE;
    }else if( prevButton1 == 0){
        DelayMs(20);
        if(PORTG & 0x40){
            prevButton1 = 1;
            return TRUE;
        }
        return FALSE;
    }else{
        prevButton1 = 1;
        return FALSE;
    }
    return FALSE;
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
    if(PORTF & 0x20){
        buttonCount++;
        buttonCountLow = 0;

    }
    else if(!(PORTF & 0x20))
    {
        buttonCountLow ++;
        stillPressed = 0;
        buttonCount = 0;
    }
    //Here begins the logic of figuring out what the next move in the state machine is
    //FIXME  This may prevent the manual stop of a log while the engine is running
    //Manual button only for track side test with engine off? maybe for zeroing sensors
    if((buttonCount >= countThreshold)&& (state == wait || state == init) && stillPressed == 0){ //Should prevent oscillations caused by holding down the button
        buttonCount = 0;
        buttonCountLow = 0;
        stillPressed = 1;
        return 1;//transition from wait to log
    }
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

BOOL checkForButton2(){
    //look for rising edge in button1
    if((PORTG & 0x80) != 0x80){
        prevButton2 = 0;
        return FALSE;
    }else if( prevButton2 == 0){
        DelayMs(20);
        if(PORTG & 0x80){
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

void startUpScreen()
{
    LCDsetFontEnlargement(0x03);
    LCDsetFontWriteCursorLoc(170, 130);
    LCDsendString("VT Motorsports");
    LCDsetFontWriteCursorLoc(220, 210);
    LCDsendString("Formula SAE");
    DelayMs(1500);
    initializeLCD();
}

BOOL getCANMessages(CANRxMessageBuffer * message){
    // Gets messages on CAN bus
    // Assigns values based on SID to global variables also used for DAQ
    // Converts values to readable format where needed after bytes are assembled
    // Format for VMC30: Most Significant Byte is FIRST, Little Endian
    int id = message->msgSID.SID;

    switch(id){
        case 0x187:
            motorSpeed = message->data[0] + ((message->data[1])<<8);
            nodeStatus = message->data[2] + ((message->data[3])<<8);
            nodeDCVoltage = message->data[4] + ((message->data[5])<<8);
            rmsMotorCurrent = message->data[6] + ((message->data[7])<<8);
            nodeDCVoltage = nodeDCVoltage/100; // VMC30 displays in centiVolts due to lack of floating point unit
            dataRead187=TRUE;
            return true;
            break;
        case 0x100:
            cellid = message->data[0];
            cellid++;
            instVoltage = message->data[2] + ((message->data[1])<<8);
            openVoltage = message->data[6] + ((message->data[5])<<8);
            checkSum = message->data[7];
            dataRead100=TRUE;
            return true;
            break;
        case 0x287:
            heatSinkTemp = message->data[0] + ((message->data[1])<<8);
            motorTemp = message->data[2] + ((message->data[3])<<8);
            dcCurrent = message->data[4] + ((message->data[5])<<8);
            voltageAngle = message->data[6] + ((message->data[7])<<8);
            dataRead287=TRUE;
            return true;
            break;
        case 0x205:
            vmc30State = message->data[0] + ((message->data[1])<<8);
            steeringFiltered = message->data[2] + ((message->data[3])<<8);
            throttleFiltered = message->data[4] + ((message->data[5])<<8);
            errorBits = message->data[6] + ((message->data[7])<<8);
            dataRead205=TRUE;
            return true;
            break;
        case 0x201:
            commandWord = message->data[0] + ((message->data[1])<<8);
            commandCurrent = message->data[2] + ((message->data[3])<<8);
            accelCurrentLimit = message->data[4] + ((message->data[5])<<8);
            decelCurrentLimit = message->data[6] + ((message->data[7])<<8);
            dataRead201=TRUE;
            return true;
            break;
        case 0x305:
            fix = 1; // Needed because C doesnt like a declaration at beginning of case statement
            int y;
            y = message->data[0] + ((message->data[1])<<8);
            int na1 = message->data[2] + ((message->data[3])<<8);
            int na2 = message->data[4] + ((message->data[5])<<8);
            int na3 = message->data[6] + ((message->data[7])<<8);
            if(y==1)  {  inRegen1 = TRUE;  }
            else { inRegen1 = FALSE; }
            dataRead305=TRUE;
            return true;
            break;
        case 0x183:
            bmsCCL = message->data[0] + ((message->data[1])<<8);
            bmsDCL = message->data[2] + ((message->data[3])<<8);
            bmsHighTemp = message->data[4] + ((message->data[5])<<8);
            bmsPackCurrent = message->data[6] + ((message->data[7])<<8);
            dataRead183 = TRUE;
            return true;
            break;
        default:
            return false;
            break;
    }
}

void cantoLCD ( int parameter, int lcdX, int lcdY )
{
    // Takes in individual parameter that is needed to be displayed at (X,Y) on the LCD
            char paramString[4];
            sprintf( paramString, "%d", parameter);
            LCDsetFontEnlargement(0x02);
            LCDsetFontWriteCursorLoc(lcdX, lcdY);
            LCDsendString(paramString);
}
void cantoLCDFloat( float paramfloat, int lcdX, int lcdY )
{
    // Needed to display floats with 2 decimal places
    // Battery voltage is given in centiVolts and needs to be displayed as a float for precision
            char paramStringF[5];
            sprintf( paramStringF, "%.2f", paramfloat);
            LCDsetFontEnlargement(0x02);
            LCDsetFontWriteCursorLoc(lcdX, lcdY);
            LCDsendString(paramStringF);
}
void sendString(char* string){
    int i = 0;
    while(string[i]){
        while(!UARTTransmitterIsReady(UART1));
        UARTSendDataByte(UART1, string[i]);
        i++;
    }
}

BOOL allMsgsRecieved(){
    int i;
    for(i = 0; i < 4; i++){
        if(canRecent[i].id == 0){
            return FALSE;
        }
    }
    return TRUE;
}

void initMsgRec(){
    int i;
    for(i = 0; i < 4; i++){
        CAN_MSG temp;
        temp.id = 0;
        temp.length = 0;
        canRecent[i] = temp;
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

void sendMsgRecUART1(){

    char uartArray[14];

    uartArray[0] = canRecent[2].data[2];
    uartArray[1] = canRecent[2].data[3];

    uartArray[2] = canRecent[2].data[0];
    uartArray[3] = canRecent[2].data[1];

    uartArray[4] = canRecent[0].data[2];
    uartArray[5] = canRecent[0].data[3];

    uartArray[6] = canRecent[3].data[2];
    uartArray[7] = canRecent[3].data[3];

    uartArray[8] = canRecent[3].data[0];
    uartArray[9] = canRecent[3].data[1];

    uartArray[10] = canRecent[1].data[2];
    uartArray[11] = canRecent[1].data[3];

    uartArray[12] = 0xFF;
    uartArray[13] = 0xFF;

    sendString(uartArray);
}

int main(void)
{
    int value;
    value = SYSTEMConfigWaitStatesAndPB( GetSystemClock() );
    CheKseg0CacheOn(); // Enable cache for USB performance

    DDPCONbits.JTAGEN = 0;
    TRISGSET  = 0xC0 ; // BTN 1 and 2
    TRISGCLR = 0xF000;   // Clear PortG bit associated with LED1
    ODCGCLR  = 0xF000;   // Normal output for LED1 (not open drain)
    LATGCLR = 0xF000;
    // Switch for Data Logging
   // TRISDSET = 0x8000; // Set JE-04 (RD15) as a Digital Input

    // Testing for Fan PMOS Control
    TRISDCLR = 0x0200; // Clear JD-01 (RD9) as Digital Output
    ODCDSET = 0x0200; // Set Output as an open drain output for PMOS control
    LATDCLR = 0x0200; // Clear output before use [Use LATFSET = 0x0100; to enable output]

    LATDSET = 0x200;

    //Setupt input for inteface button JF2 (RF05) (0x20)
    TRISFSET = 0x20;
    //RED LED - JF3 (RF04)  (0x10)
    TRISFCLR = 0x10;
    ODCFCLR = 0x10;
    LATFSET = 0x10;
    //Green LED -JF7 (RE9)  (0x200)
    TRISECLR = 0x200;
    ODCECLR = 0x200;
    LATESET = 0x200;
    //Setupt Input for DataFlag Button - JF8 - RA1
    TRISASET = 0x1;


    // CAN/UART
    CAN1Init(); // Accepts all SIDs, used for grabbing PDOs
    CAN2Init(); // Used for debugging, sends SIDs
    initMsgRec();
    DelayInit();
    initUART1();
    // Timer for Logging Frequency
    // Formula for timer period: PR = (1/loggingFrequency)*80MHz/Pre-scale
    OpenTimer2(T2_ON | T2_IDLE_CON | T2_SOURCE_INT | T2_PS_1_256 | T2_GATE_OFF, 1525);
    // Interrupts
    INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);
    INTEnableSystemMultiVectoredInt();
    INTSetVectorPriority(INT_TIMER_2_VECTOR, INT_PRIORITY_LEVEL_1);
    INTClearFlag(INT_T2);
    INTEnable(INT_T2, INT_ENABLED);
    INTEnableInterrupts();
    setI2CBus(I2C1);
    // USB for Data Logging
    value = OSCCON;
    while (!(value & 0x00000020))
    {
        value = OSCCON;    // Wait for PLL lock to stabilize
    }
    deviceAttached = FALSE;
    USBInitialize(0); // Initialize the stack
    // Variables Init
    prevButton1 = 0;
    prevButton2 = 0;
    shouldLog = FALSE;
    shouldStop = FALSE;
    state = init;
    dataRead187=FALSE;
    dataRead201=FALSE;
    dataRead205=FALSE;
    dataRead287=FALSE;
    dataRead305=FALSE;
    dataRead183=FALSE;
    dataRead100=FALSE;
    // I2C for EEPROM
    initI2CEEPROM();
    logNum = 0;
    short addy = 0x000;
    BYTE num = 0x00;

    LATFCLR = 0x10; //Turn on Red LED

    int b = 0x01;
    int c = 0;
    

    while(1)
    {
        USBTasks();
        switch(state){ // Logging State Machine
            case init:
                if( CheckLogStateChange() == 1 ) {
                    state = startLog;
                }
                else
                    state=wait;
                break;
            case startLog:
                if(USBHostMSDSCSIMediaDetect()) //if thumbdrive is plugged in
                {
                    deviceAttached = TRUE;
                    //now a device is attached
                    //See if the device is attached and in the right format
                    if(FSInit())
                    {
                        //Opening a file in mode "w" will create the file if it doesn't
                        //  exist.  If the file does exist it will delete the old file
                        //  and create a new one that is blank.
                        char nameString[30];
                        logNum = readEEPROM(addy);
                        sprintf(nameString, "vtm17e%d.csv", logNum);
                        myFile = FSfopen(nameString,"w");
                        char string[350];
                        sprintf(string, "MotorSpeed(RPM),NodeStatus,MotorCurrent,DCBusVoltage,TRACT1_CommandWord,MotorCommandCurrent(A-rms),AccelerationCL,DecelerationCL,VMC30State,SteeringFiltered,ThrottleFiltered,ErrorBits,HeatSinkTemp(C),MotorTemp(C),DcBusCurrent,VoltageAngle,Regen,CCL,DCL,HighTemp,PackCurrent,CellID,InstVoltage,OpenCVoltage,CheckSum\n");
                        FSfwrite(string,1, strlen(string),myFile);
                        LATECLR = 0x200; //Turn on Green
                        LATFSET = 0x10; //Turn off Red
                        state = log;
                    }
                }
                break;
            case log:
                if( CheckLogStateChange() == 2 ) {
                    state = stopLog;
                }
                break;
            case stopLog:
                //Always make sure to close the file so that the data gets
                //  written to the drive.
                FSfwrite("endFile", 1, 7, myFile);
                FSfclose(myFile);
                state = wait;
                logNum++;
                writeEEPROM(addy, logNum);
                LATFCLR = 0x10; //Turn on Red
                LATESET = 0x200; //Turn off Green
                break;
            case wait:
                USBTasks();
            //    if(checkForButton1()){
                if( CheckLogStateChange() == 1 ) {
                    state = startLog;
                    if (!USBHostMSDSCSIMediaDetect())
                    {
                    }
                }
                break;
            default:
                state = wait;
                break;
        } // End of Logging State Machine
        //*****************************CAN 2 TX for Debugging
//        //if(PORTG & 0x40){
//            short id = 0x168;
//            if(c % 4 == 0){
//                id = 0x305;
//            }else if(c % 4 == 1){
//                id = 0x183;
//            }else if(c % 4 == 2){
//                id = 0x205;
//            }else if(c % 4 == 3){
//                id = 0x287;
//            }
//            BYTE data[8];
//            data[0] = 0x00;
//            data[1] = 0x00;
//            data[2] = 0x10;
//            data[3] = 0x00;
//            data[4] = b;
//            data[5] = 0x02;
//            data[6] = 0x1A;
//            data[7] = 0x02;
//            int dataLength = 8;
//            CAN2TxSendCustomMsg(id, data, dataLength);
        //}
        //************************************************************
    b++;
        CANRxMessageBuffer *msg = CAN1RxMsgProcess(); // Receives message off CAN bus
        if(msg){
            if(getCANMessages(msg)){

                int msgSeen = 0;
                if(msg->msgSID.SID == 0x205){
                    msgSeen = 0;
                }
                else if(msg->msgSID.SID == 0x187){
                    msgSeen = 1;                    
                }
                else if(msg->msgSID.SID == 0x305){
                    msgSeen = 2;
                }
                else if(msg->msgSID.SID == 0x287){
                    msgSeen = 3;
                }
                else if(msg->msgSID.SID == 0x183){
                    msgSeen = 4;
                }
                CAN_MSG temp;
                temp.id = msg->msgSID.SID;
                temp.length = 8;
                int count;
                for(count = 0; count < temp.length; count++){
                    temp.data[count] = msg->data[count];
                }
                canRecent[msgSeen] = temp;
                if(allMsgsRecieved()){
                    sendMsgRecUART1();
                    initMsgRec();
                }
                } // End of if(getCANMessages() )
            } // End of if(msg)
    } // End of while(1)
 return 0;
} // End of main()

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
    // Notes:
    // - Need \n after last parameter written to USB

    if (INTGetFlag(INT_T2))
    {
        if (state == log)
        {
            char dataRead187String[50];
            char dataRead201String[50];
            char dataRead205String[50];
            char dataRead287String[50];
            char dataRead305String[50];
            char dataRead183String[50];
            char dataRead100String[50];
            if(dataRead187){
                sprintf(dataRead187String, "%d,%d,%d,%.2f,", motorSpeed, nodeStatus, rmsMotorCurrent, nodeDCVoltage );
                dataRead187 = FALSE;
            }else{
                sprintf(dataRead187String, " , , , ,");
            }
            if(dataRead201){
                sprintf(dataRead201String, "%d,%d,%d,%d,", commandWord, commandCurrent, accelCurrentLimit, decelCurrentLimit );
                dataRead201 = FALSE;
            }else{
                sprintf(dataRead201String, " , , , ,");
            }
            if(dataRead205){
                sprintf(dataRead205String, "%d,%d,%d,%d,", vmc30State, steeringFiltered, throttleFiltered, errorBits );
                dataRead205 = FALSE;
            }else{
                sprintf(dataRead205String, " , , , ,");
            }
            if(dataRead287){
                sprintf(dataRead287String, "%d,%d,%d,%d,", heatSinkTemp, motorTemp, dcCurrent, voltageAngle );
                dataRead287 = FALSE;
            }else{
                sprintf(dataRead287String, " , , , ,");
            }
            if(dataRead305){
                if( inRegen1 ) { sprintf(dataRead305String, "1,"); }
                else { sprintf(dataRead305String, "0,"); }
                dataRead305 = FALSE;
            }
            else{
                sprintf(dataRead305String, " ,");
            }
            if(dataRead183){
                sprintf(dataRead183String, "%d,%d,%d,%d,", bmsCCL, bmsDCL, bmsHighTemp, bmsPackCurrent );
                dataRead183 = FALSE;
            }else{
                sprintf(dataRead183String, " , , , ,");
            }
            if(dataRead100){
                sprintf(dataRead100String, "%d,%d,%d,%d\n", cellid, instVoltage, openVoltage, checkSum );
                dataRead100 = FALSE;
            }else{
                sprintf(dataRead100String, " , , , \n");
            }
            FSfwrite(dataRead187String,1, strlen(dataRead187String),myFile);
            FSfwrite(dataRead201String,1, strlen(dataRead201String),myFile);
            FSfwrite(dataRead205String,1, strlen(dataRead205String),myFile);
            FSfwrite(dataRead287String,1, strlen(dataRead287String),myFile);
            FSfwrite(dataRead305String,1, strlen(dataRead305String),myFile);
            FSfwrite(dataRead183String,1, strlen(dataRead183String),myFile);
            FSfwrite(dataRead100String,1, strlen(dataRead100String),myFile);
        }
    INTClearFlag(INT_T2);
    }
}
//adding the following stopped the error
//undefined reference to `_p32mxsk_putc
//that was caused by using sprintf
void _mon_putc(char c) {}