/*
 * VTM16e Dash LCD Code
 * Steven Kralick
 * Modified by Danny
 * 3/01/2016
 */
#define _PLIB_DISABLE_LEGACY

// Digilent board configuration
#pragma config ICESEL       = ICS_PGx1  // ICE/ICD Comm Channel Select
#pragma config DEBUG        = OFF       // Debugger Disabled for Starter Kit
#pragma config FNOSC        = PRIPLL	// Oscillator selection
#pragma config POSCMOD      = XT	// Primary oscillator mode
#pragma config FPLLIDIV     = DIV_2	// PLL input divider
#pragma config FPLLMUL      = MUL_20	// PLL multiplier
#pragma config FPLLODIV     = DIV_1	// PLL output divider
#pragma config FPBDIV       = DIV_1	// Peripheral bus clock divider
#pragma config FSOSCEN      = OFF	// Secondary oscillator enable
#pragma config FCANIO = OFF //required for CAN see mx7ck rm

#include <plib.h>
#include "PmodOLED.h"
#include "OledChar.h"
#include "OledGrph.h"
#include "delay.h"
#include "i2c.h"
#include "i2cLCD.h"
#include "CANFunctions.h"

#define getPBusClock() 80000000
#define UART_BAUD 9600

BYTE CAN1MessageFifoArea[2 * 8 * 16];
BYTE CAN2MessageFifoArea[2 * 8 * 16];

// Node ID 0x187 Variables (from Inverter)
int motorSpeed, nodeStatus, nodeDCVoltage, rmsMotorCurrent;
// Node ID 0x287 Varaibles (also from Inverter)
int heatSinkTemp, motorTemp, dcCurrent, voltageAngle;
// Node ID 0x205 Variables (from VMC30)
int systemState, steeringFiltered, throttleFiltered, errorBits;
// Node ID 0x201 Variables (from VMC30)
int commandWord, commandCurrent, accelCurrentLimit, decelCurrentLimit;


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

void messageToString(CANRxMessageBuffer * message, char* result){
    int id = message->msgSID.SID;

    int var1, var2, var3, var4;
    var1 = message->data[0] + ((message->data[1])<<8);
    var2 = message->data[2] + ((message->data[3])<<8);
    var3 = message->data[4] + ((message->data[5])<<8);
    var4 = message->data[6] + ((message->data[7])<<8);
    switch(id){
        case 0x186:
        case 0x187:
            sprintf(result, "COBID: %x - Heat Sink Temp: %d - Motor Temp: %d - DC Current: %d - Voltage Angle: %d", id, var1, var2, var3, var4);
            break;
        case 0x286:
        case 0x287:
            sprintf(result, "COBID: %x - Speed: %d - Status: %d - DC Bus Voltage: %d - RMS Motor Current: %d", id, var1, var2, var3, var4);
            break;

        default:
            sprintf(result, "Message Not Found");
            break;
    }

}
void getCANMessages(CANRxMessageBuffer * message){
    // Gets messages on CAN bus
    // Assigns values based on SID to global variables also used for DAQ
    // Converts values to readable format where needed
    int id = message->msgSID.SID;

    switch(id){
        case 0x187:
            motorSpeed = message->data[0] + ((message->data[1])<<8);
            nodeStatus = message->data[2] + ((message->data[3])<<8);
            nodeDCVoltage = message->data[4] + ((message->data[5])<<8);
            rmsMotorCurrent = message->data[6] + ((message->data[7])<<8);
            break;
        case 0x287:
            heatSinkTemp = message->data[0] + ((message->data[1])<<8);
            motorTemp = message->data[2] + ((message->data[3])<<8);
            dcCurrent = message->data[4] + ((message->data[5])<<8);
            voltageAngle = message->data[6] + ((message->data[7])<<8);
            break;
        case 0x205:
            systemState = message->data[0] + ((message->data[1])<<8);
            steeringFiltered = message->data[2] + ((message->data[3])<<8);
            throttleFiltered = message->data[4] + ((message->data[5])<<8);
            errorBits = message->data[6] + ((message->data[7])<<8);
            break;
        case 0x201:
            commandWord = message->data[0] + ((message->data[1])<<8);
            commandCurrent = message->data[2] + ((message->data[3])<<8);
            accelCurrentLimit = message->data[4] + ((message->data[5])<<8);
            decelCurrentLimit = message->data[6] + ((message->data[7])<<8);
            break;
        default:
            motorSpeed = 88;
            nodeStatus = 88;
            nodeDCVoltage = 88;
            rmsMotorCurrent = 88;
            heatSinkTemp = 88;
            motorTemp = 88;
            dcCurrent = 88;
            voltageAngle = 88;
            systemState = 88;
            steeringFiltered = 88;
            throttleFiltered = 88;
            errorBits = 88;
            commandWord = 88;
            commandCurrent = 88;
            accelCurrentLimit = 88;
            decelCurrentLimit = 88;
            break;
    }

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

int main()
{
    // Cerebot Init
    INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);
    INTEnableInterrupts();
    TRISGSET  = 0xC0 ;//init buttons1 and 2
    initUART1();
    DelayInit();
    setI2CBus(I2C1);

    // LCD Init
    I2CInitForLCD(getPBusClock(), 100000);//later try this at 400,000 as the desired speed // stephen you changed this for new higher pbus
    initializeLCD();

    LCDsetBackLight(TRUE, 0xFF);
    LCDsetActiveWindow(0,800, 0, 480);
    LCDsetBackGroundColor(0x0F, 0x0F, 0x0F);
    LCDsetForeGroundColor(0x00, 0x00, 0x00);
    LCDsetUpTextMode();

    LCDsetFontEnlargement(0x03);
    LCDsetFontWriteCursorLoc(180, 130);
    LCDsendString("VT Motorsports");
    LCDsetFontWriteCursorLoc(300, 210);
    LCDsendString("VTM16e");
    DelayMs(1500);
    initializeLCD();
    int fromLeft = 150;
    LCDsetFontEnlargement(0x02);
    LCDsetFontWriteCursorLoc(fromLeft, 10);
    LCDsendString("MPH");
    LCDsetFontWriteCursorLoc(fromLeft, 75);
    LCDsendString("Volts");
    LCDsetFontWriteCursorLoc(fromLeft, 140);
    LCDsendString("RMS Current");
    LCDsetFontWriteCursorLoc(fromLeft, 205);
    LCDsendString("Motor Temp (C)");
    LCDsetFontWriteCursorLoc(fromLeft, 270);
    LCDsendString("VMC30 State");

    // CAN Init
    CAN1Init();
    CAN2Init();
    initMsgRec();

    int c = 0;
    while(1)
    {
//        if(PORTG & 0x40){
//            short id = 0x168;
//            if(c % 4 == 0){
//                id = 0x186;
//            }else if(c % 4 == 1){
//                id = 0x187;
//            }else if(c % 4 == 2){
//                id = 0x286;
//            }else if(c % 4 == 3){
//                id = 0x287;
//            }
//            BYTE data[8];
//            data[0] = 0x34;
//            data[1] = 0x12;
//            data[2] = 0x78;
//            data[3] = 0x56;
//            data[4] = 0x10;
//            data[5] = 0x89;
//            data[6] = 0x12;
//            data[7] = 0x11;
//            int dataLength = 8;
//            CAN2TxSendCustomMsg(id, data, dataLength);
//        }
        CANRxMessageBuffer *msg = CAN1RxMsgProcess();
        if(msg){
            char debugMsg[50];
            //messageToString(msg, debugMsg); // this works and prints strings to be sent to LCD
            getCANMessages(msg); // this is untested and assigns CAN bytes to global varibales to be sent to LCD

            if(debugMsg[0] == 'C'){
                int msgSeen = 0;
                if(msg->msgSID.SID == 0x186){
                    msgSeen = 0;
                    LCDsetFontWriteCursorLoc(0, 80);
                }else if(msg->msgSID.SID == 0x187){
                    msgSeen = 1;
                    LCDsetFontWriteCursorLoc(0, 160);
                }else if(msg->msgSID.SID == 0x286){
                    msgSeen = 2;
                    LCDsetFontWriteCursorLoc(0, 240);
                }else if(msg->msgSID.SID == 0x287){
                    msgSeen = 3;
                    LCDsetFontWriteCursorLoc(0, 320);
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
                LCDsetFontEnlargement(0x00);
                LCDsendString(debugMsg);
            }
        }
        LCDsetFontEnlargement(0x03);
        LCDsetFontWriteCursorLoc(180, 380);
        char str[20];
        c++;
        sprintf( str, "Speed %d MPH", c);
        LCDsendString(str);
    }
 return 0;
}