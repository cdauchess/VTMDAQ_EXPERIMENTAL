//This source file defines functions for LCD when using i2c as interphace
//LCD refers to RA8875
//Stephen Kralick
//Last edit: 2-15-2105
#include <plib.h>
#include "i2cLCD.h"

void setI2CBus(I2C_MODULE bus){
    i2cBus = bus;
}

I2C_MODULE getI2CBus(){
    return i2cBus;
}

void I2CInitForLCD(unsigned int pbusClock, unsigned int i2cFrq){
    //could call I2CConfigure but its not necissary because we dont need high speed and stretching and stuff
    unsigned int actualClock = I2CSetFrequency(i2cBus, pbusClock, i2cFrq);
    if ( abs(actualClock-i2cFrq) > i2cFrq/10 )
    {
        sendString("Error: I2C1 clock frequency (%u) error exceeds 10%%.\n", (unsigned)actualClock);
    }
    I2CEnable(i2cBus, TRUE);
}

BOOL StartTransfer( BOOL restart )
{
    I2C_STATUS  status;

    // Send the Start (or Restart) signal
    if(restart)
    {
        I2CRepeatStart(i2cBus);
    }
    else
    {
        // Wait for the bus to be idle, then start the transfer
        while( !I2CBusIsIdle(i2cBus) );

        if(I2CStart(i2cBus) != I2C_SUCCESS)
        {
            sendString("Error: Bus collision during transfer Start\n");
            return FALSE;
        }
    }

    // Wait for the signal to complete
    do
    {
        status = I2CGetStatus(i2cBus);

    } while ( !(status & I2C_START) );

    return TRUE;
}

BOOL TransmitOneByte( UINT8 data )
{
    // Wait for the transmitter to be ready
    while(!I2CTransmitterIsReady(i2cBus));

    // Transmit the byte
    if(I2CSendByte(i2cBus, data) == I2C_MASTER_BUS_COLLISION)
    {
        sendString("Error: I2C Master Bus Collision\n");
        return FALSE;
    }

    // Wait for the transmission to finish
    while(!I2CTransmissionHasCompleted(i2cBus));

    return TRUE;
}

void StopTransfer( void )
{
    I2C_STATUS  status;

    // Send the Stop signal
    I2CStop(i2cBus);

    // Wait for the signal to complete
    do
    {
        status = I2CGetStatus(i2cBus);

    } while ( !(status & I2C_STOP) );
}

void cmdWriteCycle(BYTE address){
    BOOL Success = TRUE;
    //for a data read operation
    //command write 0x02
    //      drop cs low, rs = 1, WR# = 0, RD# = 0

    BYTE txData[2];
    txData[0] = (LCD_DEVICE_ID << 2) | 0x02 ;
    txData[1] = address;
    int tdDLen = 2;

    // Start the transfer to lcd
    if( !StartTransfer(FALSE) )
    {
        while(1);
    }

    // Transmit all data
    int Index = 0;
    while( Success && (Index < tdDLen) )
    {
        // Transmit a byte
        if (TransmitOneByte(txData[Index]))
        {
            // Advance to the next byte
            Index++;

            // Verify that the byte was acknowledged
            if(!I2CByteWasAcknowledged(i2cBus))
            {
                sendString("Error: Sent byte was not acknowledged\n");
                Success = FALSE;
            }
        }
        else
        {
            Success = FALSE;
        }
    }
    // End the transfer (hang here if an error occured)
    StopTransfer();
    if(!Success)
    {
        while(1);
    }//the CMD Write of 0x02 should be complete
}

void dataWriteCycle(BYTE value){
    BOOL Success = TRUE;
    //for a data read operation
    //command write 0x02
    //      drop cs low, rs = 1, WR# = 0, RD# = 0

    BYTE txData[2];
    txData[0] = (LCD_DEVICE_ID << 2) | 0x00 ;
    txData[1] = value;
    int tdDLen = 2;

    // Start the transfer to lcd
    if( !StartTransfer(FALSE) )
    {
        while(1);
    }

    // Transmit all data
    int Index = 0;
    while( Success && (Index < tdDLen) )
    {
        // Transmit a byte
        if (TransmitOneByte(txData[Index]))
        {
            // Advance to the next byte
            Index++;

            // Verify that the byte was acknowledged
            if(!I2CByteWasAcknowledged(i2cBus))
            {
                sendString("Error: Sent byte was not acknowledged\n");
                Success = FALSE;
            }
        }
        else
        {
            Success = FALSE;
        }
    }
    // End the transfer (hang here if an error occured)
    StopTransfer();
    if(!Success)
    {
        while(1);
    }//the CMD Write of 0x02 should be complete
}

BYTE dataReadCycle( void ){
    BOOL Success = TRUE;
    BYTE txMsg = ((LCD_DEVICE_ID << 2) | 0x01);
    int tdDLen = 1;
    // Start the transfer to lcd
    if( !StartTransfer(FALSE) )
    {
        while(1);
    }
    int Index = 0;
    while( Success && (Index < tdDLen) )
    {
        // Transmit a byte
        if (TransmitOneByte(txMsg))
        {
            // Advance to the next byte
            Index++;

            // Verify that the byte was acknowledged
            if(!I2CByteWasAcknowledged(i2cBus))
            {
                sendString("Error: Sent byte was not acknowledged\n");
                Success = FALSE;
            }
        }
        else
        {
            Success = FALSE;
        }
    }
    I2CReceiverEnable(i2cBus, TRUE);
    //here we read a byte send an ack
    while(!I2CReceivedDataIsAvailable);
    I2CAcknowledgeByte(i2cBus, TRUE);
    while(!I2CAcknowledgeHasCompleted(i2cBus)) ;

    BYTE result = I2CGetByte(i2cBus);
    
    DelayMs(20);//i think the correct way to do this is to not acknowlege the recieved byte
                //then the LCD will shutup and the master can send a stop transfer
    
    // End the transfer (hang here if an error occured)
    StopTransfer();
    if(!Success)
    {
        while(1);
    }

    return result;
}
//might require two calls to get the real data
BYTE statusRead( void ){
    int temp = 0;
    BYTE result;
    for(temp = 0; temp < 2; temp++){
        BOOL Success = TRUE;
        BYTE txMsg = ((LCD_DEVICE_ID << 2) | 0x03);
        int tdDLen = 1;
        // Start the transfer to lcd
        if( !StartTransfer(FALSE) )
        {
            while(1);
        }
        int Index = 0;
        while( Success && (Index < tdDLen) )
        {
            // Transmit a byte
            if (TransmitOneByte(txMsg))
            {
                // Advance to the next byte
                Index++;

                // Verify that the byte was acknowledged
                if(!I2CByteWasAcknowledged(i2cBus))
                {
                    sendString("Error: Sent byte was not acknowledged\n");
                    Success = FALSE;
                }
            }
            else
            {
                Success = FALSE;
            }
        }
        I2CReceiverEnable(i2cBus, TRUE);
        //here we read a byte send an ack
        while(!I2CReceivedDataIsAvailable);
        I2CAcknowledgeByte(i2cBus, TRUE);
        while(!I2CAcknowledgeHasCompleted(i2cBus)) ;

        result = I2CGetByte(i2cBus);

        DelayMs(20);//i think the correct way to do this is to not acknowlege the recieved byte
                    //then the LCD will shutup and the master can send a stop transfer

        // End the transfer (hang here if an error occured)
        StopTransfer();
        if(!Success)
        {
            while(1);
        }
    }
    return result;
}

BYTE readOperation(BYTE address){

    cmdWriteCycle(address);

    dataReadCycle();//read junk data

    return dataReadCycle();//read real data

}
//i should make a sucessive write that will take an array of btes and a start address
// and send one command write cycle and then several datawrite cycles and the LCD
// will automatically increment the register you are writing too
void writeOperation(BYTE address, BYTE value){

    cmdWriteCycle(address);

    dataWriteCycle(value);

}

void LCDsendString(char* string){
    int i = 0;
    while(string[i]){
        writeOperation(RA8875_MRWC, string[i]);
        DelayMs(10);
        i++;
    }
}

//something is wacky here when we do anything other than 0 for x and y
//y doesnt change and x jumps way off to the right
void LCDsetFontWriteCursorLoc(int x, int y){
    writeOperation(RA8875_FCURXL, x & 0xFF);
    writeOperation(RA8875_FCURXH, x >> 8);
    writeOperation(RA8875_FCURYL, y & 0xFF);
    writeOperation(RA8875_FCURYH, y >> 8);
}

void LCDsetBackLight(BOOL enable, BYTE brightness){
    BYTE temp = readOperation(RA8875_P1CR);
    writeOperation(RA8875_P1CR, temp  |  (enable << 7)); //clear MSB if enable
    writeOperation(RA8875_P1DCR, brightness);
}

void LCDsetBackGroundColor(BYTE red, BYTE green, BYTE blue){
    writeOperation(RA8875_BGCR0, red);
    writeOperation(RA8875_BGCR1, green);
    writeOperation(RA8875_BGCR2, blue);
}

void LCDsetForeGroundColor(BYTE red, BYTE green, BYTE blue){
    writeOperation(RA8875_FGCR0, red);
    writeOperation(RA8875_FGCR1, green);
    writeOperation(RA8875_FGCR2, blue);
}

void LCDsetActiveWindow(int startx, int endx, int starty, int endy){
    //set active window x
    writeOperation(RA8875_HSAW0, (UINT16)(startx - 1) & 0xFF);
    writeOperation(RA8875_HSAW1, (UINT16)(startx - 1) >> 8);
    writeOperation(RA8875_HEAW0, (UINT16)(endx - 1) & 0xFF);
    writeOperation(RA8875_HEAW1, (UINT16)(endx - 1) >> 8);

    //set active window y
    writeOperation(RA8875_VSAW0, (UINT16)(starty - 1) & 0xFF);
    writeOperation(RA8875_VSAW1, (UINT16)(starty - 1) >> 8);
    writeOperation(RA8875_VEAW0, (UINT16)(endy - 1) & 0xFF);
    writeOperation(RA8875_VEAW1, (UINT16)(endy - 1) >> 8);
}

//size is between 0 thru 3
void LCDsetFontEnlargement(BYTE size){
    writeOperation( 0x22, size | (size << 2));
}

void LCDsetUpTextMode(){

    writeOperation( 0x40, 0xC0);  //enable text mode //this might have to be fancy so we dont overwrite other bits
    BYTE temp = readOperation(0x21);
    temp = temp & ~0xA0;
    writeOperation( 0x21, temp); //we wanna clear bits 5 and 7 of this so we could get fancy
    //writeOperation( 0x22, 0x0F);
    LCDsetFontEnlargement(0x3);
}
//this isnt working i dont know why
void drawSquare(BYTE startX, BYTE startY, BYTE endX, BYTE endY, BYTE red, BYTE green, BYTE blue){
    writeOperation(RA8875_DLHSR0, startX & 0xFF);
    writeOperation(RA8875_DLHSR1, startX >> 8);
    writeOperation(RA8875_DLVSR0, startY & 0xFF);
    writeOperation(RA8875_DLVSR1, startY >> 8);

    writeOperation(RA8875_DLHER0, endX & 0xFF);
    writeOperation(RA8875_DLHER1, endX >> 8);
    writeOperation(RA8875_DLVER0, endY & 0xFF);
    writeOperation(RA8875_DLVER1, endY >> 8);

    writeOperation(RA8875_FGCR0, red);
    writeOperation(RA8875_FGCR1, green);
    writeOperation(RA8875_FGCR2, blue);
    
    //set draw a square
    BYTE temp1 = readOperation(0x90);
    temp1  = temp1 | 0x10;//set bit 4
    temp1 = temp1 & ~0x01;//clr bit 0
    writeOperation(0x90, temp1);
    //set fill to true
    BYTE temp2 = readOperation(0x90);
    temp2  = temp2 | 0x20;//set bit 5
    writeOperation(0x90, temp2);

    //start drawing
    BYTE temp3 = readOperation(0x90);
    temp3  = temp3 | 0x80;//set bit 7
    writeOperation(0x90, temp3);

    BYTE temp4 = readOperation(0x90);
    while(temp4 & 0x80){
        temp4 = readOperation(0x90);
    }
}

//this functinois just a copy paste and is in no way correct
void drawCircle(int centerX, int centerY, int radius, BYTE red, BYTE green, BYTE blue){
    /*writeOperation(RA8875_DLHSR0, startX & 0xFF);
    writeOperation(RA8875_DLHSR1, startX >> 8);
    writeOperation(RA8875_DLVSR0, startY & 0xFF);
    writeOperation(RA8875_DLVSR1, startY >> 8);

    writeOperation(RA8875_DLHER0, endX & 0xFF);
    writeOperation(RA8875_DLHER1, endX >> 8);
    writeOperation(RA8875_DLVER0, endY & 0xFF);
    writeOperation(RA8875_DLVER1, endY >> 8);

    writeOperation(RA8875_FGCR0, red);
    writeOperation(RA8875_FGCR1, green);
    writeOperation(RA8875_FGCR2, blue);
     * */

    //set draw a square
    BYTE temp1 = readOperation(0x90);
    temp1  = temp1 | 0x10;//set bit 4
    temp1 = temp1 & ~0x01;//clr bit 0
    writeOperation(0x90, temp1);
    //set fill to true
    BYTE temp2 = readOperation(0x90);
    temp2  = temp2 | 0x20;//set bit 5
    writeOperation(0x90, temp2);

    //start drawing
    BYTE temp3 = readOperation(0x90);
    temp3  = temp3 | 0x80;//set bit 7
    writeOperation(0x90, temp3);
}

void initializeLCD(){
    BYTE address[40];
    BYTE message[40];
    //PLL init
    address[0] = 0x88;//pll input param
    message[0] = 0x0A;
    address[1] = 0x89;//pll divider
    message[1] = 0x02;

    //other initializations
    address[2] = 0x10;//color
    message[2] = 0x0C;//16bpp
    address[3] = 0x04;//pixel clock
    message[3] = 0x80 | 0x01;

    unsigned int pixclk          = RA8875_PCSR_PDATL | RA8875_PCSR_2CLK;
    unsigned int hsync_nondisp   = 26;
    unsigned int hsync_start     = 32;
    unsigned int hsync_pw        = 96;
    unsigned int hsync_finetune  = 0;
    unsigned int vsync_nondisp   = 32;
    unsigned int vsync_start     = 23;
    unsigned int vsync_pw        = 2;

    //width setting
    address[4] = RA8875_HDWR; //width
    message[4] = 0x63;  //width/8 -1
    address[5] = RA8875_HNDFTR;
    message[5] = RA8875_HNDFTR_DE_HIGH + hsync_finetune;
    address[6] = RA8875_HNDR;
    message[6] = (hsync_nondisp - hsync_finetune - 2)/8;
    address[7] = RA8875_HSTR;
    message[7] = hsync_start/8 - 1;
    address[8] = RA8875_HPWR;
    message[8] = RA8875_HPWR_LOW + (hsync_pw/8 - 1);

    //vertical settings
    address[9] = RA8875_VDHR0;
    message[9] = (UINT16)(480 - 1) & 0xFF;
    address[10] = RA8875_VDHR1;
    message[10] = (UINT16)(480 - 1) >> 8;
    address[11] = RA8875_VNDR0;
    message[11] = vsync_nondisp-1;
    address[12] = RA8875_VNDR1;
    message[12] = vsync_nondisp >> 8;
    address[13] = RA8875_VSTR0;
    message[13] = vsync_start-1;
    address[14] = RA8875_VSTR1;
    message[14] = vsync_start >> 8;
    address[15] = RA8875_VPWR;
    message[15] = RA8875_VPWR_LOW + vsync_pw - 1;

    //set active window x
    address[16] = 0;//RA8875_HSAW0;
    message[16] = 0;//0;
    address[17] = 0;//RA8875_HSAW1;
    message[17] = 0;//0;
    address[18] = 0;//RA8875_HEAW0;
    message[18] = 0;//(UINT16)(800 - 1) & 0xFF;
    address[19] = 0;//RA8875_HEAW1;
    message[19] = 0;//(UINT16)(800 - 1) >> 8;

    //set active window y
    address[20] = 0;//RA8875_VSAW0;
    message[20] = 0;//0;
    address[21] = 0;//RA8875_VSAW1;
    message[21] = 0;//0;
    address[22] = 0;//RA8875_VEAW0;
    message[22] = 0;//(UINT16)(480 - 1) & 0xFF;
    address[23] = 0;//RA8875_VEAW1;
    message[23] = 0;//(UINT16)(480 - 1) >> 8;

    //clear the screen
    address[24] = RA8875_MCLR;
    message[24] = RA8875_MCLR_START | RA8875_MCLR_FULL;

    //display on
    address[25] = RA8875_PWRR;
    message[25] = RA8875_PWRR_NORMAL | RA8875_PWRR_DISPON;

    //use PWM to turn backlight on (my way not arduino example)
    address[26] = 0;//0x8A;
    message[26] = 0;//0x80;
    address[27] = 0;//0x8B;
    message[27] = 0;//0xFF;

    //setup text mode
    address[28] = 0;//0x40;  //enable text mode
    message[28] = 0;//0xC0;  //this might have to be fancy so we dont overwrite other bits
    address[29] = 0;//0x21;
    message[29] = 0;//0x00; //we wanna clear bits 5 and 7 of this so we could get fancy

    //set cursor 10 10
    address[30] = 0;//0x2A;
    message[30] = 0;//10 & 0xFF;
    address[31] = 0;//0x2B;
    message[31] = 0;//10 >> 8;
    address[32] = 0;//0x2C;
    message[32] = 0;//10 & 0xFF;
    address[33] = 0;//0x2D;
    message[33] = 0;//10 >> 8;

    //we could enlarge here but we dont wanna yet

    //lets write some text
    address[34] = 0;//0x2E;
    message[34] = 0;//0x05;//font spaceing width is 5 px
    address[35] = 0;//0x2F;
    message[35] = 0;//0x00;//random font setup
    address[36] = 0;//0x22;
    message[36] = 0;//0x0F;//set full font x and y enlargement

    int arrayLength = 26;
    int i = 0;
    for(i = 4; i < arrayLength; i++){
        writeOperation(address[i], message[i]);
        DelayMs(100);
    }
}
