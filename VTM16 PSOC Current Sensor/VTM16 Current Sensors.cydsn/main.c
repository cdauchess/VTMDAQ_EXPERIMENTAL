/******************************************************************************
* Project Name		: VTM16 Current Sensors
* File Name			: main.c
* Version 			: **
* Device Used		: CY8C5888LTI-LP097
* Software Used		: PSoC Creator 3.3 SP2
* Compiler    		: ARM GCC 4.8.4, ARM RVDS Generic, ARM MDK Generic
* Related Hardware	: CY8CKIT059 PSoC 5 LP Prototyping Kit 
*******************************************************************************/
//This started as the UART-ADC sample program for the PSOC 5lP.  Much of the code from the example can still be seen in here.

#include <device.h>
#include "stdio.h"

/* Project Defines */
#define FALSE  0
#define TRUE   1
#define TRANSMIT_BUFFER_SIZE  16 //UART 
/*I2C*/
#define I2C_BUFFER_SIZE 0x0C
#define I2C_RW_AREA_SIZE 0x01 //needs to be 0x01 to have a place for the offset data pointer?


//Need to use ezi2c_get_activity to develop a mechanism for data coherency
//Only write to buffer if it is not busy
void main()
{
    /* Variable to store ADC result */
    int16 Output;
    int32 data[6];
    uint8 lowByte[6];
    uint8 highByte[6];
    /* Variable to store UART received character */
    uint8 Ch;
    /* Variable used to send emulated data */
    uint8 EmulatedData;
    /* Flags used to store transmit data commands */
    uint8 ContinuouslySendData;
    uint8 SendSingleByte;
    uint8 SendEmulatedData;
    //Set Up i2c variables
    CYGlobalIntEnable; //Enable global interrupts for the I2C module
    uint8 i2cBuffer[I2C_BUFFER_SIZE];
    /* Transmit Buffer */
    char TransmitBuffer[TRANSMIT_BUFFER_SIZE];//UART
    
    
    int i;
    /* Start the components */
    ADC_SAR_Seq_1_Start();
    UART_1_Start();
    EZI2C_1_Start();
    EZI2C_1_SetBuffer1(I2C_BUFFER_SIZE, I2C_RW_AREA_SIZE, i2cBuffer); 
    
    /* Initialize Variables */
    ContinuouslySendData = FALSE;
    SendSingleByte = FALSE;
    SendEmulatedData = FALSE;
    EmulatedData = 0;
    
    /* Start the ADC conversion */
    ADC_SAR_Seq_1_StartConvert();
    
    /* Send message to verify COM port is connected properly */
    UART_1_PutString("COM Port Open");
    
    for(;;)
    {        
        /* Non-blocking call to get the latest data recieved  */
        Ch = UART_1_GetChar();
        
        /* Set flags based on UART command */
        switch(Ch)
        {
            case 0:
                /* No new data was recieved */
                break;
            case 'C':
            case 'c':
                SendSingleByte = TRUE;
                break;
            case 'S':
            case 's':
                ContinuouslySendData = TRUE;
                break;
            case 'X':
            case 'x':
                ContinuouslySendData = FALSE;
                break;
            case 'E':
            case 'e':
                SendEmulatedData = TRUE;
                break;
            default:
                /* Place error handling code here */
                break;    
        }
        
        /* Check to see if an ADC conversion has completed */
        //At some point should switch to interrupts
        if(ADC_SAR_Seq_1_IsEndConversion(ADC_SAR_Seq_1_RETURN_STATUS))
        {
            i = 0;
            for(i = 0;i<6;i++)
            {
            Output = ADC_SAR_Seq_1_GetResult16(i);
            data[i] = ADC_SAR_Seq_1_CountsTo_mVolts(Output);
            //Split the data so that each data point is two bytes so that it can be easily loaded into the transmit buffer            
            lowByte[i] = (data[i] & 0xFF);
            highByte[i] = (0xFF & (data[i] >> 8));            
            }

 
            
            if((EZI2C_1_GetActivity() & EZI2C_1_STATUS_BUSY)== 0) //Use this to not interfere with cerebot while writing to buffer
            {
            //set data into i2c transmit buffer
            //Results in 12 bytes in the transmit buffer
                for(i = 0;i<6;i++)
                {
                i2cBuffer[2*i] = lowByte[i];  //First byte
                i2cBuffer[2*i+1] = highByte[i]; //Second Byte
                }
            }
            /* Send data based on last UART command */
            if(SendSingleByte || ContinuouslySendData)
            {
                /* Format ADC result for transmition */
                sprintf(TransmitBuffer, "Overall: %ld mV\r\n", data[0]);
                /* Send out the data */
                UART_1_PutString(TransmitBuffer);
                
                sprintf(TransmitBuffer, "Production: %ld mV\r\n", data[1]);
                UART_1_PutString(TransmitBuffer);      
                sprintf(TransmitBuffer, "Fpump: %ld mV\r\n", data[2]);
                UART_1_PutString(TransmitBuffer);                  
                sprintf(TransmitBuffer, "Finj: %ld mV\r\n", data[3]);
                UART_1_PutString(TransmitBuffer);                 
                sprintf(TransmitBuffer, "Ignition: %ld mV\r\n", data[4]);
                UART_1_PutString(TransmitBuffer);                   
                sprintf(TransmitBuffer, "RefV: %ld mV\r\n\n\n\n\n\n\r", data[5]);
                UART_1_PutString(TransmitBuffer);                   
                /* Reset the send once flag */
                SendSingleByte = FALSE;
            }
            else if(SendEmulatedData)
            {
                /* Format ADC result for transmition */
                sprintf(TransmitBuffer, "Emulated Data: %x \r\n", EmulatedData);
                /* Send out the data */
                UART_1_PutString(TransmitBuffer);
                EmulatedData++;
                /* Reset the send once flag */
                SendEmulatedData = FALSE;   
            }
        }
    }
}


/* [] END OF FILE */
