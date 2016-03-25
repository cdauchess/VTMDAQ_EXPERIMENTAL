/*******************************************************************************
* File Name: Front_Right.c  
* Version 2.10
*
* Description:
*  This file contains API to enable firmware control of a Pins component.
*
* Note:
*
********************************************************************************
* Copyright 2008-2014, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#include "cytypes.h"
#include "Front_Right.h"

/* APIs are not generated for P15[7:6] on PSoC 5 */
#if !(CY_PSOC5A &&\
	 Front_Right__PORT == 15 && ((Front_Right__MASK & 0xC0) != 0))


/*******************************************************************************
* Function Name: Front_Right_Write
********************************************************************************
*
* Summary:
*  Assign a new value to the digital port's data output register.  
*
* Parameters:  
*  prtValue:  The value to be assigned to the Digital Port. 
*
* Return: 
*  None
*  
*******************************************************************************/
void Front_Right_Write(uint8 value) 
{
    uint8 staticBits = (Front_Right_DR & (uint8)(~Front_Right_MASK));
    Front_Right_DR = staticBits | ((uint8)(value << Front_Right_SHIFT) & Front_Right_MASK);
}


/*******************************************************************************
* Function Name: Front_Right_SetDriveMode
********************************************************************************
*
* Summary:
*  Change the drive mode on the pins of the port.
* 
* Parameters:  
*  mode:  Change the pins to one of the following drive modes.
*
*  Front_Right_DM_STRONG     Strong Drive 
*  Front_Right_DM_OD_HI      Open Drain, Drives High 
*  Front_Right_DM_OD_LO      Open Drain, Drives Low 
*  Front_Right_DM_RES_UP     Resistive Pull Up 
*  Front_Right_DM_RES_DWN    Resistive Pull Down 
*  Front_Right_DM_RES_UPDWN  Resistive Pull Up/Down 
*  Front_Right_DM_DIG_HIZ    High Impedance Digital 
*  Front_Right_DM_ALG_HIZ    High Impedance Analog 
*
* Return: 
*  None
*
*******************************************************************************/
void Front_Right_SetDriveMode(uint8 mode) 
{
	CyPins_SetPinDriveMode(Front_Right_0, mode);
}


/*******************************************************************************
* Function Name: Front_Right_Read
********************************************************************************
*
* Summary:
*  Read the current value on the pins of the Digital Port in right justified 
*  form.
*
* Parameters:  
*  None
*
* Return: 
*  Returns the current value of the Digital Port as a right justified number
*  
* Note:
*  Macro Front_Right_ReadPS calls this function. 
*  
*******************************************************************************/
uint8 Front_Right_Read(void) 
{
    return (Front_Right_PS & Front_Right_MASK) >> Front_Right_SHIFT;
}


/*******************************************************************************
* Function Name: Front_Right_ReadDataReg
********************************************************************************
*
* Summary:
*  Read the current value assigned to a Digital Port's data output register
*
* Parameters:  
*  None 
*
* Return: 
*  Returns the current value assigned to the Digital Port's data output register
*  
*******************************************************************************/
uint8 Front_Right_ReadDataReg(void) 
{
    return (Front_Right_DR & Front_Right_MASK) >> Front_Right_SHIFT;
}


/* If Interrupts Are Enabled for this Pins component */ 
#if defined(Front_Right_INTSTAT) 

    /*******************************************************************************
    * Function Name: Front_Right_ClearInterrupt
    ********************************************************************************
    * Summary:
    *  Clears any active interrupts attached to port and returns the value of the 
    *  interrupt status register.
    *
    * Parameters:  
    *  None 
    *
    * Return: 
    *  Returns the value of the interrupt status register
    *  
    *******************************************************************************/
    uint8 Front_Right_ClearInterrupt(void) 
    {
        return (Front_Right_INTSTAT & Front_Right_MASK) >> Front_Right_SHIFT;
    }

#endif /* If Interrupts Are Enabled for this Pins component */ 

#endif /* CY_PSOC5A... */

    
/* [] END OF FILE */
