/*******************************************************************************
* File Name: Back_Right.c  
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
#include "Back_Right.h"

/* APIs are not generated for P15[7:6] on PSoC 5 */
#if !(CY_PSOC5A &&\
	 Back_Right__PORT == 15 && ((Back_Right__MASK & 0xC0) != 0))


/*******************************************************************************
* Function Name: Back_Right_Write
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
void Back_Right_Write(uint8 value) 
{
    uint8 staticBits = (Back_Right_DR & (uint8)(~Back_Right_MASK));
    Back_Right_DR = staticBits | ((uint8)(value << Back_Right_SHIFT) & Back_Right_MASK);
}


/*******************************************************************************
* Function Name: Back_Right_SetDriveMode
********************************************************************************
*
* Summary:
*  Change the drive mode on the pins of the port.
* 
* Parameters:  
*  mode:  Change the pins to one of the following drive modes.
*
*  Back_Right_DM_STRONG     Strong Drive 
*  Back_Right_DM_OD_HI      Open Drain, Drives High 
*  Back_Right_DM_OD_LO      Open Drain, Drives Low 
*  Back_Right_DM_RES_UP     Resistive Pull Up 
*  Back_Right_DM_RES_DWN    Resistive Pull Down 
*  Back_Right_DM_RES_UPDWN  Resistive Pull Up/Down 
*  Back_Right_DM_DIG_HIZ    High Impedance Digital 
*  Back_Right_DM_ALG_HIZ    High Impedance Analog 
*
* Return: 
*  None
*
*******************************************************************************/
void Back_Right_SetDriveMode(uint8 mode) 
{
	CyPins_SetPinDriveMode(Back_Right_0, mode);
}


/*******************************************************************************
* Function Name: Back_Right_Read
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
*  Macro Back_Right_ReadPS calls this function. 
*  
*******************************************************************************/
uint8 Back_Right_Read(void) 
{
    return (Back_Right_PS & Back_Right_MASK) >> Back_Right_SHIFT;
}


/*******************************************************************************
* Function Name: Back_Right_ReadDataReg
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
uint8 Back_Right_ReadDataReg(void) 
{
    return (Back_Right_DR & Back_Right_MASK) >> Back_Right_SHIFT;
}


/* If Interrupts Are Enabled for this Pins component */ 
#if defined(Back_Right_INTSTAT) 

    /*******************************************************************************
    * Function Name: Back_Right_ClearInterrupt
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
    uint8 Back_Right_ClearInterrupt(void) 
    {
        return (Back_Right_INTSTAT & Back_Right_MASK) >> Back_Right_SHIFT;
    }

#endif /* If Interrupts Are Enabled for this Pins component */ 

#endif /* CY_PSOC5A... */

    
/* [] END OF FILE */
