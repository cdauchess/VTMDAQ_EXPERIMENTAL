/*******************************************************************************
* File Name: Back_Left.c  
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
#include "Back_Left.h"

/* APIs are not generated for P15[7:6] on PSoC 5 */
#if !(CY_PSOC5A &&\
	 Back_Left__PORT == 15 && ((Back_Left__MASK & 0xC0) != 0))


/*******************************************************************************
* Function Name: Back_Left_Write
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
void Back_Left_Write(uint8 value) 
{
    uint8 staticBits = (Back_Left_DR & (uint8)(~Back_Left_MASK));
    Back_Left_DR = staticBits | ((uint8)(value << Back_Left_SHIFT) & Back_Left_MASK);
}


/*******************************************************************************
* Function Name: Back_Left_SetDriveMode
********************************************************************************
*
* Summary:
*  Change the drive mode on the pins of the port.
* 
* Parameters:  
*  mode:  Change the pins to one of the following drive modes.
*
*  Back_Left_DM_STRONG     Strong Drive 
*  Back_Left_DM_OD_HI      Open Drain, Drives High 
*  Back_Left_DM_OD_LO      Open Drain, Drives Low 
*  Back_Left_DM_RES_UP     Resistive Pull Up 
*  Back_Left_DM_RES_DWN    Resistive Pull Down 
*  Back_Left_DM_RES_UPDWN  Resistive Pull Up/Down 
*  Back_Left_DM_DIG_HIZ    High Impedance Digital 
*  Back_Left_DM_ALG_HIZ    High Impedance Analog 
*
* Return: 
*  None
*
*******************************************************************************/
void Back_Left_SetDriveMode(uint8 mode) 
{
	CyPins_SetPinDriveMode(Back_Left_0, mode);
}


/*******************************************************************************
* Function Name: Back_Left_Read
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
*  Macro Back_Left_ReadPS calls this function. 
*  
*******************************************************************************/
uint8 Back_Left_Read(void) 
{
    return (Back_Left_PS & Back_Left_MASK) >> Back_Left_SHIFT;
}


/*******************************************************************************
* Function Name: Back_Left_ReadDataReg
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
uint8 Back_Left_ReadDataReg(void) 
{
    return (Back_Left_DR & Back_Left_MASK) >> Back_Left_SHIFT;
}


/* If Interrupts Are Enabled for this Pins component */ 
#if defined(Back_Left_INTSTAT) 

    /*******************************************************************************
    * Function Name: Back_Left_ClearInterrupt
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
    uint8 Back_Left_ClearInterrupt(void) 
    {
        return (Back_Left_INTSTAT & Back_Left_MASK) >> Back_Left_SHIFT;
    }

#endif /* If Interrupts Are Enabled for this Pins component */ 

#endif /* CY_PSOC5A... */

    
/* [] END OF FILE */
