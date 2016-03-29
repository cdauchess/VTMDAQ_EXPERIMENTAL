/*******************************************************************************
* File Name: Steering_Angle.c  
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
#include "Steering_Angle.h"

/* APIs are not generated for P15[7:6] on PSoC 5 */
#if !(CY_PSOC5A &&\
	 Steering_Angle__PORT == 15 && ((Steering_Angle__MASK & 0xC0) != 0))


/*******************************************************************************
* Function Name: Steering_Angle_Write
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
void Steering_Angle_Write(uint8 value) 
{
    uint8 staticBits = (Steering_Angle_DR & (uint8)(~Steering_Angle_MASK));
    Steering_Angle_DR = staticBits | ((uint8)(value << Steering_Angle_SHIFT) & Steering_Angle_MASK);
}


/*******************************************************************************
* Function Name: Steering_Angle_SetDriveMode
********************************************************************************
*
* Summary:
*  Change the drive mode on the pins of the port.
* 
* Parameters:  
*  mode:  Change the pins to one of the following drive modes.
*
*  Steering_Angle_DM_STRONG     Strong Drive 
*  Steering_Angle_DM_OD_HI      Open Drain, Drives High 
*  Steering_Angle_DM_OD_LO      Open Drain, Drives Low 
*  Steering_Angle_DM_RES_UP     Resistive Pull Up 
*  Steering_Angle_DM_RES_DWN    Resistive Pull Down 
*  Steering_Angle_DM_RES_UPDWN  Resistive Pull Up/Down 
*  Steering_Angle_DM_DIG_HIZ    High Impedance Digital 
*  Steering_Angle_DM_ALG_HIZ    High Impedance Analog 
*
* Return: 
*  None
*
*******************************************************************************/
void Steering_Angle_SetDriveMode(uint8 mode) 
{
	CyPins_SetPinDriveMode(Steering_Angle_0, mode);
}


/*******************************************************************************
* Function Name: Steering_Angle_Read
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
*  Macro Steering_Angle_ReadPS calls this function. 
*  
*******************************************************************************/
uint8 Steering_Angle_Read(void) 
{
    return (Steering_Angle_PS & Steering_Angle_MASK) >> Steering_Angle_SHIFT;
}


/*******************************************************************************
* Function Name: Steering_Angle_ReadDataReg
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
uint8 Steering_Angle_ReadDataReg(void) 
{
    return (Steering_Angle_DR & Steering_Angle_MASK) >> Steering_Angle_SHIFT;
}


/* If Interrupts Are Enabled for this Pins component */ 
#if defined(Steering_Angle_INTSTAT) 

    /*******************************************************************************
    * Function Name: Steering_Angle_ClearInterrupt
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
    uint8 Steering_Angle_ClearInterrupt(void) 
    {
        return (Steering_Angle_INTSTAT & Steering_Angle_MASK) >> Steering_Angle_SHIFT;
    }

#endif /* If Interrupts Are Enabled for this Pins component */ 

#endif /* CY_PSOC5A... */

    
/* [] END OF FILE */
