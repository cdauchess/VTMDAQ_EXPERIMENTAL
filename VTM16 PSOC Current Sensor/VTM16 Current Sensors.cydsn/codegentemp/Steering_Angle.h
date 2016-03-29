/*******************************************************************************
* File Name: Steering_Angle.h  
* Version 2.10
*
* Description:
*  This file containts Control Register function prototypes and register defines
*
* Note:
*
********************************************************************************
* Copyright 2008-2014, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_Steering_Angle_H) /* Pins Steering_Angle_H */
#define CY_PINS_Steering_Angle_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "Steering_Angle_aliases.h"

/* Check to see if required defines such as CY_PSOC5A are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5A)
    #error Component cy_pins_v2_10 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5A) */

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 Steering_Angle__PORT == 15 && ((Steering_Angle__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

void    Steering_Angle_Write(uint8 value) ;
void    Steering_Angle_SetDriveMode(uint8 mode) ;
uint8   Steering_Angle_ReadDataReg(void) ;
uint8   Steering_Angle_Read(void) ;
uint8   Steering_Angle_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define Steering_Angle_DM_ALG_HIZ         PIN_DM_ALG_HIZ
#define Steering_Angle_DM_DIG_HIZ         PIN_DM_DIG_HIZ
#define Steering_Angle_DM_RES_UP          PIN_DM_RES_UP
#define Steering_Angle_DM_RES_DWN         PIN_DM_RES_DWN
#define Steering_Angle_DM_OD_LO           PIN_DM_OD_LO
#define Steering_Angle_DM_OD_HI           PIN_DM_OD_HI
#define Steering_Angle_DM_STRONG          PIN_DM_STRONG
#define Steering_Angle_DM_RES_UPDWN       PIN_DM_RES_UPDWN

/* Digital Port Constants */
#define Steering_Angle_MASK               Steering_Angle__MASK
#define Steering_Angle_SHIFT              Steering_Angle__SHIFT
#define Steering_Angle_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define Steering_Angle_PS                     (* (reg8 *) Steering_Angle__PS)
/* Data Register */
#define Steering_Angle_DR                     (* (reg8 *) Steering_Angle__DR)
/* Port Number */
#define Steering_Angle_PRT_NUM                (* (reg8 *) Steering_Angle__PRT) 
/* Connect to Analog Globals */                                                  
#define Steering_Angle_AG                     (* (reg8 *) Steering_Angle__AG)                       
/* Analog MUX bux enable */
#define Steering_Angle_AMUX                   (* (reg8 *) Steering_Angle__AMUX) 
/* Bidirectional Enable */                                                        
#define Steering_Angle_BIE                    (* (reg8 *) Steering_Angle__BIE)
/* Bit-mask for Aliased Register Access */
#define Steering_Angle_BIT_MASK               (* (reg8 *) Steering_Angle__BIT_MASK)
/* Bypass Enable */
#define Steering_Angle_BYP                    (* (reg8 *) Steering_Angle__BYP)
/* Port wide control signals */                                                   
#define Steering_Angle_CTL                    (* (reg8 *) Steering_Angle__CTL)
/* Drive Modes */
#define Steering_Angle_DM0                    (* (reg8 *) Steering_Angle__DM0) 
#define Steering_Angle_DM1                    (* (reg8 *) Steering_Angle__DM1)
#define Steering_Angle_DM2                    (* (reg8 *) Steering_Angle__DM2) 
/* Input Buffer Disable Override */
#define Steering_Angle_INP_DIS                (* (reg8 *) Steering_Angle__INP_DIS)
/* LCD Common or Segment Drive */
#define Steering_Angle_LCD_COM_SEG            (* (reg8 *) Steering_Angle__LCD_COM_SEG)
/* Enable Segment LCD */
#define Steering_Angle_LCD_EN                 (* (reg8 *) Steering_Angle__LCD_EN)
/* Slew Rate Control */
#define Steering_Angle_SLW                    (* (reg8 *) Steering_Angle__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define Steering_Angle_PRTDSI__CAPS_SEL       (* (reg8 *) Steering_Angle__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define Steering_Angle_PRTDSI__DBL_SYNC_IN    (* (reg8 *) Steering_Angle__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define Steering_Angle_PRTDSI__OE_SEL0        (* (reg8 *) Steering_Angle__PRTDSI__OE_SEL0) 
#define Steering_Angle_PRTDSI__OE_SEL1        (* (reg8 *) Steering_Angle__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define Steering_Angle_PRTDSI__OUT_SEL0       (* (reg8 *) Steering_Angle__PRTDSI__OUT_SEL0) 
#define Steering_Angle_PRTDSI__OUT_SEL1       (* (reg8 *) Steering_Angle__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define Steering_Angle_PRTDSI__SYNC_OUT       (* (reg8 *) Steering_Angle__PRTDSI__SYNC_OUT) 


#if defined(Steering_Angle__INTSTAT)  /* Interrupt Registers */

    #define Steering_Angle_INTSTAT                (* (reg8 *) Steering_Angle__INTSTAT)
    #define Steering_Angle_SNAP                   (* (reg8 *) Steering_Angle__SNAP)

#endif /* Interrupt Registers */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_Steering_Angle_H */


/* [] END OF FILE */
