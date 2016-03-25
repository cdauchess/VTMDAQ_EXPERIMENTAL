/*******************************************************************************
* File Name: Back_Left.h  
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

#if !defined(CY_PINS_Back_Left_H) /* Pins Back_Left_H */
#define CY_PINS_Back_Left_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "Back_Left_aliases.h"

/* Check to see if required defines such as CY_PSOC5A are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5A)
    #error Component cy_pins_v2_10 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5A) */

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 Back_Left__PORT == 15 && ((Back_Left__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

void    Back_Left_Write(uint8 value) ;
void    Back_Left_SetDriveMode(uint8 mode) ;
uint8   Back_Left_ReadDataReg(void) ;
uint8   Back_Left_Read(void) ;
uint8   Back_Left_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define Back_Left_DM_ALG_HIZ         PIN_DM_ALG_HIZ
#define Back_Left_DM_DIG_HIZ         PIN_DM_DIG_HIZ
#define Back_Left_DM_RES_UP          PIN_DM_RES_UP
#define Back_Left_DM_RES_DWN         PIN_DM_RES_DWN
#define Back_Left_DM_OD_LO           PIN_DM_OD_LO
#define Back_Left_DM_OD_HI           PIN_DM_OD_HI
#define Back_Left_DM_STRONG          PIN_DM_STRONG
#define Back_Left_DM_RES_UPDWN       PIN_DM_RES_UPDWN

/* Digital Port Constants */
#define Back_Left_MASK               Back_Left__MASK
#define Back_Left_SHIFT              Back_Left__SHIFT
#define Back_Left_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define Back_Left_PS                     (* (reg8 *) Back_Left__PS)
/* Data Register */
#define Back_Left_DR                     (* (reg8 *) Back_Left__DR)
/* Port Number */
#define Back_Left_PRT_NUM                (* (reg8 *) Back_Left__PRT) 
/* Connect to Analog Globals */                                                  
#define Back_Left_AG                     (* (reg8 *) Back_Left__AG)                       
/* Analog MUX bux enable */
#define Back_Left_AMUX                   (* (reg8 *) Back_Left__AMUX) 
/* Bidirectional Enable */                                                        
#define Back_Left_BIE                    (* (reg8 *) Back_Left__BIE)
/* Bit-mask for Aliased Register Access */
#define Back_Left_BIT_MASK               (* (reg8 *) Back_Left__BIT_MASK)
/* Bypass Enable */
#define Back_Left_BYP                    (* (reg8 *) Back_Left__BYP)
/* Port wide control signals */                                                   
#define Back_Left_CTL                    (* (reg8 *) Back_Left__CTL)
/* Drive Modes */
#define Back_Left_DM0                    (* (reg8 *) Back_Left__DM0) 
#define Back_Left_DM1                    (* (reg8 *) Back_Left__DM1)
#define Back_Left_DM2                    (* (reg8 *) Back_Left__DM2) 
/* Input Buffer Disable Override */
#define Back_Left_INP_DIS                (* (reg8 *) Back_Left__INP_DIS)
/* LCD Common or Segment Drive */
#define Back_Left_LCD_COM_SEG            (* (reg8 *) Back_Left__LCD_COM_SEG)
/* Enable Segment LCD */
#define Back_Left_LCD_EN                 (* (reg8 *) Back_Left__LCD_EN)
/* Slew Rate Control */
#define Back_Left_SLW                    (* (reg8 *) Back_Left__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define Back_Left_PRTDSI__CAPS_SEL       (* (reg8 *) Back_Left__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define Back_Left_PRTDSI__DBL_SYNC_IN    (* (reg8 *) Back_Left__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define Back_Left_PRTDSI__OE_SEL0        (* (reg8 *) Back_Left__PRTDSI__OE_SEL0) 
#define Back_Left_PRTDSI__OE_SEL1        (* (reg8 *) Back_Left__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define Back_Left_PRTDSI__OUT_SEL0       (* (reg8 *) Back_Left__PRTDSI__OUT_SEL0) 
#define Back_Left_PRTDSI__OUT_SEL1       (* (reg8 *) Back_Left__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define Back_Left_PRTDSI__SYNC_OUT       (* (reg8 *) Back_Left__PRTDSI__SYNC_OUT) 


#if defined(Back_Left__INTSTAT)  /* Interrupt Registers */

    #define Back_Left_INTSTAT                (* (reg8 *) Back_Left__INTSTAT)
    #define Back_Left_SNAP                   (* (reg8 *) Back_Left__SNAP)

#endif /* Interrupt Registers */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_Back_Left_H */


/* [] END OF FILE */
