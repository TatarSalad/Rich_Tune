/*******************************************************************************
* File Name: ShieldPin.h  
* Version 2.20
*
* Description:
*  This file contains the Alias definitions for Per-Pin APIs in cypins.h. 
*  Information on using these APIs can be found in the System Reference Guide.
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_ShieldPin_ALIASES_H) /* Pins ShieldPin_ALIASES_H */
#define CY_PINS_ShieldPin_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"


/***************************************
*              Constants        
***************************************/
#define ShieldPin_0			(ShieldPin__0__PC)
#define ShieldPin_0_PS		(ShieldPin__0__PS)
#define ShieldPin_0_PC		(ShieldPin__0__PC)
#define ShieldPin_0_DR		(ShieldPin__0__DR)
#define ShieldPin_0_SHIFT	(ShieldPin__0__SHIFT)
#define ShieldPin_0_INTR	((uint16)((uint16)0x0003u << (ShieldPin__0__SHIFT*2u)))

#define ShieldPin_INTR_ALL	 ((uint16)(ShieldPin_0_INTR))


#endif /* End Pins ShieldPin_ALIASES_H */


/* [] END OF FILE */
