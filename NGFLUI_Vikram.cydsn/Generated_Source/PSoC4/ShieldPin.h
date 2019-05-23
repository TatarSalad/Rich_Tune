/*******************************************************************************
* File Name: ShieldPin.h  
* Version 2.20
*
* Description:
*  This file contains Pin function prototypes and register defines
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_ShieldPin_H) /* Pins ShieldPin_H */
#define CY_PINS_ShieldPin_H

#include "cytypes.h"
#include "cyfitter.h"
#include "ShieldPin_aliases.h"


/***************************************
*     Data Struct Definitions
***************************************/

/**
* \addtogroup group_structures
* @{
*/
    
/* Structure for sleep mode support */
typedef struct
{
    uint32 pcState; /**< State of the port control register */
    uint32 sioState; /**< State of the SIO configuration */
    uint32 usbState; /**< State of the USBIO regulator */
} ShieldPin_BACKUP_STRUCT;

/** @} structures */


/***************************************
*        Function Prototypes             
***************************************/
/**
* \addtogroup group_general
* @{
*/
uint8   ShieldPin_Read(void);
void    ShieldPin_Write(uint8 value);
uint8   ShieldPin_ReadDataReg(void);
#if defined(ShieldPin__PC) || (CY_PSOC4_4200L) 
    void    ShieldPin_SetDriveMode(uint8 mode);
#endif
void    ShieldPin_SetInterruptMode(uint16 position, uint16 mode);
uint8   ShieldPin_ClearInterrupt(void);
/** @} general */

/**
* \addtogroup group_power
* @{
*/
void ShieldPin_Sleep(void); 
void ShieldPin_Wakeup(void);
/** @} power */


/***************************************
*           API Constants        
***************************************/
#if defined(ShieldPin__PC) || (CY_PSOC4_4200L) 
    /* Drive Modes */
    #define ShieldPin_DRIVE_MODE_BITS        (3)
    #define ShieldPin_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - ShieldPin_DRIVE_MODE_BITS))

    /**
    * \addtogroup group_constants
    * @{
    */
        /** \addtogroup driveMode Drive mode constants
         * \brief Constants to be passed as "mode" parameter in the ShieldPin_SetDriveMode() function.
         *  @{
         */
        #define ShieldPin_DM_ALG_HIZ         (0x00u) /**< \brief High Impedance Analog   */
        #define ShieldPin_DM_DIG_HIZ         (0x01u) /**< \brief High Impedance Digital  */
        #define ShieldPin_DM_RES_UP          (0x02u) /**< \brief Resistive Pull Up       */
        #define ShieldPin_DM_RES_DWN         (0x03u) /**< \brief Resistive Pull Down     */
        #define ShieldPin_DM_OD_LO           (0x04u) /**< \brief Open Drain, Drives Low  */
        #define ShieldPin_DM_OD_HI           (0x05u) /**< \brief Open Drain, Drives High */
        #define ShieldPin_DM_STRONG          (0x06u) /**< \brief Strong Drive            */
        #define ShieldPin_DM_RES_UPDWN       (0x07u) /**< \brief Resistive Pull Up/Down  */
        /** @} driveMode */
    /** @} group_constants */
#endif

/* Digital Port Constants */
#define ShieldPin_MASK               ShieldPin__MASK
#define ShieldPin_SHIFT              ShieldPin__SHIFT
#define ShieldPin_WIDTH              1u

/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in ShieldPin_SetInterruptMode() function.
     *  @{
     */
        #define ShieldPin_INTR_NONE      ((uint16)(0x0000u)) /**< \brief Disabled             */
        #define ShieldPin_INTR_RISING    ((uint16)(0x5555u)) /**< \brief Rising edge trigger  */
        #define ShieldPin_INTR_FALLING   ((uint16)(0xaaaau)) /**< \brief Falling edge trigger */
        #define ShieldPin_INTR_BOTH      ((uint16)(0xffffu)) /**< \brief Both edge trigger    */
    /** @} intrMode */
/** @} group_constants */

/* SIO LPM definition */
#if defined(ShieldPin__SIO)
    #define ShieldPin_SIO_LPM_MASK       (0x03u)
#endif

/* USBIO definitions */
#if !defined(ShieldPin__PC) && (CY_PSOC4_4200L)
    #define ShieldPin_USBIO_ENABLE               ((uint32)0x80000000u)
    #define ShieldPin_USBIO_DISABLE              ((uint32)(~ShieldPin_USBIO_ENABLE))
    #define ShieldPin_USBIO_SUSPEND_SHIFT        CYFLD_USBDEVv2_USB_SUSPEND__OFFSET
    #define ShieldPin_USBIO_SUSPEND_DEL_SHIFT    CYFLD_USBDEVv2_USB_SUSPEND_DEL__OFFSET
    #define ShieldPin_USBIO_ENTER_SLEEP          ((uint32)((1u << ShieldPin_USBIO_SUSPEND_SHIFT) \
                                                        | (1u << ShieldPin_USBIO_SUSPEND_DEL_SHIFT)))
    #define ShieldPin_USBIO_EXIT_SLEEP_PH1       ((uint32)~((uint32)(1u << ShieldPin_USBIO_SUSPEND_SHIFT)))
    #define ShieldPin_USBIO_EXIT_SLEEP_PH2       ((uint32)~((uint32)(1u << ShieldPin_USBIO_SUSPEND_DEL_SHIFT)))
    #define ShieldPin_USBIO_CR1_OFF              ((uint32)0xfffffffeu)
#endif


/***************************************
*             Registers        
***************************************/
/* Main Port Registers */
#if defined(ShieldPin__PC)
    /* Port Configuration */
    #define ShieldPin_PC                 (* (reg32 *) ShieldPin__PC)
#endif
/* Pin State */
#define ShieldPin_PS                     (* (reg32 *) ShieldPin__PS)
/* Data Register */
#define ShieldPin_DR                     (* (reg32 *) ShieldPin__DR)
/* Input Buffer Disable Override */
#define ShieldPin_INP_DIS                (* (reg32 *) ShieldPin__PC2)

/* Interrupt configuration Registers */
#define ShieldPin_INTCFG                 (* (reg32 *) ShieldPin__INTCFG)
#define ShieldPin_INTSTAT                (* (reg32 *) ShieldPin__INTSTAT)

/* "Interrupt cause" register for Combined Port Interrupt (AllPortInt) in GSRef component */
#if defined (CYREG_GPIO_INTR_CAUSE)
    #define ShieldPin_INTR_CAUSE         (* (reg32 *) CYREG_GPIO_INTR_CAUSE)
#endif

/* SIO register */
#if defined(ShieldPin__SIO)
    #define ShieldPin_SIO_REG            (* (reg32 *) ShieldPin__SIO)
#endif /* (ShieldPin__SIO_CFG) */

/* USBIO registers */
#if !defined(ShieldPin__PC) && (CY_PSOC4_4200L)
    #define ShieldPin_USB_POWER_REG       (* (reg32 *) CYREG_USBDEVv2_USB_POWER_CTRL)
    #define ShieldPin_CR1_REG             (* (reg32 *) CYREG_USBDEVv2_CR1)
    #define ShieldPin_USBIO_CTRL_REG      (* (reg32 *) CYREG_USBDEVv2_USB_USBIO_CTRL)
#endif    
    
    
/***************************************
* The following code is DEPRECATED and 
* must not be used in new designs.
***************************************/
/**
* \addtogroup group_deprecated
* @{
*/
#define ShieldPin_DRIVE_MODE_SHIFT       (0x00u)
#define ShieldPin_DRIVE_MODE_MASK        (0x07u << ShieldPin_DRIVE_MODE_SHIFT)
/** @} deprecated */

#endif /* End Pins ShieldPin_H */


/* [] END OF FILE */
