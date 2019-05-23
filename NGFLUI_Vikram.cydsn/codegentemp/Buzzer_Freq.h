/*******************************************************************************
* File Name: Buzzer_Freq.h  
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

#if !defined(CY_PINS_Buzzer_Freq_H) /* Pins Buzzer_Freq_H */
#define CY_PINS_Buzzer_Freq_H

#include "cytypes.h"
#include "cyfitter.h"
#include "Buzzer_Freq_aliases.h"


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
} Buzzer_Freq_BACKUP_STRUCT;

/** @} structures */


/***************************************
*        Function Prototypes             
***************************************/
/**
* \addtogroup group_general
* @{
*/
uint8   Buzzer_Freq_Read(void);
void    Buzzer_Freq_Write(uint8 value);
uint8   Buzzer_Freq_ReadDataReg(void);
#if defined(Buzzer_Freq__PC) || (CY_PSOC4_4200L) 
    void    Buzzer_Freq_SetDriveMode(uint8 mode);
#endif
void    Buzzer_Freq_SetInterruptMode(uint16 position, uint16 mode);
uint8   Buzzer_Freq_ClearInterrupt(void);
/** @} general */

/**
* \addtogroup group_power
* @{
*/
void Buzzer_Freq_Sleep(void); 
void Buzzer_Freq_Wakeup(void);
/** @} power */


/***************************************
*           API Constants        
***************************************/
#if defined(Buzzer_Freq__PC) || (CY_PSOC4_4200L) 
    /* Drive Modes */
    #define Buzzer_Freq_DRIVE_MODE_BITS        (3)
    #define Buzzer_Freq_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - Buzzer_Freq_DRIVE_MODE_BITS))

    /**
    * \addtogroup group_constants
    * @{
    */
        /** \addtogroup driveMode Drive mode constants
         * \brief Constants to be passed as "mode" parameter in the Buzzer_Freq_SetDriveMode() function.
         *  @{
         */
        #define Buzzer_Freq_DM_ALG_HIZ         (0x00u) /**< \brief High Impedance Analog   */
        #define Buzzer_Freq_DM_DIG_HIZ         (0x01u) /**< \brief High Impedance Digital  */
        #define Buzzer_Freq_DM_RES_UP          (0x02u) /**< \brief Resistive Pull Up       */
        #define Buzzer_Freq_DM_RES_DWN         (0x03u) /**< \brief Resistive Pull Down     */
        #define Buzzer_Freq_DM_OD_LO           (0x04u) /**< \brief Open Drain, Drives Low  */
        #define Buzzer_Freq_DM_OD_HI           (0x05u) /**< \brief Open Drain, Drives High */
        #define Buzzer_Freq_DM_STRONG          (0x06u) /**< \brief Strong Drive            */
        #define Buzzer_Freq_DM_RES_UPDWN       (0x07u) /**< \brief Resistive Pull Up/Down  */
        /** @} driveMode */
    /** @} group_constants */
#endif

/* Digital Port Constants */
#define Buzzer_Freq_MASK               Buzzer_Freq__MASK
#define Buzzer_Freq_SHIFT              Buzzer_Freq__SHIFT
#define Buzzer_Freq_WIDTH              1u

/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in Buzzer_Freq_SetInterruptMode() function.
     *  @{
     */
        #define Buzzer_Freq_INTR_NONE      ((uint16)(0x0000u)) /**< \brief Disabled             */
        #define Buzzer_Freq_INTR_RISING    ((uint16)(0x5555u)) /**< \brief Rising edge trigger  */
        #define Buzzer_Freq_INTR_FALLING   ((uint16)(0xaaaau)) /**< \brief Falling edge trigger */
        #define Buzzer_Freq_INTR_BOTH      ((uint16)(0xffffu)) /**< \brief Both edge trigger    */
    /** @} intrMode */
/** @} group_constants */

/* SIO LPM definition */
#if defined(Buzzer_Freq__SIO)
    #define Buzzer_Freq_SIO_LPM_MASK       (0x03u)
#endif

/* USBIO definitions */
#if !defined(Buzzer_Freq__PC) && (CY_PSOC4_4200L)
    #define Buzzer_Freq_USBIO_ENABLE               ((uint32)0x80000000u)
    #define Buzzer_Freq_USBIO_DISABLE              ((uint32)(~Buzzer_Freq_USBIO_ENABLE))
    #define Buzzer_Freq_USBIO_SUSPEND_SHIFT        CYFLD_USBDEVv2_USB_SUSPEND__OFFSET
    #define Buzzer_Freq_USBIO_SUSPEND_DEL_SHIFT    CYFLD_USBDEVv2_USB_SUSPEND_DEL__OFFSET
    #define Buzzer_Freq_USBIO_ENTER_SLEEP          ((uint32)((1u << Buzzer_Freq_USBIO_SUSPEND_SHIFT) \
                                                        | (1u << Buzzer_Freq_USBIO_SUSPEND_DEL_SHIFT)))
    #define Buzzer_Freq_USBIO_EXIT_SLEEP_PH1       ((uint32)~((uint32)(1u << Buzzer_Freq_USBIO_SUSPEND_SHIFT)))
    #define Buzzer_Freq_USBIO_EXIT_SLEEP_PH2       ((uint32)~((uint32)(1u << Buzzer_Freq_USBIO_SUSPEND_DEL_SHIFT)))
    #define Buzzer_Freq_USBIO_CR1_OFF              ((uint32)0xfffffffeu)
#endif


/***************************************
*             Registers        
***************************************/
/* Main Port Registers */
#if defined(Buzzer_Freq__PC)
    /* Port Configuration */
    #define Buzzer_Freq_PC                 (* (reg32 *) Buzzer_Freq__PC)
#endif
/* Pin State */
#define Buzzer_Freq_PS                     (* (reg32 *) Buzzer_Freq__PS)
/* Data Register */
#define Buzzer_Freq_DR                     (* (reg32 *) Buzzer_Freq__DR)
/* Input Buffer Disable Override */
#define Buzzer_Freq_INP_DIS                (* (reg32 *) Buzzer_Freq__PC2)

/* Interrupt configuration Registers */
#define Buzzer_Freq_INTCFG                 (* (reg32 *) Buzzer_Freq__INTCFG)
#define Buzzer_Freq_INTSTAT                (* (reg32 *) Buzzer_Freq__INTSTAT)

/* "Interrupt cause" register for Combined Port Interrupt (AllPortInt) in GSRef component */
#if defined (CYREG_GPIO_INTR_CAUSE)
    #define Buzzer_Freq_INTR_CAUSE         (* (reg32 *) CYREG_GPIO_INTR_CAUSE)
#endif

/* SIO register */
#if defined(Buzzer_Freq__SIO)
    #define Buzzer_Freq_SIO_REG            (* (reg32 *) Buzzer_Freq__SIO)
#endif /* (Buzzer_Freq__SIO_CFG) */

/* USBIO registers */
#if !defined(Buzzer_Freq__PC) && (CY_PSOC4_4200L)
    #define Buzzer_Freq_USB_POWER_REG       (* (reg32 *) CYREG_USBDEVv2_USB_POWER_CTRL)
    #define Buzzer_Freq_CR1_REG             (* (reg32 *) CYREG_USBDEVv2_CR1)
    #define Buzzer_Freq_USBIO_CTRL_REG      (* (reg32 *) CYREG_USBDEVv2_USB_USBIO_CTRL)
#endif    
    
    
/***************************************
* The following code is DEPRECATED and 
* must not be used in new designs.
***************************************/
/**
* \addtogroup group_deprecated
* @{
*/
#define Buzzer_Freq_DRIVE_MODE_SHIFT       (0x00u)
#define Buzzer_Freq_DRIVE_MODE_MASK        (0x07u << Buzzer_Freq_DRIVE_MODE_SHIFT)
/** @} deprecated */

#endif /* End Pins Buzzer_Freq_H */


/* [] END OF FILE */
