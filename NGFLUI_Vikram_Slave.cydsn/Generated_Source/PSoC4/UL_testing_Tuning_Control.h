/***************************************************************************//**
* \file UL_testing_Tuning_Control.h
* \version 6.0
*
* \brief
*   This file provides the function prototypes of the Control Block.
*
* \see UL_testing_Tuning v6.0 Datasheet
*
*//*****************************************************************************
* Copyright (2016-2018), Cypress Semiconductor Corporation.
********************************************************************************
* This software is owned by Cypress Semiconductor Corporation (Cypress) and is
* protected by and subject to worldwide patent protection (United States and
* foreign), United States copyright laws and international treaty provisions.
* Cypress hereby grants to licensee a personal, non-exclusive, non-transferable
* license to copy, use, modify, create derivative works of, and compile the
* Cypress Source Code and derivative works for the sole purpose of creating
* custom software in support of licensee product to be used only in conjunction
* with a Cypress integrated circuit as specified in the applicable agreement.
* Any reproduction, modification, translation, compilation, or representation of
* this software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: CYPRESS MAKES NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, WITH
* REGARD TO THIS MATERIAL, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* Cypress reserves the right to make changes without further notice to the
* materials described herein. Cypress does not assume any liability arising out
* of the application or use of any product or circuit described herein. Cypress
* does not authorize its products for use as critical components in life-support
* systems where a malfunction or failure may reasonably be expected to result in
* significant injury to the user. The inclusion of Cypress' product in a life-
* support systems application implies that the manufacturer assumes all risk of
* such use and in doing so indemnifies Cypress against all charges. Use may be
* limited by and subject to the applicable Cypress software license agreement.
*******************************************************************************/

#if !defined(CY_SENSE_UL_testing_Tuning_CONTROL_H)
#define CY_SENSE_UL_testing_Tuning_CONTROL_H

#include "UL_testing_Tuning_Configuration.h"

/*******************************************************************************
* Function Prototypes
*******************************************************************************/

/*******************************************************************************
* HIGH LEVEL API
*******************************************************************************/

/**
* \cond ((SECTION_C_HIGH_LEVEL) || (SECTION_I_HIGH_LEVEL))
*
* \addtogroup group_c_high_level
* \{
*/

cystatus UL_testing_Tuning_Start(void);
cystatus UL_testing_Tuning_Stop(void);
cystatus UL_testing_Tuning_Resume(void);

cystatus UL_testing_Tuning_ProcessAllWidgets(void);
cystatus UL_testing_Tuning_ProcessWidget(uint32 widgetId);

void UL_testing_Tuning_Sleep(void);
void UL_testing_Tuning_Wakeup(void);

/** \}
* \endcond */

/**
* \cond (SECTION_C_LOW_LEVEL  || SECTION_I_LOW_LEVEL)
* \addtogroup group_c_low_level
* \{
*/

cystatus UL_testing_Tuning_ProcessWidgetExt(uint32 widgetId, uint32 mode);
cystatus UL_testing_Tuning_ProcessSensorExt(uint32 widgetId, uint32 sensorId, uint32 mode);

/** \}
* \endcond */

/*******************************************************************************
* Function Prototypes - internal functions
*******************************************************************************/

/**
* \cond SECTION_C_INTERNAL
* \addtogroup group_c_internal
* \{
*/
cystatus UL_testing_Tuning_Initialize(void);

/** \}
* \endcond */

#endif /* End CY_SENSE_UL_testing_Tuning_CONTROL_H */


/* [] END OF FILE */
