/***************************************************************************//**
* \file UL_testing_Tuning_Processing.h
* \version 6.0
*
* \brief
*   This file provides the function prototypes for the Data Processing module.
*   The Data Processing module is responsible for the low level raw counts
*   processing provided by the sensing module, maintaining baseline and
*   difference values and performing high-level widget processing like updating
*   button status or calculating slider position.
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

#if !defined(CY_SENSE_UL_testing_Tuning_DATA_PROCESS_H)
#define CY_SENSE_UL_testing_Tuning_DATA_PROCESS_H

#include "UL_testing_Tuning_Configuration.h"
#include "UL_testing_Tuning_Structure.h"

#if (0 != (UL_testing_Tuning_CSD_AUTOTUNE & UL_testing_Tuning_CSD_SS_TH_EN))
    #include "UL_testing_Tuning_SmartSense_LL.h"
#endif /* (0 != (UL_testing_Tuning_CSD_AUTOTUNE & UL_testing_Tuning_CSD_SS_TH_EN)) */

/*******************************************************************************
* Definitions
*******************************************************************************/

/* Defines the data processing tasks */

/* Applies all enabled filters in the default order to the raw counts */
#define UL_testing_Tuning_PROCESS_FILTER             (0x01Lu)

/* Updates baselines using current raw count values for the widget/sensor */
#define UL_testing_Tuning_PROCESS_BASELINE           (0x02Lu)

/* Calculates differences for the widget/sensor */
#define UL_testing_Tuning_PROCESS_DIFFCOUNTS         (0x04Lu)

/* Runs the noise envelope filter to measure noise magnitude for the widget/sensor */
#define UL_testing_Tuning_PROCESS_CALC_NOISE         (0x08Lu)

/* Updates widget thresholds based on raw counts noise magnitude */
#define UL_testing_Tuning_PROCESS_THRESHOLDS         (0x10Lu)

/* Runs the widget-specific processing algorithms and updates it status */
#define UL_testing_Tuning_PROCESS_STATUS             (0x20Lu)

/* Runs the deconvolution algorithm for the widgets with the multiphase TX scanning */
#define UL_testing_Tuning_PROCESS_DECONVOLUTION      (0x40Lu)

/* Definition that combines all possible processing tasks */
#define UL_testing_Tuning_PROCESS_ALL    (UL_testing_Tuning_PROCESS_FILTER        | \
                                         UL_testing_Tuning_PROCESS_BASELINE      | \
                                         UL_testing_Tuning_PROCESS_DIFFCOUNTS    | \
                                         UL_testing_Tuning_PROCESS_CALC_NOISE    | \
                                         UL_testing_Tuning_PROCESS_THRESHOLDS    | \
                                         UL_testing_Tuning_PROCESS_DECONVOLUTION | \
                                         UL_testing_Tuning_PROCESS_STATUS )

/*******************************************************************************
* Function Prototypes - internal functions.
*******************************************************************************/
/**
* \cond SECTION_C_INTERNAL
* \addtogroup group_c_internal
* \{
*/

void UL_testing_Tuning_DpInitialize(void);

#if ((0u != UL_testing_Tuning_BUTTON_WIDGET_EN) || (0u != UL_testing_Tuning_CSX_MATRIX_WIDGET_EN))
    uint32 UL_testing_Tuning_DpProcessButton(uint32 currStatus, UL_testing_Tuning_FLASH_WD_STRUCT const *ptrFlashWdgt);
#endif /* #if ((0u != UL_testing_Tuning_BUTTON_WIDGET_EN) || (0u != UL_testing_Tuning_CSX_MATRIX_WIDGET_EN)) */

#if (0u != UL_testing_Tuning_CSX_TOUCHPAD_WIDGET_EN)
    uint32 UL_testing_Tuning_DpProcessCsxTouchpad(
                uint32 currStatus, UL_testing_Tuning_FLASH_WD_STRUCT const *ptrFlashWdgt);
#endif /* #if (0u != UL_testing_Tuning_CSX_TOUCHPAD_WIDGET_EN) */

#if (0u != UL_testing_Tuning_PROXIMITY_WIDGET_EN)
    uint32 UL_testing_Tuning_DpProcessProximity(uint32 currStatus, UL_testing_Tuning_FLASH_WD_STRUCT const *ptrFlashWdgt);
#endif /* #if (0u != UL_testing_Tuning_PROXIMITY_WIDGET_EN) */

#if (0u != UL_testing_Tuning_ENCODERDIAL_WIDGET_EN)
    uint32 UL_testing_Tuning_DpProcessEncoderDial(uint32 currStatus, UL_testing_Tuning_FLASH_WD_STRUCT const *ptrFlashWdgt);
#endif /* #if (0u != UL_testing_Tuning_PROXIMITY_WIDGET_EN) */

#if (0u != UL_testing_Tuning_SLIDER_WIDGET_EN)
    uint32 UL_testing_Tuning_DpProcessSlider(uint32 currStatus, UL_testing_Tuning_FLASH_WD_STRUCT const *ptrFlashWdgt);
#endif /* #if (0u != UL_testing_Tuning_SLIDER_WIDGET_EN) */

#if (0u != UL_testing_Tuning_CSD_MATRIX_WIDGET_EN)
    uint32 UL_testing_Tuning_DpProcessCsdMatrix(uint32 currStatus, UL_testing_Tuning_FLASH_WD_STRUCT const *ptrFlashWdgt);
#endif /* #if (0u != UL_testing_Tuning_CSD_MATRIX_WIDGET_EN) */

#if (0u != UL_testing_Tuning_CSD_TOUCHPAD_WIDGET_EN)
    uint32 UL_testing_Tuning_DpProcessCsdTouchpad(
                uint32 currStatus, UL_testing_Tuning_FLASH_WD_STRUCT const *ptrFlashWdgt);
#endif /* #if (0u != UL_testing_Tuning_CSD_TOUCHPAD_WIDGET_EN) */

#if (0u != UL_testing_Tuning_TOTAL_CSD_WIDGETS)
    uint32 UL_testing_Tuning_DpProcessCsdWidgetRawCounts(
                    UL_testing_Tuning_FLASH_WD_STRUCT const *ptrFlashWdgt);

    uint32 UL_testing_Tuning_DpProcessCsdSensorRawCountsExt(
                    UL_testing_Tuning_FLASH_WD_STRUCT const *ptrFlashWdgt,
                    UL_testing_Tuning_RAM_SNS_STRUCT *curSnsPtr,
                    UL_testing_Tuning_PTR_FILTER_VARIANT fltrHistV,
                     uint32 mode);

    void UL_testing_Tuning_DpProcessCsdWidgetStatus(uint32 widgetId, UL_testing_Tuning_FLASH_WD_STRUCT const *ptrFlashWdgt);
#endif /* #if (0u != UL_testing_Tuning_TOTAL_CSD_WIDGETS) */

#if (0u != UL_testing_Tuning_TOTAL_CSX_WIDGETS)
    uint32 UL_testing_Tuning_DpProcessCsxWidgetRawCounts(
            UL_testing_Tuning_FLASH_WD_STRUCT const *ptrFlashWdgt);

    uint32 UL_testing_Tuning_DpProcessCsxSensorRawCountsExt(
                    UL_testing_Tuning_FLASH_WD_STRUCT const *ptrFlashWdgt,
                    UL_testing_Tuning_RAM_SNS_STRUCT *curSnsPtr,
                    UL_testing_Tuning_PTR_FILTER_VARIANT fltrHistV,
                     uint32 mode);

    void UL_testing_Tuning_DpProcessCsxWidgetStatus(
                uint32 widgetId, UL_testing_Tuning_FLASH_WD_STRUCT const *ptrFlashWdgt);
#endif /* #if (0u != UL_testing_Tuning_TOTAL_CSX_WIDGETS) */

#if (0u != UL_testing_Tuning_TOTAL_ISX_WIDGETS)
    uint32 UL_testing_Tuning_DpProcessIsxWidgetRawCounts(
            UL_testing_Tuning_FLASH_WD_STRUCT const *ptrFlashWdgt);

    uint32 UL_testing_Tuning_DpProcessIsxSensorRawCountsExt(
                    UL_testing_Tuning_FLASH_WD_STRUCT const *ptrFlashWdgt, 
                    UL_testing_Tuning_RAM_SNS_STRUCT *ptrSns,
                    UL_testing_Tuning_PTR_FILTER_VARIANT fltrHistV, 
                     uint32 mode);
    
    void UL_testing_Tuning_DpProcessIsxWidgetStatus(
                uint32 wdgtId, UL_testing_Tuning_FLASH_WD_STRUCT const *ptrFlashWdgt);
#endif /* #if (0u != UL_testing_Tuning_TOTAL_ISX_WIDGETS) */

void UL_testing_Tuning_DpUpdateDifferences(
        UL_testing_Tuning_RAM_WD_BASE_STRUCT  *ptrRamWdgt, UL_testing_Tuning_RAM_SNS_STRUCT *curSnsPtr);

#if (0u != (UL_testing_Tuning_CSD_AUTOTUNE & UL_testing_Tuning_CSD_SS_TH_EN))
    void UL_testing_Tuning_DpUpdateThresholds(
                    UL_testing_Tuning_RAM_WD_BASE_STRUCT *ptrRamWdgt,
                    SMARTSENSE_CSD_NOISE_ENVELOPE_STRUCT *ptrNoiseEnvelope,
                    uint32 startFlag);
#endif /* #if (0u != (UL_testing_Tuning_CSD_AUTOTUNE & UL_testing_Tuning_CSD_SS_TH_EN)) */

#if (0u != UL_testing_Tuning_CSX_MULTIPHASE_TX_EN)
    void UL_testing_Tuning_DpDeconvolution(UL_testing_Tuning_FLASH_WD_STRUCT const *ptrFlashWdgt);
#endif /* #if (0u != UL_testing_Tuning_CSX_MULTIPHASE_TX_EN) */


/** \}
* \endcond */

/*******************************************************************************
* API Constants
*******************************************************************************/

#if (0u != UL_testing_Tuning_ENCODERDIAL_WIDGET_EN)
    #define UL_testing_Tuning_DIALTRANSITION_CW0      (0x1u)
    #define UL_testing_Tuning_DIALTRANSITION_CW1      (0x7u)
    #define UL_testing_Tuning_DIALTRANSITION_CW2      (0x8u)
    #define UL_testing_Tuning_DIALTRANSITION_CW3      (0xEu)
    #define UL_testing_Tuning_DIALTRANSITION_CCW0     (0x2u)
    #define UL_testing_Tuning_DIALTRANSITION_CCW1     (0x4u)
    #define UL_testing_Tuning_DIALTRANSITION_CCW2     (0xBu)
    #define UL_testing_Tuning_DIALTRANSITION_CCW3     (0xDu)
#endif /* (0u != UL_testing_Tuning_ENCODERDIAL_WIDGET_EN) */

#endif /* End CY_SENSE_UL_testing_Tuning_DATA_PROCESS_H */


/* [] END OF FILE */
