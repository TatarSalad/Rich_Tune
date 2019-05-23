/***************************************************************************//**
* \file UL_testing_Tuning_Filter.c
* \version 6.0
*
* \brief
*   This file contains the implementation source code to implement all
*   firmware filters.
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

#include "UL_testing_Tuning_Filter.h"
#include "UL_testing_Tuning_Configuration.h"

#if (0 != (UL_testing_Tuning_CSD_AUTOTUNE & UL_testing_Tuning_CSD_SS_TH_EN))
    #include "UL_testing_Tuning_SmartSense_LL.h"
#endif /* (0 != (UL_testing_Tuning_CSD_AUTOTUNE & UL_testing_Tuning_CSD_SS_TH_EN)) */

#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_SELF_TEST_EN)
    #include "UL_testing_Tuning_SelfTest.h"
#endif

/*******************************************************************************
* Function Name: UL_testing_Tuning_FtInitialize
****************************************************************************//**
*
* \brief
*  Initializes all the firmware filter history, except the baseline filter.
*
* \details
*  Initializes all the firmware filter history, except the baseline filter.
*
*******************************************************************************/
void UL_testing_Tuning_FtInitialize(void)
{
    #if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_RC_FILTER_EN) || \
         (0u != (UL_testing_Tuning_CSD_AUTOTUNE & UL_testing_Tuning_CSD_SS_TH_EN)))
        UL_testing_Tuning_InitializeAllFilters();
    #endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_RC_FILTER_EN) || \
               (0u != (UL_testing_Tuning_CSD_AUTOTUNE & UL_testing_Tuning_CSD_SS_TH_EN))) */
}


/*******************************************************************************
* Function Name: UL_testing_Tuning_UpdateAllBaselines
****************************************************************************//**
*
* \brief
*  Updates the baseline for all the sensors in all the widgets.
*
* \details
*  Updates the baseline for all the sensors in all the widgets. Baseline updating is a
*  part of data processing performed by the process functions. So, no need to
*  call this function except a specific process flow is implemented.
*
*  This function ignores the value of the wdgtEnable register. Multiple calling
*  of this function (or any other function with a baseline updating task)
*  without scanning leads to unexpected behavior.
*
*  If the Self-test library is enabled, this function executes the baseline duplication
*  test. Refer to UL_testing_Tuning_CheckBaselineDuplication() for details.
*
* \return
*  Returns the status of the update baseline operation of all the widgets:
*  - CYRET_SUCCESS - The operation is successfully completed.
*  - CYRET_BAD_DATA - The baseline processing failed.
*
*******************************************************************************/
cystatus UL_testing_Tuning_UpdateAllBaselines(void)
{
    uint32 widgetId;
    cystatus bslnStatus = CYRET_SUCCESS;

    for(widgetId = UL_testing_Tuning_TOTAL_WIDGETS; widgetId-- > 0u;)
    {
        bslnStatus |= UL_testing_Tuning_UpdateWidgetBaseline(widgetId);
    }

    return(bslnStatus);
}


/*******************************************************************************
* Function Name: UL_testing_Tuning_UpdateWidgetBaseline
****************************************************************************//**
*
* \brief
*  Updates the baselines for all the sensors in a widget specified by the input
*  parameter.
*
* \details
*  This function performs exactly the same tasks as
*  UL_testing_Tuning_UpdateAllBaselines() but only for a specified widget.
*
*  This function ignores the value of the wdgtEnable register. Multiple calling
*  of this function (or any other function with a baseline updating task)
*  without scanning leads to unexpected behavior.
*
*  If the Self-test library is enabled, this function executes the baseline duplication
*  test. Refer to UL_testing_Tuning_CheckBaselineDuplication() for details.
*
* \param widgetId
*  Specifies the ID number of the widget to update the baseline of all the sensors
*  in the widget.
*  A macro for the widget ID can be found in the
*  UL_testing_Tuning Configuration header file defined as
*  UL_testing_Tuning_<WidgetName>_WDGT_ID.
*
* \return
*  Returns the status of the specified widget update baseline operation:
*  - CYRET_SUCCESS - The operation is successfully completed.
*  - CYRET_BAD_DATA - The baseline processing is failed.
*
*******************************************************************************/
cystatus UL_testing_Tuning_UpdateWidgetBaseline(uint32 widgetId)
{
    uint32 sensorId;
    uint32 sensorsNumber;

    UL_testing_Tuning_FLASH_WD_STRUCT const *ptrWidget = &UL_testing_Tuning_dsFlash.wdgtArray[widgetId];
    cystatus bslnStatus = CYRET_SUCCESS;

    /* Find total amount of sensors in specified widget */
    sensorsNumber = UL_testing_Tuning_GET_SNS_CNT_BY_PTR(ptrWidget);

    for(sensorId = sensorsNumber; sensorId-- > 0u;)
    {
        bslnStatus |= UL_testing_Tuning_UpdateSensorBaseline(widgetId, sensorId);
    }

    return(bslnStatus);
}


/*******************************************************************************
* Function Name: UL_testing_Tuning_UpdateSensorBaseline
****************************************************************************//**
*
* \brief
*  Updates the baseline for a sensor in a widget specified by the input parameters.
*
* \details
*  This function performs exactly the same tasks as
*  UL_testing_Tuning_UpdateAllBaselines() and
*  UL_testing_Tuning_UpdateWidgetBaseline() but only for a specified sensor.
*
*  This function ignores the value of the wdgtEnable register. Multiple calling
*  of this function (or any other function with a baseline updating task)
*  without scanning leads to unexpected behavior.
*
*  If the Self-test library is enabled, this function executes the baseline duplication
*  test. Refer to UL_testing_Tuning_CheckBaselineDuplication() for details.
*
* \param widgetId
*  Specifies the ID number of the widget to update the baseline of the sensor
*  specified by the sensorId argument.
*  A macro for the widget ID can be found in the UL_testing_Tuning Configuration header
*  file defined as UL_testing_Tuning_<WidgetName>_WDGT_ID.
*
* \param sensorId
*  Specifies the ID number of the sensor within the widget to update its baseline.
*  A macro for the sensor ID within a specified widget can be found in the
*  UL_testing_Tuning Configuration header file defined as
*  UL_testing_Tuning_<WidgetName>_SNS<SensorNumber>_ID.
*
* \return
*  Returns the status of the specified sensor update baseline operation:
*  - CYRET_SUCCESS - The operation is successfully completed.
*  - CYRET_BAD_DATA - The baseline processing failed.
*
*******************************************************************************/
cystatus UL_testing_Tuning_UpdateSensorBaseline(uint32 widgetId, uint32 sensorId)
{
    uint32 result = CYRET_SUCCESS;

    UL_testing_Tuning_RAM_SNS_STRUCT *ptrSensor = NULL;
    UL_testing_Tuning_FLASH_WD_STRUCT const *ptrWidget = &UL_testing_Tuning_dsFlash.wdgtArray[widgetId];
    UL_testing_Tuning_RAM_WD_BASE_STRUCT *ptrWidgetRam = (UL_testing_Tuning_RAM_WD_BASE_STRUCT *)ptrWidget->ptr2WdgtRam;

    /* Find pointer to specified sensor object */
    ptrSensor = ptrWidget->ptr2SnsRam;
    ptrSensor = &ptrSensor[sensorId];
    result = UL_testing_Tuning_FtUpdateBaseline(ptrWidgetRam, ptrSensor, (uint32)ptrWidget->wdgtType);

    #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_BSLN_DUPLICATION_EN)
        if (CYRET_SUCCESS != result)
        {
            result = CYRET_BAD_DATA;
            UL_testing_Tuning_UpdateTestResultBaselineDuplication(widgetId, sensorId);
        }
    #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_BSLN_DUPLICATION_EN) */

    return result;
}

/*******************************************************************************
* Function Name: UL_testing_Tuning_FtUpdateBaseline
****************************************************************************//**
*
* \brief
*  Updates the baseline for a sensor specified by an input parameter.
*
* \details
*  Check a matching of present baseline and its inverse duplication. If they
*  match then updates the baseline for a sensor specified by an input parameter.
*  If don't match the function return UL_testing_Tuning_TST_BSLN_DUPLICATION
*  result and don't update baseline.
*
* \param ptrWidgetRam
*  The pointer to the widget RAM structure where all the widget parameters
*  are stored.
*
* \param ptrSensor
*  The pointer to the sensor RAM structure where all the sensor parameters
*  are stored.
*
* \param wdType
*  Specifies the type of a widget.
*
* \return
*  Returns a status indicating whether the baseline has been updated:
*  - CYRET_SUCCESS if baseline updating was successful.
*  - UL_testing_Tuning_PROCESS_BASELINE_FAILED if present sensor's any channel
*    baseline and its inversion doesn't matched.
*
*******************************************************************************/
uint32 UL_testing_Tuning_FtUpdateBaseline(
                                UL_testing_Tuning_RAM_WD_BASE_STRUCT *ptrWidgetRam,
                                UL_testing_Tuning_RAM_SNS_STRUCT *ptrSensor,
                                uint32 wdType)
{
    uint32 sign;
    uint32 difference;
    uint32 freqChannel;
    uint32 baselineCoeff;
    uint32 result = CYRET_SUCCESS;

    #if (UL_testing_Tuning_TOTAL_WIDGETS)
        uint32 history;
    #endif /* (UL_testing_Tuning_TOTAL_WIDGETS) */

    /* Apply baseline to every channel in sensor */
    for(freqChannel = UL_testing_Tuning_NUM_SCAN_FREQS; freqChannel-- > 0u;)
    {
        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_BSLN_DUPLICATION_EN)
            if (ptrSensor->bslnInv[freqChannel] != ((uint16) ~(ptrSensor->bsln[freqChannel])))
            {
                result = UL_testing_Tuning_PROCESS_BASELINE_FAILED;
            }
            else
            {
        #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_BSLN_DUPLICATION_EN) */

        /* Calculate signal value and its sign */
        if(ptrSensor->raw[freqChannel] >= ptrSensor->bsln[freqChannel])
        {
            difference = (uint32)ptrSensor->raw[freqChannel] - (uint32)ptrSensor->bsln[freqChannel];
            sign = 1u;
            ptrSensor->negBslnRstCnt[freqChannel] = 0u;
        }
        else
        {
            difference = (uint32)ptrSensor->bsln[freqChannel] - (uint32)ptrSensor->raw[freqChannel];
            sign = 0u;
        }

        /* Reset baseline if condition is met */
        if((sign == 0u) && (difference > (uint32) ptrWidgetRam->nNoiseTh))
        {
            if(ptrSensor->negBslnRstCnt[freqChannel] >= ptrWidgetRam->lowBslnRst)
            {
                UL_testing_Tuning_FtInitializeBaselineChannel(ptrSensor, wdType, freqChannel);
            }
            else
            {
                ptrSensor->negBslnRstCnt[freqChannel]++;
            }
        }
        else
        {
            #if (!UL_testing_Tuning_SENSOR_AUTO_RESET_EN)
                /* Update baseline only if signal is in range between noiseThreshold and negativenoiseThreshold */
                if ((difference <= (uint32)ptrWidgetRam->noiseTh) ||
                    ((difference < (uint32) ptrWidgetRam->nNoiseTh) && (sign == 0u)))
                {
            #endif /* (UL_testing_Tuning_CSD_AUTO_RESET == UL_testing_Tuning_CSD_AUTO_RESET_DISABLE) */

            #if (UL_testing_Tuning_BASELINE_TYPE == UL_testing_Tuning_IIR_BASELINE)
                /* Calculate baseline value */
                if ((UL_testing_Tuning_WD_TYPE_ENUM)wdType != UL_testing_Tuning_WD_PROXIMITY_E)
                {
                    #if (UL_testing_Tuning_REGULAR_SENSOR_EN)
                        #if (UL_testing_Tuning_REGULAR_IIR_BL_TYPE == UL_testing_Tuning_IIR_FILTER_PERFORMANCE)
                            history = (uint32) ptrSensor->bsln[freqChannel] << 8u;
                            history |= ptrSensor->bslnExt[freqChannel];
                        #else
                            history = ptrSensor->bsln[freqChannel];
                        #endif /* (UL_testing_Tuning_REGULAR_IIR_BL_TYPE == UL_testing_Tuning_IIR_FILTER_PERFORMANCE) */

                        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_WD_BSLN_COEFF_EN)
                            baselineCoeff = ptrWidgetRam->bslnCoeff;
                        #else /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_WD_BSLN_COEFF_EN) */
                            baselineCoeff = UL_testing_Tuning_REGULAR_IIR_BL_N;
                        #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_WD_BSLN_COEFF_EN) */

                        history =  UL_testing_Tuning_FtIIR1stOrder(
                                        (uint32)ptrSensor->raw[freqChannel],
                                        history, baselineCoeff,
                                        UL_testing_Tuning_REGULAR_IIR_BL_SHIFT);

                        #if (UL_testing_Tuning_REGULAR_IIR_BL_TYPE == UL_testing_Tuning_IIR_FILTER_PERFORMANCE)
                            ptrSensor->bsln[freqChannel] = LO16(history >> 8u);
                            ptrSensor->bslnExt[freqChannel] = LO8(history);
                        #else
                            ptrSensor->bsln[freqChannel] = LO16(history);
                        #endif /* (UL_testing_Tuning_REGULAR_IIR_BL_TYPE == UL_testing_Tuning_IIR_FILTER_PERFORMANCE) */
                    #endif /* (UL_testing_Tuning_REGULAR_SENSOR_EN) */
                }
                else
                {
                    #if (UL_testing_Tuning_PROXIMITY_SENSOR_EN)
                        #if (UL_testing_Tuning_PROX_IIR_BL_TYPE == UL_testing_Tuning_IIR_FILTER_PERFORMANCE)
                            history = (uint32) ptrSensor->bsln[freqChannel] << 8u;
                            history |= ptrSensor->bslnExt[freqChannel];
                        #else
                            history = (uint32) ptrSensor->bsln[freqChannel];
                        #endif /* (UL_testing_Tuning_PROX_IIR_BL_TYPE == UL_testing_Tuning_IIR_FILTER_PERFORMANCE) */

                        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_WD_BSLN_COEFF_EN)
                            baselineCoeff = ptrWidgetRam->bslnCoeff;
                        #else /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_WD_BSLN_COEFF_EN) */
                            baselineCoeff = UL_testing_Tuning_PROX_IIR_BL_N;
                        #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_WD_BSLN_COEFF_EN) */

                        history = UL_testing_Tuning_FtIIR1stOrder(
                                        (uint32)ptrSensor->raw[freqChannel],
                                        history, baselineCoeff,
                                        UL_testing_Tuning_PROX_IIR_BL_SHIFT);

                        #if (UL_testing_Tuning_PROX_IIR_BL_TYPE == UL_testing_Tuning_IIR_FILTER_PERFORMANCE)
                            ptrSensor->bsln[freqChannel] = LO16(history >> 8u);
                            ptrSensor->bslnExt[freqChannel] = LO8(history);
                        #else
                            ptrSensor->bsln[freqChannel] = LO16(history);
                        #endif /* (UL_testing_Tuning_PROX_IIR_BL_TYPE == UL_testing_Tuning_IIR_FILTER_PERFORMANCE) */
                    #endif /* (UL_testing_Tuning_PROX_SENSOR_EN) */
                }
            #else /* (UL_testing_Tuning_CSD_BASELINE_TYPE == UL_testing_Tuning_IIR_BASELINE) */

                /******************************************************************
                * This is the place where the bucket algorithm should be implemented.
                * The bucket method will be implemented in future Component version.
                *******************************************************************/

            #endif /* (UL_testing_Tuning_CSD_BASELINE_TYPE == UL_testing_Tuning_IIR_BASELINE) */

            #if (!UL_testing_Tuning_SENSOR_AUTO_RESET_EN)
                }
            #endif /* (!UL_testing_Tuning_SENSOR_AUTO_RESET_EN) */
        }

        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_BSLN_DUPLICATION_EN)
            /* Update baseline inversion of every channel in sensor */
            ptrSensor->bslnInv[freqChannel] = ~(ptrSensor->bsln[freqChannel]);
            }
        #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_BSLN_DUPLICATION_EN) */

    }
    return result;
}


/*******************************************************************************
* Function Name: UL_testing_Tuning_InitializeAllBaselines
****************************************************************************//**
*
* \brief
*  Initializes (or re-initializes) the baselines of all the sensors of all the widgets.
*
* \details
*  Initializes the baseline for all the sensors of all the widgets. Also, this function
*  can be used to re-initialize baselines. UL_testing_Tuning_Start() calls this
*  API as part of UL_testing_Tuning operation initialization.
*
*  If any raw count filter is enabled, make sure the raw count filter history is
*  initialized as well using one of these functions:
*  - UL_testing_Tuning_InitializeAllFilters().
*  - UL_testing_Tuning_InitializeWidgetFilter().
*
*******************************************************************************/
void UL_testing_Tuning_InitializeAllBaselines(void)
{
    uint32 widgetId;

    for(widgetId = UL_testing_Tuning_TOTAL_WIDGETS; widgetId-- > 0u;)
    {
        UL_testing_Tuning_InitializeWidgetBaseline(widgetId);
    }
}


/*******************************************************************************
* Function Name: UL_testing_Tuning_InitializeWidgetBaseline
****************************************************************************//**
*
* \brief
*  Initializes (or re-initializes) the baselines of all the sensors in a widget
*  specified by the input parameter.
*
* \details
*  Initializes (or re-initializes) the baseline for all the sensors of the
*  specified widget.
*
*  If any raw count filter is enabled, make sure the raw count filter history is
*  initialized as well using one of these functions:
*  - UL_testing_Tuning_InitializeAllFilters().
*  - UL_testing_Tuning_InitializeWidgetFilter().
*
* \param widgetId
*  Specifies the ID number of a widget to initialize the baseline of all the sensors
*  in the widget.
*  A macro for the widget ID can be found in the
*  UL_testing_Tuning Configuration header file defined as
*  UL_testing_Tuning_<WidgetName>_WDGT_ID.
*
*******************************************************************************/
void UL_testing_Tuning_InitializeWidgetBaseline(uint32 widgetId)
{
    uint32 sensorId;
    uint32 sensorsNumber;

    UL_testing_Tuning_FLASH_WD_STRUCT const *ptrWidget = &UL_testing_Tuning_dsFlash.wdgtArray[widgetId];

    /* Find total amount of sensors in specified widget */
    sensorsNumber = UL_testing_Tuning_GET_SNS_CNT_BY_PTR(ptrWidget);

    for(sensorId = sensorsNumber; sensorId-- > 0u;)
    {
        UL_testing_Tuning_InitializeSensorBaseline(widgetId, sensorId);
    }
}


/*******************************************************************************
* Function Name: UL_testing_Tuning_InitializeSensorBaseline
****************************************************************************//**
*
* \brief
*  Initializes (or re-initializes) the baseline of a sensor in a widget specified
*  by the input parameters.
*
* \details
*  Initializes (or re-initializes) the baseline for a specified sensor within
*  a specified widget.
*
* \param widgetId
*  Specifies the ID number of a widget to initialize the baseline of the sensor
*  in the widget.
*  A macro for the widget ID can be found in the
*  UL_testing_Tuning Configuration header file defined as
*  UL_testing_Tuning_<WidgetName>_WDGT_ID.
*
* \param sensorId
*  Specifies the ID number of the sensor within the widget to initialize its
*  baseline.
*  A macro for the sensor ID within a specified widget can be found in the
*  UL_testing_Tuning Configuration header file defined as
*  UL_testing_Tuning_<WidgetName>_SNS<SensorNumber>_ID.
*
*******************************************************************************/
void UL_testing_Tuning_InitializeSensorBaseline(uint32 widgetId, uint32 sensorId)
{
    UL_testing_Tuning_RAM_SNS_STRUCT *ptrSensor = NULL;
    UL_testing_Tuning_FLASH_WD_STRUCT const *ptrWidget = NULL;
    ptrWidget = &UL_testing_Tuning_dsFlash.wdgtArray[widgetId];

    /* Find pointer to specified sensor object */
    ptrSensor = ptrWidget->ptr2SnsRam;
    ptrSensor = &ptrSensor[sensorId];

    UL_testing_Tuning_FtInitializeBaseline(ptrSensor, (uint32)ptrWidget->wdgtType);
}


/*******************************************************************************
* Function Name: UL_testing_Tuning_FtInitializeBaseline
****************************************************************************//**
*
* \brief
*  Initializes the baseline history for a sensor indicated by an input
*  parameter.
*
* \details
*  Initializes the baseline history for a sensor indicated by an input
*  parameter.
*
* \param *ptrSensor
*  The pointer to the sensor RAM object.
*
* \param wdType
*  Specifies the type of a widget.
*
*******************************************************************************/
void UL_testing_Tuning_FtInitializeBaseline(UL_testing_Tuning_RAM_SNS_STRUCT *ptrSensor, uint32 wdType)
{
    uint32 freqChannel;

    /* Apply baseline initialization to every channel in sensor */
    for(freqChannel = UL_testing_Tuning_NUM_SCAN_FREQS; freqChannel-- > 0u;)
    {
        UL_testing_Tuning_FtInitializeBaselineChannel(ptrSensor, wdType, freqChannel);
    }
}


/*******************************************************************************
* Function Name: UL_testing_Tuning_FtInitializeBaselineChannel
****************************************************************************//**
*
* \brief
*  Initializes the baseline history for a sensor indicated by an input
*  parameter.
*
* \details
*  Initializes the baseline history for a sensor indicated by an input
*  parameter.
*
* \param *ptrSensor
*  The pointer to the sensor RAM object.
*
* \param wdType
*  Specifies the type of a widget.
*
* \param channel
*  Specifies the number of the channel to be initialized.
*
*******************************************************************************/
void UL_testing_Tuning_FtInitializeBaselineChannel(UL_testing_Tuning_RAM_SNS_STRUCT *ptrSensor, uint32 wdType, uint32 channel)
{
    #if (UL_testing_Tuning_BASELINE_TYPE == UL_testing_Tuning_IIR_BASELINE)
        if ((UL_testing_Tuning_WD_TYPE_ENUM)wdType != UL_testing_Tuning_WD_PROXIMITY_E)
        {
            #if (UL_testing_Tuning_REGULAR_SENSOR_EN)
                #if (UL_testing_Tuning_REGULAR_IIR_BL_TYPE == UL_testing_Tuning_IIR_FILTER_PERFORMANCE)
                    ptrSensor->bslnExt[channel] = 0u;
                #endif /* (UL_testing_Tuning_REGULAR_IIR_BL_TYPE == UL_testing_Tuning_IIR_FILTER_PERFORMANCE) */
            #endif /* (UL_testing_Tuning_REGULAR_SENSOR_EN) */
        }
        else
        {
            #if (UL_testing_Tuning_PROXIMITY_SENSOR_EN)
                #if (UL_testing_Tuning_PROX_IIR_BL_TYPE == UL_testing_Tuning_IIR_FILTER_PERFORMANCE)
                    ptrSensor->bslnExt[channel] = 0u;
                #endif /* (UL_testing_Tuning_PROX_IIR_BL_TYPE == UL_testing_Tuning_IIR_FILTER_PERFORMANCE) */
            #endif /* (UL_testing_Tuning_PROX_SENSOR_EN) */
        }
    #else
        /* UL_testing_Tuning_BASELINE_TYPE == UL_testing_Tuning_BUCKET_BASELINE */
        ptrSensor->bslnExt[channel] = 0u;
    #endif /* (UL_testing_Tuning_BASELINE_TYPE == UL_testing_Tuning_IIR_BASELINE) */

    ptrSensor->bsln[channel] = ptrSensor->raw[channel];

    #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_BSLN_DUPLICATION_EN)
    /* Update baseline inversion of the channel in sensor */
        ptrSensor->bslnInv[channel] = ~(ptrSensor->bsln[channel]);
    #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_BSLN_DUPLICATION_EN) */

    ptrSensor->negBslnRstCnt[channel] = 0u;
}

#if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_RC_FILTER_EN) || \
     (0u != (UL_testing_Tuning_CSD_AUTOTUNE & UL_testing_Tuning_CSD_SS_TH_EN)))
/*******************************************************************************
* Function Name: UL_testing_Tuning_InitializeAllFilters
****************************************************************************//**
*
* \brief
*  Initializes (or re-initializes) the raw count filter history of all the
*  sensors of all the widgets.
*
* \details
*  Initializes the raw count filter history for all the sensors of all the
*  widgets. Also, this function can be used to re-initialize baselines.
*  UL_testing_Tuning_Start() calls this API as part of UL_testing_Tuning
*  operation initialization.
*
*******************************************************************************/
void UL_testing_Tuning_InitializeAllFilters(void)
{
    uint32 widgetId;

    for(widgetId = UL_testing_Tuning_TOTAL_WIDGETS; widgetId-- > 0u;)
    {
        UL_testing_Tuning_InitializeWidgetFilter(widgetId);
    }
}

/*******************************************************************************
* Function Name: UL_testing_Tuning_InitializeWidgetFilter
****************************************************************************//**
*
* \brief
*  Initializes (or re-initializes) the raw count filter history of all the sensors
*  in a widget specified by the input parameter.
*
* \details
*  Initializes (or re-initializes) the raw count filter history of all the sensors
*  in a widget specified by the input parameter.
*
* \param widgetId
*  Specifies the ID number of a widget to initialize the filter history of all
*  the sensors in the widget.
*  A macro for the widget ID can be found in the
*  UL_testing_Tuning Configuration header file defined as
*  UL_testing_Tuning_<WidgetName>_WDGT_ID.
*
*******************************************************************************/
void UL_testing_Tuning_InitializeWidgetFilter(uint32 widgetId)
{
    uint32 sensorId;
    uint32 sensorsNumber;

    UL_testing_Tuning_FLASH_WD_STRUCT const *ptrWidget;
    UL_testing_Tuning_RAM_SNS_STRUCT *ptrSensor = NULL;
    #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_RC_FILTER_EN)
        UL_testing_Tuning_PTR_FILTER_VARIANT ptrFilterHistObj;
    #endif

    #if (0u != (UL_testing_Tuning_CSD_AUTOTUNE & UL_testing_Tuning_CSD_SS_TH_EN))
        UL_testing_Tuning_RAM_WD_BASE_STRUCT *ptrWidgetRam;
        SMARTSENSE_CSD_NOISE_ENVELOPE_STRUCT *ptrNoiseEnvelope = NULL;
    #endif /* (0u != (UL_testing_Tuning_CSD_AUTOTUNE & UL_testing_Tuning_CSD_SS_TH_EN)) */

    ptrWidget = &UL_testing_Tuning_dsFlash.wdgtArray[widgetId];

    #if (0u != (UL_testing_Tuning_CSD_AUTOTUNE & UL_testing_Tuning_CSD_SS_TH_EN))
        ptrWidgetRam = ptrWidget->ptr2WdgtRam;
    #endif /* (0u != (UL_testing_Tuning_CSD_AUTOTUNE & UL_testing_Tuning_CSD_SS_TH_EN)) */

    /* Find total amount of sensors in specified widget */
    sensorsNumber = UL_testing_Tuning_GET_SNS_CNT_BY_PTR(ptrWidget);

    for (sensorId = sensorsNumber; sensorId-- > 0u;)
    {
        /* Find pointer to specified sensor object */
        ptrSensor = ptrWidget->ptr2SnsRam;
        ptrSensor = &ptrSensor[sensorId];

        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_RC_FILTER_EN)
            /* Find pointer to to specified filter sensor object */
            ptrFilterHistObj.ptr = ptrWidget->ptr2FltrHistory;
        #endif

        #if (0u != (UL_testing_Tuning_CSD_AUTOTUNE & UL_testing_Tuning_CSD_SS_TH_EN))
            /* Find pointer to specified noise envelope sensor object */
            ptrNoiseEnvelope = ptrWidget->ptr2NoiseEnvlp;
            ptrNoiseEnvelope = &ptrNoiseEnvelope[sensorId];
        #endif /* (0u != (UL_testing_Tuning_CSD_AUTOTUNE & UL_testing_Tuning_CSD_SS_TH_EN)) */

        if ((UL_testing_Tuning_WD_TYPE_ENUM)ptrWidget->wdgtType != UL_testing_Tuning_WD_PROXIMITY_E)
        {
            #if (0u != UL_testing_Tuning_REGULAR_RC_ALP_FILTER_EN)
                ptrFilterHistObj.ptrAlp = &ptrFilterHistObj.ptrAlp[sensorId];
            #elif (0u != UL_testing_Tuning_REGULAR_RC_FILTER_EN)
                ptrFilterHistObj.ptrRegular = &ptrFilterHistObj.ptrRegular[sensorId];
            #endif
        }
        else
        {
            #if (0u != UL_testing_Tuning_PROX_RC_ALP_FILTER_EN)
                ptrFilterHistObj.ptrAlp = &ptrFilterHistObj.ptrAlp[sensorId];
            #elif (0u != UL_testing_Tuning_PROX_RC_FILTER_EN)
                ptrFilterHistObj.ptrProx = &ptrFilterHistObj.ptrProx[sensorId];
            #endif /* #if (0u != UL_testing_Tuning_PROX_RC_FILTER_EN) */
        }

        #if (UL_testing_Tuning_REGULAR_RC_IIR_FILTER_EN || UL_testing_Tuning_PROX_RC_IIR_FILTER_EN)
            UL_testing_Tuning_InitializeIIRInternal(ptrFilterHistObj, ptrSensor, (uint32)ptrWidget->wdgtType);
        #endif /* (UL_testing_Tuning_REGULAR_RC_IIR_FILTER_EN || UL_testing_Tuning_PROX_RC_IIR_FILTER_EN) */

        #if (UL_testing_Tuning_REGULAR_RC_MEDIAN_FILTER_EN || UL_testing_Tuning_PROX_RC_MEDIAN_FILTER_EN)
            UL_testing_Tuning_InitializeMedianInternal(ptrFilterHistObj, ptrSensor, (uint32)ptrWidget->wdgtType);
        #endif /* (UL_testing_Tuning_REGULAR_RC_MEDIAN_FILTER_EN || UL_testing_Tuning_PROX_RC_MEDIAN_FILTER_EN) */

        #if (UL_testing_Tuning_REGULAR_RC_AVERAGE_FILTER_EN || UL_testing_Tuning_PROX_RC_AVERAGE_FILTER_EN)
            UL_testing_Tuning_InitializeAverageInternal(ptrFilterHistObj, ptrSensor, (uint32)ptrWidget->wdgtType);
        #endif /* (UL_testing_Tuning_REGULAR_RC_AVERAGE_FILTER_EN || UL_testing_Tuning_PROX_RC_AVERAGE_FILTER_EN) */

        #if (UL_testing_Tuning_ALP_FILTER_EN)
            UL_testing_Tuning_InitializeALPInternal(ptrFilterHistObj, ptrSensor, (uint32)ptrWidget->wdgtType);
        #endif

        #if (0u != (UL_testing_Tuning_CSD_AUTOTUNE & UL_testing_Tuning_CSD_SS_TH_EN))
            if (UL_testing_Tuning_SENSE_METHOD_CSD_E ==
                UL_testing_Tuning_GET_SENSE_METHOD(&UL_testing_Tuning_dsFlash.wdgtArray[widgetId]))
            {
                SmartSense_InitializeNoiseEnvelope(ptrSensor->raw[0u], ptrWidgetRam->sigPFC, ptrNoiseEnvelope);
            }
        #endif /* (0u != (UL_testing_Tuning_CSD_AUTOTUNE & UL_testing_Tuning_CSD_SS_TH_EN)) */
    }
}
#endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_RC_FILTER_EN) || \
           (0u != (UL_testing_Tuning_CSD_AUTOTUNE & UL_testing_Tuning_CSD_SS_TH_EN))) */

#if (UL_testing_Tuning_REGULAR_RC_IIR_FILTER_EN || UL_testing_Tuning_PROX_RC_IIR_FILTER_EN)
/*******************************************************************************
* Function Name: UL_testing_Tuning_InitializeIIR
****************************************************************************//**
*
* \brief
*  Initialize the IIR filter history.
*
* \details
*  Initialize the IIR filter history.
*
* \param widgetId
*  Specifies the ID number of the widget.
*
* \param sensorId
*  Specifies the ID number of the sensor in the widget.
*
*******************************************************************************/
void UL_testing_Tuning_InitializeIIR(uint32 widgetId, uint32 sensorId)
{
    UL_testing_Tuning_FLASH_WD_STRUCT const *ptrWidget = &UL_testing_Tuning_dsFlash.wdgtArray[widgetId];
    UL_testing_Tuning_RAM_SNS_STRUCT *ptrSensor = NULL;
    UL_testing_Tuning_PTR_FILTER_VARIANT ptrFilterHistObj;

    /* Find pointer to specified sensor object */
    ptrSensor = ptrWidget->ptr2SnsRam;
    ptrSensor = &ptrSensor[sensorId];

    /* Find pointer to specified filter sensor object */
    ptrFilterHistObj.ptr = ptrWidget->ptr2FltrHistory;

    if ((UL_testing_Tuning_WD_TYPE_ENUM)ptrWidget->wdgtType != UL_testing_Tuning_WD_PROXIMITY_E)
    {
        #if (UL_testing_Tuning_REGULAR_SENSOR_EN)
            ptrFilterHistObj.ptrRegular = &ptrFilterHistObj.ptrRegular[sensorId];
        #endif /* (UL_testing_Tuning_REGULAR_SENSOR_EN) */
    }
    else
    {
        #if (UL_testing_Tuning_PROXIMITY_SENSOR_EN)
            ptrFilterHistObj.ptrProx = &ptrFilterHistObj.ptrProx[sensorId];
        #endif /* (UL_testing_Tuning_PROXIMITY_SENSOR_EN) */
    }

    UL_testing_Tuning_InitializeIIRInternal(ptrFilterHistObj, ptrSensor, (uint32)ptrWidget->wdgtType);
}


/*******************************************************************************
* Function Name: UL_testing_Tuning_RunIIR
****************************************************************************//**
*
* \brief
*  Executes the IIR filter algorithm on a sensor indicated by an input
*  parameter.
*
* \details
*  Executes the IIR filter algorithm on a sensor indicated by an input
*  parameter.
*
* \param widgetId
*  Specifies the ID number of the widget.
*
* \param sensorId
*  Specifies the ID number of the sensor in the widget.
*
*******************************************************************************/
void UL_testing_Tuning_RunIIR(uint32 widgetId, uint32 sensorId)
{
    UL_testing_Tuning_FLASH_WD_STRUCT const *ptrWidget = &UL_testing_Tuning_dsFlash.wdgtArray[widgetId];
    UL_testing_Tuning_RAM_SNS_STRUCT *ptrSensor = NULL;
    UL_testing_Tuning_PTR_FILTER_VARIANT ptrFilterHistObj;

    /* Find pointer to specified sensor object */
    ptrSensor = ptrWidget->ptr2SnsRam;
    ptrSensor = &ptrSensor[sensorId];

    /* Find pointer to specified filter sensor object */
    ptrFilterHistObj.ptr = ptrWidget->ptr2FltrHistory;

    if ((UL_testing_Tuning_WD_TYPE_ENUM)ptrWidget->wdgtType != UL_testing_Tuning_WD_PROXIMITY_E)
    {
        #if (UL_testing_Tuning_REGULAR_SENSOR_EN)
            ptrFilterHistObj.ptrRegular = &ptrFilterHistObj.ptrRegular[sensorId];
        #endif /* (UL_testing_Tuning_REGULAR_SENSOR_EN) */
    }
    else
    {
        #if (UL_testing_Tuning_PROXIMITY_SENSOR_EN)
            ptrFilterHistObj.ptrProx = &ptrFilterHistObj.ptrProx[sensorId];
        #endif /* (UL_testing_Tuning_PROXIMITY_SENSOR_EN) */
    }

    UL_testing_Tuning_RunIIRInternal(ptrFilterHistObj, ptrSensor, (uint32)ptrWidget->wdgtType);
}

/*******************************************************************************
* Function Name: UL_testing_Tuning_InitializeIIRInternal
****************************************************************************//**
*
* \brief
*  Initializes the IIR filter history.
*
* \details
*  Initializes the IIR filter history.
*
* \param ptrFilterHistObj
*  The pointer to the filter RAM object of the sensor.
*
* \param ptrSensorObj
*  The pointer to the sensor RAM object of the sensor.
*
* \param wdType
*  Specifies the type of a widget.
*
*******************************************************************************/
void UL_testing_Tuning_InitializeIIRInternal(UL_testing_Tuning_PTR_FILTER_VARIANT ptrFilterHistObj,
                                            UL_testing_Tuning_RAM_SNS_STRUCT *ptrSensorObj, uint32 wdType)
{
    uint32 freqChannel;

    for(freqChannel = UL_testing_Tuning_NUM_SCAN_FREQS; freqChannel-- > 0u;)
    {
        if ((UL_testing_Tuning_WD_TYPE_ENUM)wdType != UL_testing_Tuning_WD_PROXIMITY_E)
        {
            #if (UL_testing_Tuning_REGULAR_SENSOR_EN && UL_testing_Tuning_REGULAR_RC_IIR_FILTER_EN)
                #if (UL_testing_Tuning_REGULAR_IIR_RC_TYPE == UL_testing_Tuning_IIR_FILTER_PERFORMANCE)
                    ptrFilterHistObj.ptrRegular->regularChannel[freqChannel].iirHistory = ptrSensorObj->raw[freqChannel];
                    ptrFilterHistObj.ptrRegular->regularChannel[freqChannel].iirHistoryLow = 0u;
                #else
                    ptrFilterHistObj.ptrRegular->regularChannel[freqChannel].iirHistory = ptrSensorObj->raw[freqChannel] << UL_testing_Tuning_REGULAR_IIR_RC_SHIFT;
                #endif /* (UL_testing_Tuning_REGULAR_IIR_RC_TYPE == UL_testing_Tuning_IIR_FILTER_PERFORMANCE) */
            #endif /* (UL_testing_Tuning_REGULAR_SENSOR_EN) */
        }
        else
        {
            #if (UL_testing_Tuning_PROXIMITY_SENSOR_EN && UL_testing_Tuning_PROX_RC_IIR_FILTER_EN)
                #if (UL_testing_Tuning_PROX_IIR_RC_TYPE == UL_testing_Tuning_IIR_FILTER_PERFORMANCE)
                    ptrFilterHistObj.ptrProx->proxChannel[freqChannel].iirHistory = ptrSensorObj->raw[freqChannel];
                    ptrFilterHistObj.ptrProx->proxChannel[freqChannel].iirHistoryLow = 0u;
                #else
                    ptrFilterHistObj.ptrProx->proxChannel[freqChannel].iirHistory = ptrSensorObj->raw[freqChannel] << UL_testing_Tuning_PROX_IIR_RC_SHIFT;
                #endif /* (UL_testing_Tuning_PROX_IIR_RC_TYPE == UL_testing_Tuning_IIR_FILTER_PERFORMANCE) */
            #endif /* (UL_testing_Tuning_PROX_SENSOR_EN) */
        }
    }
}


/*******************************************************************************
* Function Name: UL_testing_Tuning_RunIIRInternal
****************************************************************************//**
*
* \brief
*  Run the IIR filter.
*
* \details
*  Run the IIR filter.
*
* \param ptrFilterHistObj
*  The pointer to the filter RAM object of the sensor.
*
* \param ptrSensorObj
*  The pointer to the sensor RAM object of the sensor.
*
* \param wdType
*  Specifies the type of a widget.
*
*******************************************************************************/
void UL_testing_Tuning_RunIIRInternal(UL_testing_Tuning_PTR_FILTER_VARIANT ptrFilterHistObj,
                                     UL_testing_Tuning_RAM_SNS_STRUCT *ptrSensorObj, uint32 wdType)
{
    uint32 freqChannel;
    uint32 temp;

    for(freqChannel = UL_testing_Tuning_NUM_SCAN_FREQS; freqChannel-- > 0u;)
    {
        if ((UL_testing_Tuning_WD_TYPE_ENUM)wdType != UL_testing_Tuning_WD_PROXIMITY_E)
        {
            #if (UL_testing_Tuning_REGULAR_SENSOR_EN && UL_testing_Tuning_REGULAR_RC_IIR_FILTER_EN)
                #if (UL_testing_Tuning_REGULAR_IIR_RC_TYPE == UL_testing_Tuning_IIR_FILTER_PERFORMANCE)
                    temp = ((uint32)ptrFilterHistObj.ptrRegular->regularChannel[freqChannel].iirHistory << UL_testing_Tuning_REGULAR_IIR_RC_SHIFT);
                    temp |= ptrFilterHistObj.ptrRegular->regularChannel[freqChannel].iirHistoryLow;
                    temp = UL_testing_Tuning_FtIIR1stOrder((uint32)ptrSensorObj->raw[freqChannel],
                                                        temp,
                                                        UL_testing_Tuning_REGULAR_IIR_RC_N,
                                                        UL_testing_Tuning_REGULAR_IIR_RC_SHIFT);
                    ptrFilterHistObj.ptrRegular->regularChannel[freqChannel].iirHistory = LO16(temp >>UL_testing_Tuning_REGULAR_IIR_RC_SHIFT);
                    ptrFilterHistObj.ptrRegular->regularChannel[freqChannel].iirHistoryLow = LO8(temp);
                    ptrSensorObj->raw[freqChannel] = LO16(temp >>UL_testing_Tuning_REGULAR_IIR_RC_SHIFT);
                #else
                    temp =UL_testing_Tuning_FtIIR1stOrder((uint32)ptrSensorObj->raw[freqChannel],
                                                        (uint32)ptrFilterHistObj.ptrRegular->regularChannel[freqChannel].iirHistory,
                                                        UL_testing_Tuning_REGULAR_IIR_RC_N,
                                                        UL_testing_Tuning_REGULAR_IIR_RC_SHIFT);
                    ptrFilterHistObj.ptrRegular->regularChannel[freqChannel].iirHistory = LO16(temp);
                    ptrSensorObj->raw[freqChannel] = LO16(temp >>UL_testing_Tuning_REGULAR_IIR_RC_SHIFT);
                #endif /* (UL_testing_Tuning_REGULAR_IIR_RC_TYPE == UL_testing_Tuning_IIR_FILTER_PERFORMANCE) */
            #endif /* (UL_testing_Tuning_REGULAR_SENSOR_EN) */
        }
        else
        {
            #if (UL_testing_Tuning_PROXIMITY_SENSOR_EN && UL_testing_Tuning_PROX_RC_IIR_FILTER_EN)
                #if (UL_testing_Tuning_PROX_IIR_RC_TYPE == UL_testing_Tuning_IIR_FILTER_PERFORMANCE)
                    temp =  ptrFilterHistObj.ptrProx->proxChannel[freqChannel].iirHistory << UL_testing_Tuning_PROX_IIR_RC_SHIFT;
                    temp |= ptrFilterHistObj.ptrProx->proxChannel[freqChannel].iirHistoryLow;
                    temp =UL_testing_Tuning_FtIIR1stOrder((uint32)ptrSensorObj->raw[freqChannel],
                                                        temp,
                                                        UL_testing_Tuning_PROX_IIR_RC_N,
                                                        UL_testing_Tuning_PROX_IIR_RC_SHIFT);
                    ptrFilterHistObj.ptrProx->proxChannel[freqChannel].iirHistory = LO16(temp >>UL_testing_Tuning_PROX_IIR_RC_SHIFT);
                    ptrFilterHistObj.ptrProx->proxChannel[freqChannel].iirHistoryLow = LO8(temp);
                    ptrSensorObj->raw[freqChannel] = LO16(temp >>UL_testing_Tuning_PROX_IIR_RC_SHIFT);
                #else
                    temp =UL_testing_Tuning_FtIIR1stOrder((uint32)ptrSensorObj->raw[freqChannel],
                                                        (uint32)ptrFilterHistObj.ptrProx->proxChannel[freqChannel].iirHistory,
                                                        UL_testing_Tuning_PROX_IIR_RC_N,
                                                        UL_testing_Tuning_PROX_IIR_RC_SHIFT);
                    ptrFilterHistObj.ptrProx->proxChannel[freqChannel].iirHistory = LO16(temp);
                    ptrSensorObj->raw[freqChannel] = LO16(temp >>UL_testing_Tuning_PROX_IIR_RC_SHIFT);
                #endif /* (UL_testing_Tuning_PROX_IIR_RC_TYPE == UL_testing_Tuning_IIR_FILTER_PERFORMANCE) */
            #endif /* (UL_testing_Tuning_PROX_SENSOR_EN) */
        }
    }
}
#endif /* #if (UL_testing_Tuning_REGULAR_RC_IIR_FILTER_EN || UL_testing_Tuning_PROX_RC_IIR_FILTER_EN) */


#if (UL_testing_Tuning_REGULAR_RC_MEDIAN_FILTER_EN || UL_testing_Tuning_PROX_RC_MEDIAN_FILTER_EN)

/*******************************************************************************
* Function Name: UL_testing_Tuning_InitializeMedian
****************************************************************************//**
*
* \brief
*  Initializes the Median filter history.
*
* \details
*  Initializes the Median filter history.
*
* \param widgetId
*  Specifies the ID number of the widget.
*
* \param sensorId
*  Specifies the ID number of the sensor in the widget.
*
*******************************************************************************/
void UL_testing_Tuning_InitializeMedian(uint32 widgetId, uint32 sensorId)
{
    UL_testing_Tuning_FLASH_WD_STRUCT const *ptrWidget = &UL_testing_Tuning_dsFlash.wdgtArray[widgetId];
    UL_testing_Tuning_RAM_SNS_STRUCT *ptrSensor = NULL;
    UL_testing_Tuning_PTR_FILTER_VARIANT ptrFilterHistObj;

    /* Find pointer to specified sensor object */
    ptrSensor = ptrWidget->ptr2SnsRam;
    ptrSensor = &ptrSensor[sensorId];

    /* Find pointer to specified filter sensor object */
    ptrFilterHistObj.ptr = ptrWidget->ptr2FltrHistory;

    if ((UL_testing_Tuning_WD_TYPE_ENUM)ptrWidget->wdgtType != UL_testing_Tuning_WD_PROXIMITY_E)
    {
        #if (UL_testing_Tuning_REGULAR_SENSOR_EN)
            ptrFilterHistObj.ptrRegular = &ptrFilterHistObj.ptrRegular[sensorId];
        #endif /* (UL_testing_Tuning_REGULAR_SENSOR_EN) */
    }
    else
    {
        #if (UL_testing_Tuning_PROXIMITY_SENSOR_EN)
            ptrFilterHistObj.ptrProx = &ptrFilterHistObj.ptrProx[sensorId];
        #endif /* (UL_testing_Tuning_PROXIMITY_SENSOR_EN) */
    }

    UL_testing_Tuning_InitializeMedianInternal(ptrFilterHistObj, ptrSensor, (uint32)ptrWidget->wdgtType);
}


/*******************************************************************************
* Function Name: UL_testing_Tuning_RunMedian
****************************************************************************//**
*
* \brief
*  Executes the Median filter algorithm on a sensor indicated by an input
*  parameter.
*
* \details
*  Executes the Median filter algorithm on a sensor indicated by an input
*  parameter.
*
* \param widgetId
*  Specifies the ID number of the widget.
*
* \param sensorId
*  Specifies the ID number of the sensor in the widget.
*
*******************************************************************************/
void UL_testing_Tuning_RunMedian(uint32 widgetId, uint32 sensorId)
{
    UL_testing_Tuning_FLASH_WD_STRUCT const *ptrWidget = &UL_testing_Tuning_dsFlash.wdgtArray[widgetId];
    UL_testing_Tuning_RAM_SNS_STRUCT *ptrSensor = NULL;
    UL_testing_Tuning_PTR_FILTER_VARIANT ptrFilterHistObj;

    /* Find pointer to specified sensor object */
    ptrSensor = ptrWidget->ptr2SnsRam;
    ptrSensor = &ptrSensor[sensorId];

    /* Find pointer to specified filter sensor object */
    ptrFilterHistObj.ptr = ptrWidget->ptr2FltrHistory;

    if ((UL_testing_Tuning_WD_TYPE_ENUM)ptrWidget->wdgtType != UL_testing_Tuning_WD_PROXIMITY_E)
    {
        #if (UL_testing_Tuning_REGULAR_SENSOR_EN)
            ptrFilterHistObj.ptrRegular = &ptrFilterHistObj.ptrRegular[sensorId];
        #endif /* (UL_testing_Tuning_REGULAR_SENSOR_EN) */
    }
    else
    {
        #if (UL_testing_Tuning_PROXIMITY_SENSOR_EN)
            ptrFilterHistObj.ptrProx = &ptrFilterHistObj.ptrProx[sensorId];
        #endif /* (UL_testing_Tuning_PROXIMITY_SENSOR_EN) */
    }

    UL_testing_Tuning_RunMedianInternal(ptrFilterHistObj, ptrSensor, (uint32)ptrWidget->wdgtType);
}


/*******************************************************************************
* Function Name: UL_testing_Tuning_InitializeMedianInternal
****************************************************************************//**
*
* \brief
*  Initializes the Median filter.
*
* \details
*  Initializes the Median filter.
*
* \param ptrFilterHistObj
*  The pointer to the filter RAM object of the sensor.
*
* \param ptrSensorObj
*  The pointer to the sensor RAM object of the sensor.
*
* \param wdType
*  Specifies the type of a widget.
*
*******************************************************************************/
void UL_testing_Tuning_InitializeMedianInternal(UL_testing_Tuning_PTR_FILTER_VARIANT ptrFilterHistObj,
                                               UL_testing_Tuning_RAM_SNS_STRUCT *ptrSensorObj, uint32 wdType)
{
    uint32 freqChannel;

    for(freqChannel = UL_testing_Tuning_NUM_SCAN_FREQS; freqChannel-- > 0u;)
    {
        if ((UL_testing_Tuning_WD_TYPE_ENUM)wdType != UL_testing_Tuning_WD_PROXIMITY_E)
        {
            #if (UL_testing_Tuning_REGULAR_SENSOR_EN && UL_testing_Tuning_REGULAR_RC_MEDIAN_FILTER_EN)
                ptrFilterHistObj.ptrRegular->regularChannel[freqChannel].medHistory[0u] = ptrSensorObj->raw[freqChannel];
                ptrFilterHistObj.ptrRegular->regularChannel[freqChannel].medHistory[1u] = ptrSensorObj->raw[freqChannel];
            #endif /* (UL_testing_Tuning_REGULAR_SENSOR_EN) */
        }
        else
        {
            #if (UL_testing_Tuning_PROXIMITY_SENSOR_EN && UL_testing_Tuning_PROX_RC_MEDIAN_FILTER_EN)
                ptrFilterHistObj.ptrProx->proxChannel[freqChannel].medHistory[0u] = ptrSensorObj->raw[freqChannel];
                ptrFilterHistObj.ptrProx->proxChannel[freqChannel].medHistory[1u] = ptrSensorObj->raw[freqChannel];
            #endif /* (UL_testing_Tuning_PROXIMITY_SENSOR_EN) */
        }
    }
}


/*******************************************************************************
* Function Name: UL_testing_Tuning_RunMedianInternal
****************************************************************************//**
*
* \brief
*  Runs the Median filter.
*
* \details
*  Runs the Median filter.
*
* \param ptrFilterHistObj
*  The pointer to the filter RAM object of the sensor.
*
* \param ptrSensorObj
*  The pointer to the sensor RAM object of the sensor.
*
* \param wdType
*  Specifies the type of a widget.
*
*******************************************************************************/
void UL_testing_Tuning_RunMedianInternal(UL_testing_Tuning_PTR_FILTER_VARIANT ptrFilterHistObj,
                                        UL_testing_Tuning_RAM_SNS_STRUCT *ptrSensorObj, uint32 wdType)
{
    uint32 freqChannel;

    #if ((UL_testing_Tuning_REGULAR_SENSOR_EN && UL_testing_Tuning_REGULAR_RC_MEDIAN_FILTER_EN) || \
         (UL_testing_Tuning_PROXIMITY_SENSOR_EN && UL_testing_Tuning_PROX_RC_MEDIAN_FILTER_EN))
        uint32 temp;
    #endif

    for(freqChannel = UL_testing_Tuning_NUM_SCAN_FREQS; freqChannel-- > 0u;)
    {
        if ((UL_testing_Tuning_WD_TYPE_ENUM)wdType != UL_testing_Tuning_WD_PROXIMITY_E)
        {
            #if (UL_testing_Tuning_REGULAR_SENSOR_EN && UL_testing_Tuning_REGULAR_RC_MEDIAN_FILTER_EN)
                temp = UL_testing_Tuning_FtMedian((uint32)ptrSensorObj->raw[freqChannel],\
                                                 (uint32)ptrFilterHistObj.ptrRegular->regularChannel[freqChannel].medHistory[0u],\
                                                 (uint32)ptrFilterHistObj.ptrRegular->regularChannel[freqChannel].medHistory[1u]);
                ptrFilterHistObj.ptrRegular->regularChannel[freqChannel].medHistory[1u] = \
                ptrFilterHistObj.ptrRegular->regularChannel[freqChannel].medHistory[0u];
                ptrFilterHistObj.ptrRegular->regularChannel[freqChannel].medHistory[0u] = ptrSensorObj->raw[freqChannel];
                ptrSensorObj->raw[freqChannel] = LO16(temp);
            #endif /* (UL_testing_Tuning_REGULAR_SENSOR_EN) */
        }
        else
        {
            #if (UL_testing_Tuning_PROXIMITY_SENSOR_EN && UL_testing_Tuning_PROX_RC_MEDIAN_FILTER_EN)
                temp = UL_testing_Tuning_FtMedian((uint32)ptrSensorObj->raw[freqChannel],\
                                                 (uint32)ptrFilterHistObj.ptrProx->proxChannel[freqChannel].medHistory[0u],\
                                                 (uint32)ptrFilterHistObj.ptrProx->proxChannel[freqChannel].medHistory[1u]);
                ptrFilterHistObj.ptrProx->proxChannel[freqChannel].medHistory[1u] = \
                ptrFilterHistObj.ptrProx->proxChannel[freqChannel].medHistory[0u];
                ptrFilterHistObj.ptrProx->proxChannel[freqChannel].medHistory[0u] = ptrSensorObj->raw[freqChannel];
                ptrSensorObj->raw[freqChannel] = LO16(temp);
            #endif /* (UL_testing_Tuning_PROXIMITY_SENSOR_EN) */
        }
    }
}
#endif /* (UL_testing_Tuning_REGULAR_RC_MEDIAN_FILTER_EN || UL_testing_Tuning_PROX_RC_MEDIAN_FILTER_EN) */


#if (UL_testing_Tuning_REGULAR_RC_AVERAGE_FILTER_EN || UL_testing_Tuning_PROX_RC_AVERAGE_FILTER_EN)

/*******************************************************************************
* Function Name: UL_testing_Tuning_InitializeAverage
****************************************************************************//**
*
* \brief
*  Initializes the average filter history.
*
* \details
*  Initializes the average filter history.
*
* \param widgetId
*  Specifies the ID number of the widget.
*
* \param sensorId
*  Specifies the ID number of the sensor in the widget.
*
*******************************************************************************/
void UL_testing_Tuning_InitializeAverage(uint32 widgetId, uint32 sensorId)
{
    UL_testing_Tuning_FLASH_WD_STRUCT const *ptrWidget = &UL_testing_Tuning_dsFlash.wdgtArray[widgetId];
    UL_testing_Tuning_RAM_SNS_STRUCT *ptrSensor = NULL;
    UL_testing_Tuning_PTR_FILTER_VARIANT ptrFilterHistObj;

    /* Find pointer to specified sensor object */
    ptrSensor = ptrWidget->ptr2SnsRam;
    ptrSensor = &ptrSensor[sensorId];

    /* Find pointer to specified filter sensor object */
    ptrFilterHistObj.ptr = ptrWidget->ptr2FltrHistory;

    if ((UL_testing_Tuning_WD_TYPE_ENUM)ptrWidget->wdgtType != UL_testing_Tuning_WD_PROXIMITY_E)
    {
        #if (UL_testing_Tuning_REGULAR_SENSOR_EN)
            ptrFilterHistObj.ptrRegular = &ptrFilterHistObj.ptrRegular[sensorId];
        #endif /* (UL_testing_Tuning_REGULAR_SENSOR_EN) */
    }
    else
    {
        #if (UL_testing_Tuning_PROXIMITY_SENSOR_EN)
            ptrFilterHistObj.ptrProx = &ptrFilterHistObj.ptrProx[sensorId];
        #endif /* (UL_testing_Tuning_PROXIMITY_SENSOR_EN) */
    }

    UL_testing_Tuning_InitializeAverageInternal(ptrFilterHistObj, ptrSensor, (uint32)ptrWidget->wdgtType);
}


/*******************************************************************************
* Function Name: UL_testing_Tuning_RunAverage
****************************************************************************//**
*
* \brief
*  Executes the average filter algorithm on a sensor indicated by an input
*  parameter.
*
* \details
*  Executes the average filter algorithm on a sensor indicated by an input
*  parameter.
*
* \param widgetId
*  Specifies the ID number of the widget.
*
* \param sensorId
*  Specifies the ID number of the sensor in the widget.
*
*******************************************************************************/
void UL_testing_Tuning_RunAverage(uint32 widgetId, uint32 sensorId)
{
    UL_testing_Tuning_FLASH_WD_STRUCT const *ptrWidget = &UL_testing_Tuning_dsFlash.wdgtArray[widgetId];
    UL_testing_Tuning_RAM_SNS_STRUCT *ptrSensor = NULL;
    UL_testing_Tuning_PTR_FILTER_VARIANT ptrFilterHistObj;

    /* Find pointer to specified sensor object */
    ptrSensor = ptrWidget->ptr2SnsRam;
    ptrSensor = &ptrSensor[sensorId];

    /* Find pointer to specified filter sensor object */
    ptrFilterHistObj.ptr = ptrWidget->ptr2FltrHistory;

    if ((UL_testing_Tuning_WD_TYPE_ENUM)ptrWidget->wdgtType != UL_testing_Tuning_WD_PROXIMITY_E)
    {
        #if (UL_testing_Tuning_REGULAR_SENSOR_EN)
            ptrFilterHistObj.ptrRegular = &ptrFilterHistObj.ptrRegular[sensorId];
        #endif /* (UL_testing_Tuning_REGULAR_SENSOR_EN) */
    }
    else
    {
        #if (UL_testing_Tuning_PROXIMITY_SENSOR_EN)
            ptrFilterHistObj.ptrProx = &ptrFilterHistObj.ptrProx[sensorId];
        #endif /* (UL_testing_Tuning_PROXIMITY_SENSOR_EN) */
    }

    UL_testing_Tuning_RunAverageInternal(ptrFilterHistObj, ptrSensor, (uint32)ptrWidget->wdgtType);
}


/*******************************************************************************
* Function Name: UL_testing_Tuning_InitializeAverageInternal
****************************************************************************//**
*
* \brief
*  Initializes the average filter.
*
* \details
*  Initializes the average filter.
*
* \param ptrFilterHistObj
*  The pointer to the filter RAM object of the sensor.
*
* \param ptrSensorObj
*  The pointer to the sensor RAM object of the sensor.
*
* \param wdType
*  Specifies the type of a widget.
*
*******************************************************************************/
void UL_testing_Tuning_InitializeAverageInternal(UL_testing_Tuning_PTR_FILTER_VARIANT ptrFilterHistObj,\
                                                  UL_testing_Tuning_RAM_SNS_STRUCT *ptrSensorObj, uint32 wdType)
{
    uint32 freqChannel;

    for(freqChannel = UL_testing_Tuning_NUM_SCAN_FREQS; freqChannel-- > 0u;)
    {
        if ((UL_testing_Tuning_WD_TYPE_ENUM)wdType != UL_testing_Tuning_WD_PROXIMITY_E)
        {
            #if (UL_testing_Tuning_REGULAR_SENSOR_EN && UL_testing_Tuning_REGULAR_RC_AVERAGE_FILTER_EN)
                ptrFilterHistObj.ptrRegular->regularChannel[freqChannel].avgHistory[0] = ptrSensorObj->raw[freqChannel];
                #if (UL_testing_Tuning_REGULAR_AVERAGE_LEN == UL_testing_Tuning_AVERAGE_FILTER_LEN_4)
                    ptrFilterHistObj.ptrRegular->regularChannel[freqChannel].avgHistory[1u] = ptrSensorObj->raw[freqChannel];
                    ptrFilterHistObj.ptrRegular->regularChannel[freqChannel].avgHistory[2u] = ptrSensorObj->raw[freqChannel];
                #endif /* UL_testing_Tuning_REGULAR_AVERAGE_LEN == UL_testing_Tuning_AVERAGE_FILTER_LEN_4 */
            #endif /* (UL_testing_Tuning_REGULAR_SENSOR_EN) */
        }
        else
        {
            #if (UL_testing_Tuning_PROXIMITY_SENSOR_EN && UL_testing_Tuning_PROX_RC_AVERAGE_FILTER_EN)
                ptrFilterHistObj.ptrProx->proxChannel[freqChannel].avgHistory[0] = ptrSensorObj->raw[freqChannel];
                    #if (UL_testing_Tuning_PROX_AVERAGE_LEN == UL_testing_Tuning_AVERAGE_FILTER_LEN_4)
                        ptrFilterHistObj.ptrProx->proxChannel[freqChannel].avgHistory[1u] = ptrSensorObj->raw[freqChannel];
                        ptrFilterHistObj.ptrProx->proxChannel[freqChannel].avgHistory[2u] = ptrSensorObj->raw[freqChannel];
                    #endif /* UL_testing_Tuning_REGULAR_AVERAGE_LEN == UL_testing_Tuning_AVERAGE_FILTER_LEN_4 */
            #endif /* (UL_testing_Tuning_PROXIMITY_SENSOR_EN) */
        }
    }
}


/*******************************************************************************
* Function Name: UL_testing_Tuning_RunAverageInternal
****************************************************************************//**
*
* \brief
*  Runs the average filter.
*
* \details
*  Runs the average filter.
*
* \param ptrFilterHistObj
*  The pointer to the filter RAM object of the sensor.
*
* \param ptrSensorObj
*  The pointer to the sensor RAM object of the sensor.
*
* \param wdType
*  Specifies the type of a widget.
*
*******************************************************************************/
void UL_testing_Tuning_RunAverageInternal(UL_testing_Tuning_PTR_FILTER_VARIANT ptrFilterHistObj,\
                                           UL_testing_Tuning_RAM_SNS_STRUCT *ptrSensorObj, uint32 wdType)
{
    uint32 freqChannel;
    uint32 temp;

    for(freqChannel = UL_testing_Tuning_NUM_SCAN_FREQS; freqChannel-- > 0u;)
    {
        if ((UL_testing_Tuning_WD_TYPE_ENUM)wdType != UL_testing_Tuning_WD_PROXIMITY_E)
        {
            #if (UL_testing_Tuning_REGULAR_SENSOR_EN && UL_testing_Tuning_REGULAR_RC_AVERAGE_FILTER_EN)
                #if (UL_testing_Tuning_REGULAR_AVERAGE_LEN == UL_testing_Tuning_AVERAGE_FILTER_LEN_2)
                    temp = ((uint32)ptrSensorObj->raw[freqChannel] +
                            (uint32)ptrFilterHistObj.ptrRegular->regularChannel[freqChannel].avgHistory[0]) >> 1u;
                    ptrFilterHistObj.ptrRegular->regularChannel[freqChannel].avgHistory[0] = ptrSensorObj->raw[freqChannel];
                    ptrSensorObj->raw[freqChannel] = LO16(temp);
                #else
                    temp = ((uint32)ptrSensorObj->raw[freqChannel] +
                            (uint32)ptrFilterHistObj.ptrRegular->regularChannel[freqChannel].avgHistory[0] +
                            (uint32)ptrFilterHistObj.ptrRegular->regularChannel[freqChannel].avgHistory[1u] +
                            (uint32)ptrFilterHistObj.ptrRegular->regularChannel[freqChannel].avgHistory[2u]) >> 2u;

                    ptrFilterHistObj.ptrRegular->regularChannel[freqChannel].avgHistory[2u] =
                    ptrFilterHistObj.ptrRegular->regularChannel[freqChannel].avgHistory[1u];
                    ptrFilterHistObj.ptrRegular->regularChannel[freqChannel].avgHistory[1u] =
                    ptrFilterHistObj.ptrRegular->regularChannel[freqChannel].avgHistory[0];
                    ptrFilterHistObj.ptrRegular->regularChannel[freqChannel].avgHistory[0] = ptrSensorObj->raw[freqChannel];
                    ptrSensorObj->raw[freqChannel] = LO16(temp);
                #endif /* UL_testing_Tuning_REGULAR_AVERAGE_LEN == UL_testing_Tuning_AVERAGE_FILTER_LEN_4 */
            #endif /* (UL_testing_Tuning_REGULAR_SENSOR_EN) */
        }
        else
        {
            #if (UL_testing_Tuning_PROXIMITY_SENSOR_EN && UL_testing_Tuning_PROX_RC_AVERAGE_FILTER_EN)
                #if (UL_testing_Tuning_PROX_AVERAGE_LEN == UL_testing_Tuning_AVERAGE_FILTER_LEN_2)
                    temp = ((uint32)ptrSensorObj->raw[freqChannel] +
                            (uint32)ptrFilterHistObj.ptrProx->proxChannel[freqChannel].avgHistory[0]) >> 1u;
                    ptrFilterHistObj.ptrProx->proxChannel[freqChannel].avgHistory[0] = ptrSensorObj->raw[freqChannel];
                    ptrSensorObj->raw[freqChannel] = LO16(temp);
                #else
                    temp = ((uint32)ptrSensorObj->raw[freqChannel] +
                            (uint32)ptrFilterHistObj.ptrProx->proxChannel[freqChannel].avgHistory[0] +
                            (uint32)ptrFilterHistObj.ptrProx->proxChannel[freqChannel].avgHistory[1u] +
                            (uint32)ptrFilterHistObj.ptrProx->proxChannel[freqChannel].avgHistory[2u]) >> 2u;

                    ptrFilterHistObj.ptrProx->proxChannel[freqChannel].avgHistory[2u] =
                    ptrFilterHistObj.ptrProx->proxChannel[freqChannel].avgHistory[1u];
                    ptrFilterHistObj.ptrProx->proxChannel[freqChannel].avgHistory[1u] =
                    ptrFilterHistObj.ptrProx->proxChannel[freqChannel].avgHistory[0];
                    ptrFilterHistObj.ptrProx->proxChannel[freqChannel].avgHistory[0] = ptrSensorObj->raw[freqChannel];
                    ptrSensorObj->raw[freqChannel] = LO16(temp);
                #endif /* UL_testing_Tuning_PROX_AVERAGE_LEN == UL_testing_Tuning_AVERAGE_FILTER_LEN_4 */
            #endif /* (UL_testing_Tuning_PROXIMITY_SENSOR_EN) */
        }
    }
}
#endif /* (UL_testing_Tuning_REGULAR_RC_AVERAGE_FILTER_EN || UL_testing_Tuning_PROX_RC_AVERAGE_FILTER_EN) */

#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_ALP_FILTER_EN)
/*******************************************************************************
* Function Name: UL_testing_Tuning_InitializeALP
****************************************************************************//**
*
* \brief
*  Initializes the ALP filter history.
*
* \details
*  Initializes the ALP filter history.
*
* \param widgetId
*  Specifies the ID number of the widget.
*
* \param sensorId
*  Specifies the ID number of the sensor in the widget.
*
*******************************************************************************/
void UL_testing_Tuning_InitializeALP(uint32 widgetId, uint32 sensorId)
{
    UL_testing_Tuning_FLASH_WD_STRUCT const *ptrWidget = &UL_testing_Tuning_dsFlash.wdgtArray[widgetId];
    UL_testing_Tuning_RAM_SNS_STRUCT *ptrSensor = NULL;
    UL_testing_Tuning_PTR_FILTER_VARIANT ptrFilterHistObj;

    /* Find pointer to specified sensor object */
    ptrSensor = ptrWidget->ptr2SnsRam;
    ptrSensor = &ptrSensor[sensorId];

    /* Find pointer to specified filter sensor object */
    ptrFilterHistObj.ptr = ptrWidget->ptr2FltrHistory;
    ptrFilterHistObj.ptrAlp = &ptrFilterHistObj.ptrAlp[sensorId];
    UL_testing_Tuning_InitializeALPInternal(ptrFilterHistObj, ptrSensor, (uint32)ptrWidget->wdgtType);
}


/*******************************************************************************
* Function Name: UL_testing_Tuning_RunALP
****************************************************************************//**
*
* \brief
*  Executes the ALP filter algorithm on a sensor indicated by an input
*  parameter.
*
* \details
*  Executes the ALP filter algorithm on a sensor indicated by an input
*  parameter.
*
* \param widgetId
*  Specifies the ID number of the widget.
*
* \param sensorId
*  Specifies the ID number of the sensor in the widget.
*
*******************************************************************************/
void UL_testing_Tuning_RunALP(uint32 widgetId, uint32 sensorId)
{
    UL_testing_Tuning_FLASH_WD_STRUCT const *ptrWidget = &UL_testing_Tuning_dsFlash.wdgtArray[widgetId];
    UL_testing_Tuning_RAM_SNS_STRUCT *ptrSensor = NULL;
    UL_testing_Tuning_PTR_FILTER_VARIANT ptrFilterHistObj;
    ALP_FLTR_CONFIG_STRUCT alpFilterConfig;
    ALP_FLTR_CONFIG_STRUCT *ptrAlpFilterConfig = &alpFilterConfig;
    UL_testing_Tuning_RAM_WD_BASE_STRUCT *ptrRamWidget = ptrWidget->ptr2WdgtRam;

    /* Find pointer to specified sensor object */
    ptrSensor = ptrWidget->ptr2SnsRam;
    ptrSensor = &ptrSensor[sensorId];

    ptrAlpFilterConfig->configParam0 = ptrRamWidget->alpOnTh;
    ptrAlpFilterConfig->configParam1 = ptrRamWidget->alpOffTh;
    ptrAlpFilterConfig->configParam2 = ptrRamWidget->fingerTh;
    ptrAlpFilterConfig->configParam3 = ptrRamWidget->noiseTh;
    ptrAlpFilterConfig->configParam4 = ptrRamWidget->hysteresis;

    /* Find pointer to specified filter sensor object */
    ptrFilterHistObj.ptr = ptrWidget->ptr2FltrHistory;
    ptrFilterHistObj.ptrAlp = &ptrFilterHistObj.ptrAlp[sensorId];

    if ((UL_testing_Tuning_WD_TYPE_ENUM)ptrWidget->wdgtType != UL_testing_Tuning_WD_PROXIMITY_E)
    {
        #if (UL_testing_Tuning_REGULAR_SENSOR_EN)
            ptrAlpFilterConfig->configParam5 = UL_testing_Tuning_REGULAR_RC_ALP_FILTER_COEFF;
        #endif /* (UL_testing_Tuning_REGULAR_SENSOR_EN) */
    }
    else
    {
        #if (UL_testing_Tuning_PROXIMITY_SENSOR_EN)
            ptrAlpFilterConfig->configParam5 = UL_testing_Tuning_PROX_RC_ALP_FILTER_COEFF;
        #endif /* (UL_testing_Tuning_PROXIMITY_SENSOR_EN) */
    }
    UL_testing_Tuning_RunALPInternal(ptrFilterHistObj, ptrAlpFilterConfig, ptrSensor, (uint32)ptrWidget->wdgtType);
}


/*******************************************************************************
* Function Name: UL_testing_Tuning_InitializeALPInternal
****************************************************************************//**
*
* \brief
*  Initializes the ALP filter.
*
* \details
*  Initializes the ALP filter.
*
* \param ptrFilterHistObj
*  The pointer to the filter RAM object of the sensor.
*
* \param ptrSensorObj
*  The pointer to the sensor RAM object of the sensor.
*
* \param wdType
*  Specifies the type of a widget.
*
*
*******************************************************************************/
void UL_testing_Tuning_InitializeALPInternal(UL_testing_Tuning_PTR_FILTER_VARIANT ptrFilterHistObj,
                                            UL_testing_Tuning_RAM_SNS_STRUCT *ptrSensorObj,
                                            uint32 wdType)
{
    uint32 freqChannel;

    for(freqChannel = UL_testing_Tuning_NUM_SCAN_FREQS; freqChannel-- > 0u;)
    {
        if ((UL_testing_Tuning_WD_TYPE_ENUM)wdType != UL_testing_Tuning_WD_PROXIMITY_E)
        {
            #if (UL_testing_Tuning_REGULAR_SENSOR_EN && UL_testing_Tuning_REGULAR_RC_ALP_FILTER_EN)
                ALP_Initialize(&ptrFilterHistObj.ptrAlp->channel[freqChannel], &ptrSensorObj->raw[freqChannel]);
            #endif
        }
        else
        {
            #if (UL_testing_Tuning_PROXIMITY_SENSOR_EN && UL_testing_Tuning_PROX_RC_ALP_FILTER_EN)
                ALP_Initialize(&ptrFilterHistObj.ptrAlp->channel[freqChannel], &ptrSensorObj->raw[freqChannel]);
            #endif
        }
    }
}

/*******************************************************************************
* Function Name: UL_testing_Tuning_ConfigRunALPInternal
****************************************************************************//**
*
* \brief
*  Prepares ALP configuration structure and runs the filter.
*
* \details
*  Prepares ALP configuration structure and runs the filter.
*
* \param ptrFilterHistObj
*  The pointer to the filter RAM object of the sensor.
*
* \param ptrRamWdgt
*  The pointer to the RAM widget object.
*
* \param ptrSensorObj
*  The pointer to the sensor RAM object of the sensor.
*
* \param wdType
*  Specifies the type of a widget.
*
*******************************************************************************/
void UL_testing_Tuning_ConfigRunALPInternal(UL_testing_Tuning_PTR_FILTER_VARIANT ptrFilterHistObj,
                                    UL_testing_Tuning_RAM_WD_BASE_STRUCT *ptrRamWdgt,
                                    UL_testing_Tuning_RAM_SNS_STRUCT *ptrSensorObj,
                                    uint32 wdType)
{
    ALP_FLTR_CONFIG_STRUCT alpFilterConfig;

    alpFilterConfig.configParam0 = ptrRamWdgt->alpOnTh;
    alpFilterConfig.configParam1 = ptrRamWdgt->alpOffTh;
    alpFilterConfig.configParam2 = ptrRamWdgt->fingerTh;
    alpFilterConfig.configParam3 = ptrRamWdgt->noiseTh;
    alpFilterConfig.configParam4 = ptrRamWdgt->hysteresis;

    UL_testing_Tuning_RunALPInternal(ptrFilterHistObj, &alpFilterConfig, ptrSensorObj, wdType);
}

/*******************************************************************************
* Function Name: UL_testing_Tuning_RunALPInternal
****************************************************************************//**
*
* \brief
*  Runs the ALP filter.
*
* \details
*  Runs the ALP filter.
*
* \param ptrFilterHistObj
*  The pointer to the filter RAM object of the sensor.
*
* \param ptrAlpFilterConfig
*  The pointer to the filter configuration object of the sensor.
*
* \param ptrSensorObj
*  The pointer to the sensor RAM object of the sensor.
*
* \param wdType
*  Specifies the type of a widget.
*
*******************************************************************************/
void UL_testing_Tuning_RunALPInternal(UL_testing_Tuning_PTR_FILTER_VARIANT ptrFilterHistObj,
                                    ALP_FLTR_CONFIG_STRUCT *ptrAlpFilterConfig,
                                    UL_testing_Tuning_RAM_SNS_STRUCT *ptrSensorObj,
                                    uint32 wdType)
{
    uint32 freqChannel;

    for(freqChannel = UL_testing_Tuning_NUM_SCAN_FREQS; freqChannel-- > 0u;)
    {
        if ((UL_testing_Tuning_WD_TYPE_ENUM)wdType != UL_testing_Tuning_WD_PROXIMITY_E)
        {
            #if (UL_testing_Tuning_REGULAR_SENSOR_EN && UL_testing_Tuning_REGULAR_RC_ALP_FILTER_EN)
                ptrAlpFilterConfig->configParam5 = UL_testing_Tuning_REGULAR_RC_ALP_FILTER_COEFF;
                ALP_Run(&ptrFilterHistObj.ptrAlp->channel[freqChannel], ptrAlpFilterConfig, &ptrSensorObj->raw[freqChannel], &ptrSensorObj->bsln[freqChannel]);
            #endif
        }
        else
        {
            #if (UL_testing_Tuning_PROXIMITY_SENSOR_EN && UL_testing_Tuning_PROX_RC_ALP_FILTER_EN)
                ptrAlpFilterConfig->configParam5 = UL_testing_Tuning_PROX_RC_ALP_FILTER_COEFF;
                ALP_Run(&ptrFilterHistObj.ptrAlp->channel[freqChannel], ptrAlpFilterConfig, &ptrSensorObj->raw[freqChannel], &ptrSensorObj->bsln[freqChannel]);
            #endif
        }
    }
}
#endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_ALP_FILTER_EN) */

#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_RC_FILTER_EN)
/*******************************************************************************
* Function Name: UL_testing_Tuning_FtRunEnabledFilters
****************************************************************************//**
*
* \brief
*  Runs all enabled filters.
*
* \details
*  Runs all enabled filters.
*
* \param widgetId
*  Specifies the ID number of a widget to update the IIR filter history.
*
* \param sensorId
*  Specifies the ID number of a sensor in the widget to update the IIR
*  filter history.
*
*******************************************************************************/
void UL_testing_Tuning_FtRunEnabledFilters(uint32 widgetId, uint32 sensorId)
{
    UL_testing_Tuning_FLASH_WD_STRUCT const *ptrWidget = &UL_testing_Tuning_dsFlash.wdgtArray[widgetId];
    UL_testing_Tuning_RAM_SNS_STRUCT *ptrSensor = NULL;
    UL_testing_Tuning_PTR_FILTER_VARIANT ptrFilterHistObj;

    /* Find pointer to specified sensor object */
    ptrSensor = ptrWidget->ptr2SnsRam;
    ptrSensor = &ptrSensor[sensorId];

    /* Find pointer to specified filter sensor object */
    ptrFilterHistObj.ptr = ptrWidget->ptr2FltrHistory;

    if ((UL_testing_Tuning_WD_TYPE_ENUM)ptrWidget->wdgtType != UL_testing_Tuning_WD_PROXIMITY_E)
    {
        #if (0u != UL_testing_Tuning_REGULAR_RC_ALP_FILTER_EN)
            ptrFilterHistObj.ptr = &ptrFilterHistObj.ptrAlp[sensorId];
        #elif (0u != UL_testing_Tuning_REGULAR_RC_FILTER_EN)
            ptrFilterHistObj.ptr = &ptrFilterHistObj.ptrRegular[sensorId];
        #endif
    }
    else
    {
        #if (0u != UL_testing_Tuning_PROX_RC_ALP_FILTER_EN)
            ptrFilterHistObj.ptr = &ptrFilterHistObj.ptrAlp[sensorId];
        #elif (0u != UL_testing_Tuning_PROX_RC_FILTER_EN)
            ptrFilterHistObj.ptr = &ptrFilterHistObj.ptrProx[sensorId];
        #endif
    }

    UL_testing_Tuning_FtRunEnabledFiltersInternal(ptrFilterHistObj, ptrSensor, (uint32)ptrWidget->wdgtType);
    #if (UL_testing_Tuning_ALP_FILTER_EN)
        UL_testing_Tuning_ConfigRunALPInternal(ptrFilterHistObj, ptrWidget->ptr2WdgtRam, ptrSensor, (uint32)ptrWidget->wdgtType);
    #endif
}


/*******************************************************************************
* Function Name: UL_testing_Tuning_FtRunEnabledFiltersInternal
****************************************************************************//**
*
* \brief
*  Runs all enabled filters.
*
* \details
*  Runs all enabled filters.
*
* \param ptrFilterHistObj
*  The pointer to the filter RAM object of the sensor.
*
* \param ptrSensorObj
*  The pointer to the sensor RAM object of the sensor.
*
* \param wdType
*  Specifies the type of a widget.
*
*******************************************************************************/
void UL_testing_Tuning_FtRunEnabledFiltersInternal(UL_testing_Tuning_PTR_FILTER_VARIANT ptrFilterHistObj,
                                                  UL_testing_Tuning_RAM_SNS_STRUCT *ptrSensorObj, uint32 wdType)
{
    #if (UL_testing_Tuning_REGULAR_RC_MEDIAN_FILTER_EN || UL_testing_Tuning_PROX_RC_MEDIAN_FILTER_EN)
        UL_testing_Tuning_RunMedianInternal(ptrFilterHistObj, ptrSensorObj, wdType);
    #endif /* (UL_testing_Tuning_REGULAR_RC_MEDIAN_FILTER_EN || UL_testing_Tuning_PROX_RC_MEDIAN_FILTER_EN) */

    #if (UL_testing_Tuning_REGULAR_RC_IIR_FILTER_EN || UL_testing_Tuning_PROX_RC_IIR_FILTER_EN)
        UL_testing_Tuning_RunIIRInternal(ptrFilterHistObj, ptrSensorObj, wdType);
    #endif /* (UL_testing_Tuning_REGULAR_RC_IIR_FILTER_EN || UL_testing_Tuning_PROX_RC_IIR_FILTER_EN) */

    #if (UL_testing_Tuning_REGULAR_RC_AVERAGE_FILTER_EN || UL_testing_Tuning_PROX_RC_AVERAGE_FILTER_EN)
        UL_testing_Tuning_RunAverageInternal(ptrFilterHistObj, ptrSensorObj, wdType);
    #endif /* (UL_testing_Tuning_REGULAR_RC_AVERAGE_FILTER_EN || UL_testing_Tuning_PROX_RC_AVERAGE_FILTER_EN) */
}
#endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_RC_FILTER_EN) */


#if (0u != (UL_testing_Tuning_CSD_AUTOTUNE & UL_testing_Tuning_CSD_SS_TH_EN))
/*******************************************************************************
* Function Name: UL_testing_Tuning_RunNoiseEnvelope
****************************************************************************//**
*
* \brief
*  Executes the noise envelope  filter algorithm on a sensor indicated by
*  an input parameter to measure the pk-to-pk noise in the sensor raw count.
*
* \details
*  Executes the noise envelope  filter algorithm on a sensor indicated by
*  an input parameter to measure the pk-to-pk noise in the sensor raw count.
*
* \param widgetId
*  Specifies the ID number of the widget.
*
* \param sensorId
*  Specifies the ID number of the sensor in the widget.
*
*******************************************************************************/
void UL_testing_Tuning_RunNoiseEnvelope(uint32 widgetId, uint32 sensorId)
{
    UL_testing_Tuning_FLASH_WD_STRUCT const *ptrWidget = &UL_testing_Tuning_dsFlash.wdgtArray[widgetId];
    UL_testing_Tuning_RAM_WD_BASE_STRUCT *ptrWidgetRam = ptrWidget->ptr2WdgtRam;
    UL_testing_Tuning_RAM_SNS_STRUCT *ptrSensor = NULL;
    SMARTSENSE_CSD_NOISE_ENVELOPE_STRUCT *ptrNoiseEnvelope = NULL;

    /* Find pointer to specified sensor object */
    ptrSensor = ptrWidget->ptr2SnsRam;
    ptrSensor = &ptrSensor[sensorId];

    /* Find pointer to specified filter sensor object */
    ptrNoiseEnvelope = ptrWidget->ptr2NoiseEnvlp;
    ptrNoiseEnvelope = &ptrNoiseEnvelope[sensorId];

    SmartSense_RunNoiseEnvelope(ptrSensor->raw[0u], ptrWidgetRam->sigPFC, ptrNoiseEnvelope);
}


/*******************************************************************************
* Function Name: UL_testing_Tuning_InitializeNoiseEnvelope
****************************************************************************//**
*
* \brief
*  Initializes the noise-envelope filter
*
* \details
*  Initializes the noise-envelope filter
*
* \param widgetId
*  Specifies the ID number of the widget.
*
* \param sensorId
*  Specifies the ID number of the sensor in the widget.
*
*******************************************************************************/
void UL_testing_Tuning_InitializeNoiseEnvelope(uint32 widgetId, uint32 sensorId)
{
    UL_testing_Tuning_FLASH_WD_STRUCT const *ptrWidget = &UL_testing_Tuning_dsFlash.wdgtArray[widgetId];
    UL_testing_Tuning_RAM_WD_BASE_STRUCT *ptrWidgetRam = ptrWidget->ptr2WdgtRam;
    UL_testing_Tuning_RAM_SNS_STRUCT *ptrSensor = NULL;
    SMARTSENSE_CSD_NOISE_ENVELOPE_STRUCT *ptrNoiseEnvelope = NULL;

    /* Find pointer to specified sensor object */
    ptrSensor = ptrWidget->ptr2SnsRam;
    ptrSensor = &ptrSensor[sensorId];

    /* Find pointer to specified filter sensor object */
    ptrNoiseEnvelope = ptrWidget->ptr2NoiseEnvlp;
    ptrNoiseEnvelope = &ptrNoiseEnvelope[sensorId];

    SmartSense_InitializeNoiseEnvelope(ptrSensor->raw[0u], ptrWidgetRam->sigPFC, ptrNoiseEnvelope);
}
#endif /* (0u != (UL_testing_Tuning_CSD_AUTOTUNE & UL_testing_Tuning_CSD_SS_TH_EN)) */


#if (UL_testing_Tuning_POS_MEDIAN_FILTER_EN || UL_testing_Tuning_REGULAR_RC_MEDIAN_FILTER_EN || UL_testing_Tuning_PROX_RC_MEDIAN_FILTER_EN)
/*******************************************************************************
* Function Name: UL_testing_Tuning_FtMedian
****************************************************************************//**
*
* \brief
*  Return the median value from the three passed arguments.
*
* \details
*  Return the median value from the three passed arguments.
*
* \param x1
*  The first value to be compared.
*
* \param x2
*  The second value to be compared.
*
* \param x3
*  The third value to be compared.
*
* \return
*  Returns the median value of input arguments.
*
*******************************************************************************/
uint32 UL_testing_Tuning_FtMedian(uint32 x1, uint32 x2, uint32 x3)
{
    uint32 tmp;

    if (x1 > x2)
    {
        tmp = x2;
        x2 = x1;
        x1 = tmp;
    }

    if (x2 > x3)
    {
        x2 = x3;
    }

    return ((x1 > x2) ? x1 : x2);
}
#endif /*UL_testing_Tuning_POS_MEDIAN_FILTER_EN || UL_testing_Tuning_REGULAR_RC_MEDIAN_FILTER_EN || UL_testing_Tuning_PROX_RC_MEDIAN_FILTER_EN*/


/*******************************************************************************
* Function Name: UL_testing_Tuning_FtIIR1stOrder
****************************************************************************//**
*
* \brief
*  Return the filtered data by the IIR 1-st order algorithm
*
* \details
*  Return the filtered data by the IIR 1-st order algorithm
*
* \param input
*  The data to be filtered.
*
* \param prevOutput
*  The previous filtered data.
*
* \param n
*  The IIR filter coefficient (n/256).
*
* \param shift
*  The parameter is used to shift input data to have free LSB
*  bits for a fraction storage of the filter output calculation.
*
* \return
*  Returns the filtered data.
*
*******************************************************************************/
uint32 UL_testing_Tuning_FtIIR1stOrder(uint32 input, uint32 prevOutput, uint32 n, uint32 shift)
{
    uint32 filteredOutput;

    /*
    * n - IIR filter coefficient (n/256)
    * shift - Used to shift input data to have free LSB bits
    * for fraction storage of filter output calculation
    */
    filteredOutput = ((n * (input << shift)) + ((UL_testing_Tuning_IIR_COEFFICIENT_K - n) * prevOutput)) >> 8u;

    /* Shift operation of output will be done in upper level API if needed */
    return filteredOutput;
}


#if (UL_testing_Tuning_POS_JITTER_FILTER_EN)
/*******************************************************************************
* Function Name: UL_testing_Tuning_FtJitter
****************************************************************************//**
*
* \brief
*  Returns the filtered data by the jitter algorithm.
*
* \details
*  Returns the filtered data by the jitter algorithm.
*
* \param input
*  The data to be filtered.
*
* \param prevOutput
*  The previous filtered data.
*
* \return
*  Returns the filtered data.
*
*******************************************************************************/
uint32 UL_testing_Tuning_FtJitter(uint32 input, uint32 prevOutput)
{
    if (prevOutput > input)
    {
        input++;
    }
    else if (prevOutput < input)
    {
        input--;
    }
    else
    {
        /* Nothing to do - MISRA 14.1 requirement*/
    }
    return input;
}
#endif /* UL_testing_Tuning_POS_JITTER_FILTER_EN */


/* [] END OF FILE */
