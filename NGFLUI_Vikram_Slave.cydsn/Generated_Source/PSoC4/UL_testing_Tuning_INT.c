/***************************************************************************//**
* \file UL_testing_Tuning_INT.c
* \version 6.0
*
* \brief
*   This file contains the source code for implementation of the Component's
*   Interrupt Service Routine (ISR).
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
#include "cytypes.h"
#include "cyfitter.h"
#include "UL_testing_Tuning_Configuration.h"
#include "UL_testing_Tuning_Structure.h"
#include "UL_testing_Tuning_Sensing.h"
#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN)
    #include "UL_testing_Tuning_SensingCSD_LL.h"
#endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN) */
#include "cyapicallbacks.h"

/*******************************************************************************
* Static Function Prototypes
*******************************************************************************/
/**
* \cond SECTION_C_INTERNAL
* \addtogroup group_c_internal
* \{
*/

#if (((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN) || (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_CSX_EN)) && \
     (UL_testing_Tuning_ENABLE == UL_testing_Tuning_MULTI_FREQ_SCAN_EN))
    static void UL_testing_Tuning_SsNextFrequencyScan(void);
#endif /* (((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN) || (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_CSX_EN)) && \
            (UL_testing_Tuning_ENABLE == UL_testing_Tuning_MULTI_FREQ_SCAN_EN)) */

#if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN) || (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_CSX_EN))
    static void UL_testing_Tuning_SsCSDPostScan(void);
    static void UL_testing_Tuning_SsCSDInitNextScan(void);
#endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN) || (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_CSX_EN)) */
/** \}
* \endcond */


/**
* \cond SECTION_C_INTERRUPT
* \addtogroup group_c_interrupt
* \{
*/


#if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN) || (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_CSX_EN))

#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2)
    /* Fourth-generation HW block part */

    /*******************************************************************************
    * Function Name: UL_testing_Tuning_CSDPostSingleScan
    ****************************************************************************//**
    *
    * \brief
    *  This is an internal ISR function for the single-sensor scanning implementation.
    *
    * \details
    *  This ISR handler is triggered when the user calls the
    *  UL_testing_Tuning_CSDScanExt() function.
    *
    *  The following tasks are performed for Third-generation HW block:
    *    1. Disable the CSD interrupt.
    *    2. Read the Counter register and update the data structure with raw data.
    *    3. Connect the Vref buffer to the AMUX bus.
    *    4. Update the Scan Counter.
    *    5. Reset the BUSY flag.
    *    6. Enable the CSD interrupt.
    *
    *  The following tasks are performed for Fourth-generation HW block:
    *    1. Check if the raw data is not noisy.
    *    2. Read the Counter register and update the data structure with raw data.
    *    3. Configure and start the scan for the next frequency if the
    *      multi-frequency is enabled.
    *    4. Update the Scan Counter.
    *    5. Reset the BUSY flag.
    *    6. Enable the CSD interrupt.
    *
    *  The ISR handler changes the IMO and initializes scanning for the next frequency
    *  channels when multi-frequency scanning is enabled.
    *
    *  This function has two Macro Callbacks that allow calling the user code
    *  from macros specified in Component's generated code. Refer to the
    *  \ref group_c_macrocallbacks section of the PSoC Creator User Guide
    *  for details.
    *
    *******************************************************************************/
    CY_ISR(UL_testing_Tuning_CSDPostSingleScan)
    {
        #ifdef UL_testing_Tuning_ENTRY_CALLBACK
            UL_testing_Tuning_EntryCallback();
        #endif /* UL_testing_Tuning_ENTRY_CALLBACK */

        /* Clear pending interrupt */
        CY_SET_REG32(UL_testing_Tuning_INTR_PTR, UL_testing_Tuning_INTR_SET_MASK);

        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_NOISE_METRIC_EN)
            if ((UL_testing_Tuning_CSD_NOISE_METRIC_TH < ((CY_GET_REG32(UL_testing_Tuning_RESULT_VAL1_PTR) &
                                                        UL_testing_Tuning_RESULT_VAL1_BAD_CONVS_MASK) >>
                                                        UL_testing_Tuning_RESULT_VAL1_BAD_CONVS_SHIFT)) &&
                                                        (0u < UL_testing_Tuning_badConversionsNum))
            {
                /* Decrement bad conversions number */
                UL_testing_Tuning_badConversionsNum--;

                /* Start the re-scan */
                CY_SET_REG32(UL_testing_Tuning_SEQ_START_PTR, UL_testing_Tuning_SEQ_START_AZ0_SKIP_MASK |
                                                             UL_testing_Tuning_SEQ_START_AZ1_SKIP_MASK |
                                                             UL_testing_Tuning_SEQ_START_START_MASK);
            }
            else
            {
        #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_NOISE_METRIC_EN) */

            UL_testing_Tuning_SsCSDPostScan();

            #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_MULTI_FREQ_SCAN_EN)
                if (UL_testing_Tuning_FREQ_CHANNEL_2 > UL_testing_Tuning_scanFreqIndex)
                {
                    /* Scan the next channel */
                    UL_testing_Tuning_SsNextFrequencyScan();
                }
                else
                {
                    /* All channels are scanned. Set IMO to zero channel */
                    UL_testing_Tuning_scanFreqIndex = UL_testing_Tuning_FREQ_CHANNEL_0;

                    #if (UL_testing_Tuning_MFS_IMO == UL_testing_Tuning_MFS_METHOD)
                        UL_testing_Tuning_SsChangeImoFreq(UL_testing_Tuning_FREQ_CHANNEL_0);
                    #else
                        UL_testing_Tuning_SsChangeClkFreq(UL_testing_Tuning_FREQ_CHANNEL_0);
                    #endif /* (UL_testing_Tuning_MFS_IMO == UL_testing_Tuning_MFS_METHOD) */

                    #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_BLOCK_OFF_AFTER_SCAN_EN)
                        /* Disable Fourth-generation HW block */
                        CY_SET_REG32(UL_testing_Tuning_CONFIG_PTR, UL_testing_Tuning_configCsd);
                    #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_BLOCK_OFF_AFTER_SCAN_EN) */

                    /* Update Scan Counter */
                    UL_testing_Tuning_dsRam.scanCounter++;

                    /* Sensor is totally scanned. Reset BUSY flag */
                    UL_testing_Tuning_dsRam.status &= ~(UL_testing_Tuning_SW_STS_BUSY | UL_testing_Tuning_WDGT_SW_STS_BUSY);
                }
            #else
                {
                    #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_BLOCK_OFF_AFTER_SCAN_EN)
                        /* Disable Fourth-generation HW block */
                        CY_SET_REG32(UL_testing_Tuning_CONFIG_PTR, UL_testing_Tuning_configCsd);
                    #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_BLOCK_OFF_AFTER_SCAN_EN) */

                    /* Update Scan Counter */
                    UL_testing_Tuning_dsRam.scanCounter++;

                    /* Sensor is totally scanned. Reset BUSY flag */
                    UL_testing_Tuning_dsRam.status &= ~(UL_testing_Tuning_SW_STS_BUSY | UL_testing_Tuning_WDGT_SW_STS_BUSY);
                }
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_MULTI_FREQ_SCAN_EN) */
    #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_NOISE_METRIC_EN)
        }
    #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_NOISE_METRIC_EN) */

        #ifdef UL_testing_Tuning_EXIT_CALLBACK
            UL_testing_Tuning_ExitCallback();
        #endif /* UL_testing_Tuning_EXIT_CALLBACK */
    }


    /*******************************************************************************
    * Function Name: UL_testing_Tuning_CSDPostMultiScan
    ****************************************************************************//**
    *
    * \brief
    *  This is an internal ISR function for the multiple-sensor scanning
    *  implementation.
    *
    * \details
    *  This ISR handler is triggered when the user calls the
    *  UL_testing_Tuning_Scan() or UL_testing_Tuning_ScanAllWidgets() APIs.
    *
    *  The following tasks are performed:
    *    1. Disable the CSD interrupt.
    *    2. Read the Counter register and update the data structure with raw data.
    *    3. Connect the Vref buffer to the AMUX bus.
    *    4. Disable the CSD block (after the widget has been scanned).
    *    5. Update the Scan Counter.
    *    6. Reset the BUSY flag.
    *    7. Enable the CSD interrupt.
    *
    *  The ISR handler initializes scanning for the previous sensor when the
    *  widget has more than one sensor.
    *  The ISR handler initializes scanning for the next widget when the
    *  UL_testing_Tuning_ScanAllWidgets() APIs are called and the project has
    *  more than one widget.
    *  The ISR handler changes the IMO and initializes scanning for the next
    *  frequency channels when multi-frequency scanning is enabled.
    *
    *  This function has two Macro Callbacks that allow calling the user
    *  code from macros specified in Component's generated code. Refer to the
    *  \ref group_c_macrocallbacks section of the PSoC Creator User Guide
    *  for details.
    *
    *******************************************************************************/
    CY_ISR(UL_testing_Tuning_CSDPostMultiScan)
    {
        /* Declare and initialize ptr to sensor IO structure to appropriate address */
        UL_testing_Tuning_FLASH_IO_STRUCT const *curSnsIOPtr = (UL_testing_Tuning_FLASH_IO_STRUCT const *)
                                                          UL_testing_Tuning_dsFlash.wdgtArray[UL_testing_Tuning_widgetIndex].ptr2SnsFlash
                                                          + UL_testing_Tuning_sensorIndex;

        #ifdef UL_testing_Tuning_ENTRY_CALLBACK
            UL_testing_Tuning_EntryCallback();
        #endif /* UL_testing_Tuning_ENTRY_CALLBACK */

        /* Clear pending interrupt */
        CY_SET_REG32(UL_testing_Tuning_INTR_PTR, UL_testing_Tuning_INTR_SET_MASK);

        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_NOISE_METRIC_EN)
            if ((UL_testing_Tuning_CSD_NOISE_METRIC_TH < ((CY_GET_REG32(UL_testing_Tuning_RESULT_VAL1_PTR) &
                                                      UL_testing_Tuning_RESULT_VAL1_BAD_CONVS_MASK) >>
                                                      UL_testing_Tuning_RESULT_VAL1_BAD_CONVS_SHIFT)) &&
                                                      (0u < UL_testing_Tuning_badConversionsNum))
            {
                /* Decrement bad conversions number */
                UL_testing_Tuning_badConversionsNum--;

                /* Start the re-scan */
                CY_SET_REG32(UL_testing_Tuning_SEQ_START_PTR, UL_testing_Tuning_SEQ_START_AZ0_SKIP_MASK |
                                                             UL_testing_Tuning_SEQ_START_AZ1_SKIP_MASK |
                                                             UL_testing_Tuning_SEQ_START_START_MASK);
            }
            else
            {
        #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_NOISE_METRIC_EN) */

            UL_testing_Tuning_SsCSDPostScan();

            /* Disable sensor when all frequency channels are scanned */
            #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_MULTI_FREQ_SCAN_EN)
                if (UL_testing_Tuning_FREQ_CHANNEL_2 == UL_testing_Tuning_scanFreqIndex)
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_MULTI_FREQ_SCAN_EN) */
            {
                /* Disable sensor */
                UL_testing_Tuning_CSDDisconnectSns(curSnsIOPtr);
            }

            #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_MULTI_FREQ_SCAN_EN)
                if (UL_testing_Tuning_FREQ_CHANNEL_2 > UL_testing_Tuning_scanFreqIndex)
                {
                     /* Scan the next channel */
                    UL_testing_Tuning_SsNextFrequencyScan();
                }
                else
                {
                     /* All channels are scanned. Set IMO to zero channel */
                    UL_testing_Tuning_scanFreqIndex = UL_testing_Tuning_FREQ_CHANNEL_0;

                    #if (UL_testing_Tuning_MFS_IMO == UL_testing_Tuning_MFS_METHOD)
                        UL_testing_Tuning_SsChangeImoFreq(UL_testing_Tuning_FREQ_CHANNEL_0);
                    #else
                        UL_testing_Tuning_SsChangeClkFreq(UL_testing_Tuning_FREQ_CHANNEL_0);
                    #endif /* (UL_testing_Tuning_MFS_IMO == UL_testing_Tuning_MFS_METHOD) */

                     /* Scan the next sensor */
                    UL_testing_Tuning_SsCSDInitNextScan();
                }
            #else
                /* Scan the next sensor */
                UL_testing_Tuning_SsCSDInitNextScan();
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_MULTI_FREQ_SCAN_EN) */
        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_NOISE_METRIC_EN)
            }
        #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_NOISE_METRIC_EN) */

        #ifdef UL_testing_Tuning_EXIT_CALLBACK
            UL_testing_Tuning_ExitCallback();
        #endif /* UL_testing_Tuning_EXIT_CALLBACK */
    }


    #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_GANGED_SNS_EN)
    /*******************************************************************************
    * Function Name: UL_testing_Tuning_CSDPostMultiScanGanged
    ****************************************************************************//**
    *
    * \brief
    *  This is an internal ISR function for the multiple-sensor scanning
    *  implementation for ganged sensors.
    *
    * \details
    *  This ISR handler is triggered when the user calls the
    *  UL_testing_Tuning_Scan() API for a ganged sensor or the
    *  UL_testing_Tuning_ScanAllWidgets() API in the project with ganged sensors.
    *
    *  The following tasks are performed:
    *    1. Disable the CSD interrupt.
    *    2. Read the Counter register and update the data structure with raw data.
    *    3. Connect the Vref buffer to the AMUX bus.
    *    4. Disable the CSD block (after the widget has been scanned).
    *    5. Update the Scan Counter.
    *    6. Reset the BUSY flag.
    *    7. Enable the CSD interrupt.
    *
    *  The ISR handler initializes scanning for the previous sensor when the
    *  widget has more than one sensor.
    *  The ISR handler initializes scanning for the next widget when the
    *  UL_testing_Tuning_ScanAllWidgets() APIs are called and the project has
    *  more than one widget.
    *  The ISR handler changes the IMO and initializes scanning for the next
    *  frequency channels when multi-frequency scanning is enabled.
    *
    *  This function has two Macro Callbacks that allow calling the user
    *  code from macros specified in Component's generated code. Refer to the
    *  \ref group_c_macrocallbacks section of the PSoC Creator User Guide
    *  for details.
    *
    *******************************************************************************/
    CY_ISR(UL_testing_Tuning_CSDPostMultiScanGanged)
    {
        #ifdef UL_testing_Tuning_ENTRY_CALLBACK
            UL_testing_Tuning_EntryCallback();
        #endif /* UL_testing_Tuning_ENTRY_CALLBACK */

        /* Clear pending interrupt */
        CY_SET_REG32(UL_testing_Tuning_INTR_PTR, UL_testing_Tuning_INTR_SET_MASK);

        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_NOISE_METRIC_EN)
            if ((UL_testing_Tuning_CSD_NOISE_METRIC_TH < ((CY_GET_REG32(UL_testing_Tuning_RESULT_VAL1_PTR) &
                                                      UL_testing_Tuning_RESULT_VAL1_BAD_CONVS_MASK) >>
                                                      UL_testing_Tuning_RESULT_VAL1_BAD_CONVS_SHIFT)) &&
                                                      (0u < UL_testing_Tuning_badConversionsNum))
            {
                /* Decrement bad conversions number */
                UL_testing_Tuning_badConversionsNum--;

                /* Start the re-scan */
                CY_SET_REG32(UL_testing_Tuning_SEQ_START_PTR, UL_testing_Tuning_SEQ_START_AZ0_SKIP_MASK |
                                                             UL_testing_Tuning_SEQ_START_AZ1_SKIP_MASK |
                                                             UL_testing_Tuning_SEQ_START_START_MASK);
            }
            else
            {
        #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_NOISE_METRIC_EN) */

            UL_testing_Tuning_SsCSDPostScan();

            #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_MULTI_FREQ_SCAN_EN)
                if (UL_testing_Tuning_FREQ_CHANNEL_2 == UL_testing_Tuning_scanFreqIndex)
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_MULTI_FREQ_SCAN_EN) */
            {
                UL_testing_Tuning_SsCSDDisconnectSnsExt((uint32)UL_testing_Tuning_widgetIndex, (uint32)UL_testing_Tuning_sensorIndex);
            }

            #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_MULTI_FREQ_SCAN_EN)
                if (UL_testing_Tuning_FREQ_CHANNEL_2 > UL_testing_Tuning_scanFreqIndex)
                {
                     /* Scan the next channel */
                    UL_testing_Tuning_SsNextFrequencyScan();
                }
                else
                {
                    /* All channels are scanned. Set IMO to zero channel */
                    UL_testing_Tuning_scanFreqIndex = UL_testing_Tuning_FREQ_CHANNEL_0;

                    #if (UL_testing_Tuning_MFS_IMO == UL_testing_Tuning_MFS_METHOD)
                        UL_testing_Tuning_SsChangeImoFreq(UL_testing_Tuning_FREQ_CHANNEL_0);
                    #else
                        UL_testing_Tuning_SsChangeClkFreq(UL_testing_Tuning_FREQ_CHANNEL_0);
                    #endif /* (UL_testing_Tuning_MFS_IMO == UL_testing_Tuning_MFS_METHOD) */

                     /* Scan the next sensor */
                    UL_testing_Tuning_SsCSDInitNextScan();
                }
            #else
                 /* Scan the next sensor */
                UL_testing_Tuning_SsCSDInitNextScan();
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_MULTI_FREQ_SCAN_EN) */
        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_NOISE_METRIC_EN)
            }
        #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_NOISE_METRIC_EN) */

        #ifdef UL_testing_Tuning_EXIT_CALLBACK
            UL_testing_Tuning_ExitCallback();
        #endif /* UL_testing_Tuning_EXIT_CALLBACK */
    }
    #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_GANGED_SNS_EN) */

#else

    /* Third-generation HW block part */

    /*******************************************************************************
    * Function Name: UL_testing_Tuning_CSDPostSingleScan
    ****************************************************************************//**
    *
    * \brief
    *  This is an internal ISR function for the single-sensor scanning implementation.
    *
    * \details
    *  This ISR handler is triggered when the user calls the
    *  UL_testing_Tuning_CSDScanExt() function.
    *
    *  The following tasks are performed for Third-generation HW block:
    *    1. Disable the CSD interrupt.
    *    2. Read the Counter register and update the data structure with raw data.
    *    3. Connect the Vref buffer to the AMUX bus.
    *    4. Update the Scan Counter.
    *    5. Reset the BUSY flag.
    *    6. Enable the CSD interrupt.
    *
    *  The following tasks are performed for Fourth-generation HW block:
    *    1. Check if the raw data is not noisy.
    *    2. Read the Counter register and update the data structure with raw data.
    *    3. Configure and start the scan for the next frequency if the
    *      multi-frequency is enabled.
    *    4. Update the Scan Counter.
    *    5. Reset the BUSY flag.
    *    6. Enable the CSD interrupt.
    *
    *  The ISR handler changes the IMO and initializes scanning for the next frequency
    *  channels when multi-frequency scanning is enabled.
    *
    *  This function has two Macro Callbacks that allow calling the user code
    *  from macros specified in Component's generated code. Refer to the
    *  \ref group_c_macrocallbacks section of the PSoC Creator User Guide
    *  for details.
    *
    *******************************************************************************/
    CY_ISR(UL_testing_Tuning_CSDPostSingleScan)
    {
        #ifdef UL_testing_Tuning_ENTRY_CALLBACK
            UL_testing_Tuning_EntryCallback();
        #endif /* UL_testing_Tuning_ENTRY_CALLBACK */

        /* Clear pending interrupt */
        CY_SET_REG32(UL_testing_Tuning_INTR_PTR, UL_testing_Tuning_INTR_SET_MASK);

        /* Read Rawdata */
        UL_testing_Tuning_SsCSDPostScan();

        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_MULTI_FREQ_SCAN_EN)
            if (UL_testing_Tuning_FREQ_CHANNEL_2 > UL_testing_Tuning_scanFreqIndex)
            {
                /* Connect Vref Buffer to AMUX bus. Third-generation HW block is enabled */
                CY_SET_REG32(UL_testing_Tuning_CONFIG_PTR, UL_testing_Tuning_configCsd | UL_testing_Tuning_CTANK_PRECHARGE_CONFIG_CSD_EN);

                UL_testing_Tuning_SsNextFrequencyScan();
            }
            else
            {
                UL_testing_Tuning_scanFreqIndex = UL_testing_Tuning_FREQ_CHANNEL_0;

                #if (UL_testing_Tuning_MFS_IMO == UL_testing_Tuning_MFS_METHOD)
                    UL_testing_Tuning_SsChangeImoFreq(UL_testing_Tuning_FREQ_CHANNEL_0);
                #else
                    UL_testing_Tuning_SsChangeClkFreq(UL_testing_Tuning_FREQ_CHANNEL_0);
                #endif /* (UL_testing_Tuning_MFS_IMO == UL_testing_Tuning_MFS_METHOD) */

                #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_BLOCK_OFF_AFTER_SCAN_EN)
                    /* Disable Third-generation HW block. Connect Vref Buffer to AMUX bus */
                    #if ((UL_testing_Tuning_CSH_PRECHARGE_IO_BUF == UL_testing_Tuning_CSD_CSH_PRECHARGE_SRC) &&\
                         (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_TANK_EN))
                        CY_SET_REG32(UL_testing_Tuning_CONFIG_PTR, UL_testing_Tuning_configCsd | UL_testing_Tuning_CMOD_PRECHARGE_CONFIG);
                    #else
                        CY_SET_REG32(UL_testing_Tuning_CONFIG_PTR, UL_testing_Tuning_configCsd | UL_testing_Tuning_CTANK_PRECHARGE_CONFIG);
                    #endif /* ((UL_testing_Tuning_CSH_PRECHARGE_IO_BUF == UL_testing_Tuning_CSD_CSH_PRECHARGE_SRC) &&\
                               (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_TANK_EN)) */
                #else
                    /* Connect Vref Buffer to AMUX bus. Third-generation HW block is enabled */
                    CY_SET_REG32(UL_testing_Tuning_CONFIG_PTR, UL_testing_Tuning_configCsd | UL_testing_Tuning_CTANK_PRECHARGE_CONFIG_CSD_EN);
                #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_BLOCK_OFF_AFTER_SCAN_EN) */

                /* Update Scan Counter */
                UL_testing_Tuning_dsRam.scanCounter++;

                /* Sensor is totally scanned. Reset BUSY flag */
                UL_testing_Tuning_dsRam.status &= ~(UL_testing_Tuning_SW_STS_BUSY | UL_testing_Tuning_WDGT_SW_STS_BUSY);
            }
        #else
            {
                #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_BLOCK_OFF_AFTER_SCAN_EN)
                    /* Disable Third-generation HW block. Connect Vref Buffer to AMUX bus */
                    #if ((UL_testing_Tuning_CSH_PRECHARGE_IO_BUF == UL_testing_Tuning_CSD_CSH_PRECHARGE_SRC) &&\
                         (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_TANK_EN))
                        CY_SET_REG32(UL_testing_Tuning_CONFIG_PTR, UL_testing_Tuning_configCsd | UL_testing_Tuning_CMOD_PRECHARGE_CONFIG);
                    #else
                        CY_SET_REG32(UL_testing_Tuning_CONFIG_PTR, UL_testing_Tuning_configCsd | UL_testing_Tuning_CTANK_PRECHARGE_CONFIG);
                    #endif /* ((UL_testing_Tuning_CSH_PRECHARGE_IO_BUF == UL_testing_Tuning_CSD_CSH_PRECHARGE_SRC) &&\
                               (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_TANK_EN)) */
                #else
                    /* Connect Vref Buffer to AMUX bus. Third-generation HW block is enabled */
                    CY_SET_REG32(UL_testing_Tuning_CONFIG_PTR, UL_testing_Tuning_configCsd | UL_testing_Tuning_CTANK_PRECHARGE_CONFIG_CSD_EN);
                #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_BLOCK_OFF_AFTER_SCAN_EN) */

                /* Update Scan Counter */
                UL_testing_Tuning_dsRam.scanCounter++;

                /* Sensor is totally scanned. Reset BUSY flag */
                UL_testing_Tuning_dsRam.status &= ~(UL_testing_Tuning_SW_STS_BUSY | UL_testing_Tuning_WDGT_SW_STS_BUSY);
            }
        #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_MULTI_FREQ_SCAN_EN) */

        #ifdef UL_testing_Tuning_EXIT_CALLBACK
            UL_testing_Tuning_ExitCallback();
        #endif /* UL_testing_Tuning_EXIT_CALLBACK */
    }


    /*******************************************************************************
    * Function Name: UL_testing_Tuning_CSDPostMultiScan
    ****************************************************************************//**
    *
    * \brief
    *  This is an internal ISR function for the multiple-sensor scanning
    *  implementation.
    *
    * \details
    *  This ISR handler is triggered when the user calls the
    *  UL_testing_Tuning_Scan() or UL_testing_Tuning_ScanAllWidgets() APIs.
    *
    *  The following tasks are performed:
    *    1. Disable the CSD interrupt.
    *    2. Read the Counter register and update the data structure with raw data.
    *    3. Connect the Vref buffer to the AMUX bus.
    *    4. Disable the CSD block (after the widget has been scanned).
    *    5. Update the Scan Counter.
    *    6. Reset the BUSY flag.
    *    7. Enable the CSD interrupt.
    *
    *  The ISR handler initializes scanning for the previous sensor when the
    *  widget has more than one sensor.
    *  The ISR handler initializes scanning for the next widget when the
    *  UL_testing_Tuning_ScanAllWidgets() APIs are called and the project has
    *  more than one widget.
    *  The ISR handler changes the IMO and initializes scanning for the next
    *  frequency channels when multi-frequency scanning is enabled.
    *
    *  This function has two Macro Callbacks that allow calling the user
    *  code from macros specified in Component's generated code. Refer to the
    *  \ref group_c_macrocallbacks section of the PSoC Creator User Guide
    *  for details.
    *
    *******************************************************************************/
    CY_ISR(UL_testing_Tuning_CSDPostMultiScan)
    {
        /* Declare and initialize ptr to sensor IO structure to appropriate address        */
        UL_testing_Tuning_FLASH_IO_STRUCT const *curSnsIOPtr = (UL_testing_Tuning_FLASH_IO_STRUCT const *)
                                                          UL_testing_Tuning_dsFlash.wdgtArray[UL_testing_Tuning_widgetIndex].ptr2SnsFlash
                                                          + UL_testing_Tuning_sensorIndex;

        #ifdef UL_testing_Tuning_ENTRY_CALLBACK
            UL_testing_Tuning_EntryCallback();
        #endif /* UL_testing_Tuning_ENTRY_CALLBACK */

        /* Clear pending interrupt */
        CY_SET_REG32(UL_testing_Tuning_INTR_PTR, UL_testing_Tuning_INTR_SET_MASK);

         /* Read Rawdata */
        UL_testing_Tuning_SsCSDPostScan();

        /* Connect Vref Buffer to AMUX bus */
        CY_SET_REG32(UL_testing_Tuning_CONFIG_PTR, UL_testing_Tuning_configCsd | UL_testing_Tuning_CTANK_PRECHARGE_CONFIG_CSD_EN);

        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_MULTI_FREQ_SCAN_EN)
            /* Disable sensor when all frequency channels are scanned */
            if (UL_testing_Tuning_FREQ_CHANNEL_2 == UL_testing_Tuning_scanFreqIndex)
            {
                /* Disable sensor */
                UL_testing_Tuning_CSDDisconnectSns(curSnsIOPtr);
            }
        #else
            /* Disable sensor */
            UL_testing_Tuning_CSDDisconnectSns(curSnsIOPtr);
        #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_MULTI_FREQ_SCAN_EN) */

        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_MULTI_FREQ_SCAN_EN)
            if (UL_testing_Tuning_FREQ_CHANNEL_2 > UL_testing_Tuning_scanFreqIndex)
            {
                 /* Scan the next channel */
                UL_testing_Tuning_SsNextFrequencyScan();
            }
            else
            {
                 /* All channels are scanned. Set IMO to zero channel */
                UL_testing_Tuning_scanFreqIndex = UL_testing_Tuning_FREQ_CHANNEL_0;

                #if (UL_testing_Tuning_MFS_IMO == UL_testing_Tuning_MFS_METHOD)
                    UL_testing_Tuning_SsChangeImoFreq(UL_testing_Tuning_FREQ_CHANNEL_0);
                #else
                    UL_testing_Tuning_SsChangeClkFreq(UL_testing_Tuning_FREQ_CHANNEL_0);
                #endif /* (UL_testing_Tuning_MFS_IMO == UL_testing_Tuning_MFS_METHOD) */

                 /* Scan the next sensor */
                UL_testing_Tuning_SsCSDInitNextScan();
            }
        #else
            /* Scan the next sensor */
            UL_testing_Tuning_SsCSDInitNextScan();
        #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_MULTI_FREQ_SCAN_EN) */

        #ifdef UL_testing_Tuning_EXIT_CALLBACK
            UL_testing_Tuning_ExitCallback();
        #endif /* UL_testing_Tuning_EXIT_CALLBACK */
    }


    #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_GANGED_SNS_EN)
    /*******************************************************************************
    * Function Name: UL_testing_Tuning_CSDPostMultiScanGanged
    ****************************************************************************//**
    *
    * \brief
    *  This is an internal ISR function for the multiple-sensor scanning
    *  implementation for ganged sensors.
    *
    * \details
    *  This ISR handler is triggered when the user calls the
    *  UL_testing_Tuning_Scan() API for a ganged sensor or the
    *  UL_testing_Tuning_ScanAllWidgets() API in the project with ganged sensors.
    *
    *  The following tasks are performed:
    *    1. Disable the CSD interrupt.
    *    2. Read the Counter register and update the data structure with raw data.
    *    3. Connect the Vref buffer to the AMUX bus.
    *    4. Disable the CSD block (after the widget has been scanned).
    *    5. Update the Scan Counter.
    *    6. Reset the BUSY flag.
    *    7. Enable the CSD interrupt.
    *
    *  The ISR handler initializes scanning for the previous sensor when the
    *  widget has more than one sensor.
    *  The ISR handler initializes scanning for the next widget when the
    *  UL_testing_Tuning_ScanAllWidgets() APIs are called and the project has
    *  more than one widget.
    *  The ISR handler changes the IMO and initializes scanning for the next
    *  frequency channels when multi-frequency scanning is enabled.
    *
    *  This function has two Macro Callbacks that allow calling the user
    *  code from macros specified in Component's generated code. Refer to the
    *  \ref group_c_macrocallbacks section of the PSoC Creator User Guide
    *  for details.
    *
    *******************************************************************************/
    CY_ISR(UL_testing_Tuning_CSDPostMultiScanGanged)
    {
        #ifdef UL_testing_Tuning_ENTRY_CALLBACK
            UL_testing_Tuning_EntryCallback();
        #endif /* UL_testing_Tuning_ENTRY_CALLBACK */

        /* Clear pending interrupt */
        CY_SET_REG32(UL_testing_Tuning_INTR_PTR, UL_testing_Tuning_INTR_SET_MASK);

         /* Read Rawdata */
        UL_testing_Tuning_SsCSDPostScan();

        /* Connect Vref Buffer to AMUX bus */
        CY_SET_REG32(UL_testing_Tuning_CONFIG_PTR, UL_testing_Tuning_configCsd | UL_testing_Tuning_CTANK_PRECHARGE_CONFIG_CSD_EN);

        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_MULTI_FREQ_SCAN_EN)
            if (UL_testing_Tuning_FREQ_CHANNEL_2 == UL_testing_Tuning_scanFreqIndex)
            {
                UL_testing_Tuning_SsCSDDisconnectSnsExt((uint32)UL_testing_Tuning_widgetIndex, (uint32)UL_testing_Tuning_sensorIndex);
            }
        #else
            UL_testing_Tuning_SsCSDDisconnectSnsExt((uint32)UL_testing_Tuning_widgetIndex, (uint32)UL_testing_Tuning_sensorIndex);
        #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_MULTI_FREQ_SCAN_EN) */

        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_MULTI_FREQ_SCAN_EN)
            if (UL_testing_Tuning_FREQ_CHANNEL_2 > UL_testing_Tuning_scanFreqIndex)
            {
                 /* Scan the next channel */
                UL_testing_Tuning_SsNextFrequencyScan();
            }
            else
            {
                /* All channels are scanned. Set IMO to zero channel */
                UL_testing_Tuning_scanFreqIndex = UL_testing_Tuning_FREQ_CHANNEL_0;

                #if (UL_testing_Tuning_MFS_IMO == UL_testing_Tuning_MFS_METHOD)
                    UL_testing_Tuning_SsChangeImoFreq(UL_testing_Tuning_FREQ_CHANNEL_0);
                #else
                    UL_testing_Tuning_SsChangeClkFreq(UL_testing_Tuning_FREQ_CHANNEL_0);
                #endif /* (UL_testing_Tuning_MFS_IMO == UL_testing_Tuning_MFS_METHOD) */

                 /* Scan the next sensor */
                UL_testing_Tuning_SsCSDInitNextScan();
            }
        #else
             /* Scan the next sensor */
            UL_testing_Tuning_SsCSDInitNextScan();
        #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_MULTI_FREQ_SCAN_EN) */

        #ifdef UL_testing_Tuning_EXIT_CALLBACK
            UL_testing_Tuning_ExitCallback();
        #endif /* UL_testing_Tuning_EXIT_CALLBACK */
    }
    #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_GANGED_SNS_EN) */

#endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2) */

#endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN) || (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_CSX_EN)) */

/** \}
 * \endcond */


#if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN) || (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_CSX_EN))

/*******************************************************************************
* Function Name: UL_testing_Tuning_SsCSDPostScan
****************************************************************************//**
*
* \brief
*   This function reads rawdata and releases required HW resources after scan.
*
* \details
*   This function performs following tasks after scan:
*   - Reads SlotResult from Raw Counter;
*   - Inits bad Conversions number;
*   - Disconnects Vrefhi from AMUBUF positive input;
*   - Disconnects AMUBUF output from CSDBUSB with sych PHI2+HSCMP;
*   - Opens HCBV and HCBG switches.
*
*******************************************************************************/
static void UL_testing_Tuning_SsCSDPostScan(void)
{
    #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2)

        uint32 tmpRawData;
        uint32 tmpMaxCount;
        UL_testing_Tuning_RAM_WD_BASE_STRUCT const *ptrWdgt = (UL_testing_Tuning_RAM_WD_BASE_STRUCT *)
                                            UL_testing_Tuning_dsFlash.wdgtArray[UL_testing_Tuning_widgetIndex].ptr2WdgtRam;

        /* Read SlotResult from Raw Counter */
        tmpRawData = UL_testing_Tuning_RESULT_VAL1_VALUE_MASK & CY_GET_REG32(UL_testing_Tuning_COUNTER_PTR);

        tmpMaxCount = ((1uL << ptrWdgt->resolution) - 1uL);
        if(tmpRawData < tmpMaxCount)
        {
            UL_testing_Tuning_curRamSnsPtr->raw[UL_testing_Tuning_scanFreqIndex] = LO16(tmpRawData);
        }
        else
        {
            UL_testing_Tuning_curRamSnsPtr->raw[UL_testing_Tuning_scanFreqIndex] = LO16(tmpMaxCount);
        }

        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_NOISE_METRIC_EN)
            /* Init bad Conversions number */
            UL_testing_Tuning_badConversionsNum = UL_testing_Tuning_BAD_CONVERSIONS_NUM;
        #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_NOISE_METRIC_EN) */

        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN)
            /* Open HCBV and HCBG switches */
            CY_SET_REG32(UL_testing_Tuning_SW_SHIELD_SEL_PTR, UL_testing_Tuning_SW_SHIELD_SEL_SW_HCBV_STATIC_OPEN |
                                                             UL_testing_Tuning_SW_SHIELD_SEL_SW_HCBG_STATIC_OPEN);
        #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) */

    #else

        /* Read SlotResult from Raw Counter */
       UL_testing_Tuning_curRamSnsPtr->raw[UL_testing_Tuning_scanFreqIndex] = (uint16)CY_GET_REG32(UL_testing_Tuning_COUNTER_PTR);

    #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2) */
}


/*******************************************************************************
* Function Name: UL_testing_Tuning_SsCSDInitNextScan
****************************************************************************//**
*
* \brief
*   This function initializes the next sensor scan.
*
* \details
*   The function increments the sensor index, updates sense clock for matrix
*   or touchpad widgets only, sets up Compensation IDAC, enables the sensor and
*   scans it. When all the sensors are scanned it continues to set up the next widget
*   until all the widgets are scanned. The CSD block is disabled when all the widgets are
*   scanned.
*
*******************************************************************************/
static void UL_testing_Tuning_SsCSDInitNextScan(void)
{
    /* Declare and initialize ptr to widget and sensor structures to appropriate address */
    #if (((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_IDAC_COMP_EN) || \
          (UL_testing_Tuning_CSD_MATRIX_WIDGET_EN || UL_testing_Tuning_CSD_TOUCHPAD_WIDGET_EN)) || \
          (((UL_testing_Tuning_DISABLE == UL_testing_Tuning_CSD_COMMON_SNS_CLK_EN) && \
          (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2) && \
          (UL_testing_Tuning_CSD_MATRIX_WIDGET_EN || UL_testing_Tuning_CSD_TOUCHPAD_WIDGET_EN))))
        UL_testing_Tuning_RAM_WD_BASE_STRUCT *ptrWdgt = (UL_testing_Tuning_RAM_WD_BASE_STRUCT *)
                                                        UL_testing_Tuning_dsFlash.wdgtArray[UL_testing_Tuning_widgetIndex].ptr2WdgtRam;
    #endif

    /* Check if all the sensors are scanned in widget */
    if (((uint8)UL_testing_Tuning_dsFlash.wdgtArray[(UL_testing_Tuning_widgetIndex)].totalNumSns - 1u) > UL_testing_Tuning_sensorIndex)
    {
        /* Increment sensor index to configure next sensor in widget */
        UL_testing_Tuning_sensorIndex++;

        /* Update global pointer to UL_testing_Tuning_RAM_SNS_STRUCT to current sensor  */
        UL_testing_Tuning_curRamSnsPtr = (UL_testing_Tuning_RAM_SNS_STRUCT *)
                                                  UL_testing_Tuning_dsFlash.wdgtArray[UL_testing_Tuning_widgetIndex].ptr2SnsRam
                                                  + UL_testing_Tuning_sensorIndex;

        /* Configure clock divider to row or column */
        #if ((UL_testing_Tuning_DISABLE == UL_testing_Tuning_CSD_COMMON_SNS_CLK_EN) && \
             (UL_testing_Tuning_CSD_MATRIX_WIDGET_EN || UL_testing_Tuning_CSD_TOUCHPAD_WIDGET_EN))
            if ((UL_testing_Tuning_WD_TOUCHPAD_E == (UL_testing_Tuning_WD_TYPE_ENUM)UL_testing_Tuning_dsFlash.wdgtArray[(UL_testing_Tuning_widgetIndex)].wdgtType) ||
                (UL_testing_Tuning_WD_MATRIX_BUTTON_E == (UL_testing_Tuning_WD_TYPE_ENUM)UL_testing_Tuning_dsFlash.wdgtArray[(UL_testing_Tuning_widgetIndex)].wdgtType))
            {
                UL_testing_Tuning_SsCSDConfigClock();

                #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2)
                     /* Set up scanning resolution */
                    UL_testing_Tuning_SsCSDCalculateScanDuration(ptrWdgt);
                #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2) */
            }
        #endif /* ((UL_testing_Tuning_DISABLE == UL_testing_Tuning_CSD_COMMON_SNS_CLK_EN) && \
                   (UL_testing_Tuning_CSD_MATRIX_WIDGET_EN || UL_testing_Tuning_CSD_TOUCHPAD_WIDGET_EN))) */

        /* Setup Compensation IDAC for next sensor in widget */
        #if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_IDAC_COMP_EN) || \
             (UL_testing_Tuning_CSD_MATRIX_WIDGET_EN || UL_testing_Tuning_CSD_TOUCHPAD_WIDGET_EN))
            UL_testing_Tuning_SsCSDSetUpIdacs(ptrWdgt);
        #endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_IDAC_COMP_EN) || \
                   (UL_testing_Tuning_CSD_MATRIX_WIDGET_EN || UL_testing_Tuning_CSD_TOUCHPAD_WIDGET_EN)) */

        /* Enable sensor */
        UL_testing_Tuning_SsCSDConnectSensorExt((uint32)UL_testing_Tuning_widgetIndex, (uint32)UL_testing_Tuning_sensorIndex);

        /* Proceed scanning */
        UL_testing_Tuning_SsCSDStartSample();
    }
    /* Call scan next widget API if requested, if not, complete the scan  */
    else
    {
        UL_testing_Tuning_sensorIndex = 0u;

        /* Current widget is totally scanned. Reset WIDGET BUSY flag */
        UL_testing_Tuning_dsRam.status &= ~UL_testing_Tuning_WDGT_SW_STS_BUSY;

        /* Check if all the widgets have been scanned */
        if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_requestScanAllWidget)
        {
            /* Configure and begin scanning next widget */
            UL_testing_Tuning_SsPostAllWidgetsScan();
        }
        else
        {
            #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_BLOCK_OFF_AFTER_SCAN_EN)
                #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2)
                    /* Disable the CSD block */
                    CY_SET_REG32(UL_testing_Tuning_CONFIG_PTR, UL_testing_Tuning_configCsd);
                #else
                    /* Disable the CSD block. Connect Vref Buffer to AMUX bus */
                    #if ((UL_testing_Tuning_CSH_PRECHARGE_IO_BUF == UL_testing_Tuning_CSD_CSH_PRECHARGE_SRC) &&\
                         (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_TANK_EN))
                        CY_SET_REG32(UL_testing_Tuning_CONFIG_PTR, UL_testing_Tuning_configCsd | UL_testing_Tuning_CMOD_PRECHARGE_CONFIG);
                    #else
                        CY_SET_REG32(UL_testing_Tuning_CONFIG_PTR, UL_testing_Tuning_configCsd | UL_testing_Tuning_CTANK_PRECHARGE_CONFIG);
                    #endif /* ((UL_testing_Tuning_CSH_PRECHARGE_IO_BUF == UL_testing_Tuning_CSD_CSH_PRECHARGE_SRC) &&\
                               (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_TANK_EN)) */
                #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2) */
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_BLOCK_OFF_AFTER_SCAN_EN) */

            /* All widgets are totally scanned. Reset BUSY flag */
            UL_testing_Tuning_dsRam.status &= ~UL_testing_Tuning_SW_STS_BUSY;

            /* Update scan Counter */
            UL_testing_Tuning_dsRam.scanCounter++;
        }
    }
}

#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_MULTI_FREQ_SCAN_EN)
    /*******************************************************************************
    * Function Name: UL_testing_Tuning_SsNextFrequencyScan
    ****************************************************************************//**
    *
    * \brief
    *   This function scans the sensor on the next frequency channel.
    *
    * \details
    *   The function increments the frequency channel, changes IMO and initializes
    *   the scanning process of the same sensor.
    *
    *******************************************************************************/
    static void UL_testing_Tuning_SsNextFrequencyScan(void)
    {
        UL_testing_Tuning_RAM_WD_BASE_STRUCT const *ptrWdgt = (UL_testing_Tuning_RAM_WD_BASE_STRUCT *)
                                                        UL_testing_Tuning_dsFlash.wdgtArray[UL_testing_Tuning_widgetIndex].ptr2WdgtRam;

        UL_testing_Tuning_scanFreqIndex++;

        /* Set Immunity */
        #if (UL_testing_Tuning_MFS_IMO == UL_testing_Tuning_MFS_METHOD)
            UL_testing_Tuning_SsChangeImoFreq((uint32)UL_testing_Tuning_scanFreqIndex);
        #else
            UL_testing_Tuning_SsChangeClkFreq((uint32)UL_testing_Tuning_scanFreqIndex);
        #endif /* (UL_testing_Tuning_MFS_IMO == UL_testing_Tuning_MFS_METHOD) */

        /* Update IDAC registers */
        UL_testing_Tuning_SsCSDSetUpIdacs(ptrWdgt);

        /* Proceed scanning */
        UL_testing_Tuning_SsCSDStartSample();
    }
#endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_MULTI_FREQ_SCAN_EN) */

#endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN) || (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_CSX_EN)) */


/* [] END OF FILE */
