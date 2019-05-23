/***************************************************************************//**
* \file UL_testing_Tuning_Sensing.c
* \version 6.0
*
* \brief
*   This file contains the source of functions common for
*   different sensing methods.
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

#include "cyfitter.h"
#include "UL_testing_Tuning_Configuration.h"
#include "UL_testing_Tuning_Structure.h"
#include "UL_testing_Tuning_Sensing.h"
#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN)
    #include "UL_testing_Tuning_SensingCSX_LL.h"
#endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN) */
#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_ISX_EN)
    #include "UL_testing_Tuning_SensingISX_LL.h"
#endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_ISX_EN) */
#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN)
    #include "UL_testing_Tuning_SensingCSD_LL.h"
#endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN) */
#if (UL_testing_Tuning_CSD_SS_DIS != UL_testing_Tuning_CSD_AUTOTUNE)
    #include "UL_testing_Tuning_SmartSense_LL.h"
#endif  /* (UL_testing_Tuning_CSD_SS_DIS != UL_testing_Tuning_CSD_AUTOTUNE) */
#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_ADC_EN)
    #include "UL_testing_Tuning_Adc.h"
#endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_ADC_EN) */
#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_SELF_TEST_EN)
    #include "UL_testing_Tuning_SelfTest.h"
#endif

#if (0u != UL_testing_Tuning_ISX_EN)
    #include "UL_testing_Tuning_DigPrt2.h"
    #if (UL_testing_Tuning_USES_PORT_3)
        #include "UL_testing_Tuning_DigPrt3.h"
    #endif
#endif /* (0u != UL_testing_Tuning_ISX_EN) */
/***************************************
* API Constants
***************************************/

#define UL_testing_Tuning_IMO_FREQUENCY_OFFSET_MIN               (0u)
#define UL_testing_Tuning_IMO_FREQUENCY_OFFSET_MAX               (255u)
#define UL_testing_Tuning_CALIBRATION_RESOLUTION                 (12u)
#define UL_testing_Tuning_CALIBRATION_FREQ_KHZ                   (1500u)
#define UL_testing_Tuning_CSD_AUTOTUNE_CAL_LEVEL                 (UL_testing_Tuning_CSD_RAWCOUNT_CAL_LEVEL)
#define UL_testing_Tuning_CSD_AUTOTUNE_CAL_UNITS                 (1000u)
#define UL_testing_Tuning_CP_MIN                                 (0u)
#define UL_testing_Tuning_CP_MAX                                 (65000Lu)
#define UL_testing_Tuning_CP_ERROR                               (4000Lu)
#define UL_testing_Tuning_CLK_SOURCE_LFSR_SCALE_OFFSET           (4u)

#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2)
    #define UL_testing_Tuning_CSD_SNS_FREQ_KHZ_MAX               (6000u)
#else
    #define UL_testing_Tuning_CSD_SNS_FREQ_KHZ_MAX               (12600u)
#endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2) */

#define UL_testing_Tuning_FLIP_FLOP_DIV                          (1u)

#define UL_testing_Tuning_MOD_CSD_CLK_12000KHZ                   (12000uL)
#define UL_testing_Tuning_MOD_CSD_CLK_24000KHZ                   (24000uL)
#define UL_testing_Tuning_MOD_CSD_CLK_48000KHZ                   (48000uL)

/*****************************************************************************/
/* Enumeration types definition                                              */
/*****************************************************************************/

typedef enum
{
    UL_testing_Tuning_RES_PULLUP_E   = 0x02u,
    UL_testing_Tuning_RES_PULLDOWN_E = 0x03u
} UL_testing_Tuning_PORT_TEST_DM;

typedef enum
{
    UL_testing_Tuning_STS_RESET      = 0x01u,
    UL_testing_Tuning_STS_NO_RESET   = 0x02u
} UL_testing_Tuning_TEST_TYPE;


/*******************************************************************************
* Static Function Prototypes
*******************************************************************************/
/**
* \cond SECTION_C_INTERNAL
* \addtogroup group_c_internal
* \{
*/

#if (UL_testing_Tuning_DISABLE == UL_testing_Tuning_CSDV2)
    static void UL_testing_Tuning_SsTrimIdacs(void);
    #if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN) || \
         (UL_testing_Tuning_ENABLE == UL_testing_Tuning_ISX_EN) || \
         (UL_testing_Tuning_IDAC_SINKING == UL_testing_Tuning_CSD_IDAC_CONFIG))
        static void UL_testing_Tuning_SsTrimIdacsSinking(void);
    #endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN) || \
               (UL_testing_Tuning_IDAC_SINKING == UL_testing_Tuning_CSD_IDAC_CONFIG)) */
    #if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN) || \
         (UL_testing_Tuning_ENABLE == UL_testing_Tuning_ISX_EN) || \
         (UL_testing_Tuning_IDAC_SOURCING == UL_testing_Tuning_CSD_IDAC_CONFIG))
        static void UL_testing_Tuning_SsTrimIdacsSourcing(void);
    #endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN) || \
               (UL_testing_Tuning_IDAC_SOURCING == UL_testing_Tuning_CSD_IDAC_CONFIG)) */
#endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2) */

#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN)
    static void UL_testing_Tuning_SsCSDDisableMode(void);
#endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN) */
#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN)
    static void UL_testing_Tuning_SsDisableCSXMode(void);
#endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN) */
#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_ISX_EN)
    static void UL_testing_Tuning_SsDisableISXMode(void);
#endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_ISX_EN) */

#if(((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN) && \
     (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2) && \
     (UL_testing_Tuning_CLK_SOURCE_PRSAUTO == UL_testing_Tuning_CSX_TX_CLK_SOURCE) && \
     (UL_testing_Tuning_DISABLE == UL_testing_Tuning_CSX_SKIP_OVSMPL_SPECIFIC_NODES)) ||\
    ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN) && \
     (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2) && \
     (UL_testing_Tuning_CLK_SOURCE_PRSAUTO == UL_testing_Tuning_CSD_SNS_CLK_SOURCE)))
    static uint8 UL_testing_Tuning_SsCalcLfsrSize(uint32 snsClkDivider, uint32 conversionsNum);
    static uint8 UL_testing_Tuning_SsCalcLfsrScale(uint32 snsClkDivider, uint8 lfsrSize);
#endif

#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN)
    static void UL_testing_Tuning_SsSetWidgetSenseClkSrc(uint32 wdgtIndex, UL_testing_Tuning_RAM_WD_BASE_STRUCT * ptrWdgt);
#endif

#if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN) && (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2))
    static void UL_testing_Tuning_SsSetWidgetTxClkSrc(uint32 wdgtIndex, UL_testing_Tuning_RAM_WD_BASE_STRUCT * ptrWdgt);
#endif

/** \}
* \endcond */

/*******************************************************************************
* Defines module variables
*******************************************************************************/

UL_testing_Tuning_SENSE_METHOD_ENUM UL_testing_Tuning_currentSenseMethod = UL_testing_Tuning_UNDEFINED_E;

#if(UL_testing_Tuning_ENABLE == UL_testing_Tuning_MULTI_FREQ_SCAN_EN)
    /* Module variable keep track of multi-frequency scan channel index */
    uint8 UL_testing_Tuning_scanFreqIndex = 0u;
    /* Variable keep frequency offsets */
    uint8 UL_testing_Tuning_immunity[UL_testing_Tuning_NUM_SCAN_FREQS] = {0u, 0u, 0u};
#else
    /* const allows C-compiler to do optimization */
    const uint8 UL_testing_Tuning_scanFreqIndex = 0u;
#endif

/* Global software variables */
volatile uint8 UL_testing_Tuning_widgetIndex = 0u;    /* Index of the scanning widget */
volatile uint8 UL_testing_Tuning_sensorIndex = 0u;    /* Index of the scanning sensor */
uint8 UL_testing_Tuning_requestScanAllWidget = 0u;

/* Pointer to RAM_SNS_STRUCT structure */
UL_testing_Tuning_RAM_SNS_STRUCT *UL_testing_Tuning_curRamSnsPtr;

#if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_GANGED_SNS_EN) || \
     (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN) || \
     (UL_testing_Tuning_ENABLE == UL_testing_Tuning_ISX_EN))
    /* Pointer to Flash structure holding configuration of widget to be scanned */
    UL_testing_Tuning_FLASH_WD_STRUCT const *UL_testing_Tuning_curFlashWdgtPtr = 0u;
#endif

#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_GANGED_SNS_EN)
    /* Pointer to Flash structure holding info of sensor to be scanned */
    UL_testing_Tuning_FLASH_SNS_STRUCT const *UL_testing_Tuning_curFlashSnsPtr = 0u;
#endif

/* Pointer to Flash structure to hold Sns electrode that was connected previously */
UL_testing_Tuning_FLASH_IO_STRUCT const *UL_testing_Tuning_curSnsIOPtr;


/*******************************************************************************
* Function Name: UL_testing_Tuning_IsBusy
****************************************************************************//**
*
* \brief
*  Returns the current status of the Component (Scan is completed or Scan is in
*  progress).
*
* \details
*  This function returns a status of the hardware block whether a scan is
*  currently in progress or not. If the Component is busy, no new scan or setup
*  widgets is made. The critical section (i.e. disable global interrupt)
*  is recommended for the application when the device transitions from
*  the active mode to sleep or deep sleep modes.
*
* \return
*  Returns the current status of the Component:
*    - UL_testing_Tuning_NOT_BUSY - No scan is in progress and a next scan
*      can be initiated.
*    - UL_testing_Tuning_SW_STS_BUSY - The previous scanning is not completed
*      and the hardware block is busy.
*
*******************************************************************************/
uint32 UL_testing_Tuning_IsBusy(void)
{
    return (UL_testing_Tuning_dsRam.status & UL_testing_Tuning_SW_STS_BUSY);
}

/*******************************************************************************
* Function Name: UL_testing_Tuning_SetupWidget
****************************************************************************//**
*
* \brief
*  Performs the initialization required to scan the specified widget.
*
* \details
*  This function prepares the Component to scan all the sensors in the specified
*  widget by executing the following tasks:
*    1. Re-initialize the hardware if it is not configured to perform the
*       sensing method used by the specified widget, this happens only if multiple
*       sensing methods are used in the Component.
*    2. Initialize the hardware with specific sensing configuration (e.g.
*       sensor clock, scan resolution) used by the widget.
*    3. Disconnect all previously connected electrodes, if the electrodes
*       connected by the
*       \if SECTION_C_HIGH_LEVEL
*       UL_testing_Tuning_CSDSetupWidgetExt(), UL_testing_Tuning_CSXSetupWidgetExt(),
*       UL_testing_Tuning_CSDConnectSns() functions and not disconnected.
*       \endif
*       \if SECTION_I_REP
*       UL_testing_Tuning_ISXSetupWidgetExt() function and not disconnected.
*       \endif
*
*  This function does not start sensor scanning, the UL_testing_Tuning_Scan()
*  function must be called to start the scan sensors in the widget. If this
*  function is called more than once, it does not break the Component operation,
*  but only the last initialized widget is in effect.
*
* \param widgetId
*  Specifies the ID number of the widget to be initialized for scanning.
*  A macro for the widget ID can be found in the
*  UL_testing_Tuning Configuration header file defined as
*  UL_testing_Tuning_<WidgetName>_WDGT_ID.
*
* \return
*  Returns the status of the widget setting up operation:
*    - CYRET_SUCCESS - The operation is successfully completed.
*    - CYRET_BAD_PARAM - The widget is invalid or if the specified widget is
*      disabled
*    - CYRET_INVALID_STATE - The previous scanning is not completed and the
*      hardware block is busy.
*    - CYRET_UNKNOWN - An unknown sensing method is used by the widget or any
*      other spurious error occurred.
*
**********************************************************************************/
cystatus UL_testing_Tuning_SetupWidget(uint32 widgetId)
{
    cystatus widgetStatus;

    if (UL_testing_Tuning_WDGT_SW_STS_BUSY == (UL_testing_Tuning_dsRam.status & UL_testing_Tuning_WDGT_SW_STS_BUSY))
    {
        /* Previous widget is being scanned. Return error. */
        widgetStatus = CYRET_INVALID_STATE;
    }
    /*
     *  Check if widget id is valid, specified widget is enabled and widget did not
     *  detect any fault conditions if BIST is enabled. If all conditions are met,
     *  set widgetStatus as good, if not, set widgetStatus as bad.
     */
    else if ((UL_testing_Tuning_TOTAL_WIDGETS > widgetId) &&
        (0uL != UL_testing_Tuning_GET_WIDGET_EN_STATUS(widgetId)))

    {
        widgetStatus = CYRET_SUCCESS;
    }
    else
    {
        widgetStatus = CYRET_BAD_PARAM;
    }

    /*
     * Check widgetStatus flag that is set earlier, if flag is good, then set up only
     * widget
     */
    if (CYRET_SUCCESS == widgetStatus)
    {
        switch (UL_testing_Tuning_GET_SENSE_METHOD(&UL_testing_Tuning_dsFlash.wdgtArray[widgetId]))
        {
        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN)
            case UL_testing_Tuning_SENSE_METHOD_CSX_E:
                /* Set up widget for CSX scan */
                UL_testing_Tuning_CSXSetupWidget(widgetId);
                break;
        #endif
        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN)
            case UL_testing_Tuning_SENSE_METHOD_CSD_E:
                /* Set up widget for CSD scan */
                UL_testing_Tuning_CSDSetupWidget(widgetId);
                break;
        #endif
        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_ISX_EN)
            case UL_testing_Tuning_SENSE_METHOD_ISX_E:
                /* Set up widget for ISX scan */
                UL_testing_Tuning_ISXSetupWidget(widgetId);
                break;
        #endif
        default:
            /* Sensing method is invalid, return error to caller */
            widgetStatus = CYRET_UNKNOWN;
            break;
        }
    }
    return (widgetStatus);
}


/*******************************************************************************
* Function Name: UL_testing_Tuning_Scan
****************************************************************************//**
*
* \brief
*  Initiates scanning of all the sensors in the widget initialized by
*  UL_testing_Tuning_SetupWidget(), if no scan is in progress.
*
* \details
*  This function is called only after the UL_testing_Tuning_SetupWidget()
*  function is called to start the scanning of the sensors in the widget. The
*  status of a sensor scan must be checked using the UL_testing_Tuning_IsBusy()
*  API prior to starting a next scan or setting up another widget.
*
* \return
*  Returns the status of the scan initiation operation:
*  - CYRET_SUCCESS - Scanning is successfully started.
*  - CYRET_INVALID_STATE - The previous scanning is not completed and the
*    hardware block is busy.
*  - CYRET_UNKNOWN - An unknown sensing method is used by the widget.
*
********************************************************************************/
cystatus UL_testing_Tuning_Scan(void)
{
    cystatus scanStatus = CYRET_SUCCESS;

    if (UL_testing_Tuning_WDGT_SW_STS_BUSY == (UL_testing_Tuning_dsRam.status & UL_testing_Tuning_WDGT_SW_STS_BUSY))
    {
        /* Previous widget is being scanned. Return error. */
        scanStatus = CYRET_INVALID_STATE;
    }
    else
    {
        switch (UL_testing_Tuning_currentSenseMethod)
        {
        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN)
            case UL_testing_Tuning_SENSE_METHOD_CSX_E:
                UL_testing_Tuning_CSXScan();
                break;
        #endif
        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN)
            case UL_testing_Tuning_SENSE_METHOD_CSD_E:
                UL_testing_Tuning_CSDScan();
                break;
        #endif
        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_ISX_EN)
            case UL_testing_Tuning_SENSE_METHOD_ISX_E:
                UL_testing_Tuning_ISXScan();
                break;
        #endif
        default:
            scanStatus = CYRET_UNKNOWN;
            break;
        }
    }
    return (scanStatus);
}


/*******************************************************************************
* Function Name: UL_testing_Tuning_ScanAllWidgets
****************************************************************************//**
*
* \brief
*  Initializes the first enabled widget and scanning of all the sensors in the
*  widget, then the same process is repeated for all the widgets in the Component,
*  i.e. scanning of all the widgets in the Component.
*
* \details
*  This function initializes a widget and scans all the sensors in the widget,
*  and then repeats the same for all the widgets in the Component. The tasks of
*  the UL_testing_Tuning_SetupWidget() and UL_testing_Tuning_Scan() functions are
*  executed by these functions. The status of a sensor scan must be checked
*  using the UL_testing_Tuning_IsBusy() API prior to starting a next scan
*  or setting up another widget.
*
* \return
*  Returns the status of the operation:
*  - CYRET_SUCCESS - Scanning is successfully started.
*  - CYRET_BAD_PARAM - All the widgets are disabled.
*  - CYRET_INVALID_STATE - The previous scanning is not completed and the HW block is busy.
*  - CYRET_UNKNOWN - There are unknown errors.
*
*******************************************************************************/
cystatus UL_testing_Tuning_ScanAllWidgets(void)
{
    cystatus scanStatus = CYRET_UNKNOWN;

    uint32 wdgtIndex;

    if (UL_testing_Tuning_SW_STS_BUSY == (UL_testing_Tuning_dsRam.status & UL_testing_Tuning_SW_STS_BUSY))
    {
        /* Previous widget is being scanned. Return error. */
        scanStatus = CYRET_INVALID_STATE;
    }
    else
    {
        /*
         *  Set up widget first widget.
         *  If widget returned error, set up next, continue same until widget does not return error.
         */
        for (wdgtIndex = 0u;
             wdgtIndex < UL_testing_Tuning_TOTAL_WIDGETS;
             wdgtIndex++)
        {

            scanStatus = UL_testing_Tuning_SetupWidget(wdgtIndex);

            if (CYRET_SUCCESS == scanStatus)
            {
                #if (0u != (UL_testing_Tuning_TOTAL_WIDGETS - 1u))
                    /* If there are more than one widget to be scanned, request callback to scan next widget */
                    if ((UL_testing_Tuning_TOTAL_WIDGETS - 1u) > wdgtIndex)
                    {
                         /* Request callback to scan next widget in ISR */
                        UL_testing_Tuning_requestScanAllWidget = UL_testing_Tuning_ENABLE;
                    }
                    else
                    {
                        /* Request to exit in ISR (Do not scan the next widgets) */
                        UL_testing_Tuning_requestScanAllWidget = UL_testing_Tuning_DISABLE;
                    }
                #else
                    {
                        /* Request to exit in ISR (We have only one widget) */
                        UL_testing_Tuning_requestScanAllWidget = UL_testing_Tuning_DISABLE;
                    }
                #endif  /* (0u != (UL_testing_Tuning_TOTAL_WIDGETS - 1u)) */

                /* Initiate scan and quit loop */
                scanStatus = UL_testing_Tuning_Scan();

                break;
            }
        }
    }

    return (scanStatus);
}


/*******************************************************************************
* Function Name: UL_testing_Tuning_SsInitialize
****************************************************************************//**
*
* \brief
*  Performs hardware and firmware initialization required for proper operation
*  of the UL_testing_Tuning Component. This function is called from
*  the UL_testing_Tuning_Start() API prior to calling any other APIs of the Component.
*
* \details
*  Performs hardware and firmware initialization required for proper operation
*  of the UL_testing_Tuning Component. This function is called from
*  the UL_testing_Tuning_Start() API prior to calling any other APIs of the Component.
*  1. Depending on the configuration, the function initializes the CSD block
*     for the corresponding sensing mode.
*  2. The function updates the dsRam.wdgtWorking variable with 1 when Self-test
*     is enabled.
*
*  Calling the UL_testing_Tuning_Start API is the recommended method to initialize
*  the UL_testing_Tuning Component at power-up. The UL_testing_Tuning_SsInitialize()
*  API should not be used for initialization, resume, or wake-up operations.
*  The dsRam.wdgtWorking variable is updated.
*
* \return status
*  Returns status of operation:
*  - Zero        - Indicates successful initialization.
*  - Non-zero    - One or more errors occurred in the initialization process.
*
*******************************************************************************/
cystatus UL_testing_Tuning_SsInitialize(void)
{
    cystatus initStatus;

    #if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_MULTI_FREQ_SCAN_EN) &&\
         (UL_testing_Tuning_MFS_IMO == UL_testing_Tuning_MFS_METHOD))
        uint8 centerFreqTrim;

        centerFreqTrim = (uint8)CY_GET_REG32(CY_SYS_CLK_IMO_TRIM1_PTR);
        UL_testing_Tuning_immunity[UL_testing_Tuning_FREQ_CHANNEL_0] = centerFreqTrim;

        if (centerFreqTrim < UL_testing_Tuning_IMO_FREQUENCY_OFFSET_F1)
        {
            UL_testing_Tuning_immunity[UL_testing_Tuning_FREQ_CHANNEL_1] = centerFreqTrim + UL_testing_Tuning_IMO_FREQUENCY_OFFSET_F1;
            UL_testing_Tuning_immunity[UL_testing_Tuning_FREQ_CHANNEL_2] = centerFreqTrim + (UL_testing_Tuning_IMO_FREQUENCY_OFFSET_F1 +
                                                                                           UL_testing_Tuning_IMO_FREQUENCY_OFFSET_F2);
        }
        else if (centerFreqTrim > (UL_testing_Tuning_IMO_FREQUENCY_OFFSET_MAX - UL_testing_Tuning_IMO_FREQUENCY_OFFSET_F2))
        {
            UL_testing_Tuning_immunity[UL_testing_Tuning_FREQ_CHANNEL_1] = centerFreqTrim - UL_testing_Tuning_IMO_FREQUENCY_OFFSET_F1;
            UL_testing_Tuning_immunity[UL_testing_Tuning_FREQ_CHANNEL_2] = centerFreqTrim - (UL_testing_Tuning_IMO_FREQUENCY_OFFSET_F1 +
                                                                                           UL_testing_Tuning_IMO_FREQUENCY_OFFSET_F2);
        }
        else
        {
            UL_testing_Tuning_immunity[UL_testing_Tuning_FREQ_CHANNEL_1] = centerFreqTrim - UL_testing_Tuning_IMO_FREQUENCY_OFFSET_F1;
            UL_testing_Tuning_immunity[UL_testing_Tuning_FREQ_CHANNEL_2] = centerFreqTrim + UL_testing_Tuning_IMO_FREQUENCY_OFFSET_F2;
        }

    #endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_MULTI_FREQ_SCAN_EN) &&\
               (UL_testing_Tuning_MFS_IMO == UL_testing_Tuning_MFS_METHOD)) */

    initStatus = CYRET_SUCCESS;

    #if (UL_testing_Tuning_DISABLE == UL_testing_Tuning_CSDV2)
        UL_testing_Tuning_SsTrimIdacs();
    #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2) */

    #if((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN) ||\
    ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN) && (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2)))
        UL_testing_Tuning_SsInitializeSourceSenseClk();
    #endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN) ||\
              ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN) && (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2))) */

    /* Set all IO states to the default state */
    UL_testing_Tuning_SsSetIOsInDefaultState();

    #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_MANY_SENSE_MODES_EN)
        /* Initialize CSD block for ADC scanning */
        UL_testing_Tuning_SsSwitchSensingMode(UL_testing_Tuning_UNDEFINED_E);
    #elif (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN)
        UL_testing_Tuning_SsSwitchSensingMode(UL_testing_Tuning_SENSE_METHOD_CSD_E);
    #elif (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN)
        UL_testing_Tuning_SsSwitchSensingMode(UL_testing_Tuning_SENSE_METHOD_CSX_E);
    #elif (UL_testing_Tuning_ENABLE == UL_testing_Tuning_ISX_EN)
        UL_testing_Tuning_SsSwitchSensingMode(UL_testing_Tuning_SENSE_METHOD_ISX_E);
    #else
        #error "No sensing method enabled, Component cannot work in this mode"
        initStatus = CYRET_UNKNOWN;
    #endif

    return (initStatus);
}


/*******************************************************************************
* Function Name: UL_testing_Tuning_SetPinState
****************************************************************************//**
*
* \brief
*  Sets the state (drive mode and output state) of the port pin used by a sensor.
*  The possible states are GND, Shield, High-Z, Tx or Rx, Sensor. If the sensor
*  specified in the input parameter is a ganged sensor, then the state of all pins
*  associated with the ganged sensor is updated.
*
* \details
*  This function sets a specified state for a specified sensor element. For the
*  CSD widgets, sensor element is a sensor ID, for the CSX widgets, it is either
*  an Rx or Tx electrode ID. If the specified sensor is a ganged sensor, then
*  the specified state is set for all the electrodes belong to the sensor.
*  This function must not be called while the Component is in the busy state.
*
*  This function accepts the UL_testing_Tuning_SHIELD and
*  UL_testing_Tuning_SENSOR states as an input only if there is at least
*  one CSD widget. Similarly, this function accepts the UL_testing_Tuning_TX_PIN
*  and UL_testing_Tuning_RX_PIN states as an input only if there is at least
*  one CSX widget in the project.
*
*  Calling this function directly from the application layer is not
*  recommended. This function is used to implement only the custom-specific
*  use cases. Functions that perform a setup and scan of a sensor/widget
*  automatically set the required pin states. They ignore changes
*  in the design made by the UL_testing_Tuning_SetPinState() function.
*  This function neither check wdgtIndex nor sensorElement for the correctness.
*
* \param widgetId
*  Specifies the ID of the widget to change the pin state of the specified
*  sensor.
*  A macro for the widget ID can be found in the UL_testing_Tuning Configuration
*  header file defined as UL_testing_Tuning_<WidgetName>_WDGT_ID.
*
* \param sensorElement
*  Specifies the ID of the sensor element within the widget to change
*  its pin state.
*  \if SECTION_C_LOW_LEVEL
*  For the CSD widgets, sensorElement is the sensor ID and can be found in the
*  UL_testing_Tuning Configuration header file defined as
*  - UL_testing_Tuning_<WidgetName>_SNS<SensorNumber>_ID.
*  For the CSX widgets, sensorElement is defined either as Rx ID or Tx ID.
*  The first Rx in a widget corresponds to sensorElement = 0, the second
*  Rx in a widget corresponds to sensorElement = 1, and so on.
*  The last Tx in a widget corresponds to sensorElement = (RxNum + TxNum).
*  \endif
*  Macros for Rx and Tx IDs can be found in the
*  UL_testing_Tuning Configuration header file defined as:
*  - UL_testing_Tuning_<WidgetName>_RX<RXNumber>_ID
*  - UL_testing_Tuning_<WidgetName>_TX<TXNumber>_ID.
*
* \param state
*  Specifies the state of the sensor to be set:
*     1. UL_testing_Tuning_GROUND - The pin is connected to the ground.
*     2. UL_testing_Tuning_HIGHZ - The drive mode of the pin is set to High-Z
*        Analog.
*     3. UL_testing_Tuning_SHIELD - The shield signal is routed to the pin
*        (available only if CSD sensing method with shield electrode is enabled).
*     4. UL_testing_Tuning_SENSOR - The pin is connected to the scanning bus
*        (available only if CSD sensing method is enabled).
*     5. UL_testing_Tuning_TX_PIN - The Tx signal is routed to the sensor
*        (available only if CSX sensing method is enabled).
*     6. UL_testing_Tuning_RX_PIN - The pin is connected to the scanning bus
*        (available only if CSX sensing method is enabled).
*
*******************************************************************************/
void UL_testing_Tuning_SetPinState(uint32 widgetId, uint32 sensorElement, uint32 state)
{
    uint32 eltdNum;
    uint32 eltdIndex;
    uint8  interruptState;
    UL_testing_Tuning_FLASH_IO_STRUCT const *ioPtr;
    #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_GANGED_SNS_EN)
        UL_testing_Tuning_FLASH_SNS_STRUCT const *curFlashSnsPtr;
    #endif

    /* Getting sensor element pointer and number of electrodes */
    #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_GANGED_SNS_EN)
        /* Check the ganged sns flag */
        if (UL_testing_Tuning_GANGED_SNS_MASK == (UL_testing_Tuning_dsFlash.wdgtArray[widgetId].staticConfig & UL_testing_Tuning_GANGED_SNS_MASK))
        {
            curFlashSnsPtr = UL_testing_Tuning_dsFlash.wdgtArray[widgetId].ptr2SnsFlash;
            curFlashSnsPtr += sensorElement;
            ioPtr = &UL_testing_Tuning_ioList[curFlashSnsPtr->firstPinId];
            eltdNum = curFlashSnsPtr->numPins;
        }
        else
        {
            ioPtr = (UL_testing_Tuning_FLASH_IO_STRUCT const *)UL_testing_Tuning_dsFlash.wdgtArray[widgetId].ptr2SnsFlash + sensorElement;
            eltdNum = 1u;
        }
    #else
        ioPtr = (UL_testing_Tuning_FLASH_IO_STRUCT const *)UL_testing_Tuning_dsFlash.wdgtArray[widgetId].ptr2SnsFlash + sensorElement;
        eltdNum = 1u;
    #endif

    /* Loop through all electrodes of the specified sensor element */
    for (eltdIndex = 0u; eltdIndex < eltdNum; eltdIndex++)
    {
        /* Reset HSIOM and PC registers */
        interruptState = CyEnterCriticalSection();
        CY_SET_REG32(ioPtr->hsiomPtr, CY_GET_REG32(ioPtr->hsiomPtr) & ~(UL_testing_Tuning_HSIOM_SEL_MASK << ioPtr->hsiomShift));
        CY_SET_REG32(ioPtr->pcPtr, CY_GET_REG32(ioPtr->pcPtr) & ~(UL_testing_Tuning_GPIO_PC_MASK << ioPtr->shift));
        CyExitCriticalSection(interruptState);

        switch (state)
        {
        case UL_testing_Tuning_GROUND:

            interruptState = CyEnterCriticalSection();
            CY_SET_REG32(ioPtr->pcPtr, CY_GET_REG32(ioPtr->pcPtr) | (UL_testing_Tuning_SNS_GROUND_CONNECT << ioPtr->shift));
            CY_SET_REG32(ioPtr->drPtr, CY_GET_REG32(ioPtr->drPtr) & (uint32)(~(uint32)((uint32)1u << ioPtr->drShift)));
            CyExitCriticalSection(interruptState);
            break;

        case UL_testing_Tuning_HIGHZ:
            interruptState = CyEnterCriticalSection();
            CY_SET_REG32(ioPtr->drPtr, CY_GET_REG32(ioPtr->drPtr) & (uint32)~(uint32)((uint32)1u << ioPtr->drShift));
            CyExitCriticalSection(interruptState);
            break;

        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN)
            case UL_testing_Tuning_SENSOR:
                /* Enable sensor */
                UL_testing_Tuning_CSDConnectSns(ioPtr);
                break;

            #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN)
                case UL_testing_Tuning_SHIELD:
                    /* Set appropriate bit in HSIOM register to configure pin to shield mode */
                    interruptState = CyEnterCriticalSection();
                    CY_SET_REG32(ioPtr->hsiomPtr, CY_GET_REG32(ioPtr->hsiomPtr) | (UL_testing_Tuning_HSIOM_SEL_CSD_SHIELD << ioPtr->hsiomShift));
                    CyExitCriticalSection(interruptState);
                    break;
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) */
        #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN) */

        #if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN) || \
             (UL_testing_Tuning_ENABLE == UL_testing_Tuning_ISX_EN))
            case UL_testing_Tuning_TX_PIN:
                interruptState = CyEnterCriticalSection();
                CY_SET_REG32(ioPtr->hsiomPtr, CY_GET_REG32(ioPtr->hsiomPtr) | (UL_testing_Tuning_HSIOM_SEL_CSD_SENSE << ioPtr->hsiomShift));
                CY_SET_REG32(ioPtr->pcPtr, CY_GET_REG32(ioPtr->pcPtr) | (UL_testing_Tuning_GPIO_STRGDRV << ioPtr->shift));
                CyExitCriticalSection(interruptState);
                break;

            case UL_testing_Tuning_RX_PIN:
                interruptState = CyEnterCriticalSection();
                CY_SET_REG32(ioPtr->hsiomPtr, CY_GET_REG32(ioPtr->hsiomPtr) | (UL_testing_Tuning_HSIOM_SEL_AMUXA << ioPtr->hsiomShift));
                CY_SET_REG32(ioPtr->pcPtr, CY_GET_REG32(ioPtr->pcPtr) & ~(UL_testing_Tuning_GPIO_PC_MASK << ioPtr->shift));
                CyExitCriticalSection(interruptState);
                break;
        #endif

        default:
            /* Wrong state */
            break;
        }

        ioPtr++;
    }
}

#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN)
    /*******************************************************************************
    * Function Name: UL_testing_Tuning_SsCSDDisableMode
    ****************************************************************************//**
    *
    * \brief
    *  This function disables CSD mode.
    *
    * \details
    *  To disable CSD mode the following tasks are performed:
    *  1. Disconnect Cmod from AMUXBUS-A.
    *  2. Disconnect previous CSX electrode if it has been connected.
    *  3. Disconnect Csh from AMUXBUS-B.
    *  4. Disable Shield Electrodes.
    *
    *******************************************************************************/
    static void UL_testing_Tuning_SsCSDDisableMode(void)
    {
        uint32 newRegValue;

        /* Disconnect Cmod from AMUXBUS-A using HSIOM registers */
        newRegValue = CY_GET_REG32(UL_testing_Tuning_CMOD_HSIOM_PTR);
        newRegValue &= (uint32)(~(uint32)UL_testing_Tuning_CMOD_HSIOM_MASK);
        CY_SET_REG32(UL_testing_Tuning_CMOD_HSIOM_PTR, newRegValue);

        #if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2) && \
             ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_IDAC_COMP_EN) && \
             (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_DEDICATED_IDAC_COMP_EN)))
            /* Disconnect IDACA and IDACB */
            newRegValue = CY_GET_REG32(UL_testing_Tuning_SW_REFGEN_SEL_PTR);
            newRegValue &= (uint32)(~UL_testing_Tuning_SW_REFGEN_SEL_SW_IAIB_MASK);
            CY_SET_REG32(UL_testing_Tuning_SW_REFGEN_SEL_PTR, newRegValue);
        #endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2) && \
                   ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_IDAC_COMP_EN) && \
                   (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_DEDICATED_IDAC_COMP_EN))) */

        /* Disconnect previous CSD electrode if it has been connected */
        UL_testing_Tuning_SsCSDElectrodeCheck();

        /* Disconnect Csh from AMUXBUS-B */
        #if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) && \
             (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_TANK_EN))
            newRegValue = CY_GET_REG32(UL_testing_Tuning_CSH_HSIOM_PTR);
            newRegValue &= (uint32)(~(uint32)(UL_testing_Tuning_CSH_TO_AMUXBUS_B_MASK << UL_testing_Tuning_CSH_HSIOM_SHIFT));
            CY_SET_REG32(UL_testing_Tuning_CSH_HSIOM_PTR, newRegValue);
        #endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) && \
                   (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_TANK_EN)) */

        #if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) && \
             (0u != UL_testing_Tuning_CSD_TOTAL_SHIELD_COUNT))
            UL_testing_Tuning_SsCSDDisableShieldElectrodes();
        #endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) && \
                   (0u != UL_testing_Tuning_CSD_TOTAL_SHIELD_COUNT)) */
    }
#endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN) */


#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN)
    /*******************************************************************************
    * Function Name: UL_testing_Tuning_SsDisableCSXMode
    ****************************************************************************//**
    *
    * \brief
    *  This function disables CSX mode.
    *
    * \details
    *  To disable CSX mode the following tasks are performed:
    *  1. Disconnect CintA and CintB from AMUXBUS-A.
    *  2. Disconnect previous CSX electrode if it has been connected.
    *
    *******************************************************************************/
    static void UL_testing_Tuning_SsDisableCSXMode(void)
    {
        uint32 newRegValue;

        /* Disconnect CintA from AMUXBUS-A using HSIOM registers */
        newRegValue = CY_GET_REG32(UL_testing_Tuning_CintA_HSIOM_PTR);
        newRegValue &= (uint32)(~(uint32)UL_testing_Tuning_CintA_HSIOM_MASK);
        CY_SET_REG32(UL_testing_Tuning_CintA_HSIOM_PTR, newRegValue);

        /* Disconnect CintB from AMUXBUS-A using HSIOM registers */
        newRegValue = CY_GET_REG32(UL_testing_Tuning_CintB_HSIOM_PTR);
        newRegValue &= (uint32)(~(uint32)UL_testing_Tuning_CintB_HSIOM_MASK);
        CY_SET_REG32(UL_testing_Tuning_CintB_HSIOM_PTR, newRegValue);

        /* Disconnect previous CSX electrode if it has been connected */
        UL_testing_Tuning_CSXElectrodeCheck();
    }
#endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN) */

#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_ISX_EN)
    /*******************************************************************************
    * Function Name: UL_testing_Tuning_SsDisableISXMode
    ****************************************************************************//**
    *
    * \brief
    *  This function disables ISX mode.
    *
    * \details
    *  To disable ISX mode the following tasks are performed:
    *  1. Disconnect CintA and CintB from AMUXBUS-A
    *  2. Disconnect previous ISX electrode if it has been connected
    *
    *******************************************************************************/
    static void UL_testing_Tuning_SsDisableISXMode(void)
    {
        uint32 newRegValue;

        /* Disconnect CintA from AMUXBUS-A using HSIOM registers.   */
        newRegValue = CY_GET_REG32(UL_testing_Tuning_CintA_HSIOM_PTR);
        newRegValue &= (uint32)(~(uint32)UL_testing_Tuning_CintA_HSIOM_MASK);
        CY_SET_REG32(UL_testing_Tuning_CintA_HSIOM_PTR, newRegValue);

        /* Disconnect CintB from AMUXBUS-A using HSIOM registers.   */
        newRegValue = CY_GET_REG32(UL_testing_Tuning_CintB_HSIOM_PTR);
        newRegValue &= (uint32)(~(uint32)UL_testing_Tuning_CintB_HSIOM_MASK);
        CY_SET_REG32(UL_testing_Tuning_CintB_HSIOM_PTR, newRegValue);

        /* Disconnect previous ISX electrode if it has been connected */
        UL_testing_Tuning_ISXElectrodeCheck();
        UL_testing_Tuning_DigPrt2_Stop();
        #if (UL_testing_Tuning_USES_PORT_3)
            UL_testing_Tuning_DigPrt3_Stop();
        #endif /* UL_testing_Tuning_USES_PORT_3 */
    }
#endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_ISX_EN) */

/*******************************************************************************
* Function Name: UL_testing_Tuning_SsSwitchSensingMode
****************************************************************************//**
*
* \brief
*  This function changes the mode for case when both CSD and CSX widgets are
*  scanned.
*
* \details
*  To switch to the CSD mode the following tasks are performed:
*  1. Disconnect CintA and CintB from AMUXBUS-A.
*  2. Disconnect previous CSD electrode if it has been connected.
*  3. Initialize CSD mode.
*
*  To switch to the CSX mode the following tasks are performed:
*  1. Disconnect Cmod from AMUXBUS-A.
*  2. Disconnect previous CSX electrode if it has been connected.
*  3. Initialize CSX mode.
*
* \param mode
*  Specifies the scan mode:
*  - UL_testing_Tuning_SENSE_METHOD_CSD_E
*  - UL_testing_Tuning_SENSE_METHOD_CSX_E
*  - UL_testing_Tuning_SENSE_METHOD_ISX_E
*  - UL_testing_Tuning_SENSE_METHOD_BIST_E
*  - UL_testing_Tuning_UNDEFINED_E
*
*******************************************************************************/
void UL_testing_Tuning_SsSwitchSensingMode(UL_testing_Tuning_SENSE_METHOD_ENUM mode)
{
    if (UL_testing_Tuning_currentSenseMethod != mode)
    {
        /* The requested mode differes to the current one. Disable the current mode */
        if (UL_testing_Tuning_SENSE_METHOD_CSD_E == UL_testing_Tuning_currentSenseMethod)
        {
            #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN)
                UL_testing_Tuning_SsCSDDisableMode();
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN) */
        }
        else if (UL_testing_Tuning_SENSE_METHOD_CSX_E == UL_testing_Tuning_currentSenseMethod)
        {
            #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN)
                UL_testing_Tuning_SsDisableCSXMode();
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN) */
        }
        else if (UL_testing_Tuning_SENSE_METHOD_ISX_E == UL_testing_Tuning_currentSenseMethod)
        {
            #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_ISX_EN)
                UL_testing_Tuning_SsDisableISXMode();
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_ISX_EN) */
        }
        else if (UL_testing_Tuning_SENSE_METHOD_BIST_E == UL_testing_Tuning_currentSenseMethod)
        {
            #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_SELF_TEST_EN)
                UL_testing_Tuning_BistDisableMode();
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_SELF_TEST_EN) */
        }
        else
        {
            #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_ADC_EN)
                /* Release ADC resources */
                (void)UL_testing_Tuning_AdcReleaseResources();
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_ADC_EN) */
        }

        /* Enable the specified mode */
        if (UL_testing_Tuning_SENSE_METHOD_CSD_E == mode)
        {
            #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN)
                /* Initialize CSD mode to guarantee configured CSD mode */
                UL_testing_Tuning_SsCSDInitialize();
                UL_testing_Tuning_currentSenseMethod = UL_testing_Tuning_SENSE_METHOD_CSD_E;
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN) */
        }
        else if (UL_testing_Tuning_SENSE_METHOD_CSX_E == mode)
        {
            #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN)
                /* Initialize CSX mode to guarantee configured CSX mode */
                UL_testing_Tuning_CSXInitialize();
                UL_testing_Tuning_currentSenseMethod = UL_testing_Tuning_SENSE_METHOD_CSX_E;
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN) */
        }
        else if (UL_testing_Tuning_SENSE_METHOD_ISX_E == mode)
        {
            #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_ISX_EN)
                /* Initialize ISX mode to guarantee configured ISX mode */
                UL_testing_Tuning_ISXInitialize();
                UL_testing_Tuning_currentSenseMethod = UL_testing_Tuning_SENSE_METHOD_ISX_E;
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_ISX_EN) */
        }
        else if (UL_testing_Tuning_SENSE_METHOD_BIST_E == mode)
        {
            #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_SELF_TEST_EN)
                UL_testing_Tuning_BistInitialize();
                UL_testing_Tuning_currentSenseMethod = UL_testing_Tuning_SENSE_METHOD_BIST_E;
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_SELF_TEST_EN) */
        }
        else
        {
            UL_testing_Tuning_currentSenseMethod = UL_testing_Tuning_UNDEFINED_E;
        }
    }
}


/*******************************************************************************
* Function Name: UL_testing_Tuning_SsSetIOsInDefaultState
****************************************************************************//**
*
* \brief
*  Sets all electrodes into a default state.
*
* \details
*  Sets all the CSD/CSX IOs into a default state:
*  - HSIOM   - Disconnected, the GPIO mode.
*  - DM      - Strong drive.
*  - State   - Zero.
*
*  Sets all the ADC channels into a default state:
*  - HSIOM   - Disconnected, the GPIO mode.
*  - DM      - HiZ-Analog.
*  - State   - Zero.
*
*  It is not recommended to call this function directly from the application
*  layer.
*
*******************************************************************************/
void UL_testing_Tuning_SsSetIOsInDefaultState(void)
{
    UL_testing_Tuning_FLASH_IO_STRUCT const *ioPtr = &UL_testing_Tuning_ioList[0u];
    uint32 loopIndex;
    uint32 regValue;

    /* Loop through all electrodes */
    for (loopIndex = 0u; loopIndex < UL_testing_Tuning_TOTAL_ELECTRODES; loopIndex++)
    {
        /*
        * 1. Disconnect HSIOM.
        * 2. Set strong DM.
        * 3. Set pin state to logic 0.
        */
        regValue = CY_GET_REG32 (ioPtr->hsiomPtr);
        regValue &= ~(ioPtr->hsiomMask);
        CY_SET_REG32 (ioPtr->hsiomPtr, regValue);

        regValue = CY_GET_REG32 (ioPtr->pcPtr);
        regValue &= ~(UL_testing_Tuning_GPIO_PC_MASK << ioPtr->shift);
        regValue |=  (UL_testing_Tuning_GPIO_STRGDRV << ioPtr->shift);
        CY_SET_REG32 (ioPtr->pcPtr, regValue);

        regValue = CY_GET_REG32 (ioPtr->drPtr);
        regValue &= ~(ioPtr->mask);
        CY_SET_REG32 (ioPtr->drPtr, regValue);

        /* Get next electrode */
        ioPtr++;
    }

    #if(UL_testing_Tuning_ENABLE == UL_testing_Tuning_ADC_EN)
        UL_testing_Tuning_ClearAdcChannels();
    #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_ADC_EN) */
    
    #if(UL_testing_Tuning_ENABLE == UL_testing_Tuning_ISX_EN)
        UL_testing_Tuning_ISXSetDefaultDigital();
    #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_ISX_EN) */
}

#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_ADC_EN)
/*******************************************************************************
* Function Name: UL_testing_Tuning_SsReleaseResources()
****************************************************************************//**
*
* \brief
*  This function sets the resources that do not belong to the sensing HW block to
*  default state.
*
* \details
*  The function performs following tasks:
*  1. Checks if CSD block busy and returns error if it is busy
*  2. Disconnects integration capacitors (CintA, CintB for CSX mode and
*     Cmod for CSD mode)
*  3. Disconnect electroded if they have been connected.
*
* \return
*  Returns the status of the operation:
*  - Zero        - Resources released successfully.
*  - Non-zero    - One or more errors occurred in releasing process.
*
*******************************************************************************/
cystatus UL_testing_Tuning_SsReleaseResources(void)
{
    cystatus busyStatus = CYRET_SUCCESS;

    if (UL_testing_Tuning_SW_STS_BUSY == (UL_testing_Tuning_dsRam.status & UL_testing_Tuning_SW_STS_BUSY))
    {
        /* Previous widget is being scanned. Return error. */
        busyStatus = CYRET_INVALID_STATE;
    }
    else
    {
        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN)
            UL_testing_Tuning_SsDisableCSXMode();
        #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN) */

        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_ISX_EN)
            UL_testing_Tuning_SsDisableISXMode();
        #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_ISX_EN) */

        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN)
            UL_testing_Tuning_SsCSDDisableMode();
        #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN) */

        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_SELF_TEST_EN)
            UL_testing_Tuning_BistDisableMode();
        #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_SELF_TEST_EN) */

        #if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN) && \
             (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) &&  \
             (UL_testing_Tuning_SNS_CONNECTION_SHIELD == UL_testing_Tuning_CSD_INACTIVE_SNS_CONNECTION))
            UL_testing_Tuning_SsSetIOsInDefaultState();
        #endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN) && \
             (UL_testing_Tuning_DISABLE != UL_testing_Tuning_CSD_SHIELD_EN) &&  \
             (UL_testing_Tuning_SNS_CONNECTION_SHIELD == UL_testing_Tuning_CSD_INACTIVE_SNS_CONNECTION)) */

        /*
         * Reset of the currentSenseMethod variable to make sure that the next
         * call of SetupWidget() API setups the correct widget mode
         */
        UL_testing_Tuning_currentSenseMethod = UL_testing_Tuning_UNDEFINED_E;
    }

    return busyStatus;
}
#endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_ADC_EN) */


/*******************************************************************************
* Function Name: UL_testing_Tuning_SsPostAllWidgetsScan
****************************************************************************//**
*
* \brief
*  The ISR function for multiple widget scanning implementation.
*
* \details
*  This is the function used by the UL_testing_Tuning ISR to implement multiple widget
*  scanning.
*  Should not be used by the application layer.
*
*******************************************************************************/
void UL_testing_Tuning_SsPostAllWidgetsScan(void)
{
    /*
    *  1. Increment widget index
    *  2. Check if all the widgets are scanned
    *  3. If all the widgets are not scanned, set up and scan next widget
    */
    #if (1u != UL_testing_Tuning_TOTAL_WIDGETS)
        cystatus postScanStatus;

        do
        {
            UL_testing_Tuning_widgetIndex++;

            postScanStatus = UL_testing_Tuning_SetupWidget((uint32)UL_testing_Tuning_widgetIndex);

            if (CYRET_SUCCESS == postScanStatus)
            {
                if((UL_testing_Tuning_TOTAL_WIDGETS - 1u) == UL_testing_Tuning_widgetIndex)
                {
                    /* The last widget will be scanned. Reset flag to skip configuring the next widget setup in ISR. */
                    UL_testing_Tuning_requestScanAllWidget = UL_testing_Tuning_DISABLE;
                }
                (void)UL_testing_Tuning_Scan();
            }
            else if((UL_testing_Tuning_TOTAL_WIDGETS - 1u) == UL_testing_Tuning_widgetIndex)
            {
                #if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_BLOCK_OFF_AFTER_SCAN_EN) && \
                     (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN))
                    if (UL_testing_Tuning_SENSE_METHOD_CSD_E ==
                        UL_testing_Tuning_GET_SENSE_METHOD(&UL_testing_Tuning_dsFlash.wdgtArray[UL_testing_Tuning_widgetIndex]))
                    {
                        /* Disable the CSD block */
                        CY_SET_REG32(UL_testing_Tuning_CONFIG_PTR, UL_testing_Tuning_configCsd);
                    }
                #endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_BLOCK_OFF_AFTER_SCAN_EN) && \
                           (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN)) */

                /* Update scan Counter */
                UL_testing_Tuning_dsRam.scanCounter++;
                /* All widgets are totally processed. Reset BUSY flag */
                UL_testing_Tuning_dsRam.status &= ~UL_testing_Tuning_SW_STS_BUSY;

                /* Update status with with the failure */
                UL_testing_Tuning_dsRam.status &= ~UL_testing_Tuning_STATUS_ERR_MASK;
                UL_testing_Tuning_dsRam.status |= ((postScanStatus & UL_testing_Tuning_STATUS_ERR_SIZE) << UL_testing_Tuning_STATUS_ERR_SHIFT);

                /* Set postScanStatus to exit the while loop */
                postScanStatus = CYRET_SUCCESS;
            }
            else
            {
                /* Update status with with the failure. Configure the next widget in while() loop */
                UL_testing_Tuning_dsRam.status &= ~UL_testing_Tuning_STATUS_ERR_MASK;
                UL_testing_Tuning_dsRam.status |= ((postScanStatus & UL_testing_Tuning_STATUS_ERR_SIZE) << UL_testing_Tuning_STATUS_ERR_SHIFT);
            }
        } while (CYRET_SUCCESS != postScanStatus);
    #endif /* (1u != UL_testing_Tuning_TOTAL_WIDGETS) */
}


/*******************************************************************************
* Function Name: UL_testing_Tuning_SsIsrInitialize
****************************************************************************//**
*
* \brief
*  Enables and initializes for the function pointer for a callback for the ISR.
*
* \details
*  The "address" is a special type cyisraddress defined by Cylib. This function
*  is used by Component APIs and should not be used by an application program for
*  proper working of the Component.
*
* \param address
*  The address of the function to be called when interrupt is fired.
*
*******************************************************************************/
void UL_testing_Tuning_SsIsrInitialize(cyisraddress address)
{
    UL_testing_Tuning_ISR_StartEx(address);
}


/*******************************************************************************
* Function Name: UL_testing_Tuning_SsSetModClkClockDivider
****************************************************************************//**
*
* \brief
*  Sets the divider values for the modulator clock and then starts
*  the modulator clock.
*
* \details
*  It is not recommended to call this function directly by the application layer.
*  It is used by initialization, widget APIs or wakeup functions to
*  enable the clocks.
*
* \param modClk
*  The divider value for the modulator clock.
*
*******************************************************************************/
void UL_testing_Tuning_SsSetModClkClockDivider(uint32 modClk)
{
#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_IS_M0S8PERI_BLOCK)
    /* Stop modulator clock */
    CY_SET_REG32(UL_testing_Tuning_MODCLK_CMD_PTR,
                 ((uint32)UL_testing_Tuning_ModClk__DIV_ID <<
                 UL_testing_Tuning_MODCLK_CMD_DIV_SHIFT)|
                 ((uint32)UL_testing_Tuning_MODCLK_CMD_DISABLE_MASK));

    /*
     * Set divider value for sense modClk.
     * 1u is subtracted from modClk because Divider register value 0 corresponds
     * to dividing by 1.
     */
    CY_SET_REG32(UL_testing_Tuning_MODCLK_DIV_PTR, ((modClk - 1u) << 8u));

    /* Check whether previous modulator clock start command has finished. */
    while(0u != (CY_GET_REG32(UL_testing_Tuning_MODCLK_CMD_PTR) & UL_testing_Tuning_MODCLK_CMD_ENABLE_MASK))
    {
        /* Wait until previous modulator clock start command has finished */
    }

    /* Start modulator clock, aligned to HFCLK */
    CY_SET_REG32(UL_testing_Tuning_MODCLK_CMD_PTR,
                 (uint32)(((uint32)UL_testing_Tuning_ModClk__DIV_ID << UL_testing_Tuning_MODCLK_CMD_DIV_SHIFT) |
                  ((uint32)UL_testing_Tuning_ModClk__PA_DIV_ID << UL_testing_Tuning_MODCLK_CMD_PA_DIV_SHIFT) |
                  UL_testing_Tuning_MODCLK_CMD_ENABLE_MASK));
#else
    uint32 newRegValue;

    /* Clear bit to disable ModClk clock. */
    CY_SET_REG32(UL_testing_Tuning_MODCLK_CMD_PTR,
                 CY_GET_REG32(UL_testing_Tuning_MODCLK_CMD_PTR) &
                 (uint32)(~UL_testing_Tuning_ModClk__ENABLE_MASK));

    /*
     * Update ModClk clock divider.
     * 1u is subtracted from modClk because Divider register value has range (0-65535).
     */
    newRegValue = CY_GET_REG32(UL_testing_Tuning_MODCLK_DIV_PTR) & (uint32)(~(uint32)UL_testing_Tuning_ModClk__DIVIDER_MASK);
    newRegValue |= (modClk - 1u);
    CY_SET_REG32(UL_testing_Tuning_MODCLK_DIV_PTR, newRegValue);

    /* Set bit to enable ModClk clock. */
    CY_SET_REG32(UL_testing_Tuning_MODCLK_CMD_PTR,
                 CY_GET_REG32(UL_testing_Tuning_MODCLK_CMD_PTR) |
                UL_testing_Tuning_ModClk__ENABLE_MASK);
#endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_IS_M0S8PERI_BLOCK) */
}


/*******************************************************************************
* Function Name: UL_testing_Tuning_SsSetSnsClockDivider
****************************************************************************//**
*
* \brief
*  Sets the divider values for the sense clock and then starts
*  the sense clock.
*
* \details
*  It is not recommended to call this function directly by the application layer.
*  It is used by initialization, widget APIs or wakeup functions to
*  enable the clocks.
*
* \param
*  snsClk The divider value for the sense clock.
*
*******************************************************************************/
void UL_testing_Tuning_SsSetSnsClockDivider(uint32 snsClk)
{
#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2)
    uint32 newRegValue;

    /*
     * Set divider value for sense clock.
     * 1u is subtracted from snsClk because SENSE_DIV value 0 corresponds
     * to dividing by 1.
     */
    newRegValue = CY_GET_REG32(UL_testing_Tuning_SENSE_PERIOD_PTR);
    newRegValue &= (uint32)(~UL_testing_Tuning_SENSE_PERIOD_SENSE_DIV_MASK);
    newRegValue |= snsClk - 1u;
    CY_SET_REG32(UL_testing_Tuning_SENSE_PERIOD_PTR, newRegValue);
#else
    #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_IS_M0S8PERI_BLOCK)
        /* Stop sense clock clock   */
        CY_SET_REG32(UL_testing_Tuning_SNSCLK_CMD_PTR,
                     ((uint32)UL_testing_Tuning_SnsClk__DIV_ID <<
                     UL_testing_Tuning_SNSCLK_CMD_DIV_SHIFT)|
                     ((uint32)UL_testing_Tuning_SNSCLK_CMD_DISABLE_MASK));

        /*
         * Set divider value for sense clock.
         * 1u is subtracted from snsClk because Divider register value 0 corresponds
         * to dividing by 1.
         */
        CY_SET_REG32(UL_testing_Tuning_SNSCLK_DIV_PTR, ((snsClk - 1u) << 8u));

        /* Check whether previous sense clock start command has finished. */
        while(0u != (CY_GET_REG32(UL_testing_Tuning_SNSCLK_CMD_PTR) & UL_testing_Tuning_SNSCLK_CMD_ENABLE_MASK))
        {
            /* Wait until previous sense clock start command has finished. */
        }

        /* Start sense clock aligned to previously started modulator clock. */
        CY_SET_REG32(UL_testing_Tuning_SNSCLK_CMD_PTR,
                     (uint32)(((uint32)UL_testing_Tuning_SnsClk__DIV_ID << UL_testing_Tuning_SNSCLK_CMD_DIV_SHIFT) |
                      ((uint32)UL_testing_Tuning_ModClk__PA_DIV_ID << UL_testing_Tuning_SNSCLK_CMD_PA_DIV_SHIFT) |
                      UL_testing_Tuning_SNSCLK_CMD_ENABLE_MASK));
    #else
        uint32 newRegValue;

        /* Clear bit to disable SnsClk clock. */
        CY_SET_REG32(UL_testing_Tuning_SNSCLK_CMD_PTR,
                     CY_GET_REG32(UL_testing_Tuning_SNSCLK_CMD_PTR) &
                     (uint32)(~(uint32)UL_testing_Tuning_SnsClk__ENABLE_MASK));

        /*
         * Update snsClk clock divider.
         * 1u is subtracted from snsClk because Divider register value has range (0-65535).
         */
        newRegValue = CY_GET_REG32(UL_testing_Tuning_SNSCLK_DIV_PTR) & (uint32)(~(uint32)UL_testing_Tuning_SnsClk__DIVIDER_MASK);
        newRegValue |= (snsClk - 1u);
        CY_SET_REG32(UL_testing_Tuning_SNSCLK_DIV_PTR, newRegValue);

        /* Set bit to enable SnsClk clock. */
        CY_SET_REG32(UL_testing_Tuning_SNSCLK_CMD_PTR,
                     CY_GET_REG32(UL_testing_Tuning_SNSCLK_CMD_PTR) |
                    UL_testing_Tuning_SnsClk__ENABLE_MASK);
    #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_IS_M0S8PERI_BLOCK) */
#endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2) */
}


/*******************************************************************************
* Function Name: UL_testing_Tuning_SsSetClockDividers
****************************************************************************//**
*
* \brief
*  Sets the divider values for sense and modulator clocks and then starts
*  a modulator clock-phase aligned to HFCLK and sense clock-phase aligned to
*  the modulator clock.
*
* \details
*  It is not recommended to call this function directly by the application layer.
*  It is used by initialization, widget APIs or wakeup functions to
*  enable the clocks.
*
* \param
*  snsClk The divider value for the sense clock.
*  modClk The divider value for the modulator clock.
*
*******************************************************************************/
void UL_testing_Tuning_SsSetClockDividers(uint32 snsClk, uint32 modClk)
{
    /* Configure Mod clock */
    UL_testing_Tuning_SsSetModClkClockDivider(modClk);

    /* Configure Sns clock */
    UL_testing_Tuning_SsSetSnsClockDivider(snsClk);
}


#if (UL_testing_Tuning_ANYMODE_AUTOCAL)
    /*******************************************************************************
    * Function Name: UL_testing_Tuning_CalibrateWidget
    ****************************************************************************//**
    *
    * \brief
    *  Calibrates the IDACs for all the sensors in the specified widget to the default
    *  target, this function detects the sensing method used by the
    *  widget prior to calibration.
    *
    * \details
    *  This function performs exactly the same tasks as
    *  UL_testing_Tuning_CalibrateAllWidgets, but only for a specified widget.
    *  This function detects the sensing method used by the widgets and uses
    *  the Enable compensation IDAC parameter. 
    *  \if SECTION_I_REP
    *  For ISX mode, the frequency is also calibrated.
    *  \endif
    *
    *  \if SECTION_C_LOW_LEVEL
    *  This function is available when the CSD and/or CSX Enable IDAC
    *  auto-calibration parameter is enabled.
    *  \endif
    *  \if SECTION_I_REP
    *  This function is available when the ISX Enable auto-calibration parameter
    *  is enabled.
    *  \endif
    *
    * \param widgetId
    *  Specifies the ID number of the widget to calibrate its raw count.
    *  A macro for the widget ID can be found in the
    *  UL_testing_Tuning Configuration header file defined as
    *  UL_testing_Tuning_<WidgetName>_WDGT_ID.
    *
    * \return
    *  Returns the status of the specified widget calibration:
    *  - CYRET_SUCCESS - The operation is successfully completed.
    *  - CYRET_BAD_PARAM - The input parameter is invalid.
    *  - CYRET_BAD_DATA - The calibration failed and the Component may not
    *    operate as expected.
    *
    *******************************************************************************/
    cystatus UL_testing_Tuning_CalibrateWidget(uint32 widgetId)
    {
        cystatus calibrateStatus = CYRET_SUCCESS;

        do
        {
            if (UL_testing_Tuning_TOTAL_WIDGETS <= widgetId)
            {
                calibrateStatus = CYRET_BAD_PARAM;
            }

            /*
             *  Check if widget id is valid, specified widget did not
             *  detect any faults conditions if BIST is enabled.
             */
            #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_SELF_TEST_EN)
                if (0u != UL_testing_Tuning_GET_WIDGET_EN_STATUS(widgetId))
                {
                    calibrateStatus = CYRET_SUCCESS;
                }
                else
                {
                    calibrateStatus = CYRET_INVALID_STATE;
                }
            #endif  /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_SELF_TEST_EN) */

            if (CYRET_SUCCESS != calibrateStatus)
            {
                /* Exit from the loop because of a fail */
                break;
            }

            /* If both CSD and CSX are enabled, calibrate widget using sensing method */
            #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_CSX_EN)
                /* Check widget sensing method and call appropriate APIs */
                #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_IDAC_AUTOCAL_EN)
                    if (UL_testing_Tuning_SENSE_METHOD_CSX_E ==
                        UL_testing_Tuning_GET_SENSE_METHOD(&UL_testing_Tuning_dsFlash.wdgtArray[widgetId]))
                    {
                        /* Calibrate CSX widget */
                       UL_testing_Tuning_CSXCalibrateWidget(widgetId, UL_testing_Tuning_CSX_RAWCOUNT_CAL_LEVEL);
                    }
                #endif  /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_IDAC_AUTOCAL_EN) */

                #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_IDAC_AUTOCAL_EN)
                    if (UL_testing_Tuning_SENSE_METHOD_CSD_E ==
                        UL_testing_Tuning_GET_SENSE_METHOD(&UL_testing_Tuning_dsFlash.wdgtArray[widgetId]))
                    {
                        /* Calibrate CSD widget */
                        calibrateStatus = UL_testing_Tuning_CSDCalibrateWidget(widgetId, UL_testing_Tuning_CSD_RAWCOUNT_CAL_LEVEL);
                    }
                #endif  /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_IDAC_AUTOCAL_EN) */

            /* If only CSD is enabled, calibrate CSD sensor */
            #elif (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN)
                calibrateStatus = UL_testing_Tuning_CSDCalibrateWidget(widgetId, UL_testing_Tuning_CSD_RAWCOUNT_CAL_LEVEL);

            /* If only CSX is enabled, call CSX scan */
            #elif (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN)
                UL_testing_Tuning_CSXCalibrateWidget(widgetId, UL_testing_Tuning_CSX_RAWCOUNT_CAL_LEVEL);

            /* If only ISX is enabled, call ISX scan */
            #elif (UL_testing_Tuning_ENABLE == UL_testing_Tuning_ISX_EN)
                UL_testing_Tuning_ISXCalibrateWidget(widgetId, UL_testing_Tuning_ISX_RAWCOUNT_CAL_LEVEL);

            #else
                calibrateStatus = CYRET_UNKNOWN;
            #endif

             /* Update CRC */
            #if (UL_testing_Tuning_ENABLE ==UL_testing_Tuning_TST_WDGT_CRC_EN)
                UL_testing_Tuning_DsUpdateWidgetCrc(widgetId);
            #endif /* (UL_testing_Tuning_ENABLE ==UL_testing_Tuning_TST_WDGT_CRC_EN) */

        } while (0u);

        return calibrateStatus;
    }


    /*******************************************************************************
    * Function Name: UL_testing_Tuning_CalibrateAllWidgets
    ****************************************************************************//**
    *
    * \brief
    *  Calibrates the IDACs for all the widgets in the Component to the default
    *  target, this function detects the sensing method used by the widgets
    *  prior to calibration.
    *
    * \details
    *  Calibrates the IDACs for all the widgets in the Component to the default
    *  target value. This function detects the sensing method used by the widgets
    *  and regards the Enable compensation IDAC parameter.
    *  \if SECTION_I_REP
    *  For ISX mode, the frequency is also calibrated.
    *  \endif
    *
    *  \cond SECTION_C_LOW_LEVEL
    *  This function is available when the CSD and/or CSX Enable IDAC
    *  auto-calibration parameter is enabled.
    *  \endcond
    *  \if SECTION_I_REP
    *  This function is available when the ISX Enable Auto-calibration parameter
    *  is enabled.
    *  \endif
    *
    * \return
    *  Returns the status of the calibration process:
    *  - CYRET_SUCCESS - The operation is successfully completed.
    *  - CYRET_BAD_DATA - The calibration failed and the Component may not
    *    operate as expected.
    *
    *******************************************************************************/
    cystatus UL_testing_Tuning_CalibrateAllWidgets(void)
    {
        cystatus calibrateStatus = CYRET_SUCCESS;
        uint32 wdgtIndex;

        for (wdgtIndex = 0u; wdgtIndex < UL_testing_Tuning_TOTAL_WIDGETS; wdgtIndex++)
        {
            calibrateStatus |= UL_testing_Tuning_CalibrateWidget(wdgtIndex);
        }

        return calibrateStatus;
    }
#endif /* (UL_testing_Tuning_ANYMODE_AUTOCAL) */


#if (UL_testing_Tuning_CSD_SS_DIS != UL_testing_Tuning_CSD_AUTOTUNE)
    /*******************************************************************************
    * Function Name: UL_testing_Tuning_SsAutoTune
    ****************************************************************************//**
    *
    * \brief
    *  This API performs the parameters auto-tuning for the optimal UL_testing_Tuning operation.
    *
    * \details
    *  This API performs the following:
    *  - Calibrates Modulator and Compensation IDACs.
    *  - Tunes the Sense Clock optimal value to get a Sense Clock period greater than
    *    2*5*R*Cp.
    *  - Calculates the resolution for the optimal finger capacitance.
    *
    * \return
    *  Returns the status of the operation:
    *  - Zero     - All the widgets are auto-tuned successfully.
    *  - Non-zero - Auto-tuning failed for any widget.
    *
    *******************************************************************************/
    cystatus UL_testing_Tuning_SsAutoTune(void)
    {
        cystatus autoTuneStatus = CYRET_SUCCESS;
        uint32 wdgtIndex;

        uint32 cp = 0uL;
        #if (UL_testing_Tuning_CSD_MATRIX_WIDGET_EN || UL_testing_Tuning_CSD_TOUCHPAD_WIDGET_EN)
            uint32 cpRow = 0uL;
        #endif /* (UL_testing_Tuning_CSD_MATRIX_WIDGET_EN || UL_testing_Tuning_CSD_TOUCHPAD_WIDGET_EN) */

        uint32 cpWidget[UL_testing_Tuning_TOTAL_WIDGETS];
        UL_testing_Tuning_RAM_WD_BASE_STRUCT *ptrWdgt;
        AUTO_TUNE_CONFIG_TYPE autoTuneConfig;

        /* Configure common config variables */
        autoTuneConfig.snsClkConstantR = UL_testing_Tuning_CSD_SNSCLK_R_CONST;
        autoTuneConfig.vRef = UL_testing_Tuning_CSD_VREF_MV;
        autoTuneConfig.iDacGain = UL_testing_Tuning_CSD_IDAC_GAIN_VALUE_NA;

        /* Calculate snsClk Input Clock in KHz */
        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2)
            /* Dividers are chained (Fourth-generation HW block). Flip flop is not available */
            autoTuneConfig.snsClkInputClock = (CYDEV_BCLK__HFCLK__KHZ / UL_testing_Tuning_dsRam.modCsdClk);
        #elif ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_IS_M0S8PERI_BLOCK) && (UL_testing_Tuning_DISABLE == CY_PSOC4_4000))
            /* Dividers are not chained */
            autoTuneConfig.snsClkInputClock = CYDEV_BCLK__HFCLK__KHZ >> UL_testing_Tuning_FLIP_FLOP_DIV;
        #elif (UL_testing_Tuning_ENABLE == UL_testing_Tuning_IS_M0S8PERI_BLOCK)
            /* Dividers are not chained (PSoC 4000) */
            autoTuneConfig.snsClkInputClock = CYDEV_BCLK__HFCLK__KHZ >> UL_testing_Tuning_FLIP_FLOP_DIV;
        #else
            /* Dividers are chained (PSoC 4100, PSoC 4200) */
            autoTuneConfig.snsClkInputClock = (CYDEV_BCLK__HFCLK__KHZ / UL_testing_Tuning_dsRam.modCsdClk) >> UL_testing_Tuning_FLIP_FLOP_DIV;
        #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2) */

        /* If both CSD and CSX are enabled, calibrate widget using sensing method */
        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_CSX_EN)
            /* Initialize CSD mode */
            UL_testing_Tuning_SsCSDInitialize();
        #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_CSX_EN) */

        /*
        * Autotune phase #1:
        * - performing the first calibration at fixed settings
        * - getting sensor Cp
        * - getting sense clock frequency based on Cp
        */

        /* Tune sense clock for all the widgets */
        for (wdgtIndex = 0u; wdgtIndex < UL_testing_Tuning_TOTAL_WIDGETS; wdgtIndex++)
        {
            if (UL_testing_Tuning_SENSE_METHOD_CSD_E ==
                UL_testing_Tuning_GET_SENSE_METHOD(&UL_testing_Tuning_dsFlash.wdgtArray[wdgtIndex]))
            {
                ptrWdgt = (UL_testing_Tuning_RAM_WD_BASE_STRUCT *)
                          UL_testing_Tuning_dsFlash.wdgtArray[wdgtIndex].ptr2WdgtRam;

                /* Set calibration resolution to 12 bits */
                ptrWdgt->resolution = UL_testing_Tuning_CALIBRATION_RESOLUTION;

                /* Set clock source direct and sense clock frequency to 1.5 MHz */
                ptrWdgt->snsClkSource = (uint8)UL_testing_Tuning_CLK_SOURCE_DIRECT;
                ptrWdgt->snsClk = (uint16)((uint32)autoTuneConfig.snsClkInputClock / UL_testing_Tuning_CALIBRATION_FREQ_KHZ);
                #if (UL_testing_Tuning_CSD_MATRIX_WIDGET_EN || UL_testing_Tuning_CSD_TOUCHPAD_WIDGET_EN)
                    if ((UL_testing_Tuning_WD_TOUCHPAD_E == (UL_testing_Tuning_WD_TYPE_ENUM)UL_testing_Tuning_dsFlash.wdgtArray[wdgtIndex].wdgtType) ||
                        (UL_testing_Tuning_WD_MATRIX_BUTTON_E == (UL_testing_Tuning_WD_TYPE_ENUM)UL_testing_Tuning_dsFlash.wdgtArray[wdgtIndex].wdgtType))
                    {
                        ptrWdgt->rowSnsClkSource = (uint8)UL_testing_Tuning_CLK_SOURCE_DIRECT;
                        ptrWdgt->rowSnsClk = (uint16)((uint32)autoTuneConfig.snsClkInputClock / UL_testing_Tuning_CALIBRATION_FREQ_KHZ);
                    }
                #endif

                /* Calibrate CSD widget to the default calibration target */
                (void)UL_testing_Tuning_CSDCalibrateWidget(wdgtIndex, UL_testing_Tuning_CSD_AUTOTUNE_CAL_LEVEL);

                #if (UL_testing_Tuning_CSD_MATRIX_WIDGET_EN || UL_testing_Tuning_CSD_TOUCHPAD_WIDGET_EN)
                    if ((UL_testing_Tuning_WD_TOUCHPAD_E == (UL_testing_Tuning_WD_TYPE_ENUM)UL_testing_Tuning_dsFlash.wdgtArray[wdgtIndex].wdgtType) ||
                        (UL_testing_Tuning_WD_MATRIX_BUTTON_E == (UL_testing_Tuning_WD_TYPE_ENUM)UL_testing_Tuning_dsFlash.wdgtArray[wdgtIndex].wdgtType))
                    {
                        /* Get pointer to Sense Clock Divider for columns */
                        autoTuneConfig.ptrSenseClk = &ptrWdgt->rowSnsClk;

                        /* Get IDAC */
                        autoTuneConfig.iDac = UL_testing_Tuning_calibratedIdacRow;

                        /* Calculate achived calibration level */
                        autoTuneConfig.calTarget = (uint16)(((uint32)UL_testing_Tuning_calibratedRawcountRow * UL_testing_Tuning_CSD_AUTOTUNE_CAL_UNITS) /
                                ((uint32)(0x01uL << UL_testing_Tuning_CALIBRATION_RESOLUTION) - 1u));

                        /* Find correct sense clock value */
                        cpRow = SmartSense_TunePrescalers(&autoTuneConfig);

                        if ((UL_testing_Tuning_CP_MAX + UL_testing_Tuning_CP_ERROR) <= cpRow)
                        {
                            autoTuneStatus = CYRET_BAD_DATA;
                        }

                        /*
                        * Multiply the sense Clock Divider by 2 while the desired Sense Clock Frequency is greater
                        * than maximum supported Sense Clock Frequency.
                        */
                        while((((uint32)autoTuneConfig.snsClkInputClock) > ((uint32)ptrWdgt->snsClk * UL_testing_Tuning_CSD_SNS_FREQ_KHZ_MAX)) ||
                                (UL_testing_Tuning_MIN_SNS_CLK_DIVIDER > ptrWdgt->snsClk))
                        {
                            ptrWdgt->snsClk <<= 1u;
                        }

                        #if (UL_testing_Tuning_ENABLE != UL_testing_Tuning_CSDV2)
                            #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_IS_M0S8PERI_BLOCK)
                                /* Make sure that ModClk >= 2 * rowSnsClk */
                                if (UL_testing_Tuning_dsRam.modCsdClk >= ((uint8)(ptrWdgt->rowSnsClk << UL_testing_Tuning_FLIP_FLOP_DIV)))
                                {
                                     ptrWdgt->rowSnsClk = UL_testing_Tuning_dsRam.modCsdClk;
                                }
                            #else
                                /* Sense clock never equals to Modulator clock for chained dividers because of Flip Flop */
                            #endif
                        #endif /* (UL_testing_Tuning_ENABLE != UL_testing_Tuning_CSDV2) */
                    }
                #endif /* (UL_testing_Tuning_CSD_MATRIX_WIDGET_EN || UL_testing_Tuning_CSD_TOUCHPAD_WIDGET_EN) */

                /* Get pointer to Sense Clock Divider for columns */
                autoTuneConfig.ptrSenseClk = &ptrWdgt->snsClk;

                /* Get IDAC */
                autoTuneConfig.iDac = UL_testing_Tuning_calibratedIdac;

                /* Calculate achived calibration level */
                autoTuneConfig.calTarget = (uint16)(((uint32)UL_testing_Tuning_calibratedRawcount * UL_testing_Tuning_CSD_AUTOTUNE_CAL_UNITS) /
                        ((uint32)(0x01uL << UL_testing_Tuning_CALIBRATION_RESOLUTION) - 1u));

                /* Find correct sense clock value */
                cp = SmartSense_TunePrescalers(&autoTuneConfig);

                if ((UL_testing_Tuning_CP_MAX + UL_testing_Tuning_CP_ERROR) <= cp)
                {
                    autoTuneStatus = CYRET_BAD_DATA;
                }

                /*
                * Multiply the sense Clock Divider by 2 while the desired Sense Clock Frequency is greater
                * than MAX supported Sense Clock Frequency.
                */
                while((((uint32)autoTuneConfig.snsClkInputClock) > ((uint32)ptrWdgt->snsClk * UL_testing_Tuning_CSD_SNS_FREQ_KHZ_MAX)) ||
                        (UL_testing_Tuning_MIN_SNS_CLK_DIVIDER > ptrWdgt->snsClk))
                {
                    ptrWdgt->snsClk <<= 1u;
                }

                #if (UL_testing_Tuning_ENABLE != UL_testing_Tuning_CSDV2)
                    #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_IS_M0S8PERI_BLOCK)
                        /* Make sure that ModClk >= 2 * snsClk */
                        if (UL_testing_Tuning_dsRam.modCsdClk >= ((uint8)(ptrWdgt->snsClk << UL_testing_Tuning_FLIP_FLOP_DIV)))
                        {
                             ptrWdgt->snsClk = UL_testing_Tuning_dsRam.modCsdClk;
                        }
                    #else
                        /* Sense clock never equals to Modulator clock for chained dividers because of Flip Flop */
                    #endif
                #endif /* (UL_testing_Tuning_ENABLE != UL_testing_Tuning_CSDV2) */

                cpWidget[wdgtIndex] = cp;

                #if (UL_testing_Tuning_CSD_MATRIX_WIDGET_EN || UL_testing_Tuning_CSD_TOUCHPAD_WIDGET_EN)
                    if ((UL_testing_Tuning_WD_TOUCHPAD_E == (UL_testing_Tuning_WD_TYPE_ENUM)UL_testing_Tuning_dsFlash.wdgtArray[wdgtIndex].wdgtType) ||
                        (UL_testing_Tuning_WD_MATRIX_BUTTON_E == (UL_testing_Tuning_WD_TYPE_ENUM)UL_testing_Tuning_dsFlash.wdgtArray[wdgtIndex].wdgtType))
                    {
                        if (cpRow > cp)
                        {
                            cpWidget[wdgtIndex] = cpRow;
                        }
                    }
                #endif /* (UL_testing_Tuning_CSD_MATRIX_WIDGET_EN || UL_testing_Tuning_CSD_TOUCHPAD_WIDGET_EN) */
            }
            else
            {
                #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN)
                    /* Non-CSD widget */
                    cpWidget[wdgtIndex] = 1u;
                #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN) */
            }
        }

        /*
        * Autotune phase #2:
        * - repeating calibration with new sense clock frequency
        * - getting resolution
        */

        /* Tune resolution for all the widgets */
        for (wdgtIndex = 0u; wdgtIndex < UL_testing_Tuning_TOTAL_WIDGETS; wdgtIndex++)
        {
            if (UL_testing_Tuning_SENSE_METHOD_CSD_E ==
                UL_testing_Tuning_GET_SENSE_METHOD(&UL_testing_Tuning_dsFlash.wdgtArray[wdgtIndex]))
            {
                ptrWdgt = (UL_testing_Tuning_RAM_WD_BASE_STRUCT *)
                          UL_testing_Tuning_dsFlash.wdgtArray[wdgtIndex].ptr2WdgtRam;

                /* Calibrate CSD widget to the default calibration target */
                autoTuneStatus |= UL_testing_Tuning_CSDCalibrateWidget(wdgtIndex, UL_testing_Tuning_CSD_AUTOTUNE_CAL_LEVEL);

                /* Get pointer to Sense Clock Divider (column or row) */
                autoTuneConfig.ptrSenseClk = &ptrWdgt->snsClk;

                /* Set parasitic capacitance for columns */
                autoTuneConfig.sensorCap = cpWidget[wdgtIndex];

                /* Get IDAC */
                autoTuneConfig.iDac = UL_testing_Tuning_calibratedIdac;

                #if (UL_testing_Tuning_CSD_MATRIX_WIDGET_EN || UL_testing_Tuning_CSD_TOUCHPAD_WIDGET_EN)
                    if ((UL_testing_Tuning_WD_TOUCHPAD_E == (UL_testing_Tuning_WD_TYPE_ENUM)UL_testing_Tuning_dsFlash.wdgtArray[wdgtIndex].wdgtType) ||
                        (UL_testing_Tuning_WD_MATRIX_BUTTON_E == (UL_testing_Tuning_WD_TYPE_ENUM)UL_testing_Tuning_dsFlash.wdgtArray[wdgtIndex].wdgtType))

                    {
                        /* Set the minimum sense clock frequency to calculate the resolution */
                        if (ptrWdgt->snsClk < ptrWdgt->rowSnsClk)
                        {
                            /* Rewrite pointer to Sense Clock Divider for rows */
                            autoTuneConfig.ptrSenseClk = &ptrWdgt->rowSnsClk;

                            /* Set parasitic capacitance for rows */
                            autoTuneConfig.sensorCap = cpWidget[wdgtIndex];

                            /* Get IDAC */
                            autoTuneConfig.iDac = UL_testing_Tuning_calibratedIdacRow;
                        }
                    }
                #endif /* (UL_testing_Tuning_CSD_MATRIX_WIDGET_EN || UL_testing_Tuning_CSD_TOUCHPAD_WIDGET_EN) */

                /* Get finger capacitance */
                autoTuneConfig.fingerCap = ptrWdgt->fingerCap;

                /* Init pointer to sigPFC */
                autoTuneConfig.sigPFC = &ptrWdgt->sigPFC;

                /* Find resolution */
                ptrWdgt->resolution = SmartSense_TuneSensitivity(&autoTuneConfig);
            }
        }

        /*
        * Autotune phase #3:
        * - selecting a widget clock source if AUTO
        * - repeating calibration with found clock frequency, resolution and clock source
        * - updating sensitivity
        */

        /* Tune sensitivity for all the widgets */
        for (wdgtIndex = 0u; wdgtIndex < UL_testing_Tuning_TOTAL_WIDGETS; wdgtIndex++)
        {
            if (UL_testing_Tuning_SENSE_METHOD_CSD_E ==
                UL_testing_Tuning_GET_SENSE_METHOD(&UL_testing_Tuning_dsFlash.wdgtArray[wdgtIndex]))
            {
                ptrWdgt = (UL_testing_Tuning_RAM_WD_BASE_STRUCT *)
                          UL_testing_Tuning_dsFlash.wdgtArray[wdgtIndex].ptr2WdgtRam;

                UL_testing_Tuning_SsSetWidgetSenseClkSrc(wdgtIndex, ptrWdgt);

                /* Calibrate CSD widget to the default calibration target */
                autoTuneStatus |= UL_testing_Tuning_CSDCalibrateWidget(wdgtIndex, UL_testing_Tuning_CSD_AUTOTUNE_CAL_LEVEL);

                #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_WDGT_CRC_EN)
                    UL_testing_Tuning_DsUpdateWidgetCrc(wdgtIndex);
                #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_WDGT_CRC_EN) */
            }
        }

        return autoTuneStatus;
    }
#endif /* (UL_testing_Tuning_CSD_SS_DIS != UL_testing_Tuning_CSD_AUTOTUNE)) */


#if (UL_testing_Tuning_DISABLE == UL_testing_Tuning_CSDV2)
    #if ( ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN) \
        || (UL_testing_Tuning_ENABLE == UL_testing_Tuning_ISX_EN)) ||\
         (UL_testing_Tuning_IDAC_SINKING == UL_testing_Tuning_CSD_IDAC_CONFIG))
        /*******************************************************************************
        * Function Name: UL_testing_Tuning_SsTrimIdacsSinking
        ****************************************************************************//**
        *
        * \brief
        *  This function loads trim values from SFLASH rows to calibrate
        *  IDAC1 and IDAC2 for Sinking CSD Mode
        *
        * \details
        *  Function reads trim value from SFLASH and loads it into IDAC trim register.
        *  If Compensation IDAC is disabled the function loads trim values for IDAC1 only.
        *
        *******************************************************************************/
        static void UL_testing_Tuning_SsTrimIdacsSinking(void)
        {
            uint32 trimValue;

            /* Mod iDAC Sinking Mode */
            trimValue = CY_GET_REG32(UL_testing_Tuning_TRIM2_PTR) & ~UL_testing_Tuning_IDAC_TRIM2_MOD_SNK_MASK;
            trimValue |= ((uint32)CY_GET_REG8(UL_testing_Tuning_SFLASH_TRIM2_PTR) &
                                                      UL_testing_Tuning_SFLASH_TRIM_IDAC_MOD_MASK) ;

            #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_IDAC_COMP_EN)
                /* Comp iDAC Sinking Mode */
                trimValue &= ~UL_testing_Tuning_IDAC_TRIM2_COMP_SNK_MASK;
                trimValue |= ((uint32)CY_GET_REG8(UL_testing_Tuning_SFLASH_TRIM2_PTR) &
                                                          UL_testing_Tuning_SFLASH_TRIM_IDAC_COMP_MASK);
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_IDAC_COMP_EN) */

            /* Update IDAC trim bits for gain control in current sink mode */
            CY_SET_REG32(UL_testing_Tuning_TRIM2_PTR, trimValue);
        }
    #endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN) || \
               (UL_testing_Tuning_IDAC_SINKING == UL_testing_Tuning_CSD_IDAC_CONFIG)) */


    #if ( ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN) \
        || (UL_testing_Tuning_ENABLE == UL_testing_Tuning_ISX_EN)) || \
         (UL_testing_Tuning_IDAC_SOURCING == UL_testing_Tuning_CSD_IDAC_CONFIG))
        /*******************************************************************************
        * Function Name: UL_testing_Tuning_SsTrimIdacsSourcing
        ****************************************************************************//**
        *
        * \brief
        *  This function loads trim values from SFLASH rows to calibrate
        *  IDAC1 and IDAC2 for Sourcing CSD Mode
        *
        * \details
        *  Function reads trim value from SFLASH and loads it into IDAC trim register.
        *  If Compensation IDAC is disabled the function loads trim values for IDAC1 only.
        *
        *******************************************************************************/
        static void UL_testing_Tuning_SsTrimIdacsSourcing(void)
        {
            uint32 trimValue;

            /* Mod iDAC Sourcing Mode */
            trimValue = CY_GET_REG32(UL_testing_Tuning_TRIM1_PTR) & ~UL_testing_Tuning_IDAC_TRIM1_MOD_SRC_MASK;
            trimValue |= ((uint32)CY_GET_REG8(UL_testing_Tuning_SFLASH_TRIM1_PTR) &
                                                      UL_testing_Tuning_SFLASH_TRIM_IDAC_MOD_MASK);

            #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_IDAC_COMP_EN)
                 /* Comp iDAC Sourcing Mode */
                trimValue &= ~UL_testing_Tuning_IDAC_TRIM1_COMP_SRC_MASK;
                trimValue |= ((uint32)CY_GET_REG8(UL_testing_Tuning_SFLASH_TRIM1_PTR) &
                                                          UL_testing_Tuning_SFLASH_TRIM_IDAC_COMP_MASK);
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_IDAC_COMP_EN) */

            /* Update IDAC trim bits for gain control in current source mode */
            CY_SET_REG32(UL_testing_Tuning_TRIM1_PTR, trimValue);
        }
    #endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN) || \
               (UL_testing_Tuning_IDAC_SOURCING == UL_testing_Tuning_CSD_IDAC_CONFIG)) */


    /*******************************************************************************
    * Function Name: UL_testing_Tuning_SsTrimIdacs
    ****************************************************************************//**
    *
    * \brief
    *  This function loads trim values from SFLASH rows to calibrate
    *  IDAC1 and IDAC2 for CSD Mode
    *
    * \details
    *  If CSX mode is enabled the function loads trim values for both sink and source
    *  mode. If CSX mode is disabled the function loads trim values for sink or
    *  source mode based on sink/source mode configuration.
    *  If Compensation IDAC is disabled the function loads trim values for IDAC1 only.
    *
    *******************************************************************************/
    static void UL_testing_Tuning_SsTrimIdacs(void)
    {
        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN) || (UL_testing_Tuning_ENABLE == UL_testing_Tuning_ISX_EN)
            UL_testing_Tuning_SsTrimIdacsSinking();
            UL_testing_Tuning_SsTrimIdacsSourcing();
        #elif (UL_testing_Tuning_IDAC_SINKING == UL_testing_Tuning_CSD_IDAC_CONFIG)
            UL_testing_Tuning_SsTrimIdacsSinking();
        #elif (UL_testing_Tuning_IDAC_SOURCING == UL_testing_Tuning_CSD_IDAC_CONFIG)
            UL_testing_Tuning_SsTrimIdacsSourcing();
        #else
            #error "Not supported Mode, Component cannot work in this mode"
        #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN) || (UL_testing_Tuning_ENABLE == UL_testing_Tuning_ISX_EN) */
    }
#endif  /* (UL_testing_Tuning_DISABLE == UL_testing_Tuning_CSDV2) */


#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_MULTI_FREQ_SCAN_EN)
    /*******************************************************************************
    * Function Name: UL_testing_Tuning_SsChangeImoFreq
    ****************************************************************************//**
    *
    * \brief
    *  This function changes the IMO frequency.
    *
    * \details
    *  The IMO frequency can have three offsets: 0%, -5% and +5%. The frequency
    *  trims are contained in the UL_testing_Tuning_immunity[value] array for each
    *  frequency channel.
    *
    * \param value The frequency channel ID.
    *
    *******************************************************************************/
    void UL_testing_Tuning_SsChangeImoFreq(uint32 value)
    {
        CY_SET_REG32(CY_SYS_CLK_IMO_TRIM1_PTR, UL_testing_Tuning_immunity[value]);
    }

    /*******************************************************************************
    * Function Name: UL_testing_Tuning_SsChangeClkFreq
    ****************************************************************************//**
    *
    * \brief
    *  This function changes the sensor clock frequency by configuring
    *  the corresponding divider.
    *
    * \details
    *  This function changes the sensor clock frequency by configuring
    *  the corresponding divider.
    *
    * \param chId
    *  The frequency channel ID.
    *
    *******************************************************************************/
    void UL_testing_Tuning_SsChangeClkFreq(uint32 chId)
    {
        uint32 snsClkDivider;
        uint32 freqOffset1 = 0u;
        uint32 freqOffset2 = 0u;

        #if (0u != UL_testing_Tuning_TOTAL_CSD_WIDGETS)
            #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2)
                uint32 conversionsNum;
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2) */

            #if((UL_testing_Tuning_CLK_SOURCE_PRS8  == UL_testing_Tuning_CSD_SNS_CLK_SOURCE) ||\
                (UL_testing_Tuning_CLK_SOURCE_PRS12 == UL_testing_Tuning_CSD_SNS_CLK_SOURCE) ||\
                (UL_testing_Tuning_CLK_SOURCE_PRSAUTO == UL_testing_Tuning_CSD_SNS_CLK_SOURCE))
                uint32 snsClkSrc;
            #endif
        #endif

        #if ((0u != UL_testing_Tuning_TOTAL_CSD_WIDGETS) || \
             ((UL_testing_Tuning_DISABLE == UL_testing_Tuning_CSX_COMMON_TX_CLK_EN) && (0u != UL_testing_Tuning_TOTAL_CSX_WIDGETS)))
            UL_testing_Tuning_FLASH_WD_STRUCT const *ptrFlashWdgt = &UL_testing_Tuning_dsFlash.wdgtArray[UL_testing_Tuning_widgetIndex];
            UL_testing_Tuning_RAM_WD_BASE_STRUCT const *ptrWdgt = (UL_testing_Tuning_RAM_WD_BASE_STRUCT *)ptrFlashWdgt->ptr2WdgtRam;
        #endif

        switch(UL_testing_Tuning_GET_SENSE_METHOD(&UL_testing_Tuning_dsFlash.wdgtArray[UL_testing_Tuning_widgetIndex]))
        {
        #if (0u != UL_testing_Tuning_TOTAL_CSD_WIDGETS)
            case UL_testing_Tuning_SENSE_METHOD_CSD_E:
                /* Get sensor clock divider value */
                #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_COMMON_SNS_CLK_EN)
                    snsClkDivider = UL_testing_Tuning_dsRam.snsCsdClk;
                #else /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_COMMON_SNS_CLK_EN) */
                    #if (UL_testing_Tuning_CSD_MATRIX_WIDGET_EN || UL_testing_Tuning_CSD_TOUCHPAD_WIDGET_EN)
                        /* Get SnsClck divider for rows or columns */
                        if (UL_testing_Tuning_dsFlash.wdgtArray[UL_testing_Tuning_widgetIndex].numCols <= UL_testing_Tuning_sensorIndex)
                        {
                            snsClkDivider = ptrWdgt->rowSnsClk;
                        }
                        else
                        {
                            snsClkDivider = ptrWdgt->snsClk;
                        }
                    #else
                        snsClkDivider = ptrWdgt->snsClk;
                    #endif /* (UL_testing_Tuning_CSD_MATRIX_WIDGET_EN || UL_testing_Tuning_CSD_TOUCHPAD_WIDGET_EN) */
                #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_COMMON_SNS_CLK_EN) */

                freqOffset1 = UL_testing_Tuning_CSD_MFS_DIVIDER_OFFSET_F1;
                freqOffset2 = UL_testing_Tuning_CSD_MFS_DIVIDER_OFFSET_F2;

                #if((UL_testing_Tuning_CLK_SOURCE_PRS8  == UL_testing_Tuning_CSD_SNS_CLK_SOURCE) ||\
                    (UL_testing_Tuning_CLK_SOURCE_PRS12 == UL_testing_Tuning_CSD_SNS_CLK_SOURCE) ||\
                    (UL_testing_Tuning_CLK_SOURCE_PRSAUTO == UL_testing_Tuning_CSD_SNS_CLK_SOURCE))
                    /* Get sense clk source */
                    #if (UL_testing_Tuning_CSD_MATRIX_WIDGET_EN || UL_testing_Tuning_CSD_TOUCHPAD_WIDGET_EN)
                        /* Get SnsClc Source for rows or columns */
                        if (UL_testing_Tuning_dsFlash.wdgtArray[UL_testing_Tuning_widgetIndex].numCols <= UL_testing_Tuning_sensorIndex)
                        {
                            snsClkSrc = (uint32)ptrWdgt->rowSnsClkSource;
                        }
                        else
                        {
                            snsClkSrc = (uint32)ptrWdgt->snsClkSource;
                        }
                    #else
                        snsClkSrc = (uint32)ptrWdgt->snsClkSource;
                    #endif /* (UL_testing_Tuning_CSD_MATRIX_WIDGET_EN || UL_testing_Tuning_CSD_TOUCHPAD_WIDGET_EN) */

                    switch (snsClkSrc)
                    {
                    case UL_testing_Tuning_CLK_SOURCE_PRS8:
                    case UL_testing_Tuning_CLK_SOURCE_PRS12:
                        /* Multiply by 2 for PRS8/PRS12 mode */
                        freqOffset1 <<= 1u;
                        freqOffset2 <<= 1u;
                        break;

                    default:
                        break;
                    }
                #endif

                /* Change the divider based on the chId */
                switch (chId)
                {
                    case UL_testing_Tuning_FREQ_CHANNEL_1:
                    {
                        snsClkDivider += freqOffset1;
                        break;
                    }
                    case UL_testing_Tuning_FREQ_CHANNEL_2:
                    {
                        snsClkDivider += freqOffset2;
                        break;
                    }
                    default:
                    {
                        break;
                    }
                }

                #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2)
                    /* Set Number Of Conversions based on scanning resolution */
                    conversionsNum = UL_testing_Tuning_SsCSDGetNumberOfConversions(snsClkDivider, (uint32)ptrWdgt->resolution, (uint32)ptrWdgt->snsClkSource);
                    CY_SET_REG32(UL_testing_Tuning_SEQ_NORM_CNT_PTR, (conversionsNum & UL_testing_Tuning_SEQ_NORM_CNT_CONV_CNT_MASK));
                #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2) */

                #if((UL_testing_Tuning_CLK_SOURCE_PRS8  == UL_testing_Tuning_CSD_SNS_CLK_SOURCE) ||\
                    (UL_testing_Tuning_CLK_SOURCE_PRS12 == UL_testing_Tuning_CSD_SNS_CLK_SOURCE) ||\
                    (UL_testing_Tuning_CLK_SOURCE_PRSAUTO == UL_testing_Tuning_CSD_SNS_CLK_SOURCE))
                    switch (snsClkSrc)
                    {
                    case UL_testing_Tuning_CLK_SOURCE_PRS8:
                    case UL_testing_Tuning_CLK_SOURCE_PRS12:
                        /* Divide by 2 for PRS8/PRS12 mode */
                        snsClkDivider >>= 1;
                        if (snsClkDivider == 0u)
                        {
                            snsClkDivider = 1u;
                        }
                        break;

                    default:
                        break;
                    }
                #endif

                /* Configure the new divider */
                UL_testing_Tuning_SsSetSnsClockDivider(snsClkDivider);

                break;
        #endif /* #if (0u != UL_testing_Tuning_TOTAL_CSD_WIDGETS) */

        #if (0u != UL_testing_Tuning_TOTAL_CSX_WIDGETS)
            case UL_testing_Tuning_SENSE_METHOD_CSX_E:
                #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_COMMON_TX_CLK_EN)
                    snsClkDivider = UL_testing_Tuning_dsRam.snsCsxClk;
                #else /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_COMMON_TX_CLK_EN) */
                    snsClkDivider = ptrWdgt->snsClk;
                #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_COMMON_TX_CLK_EN) */
                freqOffset1 = UL_testing_Tuning_CSX_MFS_DIVIDER_OFFSET_F1;
                freqOffset2 = UL_testing_Tuning_CSX_MFS_DIVIDER_OFFSET_F2;

                /* Change the divider based on the chId */
                switch (chId)
                {
                    case UL_testing_Tuning_FREQ_CHANNEL_1:
                    {
                        snsClkDivider += freqOffset1;
                        break;
                    }
                    case UL_testing_Tuning_FREQ_CHANNEL_2:
                    {
                        snsClkDivider += freqOffset2;
                        break;
                    }
                    default:
                    {
                        break;
                    }
                }

                /* Configure the new divider */
                UL_testing_Tuning_SsSetSnsClockDivider(snsClkDivider);

                break;
        #endif /* #if (0u != UL_testing_Tuning_TOTAL_CSX_WIDGETS) */

        default:
            CYASSERT(0);
            break;
        }
    }
#endif  /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_MULTI_FREQ_SCAN_EN) */


#if((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN) || \
    ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN) && \
     (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2)))
/*******************************************************************************
* Function Name: UL_testing_Tuning_SsInitializeSourceSenseClk
****************************************************************************//**
*
* \brief
*  Sets a source for Sense Clk for all CSD widgets.
*
* \details
*  Updates snsClkSource and rowSnsClkSource with a source for the sense Clk.
*  for all CSD widgets.
*
*******************************************************************************/
void UL_testing_Tuning_SsInitializeSourceSenseClk(void)
{
    uint32 wdgtIndex;
    UL_testing_Tuning_RAM_WD_BASE_STRUCT *ptrWdgt;

    for (wdgtIndex = 0u; wdgtIndex < UL_testing_Tuning_TOTAL_WIDGETS; wdgtIndex++)
    {
        ptrWdgt = (UL_testing_Tuning_RAM_WD_BASE_STRUCT *)UL_testing_Tuning_dsFlash.wdgtArray[wdgtIndex].ptr2WdgtRam;

        switch (UL_testing_Tuning_GET_SENSE_METHOD(&UL_testing_Tuning_dsFlash.wdgtArray[wdgtIndex]))
        {
        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN)
            case UL_testing_Tuning_SENSE_METHOD_CSD_E:
                UL_testing_Tuning_SsSetWidgetSenseClkSrc(wdgtIndex, ptrWdgt);
                break;
        #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN) */
        #if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN) && (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2))
            case UL_testing_Tuning_SENSE_METHOD_CSX_E:
                UL_testing_Tuning_SsSetWidgetTxClkSrc(wdgtIndex, ptrWdgt);
                break;
        #endif /* (((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN) || (UL_testing_Tuning_ENABLE == UL_testing_Tuning_ISX_EN)) && (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2)) */
        default:
            break;
        }
        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_WDGT_CRC_EN)
            UL_testing_Tuning_DsUpdateWidgetCrc(wdgtIndex);
        #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_WDGT_CRC_EN) */
    }
}
#endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN) || \
           (((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN) || \
           (UL_testing_Tuning_ENABLE == UL_testing_Tuning_ISX_EN)) && \
           (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2))) */


#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN)
    /*******************************************************************************
    * Function Name: UL_testing_Tuning_SsSetWidgetSenseClkSrc
    ****************************************************************************//**
    *
    * \brief
    *  Sets a source for the sense clock for a widget.
    *
    * \param wdgtIndex
    *  Specifies the ID of the widget.
    * \param ptrWdgt
    *  The pointer to the RAM_WD_BASE_STRUCT structure.
    *
    * \details
    *  Updates snsClkSource and rowSnsClkSource with a source for the sense Clk for a
    *  widget.
    *
    *******************************************************************************/
    static void UL_testing_Tuning_SsSetWidgetSenseClkSrc(uint32 wdgtIndex, UL_testing_Tuning_RAM_WD_BASE_STRUCT * ptrWdgt)
    {
        uint8 lfsrSize;
        uint8 lfsrScale;

        #if(UL_testing_Tuning_CLK_SOURCE_PRSAUTO == UL_testing_Tuning_CSD_SNS_CLK_SOURCE)
            #if(UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2)
                uint32 conversionsNum;
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2) */
            uint32 snsClkDivider;
        #endif /* (UL_testing_Tuning_CLK_SOURCE_PRSAUTO == UL_testing_Tuning_CSD_SNS_CLK_SOURCE) */

        #if(UL_testing_Tuning_CLK_SOURCE_PRSAUTO == UL_testing_Tuning_CSD_SNS_CLK_SOURCE)
            snsClkDivider = UL_testing_Tuning_SsCSDGetColSnsClkDivider(wdgtIndex);

            #if(UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2)
                conversionsNum = UL_testing_Tuning_SsCSDGetNumberOfConversions(snsClkDivider, (uint32)ptrWdgt->resolution,
                                                                                             UL_testing_Tuning_CLK_SOURCE_DIRECT);
                lfsrSize = UL_testing_Tuning_SsCalcLfsrSize(snsClkDivider, conversionsNum);
                if (UL_testing_Tuning_CLK_SOURCE_DIRECT == lfsrSize)
                {
                    /*
                    * Multiplying of the snsClkDivider by 2 is needed to pass the
                    * average PRS frequency through the argument.
                    */
                    lfsrSize = UL_testing_Tuning_SsCSDCalcPrsSize(snsClkDivider << 1uL, (uint32)ptrWdgt->resolution);
                }
                lfsrScale = UL_testing_Tuning_SsCalcLfsrScale(snsClkDivider, lfsrSize);
            #else
                /*
                * Multiplying of the snsClkDivider by 2 is needed to pass the
                * average PRS frequency through the argument.
                */
                lfsrSize = UL_testing_Tuning_SsCSDCalcPrsSize(snsClkDivider << 1uL, (uint32)ptrWdgt->resolution);
                lfsrScale = 0u;
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2) */
        #else
            lfsrSize = (uint8)UL_testing_Tuning_DEFAULT_MODULATION_MODE;
            lfsrScale = 0u;
            (void)wdgtIndex; /* This parameter is unused in such configurations */
        #endif /* (UL_testing_Tuning_CLK_SOURCE_PRSAUTO == UL_testing_Tuning_CSD_SNS_CLK_SOURCE) */

        ptrWdgt->snsClkSource = lfsrSize | (uint8)(lfsrScale << UL_testing_Tuning_CLK_SOURCE_LFSR_SCALE_OFFSET);

        #if (UL_testing_Tuning_CSD_MATRIX_WIDGET_EN || UL_testing_Tuning_CSD_TOUCHPAD_WIDGET_EN)
            if ((UL_testing_Tuning_WD_TOUCHPAD_E == (UL_testing_Tuning_WD_TYPE_ENUM)UL_testing_Tuning_dsFlash.wdgtArray[wdgtIndex].wdgtType) ||
                (UL_testing_Tuning_WD_MATRIX_BUTTON_E == (UL_testing_Tuning_WD_TYPE_ENUM)UL_testing_Tuning_dsFlash.wdgtArray[wdgtIndex].wdgtType))
            {
                #if(UL_testing_Tuning_CLK_SOURCE_PRSAUTO == UL_testing_Tuning_CSD_SNS_CLK_SOURCE)
                    snsClkDivider = UL_testing_Tuning_SsCSDGetRowSnsClkDivider(wdgtIndex);

                    #if(UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2)
                        lfsrSize = UL_testing_Tuning_SsCalcLfsrSize(snsClkDivider, conversionsNum);
                        if (UL_testing_Tuning_CLK_SOURCE_DIRECT == lfsrSize)
                        {
                            /*
                            * Multiplying of the snsClkDivider by 2 is needed to pass the
                            * average PRS frequency through the argument.
                            */
                            lfsrSize = UL_testing_Tuning_SsCSDCalcPrsSize(snsClkDivider << 1uL, (uint32)ptrWdgt->resolution);
                        }
                        lfsrScale = UL_testing_Tuning_SsCalcLfsrScale(snsClkDivider, lfsrSize);
                    #else
                        /*
                        * Multiplying of the snsClkDivider by 2 is needed to pass the
                        * average PRS frequency through the argument.
                        */
                        lfsrSize = UL_testing_Tuning_SsCSDCalcPrsSize(snsClkDivider << 1uL, (uint32)ptrWdgt->resolution);
                        lfsrScale = 0u;
                    #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2) */
                #else
                    lfsrSize = (uint8)UL_testing_Tuning_DEFAULT_MODULATION_MODE;
                    lfsrScale = 0u;
                #endif /* (UL_testing_Tuning_CLK_SOURCE_PRSAUTO == UL_testing_Tuning_CSD_SNS_CLK_SOURCE) */
                ptrWdgt->rowSnsClkSource = lfsrSize | (uint8)(lfsrScale << UL_testing_Tuning_CLK_SOURCE_LFSR_SCALE_OFFSET);
            }
        #endif /* (UL_testing_Tuning_CSD_MATRIX_WIDGET_EN || UL_testing_Tuning_CSD_TOUCHPAD_WIDGET_EN) */
    }
#endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN) */


#if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN) && (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2))
    /*******************************************************************************
    * Function Name: UL_testing_Tuning_SsSetWidgetTxClkSrc
    ****************************************************************************//**
    *
    * \brief
    *  Sets a source for the Tx clock for a widget.
    *
    * \param wdgtIndex
    *  Specifies the ID of the widget.
    * \param ptrWdgt
    *  The pointer to the RAM_WD_BASE_STRUCT structure.
    *
    * \details
    *  Updates snsClkSource with a source for Tx Clk for a widget.
    *
    *******************************************************************************/
    static void UL_testing_Tuning_SsSetWidgetTxClkSrc(uint32 wdgtIndex, UL_testing_Tuning_RAM_WD_BASE_STRUCT * ptrWdgt)
    {
        uint8 lfsrSize;
        uint8 lfsrScale;

        #if ((UL_testing_Tuning_CLK_SOURCE_PRSAUTO == UL_testing_Tuning_CSX_TX_CLK_SOURCE) && \
             (UL_testing_Tuning_DISABLE == UL_testing_Tuning_CSX_SKIP_OVSMPL_SPECIFIC_NODES))
            uint32 conversionsNum;
            uint32 snsClkDivider;
        #endif

        #if(UL_testing_Tuning_CLK_SOURCE_PRSAUTO == UL_testing_Tuning_CSX_TX_CLK_SOURCE)
            #if (UL_testing_Tuning_DISABLE == UL_testing_Tuning_CSX_SKIP_OVSMPL_SPECIFIC_NODES)
                conversionsNum = (uint32)ptrWdgt->resolution;
                snsClkDivider = UL_testing_Tuning_SsCSXGetTxClkDivider(wdgtIndex);
                lfsrSize = UL_testing_Tuning_SsCalcLfsrSize(snsClkDivider, conversionsNum);
                lfsrScale = UL_testing_Tuning_SsCalcLfsrScale(snsClkDivider, lfsrSize);
            #else
                lfsrSize = (uint8)UL_testing_Tuning_CLK_SOURCE_DIRECT;
                lfsrScale = 0u;
                /* Unused function argument */
                (void)wdgtIndex;
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_SKIP_OVSMPL_SPECIFIC_NODES) */
        #else
            lfsrSize = (uint8)UL_testing_Tuning_CSX_TX_CLK_SOURCE;
            lfsrScale = 0u;
            /* Unused function argument */
            (void)wdgtIndex;
        #endif /* (UL_testing_Tuning_CLK_SOURCE_PRSAUTO == UL_testing_Tuning_CSX_TX_CLK_SOURCE) */

        ptrWdgt->snsClkSource = lfsrSize | (uint8)(lfsrScale << UL_testing_Tuning_CLK_SOURCE_LFSR_SCALE_OFFSET);
    }
#endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN) && (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2)) */


#if(((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN) && \
     (UL_testing_Tuning_DISABLE == UL_testing_Tuning_CSX_SKIP_OVSMPL_SPECIFIC_NODES) && \
     (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2) && \
     (UL_testing_Tuning_CLK_SOURCE_PRSAUTO == UL_testing_Tuning_CSX_TX_CLK_SOURCE)) ||\
    ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN) && \
     (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2) && \
     (UL_testing_Tuning_CLK_SOURCE_PRSAUTO == UL_testing_Tuning_CSD_SNS_CLK_SOURCE)))
/*******************************************************************************
* Function Name: UL_testing_Tuning_SsCalcLfsrSize
****************************************************************************//**
*
* \brief
*  This is an internal function that finds a SSC polynomial size in the Auto mode.
*
* \details
*  The SSC polynomial size in the auto mode is found based on the following
*  requirements:
*  - an LFSR value should be selected so that the max clock dither is limited with +/-10%.
*  - at least one full spread spectrum polynomial should pass during the scan time.
*  - the value of the number of conversions should be an integer multiple of the
*    repeat period of the programmed LFSR_SIZE.
*
* \param
*  snsClkDivider The divider value for the sense clock.
*  resolution The widget resolution.
*
* \return lfsrSize The LFSRSIZE value for the SENSE_PERIOD register.
*
*******************************************************************************/
static uint8 UL_testing_Tuning_SsCalcLfsrSize(uint32 snsClkDivider, uint32 conversionsNum)
{
    uint8 lfsrSize = 0u;

    /* Find LFSR value */
    if((UL_testing_Tuning_SNSCLK_SSC4_THRESHOLD <= snsClkDivider) &&
       (UL_testing_Tuning_SNSCLK_SSC4_PERIOD <= conversionsNum) &&
       (0uL == (conversionsNum % UL_testing_Tuning_SNSCLK_SSC4_PERIOD)))
    {
        lfsrSize = UL_testing_Tuning_CLK_SOURCE_SSC4;
    }
    else if((UL_testing_Tuning_SNSCLK_SSC3_THRESHOLD <= snsClkDivider) &&
            (UL_testing_Tuning_SNSCLK_SSC3_PERIOD <= conversionsNum) &&
            (0uL == (conversionsNum % UL_testing_Tuning_SNSCLK_SSC3_PERIOD)))
    {
        lfsrSize = UL_testing_Tuning_CLK_SOURCE_SSC3;
    }
    else if((UL_testing_Tuning_SNSCLK_SSC2_THRESHOLD <= snsClkDivider) &&
            (UL_testing_Tuning_SNSCLK_SSC2_PERIOD <= conversionsNum) &&
            (0uL == (conversionsNum % UL_testing_Tuning_SNSCLK_SSC2_PERIOD)))
    {
        lfsrSize = UL_testing_Tuning_CLK_SOURCE_SSC2;
    }
    else if((UL_testing_Tuning_SNSCLK_SSC1_THRESHOLD <= snsClkDivider) &&
            (UL_testing_Tuning_SNSCLK_SSC1_PERIOD <= conversionsNum) &&
            (0uL == (conversionsNum % UL_testing_Tuning_SNSCLK_SSC1_PERIOD)))
    {
        lfsrSize = UL_testing_Tuning_CLK_SOURCE_SSC1;
    }
    else
    {
        lfsrSize = (uint8)UL_testing_Tuning_CLK_SOURCE_DIRECT;
    }

    return (lfsrSize);
}


/*******************************************************************************
* Function Name: UL_testing_Tuning_SsCalcLfsrScale
****************************************************************************//**
*
* \brief
*  This is an internal function that calculates the LFSR scale value.
*
* \details
*  The LFSR scale value is used to increase the clock dither if the desired dither
*  is wider than can be achieved with the current Sense Clock Divider and current LFSR
*  period.
*
*  This returns the LFSR scale value needed to keep the clock dither in
*  range +/-10%.
*
* \param
*  snsClkDivider The divider value for the sense clock.
*  lfsrSize The period of the LFSR sequence.
*          For devices with UL_testing_Tuning_CSDV2_REF9P6UA_EN = TRUE, the
*          mode parameters can take the following values:
*          UL_testing_Tuning_CLK_SOURCE_DIRECT The spreadspectrum is not used.
*          UL_testing_Tuning_CLK_SOURCE_SSC1   The length of LFSR sequence is 63 cycles.
*          UL_testing_Tuning_CLK_SOURCE_SSC2   The length of LFSR sequence is 127 cycles.
*          UL_testing_Tuning_CLK_SOURCE_SSC3   The length of LFSR sequence is 255 cycles.
*          UL_testing_Tuning_CLK_SOURCE_SSC4   The length of LFSR sequence is 511 cycles.
*
*          For devices with UL_testing_Tuning_CSDV2_REF9P6UA_EN = TRUE, the
*          mode parameters can take the following values:
*          UL_testing_Tuning_CLK_SOURCE_DIRECT The spreadspectrum is not used.
*          UL_testing_Tuning_CLK_SOURCE_SSC1   The length of LFSR sequence is 3 cycles.
*          UL_testing_Tuning_CLK_SOURCE_SSC2   The length of LFSR sequence is 7 cycles.
*          UL_testing_Tuning_CLK_SOURCE_SSC3   The length of LFSR sequence is 15 cycles.
*          UL_testing_Tuning_CLK_SOURCE_SSC4   The length of LFSR sequence is 31 cycles.
*
* \return
*  Returns the LFSR scale value needed to keep the clock dither in range +/-10%.
*
*******************************************************************************/
static uint8 UL_testing_Tuning_SsCalcLfsrScale(uint32 snsClkDivider, uint8 lfsrSize)
{
    uint32 lfsrScale;
    uint32 lfsrRange;
    uint32 lfsrDither;

    /* Initialize the lfsrSize variable with the LFSR Range for given Lfsr Size. */
    switch(lfsrSize)
    {
        case UL_testing_Tuning_CLK_SOURCE_SSC1:
        {
            lfsrRange = UL_testing_Tuning_SNSCLK_SSC1_RANGE;
            break;
        }
        case UL_testing_Tuning_CLK_SOURCE_SSC2:
        {
            lfsrRange = UL_testing_Tuning_SNSCLK_SSC2_RANGE;
            break;
        }
        case UL_testing_Tuning_CLK_SOURCE_SSC3:
        {
            lfsrRange = UL_testing_Tuning_SNSCLK_SSC3_RANGE;
            break;
        }
        case UL_testing_Tuning_CLK_SOURCE_SSC4:
        {
            lfsrRange = UL_testing_Tuning_SNSCLK_SSC4_RANGE;
            break;
        }
        default:
        {
            lfsrRange = 0u;
            break;
        }
    }

    /* Calculate the LFSR Scale value that is required to keep the Clock dither
     * as close as possible to the +/-10% limit of the used frequency.
     */
    if((lfsrSize != UL_testing_Tuning_CLK_SOURCE_DIRECT) && (0u != lfsrRange))
    {
        /* Calculate the LFSR Dither in percents. */
        lfsrDither  = ((lfsrRange * 100uL) / snsClkDivider);
        lfsrScale = 0uL;

        while(lfsrDither < UL_testing_Tuning_LFSR_DITHER_PERCENTAGE)
        {
            lfsrScale++;
            lfsrDither <<=1uL;
        }

        if(lfsrDither > UL_testing_Tuning_LFSR_DITHER_PERCENTAGE)
        {
            lfsrScale--;
        }
    }
    else
    {
        lfsrScale = 0uL;
    }

    return ((uint8)lfsrScale);
}

#endif /* (((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN) && \
           (UL_testing_Tuning_DISABLE == UL_testing_Tuning_CSX_SKIP_OVSMPL_SPECIFIC_NODES) && \
           (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2) && \
           (UL_testing_Tuning_CLK_SOURCE_PRSAUTO == UL_testing_Tuning_CSX_TX_CLK_SOURCE)) ||\
          ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN) && \
           (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2) && \
           (UL_testing_Tuning_CLK_SOURCE_PRSAUTO == UL_testing_Tuning_CSD_SNS_CLK_SOURCE))) */


#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN)
    /*******************************************************************************
    * Function Name: UL_testing_Tuning_SsClearCSDSensors
    ****************************************************************************//**
    *
    * \brief
    *  Resets all the CSD sensors to the non-sampling state by sequentially
    *  disconnecting all the sensors from the Analog MUX bus and putting them to
    *  an inactive state.
    *
    * \details
    *  The function goes through all the widgets and updates appropriate bits in
    *  the IO HSIOM, PC and DR registers depending on the Inactive sensor connection
    *  parameter. DR register bits are set to zero when the Inactive sensor
    *  connection is Ground or Hi-Z.
    *
    *******************************************************************************/
    void UL_testing_Tuning_SsClearCSDSensors(void)
    {
        uint32 wdgtIndex;
        uint32 snsIndex;

        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_GANGED_SNS_EN)
            uint32 pinIndex;
        #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_GANGED_SNS_EN) */

        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_GANGED_SNS_EN)
            /* Declare ptr to sensor IO structure */
            UL_testing_Tuning_FLASH_IO_STRUCT const *curDedicatedSnsIOPtr;
            /* Pointer to Flash structure holding info of sensor to be scanned */
            UL_testing_Tuning_FLASH_SNS_STRUCT const *curFlashSnsPtr;
        #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_GANGED_SNS_EN) */
        UL_testing_Tuning_FLASH_IO_STRUCT const *curSnsIOPtr;

        for (wdgtIndex = 0u; wdgtIndex < UL_testing_Tuning_TOTAL_WIDGETS; wdgtIndex++)
        {
            if (UL_testing_Tuning_SENSE_METHOD_CSD_E ==
                UL_testing_Tuning_GET_SENSE_METHOD(&UL_testing_Tuning_dsFlash.wdgtArray[wdgtIndex]))
            {
                curSnsIOPtr = (UL_testing_Tuning_FLASH_IO_STRUCT const *)
                                                UL_testing_Tuning_dsFlash.wdgtArray[wdgtIndex].ptr2SnsFlash;

                /* Go through all the sensors in widget */
                for (snsIndex = 0u; snsIndex < (uint8)UL_testing_Tuning_dsFlash.wdgtArray[wdgtIndex].totalNumSns; snsIndex++)
                {
                    #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_GANGED_SNS_EN)
                        /* Check ganged sns flag */
                        if (UL_testing_Tuning_GANGED_SNS_MASK == (UL_testing_Tuning_dsFlash.wdgtArray[wdgtIndex].staticConfig &
                                                                 UL_testing_Tuning_GANGED_SNS_MASK))
                        {
                            /* Get sns pointer */
                            curFlashSnsPtr = (UL_testing_Tuning_FLASH_SNS_STRUCT const *)
                                                               UL_testing_Tuning_dsFlash.wdgtArray[wdgtIndex].ptr2SnsFlash +
                                                               snsIndex;

                            for(pinIndex = 0u; pinIndex < curFlashSnsPtr->numPins; pinIndex++)
                            {
                                /* Get IO pointer for dedicated pin */
                                curDedicatedSnsIOPtr = &UL_testing_Tuning_ioList[curFlashSnsPtr->firstPinId + pinIndex];

                                /* Disconnect dedicated pin */
                                UL_testing_Tuning_CSDDisconnectSns(curDedicatedSnsIOPtr);
                            }
                        }
                        else
                        {
                            /* Disable sensor */
                            UL_testing_Tuning_CSDDisconnectSns(&curSnsIOPtr[snsIndex]);
                        }
                    #else
                        /* Disable sensor */
                        UL_testing_Tuning_CSDDisconnectSns(&curSnsIOPtr[snsIndex]);
                    #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_GANGED_SNS_EN) */
                }
            }
        }
    }

    /*******************************************************************************
    * Function Name: UL_testing_Tuning_SsCSDGetColSnsClkDivider
    ****************************************************************************//**
    *
    * \brief
    *  This function gets the Sense Clock Divider value for one-dimension widgets
    *  and Column Sense Clock divider value for two-dimension widgets.
    *
    * \details
    *  This function gets the Sense Clock Divider value based on the Component
    *  configuration. The function is applicable for one-dimension widgets and for
    *  two-dimension widgets.
    *
    * \param widgetId
    *  Specifies the ID of the widget.
    *
    * \return
    *  Returns the Sense Clock Divider value for one-dimension widgets
    *  and the Column Sense Clock divider value for two-dimension widgets.
    *
    *******************************************************************************/
    uint32 UL_testing_Tuning_SsCSDGetColSnsClkDivider(uint32 widgetId)
    {
        uint32 retVal;

        /* Get sense divider based on configuration */
        #if (UL_testing_Tuning_ENABLE != UL_testing_Tuning_CSD_COMMON_SNS_CLK_EN)
            UL_testing_Tuning_RAM_WD_BASE_STRUCT *ptrWdgt;

            ptrWdgt = (UL_testing_Tuning_RAM_WD_BASE_STRUCT *)
            UL_testing_Tuning_dsFlash.wdgtArray[widgetId].ptr2WdgtRam;

            retVal = (uint32)(ptrWdgt->snsClk);
        #else
            retVal = (uint32)UL_testing_Tuning_dsRam.snsCsdClk;

            (void)widgetId; /* This parameter is unused in such configurations */
        #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_COMMON_SNS_CLK_EN) */

        return (retVal);
    }


    /*******************************************************************************
    * Function Name: UL_testing_Tuning_SsCSDSetColSnsClkDivider
    ****************************************************************************//**
    *
    * \brief
    *  This function sets the Sense Clock Divider value for one-dimension widgets
    *  and Column Sense Clock divider value for two-dimension widgets.
    *
    * \details
    *  This function sets the Sense Clock Divider value based on the Component
    *  configuration. The function is applicable for one-dimension widgets and for
    *  two-dimension widgets.
    *
    * \param
    *  widgetId Specifies the ID of the widget.
    *
    * \return The Sense Clock Divider value for one-dimension widgets
    *        and the Column Sense Clock divider value for two-dimension widgets.
    *
    *******************************************************************************/
    void UL_testing_Tuning_SsCSDSetColSnsClkDivider(uint32 widgetId, uint32 dividerVal)
    {
        /* Get sense divider based on configuration */
        #if (UL_testing_Tuning_ENABLE != UL_testing_Tuning_CSD_COMMON_SNS_CLK_EN)
            UL_testing_Tuning_RAM_WD_BASE_STRUCT *ptrWdgt;

            ptrWdgt = (UL_testing_Tuning_RAM_WD_BASE_STRUCT *)
            UL_testing_Tuning_dsFlash.wdgtArray[widgetId].ptr2WdgtRam;

            ptrWdgt->snsClk = (uint16)dividerVal;
        #else
            UL_testing_Tuning_dsRam.snsCsdClk = (uint16)dividerVal;

            (void)widgetId; /* This parameter is unused in such configurations */
        #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_COMMON_SNS_CLK_EN) */
    }


    #if (UL_testing_Tuning_CSD_MATRIX_WIDGET_EN || UL_testing_Tuning_CSD_TOUCHPAD_WIDGET_EN)
        /*******************************************************************************
        * Function Name: UL_testing_Tuning_SsCSDGetRowSnsClkDivider
        ****************************************************************************//**
        *
        * \brief
        *  This function gets the Sense Clock Divider value for one-dimension widgets
        *  and the Column Sense Clock divider value for two-dimension widgets.
        *
        * \details
        *  This function gets the Sense Clock Divider value based on the Component
        *  configuration. The function is applicable for one-dimension widgets and for
        *  two-dimension widgets.
        *
        * \param
        *  widgetId Specifies the ID of the widget.
        *
        * \return
        *  Returns the sense clock divider value for one-dimension widgets
        *  and column sense clock divider value for two-dimension widgets.
        *
        *******************************************************************************/
        uint32 UL_testing_Tuning_SsCSDGetRowSnsClkDivider(uint32 widgetId)
        {
            uint32 retVal;

            /* Get sense divider based on configuration */
            #if (UL_testing_Tuning_ENABLE != UL_testing_Tuning_CSD_COMMON_SNS_CLK_EN)
                UL_testing_Tuning_RAM_WD_BASE_STRUCT *ptrWdgt;

                ptrWdgt = (UL_testing_Tuning_RAM_WD_BASE_STRUCT *)
                UL_testing_Tuning_dsFlash.wdgtArray[widgetId].ptr2WdgtRam;

                retVal = ptrWdgt->rowSnsClk;
            #else
                retVal = (uint32)UL_testing_Tuning_dsRam.snsCsdClk;

                (void)widgetId; /* This parameter is unused in such configurations */
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_COMMON_SNS_CLK_EN) */

            return (retVal);
        }

    #endif /* (UL_testing_Tuning_CSD_MATRIX_WIDGET_EN || UL_testing_Tuning_CSD_TOUCHPAD_WIDGET_EN) */


    #if (UL_testing_Tuning_CLK_SOURCE_PRSAUTO == UL_testing_Tuning_CSD_SNS_CLK_SOURCE)
        /*******************************************************************************
        * Function Name: UL_testing_Tuning_SsCSDCalcPrsSize
        ****************************************************************************//**
        *
        * \brief
        *  The function finds PRS polynomial size in the Auto mode.
        *
        * \details
        *  The PRS polynomial size in the Auto mode is found based on the following
        *  requirements:
        *  - at least one full spread spectrum polynomial should pass during scan time.
        *
        * \param
        *  snsClkDivider The divider value for the sense clock.
        *  resolution The widget resolution.
        *
        * \return prsSize PRS value for SENSE_PERIOD register.
        *
        *******************************************************************************/
        uint8 UL_testing_Tuning_SsCSDCalcPrsSize(uint32 snsClkDivider, uint32 resolution)
        {
            uint32 prsSize;
            uint32 modClkDivider = 1u;

            #if ((UL_testing_Tuning_ENABLE != UL_testing_Tuning_CSDV2) && \
                 (UL_testing_Tuning_ENABLE == UL_testing_Tuning_IS_M0S8PERI_BLOCK))
                    modClkDivider = (uint32)UL_testing_Tuning_dsRam.modCsdClk;
            #endif

            if ((snsClkDivider * UL_testing_Tuning_PRS_LENGTH_12_BITS) <= (modClkDivider * ((0x00000001Lu << resolution) - 1u)))
            {
                /* Set PRS12 mode */
                prsSize = UL_testing_Tuning_PRS_12_CONFIG;
            }
            else if ((snsClkDivider * UL_testing_Tuning_PRS_LENGTH_8_BITS) <= (modClkDivider * ((0x00000001Lu << resolution) - 1u)))
            {
                /* Set PRS8 mode */
                prsSize = UL_testing_Tuning_PRS_8_CONFIG;
            }
            else
            {
                /* Set Direct clock mode */
                prsSize = UL_testing_Tuning_CLK_SOURCE_DIRECT;
            }

            return (uint8)prsSize;
        }
    #endif /* (UL_testing_Tuning_CLK_SOURCE_PRSAUTO == UL_testing_Tuning_CSD_SNS_CLK_SOURCE) */
#endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN) */


#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2)
    /*******************************************************************************
    * Function Name: UL_testing_Tuning_BistDischargeExtCapacitors
    ****************************************************************************//**
    *
    * \brief
    *  The function discharge available external capacitors.
    *
    * \details
    *  The function discharge available external capacitors by connection them
    *  to GND using STRONG GPIO drive mode. Additionaly, the function disconnects 
    *  the capacitors from analog mux buses if connected.
    *  Note: the function does not restore the connection to analog mux busses 
    *  and supposes that all the capacitors belong to a single device port.
    *
    *******************************************************************************/
    void UL_testing_Tuning_BistDischargeExtCapacitors(void)
    {
        uint32 newRegValue;
        uint8  interruptState;
        
        /* Disconnect Ext Cap from AMUXBUS-A using HSIOM registers */
        interruptState = CyEnterCriticalSection();
        newRegValue = CY_GET_REG32(UL_testing_Tuning_EXT_CAP_HSIOM_PTR);
        newRegValue &= (uint32)(~(uint32)UL_testing_Tuning_EXT_CAP_HSIOM_MASK);
        CY_SET_REG32(UL_testing_Tuning_EXT_CAP_HSIOM_PTR, newRegValue);
        CyExitCriticalSection(interruptState);

        /* Set output port register to 0 to connect to GND */
        interruptState = CyEnterCriticalSection();
        newRegValue = CY_GET_REG32(UL_testing_Tuning_EXT_CAP_DR_PTR);
        newRegValue &= (uint32)(~(uint32)UL_testing_Tuning_EXT_CAP_DR_MASK);
        CY_SET_REG32(UL_testing_Tuning_EXT_CAP_DR_PTR, newRegValue);
        CyExitCriticalSection(interruptState);

        /* Set port configuration register (drive mode) in STRONG mode */
        interruptState = CyEnterCriticalSection();
        newRegValue = CY_GET_REG32(UL_testing_Tuning_EXT_CAP_PC_PTR);
        newRegValue &= (uint32)(~((uint32)UL_testing_Tuning_EXT_CAP_PC_MASK));
        newRegValue |= (UL_testing_Tuning_EXT_CAP_PC_STRONG_CFG);
        CY_SET_REG32(UL_testing_Tuning_EXT_CAP_PC_PTR, newRegValue);
        CyExitCriticalSection(interruptState);

        /* Delay to discharge external capacitors */
        CyDelayUs(UL_testing_Tuning_EXT_CAP_DISCHARGE_TIME);

        /* Set port configuration register (drive mode) in STRONG mode */
        interruptState = CyEnterCriticalSection();
        newRegValue = CY_GET_REG32(UL_testing_Tuning_EXT_CAP_PC_PTR);
        newRegValue &= (uint32)(~((uint32)UL_testing_Tuning_EXT_CAP_PC_MASK));
        CY_SET_REG32(UL_testing_Tuning_EXT_CAP_PC_PTR, newRegValue);
        CyExitCriticalSection(interruptState);
    }
#endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2) */


/* [] END OF FILE */
