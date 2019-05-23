/***************************************************************************//**
* \file UL_testing_Tuning_SelfTest.c
* \version 6.0
*
* \brief
*   This file provides the source code for the Built-In Self-Test library.
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
#include "CyLib.h"
#include "UL_testing_Tuning_Configuration.h"
#include "UL_testing_Tuning_Structure.h"
#include "UL_testing_Tuning_Sensing.h"
#include "UL_testing_Tuning_SelfTest.h"

/***********************************************************************************************************************
* Local definition
***********************************************************************************************************************/
#define UL_testing_Tuning_TST_CRC_GROUP_EN                       (UL_testing_Tuning_TST_GLOBAL_CRC_EN || \
                                                                 UL_testing_Tuning_TST_WDGT_CRC_EN)

#define UL_testing_Tuning_TST_BASELINE_GROUP_EN                  (UL_testing_Tuning_TST_BSLN_DUPLICATION_EN || \
                                                                 UL_testing_Tuning_TST_BSLN_RAW_OUT_RANGE_EN)

#define UL_testing_Tuning_TST_HW_GROUP_EN                        (UL_testing_Tuning_TST_SHORT_GROUP_EN || \
                                                                 UL_testing_Tuning_TST_MEASUREMENT_GROUP_EN)

#define UL_testing_Tuning_TST_SHORT_GROUP_EN                     (UL_testing_Tuning_TST_SNS_SHORT_EN || \
                                                                 UL_testing_Tuning_TST_SNS2SNS_SHORT_EN)

#define UL_testing_Tuning_TST_MEASUREMENT_GROUP_EN               (UL_testing_Tuning_TST_SNS_CAP_EN || \
                                                                 UL_testing_Tuning_TST_SH_CAP_EN || \
                                                                 UL_testing_Tuning_TST_EXTERNAL_CAP_EN)

#define UL_testing_Tuning_TST_SW_WATCHDOG_CYCLES_NUM             (0x0000FFFFLu)
/***********************************************************************************************************************
* Definition of IO register modes
***********************************************************************************************************************/

/* Port Select HSIOM Register */
#define UL_testing_Tuning_TST_HSIOM_GPIO                         (0Lu)
#define UL_testing_Tuning_TST_HSIOM_CSD_SENSE                    (4Lu)
#define UL_testing_Tuning_TST_HSIOM_CSD_SHIELD                   (5Lu)
#define UL_testing_Tuning_TST_HSIOM_AMUXA                        (6Lu)
#define UL_testing_Tuning_TST_HSIOM_AMUXB                        (7Lu)

/* Port Configuration Register */
#define UL_testing_Tuning_TST_PC_HIGHZA                          (0Lu)
#define UL_testing_Tuning_TST_PC_PULLUP                          (2Lu)
#define UL_testing_Tuning_TST_PC_PULLDOWN                        (3Lu)
#define UL_testing_Tuning_TST_PC_STRONG                          (6Lu)

/* Port Data Register */
#define UL_testing_Tuning_TST_DR_PIN2GND                         (0Lu)
#define UL_testing_Tuning_TST_DR_PIN2VDD                         (1Lu)

/* Defining the drive mode of pins depending on the Inactive sensor connection setting in the Component customizer. */
#if(UL_testing_Tuning_SNS_CONNECTION_GROUND == UL_testing_Tuning_CSD_INACTIVE_SNS_CONNECTION)
    #define UL_testing_Tuning_TST_INACTIVE_SNS_GPIO_DM               (UL_testing_Tuning_TST_PC_STRONG)
#elif(UL_testing_Tuning_SNS_CONNECTION_HIGHZ == UL_testing_Tuning_CSD_INACTIVE_SNS_CONNECTION)
    #define UL_testing_Tuning_TST_INACTIVE_SNS_GPIO_DM               (UL_testing_Tuning_TST_PC_HIGHZA)
#elif(UL_testing_Tuning_SNS_CONNECTION_SHIELD == UL_testing_Tuning_CSD_INACTIVE_SNS_CONNECTION)
    #define UL_testing_Tuning_TST_INACTIVE_SNS_GPIO_DM               (UL_testing_Tuning_TST_PC_HIGHZA)
#else
    #error "Unsupported inactive connection for the inactive sensors."
#endif /* (CS_123_SNS_CONNECTION_GROUND == CS_123_CSD_INACTIVE_SNS_CONNECTION) */

/***************************************************************************//**
* \brief Defines Self-test HW Configuration
*
* \detail
* Defines the HW configuration depended on the test type. It includes the
* HW CSD block configuration, clock configuration, IDAC configuration and
* all electrode configuration.
*
*******************************************************************************/
typedef enum
{
    UL_testing_Tuning_BIST_HW_UNDEFINED_E        = 0x00u,
    UL_testing_Tuning_BIST_HW_SNS_SHORT_E        = 0x01u,
    UL_testing_Tuning_BIST_HW_S2S_SHORT_E        = 0x02u,
    UL_testing_Tuning_BIST_HW_SNS_CAP_CSD_E      = 0x03u,
    UL_testing_Tuning_BIST_HW_SNS_CAP_CSX_E      = 0x04u,
    UL_testing_Tuning_BIST_HW_SH_CAP_E           = 0x05u,
    UL_testing_Tuning_BIST_HW_EXT_CAP_E          = 0x06u,
    UL_testing_Tuning_BIST_HW_VDDA_E             = 0x07u
} UL_testing_Tuning_BIST_HW_CONFIG_ENUM;

/***************************************************************************//**
* \brief Defines Self-test IO Configuration
*
* \detail
* Defines the IO configuration depends on the test type.
*
*******************************************************************************/
typedef enum
{
    /*
    * The previous state
    */
    UL_testing_Tuning_BIST_IO_UNDEFINED_E        = 0x00u,
    /*
    * Sensor2Sensor Short
    * Sensor capacitance measurement of CSX widget
    */
    UL_testing_Tuning_BIST_IO_STRONG_E           = 0x01u,
    /*
    * Sensor Short
    * Ext capacitor measurement
    */
    UL_testing_Tuning_BIST_IO_HIGHZA_E           = 0x02u,
    /*
    * Sensor capacitance measurement of CSD widget:
    *     All CSX sensors are set to Strong
    *     All CSD sensors depends on Inactive sensor parameter and could be:
    *       High-Z Analog | Strong | CSD_Shield
    */
    UL_testing_Tuning_BIST_IO_CSD_E              = 0x03u,
    /*
    * Shield capacitance measurement
    *     All CSX sensors is set to Strong
    *     All CSD sensors depends on Inactive sensor parameter and could be:
    *     High-Z Analog | Strong | CSD_Sense
    */
    UL_testing_Tuning_BIST_IO_SHIELD_E           = 0x04u
} UL_testing_Tuning_BIST_IO_CONFIG_ENUM;

/***********************************************************************************************************************
* Local variables
***********************************************************************************************************************/
#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_HW_GROUP_EN)
static UL_testing_Tuning_BIST_HW_CONFIG_ENUM UL_testing_Tuning_bistHwConfig = UL_testing_Tuning_BIST_HW_UNDEFINED_E;
static UL_testing_Tuning_BIST_IO_CONFIG_ENUM UL_testing_Tuning_bistIoConfig = UL_testing_Tuning_BIST_IO_UNDEFINED_E;
#endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_HW_GROUP_EN) */

#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_MEASUREMENT_GROUP_EN)
    static volatile uint16 UL_testing_Tuning_bistRawcount;
    static volatile uint32 UL_testing_Tuning_bistConfigCsd = UL_testing_Tuning_BIST_CSD_CONFIG;

    #if (UL_testing_Tuning_ENABLE != UL_testing_Tuning_CSDV2)
        static volatile uint32 UL_testing_Tuning_bistCounterResolution = UL_testing_Tuning_BIST_SCAN_DURATION;
    #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2) */
#endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_MEASUREMENT_GROUP_EN) */

/***********************************************************************************************************************
* Static function prototypes
***********************************************************************************************************************/

/**

* \cond SECTION_C_INTERNAL
* \addtogroup group_c_internal
* \{
*/

/* CRC group function prototypes */
#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_WDGT_CRC_EN)
    static uint32 UL_testing_Tuning_CheckAllWidgetCRC(void);
#endif

#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_CRC_GROUP_EN)
    static uint16 UL_testing_Tuning_DsCalculateCrc16(uint8 *ptrData, uint32 len);
#endif

/* Baseline group function prototypes */
#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_BSLN_DUPLICATION_EN)
    static uint32 UL_testing_Tuning_CheckAllBaselineDuplication(void);
#endif

/* Sensor short test function prototypes */
#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS_SHORT_EN)
    static uint32 UL_testing_Tuning_CheckAllSensorShort(void);
    static void UL_testing_Tuning_UpdateTestResultSnsShort(uint32 wdgtId, uint32 snsId);
    static uint32 UL_testing_Tuning_ExecuteSnsShort(uint32 wdgtId, uint32 snsId);
    static uint32 UL_testing_Tuning_ExecuteSnsShortExt(uint32 wdgtId, uint32 ioSnsId);
#endif /*(UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS_SHORT_EN) */

/* Sensor to sensor test function prototypes */
#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS2SNS_SHORT_EN)
    static uint32 UL_testing_Tuning_CheckAllSns2SnsShort(void);
    static void UL_testing_Tuning_UpdateTestResultSns2SnsShort(uint32 wdgtId, uint32 snsId);
    static uint32 UL_testing_Tuning_ExecuteSns2SnsShort(uint32 wdgtId, uint32 snsId);
    static uint32 UL_testing_Tuning_ExecuteSns2SnsShortExt(uint32 wdgtId, uint32 ioSnsId);
#endif /*(UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS2SNS_SHORT_EN) */

/* Short group function prototypes */
#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SHORT_GROUP_EN)
    static uint32 UL_testing_Tuning_ReadElectrodeState(UL_testing_Tuning_FLASH_IO_STRUCT const *ioPtr);
#endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SHORT_GROUP_EN) */

/* Shield capacitance measurement test function prototypes */
#if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SH_CAP_EN) && \
     (0u != UL_testing_Tuning_CSD_TOTAL_SHIELD_COUNT))
    static void UL_testing_Tuning_BistConnectShieldAsSensor(void);
#endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SH_CAP_EN) && \
           (0u != UL_testing_Tuning_CSD_TOTAL_SHIELD_COUNT)) */

/* Sensor capacitance measurement test function prototypes */
#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS_CAP_EN)
    static uint32 UL_testing_Tuning_GetSensorCapacitanceExt(uint32 wdgtId, uint32 ioSnsId);
    static uint32 UL_testing_Tuning_MeasureAllSensorCapacitance(void);
    static void UL_testing_Tuning_BistConnectSensor(uint32 wdgtId, uint32 ioSnsId);
    #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN)
        #if (0u != UL_testing_Tuning_CSD_TOTAL_SHIELD_COUNT)
            static void UL_testing_Tuning_BistEnableShieldElectrodes(void);
        #endif /* (0u != UL_testing_Tuning_CSD_TOTAL_SHIELD_COUNT) */
        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_TANK_EN)
            static void UL_testing_Tuning_BistEnableShieldTank(void);
        #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_TANK_EN) */
    #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) */
#endif /*(UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS_CAP_EN) */

/* External capacitor measurement test function prototypes */
#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_EXTERNAL_CAP_EN)
    static uint32 UL_testing_Tuning_MeasureAllExtCapCapacitance(void);
    static uint32 UL_testing_Tuning_MeasureExtCapCapacitance(uint32 extCapId);
    static void UL_testing_Tuning_BistConnectExtCapacitor(uint32 extCapId);
    static void UL_testing_Tuning_BistDisconnectExtCapacitor(uint32 extCapId);
    static void UL_testing_Tuning_GetExtCapCapacitanceExt(uint32 modClkDivider, uint32 iDacVal);
#endif /*(UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_EXTERNAL_CAP_EN) */

/* Sensor and Shield capacitance measurement test function prototypes */
#if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS_CAP_EN) || \
     (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SH_CAP_EN))
    static void UL_testing_Tuning_BistDisableHwSnsCap(void);
    static uint32 UL_testing_Tuning_GetCapacitanceExt(void);
    static void UL_testing_Tuning_BistSetInactiveSensorConnection(uint32 wdgtId, uint32 ioSnsId);
    #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN)
        static void UL_testing_Tuning_BistSetAllInactiveSensorConnection(void);
    #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN) */
    #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN)
        #if (0u != UL_testing_Tuning_CSD_TOTAL_SHIELD_COUNT)
            static void UL_testing_Tuning_BistDisableShieldElectrodes(void);
        #endif /* (0u != UL_testing_Tuning_CSD_TOTAL_SHIELD_COUNT) */
    #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) */
#endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS_CAP_EN) || \
           (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SH_CAP_EN)) */

/* Measurement test group function prototypes */
#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_MEASUREMENT_GROUP_EN)
    /* Capturing/Releasing HW resources by measurement test group */
    static void UL_testing_Tuning_BistEnableHwSnsCap(void);
    /* Scanning specific function */
    static void UL_testing_Tuning_BistCmodPrecharge(void);
    static void UL_testing_Tuning_BistStartSample(void);
    static void UL_testing_Tuning_BistConfigClock(uint32 snsClk);
    static void UL_testing_Tuning_BistSetScanDuration(void);
    static void UL_testing_Tuning_BistSetIdac(uint32 idacValue);
    static void UL_testing_Tuning_BistTriggerScan(void);
    static void UL_testing_Tuning_WaitEndOfScan(uint32 timeout);
    #if (UL_testing_Tuning_DISABLE == UL_testing_Tuning_CSDV2)
        static void UL_testing_Tuning_BistClockRestart(void);
    #endif /* (UL_testing_Tuning_DISABLE == UL_testing_Tuning_CSDV2) */
#endif /*(UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_MEASUREMENT_GROUP_EN) */

#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_HW_GROUP_EN)
    /* HW Config Switching function */
    static void UL_testing_Tuning_BistSwitchHwConfig(UL_testing_Tuning_BIST_HW_CONFIG_ENUM config);
    static void UL_testing_Tuning_SwitchAllIOState(UL_testing_Tuning_BIST_IO_CONFIG_ENUM config);
    /* IOs configuration function */
    static void UL_testing_Tuning_SetAllIODrState(uint32 value);
    static void UL_testing_Tuning_SetAllIOPcState(uint32 value);
    static void UL_testing_Tuning_SetElectrodePc(UL_testing_Tuning_FLASH_IO_STRUCT const *ioPtr, uint32 value);
    static void UL_testing_Tuning_SetElectrodeDr(UL_testing_Tuning_FLASH_IO_STRUCT const *ioPtr, uint32 value);
    #if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) && \
        (UL_testing_Tuning_SNS_CONNECTION_SHIELD == UL_testing_Tuning_CSD_INACTIVE_SNS_CONNECTION))
        static void UL_testing_Tuning_SetAllIOHsiomState(uint32 value);
    #endif
#endif

#if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS_CAP_EN) || \
     (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SH_CAP_EN) || \
    ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) && \
     (UL_testing_Tuning_SNS_CONNECTION_SHIELD == UL_testing_Tuning_CSD_INACTIVE_SNS_CONNECTION)))
    static void UL_testing_Tuning_SetElectrodeHsiom(UL_testing_Tuning_FLASH_IO_STRUCT const *ioPtr, uint32 value);
#endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS_CAP_EN) || \
           (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SH_CAP_EN) || \
          ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) && \
           (UL_testing_Tuning_SNS_CONNECTION_SHIELD == UL_testing_Tuning_CSD_INACTIVE_SNS_CONNECTION))) */
#if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_VDDA_EN) &&\
     (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2))
    static void UL_testing_Tuning_BistEnableVddaMeasurement(void);
    static uint16 UL_testing_Tuning_BistVddaRunMeasurement(void);
#endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_VDDA_EN) &&\
           (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2)) */

/** \}
* \endcond */

/*******************************************************************************
* Function Name: UL_testing_Tuning_RunSelfTest
****************************************************************************//**
*
* \brief
*  Runs built-in self-tests specified by the test enable mask.
*
* \details
*  The function performs various self-tests on all the enabled widgets
*  and sensors in the Component. The required set of tests can be selected
*  using the bit-mask in testEnMask parameter.
*
*  Use UL_testing_Tuning_TST_RUN_SELF_TEST_MASK to execute all the self-tests or
*  any combination of the masks (defined in testEnMask parameter) to specify the
*  test list.
*
*  To execute a single-element test (i.e. for one widget or sensor), the following
*  functions available:
*  - UL_testing_Tuning_CheckGlobalCRC()
*  - UL_testing_Tuning_CheckWidgetCRC()
*  - UL_testing_Tuning_CheckBaselineDuplication()
*  - UL_testing_Tuning_CheckSensorShort()
*  - UL_testing_Tuning_CheckSns2SnsShort()
*  - UL_testing_Tuning_GetSensorCapacitance()
*  - UL_testing_Tuning_GetShieldCapacitance()
*  - UL_testing_Tuning_GetExtCapCapacitance()
*  - UL_testing_Tuning_GetVdda()
*
*  Refer to these functions for detail information on the corresponding test.
*
* \param testEnMask
*  Specifies the tests to be executed. Each bit corresponds to one test. It is
*  possible to launch the function with any combination of the available tests.
*  - UL_testing_Tuning_TST_GLOBAL_CRC       - Verifies the RAM structure CRC of global parameters
*  - UL_testing_Tuning_TST_WDGT_CRC         - Verifies the RAM widget structure CRC for all the widgets
*  - UL_testing_Tuning_TST_BSLN_DUPLICATION - Verifies the baseline consistency of all the sensors (inverse copy)
*  - UL_testing_Tuning_TST_SNS_SHORT        - Checks all the sensors for a short to GND or VDD
*  - UL_testing_Tuning_TST_SNS2SNS_SHORT    - Checks all the sensors for a short to other sensors
*  - UL_testing_Tuning_TST_SNS_CAP          - Measures all the sensors capacitance
*  - UL_testing_Tuning_TST_SH_CAP           - Measures the shield capacitance
*  - UL_testing_Tuning_TST_EXTERNAL_CAP     - Measures the capacitance of the available external capacitors
*  - UL_testing_Tuning_TST_VDDA             - Measures the Vdda voltage
*  - UL_testing_Tuning_TST_RUN_SELF_TEST_MASK - Executes all available tests.
*
* \return
*  Returns a bit-mask with a status of execution of the specified tests:
*  - CY_RET_SUCCESS - All the tests passed.
*  - UL_testing_Tuning_TST_NOT_EXECUTED - The previously triggered scanning is not
*    completed.
*  - UL_testing_Tuning_TST_BAD_PARAM - A non-defined test was requested in the
*    testEnMask parameter.
*  - The bit-mask of the failed tests.
*
*******************************************************************************/
uint32 UL_testing_Tuning_RunSelfTest(uint32 testEnMask)
{
    uint32 result = CYRET_SUCCESS;

    if(UL_testing_Tuning_NOT_BUSY != UL_testing_Tuning_IsBusy())
    {
        result = UL_testing_Tuning_TST_NOT_EXECUTED;
    }
    else
    {
        if (0u == (testEnMask & (~UL_testing_Tuning_TST_RUN_SELF_TEST_MASK)))
        {
            #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_GLOBAL_CRC_EN)
                if (0u != (UL_testing_Tuning_TST_GLOBAL_CRC & testEnMask))
                {
                    result |= UL_testing_Tuning_CheckGlobalCRC();
                }
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_GLOBAL_CRC_EN) */

            #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_WDGT_CRC_EN)
                if (0u != (UL_testing_Tuning_TST_WDGT_CRC & testEnMask))
                {
                    result |= UL_testing_Tuning_CheckAllWidgetCRC();
                }
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_WDGT_CRC_EN) */

            #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_BSLN_DUPLICATION_EN)
                if (0u != (UL_testing_Tuning_TST_BSLN_DUPLICATION & testEnMask))
                {
                    result |= UL_testing_Tuning_CheckAllBaselineDuplication();
                }
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_BSLN_DUPLICATION_EN) */

            /* All the next tests are hardware dependent, so requesting to switch the sense method */
            UL_testing_Tuning_SsSwitchSensingMode(UL_testing_Tuning_SENSE_METHOD_BIST_E);

            #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS_SHORT_EN)
                if (0u != (UL_testing_Tuning_TST_SNS_SHORT & testEnMask))
                {
                    result |= UL_testing_Tuning_CheckAllSensorShort();
                }
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS_SHORT_EN) */

            #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS2SNS_SHORT_EN)
                if (0u != (UL_testing_Tuning_TST_SNS2SNS_SHORT & testEnMask))
                {
                    result |= UL_testing_Tuning_CheckAllSns2SnsShort();
                }
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS2SNS_SHORT_EN) */

            #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS_CAP_EN)
                if (0u != (UL_testing_Tuning_TST_SNS_CAP & testEnMask))
                {
                    result |= UL_testing_Tuning_MeasureAllSensorCapacitance();
                }
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS_CAP_EN) */

            #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SH_CAP_EN)
                if (0u != (UL_testing_Tuning_TST_SH_CAP & testEnMask))
                {
                    (void)UL_testing_Tuning_GetShieldCapacitance();
                }
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SH_CAP_EN) */

            #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_EXTERNAL_CAP_EN)
                if (0u != (UL_testing_Tuning_TST_EXTERNAL_CAP & testEnMask))
                {
                    result |= UL_testing_Tuning_MeasureAllExtCapCapacitance();
                }
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_EXTERNAL_CAP_EN) */

            #if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_VDDA_EN) &&\
                 (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2))
                if (0u != (UL_testing_Tuning_TST_VDDA & testEnMask))
                {
                    result |= UL_testing_Tuning_GetVdda();
                }
            #endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_VDDA_EN) &&\
                       (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2)) */
        }
        else
        {
            result |= UL_testing_Tuning_TST_BAD_PARAM;
        }
    }

    return result;
}

#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_GLOBAL_CRC_EN)

/*******************************************************************************
* Function Name: UL_testing_Tuning_CheckGlobalCRC
****************************************************************************//**
*
* \brief
*  Checks the stored CRC of the UL_testing_Tuning_RAM_STRUCT data structure.
*
* \details
*  This function validates the data integrity of the
*  UL_testing_Tuning_RAM_STRUCT data structure by calculating the CRC and
*  comparing it with the stored CRC value (i.e. UL_testing_Tuning_GLB_CRC_VALUE).
*
*  If the stored and calculated CRC values differ, the calculated CRC
*  is stored to the UL_testing_Tuning_GLB_CRC_CALC_VALUE register and
*  the UL_testing_Tuning_TST_GLOBAL_CRC bit is set in the
*  UL_testing_Tuning_TEST_RESULT_MASK_VALUE register. The function never
*  clears the UL_testing_Tuning_TST_GLOBAL_CRC bit.
*
*  It is recommended to use the UL_testing_Tuning_SetParam() function to
*  change a value of UL_testing_Tuning_RAM_STRUCT data structure register/elements
*  as CRC is updated by the UL_testing_Tuning_SetParam() function.
*
*  This test also can be initiated by using UL_testing_Tuning_RunSelfTest()
*  function with the UL_testing_Tuning_TST_GLOBAL_CRC mask input.
*
* \return
*  Returns a status of the executed test:
*  - CY_RET_SUCCESS - The stored CRC matches the calculated CRC
*  - UL_testing_Tuning_TST_GLOBAL_CRC - The stored CRC is wrong.
*
*******************************************************************************/
uint32 UL_testing_Tuning_CheckGlobalCRC(void)
{
    uint32 result = UL_testing_Tuning_TST_GLOBAL_CRC;
    uint16 crcValue = UL_testing_Tuning_DsCalculateCrc16((uint8 *)&UL_testing_Tuning_dsRam +
                                    UL_testing_Tuning_GLOBAL_CRC_AREA_START,
                                    UL_testing_Tuning_GLOBAL_CRC_AREA_SIZE);
    if (UL_testing_Tuning_dsRam.glbCrc == crcValue)
    {
        result = CYRET_SUCCESS;
    }
    else
    {
        UL_testing_Tuning_dsRam.selfTest.glbCrcCalc = crcValue;
        UL_testing_Tuning_dsRam.selfTest.testResultMask |= UL_testing_Tuning_TST_GLOBAL_CRC;
    }
    return result;
}

/*******************************************************************************
* Function Name: UL_testing_Tuning_DsUpdateGlobalCrc
****************************************************************************//**
*
* \brief
*   Recalculates the CRC for the global Component data (certain registers)
*   and writes it to the Data Structure.
*
* \details
*  Uses the function UL_testing_Tuning_DsCalculateCrc16.
*
*******************************************************************************/
void UL_testing_Tuning_DsUpdateGlobalCrc(void)
{
    UL_testing_Tuning_dsRam.glbCrc = UL_testing_Tuning_DsCalculateCrc16((uint8 *)&UL_testing_Tuning_dsRam +
                                                        UL_testing_Tuning_GLOBAL_CRC_AREA_START,
                                                        UL_testing_Tuning_GLOBAL_CRC_AREA_SIZE);
}
#endif /*(UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_GLOBAL_CRC_EN) */

#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_WDGT_CRC_EN)

/*******************************************************************************
* Function Name: UL_testing_Tuning_CheckWidgetCRC
****************************************************************************//**
*
* \brief
*  Checks the stored CRC of the UL_testing_Tuning_RAM_WD_BASE_STRUCT data
*  structure of the specified widget.
*
* \details
*  This function validates the data integrity of the
*  UL_testing_Tuning_RAM_WD_BASE_STRUCT data structure of the specified widget
*  by calculating the CRC and comparing it with the stored CRC value
*  (i.e. UL_testing_Tuning_<WidgetName>_CRC_VALUE).
*
*  If the stored and calculated CRC values differ:
*  1. The calculated CRC is stored to the UL_testing_Tuning_WDGT_CRC_CALC_VALUE register
*  2. The widget ID is stored to the UL_testing_Tuning_WDGT_CRC_ID_VALUE register
*  3. The UL_testing_Tuning_TST_WDGT_CRC bit is set in the
*     UL_testing_Tuning_TEST_RESULT_MASK_VALUE register.
*
*  The function never clears the UL_testing_Tuning_TST_WDGT_CRC bit.
*  If the UL_testing_Tuning_TST_WDGT_CRC bit is set, the UL_testing_Tuning_WDGT_CRC_CALC_VALUE
*  and UL_testing_Tuning_WDGT_CRC_ID_VALUE registers are not updated.
*
*  It is recommended to use the UL_testing_Tuning_SetParam() function to change
*  a value of UL_testing_Tuning_RAM_WD_BASE_STRUCT data structure register/elements
*  as the CRC is updated by UL_testing_Tuning_SetParam() function.
*
*  This test can be initiated by UL_testing_Tuning_RunSelfTest() function with
*  the UL_testing_Tuning_TST_WDGT_CRC mask as an input.
*
*  The function updates the wdgtWorking register
*  UL_testing_Tuning_WDGT_WORKING<Number>_VALUE by clearing the widget-corresponding bit.
*  Those non-working widgets are skipped by the high-level API.
*  Restoring a widget to its working state should be done by the application level.
*
* \param widgetId
*  Specifies the ID number of the widget to be processed.
*  A macro for the widget ID can be found in the UL_testing_Tuning Configuration
*  header file defined as UL_testing_Tuning_<WidgetName>_WDGT_ID.
*
* \return
*  Returns a status of the test execution:
*  - CY_RET_SUCCESS - The stored CRC matches the calculated CRC.
*  - UL_testing_Tuning_TST_WDGT_CRC - The widget CRC is wrong.
*  - UL_testing_Tuning_TST_BAD_PARAM - The input parameter is invalid.
*
*******************************************************************************/
uint32 UL_testing_Tuning_CheckWidgetCRC(uint32 widgetId)
{
    uint32 result = UL_testing_Tuning_TST_WDGT_CRC;
    uint16 crcValue = 0u;
    UL_testing_Tuning_RAM_WD_BASE_STRUCT *ptrWdgt;

    if (UL_testing_Tuning_TOTAL_WIDGETS > widgetId)
    {
        /* Get pointer to RAM object data */
        ptrWdgt = (UL_testing_Tuning_RAM_WD_BASE_STRUCT *)UL_testing_Tuning_dsFlash.wdgtArray[widgetId].ptr2WdgtRam;
        crcValue = UL_testing_Tuning_DsCalculateCrc16((uint8 *)ptrWdgt +
                                    UL_testing_Tuning_WIDGET_CRC_AREA_START,
                                    UL_testing_Tuning_WIDGET_CRC_AREA_SIZE);
        if ((ptrWdgt->crc) != crcValue)
        {
            /* Write to self-test data structure wdgtId of the first bad tested widget */
            if (0Lu == (UL_testing_Tuning_dsRam.selfTest.testResultMask &
                       UL_testing_Tuning_TST_WDGT_CRC))
            {
                UL_testing_Tuning_dsRam.selfTest.wdgtCrcCalc = crcValue;
                UL_testing_Tuning_dsRam.selfTest.wdgtCrcId = (uint8)widgetId;
                UL_testing_Tuning_dsRam.selfTest.testResultMask |= UL_testing_Tuning_TST_WDGT_CRC;
            }
            else
            {
            }
            UL_testing_Tuning_dsRam.wdgtWorking[UL_testing_Tuning_GET_WDGT_STATUS_INDEX(widgetId)] &=
                                             ~(UL_testing_Tuning_GET_WDGT_STATUS_MASK(widgetId));
        }
        else
        {
            result = CYRET_SUCCESS;
        }
    }
    else
    {
        result = UL_testing_Tuning_TST_BAD_PARAM;
    }
    return result;
}

/*******************************************************************************
* Function Name: UL_testing_Tuning_CheckAllWidgetCRC
****************************************************************************//**
*
* \brief
*  The internal fuction that checks CRC of all widget structures.
*
* \details
*  The function calculates CRC of all widget structures and compare it
*  to the stored CRCs.
*  In case of failed comparison the function updates
*  testResultMask and returns the status.
*  The function use UL_testing_Tuning_CheckWidgetCRC.
*
* \return
*  Returns the status of the test processing:
*  - CYRET_SUCCESS if all widget CRC are OK;
*  - UL_testing_Tuning_TST_WDGT_CRC if any widget CRC is wrong.
*  - UL_testing_Tuning_TST_NOT_EXECUTED if any widget is unavailable.
*
*******************************************************************************/
static uint32 UL_testing_Tuning_CheckAllWidgetCRC(void)
{
    uint32 result = CYRET_SUCCESS;
    uint32 wdgtIndex = 0u;

    for (wdgtIndex = 0u;
         wdgtIndex < UL_testing_Tuning_TOTAL_WIDGETS;
         wdgtIndex++)
    {
        if (0u != (UL_testing_Tuning_CheckWidgetCRC(wdgtIndex)))
        {
            result |= UL_testing_Tuning_TST_WDGT_CRC;
        }
    }
    return result;
}

/*******************************************************************************
* Function Name: UL_testing_Tuning_DsUpdateWidgetCrc
****************************************************************************//**
*
* \brief
*   Recalculates the CRC for the specified widgetId and
*   writes it to the Data Structure.
*
* \details
*  The maximum size of the RAM Widget Object is 192 bits. So, the best suitable
*  CRC polynomial is 0xAC9A that has hamming distance 5 for the data up to 241 bits.
*
*  Reference:  "P. Koopman, T. Chakravarthy,
*  "Cyclic Redundancy Code (CRC) Polynomial Selection for Embedded Networks",
*  The International Conference on Dependable Systems and Networks, DSN-2004"
*
* \param widgetId  The widget index number.
*
*******************************************************************************/
void UL_testing_Tuning_DsUpdateWidgetCrc(uint32 widgetId)
{
    uint32 len;
    uint8 *ptrData;
    UL_testing_Tuning_RAM_WD_BASE_STRUCT *ptrWdgt;

    if (widgetId < UL_testing_Tuning_TOTAL_WIDGETS)
    {
        /* Get pointer to RAM object data */
        ptrWdgt = (UL_testing_Tuning_RAM_WD_BASE_STRUCT *)UL_testing_Tuning_dsFlash.wdgtArray[widgetId].ptr2WdgtRam;
        ptrData = (uint8 *)UL_testing_Tuning_dsFlash.wdgtArray[widgetId].ptr2WdgtRam;
        len = (uint32)UL_testing_Tuning_WIDGET_CRC_AREA_SIZE;

        /* Skip CRC in RAM object structure for CRC calculation */
        ptrData += UL_testing_Tuning_WIDGET_CRC_AREA_START;

        ptrWdgt->crc = UL_testing_Tuning_DsCalculateCrc16(ptrData, len);
    }
}
#endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_WDGT_CRC_EN) */

#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_CRC_GROUP_EN)

/*******************************************************************************
* Function Name: DsCalculateCrc16
****************************************************************************//**
*
* \brief
*   Calculates CRC for the specified buffer and length. CRC Poly: 0xAC9A
*
* \details
*  This API is used for the CRC protection of the RAM Widget Object.
*  The maximum size of the RAM Widget Object is 192 bits. So, the best suitable
*  CRC polynomial is 0xAC9A. It has a Hamming distance 5 for data words up to 241 bits.
*
*  Reference:  "P. Koopman, T. Chakravarthy,
*  "Cyclic Redundancy Code (CRC) Polynomial Selection for Embedded Networks",
*  The International Conference on Dependable Systems and Networks, DSN-2004"
*
* \param ptrData The pointer to the data.
* \param len     The length of the data in bytes.
*
* \return A calculated CRC-16 value.
*
*******************************************************************************/
static uint16 UL_testing_Tuning_DsCalculateCrc16(uint8 *ptrData, uint32 len)
{
    uint32 idx;
    uint32 actualCrc = 0Lu;
    const uint16 crcTable[] =
    {
        0x0000u, 0xAC9Au, 0xF5AEu, 0x5934u, 0x47C6u, 0xEB5Cu, 0xB268u, 0x1EF2u,
        0x8F8Cu, 0x2316u, 0x7A22u, 0xD6B8u, 0xC84Au, 0x64D0u, 0x3DE4u, 0x917Eu
    };

    for (;len-- > 0u;)
    {
        /* Process HI Nibble */
        idx = ((actualCrc >> 12u) ^ (((uint32)*ptrData) >> 4u)) & 0xFLu;
        actualCrc = crcTable[idx] ^ (actualCrc << 4u);

        /* Process LO Nibble */
        idx = ((actualCrc >> 12u) ^ (uint32)*ptrData) & 0xFLu;
        actualCrc = crcTable[idx] ^ (actualCrc << 4u);

        ptrData++;
    }

    return (uint16)actualCrc;
}
#endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_CRC_GROUP_EN) */

#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_BSLN_DUPLICATION_EN)
/*******************************************************************************
* Function Name: UL_testing_Tuning_CheckBaselineDuplication
****************************************************************************//**
*
* \brief
*  Checks that the baseline of the specified widget/sensor is not corrupted by
*  comparing it with a baseline inverse copy.
*
* \details
*  This function validates the integrity of baseline of sensor by comparing
*  the conformity of the baseline and its inversion.
*
*  If the baseline does not match its inverse copy:
*  1. The widget ID is stored to the UL_testing_Tuning_INV_BSLN_WDGT_ID_VALUE register
*  2. The sensor ID is stored to the UL_testing_Tuning_INV_BSLN_SNS_ID_VALUE register
*  3. The UL_testing_Tuning_TST_BSLN_DUPLICATION bit is set in the
*     UL_testing_Tuning_TEST_RESULT_MASK_VALUE register.
*
*  The function never clears the UL_testing_Tuning_TST_BSLN_DUPLICATION bit.
*  If the UL_testing_Tuning_TST_BSLN_DUPLICATION bit is set, the UL_testing_Tuning_INV_BSLN_WDGT_ID_VALUE
*  and UL_testing_Tuning_INV_BSLN_SNS_ID_VALUE registers are not updated.
*
*  It is possible to execute a test for all the widgets using UL_testing_Tuning_RunSelfTest() function
*  with the UL_testing_Tuning_TST_BSLN_DUPLICATION mask.
*  In this case, the UL_testing_Tuning_INV_BSLN_WDGT_ID_VALUE and
*  UL_testing_Tuning_INV_BSLN_SNS_ID_VALUE registers contain the widget and
*  sensor ID of the first detected fail.
*
*  The function updates the wdgtWorking register UL_testing_Tuning_WDGT_WORKING<Number>_VALUE
*  by clearing the widget-corresponding bit. Those non-working widgets are skipped by
*  the high-level API. Restoring a widget to its working state should be done by the application level.
*
*  The test is integrated into the UL_testing_Tuning Component. All UL_testing_Tuning
*  processing functions like UL_testing_Tuning_ProcessAllWidgets()
*  or UL_testing_Tuning_UpdateSensorBaseline() automatically verify the baseline
*  value before using it and update its inverse copy after processing.
*  If fail is detected during a baseline update a CYRET_BAD_DATA result
*  is returned. The baseline initialization functions do not verify the
*  baseline and update the baseline inverse copy.
*
* \param widgetId
*  Specifies the ID number of the widget to be processed.
*  A macro for the widget ID can be found in the
*  UL_testing_Tuning Configuration header file defined as
*  UL_testing_Tuning_<WidgetName>_WDGT_ID.
*
* \param sensorId
*  Specifies the ID number of the sensor within the widget.
*  A macro for the sensor ID within the specified widget can be found in
*  the UL_testing_Tuning Configuration header file defined as
*  UL_testing_Tuning_<WidgetName>_SNS<SensorNumber>_ID.
*
* \return
*  Returns the status of the test execution:
*  - CY_RET_SUCCESS - The baseline matches its inverse copy.
*  - UL_testing_Tuning_TST_BSLN_DUPLICATION - The test failed.
*  - UL_testing_Tuning_TST_BAD_PARAM - The input parameters are invalid.
*
*******************************************************************************/
uint32 UL_testing_Tuning_CheckBaselineDuplication(uint32 widgetId, uint32 sensorId)
{
    uint32 result = CYRET_SUCCESS;
    uint32 freqChannel;
    UL_testing_Tuning_RAM_SNS_STRUCT *ptrSensor = NULL;
    UL_testing_Tuning_FLASH_WD_STRUCT const *ptrWidget = NULL;

    if ((UL_testing_Tuning_TOTAL_WIDGETS > widgetId) && ((UL_testing_Tuning_GET_SENSOR_COUNT(widgetId)) > sensorId))
    {
        ptrWidget = &UL_testing_Tuning_dsFlash.wdgtArray[widgetId];
        /* Find pointer to specified sensor object */
        ptrSensor = ptrWidget->ptr2SnsRam;
        ptrSensor = &ptrSensor[sensorId];
        /* Check baseline inversion of every channel in sensor */
        for(freqChannel = 0u; freqChannel < UL_testing_Tuning_NUM_SCAN_FREQS; freqChannel++)
        {
            if (ptrSensor->bsln[freqChannel] != ((uint16) ~(ptrSensor->bslnInv[freqChannel])))
            {
                UL_testing_Tuning_UpdateTestResultBaselineDuplication(widgetId, sensorId);
                result |= UL_testing_Tuning_TST_BSLN_DUPLICATION;
            }
        }
    }
    else
    {
        result |= UL_testing_Tuning_TST_BAD_PARAM;
    }
    return result;
}

/*******************************************************************************
* Function Name: UL_testing_Tuning_CheckAllBaselineDuplication
****************************************************************************//**
*
* \brief
*  The internal fuction that checks baseline duplication of all the sensors.
*
* \details
*  The function checks baseline duplication of all the sensors by using
*  UL_testing_Tuning_CheckBaselineDuplication() function.
*
* \return
*  Returns the status of the test processing:
*    - CYRET_SUCCESS if the test is successfully completed;
*    - UL_testing_Tuning_TST_BSLN_DUPLICATION if any sensor baseline duplication
       is wrong;
*******************************************************************************/
static uint32 UL_testing_Tuning_CheckAllBaselineDuplication(void)
{
    uint32 result = CYRET_SUCCESS;
    uint32 wdgtIndex;
    uint32 snsIndex;

    for (wdgtIndex = 0u; wdgtIndex < UL_testing_Tuning_TOTAL_WIDGETS; wdgtIndex++)
    {
        for (snsIndex = 0u; snsIndex < (UL_testing_Tuning_GET_SENSOR_COUNT(wdgtIndex)); snsIndex++)
        {
            if (CYRET_SUCCESS != UL_testing_Tuning_CheckBaselineDuplication(wdgtIndex, snsIndex))
            {
                result |= UL_testing_Tuning_TST_BSLN_DUPLICATION;
                break;
            }
        }
    }
    return result;
}

/*******************************************************************************
* Function Name: UL_testing_Tuning_UpdateTestResultBaselineDuplication
****************************************************************************//**
*
* \brief
*  The internal fuction that updates SefTest data structure and
*  widgetWorking register.
*
* \details
*  The function resets a corresponding bit in widgetWorking register,
*  checks TestResultMask for UL_testing_Tuning_TST_BSLN_DUPLICATION
*  bit and if it was not setted to 1 sets it and memorizes wdgtId and snsId
*  in registers invBslnWdgtId and invBslnSnsId of the SefTest data structure.
*
* \param widgetId
*  Specifies the ID number of the widget to be processed.
*
* \param sensorId
*  Specifies the ID number of the sensor within the widget which
*  will be processed.
*
*******************************************************************************/
void UL_testing_Tuning_UpdateTestResultBaselineDuplication(uint32 widgetId, uint32 sensorId)
{
    UL_testing_Tuning_dsRam.wdgtWorking[UL_testing_Tuning_GET_WDGT_STATUS_INDEX(widgetId)] &=
                                     ~(UL_testing_Tuning_GET_WDGT_STATUS_MASK(widgetId));
    /* Write to self-test data structure wdgtId and snsId of the first sensor with a damaged baseline inversion */
    if (0Lu == (UL_testing_Tuning_dsRam.selfTest.testResultMask &
                UL_testing_Tuning_TST_BSLN_DUPLICATION))
    {
        UL_testing_Tuning_dsRam.selfTest.testResultMask |= UL_testing_Tuning_TST_BSLN_DUPLICATION;
        UL_testing_Tuning_dsRam.selfTest.invBslnWdgtId = (uint8)widgetId;
        UL_testing_Tuning_dsRam.selfTest.invBslnSnsId = (uint8)sensorId;
    }
}
#endif /*(UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_BSLN_DUPLICATION_EN)*/

#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_BSLN_RAW_OUT_RANGE_EN)

/*******************************************************************************
* Function Name: UL_testing_Tuning_CheckBaselineRawcountRange
****************************************************************************//**
*
* \brief
*  Checks that raw count and baseline of the specified widget/sensor are
*  within the specified range.
*
* \details
*  The baseline and raw count shall be within specific range (based on
*  calibration target) for good units. The function checks whether or not
*  the baseline and raw count are within the limits defined by the user in the
*  ranges function argument. If baseline or raw count are out of limits this
*  function sets the UL_testing_Tuning_TST_BSLN_RAW_OUT_RANGE bit in the
*  UL_testing_Tuning_TEST_RESULT_MASK_VALUE register.
*
*  Unlike other tests, this test does not update
*  UL_testing_Tuning_WDGT_WORKING<Number>_VALUE register and is not available in
*  the UL_testing_Tuning_RunSelfTest() function.
*
*  Use this function to verify the uniformity of sensors, for example, at
*  mass-production or during an operation phase.
*
* \param widgetId
*  Specifies the ID number of the widget.
*  A macro for the widget ID can be found in the
*  UL_testing_Tuning Configuration header file defined as
*  UL_testing_Tuning_<WidgetName>_WDGT_ID.
*
* \param sensorId
*  Specifies the ID number of the sensor within the widget.
*  A macro for the sensor ID within the specified widget can be found in
*  the UL_testing_Tuning Configuration header file defined as
*  UL_testing_Tuning_<WidgetName>_SNS<SensorNumber>_ID.
*
* \param *ranges
*  Specifies the pointer to the UL_testing_Tuning_BSLN_RAW_RANGE_STRUCT structure
*  with valid ranges for the raw count and baseline.
*
* \return
*  Returns a status of the test execution:
*  - CY_RET_SUCCESS - The raw count and baseline are within the specified range
*  - UL_testing_Tuning_TST_BSLN_RAW_OUT_RANGE - The test failed and baseline or
     raw count or both are out of the specified limit.
*  - UL_testing_Tuning_TST_BAD_PARAM - The input parameters are invalid.
*
*******************************************************************************/
uint32 UL_testing_Tuning_CheckBaselineRawcountRange(uint32 widgetId, uint32 sensorId,UL_testing_Tuning_BSLN_RAW_RANGE_STRUCT *ranges)
{
    uint32 result = CYRET_SUCCESS;
    uint32 freqChannel;
    UL_testing_Tuning_RAM_SNS_STRUCT *ptrSensor = NULL;
    UL_testing_Tuning_FLASH_WD_STRUCT const *ptrWidget = NULL;


    if ((UL_testing_Tuning_TOTAL_WIDGETS > widgetId) &&
       ((UL_testing_Tuning_GET_SENSOR_COUNT(widgetId)) > sensorId))
    {
        ptrWidget = &UL_testing_Tuning_dsFlash.wdgtArray[widgetId];
        /* Find pointer to specified sensor object */
        ptrSensor = ptrWidget->ptr2SnsRam;
        ptrSensor = &ptrSensor[sensorId];

        #if(UL_testing_Tuning_ENABLE == UL_testing_Tuning_MULTI_FREQ_SCAN_EN)
        /* Check baseline and rawcont of every channel in sensor */
            for(freqChannel = 0u;
                freqChannel < UL_testing_Tuning_NUM_SCAN_FREQS;
                freqChannel++)
            {
        #else
            freqChannel = 0u;
        #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_MULTI_FREQ_SCAN_EN) */

                if ((ptrSensor->bsln[freqChannel] > ranges->bslnHiLim) ||
                    (ptrSensor->bsln[freqChannel] < ranges->bslnLoLim) ||
                    (ptrSensor->raw[freqChannel]  > ranges->rawHiLim)  ||
                    (ptrSensor->raw[freqChannel]  < ranges->rawLoLim))
                {
                    UL_testing_Tuning_dsRam.selfTest.testResultMask |= UL_testing_Tuning_TST_BSLN_RAW_OUT_RANGE;
                    result = UL_testing_Tuning_TST_BSLN_RAW_OUT_RANGE;
                }

        #if(UL_testing_Tuning_ENABLE == UL_testing_Tuning_MULTI_FREQ_SCAN_EN)
            }
        #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_MULTI_FREQ_SCAN_EN) */

    }
    else
    {
        result |= UL_testing_Tuning_TST_BAD_PARAM;
    }
    return result;
}
#endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_BSLN_RAW_OUT_RANGE_EN) */

#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS_SHORT_EN)

/*******************************************************************************
* Function Name: UL_testing_Tuning_CheckSensorShort
****************************************************************************//**
*
* \brief
*  Checks the specified widget/sensor for shorts to GND or VDD.
*
* \details
*  The function performs test to check for a short to GND or VDD on
*  the specified sensor. The resistance of electrical short must be
*  less than 1100 Ohm including series resistors on sensor for the short to
*  be detected. The UL_testing_Tuning_GetSensorCapacitance() function can be
*  used to check an electrical short with resistance higher than 1100 Ohm or
*  when the specified ganged sensor consists of two or more electrodes.
*
*  This function performs the following tasks:
*  - If a short is detected, the widget ID is stored to the
*    UL_testing_Tuning_SHORTED_WDGT_ID_VALUE register.
*  - If a short is detected, the sensor ID is stored to the
*    UL_testing_Tuning_SHORTED_SNS_ID_VALUE register.
*  - If a short is detected, the UL_testing_Tuning_TST_SNS_SHORT bit is
*    set in the UL_testing_Tuning_TEST_RESULT_MASK_VALUE register.
*  - If a short is detected, the bit corresponding to the specified widget
*    (UL_testing_Tuning_WDGT_WORKING<Number>_VALUE) is cleared in the
*    wdgtWorking register to indicate fault with the widget. Once the bit
*    is cleared, the widget is treated as non-working widget by the
*    high-level functions and further processing are skipped. Restoring
*    the bit corresponding the widget can be done by application layer
*    to restore the operation of the high-level functions.
*  - If UL_testing_Tuning_TST_SNS_SHORT is already set due to previously
*    detected fault on any of the sensor, the UL_testing_Tuning_TST_SNS_SHORT
*    register is not cleared by this function and
*    UL_testing_Tuning_SHORTED_WDGT_ID_VALUE and
*    UL_testing_Tuning_SHORTED_SNS_ID_VALUE registers are not updated.
*    For this reason, remember to read details of defective sensor and clear
*    UL_testing_Tuning_TST_SNS_SHORT prior to calling this function on the
*    same or different sensor.
*
*  This function performs the test on one specific sensor. The
*  UL_testing_Tuning_RunSelfTest() function with the
*  UL_testing_Tuning_TST_SNS_SHORT mask, performs the short test on all
*  the widgets and sensors in the Component. In this case,
*  UL_testing_Tuning_SHORTED_WDGT_ID_VALUE and
*  UL_testing_Tuning_SHORTED_SNS_ID_VALUE registers stores the widget and sensor
*  ID of the first faulty sensor.
*
* \param widgetId
*  Specifies the ID number of the widget to be processed.
*  A macro for the widget ID can be found in the
*  UL_testing_Tuning Configuration header file defined as
*  UL_testing_Tuning_<WidgetName>_WDGT_ID.
*
* \param sensorId
*  Specifies the ID number of the sensor within the widget.
*  A macro for the sensor ID within the specified widget can be found in
*  the UL_testing_Tuning Configuration header file defined as
*  UL_testing_Tuning_<WidgetName>_SNS<SensorNumber>_ID.
*
* \return
*  Returns a status of the test execution:
*  - CY_RET_SUCCESS - The sensor of the widget does not have a short to VDD or
*    GND and is in working condition.
*  - UL_testing_Tuning_TST_SNS_SHORT - A short is detected on the specified sensor.
*  - UL_testing_Tuning_TST_BAD_PARAM - The input parameters are invalid.
*
*******************************************************************************/
uint32 UL_testing_Tuning_CheckSensorShort(uint32 widgetId, uint32 sensorId)
{
    uint32 result = CYRET_SUCCESS;

    if ((UL_testing_Tuning_TOTAL_WIDGETS > widgetId) && ((UL_testing_Tuning_GET_SENSOR_COUNT(widgetId)) > sensorId))
    {
        /* Release previously captured HW resources by other mode and capture them for BIST */
        UL_testing_Tuning_SsSwitchSensingMode(UL_testing_Tuning_SENSE_METHOD_BIST_E);
        /* Switch HW resource configuration to sensor short test */
        UL_testing_Tuning_BistSwitchHwConfig(UL_testing_Tuning_BIST_HW_SNS_SHORT_E);
        /* Execute the test */
        result = UL_testing_Tuning_ExecuteSnsShort(widgetId, sensorId);
    }
    else
    {
        result = UL_testing_Tuning_TST_BAD_PARAM;
    }
    return result;
}

/*******************************************************************************
* Function Name: UL_testing_Tuning_CheckAllSensorShort
****************************************************************************//**
*
* \brief
*  The internal fuction that checks all the sensors short to GND or VDD.
*
* \details
*  The fuction that checks for shorts on Vdd/GND of all sensor (not
*  electrode) by using UL_testing_Tuning_ExecuteSnsShort. The function returns
*  the status and updates testResultMask and  wdgtWorking register
*  in a case of any shorts detection.
*
* \return
*  Returns the status of the test processing:
*  - CYRET_SUCCESS if test passed successfully;
*  - UL_testing_Tuning_TST_SNS_SHORT if any sensor of any widget is
*    shorted to Vdd or GND.
*
*******************************************************************************/
static uint32 UL_testing_Tuning_CheckAllSensorShort(void)
{
    uint32 wdgtIndex;
    uint32 snsIndex;
    uint32 result = CYRET_SUCCESS;
    uint32 resultWidget = CYRET_SUCCESS;

    /* Releasing of previously captured HW resources by other mode was done in RunSelfTest function */
    /* Switch HW resource configuration to sensor short test */
    UL_testing_Tuning_BistSwitchHwConfig(UL_testing_Tuning_BIST_HW_SNS_SHORT_E);
    /* Execute the test */
    for (wdgtIndex = 0u; wdgtIndex < UL_testing_Tuning_TOTAL_WIDGETS; wdgtIndex++)
    {
        resultWidget = CYRET_SUCCESS;
        for (snsIndex = 0u; snsIndex < (UL_testing_Tuning_GET_SENSOR_COUNT(wdgtIndex)); snsIndex++)
        {
            resultWidget = UL_testing_Tuning_ExecuteSnsShort(wdgtIndex, snsIndex);
            if (CYRET_SUCCESS != resultWidget)
            {
                break;
            }
        }
        result |= resultWidget;
    }
    return result;
}

/*******************************************************************************
* Function Name: UL_testing_Tuning_UpdateTestResultSnsShort
****************************************************************************//**
* \brief
*  The internal fuction that updates SefTest data structure and
*  widgetWorking register.
*
* \details
*  The function resets a corresponding bit in widgetWorking register,
*  checks TestResultMask for UL_testing_Tuning_TST_SNS_SHORT bit and if
*  it was not setted to 1 sets it and memorizes wdgtId and snsId
*  in corresponding registers of the SefTest data structure.
*
* \param wdgtId
*  Specifies the ID number of the widget to be processed.
*
* \param snsId
*  Specifies the ID number of the sensor within the widget which
*  will be processed.
*******************************************************************************/
static void UL_testing_Tuning_UpdateTestResultSnsShort(uint32 wdgtId, uint32 snsId)
{
    UL_testing_Tuning_dsRam.wdgtWorking[UL_testing_Tuning_GET_WDGT_STATUS_INDEX(wdgtId)] &=
                                     ~(UL_testing_Tuning_GET_WDGT_STATUS_MASK(wdgtId));
    if (0Lu == (UL_testing_Tuning_dsRam.selfTest.testResultMask & UL_testing_Tuning_TST_SNS_SHORT))
    {
        /* Write to self-test data structure wdgtId and snsId of the first shorted sensor */
        UL_testing_Tuning_dsRam.selfTest.testResultMask |= UL_testing_Tuning_TST_SNS_SHORT;
        UL_testing_Tuning_dsRam.selfTest.shortedWdgtId = (uint8)wdgtId;
        UL_testing_Tuning_dsRam.selfTest.shortedSnsId = (uint8)snsId;
    }
}

/*******************************************************************************
* Function Name: UL_testing_Tuning_ExecuteSnsShort
****************************************************************************//**
*
* \brief
*  The internal fuction that checks one sensor for shorts on GND/VDD.
*
* \details
*  The function that checks for shorts on GND/VDD of a certain sensor for CSD
*  widgets or of Rx and Tx electrodes of a certain sensor for CSX widgets.
*  The function uses UL_testing_Tuning_ExecuteSnsShortExt.
*  In a case of a shorts detection the function returns failed result.
*
* \param wdgtId
*  Specifies the ID number of the widget to be processed.
*
* \param snsId
*  Specifies the ID number of the sensor within the widget to be processed.
*
* \return
*  Returns the status of the test processing:
*  - CYRET_SUCCESS if any sensor's electrode of the widget isn't shorted to VDD or GND;
*  - UL_testing_Tuning_TST_SNS_SHORT if the sensor is shorted to VDD or GND;
*
*******************************************************************************/
static uint32 UL_testing_Tuning_ExecuteSnsShort(uint32 wdgtId, uint32 snsId)
{
    uint32 ioSnsId;
    uint32 result = CYRET_SUCCESS;

    #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_CSX_EN)
        /* For CSX widgets get an index of Rx electrode, for CSD widgets an electrode index is the same as snsId */
        if (UL_testing_Tuning_SENSE_METHOD_CSX_E == UL_testing_Tuning_GET_SENSE_METHOD(&UL_testing_Tuning_dsFlash.wdgtArray[wdgtId]))
        {
            ioSnsId = snsId / UL_testing_Tuning_dsFlash.wdgtArray[wdgtId].numRows;
        }
        else
        {
            ioSnsId = snsId;
        }
    #elif (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN)
        ioSnsId = snsId / UL_testing_Tuning_dsFlash.wdgtArray[wdgtId].numRows;
    #else
        ioSnsId = snsId;
    #endif

    result = UL_testing_Tuning_ExecuteSnsShortExt(wdgtId, ioSnsId);

    /* For CSX widgets, get an index of a Tx electrode and repeat the test */
    #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN)
        if (CYRET_SUCCESS == result)
        {
            if (UL_testing_Tuning_SENSE_METHOD_CSX_E == UL_testing_Tuning_GET_SENSE_METHOD(&UL_testing_Tuning_dsFlash.wdgtArray[wdgtId]))
            {
                ioSnsId = (uint32)(snsId % UL_testing_Tuning_dsFlash.wdgtArray[wdgtId].numRows) +
                    (uint32)UL_testing_Tuning_dsFlash.wdgtArray[wdgtId].numCols;
                result = UL_testing_Tuning_ExecuteSnsShortExt(wdgtId, ioSnsId);
            }
        }
    #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN) */

    if (CYRET_SUCCESS != result)
    {
        UL_testing_Tuning_UpdateTestResultSnsShort(wdgtId, snsId);
    }
    return result;
}

/*******************************************************************************
* Function Name: UL_testing_Tuning_ExecuteSnsShortExt
****************************************************************************//**
*
* \brief
*  The internal fuction that checks one sensor for shorts on GND/VDD.
*
* \details
*  The function that checks for shorts on GND/VDD of a certain sensor (electrode)
*  for CSD widgets or of Rx or Tx electrode of a certain sensor for CSX widgets.
*  In a case of a shorts detection the function returns failed result.
*  The function proceeds ganged sensors (electrodes).
*
* \param wdgtId
*  Specifies the ID number of the widget to be processed.
*
* \param snsId
*  Specifies the ID number of the sensor (electrode) within the widget to be processed.
*
* \return
*  Returns the status of the test processing:
*  - CYRET_SUCCESS if the electrode of the widget isn't shorted to VDD or GND;
*  - UL_testing_Tuning_TST_SNS_SHORT if the sensor or Rx or Tx electrode
*    is shorted to VDD or GND.
*
*******************************************************************************/
static uint32 UL_testing_Tuning_ExecuteSnsShortExt(uint32 wdgtId, uint32 ioSnsId)
{
    uint32 eltdNum;
    uint32 eltdIndex;
    uint32 result = CYRET_SUCCESS;
    UL_testing_Tuning_FLASH_IO_STRUCT const *ioPtr;
    UL_testing_Tuning_FLASH_SNS_STRUCT const *curFlashSnsPtr;

    /* Check ganged sns flag */
    if (UL_testing_Tuning_GANGED_SNS_MASK == (UL_testing_Tuning_dsFlash.wdgtArray[wdgtId].staticConfig & UL_testing_Tuning_GANGED_SNS_MASK))
    {
        curFlashSnsPtr = (UL_testing_Tuning_FLASH_SNS_STRUCT const *)UL_testing_Tuning_dsFlash.wdgtArray[wdgtId].ptr2SnsFlash;
        curFlashSnsPtr += ioSnsId;
        ioPtr = &UL_testing_Tuning_ioList[curFlashSnsPtr->firstPinId];
        eltdNum = curFlashSnsPtr->numPins;
    }
    else
    {
        ioPtr = (UL_testing_Tuning_FLASH_IO_STRUCT const *)UL_testing_Tuning_dsFlash.wdgtArray[wdgtId].ptr2SnsFlash + ioSnsId;
        eltdNum = 1u;
    }

    /* Loop through all electrodes of specified sensor */
    for (eltdIndex = 0u; eltdIndex < eltdNum; eltdIndex++)
    {
        /* Set electrode to pull up drive mode and check that electrode state is logical 1 */
        UL_testing_Tuning_SetElectrodeDr(ioPtr, UL_testing_Tuning_TST_DR_PIN2VDD);
        UL_testing_Tuning_SetElectrodePc(ioPtr, UL_testing_Tuning_TST_PC_PULLUP);
        if (0uL == UL_testing_Tuning_ReadElectrodeState(ioPtr))
        {
            result = UL_testing_Tuning_TST_SNS_SHORT;
        }
        /* Set electrode to pull down drive mode and check that electrode state is logical 0 */
        UL_testing_Tuning_SetElectrodeDr(ioPtr, UL_testing_Tuning_TST_DR_PIN2GND);
        UL_testing_Tuning_SetElectrodePc(ioPtr, UL_testing_Tuning_TST_PC_PULLDOWN);
        if (0uL != UL_testing_Tuning_ReadElectrodeState(ioPtr))
        {
            result = UL_testing_Tuning_TST_SNS_SHORT;
        }
        /* Revert electrode to default drive mode */
        UL_testing_Tuning_SetElectrodePc(ioPtr, UL_testing_Tuning_TST_PC_HIGHZA);

        if (UL_testing_Tuning_TST_SNS_SHORT == result)
        {
            break;
        }

        /* Get the next electrode */
        ioPtr++;
    }
    return result;
}
#endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS_SHORT_EN) */

#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS2SNS_SHORT_EN)
/*******************************************************************************
* Function Name: UL_testing_Tuning_CheckSns2SnsShort
****************************************************************************//**
*
* \brief
*  Checks the specified widget/sensor for shorts to any other UL_testing_Tuning
*  sensors.
*
* \details
*  The function performs the test on the specified sensor to check for a short
*  to other sensors in the Component. The resistance of an electrical short
*  must be less than 1100 Ohm including the series resistors on the sensor for
*  the short to be detected. The UL_testing_Tuning_GetSensorCapacitance()
*  function can be used to check the electrical short with the resistance
*  higher than 1100 Ohm or when the specified ganged sensor consists of two or
*  more electrodes.
*
*  This function performs the following tasks:
*  - If a short is detected, widget ID is stored to the
*    UL_testing_Tuning_P2P_WDGT_ID_VALUE register.
*  - If a short is detected, sensor ID is stored to the
*    UL_testing_Tuning_P2P_SNS_ID_VALUE register.
*  - If a short is detected, UL_testing_Tuning_TST_SNS2SNS_SHORT bit is set
*    in the UL_testing_Tuning_TEST_RESULT_MASK_VALUE register.
*  - If a short is detected, the bit corresponding to widget
*    (UL_testing_Tuning_WDGT_WORKING<Number>_VALUE) is cleared in the
*    wdgtWorking register to indicate fault with widget. Once the bit is
*    cleared, the widget is treated as a non-working widget by the high-level
*    functions and further processing are skipped. Restoring the bit
*    corresponding to the widget can be done by application layer to restore
*    the operation of high-level functions.
*  - If UL_testing_Tuning_TST_SNS2SNS_SHORT is already set due to the
*    previously detected fault on any of the sensor, the
*    UL_testing_Tuning_TST_SNS2SNS_SHORT register is not cleared by this
*    function and UL_testing_Tuning_P2P_WDGT_ID_VALUE and
*    UL_testing_Tuning_P2P_SNS_ID_VALUE registers are not updated.
*    For this reason, remember to read details of defective sensor and clear
*    UL_testing_Tuning_TST_SNS2SNS_SHORT prior to calling this function on the
*    same or different sensor.
*
*  This function performs the test on one specific sensor.
*  The UL_testing_Tuning_RunSelfTest() function with the
*  UL_testing_Tuning_TST_SNS2SNS_SHORT mask performs the short test on all the
*  widgets and sensors in the Component. In this case,
*  UL_testing_Tuning_P2P_WDGT_ID_VALUE and
*  UL_testing_Tuning_P2P_SNS_ID_VALUE registers store the widget and sensor ID
*  of the first faulty sensor.
*
* \param widgetId
*  Specifies the ID number of the widget to be processed.
*  A macro for the widget ID can be found in the
*  UL_testing_Tuning Configuration header file defined as
*  UL_testing_Tuning_<WidgetName>_WDGT_ID.
*
* \param sensorId
*  Specifies the ID number of the sensor within the widget.
*  A macro for the sensor ID within the specified widget can be found in
*  the UL_testing_Tuning Configuration header file defined as
*  UL_testing_Tuning_<WidgetName>_SNS<SensorNumber>_ID.
*
* \return
*  Returns a status of the test execution:
*  - CY_RET_SUCCESS - The sensor is not shorted to any other sensor is in
*    working condition.
*  - UL_testing_Tuning_TST_SNS2SNS_SHORT - A short is detected with one or
*    more sensors in the Component.
*  - UL_testing_Tuning_TST_BAD_PARAM - The input parameters are invalid.
*
*******************************************************************************/
uint32 UL_testing_Tuning_CheckSns2SnsShort(uint32 widgetId, uint32 sensorId)
{
    uint32 result = CYRET_SUCCESS;

    if ((UL_testing_Tuning_TOTAL_WIDGETS > widgetId) && ((UL_testing_Tuning_GET_SENSOR_COUNT(widgetId)) > sensorId))
    {
        /* Release previously captured HW resources by other mode and capture them for BIST */
        UL_testing_Tuning_SsSwitchSensingMode(UL_testing_Tuning_SENSE_METHOD_BIST_E);
        /* Switch HW resource configuration to sensor to sensor short test */
        UL_testing_Tuning_BistSwitchHwConfig(UL_testing_Tuning_BIST_HW_S2S_SHORT_E);
        /* Execute the test */
        result = UL_testing_Tuning_ExecuteSns2SnsShort(widgetId, sensorId);
    }
    else
    {
        result |= UL_testing_Tuning_TST_BAD_PARAM;
    }
    return result;
}

/*******************************************************************************
* Function Name: UL_testing_Tuning_CheckAllSns2SnsShort
****************************************************************************//**
*
* \brief
*  The internal fuction that checks for shorts between all of
*  UL_testing_Tuning sensors.
*
* \details
*  The function checks for all sensor shorts between them by using
*  UL_testing_Tuning_ExecuteSns2SnsShort.
*  The function returns the status and updates testResultMask and
*  wdgtWorking register in a case of any shorts detection.
*
* \return
*  Returns the status of the test processing:
*  - CYRET_SUCCESS - All the sensors of all the widgets aren't shorted to
*    any other sensors.
*  - UL_testing_Tuning_TST_SNS_SHORT - Any sensor of any widget is
*    shorted on any other sensor.
*
*******************************************************************************/
static uint32 UL_testing_Tuning_CheckAllSns2SnsShort(void)
{
    uint32 wdgtIndex;
    uint32 snsIndex;
    uint32 result = CYRET_SUCCESS;
    uint32 resultWidget = CYRET_SUCCESS;

    /* Releasing of previously captured HW resources by other mode was done in RunSelfTest function */
    /* Switch HW resource configuration to sensor to sensor to sensor short test */
    UL_testing_Tuning_BistSwitchHwConfig(UL_testing_Tuning_BIST_HW_S2S_SHORT_E);

    /* Execute the test */
    for (wdgtIndex = 0uL; wdgtIndex < UL_testing_Tuning_TOTAL_WIDGETS; wdgtIndex++)
    {
        resultWidget = CYRET_SUCCESS;
        for (snsIndex = 0u; snsIndex < (UL_testing_Tuning_GET_SENSOR_COUNT(wdgtIndex)); snsIndex++)
        {
            resultWidget = UL_testing_Tuning_ExecuteSns2SnsShort(wdgtIndex, snsIndex);
            if (CYRET_SUCCESS != resultWidget)
            {
                break;
            }
        }
        result |= resultWidget;
    }
    return result;
}

/*******************************************************************************
* Function Name: UL_testing_Tuning_UpdateTestResultSns2SnsShort
****************************************************************************//**
* \brief
*  The internal fuction that updates self-test data structure and
*  widgetWorking register.
*
* \details
*  The function resets a corresponding bit in widgetWorking register,
*  checks TestResultMask for UL_testing_Tuning_TST_SNS2SNS_SHORT bit and
*  if it was not setted to 1 sets it and memorizes wdgtId and snsId
*  in corresponding registers of the SefTest data structure.
*
* \param wdgtId
*  Specifies the ID number of the widget to be processed.
*
* \param snsId
*  Specifies the ID number of the sensor within the widget which
*  will be processed.
*******************************************************************************/
static void UL_testing_Tuning_UpdateTestResultSns2SnsShort(uint32 wdgtId, uint32 snsId)
{
    UL_testing_Tuning_dsRam.wdgtWorking[UL_testing_Tuning_GET_WDGT_STATUS_INDEX(wdgtId)] &=
                                     ~(UL_testing_Tuning_GET_WDGT_STATUS_MASK(wdgtId));
    /* Write to self-test data structure wdgtId and snsId of the first shorted sensor2sensor */
    if (0uL == (UL_testing_Tuning_dsRam.selfTest.testResultMask & UL_testing_Tuning_TST_SNS2SNS_SHORT))
    {
        UL_testing_Tuning_dsRam.selfTest.testResultMask |= UL_testing_Tuning_TST_SNS2SNS_SHORT;
        UL_testing_Tuning_dsRam.selfTest.p2pWdgtId = (uint8)wdgtId;
        UL_testing_Tuning_dsRam.selfTest.p2pSnsId = (uint8)snsId;
    }
}

/*******************************************************************************
* Function Name: UL_testing_Tuning_ExecuteSns2SnsShort
****************************************************************************//**
*
* \brief
*  The internal fuction that checks one sensor for shorts to another sensor.
*
* \details
*  The function that checks for shorts to another sensor of a certain sensor
*  for CSD widgets or Rx and Tx electrodes of a certain sensor for CSX widgets.
*  The function uses UL_testing_Tuning_ExecuteSnsShortExt.
*  In a case of a shorts detection the function returns failed result.
*
* \param wdgtId
*  Specifies the ID number of the widget to be processed.
*
* \param snsId
*  Specifies the ID number of the sensor within the widget to be processed.
*
* \return
*  Returns the status of the test processing:
*  - CYRET_SUCCESS if the electrode of the widget isn't shorted to another sensor;
*  - UL_testing_Tuning_TST_SNS2SNS_SHORT if the sensor is shorted to another sensor.
*
*******************************************************************************/
static uint32 UL_testing_Tuning_ExecuteSns2SnsShort(uint32 wdgtId, uint32 snsId)
{
    uint32 ioSnsId;
    uint32 result = CYRET_SUCCESS;

    #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_CSX_EN)
        /* For CSX widgets get an index of Rx electrode, for CSD widgets an electrode index is the same as snsId */
        if (UL_testing_Tuning_SENSE_METHOD_CSX_E == UL_testing_Tuning_GET_SENSE_METHOD(&UL_testing_Tuning_dsFlash.wdgtArray[wdgtId]))
        {
            ioSnsId = snsId / UL_testing_Tuning_dsFlash.wdgtArray[wdgtId].numRows;
        }
        else
        {
            ioSnsId = snsId;
        }
    #elif (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN)
        ioSnsId = snsId / UL_testing_Tuning_dsFlash.wdgtArray[wdgtId].numRows;
    #else
        ioSnsId = snsId;
    #endif

    result = UL_testing_Tuning_ExecuteSns2SnsShortExt(wdgtId, ioSnsId);

    /* For CSX widgets get an index of Tx electrode and repeat the test */
    #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN)
        if (CYRET_SUCCESS == result)
        {
            if (UL_testing_Tuning_SENSE_METHOD_CSX_E == UL_testing_Tuning_GET_SENSE_METHOD(&UL_testing_Tuning_dsFlash.wdgtArray[wdgtId]))
            {
                ioSnsId = (uint32)(snsId % UL_testing_Tuning_dsFlash.wdgtArray[wdgtId].numRows) +
                    (uint32)UL_testing_Tuning_dsFlash.wdgtArray[wdgtId].numCols;
                result = UL_testing_Tuning_ExecuteSns2SnsShortExt(wdgtId, ioSnsId);
            }
        }
    #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN) */

    if (CYRET_SUCCESS != result)
    {
        UL_testing_Tuning_UpdateTestResultSns2SnsShort(wdgtId, snsId);
    }
    return result;
}

/*******************************************************************************
* Function Name: UL_testing_Tuning_ExecuteSns2SnsShortExt
****************************************************************************//**
*
* \brief
*  The internal fuction that checks one sensor for shorts to another sensor.
*
* \details
*  The function that checks for shorts to another sensor of a certain sensor(electrode)
*  for CSD widgets or of Rx or Tx electrode of a certain sensor for CSX widgets.
*  In a case of a shorts detection the function returns failed result.
*  The function proceeds ganged sensors(electrodes).
*
* \param wdgtId
*  Specifies the ID number of the widget to be processed.
*
* \param snsId
*  Specifies the ID number of the sensor within the widget to be processed.
*
* \return
*  Returns the status of the test processing:
*  - CYRET_SUCCESS if the electrode of the widget isn't shorted to another sensor;
*  - UL_testing_Tuning_TST_SNS2SNS_SHORT if the sensor or Rx or Tx electrode
*    is shorted to another sensor.
*
*******************************************************************************/
static uint32 UL_testing_Tuning_ExecuteSns2SnsShortExt(uint32 wdgtId, uint32 ioSnsId)
{
    uint32 eltdNum;
    uint32 eltdIndex;
    uint32 result = CYRET_SUCCESS;
    UL_testing_Tuning_FLASH_IO_STRUCT const *ioPtr;
    UL_testing_Tuning_FLASH_SNS_STRUCT const *curFlashSnsPtr;

    /* Check ganged sns flag */
    if (UL_testing_Tuning_GANGED_SNS_MASK == (UL_testing_Tuning_dsFlash.wdgtArray[wdgtId].staticConfig & UL_testing_Tuning_GANGED_SNS_MASK))
    {
        curFlashSnsPtr = (UL_testing_Tuning_FLASH_SNS_STRUCT const *)UL_testing_Tuning_dsFlash.wdgtArray[wdgtId].ptr2SnsFlash;
        curFlashSnsPtr += ioSnsId;
        ioPtr = &UL_testing_Tuning_ioList[curFlashSnsPtr->firstPinId];
        eltdNum = curFlashSnsPtr->numPins;
    }
    else
    {
        ioPtr = (UL_testing_Tuning_FLASH_IO_STRUCT const *)UL_testing_Tuning_dsFlash.wdgtArray[wdgtId].ptr2SnsFlash + ioSnsId;
        eltdNum = 1uL;
    }

    /* Loop through all electrodes of specified sensor and set pull up drive mode */
    for (eltdIndex = 0uL; eltdIndex < eltdNum; eltdIndex++)
    {
        /* Set electrode to pull up drive mode and check that electrode state is logical 1 */
        UL_testing_Tuning_SetElectrodeDr(ioPtr + eltdIndex, UL_testing_Tuning_TST_DR_PIN2VDD);
        UL_testing_Tuning_SetElectrodePc(ioPtr + eltdIndex, UL_testing_Tuning_TST_PC_PULLUP);

        if (0uL == UL_testing_Tuning_ReadElectrodeState(ioPtr + eltdIndex))
        {
            result = UL_testing_Tuning_TST_SNS2SNS_SHORT;
            break;
        }

        /* Revert electrode to default drive mode */
        UL_testing_Tuning_SetElectrodeDr(ioPtr + eltdIndex, UL_testing_Tuning_TST_DR_PIN2GND);
        UL_testing_Tuning_SetElectrodePc(ioPtr + eltdIndex, UL_testing_Tuning_TST_PC_STRONG);
    }

    return(result);
}
#endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS2SNS_SHORT_EN) */

#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SHORT_GROUP_EN)

/*******************************************************************************
* Function Name: UL_testing_Tuning_ReadElectrodeState
****************************************************************************//**
*
* \brief
*  The internal function that reds a certain electrode state.
*
* \details
*  The function activates an input buffer for the specified electrode and reads
*  the correspondent input bit. After that it disables the input buffer.
*
* \param *ioPtr
*  A pointer to IO data structure of the specified electrode.
*
* \return
*  Returns the state of the input buffer of the electrode (0 or non 0).
*
*******************************************************************************/
static uint32 UL_testing_Tuning_ReadElectrodeState(UL_testing_Tuning_FLASH_IO_STRUCT const *ioPtr)
{
    uint32 newRegisterValue;
    uint8  interruptState;
    uint32 result;

    /* Enable the input buffer for the specified electrode */
    interruptState = CyEnterCriticalSection();
    newRegisterValue = CY_GET_REG32(ioPtr->pc2Ptr);
    newRegisterValue &= ~(ioPtr->mask);
    CY_SET_REG32(ioPtr->pc2Ptr, newRegisterValue);
    CyExitCriticalSection(interruptState);

    /* Read the bit from the IO pad state register (PS) */
    result = CY_GET_REG32(ioPtr->psPtr) & ioPtr->mask;

    /* Disable the input buffer for this pin (set PC2 bit to logic 1) */
    interruptState = CyEnterCriticalSection();
    newRegisterValue = CY_GET_REG32(ioPtr->pc2Ptr);
    newRegisterValue |= ioPtr->mask;
    CY_SET_REG32(ioPtr->pc2Ptr, newRegisterValue);
    CyExitCriticalSection(interruptState);

    return(result);
}
#endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SHORT_GROUP_EN) */

#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_EXTERNAL_CAP_EN)
/*******************************************************************************
* Function Name: UL_testing_Tuning_GetExtCapCapacitance
****************************************************************************//**
*
* \brief
*  Measures the capacitance of the specified external capacitor.
*
* \details
*  The function measures the capacitance of the specified external capacitor
*  such as Cmod and returns the result, alternatively the result is stored in
*  the UL_testing_Tuning_EXT_CAP<EXT_CAP_ID>_VALUE register in data structure.
*
*  The measurable capacitance range using this function is
*  from 200pF to 60,000pF with measurement accuracy of 10%.
*
*  This test can be executed for all the external capacitors at once using
*  the UL_testing_Tuning_RunSelfTest() function with the
*  UL_testing_Tuning_TST_EXTERNAL_CAP mask.
*
* \param extCapId
*  Specifies the ID number of the external capacitor to be measured:
*  - UL_testing_Tuning_TST_CMOD_ID - Cmod capacitor
*  - UL_testing_Tuning_TST_CSH_ID - Csh capacitor
*  - UL_testing_Tuning_TST_CINTA_ID - CintA capacitor
*  - UL_testing_Tuning_TST_CINTB_ID - CintB capacitor
*
* \return
*  Returns a status of the test execution:
*  - The capacitance (in pF) of the specified external capacitor
*  - UL_testing_Tuning_TST_BAD_PARAM if the input parameter is invalid.
*
*******************************************************************************/
uint32 UL_testing_Tuning_GetExtCapCapacitance(uint32 extCapId)
{
    uint32 result = CYRET_SUCCESS;

    if (UL_testing_Tuning_TST_EXT_CAPS_NUM > extCapId)
    {
        UL_testing_Tuning_SsSwitchSensingMode(UL_testing_Tuning_SENSE_METHOD_BIST_E);
        UL_testing_Tuning_BistSwitchHwConfig(UL_testing_Tuning_BIST_HW_EXT_CAP_E);
        UL_testing_Tuning_BistConfigClock((uint32)UL_testing_Tuning_TST_EXT_CAP_RESOLUTION);
        /* Set resolution to fixed value */
        UL_testing_Tuning_BistSetScanDuration();
        result = UL_testing_Tuning_MeasureExtCapCapacitance(extCapId);
    }
    else
    {
        result |= UL_testing_Tuning_TST_BAD_PARAM;
    }
    return result;
}

/*******************************************************************************
* Function Name: UL_testing_Tuning_MeasureExtCapCapacitance
****************************************************************************//**
*
* \brief
*  The internal function that measures a capacitance of an external capacitor
*  when HW block is configured for an external capasitors measuring.
*
* \details
*  The function connect an external capacitor (Cmod, Csh, CintA/CintB)
*  to CSD block, defines a modclock frequency divider and an Idac code
*  and measures a capacitance in range of 0,5 to 5,0 nF.
*  If the result exceeds 75 % of a range high bound the function divides modClk,
*  multiply Idac code to the fixed value to wide a measurement range and performs
*  another measuring cycle for a capacitance in range of 5,0 to 60,0 nF.
*  After that the fuction disconnect the external capacitor, calculate a result
*  and stores it to a correspondent register of the self-test data structure.
*
* \param extCapId
*  An index of the external cap to measure.
*
* \return
*  - Cext of the external capacitor in pF units.
*
*******************************************************************************/
static uint32 UL_testing_Tuning_MeasureExtCapCapacitance(uint32 extCapId)
{
    uint32 modClkDivider;
    uint32 iDacVal;
    uint32 result = 0uL;

    /* Connect an extCap */
    UL_testing_Tuning_BistConnectExtCapacitor(extCapId);
     /* Set modClk and Idac to the fixed value for a fine scan */
    #if (CYDEV_BCLK__HFCLK__MHZ < UL_testing_Tuning_TST_EXT_CAP_MODCLK_MHZ)
        modClkDivider = 1uL;
    #else
        modClkDivider = CYDEV_BCLK__HFCLK__MHZ / UL_testing_Tuning_TST_EXT_CAP_MODCLK_MHZ;
    #endif /* (CYDEV_BCLK__HFCLK__MHZ < UL_testing_Tuning_TST_EXT_CAP_MODCLK_MHZ) */

    iDacVal = (((UL_testing_Tuning_TST_EXT_CAP_LOW_RANGE * UL_testing_Tuning_CSD_VREF_MV * CYDEV_BCLK__HFCLK__MHZ) /
              UL_testing_Tuning_TST_SNS_CAP_IDAC_GAIN) / modClkDivider) + 1uL;
    UL_testing_Tuning_GetExtCapCapacitanceExt(modClkDivider, iDacVal);

    /* For results more then 75 % of the resolution limit perform a second wide range scan */
    if (UL_testing_Tuning_TST_EXT_CAP_RESOLUTION_75 < UL_testing_Tuning_bistRawcount)
    {
         /* Divide modClk and Multiply Idac to the fixed value to wide a measurement range */
        modClkDivider <<= 2u;

        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2)
            iDacVal <<= 2u;
        #else /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2) */
            iDacVal = UL_testing_Tuning_BIST_MAX_IDAC_VALUE;
        #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2) */

        UL_testing_Tuning_GetExtCapCapacitanceExt(modClkDivider, iDacVal);
    }
    /* Disconnect ext capacitor */
    UL_testing_Tuning_BistDisconnectExtCapacitor(extCapId);
    /* Calculate result in 2 steps to prevent overflow */
    result = (iDacVal * modClkDivider * UL_testing_Tuning_bistRawcount) / CYDEV_BCLK__HFCLK__MHZ;
    result = (result * UL_testing_Tuning_TST_SNS_CAP_IDAC_GAIN) / UL_testing_Tuning_CSD_VREF_MV;
    UL_testing_Tuning_dsRam.selfTest.extCap[extCapId] = (uint16)result;

    return result;
}

/*******************************************************************************
* Function Name: UL_testing_Tuning_GetExtCapCapacitanceExt
****************************************************************************//**
*
* \brief
*  The internal function that performs a scan for an external capacitor measuring.
*
* \details
*  The function sets the modclock and Idac to defined values and performs a scan
*  by using CSD mode for an external capacitor (Cmod, Csh, or CintA/CintB).
*
* \param modClkDivider
*  Specifies a divider for a modulation clock frequency.
*
* \param iDacVal
*  Specifies an Idac modulation code.
*
*******************************************************************************/
static void UL_testing_Tuning_GetExtCapCapacitanceExt(uint32 modClkDivider, uint32 iDacVal)
{
    #if ((UL_testing_Tuning_ENABLE != UL_testing_Tuning_CSDV2) &&\
         (UL_testing_Tuning_ENABLE == UL_testing_Tuning_IS_M0S8PERI_BLOCK))
        UL_testing_Tuning_BistConfigClock(UL_testing_Tuning_TST_EXT_CAP_RESOLUTION * modClkDivider);
    #endif /* ((UL_testing_Tuning_ENABLE != UL_testing_Tuning_CSDV2) &&\
               (UL_testing_Tuning_ENABLE == UL_testing_Tuning_IS_M0S8PERI_BLOCK)) */

    UL_testing_Tuning_SsSetModClkClockDivider(modClkDivider);
    UL_testing_Tuning_BistSetIdac(iDacVal);
    /* Start the scan */
    UL_testing_Tuning_dsRam.status |= UL_testing_Tuning_SW_STS_BUSY;
    UL_testing_Tuning_BistStartSample();
    /* Initialize Watchdog Counter with time interval which is enough to charge 100 nF capacitor */
    UL_testing_Tuning_WaitEndOfScan(UL_testing_Tuning_BIST_MEASURE_WATCHDOG_CYCLES_NUM);
}

/*******************************************************************************
* Function Name: UL_testing_Tuning_BistConnectExtCapacitor
****************************************************************************//**
*
* \brief
*  The internal function that connects an external capacitor to the
*  UL_testing_Tuning block.
*
* \details
*  The function that that connects an external capacitor to the UL_testing_Tuning
*  block by using HSIOM in CSD mode.
*
* \param extCapId
*  An index of the external cap to connect.
*
*******************************************************************************/
static void UL_testing_Tuning_BistConnectExtCapacitor(uint32 extCapId)
{
    uint32 newRegValue;
    uint8  interruptState;

    #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2)
        /* Connect AMUXBUS-A to CSDCOMP(sense path) */
    CY_SET_REG32(UL_testing_Tuning_SW_CMP_P_SEL_PTR, UL_testing_Tuning_SW_CMP_P_SEL_SW_SFMA_STATIC_CLOSE);
    #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2) */

    switch(UL_testing_Tuning_extCapMap[extCapId])
    {
        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN)
            case UL_testing_Tuning_TST_CMOD_MAP:
            {
                /* Connect Cmod pin using HSIOM registers */
                interruptState = CyEnterCriticalSection();
                newRegValue = CY_GET_REG32(UL_testing_Tuning_CMOD_HSIOM_PTR);
                newRegValue &= (uint32)(~(uint32)UL_testing_Tuning_CMOD_HSIOM_MASK);
                newRegValue |= (UL_testing_Tuning_HSIOM_SEL_CSD_SENSE << UL_testing_Tuning_CMOD_HSIOM_SHIFT);
                CY_SET_REG32(UL_testing_Tuning_CMOD_HSIOM_PTR, newRegValue);
                CyExitCriticalSection(interruptState);
                break;
            }
                #if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) && \
                     (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_TANK_EN))
                    case UL_testing_Tuning_TST_CSH_MAP:
                    {
                        /* Connect CTANK to AMUXBUS-A */
                        interruptState = CyEnterCriticalSection();
                        newRegValue = CY_GET_REG32(UL_testing_Tuning_CSH_HSIOM_PTR);
                        newRegValue &= (uint32)(~(uint32)UL_testing_Tuning_CSH_HSIOM_MASK);
                        newRegValue |= (UL_testing_Tuning_HSIOM_SEL_CSD_SENSE << UL_testing_Tuning_CSH_HSIOM_SHIFT);
                        CY_SET_REG32(UL_testing_Tuning_CSH_HSIOM_PTR, newRegValue);
                        CyExitCriticalSection(interruptState);
                        break;
                    }
                #endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) && \
                      (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_TANK_EN)) */

        #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN) */

        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN)
            case UL_testing_Tuning_TST_CINTA_MAP:
            {
                /* Connect the CintA pin using HSIOM registers */
                interruptState = CyEnterCriticalSection();
                newRegValue = CY_GET_REG32(UL_testing_Tuning_CintA_HSIOM_PTR);
                newRegValue &= ~UL_testing_Tuning_CintA_HSIOM_MASK;
                newRegValue |= (UL_testing_Tuning_HSIOM_SEL_CSD_SENSE << UL_testing_Tuning_CintA_HSIOM_SHIFT);
                CY_SET_REG32(UL_testing_Tuning_CintA_HSIOM_PTR, newRegValue);
                CyExitCriticalSection(interruptState);
                break;
            }
            case UL_testing_Tuning_TST_CINTB_MAP:
            {
                /* Connect CintB to AMUXBUS-A using HSIOM registers */
                interruptState = CyEnterCriticalSection();
                newRegValue = CY_GET_REG32(UL_testing_Tuning_CintB_HSIOM_PTR);
                newRegValue &= ~UL_testing_Tuning_CintB_HSIOM_MASK;
                newRegValue |= (UL_testing_Tuning_HSIOM_SEL_CSD_SENSE << UL_testing_Tuning_CintB_HSIOM_SHIFT);
                CY_SET_REG32(UL_testing_Tuning_CintB_HSIOM_PTR, newRegValue);
                CyExitCriticalSection(interruptState);
                break;
            }
        #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN) */

        default:
        {
            break;
        }
    }
}

/*******************************************************************************
* Function Name: UL_testing_Tuning_BistDisconnectExtCapacitor
****************************************************************************//**
*
* \brief
*  The internal function that disconnects an external capacitor from the
*  UL_testing_Tuning block.
*
* \details
*  The function that that disconnects an external capacitor from the
*  UL_testing_Tuning block.
*  After that the function dicharges the specified capacitor to GND to provide
*  next regular scans.
*
* \param extCapId
*  An index of the external cap to disconnect.
*
*******************************************************************************/
static void UL_testing_Tuning_BistDisconnectExtCapacitor(uint32 extCapId)
{
    uint32 newRegValue;
    uint8  interruptState;

    #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2)
        /* Disconnect AMUXBUS-A from CSDCOMP(sense path) */
        CY_SET_REG32(UL_testing_Tuning_SW_CMP_P_SEL_PTR, UL_testing_Tuning_SW_CMP_P_SEL_SW_SFMA_STATIC_OPEN);
    #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2) */

    switch(UL_testing_Tuning_extCapMap[extCapId])
    {
        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN)
            case UL_testing_Tuning_TST_CMOD_MAP:
            {
                /* Disconnect Cmod from AMUXBUS-A using HSIOM registers */
                interruptState = CyEnterCriticalSection();
                newRegValue = CY_GET_REG32(UL_testing_Tuning_CMOD_HSIOM_PTR);
                newRegValue &= (uint32)(~(uint32)UL_testing_Tuning_CMOD_HSIOM_MASK);
                CY_SET_REG32(UL_testing_Tuning_CMOD_HSIOM_PTR, newRegValue);
                CyExitCriticalSection(interruptState);

                #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2)
                    /* Disconnect CSDCMP from CSDBUS-A (parallel pass via SFCA switch) */
                    newRegValue = CY_GET_REG32(UL_testing_Tuning_SW_CMP_P_SEL_PTR);
                    newRegValue &= (uint32) (~UL_testing_Tuning_SW_CMP_P_SEL_SW_SFCA_MASK);
                    CY_SET_REG32(UL_testing_Tuning_SW_CMP_P_SEL_PTR, newRegValue);
                #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2) */

                /* Discharge CMOD by using GPIO mode */
                interruptState = CyEnterCriticalSection();
                newRegValue = CY_GET_REG32(UL_testing_Tuning_CMOD_HSIOM_PTR);
                newRegValue &= (uint32)(~(uint32)UL_testing_Tuning_CMOD_HSIOM_MASK);
                newRegValue |= (UL_testing_Tuning_HSIOM_SEL_GPIO << UL_testing_Tuning_CMOD_HSIOM_SHIFT);
                CY_SET_REG32(UL_testing_Tuning_CMOD_HSIOM_PTR, newRegValue);
                CyExitCriticalSection(interruptState);

                /* Set output port register for Cmod to 0 to connect Cmod to GND */
                interruptState = CyEnterCriticalSection();
                newRegValue = CY_GET_REG32(UL_testing_Tuning_CMOD_DR_PTR);
                newRegValue &= (uint32)(~(uint32) (1uL << UL_testing_Tuning_CMOD_DR_SHIFT));
                CY_SET_REG32(UL_testing_Tuning_CMOD_DR_PTR, newRegValue);
                CyExitCriticalSection(interruptState);

                /* Set port configuration register (drive mode) for Cmod in STRONG mode */
                interruptState = CyEnterCriticalSection();
                newRegValue = CY_GET_REG32(UL_testing_Tuning_CMOD_PC_PTR);
                newRegValue &= (uint32)(~((uint32)UL_testing_Tuning_GPIO_PC_MASK << UL_testing_Tuning_CMOD_PC_SHIFT));
                newRegValue |= (UL_testing_Tuning_GPIO_STRGDRV << UL_testing_Tuning_CMOD_PC_SHIFT);
                CY_SET_REG32(UL_testing_Tuning_CMOD_PC_PTR, newRegValue);
                CyExitCriticalSection(interruptState);

                /* Delay 10 uS to discharge Cmod */
                CyDelayUs(10u);

                /* Set port configuration register (drive mode) for Cmod in HIGHZ-A mode (PC=000) */
                interruptState = CyEnterCriticalSection();
                newRegValue = CY_GET_REG32(UL_testing_Tuning_CMOD_PC_PTR);
                newRegValue &= (uint32)(~((uint32)UL_testing_Tuning_GPIO_PC_MASK << UL_testing_Tuning_CMOD_PC_SHIFT));
                CY_SET_REG32(UL_testing_Tuning_CMOD_PC_PTR, newRegValue);
                CyExitCriticalSection(interruptState);

                break;
            }

            #if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) && \
                 (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_TANK_EN))
                case UL_testing_Tuning_TST_CSH_MAP:
                {
                    /* Disconnect CTANK from AMUXBUS-A */
                    interruptState = CyEnterCriticalSection();
                    newRegValue = CY_GET_REG32(UL_testing_Tuning_CSH_HSIOM_PTR);
                    newRegValue &= (uint32)(~(uint32)UL_testing_Tuning_CSH_HSIOM_MASK);
                    CY_SET_REG32(UL_testing_Tuning_CSH_HSIOM_PTR, newRegValue);
                    CyExitCriticalSection(interruptState);

                    /* Discharge CTANK by using GPIO mode */
                    interruptState = CyEnterCriticalSection();
                    newRegValue = CY_GET_REG32(UL_testing_Tuning_CSH_HSIOM_PTR);
                    newRegValue &= (uint32)(~(uint32)UL_testing_Tuning_CSH_HSIOM_MASK);
                    newRegValue |= (UL_testing_Tuning_HSIOM_SEL_GPIO << UL_testing_Tuning_CSH_HSIOM_SHIFT);
                    CY_SET_REG32(UL_testing_Tuning_CSH_HSIOM_PTR, newRegValue);
                    CyExitCriticalSection(interruptState);

                    /* Set output port register for Cmod to 0 to connect Cmod to GND */
                    interruptState = CyEnterCriticalSection();
                    newRegValue = CY_GET_REG32(UL_testing_Tuning_CSH_DR_PTR);
                    newRegValue &= (uint32)(~(uint32) (1uL << UL_testing_Tuning_CSH_SHIFT));
                    CY_SET_REG32(UL_testing_Tuning_CSH_DR_PTR, newRegValue);
                    CyExitCriticalSection(interruptState);

                    /* Set port configuration register (drive mode) for CTANK in STRONG mode */
                    interruptState = CyEnterCriticalSection();
                    newRegValue = CY_GET_REG32(UL_testing_Tuning_CSH_PC_PTR);
                    newRegValue &= (uint32)(~((uint32)UL_testing_Tuning_GPIO_PC_MASK << UL_testing_Tuning_CSH_PC_SHIFT));
                    newRegValue |= (UL_testing_Tuning_GPIO_STRGDRV << UL_testing_Tuning_CSH_PC_SHIFT);
                    CY_SET_REG32(UL_testing_Tuning_CSH_PC_PTR, newRegValue);
                    CyExitCriticalSection(interruptState);

                    /* Delay 10 uS to discharge CTANK */
                    CyDelayUs(10u);

                    /* Set port configuration register (drive mode) for CTANK in HIGHZ-A mode */
                    interruptState = CyEnterCriticalSection();
                    newRegValue = CY_GET_REG32(UL_testing_Tuning_CSH_PC_PTR);
                    newRegValue &= (uint32)(~((uint32)UL_testing_Tuning_GPIO_PC_MASK << UL_testing_Tuning_CSH_PC_SHIFT));
                    CY_SET_REG32(UL_testing_Tuning_CSH_PC_PTR, newRegValue);
                    CyExitCriticalSection(interruptState);

                    break;
                }
            #endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) && \
                       (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_TANK_EN)) */

        #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN) */

        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN)
            case UL_testing_Tuning_TST_CINTA_MAP:
            {
                /* Disconnect CintA from AMUXBUS-A using HSIOM registers. */
                interruptState = CyEnterCriticalSection();
                newRegValue = CY_GET_REG32(UL_testing_Tuning_CintA_HSIOM_PTR);
                newRegValue &= (uint32)(~(uint32)UL_testing_Tuning_CintA_HSIOM_MASK);
                CY_SET_REG32(UL_testing_Tuning_CintA_HSIOM_PTR, newRegValue);
                CyExitCriticalSection(interruptState);

                /* Discharge CintA by using GPIO mode */
                interruptState = CyEnterCriticalSection();
                newRegValue = CY_GET_REG32(UL_testing_Tuning_CintA_HSIOM_PTR);
                newRegValue &= (uint32)(~(uint32)UL_testing_Tuning_CintA_HSIOM_MASK);
                newRegValue |= (UL_testing_Tuning_HSIOM_SEL_GPIO << UL_testing_Tuning_CintA_HSIOM_SHIFT);
                CY_SET_REG32(UL_testing_Tuning_CintA_HSIOM_PTR, newRegValue);
                CyExitCriticalSection(interruptState);

                /* Set output port register for CintA to 0 to connect CintA to GND */
                interruptState = CyEnterCriticalSection();
                newRegValue = CY_GET_REG32(UL_testing_Tuning_CintA_DR_PTR);
                newRegValue &= (uint32)(~(uint32) (1uL << UL_testing_Tuning_CintA_SHIFT));
                CY_SET_REG32(UL_testing_Tuning_CintA_DR_PTR, newRegValue);
                CyExitCriticalSection(interruptState);

                /* Set port configuration register (drive mode) for CintA in STRONG mode */
                interruptState = CyEnterCriticalSection();
                newRegValue = CY_GET_REG32(UL_testing_Tuning_CintA_PC_PTR);
                newRegValue &= (uint32)(~((uint32)UL_testing_Tuning_GPIO_PC_MASK << (UL_testing_Tuning_CintA_SHIFT * 3uL)));
                newRegValue |= (UL_testing_Tuning_GPIO_STRGDRV << (UL_testing_Tuning_CintA_SHIFT * 3uL));
                CY_SET_REG32(UL_testing_Tuning_CintA_PC_PTR, newRegValue);
                CyExitCriticalSection(interruptState);

                /* Delay 10 uS to discharge CintA */
                CyDelayUs(10u);

                /* Set port configuration register (drive mode) for CintA in HIGHZ-A mode (PC=000) */
                interruptState = CyEnterCriticalSection();
                newRegValue = CY_GET_REG32(UL_testing_Tuning_CintA_PC_PTR);
                newRegValue &= (uint32)(~((uint32)UL_testing_Tuning_GPIO_PC_MASK << (UL_testing_Tuning_CintA_SHIFT * 3uL)));
                CY_SET_REG32(UL_testing_Tuning_CintA_PC_PTR, newRegValue);
                CyExitCriticalSection(interruptState);

                break;
            }
            case UL_testing_Tuning_TST_CINTB_MAP:
            {
                /* Discharge CintB by using GPIO mode */
                interruptState = CyEnterCriticalSection();
                newRegValue = CY_GET_REG32(UL_testing_Tuning_CintB_HSIOM_PTR);
                newRegValue &= (uint32)(~(uint32)UL_testing_Tuning_CintB_HSIOM_MASK);
                CY_SET_REG32(UL_testing_Tuning_CintB_HSIOM_PTR, newRegValue);
                CyExitCriticalSection(interruptState);

                /* Set a CintB HSIOM register to the GPIO mode */
                interruptState = CyEnterCriticalSection();
                newRegValue = CY_GET_REG32(UL_testing_Tuning_CintB_HSIOM_PTR);
                newRegValue &= (uint32)(~(uint32)UL_testing_Tuning_CintB_HSIOM_MASK);
                newRegValue |= (UL_testing_Tuning_HSIOM_SEL_GPIO << UL_testing_Tuning_CintB_HSIOM_SHIFT);
                CY_SET_REG32(UL_testing_Tuning_CintB_HSIOM_PTR, newRegValue);
                CyExitCriticalSection(interruptState);

                /* Set output port register for CintB to 0 to connect CintB to GND */
                interruptState = CyEnterCriticalSection();
                newRegValue = CY_GET_REG32(UL_testing_Tuning_CintB_DR_PTR);
                newRegValue &= (uint32)(~(uint32) (1uL << UL_testing_Tuning_CintB_SHIFT));
                CY_SET_REG32(UL_testing_Tuning_CintB_DR_PTR, newRegValue);
                CyExitCriticalSection(interruptState);

                /* Set port configuration register (drive mode) for CintB in STRONG mode */
                interruptState = CyEnterCriticalSection();
                newRegValue = CY_GET_REG32(UL_testing_Tuning_CintB_PC_PTR);
                newRegValue &= (uint32)(~((uint32)UL_testing_Tuning_GPIO_PC_MASK << (UL_testing_Tuning_CintB_SHIFT * 3uL)));
                newRegValue |= (UL_testing_Tuning_GPIO_STRGDRV << (UL_testing_Tuning_CintB_SHIFT * 3uL));
                CY_SET_REG32(UL_testing_Tuning_CintB_PC_PTR, newRegValue);
                CyExitCriticalSection(interruptState);

                /* Delay 10 uS to discharge CintB */
                CyDelayUs(10u);

                /* Set port configuration register (drive mode) for CintB in HIGHZ-A mode */
                interruptState = CyEnterCriticalSection();
                newRegValue = CY_GET_REG32(UL_testing_Tuning_CintB_PC_PTR);
                newRegValue &= (uint32)(~((uint32)UL_testing_Tuning_GPIO_PC_MASK << (UL_testing_Tuning_CintB_SHIFT * 3uL)));
                CY_SET_REG32(UL_testing_Tuning_CintB_PC_PTR, newRegValue);
                CyExitCriticalSection(interruptState);

                break;
            }
        #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN) */

        default:
        {
            break;
        }
    }
}

/*******************************************************************************
* Function Name: UL_testing_Tuning_MeasureAllExtCapCapacitance
****************************************************************************//**
*
* \brief
*  The internal function that measures a capacitance of all external
*  capacitors.
*
* \details
*  The function measures capacitances of all external  capacitor (Cmod,
*  Csh, CintA, CintB).
*  The function stores cap values in corresponding registers.
*  The function does not affect on another Component parameters or
*  functionalities.
*
* \return
*  Returns the status of the measuring process:
*  - CYRET_SUCCESS if all measurings is successfull;
*  - UL_testing_Tuning_TST_EXTERNAL_CAP if any measuring is failed.
*
*******************************************************************************/
static uint32 UL_testing_Tuning_MeasureAllExtCapCapacitance(void)
{
    uint32 extCapIndex;

    UL_testing_Tuning_SsSwitchSensingMode(UL_testing_Tuning_SENSE_METHOD_BIST_E);
    UL_testing_Tuning_BistSwitchHwConfig(UL_testing_Tuning_BIST_HW_EXT_CAP_E);
    UL_testing_Tuning_BistConfigClock(UL_testing_Tuning_TST_EXT_CAP_RESOLUTION);
    /* Set resolution to fixed value) */
    UL_testing_Tuning_BistSetScanDuration();

    for (extCapIndex = 0u; extCapIndex < UL_testing_Tuning_TST_EXT_CAPS_NUM; extCapIndex++)
    {
        (void)UL_testing_Tuning_MeasureExtCapCapacitance(extCapIndex);
    }

    return (CYRET_SUCCESS);
}
#endif /*(UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_EXTERNAL_CAP_EN) */

/*******************************************************************************
* Function Name: UL_testing_Tuning_BistDisableMode
****************************************************************************//**
*
* \brief
*  Releases captured HW resources.
*
* \details
*  Releases captured HW resources to be used by other modes.
*
*******************************************************************************/
void UL_testing_Tuning_BistDisableMode(void)
{
    #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_HW_GROUP_EN)
        UL_testing_Tuning_BistSwitchHwConfig(UL_testing_Tuning_BIST_HW_UNDEFINED_E);
        UL_testing_Tuning_bistIoConfig = UL_testing_Tuning_BIST_IO_UNDEFINED_E;
    #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_HW_GROUP_EN) */
}

/*******************************************************************************
* Function Name: UL_testing_Tuning_BistInitialize
****************************************************************************//**
*
* \brief
*  Initialize the HW resources to Built-in Self-test mode.
*
* \details
*  Preserved the function to be consistent with other modes (CSX or CSD).
*  In practice resource capturing and initialization are performed later when
*  defined which test type should be executed. Depends on the test type HW
*  resources initialized differently. Only disconnection of all IO from AMUX
*  is common for all the test types.
*
*******************************************************************************/
void UL_testing_Tuning_BistInitialize(void)
{
    #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_HW_GROUP_EN)
        #if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) && \
            (UL_testing_Tuning_SNS_CONNECTION_SHIELD == UL_testing_Tuning_CSD_INACTIVE_SNS_CONNECTION))
            /* Disconnect all the IOs from analog mux busses */
            UL_testing_Tuning_SetAllIOHsiomState(UL_testing_Tuning_TST_HSIOM_GPIO);
        #endif
        /* Set data register to GND that is applicable for all the tests */
        UL_testing_Tuning_SetAllIODrState(UL_testing_Tuning_TST_DR_PIN2GND);
    #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_HW_GROUP_EN) */
}

#if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SH_CAP_EN) && \
     (0u != UL_testing_Tuning_CSD_TOTAL_SHIELD_COUNT))

/*******************************************************************************
* Function Name: UL_testing_Tuning_BistConnectShieldAsSensor
****************************************************************************//**
*
* \brief
*  Connects (a) shield electrode(s) to the HW block via the AMUX bus.
*
* \details
*   The function gets the IO configuration registers addresses, their shifts and
*   masks from the SHIELD_IO_STRUCT object. Basing on this data, it updates
*   the HSIOM and PC registers.
*
*******************************************************************************/
static void UL_testing_Tuning_BistConnectShieldAsSensor(void)
{
    uint8 interruptState;
    uint32 regValue;
    uint32 eltdIndex;
    UL_testing_Tuning_SHIELD_IO_STRUCT const *ioPtr;

    /* Loop through all electrodes of Shield */
    for (eltdIndex = 0u; eltdIndex < UL_testing_Tuning_CSD_TOTAL_SHIELD_COUNT; eltdIndex++)
    {
        ioPtr = &UL_testing_Tuning_shieldIoList[eltdIndex];

        /* Set electrode to High-Z Analog */
        interruptState = CyEnterCriticalSection();
        regValue = CY_GET_REG32(ioPtr->pcPtr);
        regValue &= ~(UL_testing_Tuning_GPIO_PC_MASK << ioPtr->shift);
        CY_SET_REG32(ioPtr->pcPtr, regValue);
        CyExitCriticalSection(interruptState);

        /* Update HSIOM port select register */
        interruptState = CyEnterCriticalSection();
        regValue = CY_GET_REG32(ioPtr->hsiomPtr);
        regValue &= ~(ioPtr->hsiomMask);
        regValue |= (UL_testing_Tuning_TST_HSIOM_CSD_SENSE << ioPtr->hsiomShift);
        CY_SET_REG32(ioPtr->hsiomPtr, regValue);
        CyExitCriticalSection(interruptState);
    }
}
#endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SH_CAP_EN) && \
           (0u != UL_testing_Tuning_CSD_TOTAL_SHIELD_COUNT)) */

#if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS_CAP_EN) && \
     (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) && \
     (0u != UL_testing_Tuning_CSD_TOTAL_SHIELD_COUNT))

/*******************************************************************************
* Function Name: UL_testing_Tuning_BistEnableShieldElectrodes
****************************************************************************//**
*
* \brief
*  This internal function initializes Shield Electrodes.
*
* \details
*  The function sets the bit in the HSIOM register which enables the shield electrode
*  functionality on the pin. The port and pin configurations are stored in
*  the UL_testing_Tuning_shieldIoList structure.
*
*******************************************************************************/
static void UL_testing_Tuning_BistEnableShieldElectrodes(void)
{
    uint32 newRegValue;
    uint32 shieldIndex;

    for (shieldIndex = 0u; shieldIndex < UL_testing_Tuning_CSD_TOTAL_SHIELD_COUNT; shieldIndex++)
    {
        /* Set drive mode to Analog */
        CY_SET_REG32(UL_testing_Tuning_shieldIoList[shieldIndex].pcPtr, CY_GET_REG32(UL_testing_Tuning_shieldIoList[shieldIndex].pcPtr) &
                   ~(UL_testing_Tuning_GPIO_PC_MASK <<(UL_testing_Tuning_shieldIoList[shieldIndex].shift)));

        /* Set HSIOM register mode to CSD_SHIELD or AMUXBUS B connection */
        newRegValue = CY_GET_REG32(UL_testing_Tuning_shieldIoList[shieldIndex].hsiomPtr);
        newRegValue &= ~(UL_testing_Tuning_HSIOM_SEL_MASK << (UL_testing_Tuning_shieldIoList[shieldIndex].hsiomShift));

        /* Set HSIOM register mode to CSD_SHIELD */
        newRegValue |= (UL_testing_Tuning_HSIOM_SEL_CSD_SHIELD << (UL_testing_Tuning_shieldIoList[shieldIndex].hsiomShift));

        CY_SET_REG32(UL_testing_Tuning_shieldIoList[shieldIndex].hsiomPtr, newRegValue);
    }
}
#endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS_CAP_EN) && \
           (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) && \
           (0u != UL_testing_Tuning_CSD_TOTAL_SHIELD_COUNT)) */

#if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) && \
     (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS_CAP_EN) && \
     (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_TANK_EN))

/*******************************************************************************
* Function Name: UL_testing_Tuning_BistEnableShieldTank
****************************************************************************//**
*
* \brief
*   This function configures Csh pin
*
* \details
*   The function updates the CSH_HSIOM register to connect Csh to AMUXBUS-B.
*   For Third-generation HW block it additionally sets Strong drive mode for Csh
*   and applies High level to Csh pin when precharge is set to IO buffer.
*
*******************************************************************************/
static void UL_testing_Tuning_BistEnableShieldTank(void)
{
    uint32 newRegValue;

    /* Update the CSH_HSIOM register to connect Csh to AMUXBUS-B */
    newRegValue = CY_GET_REG32(UL_testing_Tuning_CSH_HSIOM_PTR);
    newRegValue &= (uint32)(~(uint32)UL_testing_Tuning_CSH_HSIOM_MASK);
    newRegValue |= (UL_testing_Tuning_CSH_TO_AMUXBUS_B_MASK << UL_testing_Tuning_CSH_HSIOM_SHIFT);
    CY_SET_REG32(UL_testing_Tuning_CSH_HSIOM_PTR, newRegValue);

    #if ((UL_testing_Tuning_CSH_PRECHARGE_IO_BUF == UL_testing_Tuning_CSD_CSH_PRECHARGE_SRC) &&\
         (UL_testing_Tuning_DISABLE == UL_testing_Tuning_CSDV2))
        /* Set the Strong drive mode for Csh */
        newRegValue = CY_GET_REG32(UL_testing_Tuning_CSH_PC_PTR);
        newRegValue &= (uint32)(~(UL_testing_Tuning_CSH_PC_MASK << UL_testing_Tuning_CSH_PC_SHIFT));
        newRegValue |= (UL_testing_Tuning_CSH_PC_STRONG_MODE << UL_testing_Tuning_CSH_PC_SHIFT);
        CY_SET_REG32(UL_testing_Tuning_CSH_PC_PTR, newRegValue);

        /* Apply High level to the Csh pin for Csh */
        newRegValue = CY_GET_REG32(UL_testing_Tuning_CSH_DR_PTR);
        newRegValue &= (uint32)(~(UL_testing_Tuning_DR_MASK << UL_testing_Tuning_CSH_SHIFT));
        newRegValue |= (uint32)UL_testing_Tuning_CSH_DR_CONFIG;
        CY_SET_REG32(UL_testing_Tuning_CSH_DR_PTR, newRegValue);
    #endif /* ((UL_testing_Tuning_CSH_PRECHARGE_IO_BUF == UL_testing_Tuning_CSD_CSH_PRECHARGE_SRC) &&\
               (UL_testing_Tuning_DISABLE == UL_testing_Tuning_CSDV2)) */
}
#endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) && \
           (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS_CAP_EN) && \
           (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_TANK_EN)) */

/** \cond SECTION_C_LOW_LEVEL */
#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS_CAP_EN)

/*******************************************************************************
* Function Name: UL_testing_Tuning_GetSensorCapacitance
****************************************************************************//**
*
* \brief
*  Measures the specified widget/sensor capacitance.
*
* \details
*  The function measures capacitance of the specified sensor and returns the
*  result, alternatively the result is stored in the Component data structure.
*
*  For CSD sensors, the capacitance of the specified sensor is measured.
*  For CSX sensors, the capacitance the both Rx and Tx electrodes of the sensor
*  is measured. For ganged sensor, total capacitance of all electrodes associated
*  with the sensor is measured. The capacitance measurement result is independent
*  on the Component or sensor tuning parameters and neither tuning parameter nor
*  the capacitance measurement function creates interference to other.
*
*  While measuring capacitance of a CSX sensor electrode, all the non-measured
*  electrodes and CSD shield electrodes (if enabled) are set to active low (GND).
*  While measuring capacitance of a CSD sensor electrode, all the CSX sensor
*  electrodes are set to active low (GND) and all CSD sensor electrodes are
*  set to the state defined by the inactive sensor state parameter in the
*  Component CSD Setting tab of the customizer. If the shield electrode
*  is enabled, it is enabled during CSD sensor capacitance measurement.
*
*  The measurable capacitance range using this function is from 5pF to 255pF.
*  If a returned value is 255, the sensor capacitance can be higher.
*
*  The measured capacitance is stored in the UL_testing_Tuning_RAM_SNS_CP_STRUCT structure.
*  The UL_testing_Tuning_<WidgetName>_PTR2SNS_CP_VALUE register contains a pointer to
*  the array of the specified widget with the sensor capacitance.
*
*  This test can be executed for all the sensors at once using the
*  UL_testing_Tuning_RunSelfTest() function along with the
*  UL_testing_Tuning_TST_SNS_CAP mask.
*
* \param widgetId
*  Specifies the ID number of the widget to be processed.
*  A macro for the widget ID can be found in the
*  UL_testing_Tuning Configuration header file defined as
*  UL_testing_Tuning_<WidgetName>_WDGT_ID.
*
* \param sensorId
*  Specifies the ID number of the sensor within the widget.
*  A macro for the sensor ID within the specified widget can be found in
*  the UL_testing_Tuning Configuration header file defined as
*  UL_testing_Tuning_<WidgetName>_SNS<SensorNumber>_ID.
*
* \return
*  Returns a result of the test execution:
*  - Bits [7..0] The capacitance (in pF) of the CSD electrode or the capacitance
*    of Rx electrode of CSX sensor.
*  - Bits [15..8] The capacitance (in pF) of Tx electrode of CSX sensor.
*  - Bit [30] UL_testing_Tuning_TST_BAD_PARAM The input parameters are invalid.
*
*******************************************************************************/
uint32 UL_testing_Tuning_GetSensorCapacitance(uint32 widgetId, uint32 sensorId)
{
    uint32 result = 0uL;
    uint32 ioSnsId = sensorId;

    if ((UL_testing_Tuning_TOTAL_WIDGETS > widgetId) &&
        (UL_testing_Tuning_dsFlash.wdgtArray[widgetId].totalNumSns > sensorId))
    {
        UL_testing_Tuning_SsSwitchSensingMode(UL_testing_Tuning_SENSE_METHOD_BIST_E);

        switch(UL_testing_Tuning_GET_SENSE_METHOD(&UL_testing_Tuning_dsFlash.wdgtArray[widgetId]))
        {
        #if (0u != UL_testing_Tuning_TOTAL_CSD_WIDGETS)
            case UL_testing_Tuning_SENSE_METHOD_CSD_E:
                UL_testing_Tuning_BistSwitchHwConfig(UL_testing_Tuning_BIST_HW_SNS_CAP_CSD_E);
                break;
        #endif /* #if (0u != UL_testing_Tuning_TOTAL_CSD_WIDGETS) */

        #if (0u != UL_testing_Tuning_TOTAL_CSX_WIDGETS)
            case UL_testing_Tuning_SENSE_METHOD_CSX_E:
                UL_testing_Tuning_BistSwitchHwConfig(UL_testing_Tuning_BIST_HW_SNS_CAP_CSX_E);
                break;
        #endif /* #if (0u != UL_testing_Tuning_TOTAL_CSX_WIDGETS) */

        default:
            CYASSERT(0u);
            break;
        }

        /* For CSX widgets get an index of Rx electrode, for CSD widgets an electrode index is the same as snsId */
        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_CSX_EN)
            if (UL_testing_Tuning_SENSE_METHOD_CSX_E == UL_testing_Tuning_GET_SENSE_METHOD(&UL_testing_Tuning_dsFlash.wdgtArray[widgetId]))
            {
                ioSnsId = sensorId / UL_testing_Tuning_dsFlash.wdgtArray[widgetId].numRows;
            }
            else
            {
                ioSnsId = sensorId;
            }
        #elif (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN)
            ioSnsId = sensorId / UL_testing_Tuning_dsFlash.wdgtArray[widgetId].numRows;
        #else
            ioSnsId = sensorId;
        #endif
        result = UL_testing_Tuning_GetSensorCapacitanceExt(widgetId, ioSnsId);

        /* For CSX widgets get an index of Tx electrode and repeat the test */
        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN)
            if (UL_testing_Tuning_SENSE_METHOD_CSX_E == UL_testing_Tuning_GET_SENSE_METHOD(&UL_testing_Tuning_dsFlash.wdgtArray[widgetId]))
            {
                ioSnsId = (sensorId % UL_testing_Tuning_dsFlash.wdgtArray[widgetId].numRows) +
                                   UL_testing_Tuning_dsFlash.wdgtArray[widgetId].numCols;

                result |= (UL_testing_Tuning_GetSensorCapacitanceExt(widgetId, ioSnsId) << UL_testing_Tuning_08_BIT_SHIFT);

            }
        #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN) */
    }
    else
    {
        result = UL_testing_Tuning_TST_BAD_PARAM;
    }

    return result;
}

/*******************************************************************************
* Function Name: UL_testing_Tuning_GetSensorCapacitanceExt
****************************************************************************//**
*
* \brief
*  The internal function that measures a certain sensor
*  (electrode for CSX widgets) capacitance.
*
* \details
*  The function connects a certain sensor (electrode for CSX widgets)
*  to the previously configured UL_testing_Tuning HW block by using the CSD mode.
*  Then measures a Cp by using the UL_testing_Tuning_GetCapacitanceExt function.
*  After measuring the function sets an inactive connection of the sensor
*  (electrode for CSX widgets) and stores the Cp value in the corresponding
*  register of the UL_testing_Tuning_RAM_SNS_STRUCT.
*
* \param wdgtId
*  Specifies the ID number of the widget to be processed.
*
* \param snsId
*  Specifies the ID number of the sensor within the widget
*  which capacitance will be measured.
*
* \return
*  - Cp of a certain capacitor in pF.
*
*******************************************************************************/
static uint32 UL_testing_Tuning_GetSensorCapacitanceExt(uint32 wdgtId, uint32 ioSnsId)
{
    uint32 result;
    uint8 * cpPtr = UL_testing_Tuning_dsFlash.wdgtArray[wdgtId].ptr2SnsCpArr + ioSnsId;

    /* Connect the specified sensor */
    UL_testing_Tuning_BistConnectSensor(wdgtId, ioSnsId);

    result = UL_testing_Tuning_GetCapacitanceExt();

    /* Disconnect the sensor */
    UL_testing_Tuning_BistSetInactiveSensorConnection(wdgtId, ioSnsId);

    if (UL_testing_Tuning_BIST_SNS_CAP_MAX_CP < result)
    {
        result = UL_testing_Tuning_BIST_SNS_CAP_MAX_CP;
    }

    /* Save cp to data structure */
    *cpPtr = (uint8)result;

    return result;
}

/*******************************************************************************
* Function Name: UL_testing_Tuning_BistConnectSensor
****************************************************************************//**
*
* \brief
*  Connects a sensor (including ganged) port-pin to the HW block via the AMUX
*  bus.
*
* \details
*   The function gets the IO configuration registers addresses, their shifts and
*   masks from the FLASH_IO_STRUCT object. Basing on this data, it updates the HSIOM and
*   PC registers.
*
* \param
*   wdgtIndex Specifies ID of the widget.
*   sensorIndex Specifies ID of the sensor in the widget.
*
*******************************************************************************/
static void UL_testing_Tuning_BistConnectSensor(uint32 wdgtId, uint32 ioSnsId)
{
    uint32 eltdNum;
    uint32 eltdIndex;
    UL_testing_Tuning_FLASH_IO_STRUCT const *ioPtr;
    UL_testing_Tuning_FLASH_SNS_STRUCT const *curFlashSnsPtr;

    /* Check ganged sns flag */
    if (UL_testing_Tuning_GANGED_SNS_MASK == (UL_testing_Tuning_dsFlash.wdgtArray[wdgtId].staticConfig & UL_testing_Tuning_GANGED_SNS_MASK))
    {
        curFlashSnsPtr = (UL_testing_Tuning_FLASH_SNS_STRUCT const *)UL_testing_Tuning_dsFlash.wdgtArray[wdgtId].ptr2SnsFlash;
        curFlashSnsPtr += ioSnsId;
        ioPtr = &UL_testing_Tuning_ioList[curFlashSnsPtr->firstPinId];
        eltdNum = curFlashSnsPtr->numPins;
    }
    else
    {
        ioPtr = (UL_testing_Tuning_FLASH_IO_STRUCT const *)UL_testing_Tuning_dsFlash.wdgtArray[wdgtId].ptr2SnsFlash + ioSnsId;
        eltdNum = 1u;
    }

    /* Loop through all electrodes of specified sensor */
    for (eltdIndex = 0u; eltdIndex < eltdNum; eltdIndex++)
    {
        /* Set electrode to CSD_SENSE mode to be scanned */
        UL_testing_Tuning_SetElectrodePc(ioPtr, UL_testing_Tuning_TST_PC_HIGHZA);
        UL_testing_Tuning_SetElectrodeHsiom(ioPtr, UL_testing_Tuning_TST_HSIOM_CSD_SENSE);
        ioPtr++;
    }
}

/*******************************************************************************
* Function Name: UL_testing_Tuning_MeasureAllSensorCapacitance
****************************************************************************//**
*
* \brief
*  The internal function that measures all the sensors Cp capacitance.
*
* \details
*  The function that measures Cp of all the sensors by using
*  UL_testing_Tuning_GetSensorCapacitance function.
*  The function stores the Cp values in the corresponding BIST data
*  structure registers.
*
* \return
*  Returns the status of the measuring process:
*  - CYRET_SUCCESS.
*
*******************************************************************************/
static uint32 UL_testing_Tuning_MeasureAllSensorCapacitance(void)
{
    uint32 wdgtId;
    uint32 ioSnsId;
    uint32 totalNumIoSns;
    uint32 result = CYRET_SUCCESS;

    /* Loop through all the widgets */
    for (wdgtId = 0u; wdgtId < UL_testing_Tuning_TOTAL_WIDGETS; wdgtId++)
    {
        switch(UL_testing_Tuning_GET_SENSE_METHOD(&UL_testing_Tuning_dsFlash.wdgtArray[wdgtId]))
        {
        #if (0u != UL_testing_Tuning_TOTAL_CSD_WIDGETS)
            case UL_testing_Tuning_SENSE_METHOD_CSD_E:
                UL_testing_Tuning_BistSwitchHwConfig(UL_testing_Tuning_BIST_HW_SNS_CAP_CSD_E);
                break;
        #endif /* #if (0u != UL_testing_Tuning_TOTAL_CSD_WIDGETS) */

        #if (0u != UL_testing_Tuning_TOTAL_CSX_WIDGETS)
            case UL_testing_Tuning_SENSE_METHOD_CSX_E:
                UL_testing_Tuning_BistSwitchHwConfig(UL_testing_Tuning_BIST_HW_SNS_CAP_CSX_E);
                break;
        #endif /* #if (0u != UL_testing_Tuning_TOTAL_CSX_WIDGETS) */

        default:
            CYASSERT(0u);
            break;
        }

        /* Get a total number of widget elements: for CSX it is numRows + numCols, for CSD it is totalNumSns */
        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_CSX_EN)
            if (UL_testing_Tuning_SENSE_METHOD_CSX_E == UL_testing_Tuning_GET_SENSE_METHOD(&UL_testing_Tuning_dsFlash.wdgtArray[wdgtId]))
            {
                totalNumIoSns = (uint32)UL_testing_Tuning_dsFlash.wdgtArray[wdgtId].numRows + UL_testing_Tuning_dsFlash.wdgtArray[wdgtId].numCols;
            }
            else
            {
                totalNumIoSns = (uint32)UL_testing_Tuning_dsFlash.wdgtArray[wdgtId].totalNumSns;
            }
        #elif (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN)
            totalNumIoSns = (uint32)UL_testing_Tuning_dsFlash.wdgtArray[wdgtId].numRows + UL_testing_Tuning_dsFlash.wdgtArray[wdgtId].numCols;
        #else
            totalNumIoSns = (uint32)UL_testing_Tuning_dsFlash.wdgtArray[wdgtId].totalNumSns;
        #endif

        /* Loop through all the sensors */
        for (ioSnsId = 0u; ioSnsId < totalNumIoSns; ioSnsId++)
        {
            (void)UL_testing_Tuning_GetSensorCapacitanceExt(wdgtId, ioSnsId);
        }
    }
    return result;
}
#endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS_CAP_EN) */

#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SH_CAP_EN)

/*******************************************************************************
* Function Name: UL_testing_Tuning_GetShieldCapacitance
****************************************************************************//**
*
* \brief
*  Measures the shield electrode capacitance.
*
* \details
*  The function measures capacitance of the shield electrode and returns the
*  result, alternatively the result is stored in
*  UL_testing_Tuning_SHIELD_CAP_VALUE of data structure. If the shield consists
*  of several electrodes, total capacitance of all shield electrodes is
*  reported.
*
*  While measuring capacitance of shield electrode, the sensor states are
*  inherited from the Component configuration. All the CSX electrodes are
*  set to active low (GND) and all the CSD electrodes are set to state defined
*  by the inactive sensor state parameter in the Component CSD Setting tab
*  of the customizer.
*
*  The measurable capacitance range using this function is from 5pF to 255pF.
*
*  This test can be executed for all the sensors at once using the
*  UL_testing_Tuning_RunSelfTest() function with the UL_testing_Tuning_TST_SH_CAP
*  mask.
*
* \return
*  The shield electrode capacitance (in pF)
*
*******************************************************************************/
uint32 UL_testing_Tuning_GetShieldCapacitance(void)
{
    uint32 result;

    UL_testing_Tuning_SsSwitchSensingMode(UL_testing_Tuning_SENSE_METHOD_BIST_E);
    UL_testing_Tuning_BistSwitchHwConfig(UL_testing_Tuning_BIST_HW_SH_CAP_E);

    #if (0u != UL_testing_Tuning_CSD_TOTAL_SHIELD_COUNT)
        UL_testing_Tuning_BistConnectShieldAsSensor();
    #endif /* (0u != UL_testing_Tuning_CSD_TOTAL_SHIELD_COUNT) */

    result = UL_testing_Tuning_GetCapacitanceExt();

    /* Disconnect shield electrodes */
    #if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) && \
         (0u != UL_testing_Tuning_CSD_TOTAL_SHIELD_COUNT))
        UL_testing_Tuning_BistDisableShieldElectrodes();
    #endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) && \
               (0u != UL_testing_Tuning_CSD_TOTAL_SHIELD_COUNT)) */

    if (UL_testing_Tuning_BIST_SNS_CAP_MAX_CP < result)
    {
        result = UL_testing_Tuning_BIST_SNS_CAP_MAX_CP;
    }

    /* Save a capacitance value to the data structure */
    UL_testing_Tuning_dsRam.selfTest.shieldCap = (uint16)result;

    return result;
}
#endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SH_CAP_EN) */

#if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS_CAP_EN) || \
     (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SH_CAP_EN))

/*******************************************************************************
* Function Name: UL_testing_Tuning_GetCapacitanceExt
****************************************************************************//**
*
* \brief
*  The internal function that measures a capacitance attached to AMUXBUS.
*
* \details
*  The function measures Cp of a certain sensor or shield by using the
*  CSD mode and defined Idac mode, sense clock frequency and resolution.
*  The range for sensor measuring is 5 to 250 pF, for shield 10 to 250 pf.
*  The function calibrates the Idac for reach a defined target of raw counts.
*  If reached Idac will be lower then a certain value the function sets
*  the higher sense clock frequency.
*  The function stores the Cp value in the corresponding register of
*  UL_testing_Tuning_RAM_SNS_STRUCT.
*
* \return
*  - Cp of  a certain sensor or shield in pF.
*
*******************************************************************************/
static uint32 UL_testing_Tuning_GetCapacitanceExt(void)
{
    uint32 cp;
    uint32 idacValue = 0uL;
    uint32 snsClkIncrementIndex;
    uint32 idacMask = ((uint32)UL_testing_Tuning_BIST_CAL_MIDDLE_BIT << 1uL);

    if (UL_testing_Tuning_BIST_HW_SH_CAP_E == UL_testing_Tuning_bistHwConfig)
    {
        /* Set SnsClk frequency for shield measurement up to 500pF */
        snsClkIncrementIndex = 0uL;
    }
    else
    {
        /* Set SnsClk frequency for sensor measurement up to 250pF */
        snsClkIncrementIndex = 1uL;
    }

    /* Initialize raw count for first cycle pass */
    UL_testing_Tuning_bistRawcount = 0u;

    UL_testing_Tuning_BistSetScanDuration();

    /* Finding optimal IDAC and Frequency */
    while (0uL != idacMask)
    {
        /* Decrease IDAC until rawData reaches the calibration target */
        if (UL_testing_Tuning_BIST_CALIBRATION_TARGET > UL_testing_Tuning_bistRawcount)
        {
            idacValue &= (uint32)(~idacMask);
        }
        idacMask >>= 1uL;
        idacValue |= idacMask;
        if (0uL == idacValue)
        {
            idacValue++;
        }

        /* Revert back Idac value and increase SnsClk frequency if Idac is too small */
        if (idacValue < UL_testing_Tuning_BIST_MIN_IDAC_VALUE)
        {
            if ((UL_testing_Tuning_TST_BASE_SNS_CLK_DIVIDER >> (snsClkIncrementIndex + 1uL)) >= UL_testing_Tuning_TST_MIN_SNS_CLK_DIVIDER)
            {
                snsClkIncrementIndex++;
                idacValue &= (uint32)(~idacMask);
                idacMask = (uint32)(UL_testing_Tuning_BIST_CAL_MIDDLE_BIT);
                idacValue = idacMask;
            }
        }

        /*
        * Setup scan parameters:
        * - IDAC
        * - SnsClk
        * - Resolution (conversion number)
        */
        UL_testing_Tuning_BistSetIdac(idacValue);
        UL_testing_Tuning_BistConfigClock((uint32)UL_testing_Tuning_TST_BASE_SNS_CLK_DIVIDER >> snsClkIncrementIndex);
        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2)
            CY_SET_REG32(UL_testing_Tuning_SEQ_NORM_CNT_PTR, ((uint32)UL_testing_Tuning_BIST_10_BIT_MASK /
                                            ((uint32)UL_testing_Tuning_TST_BASE_SNS_CLK_DIVIDER >> snsClkIncrementIndex)) &
                                            UL_testing_Tuning_SEQ_NORM_CNT_CONV_CNT_MASK);
        #endif

        /* Perform scanning */
        UL_testing_Tuning_dsRam.status |= UL_testing_Tuning_SW_STS_BUSY;
        UL_testing_Tuning_BistStartSample();
        UL_testing_Tuning_WaitEndOfScan(UL_testing_Tuning_BIST_MEASURE_WATCHDOG_CYCLES_NUM);
    }

    /*
    * Capacitance calculation:
    *   cp(pF) = Rawcount * IdacCode * IdacGain(nA) * 1000 / (MaxRaw * SnsClk(kHz) * Vref(mV))
    * Details:
    *   - The order of calculation is changed to avoid the potential overflow
    *   - The half of the last divider is added to implement the value rounding
    */
    cp = (UL_testing_Tuning_TST_SNS_CAP_IDAC_GAIN * UL_testing_Tuning_BIST_SNS_CAP_UNIT_SCALE) / UL_testing_Tuning_CSD_VREF_MV;
    cp = (cp * idacValue) / UL_testing_Tuning_BIST_10_BIT_MASK;
    cp = cp * (uint32)UL_testing_Tuning_bistRawcount;
    cp = (cp + (((uint32)UL_testing_Tuning_TST_BASE_SNS_CLK_KHZ << snsClkIncrementIndex) >> 1uL)) /
                ((uint32)UL_testing_Tuning_TST_BASE_SNS_CLK_KHZ << snsClkIncrementIndex);

    return cp;
}
#endif /* #if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS_CAP_EN) || \
     (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SH_CAP_EN)) */
/** \endcond */
#if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS_CAP_EN) || \
     (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SH_CAP_EN))
/*******************************************************************************
* Function Name: UL_testing_Tuning_BistDisableHwSnsCap
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
static void UL_testing_Tuning_BistDisableHwSnsCap(void)
{
    uint32 regValue;
    uint8  interruptState;

    /* Disconnect Cmod from AMUXBUS-A using HSIOM registers */
    #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN)
        interruptState = CyEnterCriticalSection();
        regValue = CY_GET_REG32(UL_testing_Tuning_CMOD_HSIOM_PTR);
        regValue &= (uint32)(~(uint32)UL_testing_Tuning_CMOD_HSIOM_MASK);
        CY_SET_REG32(UL_testing_Tuning_CMOD_HSIOM_PTR, regValue);
        CyExitCriticalSection(interruptState);
    #else
        interruptState = CyEnterCriticalSection();
        regValue = CY_GET_REG32(UL_testing_Tuning_CintA_HSIOM_PTR);
        regValue &= (uint32)(~(uint32)UL_testing_Tuning_CintA_HSIOM_MASK);
        regValue |= (UL_testing_Tuning_HSIOM_SEL_GPIO << UL_testing_Tuning_CintA_HSIOM_SHIFT);
        CY_SET_REG32(UL_testing_Tuning_CintA_HSIOM_PTR, regValue);
        CyExitCriticalSection(interruptState);

        interruptState = CyEnterCriticalSection();
        regValue = CY_GET_REG32(UL_testing_Tuning_CintB_HSIOM_PTR);
        regValue &= (uint32)(~(uint32)UL_testing_Tuning_CintB_HSIOM_MASK);
        regValue |= (UL_testing_Tuning_HSIOM_SEL_GPIO << UL_testing_Tuning_CintB_HSIOM_SHIFT);
        CY_SET_REG32(UL_testing_Tuning_CintB_HSIOM_PTR, regValue);
        CyExitCriticalSection(interruptState);
    #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN) */

    #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS_CAP_EN)
        /* Verification of the bist mode */
        if (UL_testing_Tuning_BIST_HW_SNS_CAP_CSD_E == UL_testing_Tuning_bistHwConfig)
        {
            /* Disconnect Csh from AMUXBUS-B */
            #if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) && \
                 (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_TANK_EN))
                interruptState = CyEnterCriticalSection();
                regValue = CY_GET_REG32(UL_testing_Tuning_CSH_HSIOM_PTR);
                regValue &= (uint32)(~(uint32)(UL_testing_Tuning_CSH_TO_AMUXBUS_B_MASK << UL_testing_Tuning_CSH_HSIOM_SHIFT));
                CY_SET_REG32(UL_testing_Tuning_CSH_HSIOM_PTR, regValue);
                CyExitCriticalSection(interruptState);
            #endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) && \
                       (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_TANK_EN)) */

            /* Disconnect shield electrodes */
            #if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) && \
                 (0u != UL_testing_Tuning_CSD_TOTAL_SHIELD_COUNT))
                UL_testing_Tuning_BistDisableShieldElectrodes();
            #endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) && \
                       (0u != UL_testing_Tuning_CSD_TOTAL_SHIELD_COUNT)) */

            #if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) && \
                (UL_testing_Tuning_SNS_CONNECTION_SHIELD == UL_testing_Tuning_CSD_INACTIVE_SNS_CONNECTION))
                /*
                * Disconnect all IOs from AMUXBUS_B as all rest tests suppose the electrodes are not
                * connected to any bus. It is done for performance optiomization:
                * not to write HSIOM register each time at test type switching.
                */
                UL_testing_Tuning_SetAllIOHsiomState(UL_testing_Tuning_TST_HSIOM_GPIO);
            #endif
        }
    #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS_CAP_EN) */
}


#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN)
    /*******************************************************************************
    * Function Name: UL_testing_Tuning_BistSetAllInactiveSensorConnection
    ****************************************************************************//**
    *
    * \brief
    *  Resets all the CSD sensors to the non-sampling state by sequential
    *  disconnecting all the sensors from the Analog MUX bus and putting them to
    *  an inactive state.
    *
    * \details
    *   The function goes through all the sensors of CSD widgets and updates appropriate bits in
    *   the IO HSIOM, PC and DR registers depending on the Inactive sensor connection
    *   parameter. DR register bits are set to zero when the Inactive sensor
    *   connection is Ground or Hi-Z.
    *
    *******************************************************************************/
    static void UL_testing_Tuning_BistSetAllInactiveSensorConnection(void)
    {
        uint32 wdgtId;
        uint32 snsId;

        for (wdgtId = 0uL; wdgtId < UL_testing_Tuning_TOTAL_WIDGETS; wdgtId++)
        {
            if (UL_testing_Tuning_SENSE_METHOD_CSD_E == UL_testing_Tuning_GET_SENSE_METHOD(&UL_testing_Tuning_dsFlash.wdgtArray[wdgtId]))
            {
                /* Go through all the sensors in widget */
                for (snsId = 0uL; snsId < (uint8)UL_testing_Tuning_dsFlash.wdgtArray[wdgtId].totalNumSns; snsId++)
                {
                    UL_testing_Tuning_BistSetInactiveSensorConnection(wdgtId, snsId);
                }
            }
        }
    }
#endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN) */


/*******************************************************************************
* Function Name: UL_testing_Tuning_BistSetInactiveSensorConnection
****************************************************************************//**
*
* \brief
*  Sets an inactive state for a certain sensor (electrode).
*
* \details
*  The function updates appropriate bits in the IO HSIOM and PC registers
*  depending on the Inactive sensor connection parameter.
*  The function proceeds ganged sensors(electrodes).
*
* \param wdgtId
*  Specifies the ID number of the widget to be processed.
*
* \param ioSnsId
*  Specifies the ID number of the sensor (electrode) within the widget to be processed.
*
*******************************************************************************/
static void UL_testing_Tuning_BistSetInactiveSensorConnection(uint32 wdgtId, uint32 ioSnsId)
{
    uint32 eltdNum;
    uint32 eltdIndex;
    UL_testing_Tuning_FLASH_IO_STRUCT const *ioPtr;
    UL_testing_Tuning_FLASH_SNS_STRUCT const *curFlashSnsPtr;

    /* Check ganged sns flag */
    if (UL_testing_Tuning_GANGED_SNS_MASK == (UL_testing_Tuning_dsFlash.wdgtArray[wdgtId].staticConfig & UL_testing_Tuning_GANGED_SNS_MASK))
    {
        curFlashSnsPtr = (UL_testing_Tuning_FLASH_SNS_STRUCT const *)UL_testing_Tuning_dsFlash.wdgtArray[wdgtId].ptr2SnsFlash;
        curFlashSnsPtr += ioSnsId;
        ioPtr = &UL_testing_Tuning_ioList[curFlashSnsPtr->firstPinId];
        eltdNum = curFlashSnsPtr->numPins;
    }
    else
    {
        ioPtr = (UL_testing_Tuning_FLASH_IO_STRUCT const *)UL_testing_Tuning_dsFlash.wdgtArray[wdgtId].ptr2SnsFlash + ioSnsId;
        eltdNum = 1uL;
    }

    /* Loop through all electrodes of specified sensor */
    for (eltdIndex = 0uL; eltdIndex < eltdNum; eltdIndex++)
    {
        if (UL_testing_Tuning_BIST_HW_SNS_CAP_CSD_E == UL_testing_Tuning_bistHwConfig)
        {
            #if (UL_testing_Tuning_SNS_CONNECTION_SHIELD == UL_testing_Tuning_CSD_INACTIVE_SNS_CONNECTION)
                /* Update port configuration register (drive mode to HiZ Analog) for sensor */
                UL_testing_Tuning_SetElectrodePc(ioPtr, UL_testing_Tuning_TST_PC_HIGHZA);
                /* Connect to Shield */
                UL_testing_Tuning_SetElectrodeHsiom(ioPtr, UL_testing_Tuning_TST_HSIOM_CSD_SHIELD);
            #else
                /* Update HSIOM register to disconnect sensor */
                UL_testing_Tuning_SetElectrodeHsiom(ioPtr, UL_testing_Tuning_TST_HSIOM_GPIO);
                /* Set drive mode for elecctrode based on inactive sensor connection parameter */
                UL_testing_Tuning_SetElectrodePc(ioPtr, UL_testing_Tuning_TST_INACTIVE_SNS_GPIO_DM);
            #endif /* (UL_testing_Tuning_SNS_CONNECTION_SHIELD != UL_testing_Tuning_CSD_INACTIVE_SNS_CONNECTION) */
        }
        else if (UL_testing_Tuning_BIST_HW_SNS_CAP_CSX_E == UL_testing_Tuning_bistHwConfig)
        {
            /* Update HSIOM register to disconnect sensor */
            UL_testing_Tuning_SetElectrodeHsiom(ioPtr, UL_testing_Tuning_TST_HSIOM_GPIO);
            /* Update port configuration register (drive mode to HiZ Analog) for sensor */
            UL_testing_Tuning_SetElectrodePc(ioPtr, UL_testing_Tuning_TST_PC_STRONG);
        }
        else if (UL_testing_Tuning_BIST_HW_SH_CAP_E == UL_testing_Tuning_bistHwConfig)
        {
            #if (UL_testing_Tuning_SNS_CONNECTION_SHIELD == UL_testing_Tuning_CSD_INACTIVE_SNS_CONNECTION)
                /* Set electrode to CSD_SENSE mode to be scanned */
                UL_testing_Tuning_SetElectrodePc(ioPtr, UL_testing_Tuning_TST_PC_HIGHZA);
                UL_testing_Tuning_SetElectrodeHsiom(ioPtr, UL_testing_Tuning_TST_HSIOM_CSD_SENSE);
            #else
                /* Update HSIOM register to disconnect sensor */
                UL_testing_Tuning_SetElectrodeHsiom(ioPtr, UL_testing_Tuning_TST_HSIOM_GPIO);
                /* Set drive mode for elecctrode based on inactive sensor connection parameter */
                UL_testing_Tuning_SetElectrodePc(ioPtr, UL_testing_Tuning_TST_INACTIVE_SNS_GPIO_DM);
            #endif /* (UL_testing_Tuning_SNS_CONNECTION_SHIELD != UL_testing_Tuning_CSD_INACTIVE_SNS_CONNECTION) */
        }
        else
        {
            /* Do nothing */
        }
        ioPtr++;
    }
}

#if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) && \
     (0u != UL_testing_Tuning_CSD_TOTAL_SHIELD_COUNT))

/*******************************************************************************
* Function Name: UL_testing_Tuning_BistDisableShieldElectrodes
****************************************************************************//**
*
* \brief
*   This internal function disables Shield Electrodes.
*
* \details
*   The function resets the bit in the HSIOM register which disables the shield
*   electrode functionality on the pin. The port and pin configurations are
*   stored in  the UL_testing_Tuning_shieldIoList structure.
*
*******************************************************************************/
static void UL_testing_Tuning_BistDisableShieldElectrodes(void)
{
    uint32 newRegValue;
    uint32 shieldIndex;

    for (shieldIndex = 0u; shieldIndex < UL_testing_Tuning_CSD_TOTAL_SHIELD_COUNT; shieldIndex++)
    {
        /* Reset HSIOM register (to GPIO state) */
        CY_SET_REG32(UL_testing_Tuning_shieldIoList[shieldIndex].hsiomPtr, CY_GET_REG32(UL_testing_Tuning_shieldIoList[shieldIndex].hsiomPtr) &
                     (uint32)~(uint32)(UL_testing_Tuning_HSIOM_SEL_MASK << UL_testing_Tuning_shieldIoList[shieldIndex].hsiomShift));

        /* Update port configuration register (drive mode to ground)  */
        newRegValue = CY_GET_REG32(UL_testing_Tuning_shieldIoList[shieldIndex].pcPtr);
        newRegValue &= ~(UL_testing_Tuning_GPIO_PC_MASK << UL_testing_Tuning_shieldIoList[shieldIndex].shift);
        newRegValue |= (UL_testing_Tuning_SNS_GROUND_CONNECT << UL_testing_Tuning_shieldIoList[shieldIndex].shift);
        CY_SET_REG32(UL_testing_Tuning_shieldIoList[shieldIndex].pcPtr, newRegValue);

        /* Set logic 0 to port data register */
        CY_SET_REG32(UL_testing_Tuning_shieldIoList[shieldIndex].drPtr, CY_GET_REG32(UL_testing_Tuning_shieldIoList[shieldIndex].drPtr) &
                   (uint32)~(uint32)((uint32)1u << UL_testing_Tuning_shieldIoList[shieldIndex].drShift));
    }
}
#endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) && \
           (0u != UL_testing_Tuning_CSD_TOTAL_SHIELD_COUNT)) */

#endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS_CAP_EN) || \
           (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SH_CAP_EN)) */

#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_MEASUREMENT_GROUP_EN)

/*******************************************************************************
* Function Name: UL_testing_Tuning_BistPostSingleScan
****************************************************************************//**
*
* \brief
*  This is an internal ISR function for the single-sensor scanning implementation.
*
* \details
*  This ISR handler is triggered when the CSD scan for a capacitance measuring
*  is finished.
*
*  The following tasks are performed:
*    1. Disable the CSD interrupt.
*    2. Read the Counter register and update the UL_testing_Tuning_bistRawcount with raw data.
*    3. Reset the BUSY flag.
*    4. Connect the Vref buffer to the AMUX bus (for Third-generation HW block).
*
*******************************************************************************/
CY_ISR(UL_testing_Tuning_BistPostSingleScan)
{
    uint32 tmpRawData;

    /* Clear pending interrupt */
    CY_SET_REG32(UL_testing_Tuning_INTR_PTR, UL_testing_Tuning_INTR_SET_MASK);
    /* Read SlotResult from Raw Counter */
    tmpRawData = (uint16)CY_GET_REG32(UL_testing_Tuning_COUNTER_PTR);

    #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2)
        if((UL_testing_Tuning_TST_EXT_CAP_RESOLUTION - 1uL) < tmpRawData)
        {
            tmpRawData = (UL_testing_Tuning_TST_EXT_CAP_RESOLUTION - 1uL);
        }
    #endif

    UL_testing_Tuning_bistRawcount = LO16(tmpRawData);

    /* Sensor is totally scanned. Reset BUSY flag */
    UL_testing_Tuning_dsRam.status &= ~(UL_testing_Tuning_SW_STS_BUSY | UL_testing_Tuning_WDGT_SW_STS_BUSY);
    #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2)
        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN)
        /* Open HCBV and HCBG switches */
            CY_SET_REG32(UL_testing_Tuning_SW_SHIELD_SEL_PTR, UL_testing_Tuning_SW_SHIELD_SEL_SW_HCBV_STATIC_OPEN |
                                                             UL_testing_Tuning_SW_SHIELD_SEL_SW_HCBG_STATIC_OPEN);
        #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) */
        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_BLOCK_OFF_AFTER_SCAN_EN)
            /* Disable the Fourth-generation HW block */
            CY_SET_REG32(UL_testing_Tuning_CONFIG_PTR, UL_testing_Tuning_bistConfigCsd);
        #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_BLOCK_OFF_AFTER_SCAN_EN) */
    #else /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2) */
        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_BLOCK_OFF_AFTER_SCAN_EN)
            /* Disable the Third-generation HW block. Connect the Vref buffer to AMUX bus */
            #if ((UL_testing_Tuning_CSH_PRECHARGE_IO_BUF == UL_testing_Tuning_CSD_CSH_PRECHARGE_SRC) &&\
                 (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_TANK_EN))
                CY_SET_REG32(UL_testing_Tuning_CONFIG_PTR, UL_testing_Tuning_bistConfigCsd | UL_testing_Tuning_BIST_CMOD_PRECHARGE_CONFIG);
            #else
                CY_SET_REG32(UL_testing_Tuning_CONFIG_PTR, UL_testing_Tuning_bistConfigCsd | UL_testing_Tuning_BIST_CTANK_PRECHARGE_CONFIG);
            #endif /* ((UL_testing_Tuning_CSH_PRECHARGE_IO_BUF == UL_testing_Tuning_CSD_CSH_PRECHARGE_SRC) &&\
                       (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_TANK_EN)) */
        #else
            /* Connect the Vref buffer to AMUX bus. The Third-generation HW block is enabled */
            CY_SET_REG32(UL_testing_Tuning_CONFIG_PTR, UL_testing_Tuning_bistConfigCsd | UL_testing_Tuning_BIST_CTANK_PRECHARGE_CONFIG_CSD_EN);
        #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_BLOCK_OFF_AFTER_SCAN_EN) */
    #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2) */
}

#if (UL_testing_Tuning_DISABLE == UL_testing_Tuning_CSDV2)

/*******************************************************************************
* Function Name: UL_testing_Tuning_BistClockRestart
****************************************************************************//**
*
* \brief
*  This function restarts the clocks.
*
* \details
*  The function performs the following:
*  1) Stops, sets dividers and starts ModClk clock.
*  2) Stops and starts the SnsClk clock.
*  3) Scans one cycle to reset the flip-flop for the Direct clock mode.
*
*******************************************************************************/
static void UL_testing_Tuning_BistClockRestart(void)
{
    /* Stop clocks, set dividers, and start the clock to align the clock phase */
    UL_testing_Tuning_SsSetModClkClockDivider(UL_testing_Tuning_TST_MOD_CLK_DIVIDER);

    #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_IS_M0S8PERI_BLOCK)
        /* Stop the sense clock   */
        CY_SET_REG32(UL_testing_Tuning_SNSCLK_CMD_PTR,
                     ((uint32)UL_testing_Tuning_SnsClk__DIV_ID <<
                     UL_testing_Tuning_SNSCLK_CMD_DIV_SHIFT)|
                     ((uint32)UL_testing_Tuning_SNSCLK_CMD_DISABLE_MASK));

        /* Check whether the previous sense clock start command has completed */
        while(0u != (CY_GET_REG32(UL_testing_Tuning_SNSCLK_CMD_PTR) & UL_testing_Tuning_SNSCLK_CMD_ENABLE_MASK))
        {
            /* Wait until the previous sense clock start command has completed */
        }

        /* Start the sense clock aligned to a previously started modulator clock */
        CY_SET_REG32(UL_testing_Tuning_SNSCLK_CMD_PTR,
                     (uint32)(((uint32)UL_testing_Tuning_SnsClk__DIV_ID << UL_testing_Tuning_SNSCLK_CMD_DIV_SHIFT) |
                      ((uint32)UL_testing_Tuning_ModClk__PA_DIV_ID << UL_testing_Tuning_SNSCLK_CMD_PA_DIV_SHIFT) |
                      UL_testing_Tuning_SNSCLK_CMD_ENABLE_MASK));
    #else
        /* Clear the bit to disable the SnsClk clock */
        CY_SET_REG32(UL_testing_Tuning_SNSCLK_CMD_PTR,
                     CY_GET_REG32(UL_testing_Tuning_SNSCLK_CMD_PTR) &
                     (uint32)(~(uint32)UL_testing_Tuning_SnsClk__ENABLE_MASK));

        /* Set a bit to enable the SnsClk clock */
        CY_SET_REG32(UL_testing_Tuning_SNSCLK_CMD_PTR,
                     CY_GET_REG32(UL_testing_Tuning_SNSCLK_CMD_PTR) |
                    UL_testing_Tuning_SnsClk__ENABLE_MASK);
    #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_IS_M0S8PERI_BLOCK) */

    /* Scan one cycle to reset the flip-flop for the Direct clock mode */
    CyIntDisable(UL_testing_Tuning_ISR_NUMBER);
    CY_SET_REG32(UL_testing_Tuning_COUNTER_PTR, UL_testing_Tuning_ONE_CYCLE);

    while(0u != (CY_GET_REG32(UL_testing_Tuning_COUNTER_PTR) & UL_testing_Tuning_RESOLUTION_16_BITS))
    {
        /* Wait until scanning is complete */
    }
    CY_SET_REG32(UL_testing_Tuning_INTR_PTR, UL_testing_Tuning_INTR_SET_MASK);
    CyIntClearPending(UL_testing_Tuning_ISR_NUMBER);
}
#endif /* (UL_testing_Tuning_DISABLE == UL_testing_Tuning_CSDV2) */

/*******************************************************************************
* Function Name: UL_testing_Tuning_BistStartSample
****************************************************************************//**
*
* \brief
*   Starts the CSD conversion for a capacitance measuring.
*
* \details
*   This function assumes that the CSD block is already set up using
*   the UL_testing_Tuning_BistEnableHwSnsCap function and the capacitance
*   is connected to the CSD block.
*   The function performs the following tasks:
*   1. Disables the CSD interrupt.
*   2. Enables CSD block if it was disabled.
*   3. Restarts the clock (only for the Third-generation HW block and only for sensor capacitance measuring).
*   3. Pre charges Cmod (only for a sensor capacitance measuring or for the Third-generation HW block).
*   4. Starts the scan.
*
*******************************************************************************/
static void UL_testing_Tuning_BistStartSample(void)
{
#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2)
    /* Fourth-generation HW block section */

    /* Disable the CSD interrupt to prevent it during coarse initialization */
    CyIntDisable(UL_testing_Tuning_ISR_NUMBER);
    /* Enable power to sub-blocks */
    CY_SET_REG32(UL_testing_Tuning_HSCMP_PTR, UL_testing_Tuning_HSCMP_EN_MASK);

    #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_BLOCK_OFF_AFTER_SCAN_EN)
        CY_SET_REG32(UL_testing_Tuning_CONFIG_PTR, UL_testing_Tuning_bistConfigCsd |
                                                  UL_testing_Tuning_CONFIG_SENSE_EN_MASK |
                                                  UL_testing_Tuning_CONFIG_ENABLE_MASK);

        /* Wait until CSD block power is ON */
        CyDelayCycles(UL_testing_Tuning_BIST_BLOCK_ON_DELAY);
    #else
        /* Wait until HSCMP power is ON */
        CyDelayCycles(UL_testing_Tuning_BIST_HSCMP_ON_DELAY);
    #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_BLOCK_OFF_AFTER_SCAN_EN) */

    if (UL_testing_Tuning_bistHwConfig != UL_testing_Tuning_BIST_HW_EXT_CAP_E)
    {
        /* Precharging Cmod and Csh */
        UL_testing_Tuning_BistCmodPrecharge();
    }

    /* Trigger Scan */
    UL_testing_Tuning_BistTriggerScan();

#else
    /* Third-generation HW block section */
    uint8 interruptState;

    interruptState = CyEnterCriticalSection();

    /* Enable the CSD block. Disconnect the Vref buffer from AMUX */
    #if ((UL_testing_Tuning_CSH_PRECHARGE_IO_BUF == UL_testing_Tuning_CSD_CSH_PRECHARGE_SRC) &&\
         (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_TANK_EN))
        CY_SET_REG32(UL_testing_Tuning_CONFIG_PTR, UL_testing_Tuning_bistConfigCsd | UL_testing_Tuning_BIST_CMOD_PRECHARGE_CONFIG_CSD_EN);
    #else
        CY_SET_REG32(UL_testing_Tuning_CONFIG_PTR, UL_testing_Tuning_bistConfigCsd | UL_testing_Tuning_BIST_CTANK_PRECHARGE_CONFIG_CSD_EN);
    #endif

    /* Restart the clocks. Scan one cycle to reset the flip-flop for the Direct clock mode */
    if (UL_testing_Tuning_bistHwConfig != UL_testing_Tuning_BIST_HW_EXT_CAP_E)
    {
        UL_testing_Tuning_BistClockRestart();
    }
    /* Pre-charging Cmod and Csh */
    UL_testing_Tuning_BistCmodPrecharge();
    /* Trigger Scan */
    UL_testing_Tuning_BistTriggerScan();

    CyExitCriticalSection(interruptState);
#endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2) */
}

/*******************************************************************************
* Function Name: UL_testing_Tuning_BistEnableHwSnsCap
****************************************************************************//**
*
* \brief
*  Sets up the UL_testing_Tuning block to perform a capacitanse measuring.
*
* \details
*   This function prepare HW of the UL_testing_Tuning block to CSD sensing mode
*   with BIST defined parameters.
*   The function performs the following tasks:
*   1. Sets the default CSD configuration for BIST scans
*   2. Enables shield electrodes and Csh (if defined)
*   3. Sets Idac config for BIST scans
*   4. Sets a sensor clock frequency (only for a sensor capacitance measuring)
*   5. Sets the ISR handler for BIST scans.
*
*******************************************************************************/
static void UL_testing_Tuning_BistEnableHwSnsCap(void)
{
    uint32 newRegValue;
    uint8  interruptState;

    #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2)
        UL_testing_Tuning_BistDischargeExtCapacitors();
    #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2) */
    
    #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS_CAP_EN)
        /* Verification of the bist mode */
        if (UL_testing_Tuning_BIST_HW_SNS_CAP_CSD_E == UL_testing_Tuning_bistHwConfig)
        {
            #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN)
                /* Connect shields to AMUX-B bus (config HSIOM regs) */
                #if (0u != UL_testing_Tuning_CSD_TOTAL_SHIELD_COUNT)
                    UL_testing_Tuning_BistEnableShieldElectrodes();
                #endif /* (0u != UL_testing_Tuning_CSD_TOTAL_SHIELD_COUNT) */

                #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_TANK_EN)
                    /* Configure Csh */
                    UL_testing_Tuning_BistEnableShieldTank();
                #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_TANK_EN) */
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) */
        }
    #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS_CAP_EN) */

    #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2)

        /* Initialize the unused CSD registers to defaut state */
        CY_SET_REG32(UL_testing_Tuning_INTR_SET_PTR,         UL_testing_Tuning_BIST_INTR_SET_CFG);
        CY_SET_REG32(UL_testing_Tuning_SW_FW_TANK_SEL_PTR,   UL_testing_Tuning_BIST_SW_FW_TANK_SEL_CFG);
        CY_SET_REG32(UL_testing_Tuning_SW_DSI_SEL_PTR,       UL_testing_Tuning_BIST_DEFAULT_SW_DSI_SEL);
        CY_SET_REG32(UL_testing_Tuning_ADC_CTL_PTR,          UL_testing_Tuning_BIST_ADC_CTL_CFG);
        CY_SET_REG32(UL_testing_Tuning_AMBUF_PTR,            UL_testing_Tuning_BIST_AMBUF_PWR_MODE_OFF);
        CY_SET_REG32(UL_testing_Tuning_SW_SHIELD_SEL_PTR,    UL_testing_Tuning_BIST_SW_SHIELD_SEL_CFG);
        CY_SET_REG32(UL_testing_Tuning_SW_HS_P_SEL_PTR,      UL_testing_Tuning_BIST_SW_HS_P_SEL_CFG);
        CY_SET_REG32(UL_testing_Tuning_SW_HS_N_SEL_PTR,      UL_testing_Tuning_BIST_SW_HS_N_SEL_CFG);
        CY_SET_REG32(UL_testing_Tuning_HSCMP_PTR,            UL_testing_Tuning_BIST_HSCMP_CFG);
        CY_SET_REG32(UL_testing_Tuning_SEQ_TIME_PTR,         UL_testing_Tuning_BIST_AUTO_ZERO_TIME);
        CY_SET_REG32(UL_testing_Tuning_SW_RES_PTR,           UL_testing_Tuning_BIST_SW_RES_INIT);

        /* Connect AMUXBUS-A to CSDBUS-A */
        interruptState = CyEnterCriticalSection();
        newRegValue = CY_GET_REG32(UL_testing_Tuning_SW_BYP_SEL_PTR);
        newRegValue |= UL_testing_Tuning_SW_BYP_SEL_SW_BYA_MASK;
        CY_SET_REG32(UL_testing_Tuning_SW_BYP_SEL_PTR, newRegValue);
        CyExitCriticalSection(interruptState);

        /* Verification of the bist mode */
        if (UL_testing_Tuning_BIST_HW_EXT_CAP_E != UL_testing_Tuning_bistHwConfig)
        {
            #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN)
                /* Connect CMOD to AMUXBUS-A */
                interruptState = CyEnterCriticalSection();
                newRegValue = CY_GET_REG32(UL_testing_Tuning_CMOD_HSIOM_PTR);
                newRegValue &= (uint32)(~(uint32)UL_testing_Tuning_CMOD_HSIOM_MASK);
                newRegValue |= (UL_testing_Tuning_HSIOM_SEL_AMUXA << UL_testing_Tuning_CMOD_HSIOM_SHIFT);
                CY_SET_REG32(UL_testing_Tuning_CMOD_HSIOM_PTR, newRegValue);
                CyExitCriticalSection(interruptState);

                /* Connect CMOD to (sense path) to CSDCOMP */
                #if (UL_testing_Tuning_CSD__CMOD_PAD == UL_testing_Tuning_CMOD_CONNECTION)
                    CY_SET_REG32(UL_testing_Tuning_SW_CMP_P_SEL_PTR, UL_testing_Tuning_SW_CMP_P_SEL_SW_SFPM_STATIC_CLOSE);
                #elif (UL_testing_Tuning_CSD__CSHIELD_PAD == UL_testing_Tuning_CMOD_CONNECTION)
                    CY_SET_REG32(UL_testing_Tuning_SW_CMP_P_SEL_PTR, UL_testing_Tuning_SW_CMP_P_SEL_SW_SFPS_STATIC_CLOSE);
                #else
                    CY_SET_REG32(UL_testing_Tuning_SW_CMP_P_SEL_PTR, UL_testing_Tuning_SW_CMP_P_SEL_SW_SFPT_STATIC_CLOSE);
                #endif /* (UL_testing_Tuning_CSD__CMOD_PAD == UL_testing_Tuning_CMOD_CONNECTION) */
            #else
                interruptState = CyEnterCriticalSection();
                newRegValue = CY_GET_REG32(UL_testing_Tuning_CintA_HSIOM_PTR);
                newRegValue &= (uint32)(~(uint32)UL_testing_Tuning_CintA_HSIOM_MASK);
                newRegValue |= (UL_testing_Tuning_HSIOM_SEL_AMUXA << UL_testing_Tuning_CintA_HSIOM_SHIFT);
                CY_SET_REG32(UL_testing_Tuning_CintA_HSIOM_PTR, newRegValue);
                CyExitCriticalSection(interruptState);

                interruptState = CyEnterCriticalSection();
                newRegValue = CY_GET_REG32(UL_testing_Tuning_CintB_HSIOM_PTR);
                newRegValue &= (uint32)(~(uint32)UL_testing_Tuning_CintB_HSIOM_MASK);
                newRegValue |= (UL_testing_Tuning_HSIOM_SEL_AMUXA << UL_testing_Tuning_CintB_HSIOM_SHIFT);
                CY_SET_REG32(UL_testing_Tuning_CintB_HSIOM_PTR, newRegValue);
                CyExitCriticalSection(interruptState);

                CY_SET_REG32(UL_testing_Tuning_SW_CMP_P_SEL_PTR, UL_testing_Tuning_SW_CMP_P_SEL_SW_SFPT_STATIC_CLOSE);
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN) */
        }

        /* Verification of the bist mode */
        if (UL_testing_Tuning_BIST_HW_SNS_CAP_CSD_E == UL_testing_Tuning_bistHwConfig)
        {
            /* Configure shield driving path */
            #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN)
                /* Connect AMUXBUS-B to CSDBUS-B (and AMUXBUS-A to CSDBUS-A ) */
            CY_SET_REG32(UL_testing_Tuning_SW_BYP_SEL_PTR, UL_testing_Tuning_SW_BYP_SEL_SW_BYA_MASK |
                                                          UL_testing_Tuning_SW_BYP_SEL_SW_BYB_MASK);
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) */
        }

        /* Connect VREF to REFGEN (IAIB switch is open) */
        CY_SET_REG32(UL_testing_Tuning_SW_REFGEN_SEL_PTR, UL_testing_Tuning_BIST_SW_REFGEN_SEL_CFG);

        #if (UL_testing_Tuning_2000_MV > UL_testing_Tuning_CYDEV_VDDA_MV)

            /* Connect VREFHI to CSDCOMP */
            CY_SET_REG32(UL_testing_Tuning_SW_CMP_N_SEL_PTR, UL_testing_Tuning_SW_CMP_N_SEL_SW_SCRH_STATIC_CLOSE);

            /* Configure REFGEN. Set reference voltage when Vdda < 2 V */
            CY_SET_REG32(UL_testing_Tuning_REFGEN_PTR, UL_testing_Tuning_REFGEN_LV);

            #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN)
                CY_SET_REG32(UL_testing_Tuning_AMBUF_PTR, UL_testing_Tuning_AMBUF_PWR_MODE_NORM);
                CY_SET_REG32(UL_testing_Tuning_SW_AMUXBUF_SEL_PTR, UL_testing_Tuning_SW_AMUXBUF_SEL_SW_IRH_STATIC_CLOSE |
                                                                  UL_testing_Tuning_SW_AMUXBUF_SEL_SW_IRLB_STATIC_CLOSE |
                                                                  UL_testing_Tuning_SW_AMUXBUF_SEL_SW_ICB_PHI2);
            #else
                CY_SET_REG32(UL_testing_Tuning_AMBUF_PTR, UL_testing_Tuning_AMBUF_PWR_MODE_NORM);
                CY_SET_REG32(UL_testing_Tuning_SW_AMUXBUF_SEL_PTR, UL_testing_Tuning_SW_AMUXBUF_SEL_SW_IRH_STATIC_CLOSE |
                                                                  UL_testing_Tuning_SW_AMUXBUF_SEL_SW_IRLB_STATIC_CLOSE);
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) */
        #else

            /* Connect VREFHI (from RefGen) to CSDCOMP when Vdda >= 2 V */
            CY_SET_REG32(UL_testing_Tuning_SW_CMP_N_SEL_PTR, UL_testing_Tuning_SW_CMP_N_SEL_SW_SCRH_STATIC_CLOSE);

            /* Configure REFGEN. Set reference voltage when Vdda >= 2 V */
            CY_SET_REG32(UL_testing_Tuning_REFGEN_PTR, UL_testing_Tuning_REFGEN_HV);

            #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN)
                /* Turn on CSD_AMBUF high power level when Vdaa >= 2V */
                CY_SET_REG32(UL_testing_Tuning_AMBUF_PTR, UL_testing_Tuning_AMBUF_PWR_MODE_HI);
                CY_SET_REG32(UL_testing_Tuning_SW_AMUXBUF_SEL_PTR, UL_testing_Tuning_SW_AMUXBUF_SEL_SW_IRH_STATIC_CLOSE |
                                                                  UL_testing_Tuning_SW_AMUXBUF_SEL_SW_ICB_PHI2);
            #else

                CY_SET_REG32(UL_testing_Tuning_AMBUF_PTR, UL_testing_Tuning_AMBUF_PWR_MODE_OFF);

                /* Configure REFGEN. Set reference voltage when Vdda >= 2 V */
                CY_SET_REG32(UL_testing_Tuning_SW_AMUXBUF_SEL_PTR, UL_testing_Tuning_SW_AMUXBUF_SEL_SW_DEFAULT);
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) */
        #endif /* (UL_testing_Tuning_2000_MV > UL_testing_Tuning_CYDEV_VDDA_MV) */

        if (UL_testing_Tuning_BIST_HW_EXT_CAP_E != UL_testing_Tuning_bistHwConfig)
        {
            /* Configure ModClk */
            UL_testing_Tuning_SsSetModClkClockDivider(UL_testing_Tuning_TST_MOD_CLK_DIVIDER);
        }

        /* Verification of the bist mode */
        if (UL_testing_Tuning_BIST_HW_SNS_CAP_CSD_E == UL_testing_Tuning_bistHwConfig)
        {
            #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN)
                /* Update CSD config register shield delay bits with shield delay value */
                UL_testing_Tuning_bistConfigCsd &= (uint32)(~UL_testing_Tuning_CONFIG_SHIELD_DELAY_MASK);
                UL_testing_Tuning_bistConfigCsd |= ((uint32)UL_testing_Tuning_CSD_SHIELD_DELAY <<
                                                                       CYFLD_CSD_SHIELD_DELAY__OFFSET);
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) */
        }

        /* Configure HW block filter delay */
        UL_testing_Tuning_bistConfigCsd &= (uint32)(~UL_testing_Tuning_CONFIG_FILTER_DELAY_MASK);
        UL_testing_Tuning_bistConfigCsd |= UL_testing_Tuning_FILTER_DELAY_CFG;

        /* Enable power to sub-blocks */
        CY_SET_REG32(UL_testing_Tuning_CONFIG_PTR,     UL_testing_Tuning_bistConfigCsd |
                                                      UL_testing_Tuning_CONFIG_SENSE_EN_MASK |
                                                      UL_testing_Tuning_CONFIG_ENABLE_MASK);

        #if(UL_testing_Tuning_TST_ANALOG_STARTUP_DELAY_US > 0uL)
            CyDelayUs(UL_testing_Tuning_TST_ANALOG_STARTUP_DELAY_US);
        #endif /* (UL_testing_Tuning_TST_ANALOG_STARTUP_DELAY_US > 0uL) */

        /* Set the number of dummy fine initialization cycles depending of the bist mode */
        /* Set Phi1 and Phi2 switches to non-overlapping (no low EMI) and width depending of the bist mode */
        if (UL_testing_Tuning_BIST_HW_EXT_CAP_E != UL_testing_Tuning_bistHwConfig)
        {
            CY_SET_REG32(UL_testing_Tuning_SEQ_INIT_CNT_PTR, UL_testing_Tuning_TST_FINE_INIT_TIME);
            CY_SET_REG32(UL_testing_Tuning_SENSE_DUTY_PTR, UL_testing_Tuning_SENSE_DUTY_OVERLAP_PHI1_MASK |
                                                          UL_testing_Tuning_SENSE_DUTY_OVERLAP_PHI2_MASK);
        }
        else
        {

            CY_SET_REG32(UL_testing_Tuning_SEQ_INIT_CNT_PTR, UL_testing_Tuning_SEQ_INIT_CNT_FINE_INIT_SKIP);
            CY_SET_REG32(UL_testing_Tuning_SENSE_DUTY_PTR, UL_testing_Tuning_SENSE_DUTY_OVERLAP_PHI1_MASK |
                                                          UL_testing_Tuning_SENSE_DUTY_OVERLAP_PHI2_MASK |
                                                          UL_testing_Tuning_SENSE_DUTY_TST_EXT_CAP_WIDTH );
        }
    #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2) */

    #if (UL_testing_Tuning_DISABLE == UL_testing_Tuning_CSDV2)

        if (UL_testing_Tuning_BIST_HW_EXT_CAP_E != UL_testing_Tuning_bistHwConfig)
        {
            #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN)
                /* Connect Cmod to AMUXBUS-A using the HSIOM registers */
                interruptState = CyEnterCriticalSection();
                newRegValue = CY_GET_REG32(UL_testing_Tuning_CMOD_HSIOM_PTR);
                newRegValue &= (uint32)(~(uint32)UL_testing_Tuning_CMOD_HSIOM_MASK);
                newRegValue |= (UL_testing_Tuning_HSIOM_SEL_AMUXA << UL_testing_Tuning_CMOD_HSIOM_SHIFT);
                CY_SET_REG32(UL_testing_Tuning_CMOD_HSIOM_PTR, newRegValue);
                CyExitCriticalSection(interruptState);
            #else
                interruptState = CyEnterCriticalSection();
                newRegValue = CY_GET_REG32(UL_testing_Tuning_CintB_HSIOM_PTR);
                newRegValue &= ~UL_testing_Tuning_CintB_HSIOM_MASK;
                newRegValue |= (UL_testing_Tuning_HSIOM_SEL_AMUXA << UL_testing_Tuning_CintB_HSIOM_SHIFT);
                CY_SET_REG32(UL_testing_Tuning_CintB_HSIOM_PTR, newRegValue);
                CyExitCriticalSection(interruptState);
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN) */
        }
        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN)
            /* Verification of the BIST mode */
            if (UL_testing_Tuning_BIST_HW_SNS_CAP_CSD_E == UL_testing_Tuning_bistHwConfig)
            {
                /* Update CSD config register shield delay bits with a shield delay value */
                UL_testing_Tuning_bistConfigCsd &= (uint32)(~UL_testing_Tuning_CONFIG_SHIELD_DELAY_MASK);
                UL_testing_Tuning_bistConfigCsd |= ((uint32)UL_testing_Tuning_CSD_SHIELD_DELAY << UL_testing_Tuning_SHIELD_DELAY_SHIFT);
            }
        #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) */

        if (UL_testing_Tuning_BIST_HW_EXT_CAP_E != UL_testing_Tuning_bistHwConfig)
        {
            /* Enable the CSD block. Connect Vref buffer to AMUX bus to make sure that Cmod is charged before scanning in the active mode */
            #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_BLOCK_OFF_AFTER_SCAN_EN)
                CY_SET_REG32(UL_testing_Tuning_CONFIG_PTR, UL_testing_Tuning_bistConfigCsd);
            #else
                CY_SET_REG32(UL_testing_Tuning_CONFIG_PTR, UL_testing_Tuning_bistConfigCsd | UL_testing_Tuning_BIST_CTANK_PRECHARGE_CONFIG_CSD_EN);
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_BLOCK_OFF_AFTER_SCAN_EN) */
        }
    #endif /* (UL_testing_Tuning_DISABLE == UL_testing_Tuning_CSDV2) */

    /* Configure IDAC */
    UL_testing_Tuning_BistSetIdac(0uL);

    if (UL_testing_Tuning_BIST_HW_EXT_CAP_E != UL_testing_Tuning_bistHwConfig)
    {
        /* Set clock dividers and clock source mode */
        UL_testing_Tuning_BistConfigClock(UL_testing_Tuning_TST_BASE_SNS_CLK_DIVIDER);
    }
    else
    {
        #if (UL_testing_Tuning_DISABLE == UL_testing_Tuning_CSDV2)
            /* Enable the CSD block. Connect Vref buffer to AMUX bus to make sure that Cmod is charged before scanning in the active mode */
            CY_SET_REG32(UL_testing_Tuning_CONFIG_PTR, UL_testing_Tuning_bistConfigCsd | UL_testing_Tuning_CONFIG_ENABLE_MASK);
        #endif /* (UL_testing_Tuning_DISABLE == UL_testing_Tuning_CSDV2) */
    }
    UL_testing_Tuning_ISR_StartEx(&UL_testing_Tuning_BistPostSingleScan);
}

/*******************************************************************************
* Function Name: UL_testing_Tuning_BistConfigClock
****************************************************************************//**
*
* \brief
*   Sets the sensor clock frequency for BIST scans
*
* \details
*   For a sensor or shield capasitance measurement sets a calculated value of
*   the sensor clock frequency and for an external capacitance measurement sets
*   the value for a one period scan.
*
*******************************************************************************/
static void UL_testing_Tuning_BistConfigClock(uint32 snsClk)
{
    #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2)
        uint32 newRegValue;

        /*
         * Set divider value for sense clock.
         * 1u is subtracted from snsClk because SENSE_DIV value 0 corresponds
         * to dividing by 1.
         */
        newRegValue = (snsClk - 1u) & UL_testing_Tuning_SENSE_PERIOD_SENSE_DIV_MASK;
        CY_SET_REG32(UL_testing_Tuning_SENSE_PERIOD_PTR, newRegValue);
    #else
        /* Set a sense clock divider */
        UL_testing_Tuning_SsSetSnsClockDivider(snsClk);
    #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2) */
}

/*******************************************************************************
* Function Name: UL_testing_Tuning_BistSetScanDuration
****************************************************************************//**
*
* \brief
*   Calculates Scan Duration which is defined by scan resolution
*
* \details
*   For Fourth-generation HW block: The function calculates the number of conversions and updates the
*   SEQ_NORM_CNT register. The number of conversions depends on the resolution and
*   snsClk divider.
*   For Third-generation HW block: The function recalculates the resolution using the following equation:
*   2^resolution - 1. The calculated value is contained in the
*   UL_testing_Tuning_bistCounterResolution global variable and used in the
*   UL_testing_Tuning_bistStartSample() function to trigger a scan process.
*
*******************************************************************************/
static void UL_testing_Tuning_BistSetScanDuration(void)
{
    uint32 scanDuration;

    if (UL_testing_Tuning_bistHwConfig == UL_testing_Tuning_BIST_HW_EXT_CAP_E)
    {
        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2)
            scanDuration = UL_testing_Tuning_TST_EXT_CAP_DURATION;
        #else
            scanDuration = UL_testing_Tuning_BIST_SCAN_DURATION_255;
        #endif
    }
    else
    {
        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2)
            /* Calculate scanning resolution value in register */
            scanDuration = (uint32) (UL_testing_Tuning_BIST_10_BIT_MASK / UL_testing_Tuning_TST_BASE_SNS_CLK_DIVIDER);
        #else
            scanDuration = UL_testing_Tuning_BIST_SCAN_DURATION;
        #endif
    }

    #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2)
        /* Set Number Of Conversions based on scanning resolution */
        CY_SET_REG32(UL_testing_Tuning_SEQ_NORM_CNT_PTR, scanDuration & UL_testing_Tuning_SEQ_NORM_CNT_CONV_CNT_MASK);
    #else
        /* Set up a scanning resolution */
        UL_testing_Tuning_bistCounterResolution = scanDuration;
    #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2) */
}

/*******************************************************************************
* Function Name: UL_testing_Tuning_BistSetIdac
****************************************************************************//**
*
* \brief
*   This internal function changes the IDACs values for BIST scan.
*
* \details
*   The functions disables the compensation IDAC if enabled and sets
*   desired Idac value.
*
* \param idacValue
*   The Idac code to be set.
*
*******************************************************************************/
static void UL_testing_Tuning_BistSetIdac(uint32 idacValue)
{
    #if ((UL_testing_Tuning_ENABLE != UL_testing_Tuning_CSDV2) && \
        (UL_testing_Tuning_ENABLE != UL_testing_Tuning_CSD_IDAC_COMP_EN))
        uint32 regValue;
        uint8  interruptState;
    #endif

    #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2)
        CY_SET_REG32(UL_testing_Tuning_IDAC_MOD_PTR, UL_testing_Tuning_BIST_IDACA_CFG | (idacValue & UL_testing_Tuning_IDAC_MOD_VAL_MASK));
        #if((UL_testing_Tuning_ENABLE == UL_testing_Tuning_ADC_EN) || (UL_testing_Tuning_ENABLE == UL_testing_Tuning_IDACB_USED))
            CY_SET_REG32(UL_testing_Tuning_IDAC_COMP_PTR, UL_testing_Tuning_BIST_IDACB_CFG);
        #endif
    #else
        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_IDAC_COMP_EN)
            CY_SET_REG32(UL_testing_Tuning_IDAC_PTR, UL_testing_Tuning_BIST_IDAC_CFG | (idacValue & UL_testing_Tuning_BIST_IDAC_MOD_VAL_MASK));
        #else
            interruptState = CyEnterCriticalSection();
            regValue = CY_GET_REG32(UL_testing_Tuning_IDAC_PTR);
            regValue &= ~(UL_testing_Tuning_BIST_IDAC_MOD_MASK);
            regValue |= (UL_testing_Tuning_BIST_IDAC_CFG | (idacValue & UL_testing_Tuning_BIST_IDAC_MOD_VAL_MASK));
            CY_SET_REG32(UL_testing_Tuning_IDAC_PTR, regValue);
            CyExitCriticalSection(interruptState);
        #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_IDAC_COMP_EN) */
    #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2) */
}

/*******************************************************************************
* Function Name: UL_testing_Tuning_BistCmodPrecharge
****************************************************************************//**
*
* \brief
*  This function initializes the Cmod charging to Vref.
*
* \details
*  For Third-generation HW block:
*  The function initializes the Cmod charging to Vref.
*  Then it waits until Cmod is completely charged
*  to Vref to have stable raw counts. The Software Watchdog Counter is implemented to
*  prevent project hanging.
*
*  For Fourth-generation HW block:
*  Coarse initialization for CMOD and Cch.
*  The coarse initialization is performed by HSCOMP.
*  The HSCOMP monitors the Cmod voltage via the Cmod sense path
*  and charges the Cmod using the HCAV switch via CSDBUS-A, AMUXBUS-A
*  and static connection of Cmod to AMUXBUS-A.
*
*******************************************************************************/
static void UL_testing_Tuning_BistCmodPrecharge(void)
{
    #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2)
        /* Fourth-generation HW block section */

        volatile uint32 watchdogCounter;
        uint32 nonVolWDC;
        
        #if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) && (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN))
            #if(0u != UL_testing_Tuning_CSD_SHIELD_TANK_EN)
                uint32 tmpRegValue;
            #endif
        #endif

        CY_SET_REG32(UL_testing_Tuning_CSDCMP_PTR, UL_testing_Tuning_BIST_CSDCMP_INIT);
        CY_SET_REG32(UL_testing_Tuning_SW_FW_MOD_SEL_PTR,  UL_testing_Tuning_BIST_SW_FW_MOD_SEL_INIT);
        CY_SET_REG32(UL_testing_Tuning_SW_FW_TANK_SEL_PTR, UL_testing_Tuning_BIST_SW_FW_TANK_SEL_INIT);
        CY_SET_REG32(UL_testing_Tuning_SW_SHIELD_SEL_PTR,  UL_testing_Tuning_BIST_SW_SHIELD_SEL_INIT);

        /* Connect CMOD to (sense path) to HSCOMP: HMPM or HMPS or HMPT switches depend on Cmod connects to certain pad */
        CY_SET_REG32(UL_testing_Tuning_SW_HS_P_SEL_PTR, UL_testing_Tuning_BIST_SW_HS_P_SEL_COARSE);
        CY_SET_REG32(UL_testing_Tuning_SW_HS_N_SEL_PTR, UL_testing_Tuning_SW_HS_N_SEL_SW_HCRH_STATIC_CLOSE);

        #if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) && (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN))
            #if(UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_TANK_EN)
                /* Verification of the bist mode */
                if (UL_testing_Tuning_BIST_HW_SNS_CAP_CSD_E == UL_testing_Tuning_bistHwConfig)
                {
                    /* Connect CTANK to AMUXBUS-A */
                    tmpRegValue = CY_GET_REG32(UL_testing_Tuning_CSH_HSIOM_PTR);
                    tmpRegValue &= (uint32)(~(uint32)UL_testing_Tuning_CSH_HSIOM_MASK);
                    tmpRegValue |= (UL_testing_Tuning_HSIOM_SEL_AMUXA << UL_testing_Tuning_CSH_HSIOM_SHIFT);
                    CY_SET_REG32(UL_testing_Tuning_CSH_HSIOM_PTR, tmpRegValue);
                }
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_TANK_EN) */
        #endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) && (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN)) */

        CY_SET_REG32(UL_testing_Tuning_SW_RES_PTR, UL_testing_Tuning_BIST_SW_RES_INIT);

        /* Clear all interrupt pending requests */
        CY_SET_REG32(UL_testing_Tuning_INTR_PTR, UL_testing_Tuning_INTR_SET_MASK);
        (void)CY_GET_REG32(UL_testing_Tuning_INTR_PTR);

        /* Unmask INIT interrupt (enable interrupt) */
        CY_SET_REG32(UL_testing_Tuning_INTR_MASK_PTR, UL_testing_Tuning_INTR_MASK_CLEAR_MASK);

        /* Start SEQUENCER for coarse initialization for Cmod */
        CY_SET_REG32(UL_testing_Tuning_SEQ_START_PTR, UL_testing_Tuning_SEQ_START_SEQ_MODE_MASK |
                                                     UL_testing_Tuning_SEQ_START_START_MASK);

        /* Init Watchdog Counter to prevent a hang */
        watchdogCounter = UL_testing_Tuning_BIST_PRECHARGE_WATCHDOG_CYCLES_NUM;
        nonVolWDC = watchdogCounter;
        /* Wait for INTR.INIT goes high */
        while((0u == (CY_GET_REG32(UL_testing_Tuning_INTR_PTR) & UL_testing_Tuning_INTR_INIT_MASK)) && (0u != nonVolWDC))
        {
            watchdogCounter--;
            nonVolWDC = watchdogCounter;
        }

        if (0u == watchdogCounter)
        {
            /* Set sequencer to idle state if coarse initialization fails */
            CY_SET_REG32(UL_testing_Tuning_SEQ_START_PTR, UL_testing_Tuning_SEQ_START_ABORT_MASK);
        }

        /* Set scanning configuration for switches */
        CY_SET_REG32(UL_testing_Tuning_SW_FW_MOD_SEL_PTR, UL_testing_Tuning_BIST_SW_FW_MOD_SEL_SCAN);
        CY_SET_REG32(UL_testing_Tuning_SW_FW_TANK_SEL_PTR, UL_testing_Tuning_BIST_SW_FW_TANK_SEL_SCAN);

    #else /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2) */

        uint32 newRegValue;
        volatile uint32 watchdogCounter;
        uint32 nonVolWDC;

        /* Save the current IDAC configuration */
        newRegValue = CY_GET_REG32(UL_testing_Tuning_IDAC_PTR);

        /* Wait until Cmod discharges below Vref */
        CyDelayCycles(UL_testing_Tuning_CMOD_DISCHARGE_CYCLES);

        /* Set the IDAC configuration for fast Cmod pre-charge to Vref */
        CY_SET_REG32(UL_testing_Tuning_IDAC_PTR, UL_testing_Tuning_CSD_IDAC_PRECHARGE_CONFIG);

        /* Initialize the watchdog counter to prevent a hang */
        watchdogCounter = UL_testing_Tuning_BIST_PRECHARGE_WATCHDOG_CYCLES_NUM;
        nonVolWDC = watchdogCounter;
        /* Wait until Cmod reaches Vref */
        while((0u == (CY_GET_REG32(UL_testing_Tuning_STATUS_PTR) & UL_testing_Tuning_STATUS_SAMPLE)) && (0u != nonVolWDC))
        {
            watchdogCounter--;
            nonVolWDC = watchdogCounter;
        }

        /* Restore the current IDAC configuration */
        CY_SET_REG32(UL_testing_Tuning_IDAC_PTR, newRegValue);

        /* Enable the sense modulator output */
        CY_SET_REG32(UL_testing_Tuning_CONFIG_PTR, UL_testing_Tuning_bistConfigCsd | UL_testing_Tuning_BIST_CTANK_PRECHARGE_CONFIG_CSD_EN | UL_testing_Tuning_CONFIG_SENSE_EN_MASK);

        CyIntEnable(UL_testing_Tuning_ISR_NUMBER);

    #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2) */
}

/*******************************************************************************
* Function Name: UL_testing_Tuning_WaitEndOfScan
****************************************************************************//**
*
* \brief
*   Waits the end of scan for capacitance measurements.
*
* \details
*   Apart the waiting end of scan the function sets a watchdog to prevent
*   a hangout
*
*******************************************************************************/
static void UL_testing_Tuning_WaitEndOfScan(uint32 timeout)
{
    volatile uint32 watchdogCounter = timeout;
    uint32 nonVolWDC = watchdogCounter;
    while (((*(volatile uint8 *)&UL_testing_Tuning_dsRam.status & UL_testing_Tuning_SW_STS_BUSY) != 0u) &&
           (0u != nonVolWDC))
    {
        watchdogCounter--;
        nonVolWDC = watchdogCounter;
    }
    (void)watchdogCounter;
}

/*******************************************************************************
* Function Name: UL_testing_Tuning_BistTriggerScan
****************************************************************************//**
*
* \brief
*  This function triggers the scanning.
*
* \details
*  For Third-generation HW block:
*  Writes a resolution to start scanning.
*
*  For Fourth-generation HW block:
*  Trigger the fine initialization (scan some dummy cycles only for sensor or
*  shield capacitance meterings) and start sampling.
*  For the fine initialization and sampling the measured capacitance is charged
*  from Idac and discharged to ground using the switches in GPIO cell.
*  The CSDCOMP monitors voltage on the measured capacitance using the sense path
*  and charges it to Vref using IDAC.
*
*******************************************************************************/
static void UL_testing_Tuning_BistTriggerScan(void)
{
    #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2)
        /* Fourth-generation HW block section */
        volatile uint32 watchdogCounter;
        uint32 nonVolWDC;
        
        /* Clear previous interrupts */
        CY_SET_REG32(UL_testing_Tuning_INTR_PTR, UL_testing_Tuning_INTR_SET_MASK);
        (void)CY_GET_REG32(UL_testing_Tuning_INTR_PTR);

        /* Clear pending interrupts  */
        CyIntClearPending(UL_testing_Tuning_ISR_NUMBER);
        /* Enable the CSD interrupt */
        CyIntEnable(UL_testing_Tuning_ISR_NUMBER);

        CY_SET_REG32(UL_testing_Tuning_SW_HS_P_SEL_PTR, UL_testing_Tuning_BIST_HS_P_SEL_SCAN);
        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS_CAP_EN)
        /* Verification of the bist mode */
            if (UL_testing_Tuning_BIST_HW_SNS_CAP_CSD_E == UL_testing_Tuning_bistHwConfig)
            {
                #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN)
                    CY_SET_REG32(UL_testing_Tuning_SW_SHIELD_SEL_PTR,  UL_testing_Tuning_BIST_SW_SHIELD_SEL_CFG);
                    #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_TANK_EN)
                        UL_testing_Tuning_BistEnableShieldTank();
                    #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_TANK_EN) */
                #else
                    CY_SET_REG32(UL_testing_Tuning_SW_SHIELD_SEL_PTR, UL_testing_Tuning_STATIC_OPEN);
                    CY_SET_REG32(UL_testing_Tuning_SW_HS_P_SEL_PTR, UL_testing_Tuning_STATIC_OPEN);
                #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) */
            }
            else
            {
                CY_SET_REG32(UL_testing_Tuning_SW_SHIELD_SEL_PTR, UL_testing_Tuning_STATIC_OPEN);
                CY_SET_REG32(UL_testing_Tuning_SW_HS_P_SEL_PTR,   UL_testing_Tuning_STATIC_OPEN);
            }
        #else
            CY_SET_REG32(UL_testing_Tuning_SW_SHIELD_SEL_PTR, UL_testing_Tuning_STATIC_OPEN);
            CY_SET_REG32(UL_testing_Tuning_SW_HS_P_SEL_PTR,   UL_testing_Tuning_STATIC_OPEN);
        #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS_CAP_EN) */

        CY_SET_REG32(UL_testing_Tuning_SW_RES_PTR, UL_testing_Tuning_BIST_SW_RES_SCAN);

        /* Enable CSDCMP */
        if (UL_testing_Tuning_BIST_HW_EXT_CAP_E == UL_testing_Tuning_bistHwConfig)
        {
            CY_SET_REG32(UL_testing_Tuning_CSDCMP_PTR,         UL_testing_Tuning_CSDCMP_TST_CAP_MEAS);
            CY_SET_REG32(UL_testing_Tuning_SW_FW_MOD_SEL_PTR,  UL_testing_Tuning_STATIC_OPEN);
            CY_SET_REG32(UL_testing_Tuning_SW_FW_TANK_SEL_PTR, UL_testing_Tuning_STATIC_OPEN);
        }
        else
        {
            CY_SET_REG32(UL_testing_Tuning_CSDCMP_PTR, UL_testing_Tuning_CSDCMP_CSDCMP_EN_MASK);
        }

    /* Verification of the bist mode */
        if (UL_testing_Tuning_BIST_HW_SNS_CAP_CSD_E == UL_testing_Tuning_bistHwConfig)
        {
            #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN)
                CY_SET_REG32(UL_testing_Tuning_HSCMP_PTR, UL_testing_Tuning_BIST_HSCMP_SCAN_MASK);
            #else
                /* Disable HSCOMP during the sampling phase when shield is disabled */
                CY_SET_REG32(UL_testing_Tuning_HSCMP_PTR, 0uL);
            #endif /* (UL_testing_Tuning_DISABLE == UL_testing_Tuning_CSD_SHIELD_EN) */
        }
        else
        {
            /* Disable HSCOMP during the sampling phase when shield is disabled */
            CY_SET_REG32(UL_testing_Tuning_HSCMP_PTR, 0uL);
        }
        /* Enable SAMPLE interrupt */
        CY_SET_REG32(UL_testing_Tuning_INTR_MASK_PTR, UL_testing_Tuning_INTR_MASK_SAMPLE_MASK);

        /* Init Watchdog Counter to prevent a hang */
        watchdogCounter = UL_testing_Tuning_BIST_PRECHARGE_WATCHDOG_CYCLES_NUM;
        nonVolWDC = watchdogCounter;
        /* Wait for IDLE state of the Fourth-generation HW block sequencer */
        while((0u != (UL_testing_Tuning_STAT_SEQ_REG & UL_testing_Tuning_STAT_SEQ_SEQ_STATE_MASK)) && (0u != nonVolWDC))
        {
            watchdogCounter--;
            nonVolWDC = watchdogCounter;
        }

        /* Start SEQUENCER for fine initialization scan for Cmod and then for normal scan */
        CY_SET_REG32(UL_testing_Tuning_SEQ_START_PTR, UL_testing_Tuning_SEQ_START_AZ0_SKIP_MASK |
                                                     UL_testing_Tuning_SEQ_START_AZ1_SKIP_MASK |
                                                     UL_testing_Tuning_SEQ_START_START_MASK);

    #else

        /* Third-generation HW block section */

        /* Set a resolution to the counter register to start scanning */
        CY_SET_REG32(UL_testing_Tuning_COUNTER_PTR, UL_testing_Tuning_bistCounterResolution);

    #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2) */
}
#endif /*(UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_MEASUREMENT_GROUP_EN) */

#if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_VDDA_EN) &&\
     (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2))
/*******************************************************************************
* Function Name: UL_testing_Tuning_GetVdda
****************************************************************************//**
*
* \brief
*  Measures and returns the VDDA voltage.
*
* \details
*  This function measures voltage on VDDA terminal of the chip and returns
*  the result, alternatively the result is stored in the
*  UL_testing_Tuning_VDDA_VOLTAGE_VALUE register of data structure.
*
* \return
*  The VDDA voltage in mV.
*
*******************************************************************************/
uint16 UL_testing_Tuning_GetVdda(void)
{
    uint16 tmpResult;

    UL_testing_Tuning_SsSwitchSensingMode(UL_testing_Tuning_SENSE_METHOD_BIST_E);
    UL_testing_Tuning_BistSwitchHwConfig(UL_testing_Tuning_BIST_HW_VDDA_E);
    tmpResult = UL_testing_Tuning_BistVddaRunMeasurement();
    UL_testing_Tuning_dsRam.selfTest.vddaVoltage = tmpResult;

    return (tmpResult);
}


/*******************************************************************************
* Function Name: UL_testing_Tuning_BistEnableVddaMeasurement
****************************************************************************//**
*
* \brief
*  Configures the CSD block to be used for VDDA measurement.
*
* \details
*  Configures the IDACB, HW block internal switches, REFGEN, HSCOMP and
*  interrupt.
*
*******************************************************************************/
static void UL_testing_Tuning_BistEnableVddaMeasurement(void)
{
    /* Configure clocks */
    UL_testing_Tuning_SsSetModClkClockDivider(UL_testing_Tuning_TST_VDDA_MODCLK_DIV_DEFAULT);
    CY_SET_REG32(UL_testing_Tuning_SENSE_PERIOD_PTR, (UL_testing_Tuning_TST_VDDA_SENSE_DIV_DEFAULT - 1uL));

    /* Configure the IDAC */
    CY_SET_REG32(UL_testing_Tuning_CONFIG_PTR, UL_testing_Tuning_TST_VDDA_CONFIG_DEFAULT_CFG);

    #if(UL_testing_Tuning_TST_ANALOG_STARTUP_DELAY_US > 0uL)
        CyDelayUs(UL_testing_Tuning_TST_ANALOG_STARTUP_DELAY_US);
    #endif /* (UL_testing_Tuning_TST_ANALOG_STARTUP_DELAY_US > 0uL) */

    CY_SET_REG32(UL_testing_Tuning_IDAC_COMP_PTR, UL_testing_Tuning_TST_VDDA_IDACB_DEFAULT_CFG);

    /* Configure AZ Time */
    CY_SET_REG32(UL_testing_Tuning_SEQ_TIME_PTR, (UL_testing_Tuning_TST_VDDA_SEQ_TIME_DEFAULT - 1u));

    CY_SET_REG32(UL_testing_Tuning_IDAC_MOD_PTR, UL_testing_Tuning_TST_VDDA_IDACA_DEFAULT_CFG);
    CY_SET_REG32(UL_testing_Tuning_CSDCMP_PTR, UL_testing_Tuning_TST_CSDCMP_DEFAULT_CFG);
    CY_SET_REG32(UL_testing_Tuning_SW_DSI_SEL_PTR, UL_testing_Tuning_TST_SW_DSI_SEL_DEFAULT_CFG);

    CY_SET_REG32(UL_testing_Tuning_SENSE_DUTY_PTR, UL_testing_Tuning_TST_SENSE_DUTY_VDDA_CFG);
    CY_SET_REG32(UL_testing_Tuning_SEQ_INIT_CNT_PTR, UL_testing_Tuning_TST_SEQ_INIT_CNT_DEFAULT_CFG);
    CY_SET_REG32(UL_testing_Tuning_SEQ_NORM_CNT_PTR, UL_testing_Tuning_TST_SEQ_NORM_CNT_DEFAULT_CFG);

    /* Configure the block-level routing */
    CY_SET_REG32(UL_testing_Tuning_SW_HS_P_SEL_PTR, UL_testing_Tuning_TST_SW_HS_P_SEL_DEFAULT_CFG);
    CY_SET_REG32(UL_testing_Tuning_SW_HS_N_SEL_PTR, UL_testing_Tuning_TST_SW_HS_N_SEL_DEFAULT_CFG);
    CY_SET_REG32(UL_testing_Tuning_SW_SHIELD_SEL_PTR, UL_testing_Tuning_TST_SW_SHIELD_SEL_DEFAULT_CFG);
    CY_SET_REG32(UL_testing_Tuning_SW_BYP_SEL_PTR, UL_testing_Tuning_TST_SW_BYP_SEL_DEFAULT_CFG);
    CY_SET_REG32(UL_testing_Tuning_SW_CMP_P_SEL_PTR, UL_testing_Tuning_TST_SW_CMP_P_SEL_DEFAULT_CFG);
    CY_SET_REG32(UL_testing_Tuning_SW_CMP_N_SEL_PTR, UL_testing_Tuning_TST_SW_CMP_N_SEL_DEFAULT_CFG);
    CY_SET_REG32(UL_testing_Tuning_SW_FW_MOD_SEL_PTR, UL_testing_Tuning_TST_SW_FW_MOD_SEL_DEFAULT_CFG);
    CY_SET_REG32(UL_testing_Tuning_SW_FW_TANK_SEL_PTR, UL_testing_Tuning_TST_SW_FW_TANK_SEL_DEFAULT_CFG);
    CY_SET_REG32(UL_testing_Tuning_SW_REFGEN_SEL_PTR, UL_testing_Tuning_TST_SW_REFGEN_SEL_DEFAULT_CFG);

    /* Config RefGen */
    CY_SET_REG32(UL_testing_Tuning_REFGEN_PTR, UL_testing_Tuning_TST_REFGEN_DEFAULT_CFG);
    CY_SET_REG32(UL_testing_Tuning_SW_AMUXBUF_SEL_PTR, UL_testing_Tuning_TST_SW_AMUXBUF_SEL_DEFAULT_CFG);

    /* Configure HSCOMP */
    CY_SET_REG32(UL_testing_Tuning_HSCMP_PTR, UL_testing_Tuning_TST_HSCMP_DEFAULT_CFG);
    CY_SET_REG32(UL_testing_Tuning_INTR_MASK_PTR, UL_testing_Tuning_INTR_MASK_CLEAR_MASK);
}


/*******************************************************************************
* Function Name: UL_testing_Tuning_BistVddaRunMeasurement
****************************************************************************//**
*
* \brief
*  This function triggers the VDDA measurement.
*
* \details
*  The function initiates a conversion, waits for completion and
*  returns the result in mV.
*  The function returns voltage in millivolts or
*  UL_testing_Tuning_TST_VDDA_BAD_RESULT if:
*   - The ADC conversion is not started
*   - The ADC conversion watch-dog triggered.
*
*******************************************************************************/
static uint16 UL_testing_Tuning_BistVddaRunMeasurement(void)
{
    uint16 tVdda2Vref;
    uint16 tVssa2Vref;
    uint16 tRecover;
    uint16 tmpResult;
    uint32 tmpVrefCal;
    uint32 registerValue;
    uint32 watchdogCounter;
    uint32 nonVolWDC;
    uint8 interruptState;
    uint8 tmpFlashByte0;
    uint8 tmpFlashByte1;

    watchdogCounter = UL_testing_Tuning_TST_SW_WATCHDOG_CYCLES_NUM;
    nonVolWDC = watchdogCounter;
    CY_SET_REG32(UL_testing_Tuning_ADC_CTL_PTR, (UL_testing_Tuning_TST_MEASMODE_VREF | (UL_testing_Tuning_VDDA_ACQUISITION_BASE - 1u)));
    CY_SET_REG32(UL_testing_Tuning_SEQ_START_PTR, UL_testing_Tuning_TST_VDDA_SEQ_START_MEASURE_CFG);

    /* Wait for INTR.INIT goes high */
    while((0u == (CY_GET_REG32(UL_testing_Tuning_INTR_PTR) & UL_testing_Tuning_INTR_ADC_RES_MASK)) && (0u != nonVolWDC))
    {
        watchdogCounter--;
        nonVolWDC = watchdogCounter;
    }

    /* Clear all interrupt pending requests */
    CY_SET_REG32(UL_testing_Tuning_INTR_PTR, UL_testing_Tuning_INTR_SET_MASK);
    (void)CY_GET_REG32(UL_testing_Tuning_INTR_PTR);

    tVssa2Vref = (uint16)CY_GET_REG32(UL_testing_Tuning_ADC_RES_PTR);

    if(0uL != watchdogCounter)
    {
        watchdogCounter = UL_testing_Tuning_TST_SW_WATCHDOG_CYCLES_NUM;
        nonVolWDC = watchdogCounter;
        CY_SET_REG32(UL_testing_Tuning_ADC_CTL_PTR, (UL_testing_Tuning_TST_MEASMODE_VREFBY2 | (UL_testing_Tuning_VDDA_ACQUISITION_BASE - 1u)));
        CY_SET_REG32(UL_testing_Tuning_SEQ_START_PTR, UL_testing_Tuning_TST_VDDA_SEQ_START_MEASURE_CFG);

        /* Wait for INTR.INIT goes high */
        while((0u == (CY_GET_REG32(UL_testing_Tuning_INTR_PTR) & UL_testing_Tuning_INTR_ADC_RES_MASK)) && (0u != nonVolWDC))
        {
            watchdogCounter--;
            nonVolWDC = watchdogCounter;
        }

        CY_SET_REG32(UL_testing_Tuning_INTR_PTR, UL_testing_Tuning_INTR_SET_MASK);
        (void)CY_GET_REG32(UL_testing_Tuning_INTR_PTR);
    }
    else
    {
        /* Do nothing. */
    }

    tRecover = (uint16)CY_GET_REG32(UL_testing_Tuning_ADC_RES_PTR);
    if(0uL != watchdogCounter)
    {
        /* Disconnect amuxbusB, Connect VDDA to csdbusB */
        interruptState = CyEnterCriticalSection();
        registerValue = CY_GET_REG32(UL_testing_Tuning_SW_BYP_SEL_PTR);
        registerValue &= (uint32)(~(UL_testing_Tuning_SW_BYP_SEL_SW_BYA_MASK | UL_testing_Tuning_SW_BYP_SEL_SW_BYB_MASK));
        CY_SET_REG32(UL_testing_Tuning_SW_BYP_SEL_PTR, registerValue);
        CyExitCriticalSection(interruptState);
        CY_SET_REG32(UL_testing_Tuning_SW_SHIELD_SEL_PTR, UL_testing_Tuning_SW_SHIELD_SEL_SW_HCBV_STATIC_CLOSE);

        watchdogCounter = UL_testing_Tuning_TST_SW_WATCHDOG_CYCLES_NUM;
        nonVolWDC = watchdogCounter;
        CY_SET_REG32(UL_testing_Tuning_ADC_CTL_PTR, (UL_testing_Tuning_TST_MEASMODE_VIN | (UL_testing_Tuning_VDDA_ACQUISITION_BASE - 1u)));
        CY_SET_REG32(UL_testing_Tuning_SEQ_START_PTR, UL_testing_Tuning_TST_VDDA_SEQ_START_MEASURE_CFG);

        /* Wait for INTR.INIT goes high */
        while((0u == (CY_GET_REG32(UL_testing_Tuning_INTR_PTR) & UL_testing_Tuning_INTR_ADC_RES_MASK)) && (0u != nonVolWDC))
        {
            watchdogCounter--;
            nonVolWDC = watchdogCounter;
        }

        CY_SET_REG32(UL_testing_Tuning_INTR_PTR, UL_testing_Tuning_INTR_SET_MASK);
        (void)CY_GET_REG32(UL_testing_Tuning_INTR_PTR);

        tVdda2Vref = (uint16)CY_GET_REG32(UL_testing_Tuning_ADC_RES_PTR);
        /* Test whether the full range is 2-bits higher than adc resolution */

        tVdda2Vref = (uint16)(((((uint32)tVdda2Vref << 1u) * tRecover) + ((uint32)tVssa2Vref >> 1u)) / tVssa2Vref);

        /* Get Vref trim-value */
        tmpFlashByte0 = CY_GET_REG8(CYREG_SFLASH_CSDV2_CSD0_ADC_TRIM1);
        tmpFlashByte1 = CY_GET_REG8(CYREG_SFLASH_CSDV2_CSD0_ADC_TRIM2);

        tmpVrefCal = (uint32)tmpFlashByte0 | (((uint32)tmpFlashByte1) << 8u);

        /* Update nominal Vref to real Vref */
        tmpVrefCal *= UL_testing_Tuning_TST_VDDA_VREF_MV;
        tmpVrefCal /= UL_testing_Tuning_TST_VDDA_VREF_CALIB_USED;

        tmpResult = (uint16)tmpVrefCal;

        /* Calculate deviation of trim register */
        if (tmpVrefCal > UL_testing_Tuning_TST_VDDA_VREF_MV)
        {
            tmpVrefCal = tmpVrefCal - UL_testing_Tuning_TST_VDDA_VREF_MV;
        }
        else
        {
            tmpVrefCal = UL_testing_Tuning_TST_VDDA_VREF_MV - tmpVrefCal;
        }
        tmpVrefCal = (tmpVrefCal * UL_testing_Tuning_PERCENTAGE_100) / UL_testing_Tuning_TST_VDDA_VREF_MV;

        /* Use nominal Vref if trim-value is not within the allowed range */
        if (UL_testing_Tuning_TST_VDDA_VREF_TRIM_MAX_DEVIATION < tmpVrefCal)
        {
            tmpResult = UL_testing_Tuning_TST_VDDA_VREF_MV;
        }

        tmpResult = tmpResult + (((tmpResult * tVdda2Vref) + (tVssa2Vref >> 1uL)) / tVssa2Vref);
    }
    else
    {
        tmpResult = UL_testing_Tuning_TST_VDDA_BAD_RESULT;
    }

    return(tmpResult);
}

#endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_VDDA_EN) &&\
           (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2)) */

#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_HW_GROUP_EN)

/*******************************************************************************
* Function Name: UL_testing_Tuning_BistSwitchHwConfig
****************************************************************************//**
*
* \brief
*   The function switches HW configuration.
*
* \details
*   The function checks the current HW configuration of UL_testing_Tuning block.
*   If it differs from a desired configuration the function disables the current
*   configuration and sets the desired one.
*
* \param config
*   Specifies desired configuration.
*
*******************************************************************************/
static void UL_testing_Tuning_BistSwitchHwConfig(UL_testing_Tuning_BIST_HW_CONFIG_ENUM config)
{
    if (UL_testing_Tuning_bistHwConfig != config)
    {
        /* The requested config differes to the current one. Disable the current config */
        switch(UL_testing_Tuning_bistHwConfig)
        {
            #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS_SHORT_EN)
                case UL_testing_Tuning_BIST_HW_SNS_SHORT_E:
                {
                    /* Nothing to do */
                    break;
                }
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS_SHORT_EN) */
            #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS2SNS_SHORT_EN)
                case UL_testing_Tuning_BIST_HW_S2S_SHORT_E:
                {
                    /* Nothing to do */
                    break;
                }
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS2SNS_SHORT_EN) */
            #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS_CAP_EN)
                case UL_testing_Tuning_BIST_HW_SNS_CAP_CSD_E:
                {
                    UL_testing_Tuning_BistDisableHwSnsCap();
                    break;
                }
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS_CAP_EN) */
            #if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS_CAP_EN) && \
                 (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN))
                case UL_testing_Tuning_BIST_HW_SNS_CAP_CSX_E:
                {
                    UL_testing_Tuning_BistDisableHwSnsCap();
                    break;
                }
            #endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS_CAP_EN) && \
                       (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN)) */
            #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SH_CAP_EN)
                case UL_testing_Tuning_BIST_HW_SH_CAP_E:
                {
                    UL_testing_Tuning_BistDisableHwSnsCap();
                    break;
                }
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SH_CAP_EN) */
            #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_EXTERNAL_CAP_EN)
                case UL_testing_Tuning_BIST_HW_EXT_CAP_E:
                {
                    /* Nothing to do */
                    break;
                }
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_EXTERNAL_CAP_EN) */
            #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_VDDA_EN)
                case UL_testing_Tuning_BIST_HW_VDDA_E:
                {
                    /* Nothing to do */
                    break;
                }
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_VDDA_EN) */
            default:
            {
                break;
            }
        }

        UL_testing_Tuning_bistHwConfig = config;

        /* Enable the specified mode */
        switch(config)
        {
            #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS_SHORT_EN)
                case UL_testing_Tuning_BIST_HW_SNS_SHORT_E:
                {
                    /* No HW CSD Block changes required */
                    UL_testing_Tuning_SwitchAllIOState(UL_testing_Tuning_BIST_IO_HIGHZA_E);
                    break;
                }
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS_SHORT_EN) */
            #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS2SNS_SHORT_EN)
                case UL_testing_Tuning_BIST_HW_S2S_SHORT_E:
                {
                    /* No HW CSD Block changes required */
                    UL_testing_Tuning_SwitchAllIOState(UL_testing_Tuning_BIST_IO_STRONG_E);
                    break;
                }
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS2SNS_SHORT_EN) */
            #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS_CAP_EN)
                case UL_testing_Tuning_BIST_HW_SNS_CAP_CSD_E:
                {
                    UL_testing_Tuning_BistEnableHwSnsCap();
                    UL_testing_Tuning_SwitchAllIOState(UL_testing_Tuning_BIST_IO_CSD_E);
                    break;
                }
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS_CAP_EN) */
            #if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS_CAP_EN) && \
                 (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN))
                case UL_testing_Tuning_BIST_HW_SNS_CAP_CSX_E:
                {
                    UL_testing_Tuning_BistEnableHwSnsCap();
                    UL_testing_Tuning_SwitchAllIOState(UL_testing_Tuning_BIST_IO_STRONG_E);
                    break;
                }
            #endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS_CAP_EN) && \
                       (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN)) */
            #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SH_CAP_EN)
                case UL_testing_Tuning_BIST_HW_SH_CAP_E:
                {
                    UL_testing_Tuning_BistEnableHwSnsCap();
                    UL_testing_Tuning_SwitchAllIOState(UL_testing_Tuning_BIST_IO_SHIELD_E);
                    break;
                }
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SH_CAP_EN) */
            #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_EXTERNAL_CAP_EN)
                case UL_testing_Tuning_BIST_HW_EXT_CAP_E:
                {
                    UL_testing_Tuning_BistEnableHwSnsCap();
                    UL_testing_Tuning_SwitchAllIOState(UL_testing_Tuning_BIST_IO_HIGHZA_E);
                    break;
                }
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_EXTERNAL_CAP_EN) */
            #if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_VDDA_EN) &&\
                 (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2))
                case UL_testing_Tuning_BIST_HW_VDDA_E:
                {
                    UL_testing_Tuning_BistEnableVddaMeasurement();
                    break;
                }
            #endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_VDDA_EN) &&\
                       (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2)) */
            default:
            {
                break;
            }
        }
    }
}

/*******************************************************************************
* Function Name: UL_testing_Tuning_SwitchAllIOState
****************************************************************************//**
*
* \brief
*   Sets all the sensors pins in a desired state.
*
* \details
*   Sets all the sensors and shield (if defined) pins in a desired state.
*
* \param config
*   Specifies desired IO configuration.
*
*******************************************************************************/
static void UL_testing_Tuning_SwitchAllIOState(UL_testing_Tuning_BIST_IO_CONFIG_ENUM config)
{
    /* Re-configure the IOs if mode is changed */
    if (UL_testing_Tuning_bistIoConfig != config)
    {
        switch(config)
        {
            case UL_testing_Tuning_BIST_IO_STRONG_E:
            {
                UL_testing_Tuning_SetAllIOPcState(UL_testing_Tuning_TST_PC_STRONG);
                break;
            }
            case UL_testing_Tuning_BIST_IO_HIGHZA_E:
            {
                UL_testing_Tuning_SetAllIOPcState(UL_testing_Tuning_TST_PC_HIGHZA);
                break;
            }
            #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS_CAP_EN)
                case UL_testing_Tuning_BIST_IO_CSD_E:
                {
                    /* Configure CSX sensors to Strong */
                    #if (0u != UL_testing_Tuning_CSX_EN)
                        /* Set all IO to strong (including CSX IOs). It is faster than loop through only CSX sensors */
                        UL_testing_Tuning_SetAllIOPcState(UL_testing_Tuning_TST_PC_STRONG);
                    #endif

                    #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN)
                        /* Reconfigure only CSD sensors to required state */
                        UL_testing_Tuning_BistSetAllInactiveSensorConnection();
                    #endif

                    break;
                }
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS_CAP_EN) */
            #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SH_CAP_EN)
                case UL_testing_Tuning_BIST_IO_SHIELD_E:
                {
                    /* Configure CSX sensors to Strong */
                    #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN)
                        /* Set all IO to strong (including CSX IOs). It is faster than loop through only CSX sensors */
                        UL_testing_Tuning_SetAllIOPcState(UL_testing_Tuning_TST_PC_STRONG);
                    #endif
                        /* Reconfigure only CSD sensors to required state */
                        UL_testing_Tuning_BistSetAllInactiveSensorConnection();
                    break;
                }
            #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SH_CAP_EN) */
            default:
            {
                break;
            }
        }
        /* Save the configured state */
        UL_testing_Tuning_bistIoConfig = config;
    }
}

/*******************************************************************************
* Function Name: UL_testing_Tuning_SetElectrodeDr
****************************************************************************//**
*
* \brief
*  The internal function that sets a certain electrode port output data.
*
* \details
*  The function sets an electrode port output data register (DR) in a demand state.
*
* \param *ioPtr
*  A pointer to the specified electrode in the flash IO structure.
*
* \param value
*  A port output data which will be setted for an electrode.
*
*******************************************************************************/
static void UL_testing_Tuning_SetElectrodeDr(UL_testing_Tuning_FLASH_IO_STRUCT const *ioPtr, uint32 value)
{
    uint32 regValue;
    uint8  interruptState;

    /* Set a data register */
    interruptState = CyEnterCriticalSection();
    regValue = CY_GET_REG32(ioPtr->drPtr);
    regValue &= ~(ioPtr->mask);
    regValue |= value << ioPtr->drShift;
    CY_SET_REG32(ioPtr->drPtr, regValue);
    CyExitCriticalSection(interruptState);
}

/*******************************************************************************
* Function Name: UL_testing_Tuning_SetElectrodePc
****************************************************************************//**
*
* \brief
*  The internal function that sets a certain electrode drive mode.
*
* \details
*  The function sets an electrode drive mode (PC register) in a demand state.
*
* \param *ioPtr
*  A pointer to the specified electrode in the flash IO structure.
*
* \param value
*  A drive mode which will be setted for an electrode.
*
*******************************************************************************/
static void UL_testing_Tuning_SetElectrodePc(UL_testing_Tuning_FLASH_IO_STRUCT const *ioPtr, uint32 value)
{
    uint32 regValue;
    uint8  interruptState;

    /* Update port configuration register (drive mode) */
    interruptState = CyEnterCriticalSection();
    regValue = CY_GET_REG32(ioPtr->pcPtr);
    regValue &= ~(UL_testing_Tuning_GPIO_PC_MASK << ioPtr->shift);
    regValue |= value << ioPtr->shift;
    CY_SET_REG32(ioPtr->pcPtr, regValue);
    CyExitCriticalSection(interruptState);
}

#if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) && \
    (UL_testing_Tuning_SNS_CONNECTION_SHIELD == UL_testing_Tuning_CSD_INACTIVE_SNS_CONNECTION))

/*******************************************************************************
* Function Name: UL_testing_Tuning_SetAllIOHsiomState
****************************************************************************//**
*
* \brief
*  The internal function that sets all pins in a demand HSIOM mode.
*
* \details
*  The function sets all pins to a demand HSIOM mode.
*
* \param value
*  A demand HSIOM mode.
*
*******************************************************************************/
static void UL_testing_Tuning_SetAllIOHsiomState(uint32 value)
{
    uint32 loopIndex;
    UL_testing_Tuning_FLASH_IO_STRUCT const *ioPtr = &UL_testing_Tuning_ioList[0u];

    /* Loop through all electrodes */
    for (loopIndex = 0u; loopIndex < UL_testing_Tuning_TOTAL_ELECTRODES; loopIndex++)
    {
        UL_testing_Tuning_SetElectrodeHsiom(ioPtr, value);
        ioPtr++;
    }
}
#endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) && \
          (UL_testing_Tuning_SNS_CONNECTION_SHIELD == UL_testing_Tuning_CSD_INACTIVE_SNS_CONNECTION)) */

/*******************************************************************************
* Function Name: UL_testing_Tuning_SetAllIODrState
****************************************************************************//**
*
* \brief
*  The internal function that sets a specific port output data for all electrodes.
*
* \details
*  The function sets a specific port output data register (DR) in a demand state for all electrodes.
*
* \param value
*  A port output data which will be set for all electrodes.
*
*******************************************************************************/
static void UL_testing_Tuning_SetAllIODrState(uint32 value)
{
    uint32 loopIndex;
    UL_testing_Tuning_FLASH_IO_STRUCT const *ioPtr = &UL_testing_Tuning_ioList[0u];

    /* Loop through all electrodes */
    for (loopIndex = 0u; loopIndex < UL_testing_Tuning_TOTAL_ELECTRODES; loopIndex++)
    {
        UL_testing_Tuning_SetElectrodeDr(ioPtr, value);
        ioPtr++;
    }
}

/*******************************************************************************
* Function Name: UL_testing_Tuning_SetAllIOPcState
****************************************************************************//**
*
* \brief
*  The internal function that sets a specific drive mode for all electrodes.
*
* \details
*  The function sets a specific drive mode (PC register) in a demand state for all electrodes.
*
* \param value
*  A drive mode which will be set for all electrodes.
*
*******************************************************************************/
static void UL_testing_Tuning_SetAllIOPcState(uint32 value)
{
    uint32 loopIndex;
    UL_testing_Tuning_FLASH_IO_STRUCT const *ioPtr = &UL_testing_Tuning_ioList[0u];

    /* Loop through all electrodes */
    for (loopIndex = 0u; loopIndex < UL_testing_Tuning_TOTAL_ELECTRODES; loopIndex++)
    {
        UL_testing_Tuning_SetElectrodePc(ioPtr, value);
        ioPtr++;
    }
}

#endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_HW_GROUP_EN) */

#if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS_CAP_EN) || \
     (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SH_CAP_EN) || \
    ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) && \
     (UL_testing_Tuning_SNS_CONNECTION_SHIELD == UL_testing_Tuning_CSD_INACTIVE_SNS_CONNECTION)))

/*******************************************************************************
* Function Name: UL_testing_Tuning_SetElectrodeHsiom
****************************************************************************//**
*
* \brief
*  The internal function that sets a certain electrode in a demand HSIOM mode.
*
* \details
*  The function sets an electrode to a demand HSIOM mode.
*
* \param *ioPtr
*  A pointer to the specified electrode in the flash IO structure.
*
* \param value
*  A demand HSIOM mode.
*
*******************************************************************************/
static void UL_testing_Tuning_SetElectrodeHsiom(UL_testing_Tuning_FLASH_IO_STRUCT const *ioPtr, uint32 value)
{
    uint32 regValue;
    uint8  interruptState;

    /* Update HSIOM port select register */
    interruptState = CyEnterCriticalSection();
    regValue = CY_GET_REG32(ioPtr->hsiomPtr);
    regValue &= ~(ioPtr->hsiomMask);
    regValue |= (value << ioPtr->hsiomShift);
    CY_SET_REG32(ioPtr->hsiomPtr, regValue);
    CyExitCriticalSection(interruptState);
}
#endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS_CAP_EN) || \
           (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SH_CAP_EN) || \
          ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) && \
           (UL_testing_Tuning_SNS_CONNECTION_SHIELD == UL_testing_Tuning_CSD_INACTIVE_SNS_CONNECTION))) */


/* [] END OF FILE */
