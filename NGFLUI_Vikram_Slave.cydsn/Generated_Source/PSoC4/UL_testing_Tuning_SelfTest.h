/***************************************************************************//**
* \file UL_testing_Tuning_SelfTest.h
* \version 6.0
*
* \brief
*   This file provides the function prototypes for the Built-In Self-Test
*   library.
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

#if !defined(CY_SENSE_UL_testing_Tuning_SELFTEST_H)
#define CY_SENSE_UL_testing_Tuning_SELFTEST_H

#include "cytypes.h"
#include "UL_testing_Tuning_Configuration.h"
#include "UL_testing_Tuning_Sensing.h"

#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_SELF_TEST_EN)

/*******************************************************************************
* Function Prototypes - High-level API
*******************************************************************************/

/**
* \cond (SECTION_C_HIGH_LEVEL)
* \addtogroup group_c_high_level
* \{
*/

uint32 UL_testing_Tuning_RunSelfTest(uint32 testEnMask);

/** \}
* \endcond */

/*******************************************************************************
* Function Prototypes - Low-level API
*******************************************************************************/

/**
* \cond (SECTION_C_LOW_LEVEL)
* \addtogroup group_c_low_level
* \{
*/

/* CRC test group */
#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_GLOBAL_CRC_EN)
    uint32 UL_testing_Tuning_CheckGlobalCRC(void);
#endif /*(UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_GLOBAL_CRC_EN) */
#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_WDGT_CRC_EN)
    uint32 UL_testing_Tuning_CheckWidgetCRC(uint32 widgetId);
#endif /*(UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_WDGT_CRC_EN) */

/* Baseline test group */
#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_BSLN_DUPLICATION_EN)
    uint32 UL_testing_Tuning_CheckBaselineDuplication(uint32 widgetId, uint32 sensorId);
#endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_BSLN_DUPLICATION_EN) */
#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_BSLN_RAW_OUT_RANGE_EN)
    uint32 UL_testing_Tuning_CheckBaselineRawcountRange(uint32 widgetId, uint32 sensorId,
                                    UL_testing_Tuning_BSLN_RAW_RANGE_STRUCT *ranges);
#endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_BSLN_RAW_OUT_RANGE_EN) */

/* Short test group */
#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS_SHORT_EN)
    uint32 UL_testing_Tuning_CheckSensorShort(uint32 widgetId, uint32 sensorId);
#endif /*(UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS_SHORT_EN) */
#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS2SNS_SHORT_EN)
    uint32 UL_testing_Tuning_CheckSns2SnsShort(uint32 widgetId, uint32 sensorId);
#endif /*(UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS2SNS_SHORT_EN) */

/* Capacitance measurement test group */
#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS_CAP_EN)
    uint32 UL_testing_Tuning_GetSensorCapacitance(uint32 widgetId, uint32 sensorId);
#endif /*(UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS_CAP_EN) */
#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SH_CAP_EN)
        uint32 UL_testing_Tuning_GetShieldCapacitance(void);
#endif /*(UL_testing_Tuning_ENABLE == UL_testing_Tuning__TST_SH_CAP_EN) */
#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_EXTERNAL_CAP_EN)
    uint32 UL_testing_Tuning_GetExtCapCapacitance(uint32 extCapId);
#endif /*(UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_EXTERNAL_CAP_EN) */

/* Vdda measurement */
#if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_VDDA_EN) && (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2))
    uint16 UL_testing_Tuning_GetVdda(void);
#endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_VDDA_EN) && (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2)) */

/** \}
* \endcond */

/*******************************************************************************
* Function Prototypes - Internal Functions
*******************************************************************************/

/**
* \cond SECTION_C_INTERNAL
* \addtogroup group_c_internal
* \{
*/
#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_GLOBAL_CRC_EN)
    void UL_testing_Tuning_DsUpdateGlobalCrc(void);
#endif /*(UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_GLOBAL_CRC_EN) */
#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_WDGT_CRC_EN)
    void UL_testing_Tuning_DsUpdateWidgetCrc(uint32 widgetId);
#endif /*(UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_WDGT_CRC_EN) */
#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_BSLN_DUPLICATION_EN)
    void UL_testing_Tuning_UpdateTestResultBaselineDuplication(uint32 widgetId, uint32 sensorId);
#endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_BSLN_DUPLICATION_EN) */

void UL_testing_Tuning_BistInitialize(void);
void UL_testing_Tuning_BistDisableMode(void);

#if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SNS_CAP_EN) || \
     (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_SH_CAP_EN) || \
     (UL_testing_Tuning_ENABLE == UL_testing_Tuning_TST_EXTERNAL_CAP_EN))
    CY_ISR_PROTO(UL_testing_Tuning_BistPostSingleScan);
#endif

/** \}
* \endcond */

/***********************************************************************************************************************
* Local definition
***********************************************************************************************************************/

/* Defines the mask for baseline data processing failure */
#define UL_testing_Tuning_PROCESS_BASELINE_FAILED                        (0x00000001uL << 31u)

/* Defines constants for Self-Test library */
#define UL_testing_Tuning_TST_LSBYTE                                     (0x000000FFuL)
#define UL_testing_Tuning_TST_FAILED                                     (0x0000FFFFuL)
#define UL_testing_Tuning_TST_BAD_PARAM                                  (0x00000001uL << 30u)
#define UL_testing_Tuning_TST_NOT_EXECUTED                               (0x00000001uL << 31u)

    /* Defines external capacitor ID */
    #define UL_testing_Tuning_TST_CMOD_MAP                               (0u)
    #define UL_testing_Tuning_TST_CSH_MAP                                (1u)
    #define UL_testing_Tuning_TST_CINTA_MAP                              (2u)
    #define UL_testing_Tuning_TST_CINTB_MAP                              (3u)

    #define UL_testing_Tuning_08_BIT_SHIFT                               (8uL)

    #define UL_testing_Tuning_BIST_SNS_CAP_MAX_CP                        ((0x00000001uL << 8uL) - 1uL)
    #define UL_testing_Tuning_BIST_SH_CAP_MAX_CP                         ((0x00000001uL << 16uL) - 1uL)
    #define UL_testing_Tuning_BIST_SNS_CAP_UNIT_SCALE                    (1000uL)
    #define UL_testing_Tuning_BIST_10_BIT_MASK                           ((0x00000001uL << 10uL) - 1uL)
    #define UL_testing_Tuning_BIST_CALIBRATION_TARGET                    ((UL_testing_Tuning_BIST_10_BIT_MASK * \
                                                                          UL_testing_Tuning_CSD_RAWCOUNT_CAL_LEVEL) / \
                                                                          UL_testing_Tuning_PERCENTAGE_100)

    #define UL_testing_Tuning_BIST_AVG_CYCLES_PER_LOOP                   (5u)
    #define UL_testing_Tuning_BIST_MEASURE_MAX_TIME_US                   (3000u)
    #define UL_testing_Tuning_BIST_PRECHARGE_MAX_TIME_US                 (250u)

    #define UL_testing_Tuning_BIST_MEASURE_WATCHDOG_CYCLES_NUM           (((CYDEV_BCLK__HFCLK__MHZ) * (UL_testing_Tuning_BIST_MEASURE_MAX_TIME_US)) /\
                                                                        (UL_testing_Tuning_BIST_AVG_CYCLES_PER_LOOP))
    #define UL_testing_Tuning_BIST_PRECHARGE_WATCHDOG_CYCLES_NUM         (((CYDEV_BCLK__HFCLK__MHZ) * (UL_testing_Tuning_BIST_PRECHARGE_MAX_TIME_US)) /\
                                                                        (UL_testing_Tuning_BIST_AVG_CYCLES_PER_LOOP))

    /* Min idac code at which we can guarantee 10% of raw count step per idac changing for one */
    #define UL_testing_Tuning_BIST_MIN_IDAC_VALUE                        (12uL)
    #define UL_testing_Tuning_BIST_MAX_IDAC_VALUE                        ((1uL << UL_testing_Tuning_IDAC_BITS_USED) - 5uL)
    #define UL_testing_Tuning_BIST_MAX_MODCLK_DIVIDER                    (0x000000FFuL)

#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2)
    #define UL_testing_Tuning_TST_EXT_CAP_LOW_RANGE                  (5uL)
    #define UL_testing_Tuning_TST_EXT_CAP_RESOLUTION                 (0x00000001uL << 10uL)
    #define UL_testing_Tuning_TST_EXT_CAP_DURATION                   (1uL)
    #define UL_testing_Tuning_TST_EXT_CAP_MODCLK_MHZ                 (4uL)
    #define UL_testing_Tuning_SENSE_DUTY_TST_EXT_CAP_WIDTH           (10uL)
    #define UL_testing_Tuning_SEQ_INIT_CNT_FINE_INIT_SKIP            (0x00000000uL)
    #define UL_testing_Tuning_IDAC_BITS_USED                         (7u)
#else /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2) */
    #define UL_testing_Tuning_TST_EXT_CAP_LOW_RANGE                  (5uL << 3uL)
    #define UL_testing_Tuning_TST_EXT_CAP_RESOLUTION                 (0xFFuL)
    #define UL_testing_Tuning_TST_EXT_CAP_SNSCLK_DIVIDER             (0xFFuL)
    #define UL_testing_Tuning_TST_EXT_CAP_DURATION                   (UL_testing_Tuning_TST_EXT_CAP_RESOLUTION << \
                                                                     UL_testing_Tuning_RESOLUTION_OFFSET)
    #define UL_testing_Tuning_TST_EXT_CAP_MODCLK_MHZ                 (2uL)
    #define UL_testing_Tuning_IDAC_BITS_USED                         (8u)
#endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2) */

#define UL_testing_Tuning_BIST_CAL_MIDDLE_BIT                        (1uL << (UL_testing_Tuning_IDAC_BITS_USED - 1u))
#define UL_testing_Tuning_TST_EXT_CAP_RESOLUTION_75                  ((UL_testing_Tuning_TST_EXT_CAP_RESOLUTION >> 1uL) +\
                                                                     (UL_testing_Tuning_TST_EXT_CAP_RESOLUTION >> 2uL))
#define UL_testing_Tuning_TST_EXT_CAP_MLTPLR                         (2u)
#define UL_testing_Tuning_CSDCMP_TST_CAP_MEAS                        (0x00000201uL)

/***********************************************************************************************************************
* HW CSD Block Config
***********************************************************************************************************************/
#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2)

    #define UL_testing_Tuning_BIST_INTR_SET_CFG                          (0x00000000uL)
    #define UL_testing_Tuning_BIST_SW_FW_TANK_SEL_CFG                    (0x00000000uL)
    #define UL_testing_Tuning_BIST_CAP_SENSE_DUTY_SEL                    (0x00010000uL)
    #define UL_testing_Tuning_BIST_AMBUF_PWR_MODE_OFF                    (UL_testing_Tuning_AMBUF_PWR_MODE_OFF)
    #define UL_testing_Tuning_BIST_AUTO_ZERO_TIME                        (UL_testing_Tuning_CSD_AUTO_ZERO_TIME)

    #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN)
        #if (UL_testing_Tuning_CSD__CMOD_PAD == UL_testing_Tuning_CMOD_CONNECTION)
            #define UL_testing_Tuning_BIST_SW_DSI_SEL_CMODPAD                (UL_testing_Tuning_SW_DSI_CMOD)
        #elif (UL_testing_Tuning_CSD__CSH_TANK_PAD == UL_testing_Tuning_CMOD_CONNECTION)
            #define UL_testing_Tuning_BIST_SW_DSI_SEL_CMODPAD                (UL_testing_Tuning_SW_DSI_CTANK)
        #else
            #define UL_testing_Tuning_BIST_SW_DSI_SEL_CMODPAD                (0x00000000uL)
        #endif

        #if (UL_testing_Tuning_CSD__CSH_TANK_PAD == UL_testing_Tuning_CTANK_CONNECTION)
            #define UL_testing_Tuning_BIST_SW_DSI_SEL_TANKPAD                (UL_testing_Tuning_SW_DSI_CTANK)
        #elif (UL_testing_Tuning_CSD__CMOD_PAD == UL_testing_Tuning_CTANK_CONNECTION)
            #define UL_testing_Tuning_BIST_SW_DSI_SEL_TANKPAD                (UL_testing_Tuning_SW_DSI_CMOD)
        #else
            #define UL_testing_Tuning_BIST_SW_DSI_SEL_TANKPAD                (0x00000000uL)
        #endif
    #else
        #define UL_testing_Tuning_BIST_SW_DSI_SEL_CMODPAD                (UL_testing_Tuning_SW_DSI_CMOD)
        #define UL_testing_Tuning_BIST_SW_DSI_SEL_TANKPAD                (UL_testing_Tuning_SW_DSI_CTANK)
    #endif

    #define UL_testing_Tuning_BIST_DEFAULT_SW_DSI_SEL                    (UL_testing_Tuning_BIST_SW_DSI_SEL_CMODPAD | \
                                                                         UL_testing_Tuning_BIST_SW_DSI_SEL_TANKPAD)
    #define UL_testing_Tuning_BIST_ADC_CTL_CFG                           (0x00000000uL)
    /* Shield switch default config */
    #define UL_testing_Tuning_BIST_SW_SHIELD_SEL_INIT                 (UL_testing_Tuning_SW_SHIELD_SEL_SW_HCAV_HSCMP)
    #if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) && \
         (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_TANK_EN))
        #if (UL_testing_Tuning_IDAC_SINKING == UL_testing_Tuning_CSD_IDAC_CONFIG)
            #define UL_testing_Tuning_BIST_SW_SHIELD_SEL_CFG          (UL_testing_Tuning_SW_SHIELD_SEL_SW_HCBG_HSCMP)
        #else
            #define UL_testing_Tuning_BIST_SW_SHIELD_SEL_CFG          (UL_testing_Tuning_SW_SHIELD_SEL_SW_HCBV_HSCMP)
        #endif /* (UL_testing_Tuning_IDAC_SINKING == UL_testing_Tuning_CSD_IDAC_CONFIG) */
    #elif(UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN)
        #if (UL_testing_Tuning_IDAC_SINKING == UL_testing_Tuning_CSD_IDAC_CONFIG)
            #define UL_testing_Tuning_BIST_SW_SHIELD_SEL_CFG          (UL_testing_Tuning_SW_SHIELD_SEL_SW_HCBV_PHI1 | \
                                                                      UL_testing_Tuning_SW_SHIELD_SEL_SW_HCBG_PHI2_HSCMP)
        #else
            #define UL_testing_Tuning_BIST_SW_SHIELD_SEL_CFG          (UL_testing_Tuning_SW_SHIELD_SEL_SW_HCBG_PHI1 | \
                                                                      UL_testing_Tuning_SW_SHIELD_SEL_SW_HCBV_PHI2_HSCMP)
        #endif /* (UL_testing_Tuning_IDAC_SINKING == UL_testing_Tuning_CSD_IDAC_CONFIG) */
    #else
        #define UL_testing_Tuning_BIST_SW_SHIELD_SEL_CFG              (0x00000000uL)
    #endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) && \
               (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_TANK_EN)) */

    /* SW_HS_P_SEL switches state for Coarse initialization of CMOD (sense path) */
    #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN)
        #if (UL_testing_Tuning_CSD__CMOD_PAD == UL_testing_Tuning_CMOD_CONNECTION)
            #define UL_testing_Tuning_BIST_HS_P_SEL_COARSE_CMOD               (UL_testing_Tuning_SW_HS_P_SEL_SW_HMPM_STATIC_CLOSE)
        #elif (UL_testing_Tuning_CSD__CSHIELD_PAD == UL_testing_Tuning_CMOD_CONNECTION)
            #define UL_testing_Tuning_BIST_HS_P_SEL_COARSE_CMOD               (UL_testing_Tuning_SW_HS_P_SEL_SW_HMPS_STATIC_CLOSE)
        #else
            #define UL_testing_Tuning_BIST_HS_P_SEL_COARSE_CMOD               (UL_testing_Tuning_SW_HS_P_SEL_SW_HMPT_STATIC_CLOSE)
        #endif /* (UL_testing_Tuning_CSD__CMOD_PAD == UL_testing_Tuning_CMOD_CONNECTION) */
    #else
        #define UL_testing_Tuning_BIST_HS_P_SEL_COARSE_CMOD                   (UL_testing_Tuning_SW_HS_P_SEL_SW_HMPM_STATIC_CLOSE)
    #endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_CSX_EN) && (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN)) */

    #if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_TANK_EN) && \
        (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN))
        /* SW_HS_P_SEL switches state for Coarse initialization of CTANK (sense path) */
        #if (UL_testing_Tuning_CSD__CSH_TANK_PAD == UL_testing_Tuning_CTANK_CONNECTION)
            #define UL_testing_Tuning_BIST_HS_P_SEL_COARSE_TANK               (UL_testing_Tuning_SW_HS_P_SEL_SW_HMPT_STATIC_CLOSE)
            #define UL_testing_Tuning_BIST_HS_P_SEL_SCAN_TANK                (UL_testing_Tuning_SW_HS_P_SEL_SW_HMPT_STATIC_CLOSE)
        #elif (UL_testing_Tuning_CSD__CSHIELD_PAD == UL_testing_Tuning_CTANK_CONNECTION)
            #define UL_testing_Tuning_BIST_HS_P_SEL_COARSE_TANK               (UL_testing_Tuning_SW_HS_P_SEL_SW_HMPS_STATIC_CLOSE)
            #define UL_testing_Tuning_BIST_HS_P_SEL_SCAN_TANK                (UL_testing_Tuning_SW_HS_P_SEL_SW_HMPS_STATIC_CLOSE)
        #elif (UL_testing_Tuning_CSD__CMOD_PAD == UL_testing_Tuning_CTANK_CONNECTION)
            #define UL_testing_Tuning_BIST_HS_P_SEL_COARSE_TANK               (UL_testing_Tuning_SW_HS_P_SEL_SW_HMPM_STATIC_CLOSE)
            #define UL_testing_Tuning_BIST_HS_P_SEL_SCAN_TANK                (UL_testing_Tuning_SW_HS_P_SEL_SW_HMPM_STATIC_CLOSE)
        #else
            #define UL_testing_Tuning_BIST_HS_P_SEL_COARSE_TANK               (UL_testing_Tuning_SW_HS_P_SEL_SW_HMMA_STATIC_CLOSE)
            #define UL_testing_Tuning_BIST_HS_P_SEL_SCAN_TANK                (UL_testing_Tuning_SW_HS_P_SEL_SW_HMMB_STATIC_CLOSE)
        #endif /* (UL_testing_Tuning_CSD__CSH_TANK_PAD == UL_testing_Tuning_CTANK_CONNECTION) */
    #else
        #define UL_testing_Tuning_BIST_HS_P_SEL_COARSE_TANK                   (UL_testing_Tuning_SW_HS_P_SEL_SW_HMPT_STATIC_CLOSE)
        #define UL_testing_Tuning_BIST_HS_P_SEL_SCAN_TANK                     (UL_testing_Tuning_STATIC_OPEN)
    #endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_TANK_EN) && \
               (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN)) */

    #if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_TANK_EN) && \
        (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN))
        #define UL_testing_Tuning_BIST_HS_P_SEL_SCAN                       (UL_testing_Tuning_BIST_HS_P_SEL_SCAN_TANK)
    #elif(UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN)
        #define UL_testing_Tuning_BIST_HS_P_SEL_SCAN                       (UL_testing_Tuning_SW_HS_P_SEL_SW_HMMB_STATIC_CLOSE)
    #else
        #define UL_testing_Tuning_BIST_HS_P_SEL_SCAN                       (UL_testing_Tuning_STATIC_OPEN)
    #endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_TANK_EN) && \
               (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN)) */

    #define UL_testing_Tuning_BIST_SW_HS_P_SEL_COARSE                    (UL_testing_Tuning_BIST_HS_P_SEL_COARSE_CMOD | UL_testing_Tuning_BIST_HS_P_SEL_COARSE_TANK)
    #define UL_testing_Tuning_BIST_SW_HS_P_SEL_CFG                       (0x00000000uL)
    #define UL_testing_Tuning_BIST_SW_HS_N_SEL_CFG                       (0x00000000uL)

    #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_AUTO_ZERO_EN)
        #define UL_testing_Tuning_BIST_AZ_ENABLE_CFG                  (UL_testing_Tuning_CSD_AZ_EN_MASK)
    #else
        #define UL_testing_Tuning_BIST_AZ_ENABLE_CFG                  (0uL)
    #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_AUTO_ZERO_EN) */

    #define UL_testing_Tuning_BIST_HSCMP_CFG                             (UL_testing_Tuning_HSCMP_EN_MASK | UL_testing_Tuning_BIST_AZ_ENABLE_CFG)
    #define UL_testing_Tuning_BIST_CSDCMP_INIT                           (UL_testing_Tuning_CSDCMP_CSDCMP_DISABLED)

    #define UL_testing_Tuning_BIST_BLOCK_ON_DELAY                        (3uL * CYDEV_BCLK__HFCLK__MHZ)
    #define UL_testing_Tuning_BIST_HSCMP_ON_DELAY                        (1uL * CYDEV_BCLK__HFCLK__MHZ)

    #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN)
        #if (UL_testing_Tuning_CSD__CMOD_PAD == UL_testing_Tuning_CMOD_CONNECTION)
            #define UL_testing_Tuning_BIST_SW_FW_MOD_SEL_INIT    (UL_testing_Tuning_SW_FW_MOD_SEL_SW_F1PM_STATIC_CLOSE |\
                                                                 UL_testing_Tuning_SW_FW_MOD_SEL_SW_F1MA_STATIC_CLOSE |\
                                                                 UL_testing_Tuning_SW_FW_MOD_SEL_SW_F1CA_STATIC_CLOSE)
        #else
            #define UL_testing_Tuning_BIST_SW_FW_MOD_SEL_INIT    (0x00000000uL)
        #endif /* (UL_testing_Tuning_CSD__CMOD_PAD == UL_testing_Tuning_CMOD_CONNECTION) */

        #if (UL_testing_Tuning_CSD__CSH_TANK_PAD == UL_testing_Tuning_CTANK_CONNECTION)
            #define UL_testing_Tuning_BIST_SW_FW_TANK_SEL_INIT   (UL_testing_Tuning_SW_FW_TANK_SEL_SW_F2PT_STATIC_CLOSE |\
                                                                 UL_testing_Tuning_SW_FW_TANK_SEL_SW_F2MA_STATIC_CLOSE |\
                                                                 UL_testing_Tuning_SW_FW_TANK_SEL_SW_F2CA_STATIC_CLOSE)
        #else
            #define UL_testing_Tuning_BIST_SW_FW_TANK_SEL_INIT   (0x00000000uL)
        #endif /* (UL_testing_Tuning_CSD__CSH_TANK_PAD == UL_testing_Tuning_CTANK_CONNECTION) */

    #elif (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSX_EN)
        #define UL_testing_Tuning_BIST_SW_FW_MOD_SEL_INIT        (UL_testing_Tuning_SW_FW_MOD_SEL_SW_F1PM_STATIC_CLOSE |\
                                                                 UL_testing_Tuning_SW_FW_MOD_SEL_SW_F1MA_STATIC_CLOSE |\
                                                                 UL_testing_Tuning_SW_FW_MOD_SEL_SW_F1CA_STATIC_CLOSE)

        #define UL_testing_Tuning_BIST_SW_FW_TANK_SEL_INIT       (UL_testing_Tuning_SW_FW_TANK_SEL_SW_F2PT_STATIC_CLOSE |\
                                                                 UL_testing_Tuning_SW_FW_TANK_SEL_SW_F2MA_STATIC_CLOSE |\
                                                                 UL_testing_Tuning_SW_FW_TANK_SEL_SW_F2CA_STATIC_CLOSE)
    #else
        #define UL_testing_Tuning_BIST_SW_FW_MOD_SEL_INIT        (0x00000000uL)
        #define UL_testing_Tuning_BIST_SW_FW_TANK_SEL_INIT       (0x00000000uL)
    #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN) */


    #define UL_testing_Tuning_BIST_SW_RES_INIT                           (UL_testing_Tuning_CSD_INIT_SWITCH_RES << CYFLD_CSD_RES_HCAV__OFFSET)
    #define UL_testing_Tuning_BIST_SW_FW_MOD_SEL_SCAN                    (0x00000000uL)
    #if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) && \
         (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_TANK_EN) && \
         (UL_testing_Tuning_CSD__CSH_TANK_PAD == UL_testing_Tuning_CTANK_CONNECTION))
        #define UL_testing_Tuning_BIST_SW_FW_TANK_SEL_SCAN               (UL_testing_Tuning_SW_FW_TANK_SEL_SW_F2PT_STATIC_CLOSE |\
                                                                         UL_testing_Tuning_SW_FW_TANK_SEL_SW_F2CB_PHI2)
    #else
        #define UL_testing_Tuning_BIST_SW_FW_TANK_SEL_SCAN               (0x00000000uL)
    #endif

    #if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) && \
         (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_TANK_EN) && \
         (UL_testing_Tuning_CSD__CSH_TANK_PAD == UL_testing_Tuning_CTANK_CONNECTION))
        #define UL_testing_Tuning_BIST_SW_SHIELD_SEL                     (UL_testing_Tuning_SW_SHIELD_SEL_SW_HCBV_HSCMP)
    #elif((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) && \
          (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_TANK_EN))
        #define UL_testing_Tuning_BIST_SW_SHIELD_SEL                     (UL_testing_Tuning_SW_SHIELD_SEL_SW_HCBV_HSCMP)
    #elif(UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN)
        #define UL_testing_Tuning_BIST_SW_SHIELD_SEL                     (UL_testing_Tuning_SW_SHIELD_SEL_SW_HCBG_PHI1 | \
                                                                         UL_testing_Tuning_SW_SHIELD_SEL_SW_HCBV_PHI2_HSCMP)
    #else
        #define UL_testing_Tuning_BIST_SW_SHIELD_SEL                     (0x00000000uL)
    #endif
    #define UL_testing_Tuning_BIST_SW_RES_SCAN                           ((UL_testing_Tuning_CSD_SHIELD_SWITCH_RES << CYFLD_CSD_RES_HCBV__OFFSET) |\
                                                                         (UL_testing_Tuning_CSD_SHIELD_SWITCH_RES << CYFLD_CSD_RES_HCBG__OFFSET))
    #define UL_testing_Tuning_BIST_HSCMP_SCAN_MASK                       (UL_testing_Tuning_HSCMP_EN_MASK)
    #define UL_testing_Tuning_BIST_IDACB_CFG                             (0x00000000uL)
    #define UL_testing_Tuning_BIST_IDACA_CFG                             (0x01830000uL)

    /* IDAC Gain in nA */
    #define UL_testing_Tuning_TST_SNS_CAP_IDAC_GAIN                      (2400uL)

    #if ((CYDEV_BCLK__HFCLK__HZ / UL_testing_Tuning_TST_MOD_CLK_DIVIDER) <= UL_testing_Tuning_MOD_CSD_CLK_12MHZ)
        #define UL_testing_Tuning_FILTER_DELAY_CFG                       (UL_testing_Tuning_CONFIG_FILTER_DELAY_12MHZ)
    #elif ((CYDEV_BCLK__HFCLK__HZ / UL_testing_Tuning_TST_MOD_CLK_DIVIDER) <= UL_testing_Tuning_MOD_CSD_CLK_24MHZ)
        #define UL_testing_Tuning_FILTER_DELAY_CFG                       (UL_testing_Tuning_CONFIG_FILTER_DELAY_24MHZ)
    #else
        /* ((CYDEV_BCLK__HFCLK__HZ / UL_testing_Tuning_TST_MOD_CLK_DIVIDER) <= UL_testing_Tuning_MOD_CSD_CLK_48MHZ) */
        #define UL_testing_Tuning_FILTER_DELAY_CFG                       (UL_testing_Tuning_CONFIG_FILTER_DELAY_48MHZ)
    #endif

    #define UL_testing_Tuning_BIST_CSD_CONFIG                            (UL_testing_Tuning_CONFIG_FILTER_DELAY_2_CYCLES)
    #define UL_testing_Tuning_BIST_SW_REFGEN_SEL_CFG                     (UL_testing_Tuning_SW_REFGEN_SEL_SW_SGR_MASK)

    #define UL_testing_Tuning_TST_MEASMODE_VREF                              (0x1uL << CYFLD_CSD_ADC_MODE__OFFSET)
    #define UL_testing_Tuning_TST_MEASMODE_VREFBY2                           (0x2uL << CYFLD_CSD_ADC_MODE__OFFSET)
    #define UL_testing_Tuning_TST_MEASMODE_VIN                               (0x3uL << CYFLD_CSD_ADC_MODE__OFFSET)

    /* Clock defines */
    #define UL_testing_Tuning_TST_VDDA_MODCLK_DIV_DEFAULT                    (UL_testing_Tuning_TST_MOD_CLK_DIVIDER)
    #define UL_testing_Tuning_TST_VDDA_SENSE_DIV_DEFAULT                     (0x4uL)
    #define UL_testing_Tuning_TST_VDDA_TOTAL_CLOCK_DIV                       (UL_testing_Tuning_TST_VDDA_MODCLK_DIV_DEFAULT * \
                                                                             UL_testing_Tuning_TST_VDDA_SENSE_DIV_DEFAULT)

    /* Acquisition time definitions: ADC_CTL */
    #define UL_testing_Tuning_VDDA_ACQUISITION_TIME_US                       (10uL)
    #define UL_testing_Tuning_VDDA_ACQUISITION_BASE                          ((UL_testing_Tuning_VDDA_ACQUISITION_TIME_US * \
                                                                            (CYDEV_BCLK__HFCLK__MHZ)) / \
                                                                            UL_testing_Tuning_TST_VDDA_TOTAL_CLOCK_DIV)

    #define UL_testing_Tuning_TST_VDDA_ADC_AZ_TIME                           (5uL)
    #define UL_testing_Tuning_TST_VDDA_SEQ_TIME_BASE                         (((CYDEV_BCLK__HFCLK__HZ * UL_testing_Tuning_TST_VDDA_ADC_AZ_TIME) / \
                                                                               UL_testing_Tuning_TST_VDDA_TOTAL_CLOCK_DIV) / 1000000uL)

    #if (0 == UL_testing_Tuning_TST_VDDA_SEQ_TIME_BASE)
        #define UL_testing_Tuning_TST_VDDA_SEQ_TIME_DEFAULT                  (1u)
    #else
        #define UL_testing_Tuning_TST_VDDA_SEQ_TIME_DEFAULT                  (UL_testing_Tuning_TST_VDDA_SEQ_TIME_BASE)
    #endif




    #define UL_testing_Tuning_TST_VDDA_FILTER_DELAY                          (2uL)
    #define UL_testing_Tuning_TST_VDDA_FINE_INIT_TIME                        (UL_testing_Tuning_TST_FINE_INIT_TIME)
    #define UL_testing_Tuning_TST_VDDA_SCAN_DURATION                         (2uL)

    #define UL_testing_Tuning_TST_VDDA_VREF_TRIM_MAX_DEVIATION               (20uL)

    /* The reference voltage macros */
    #define UL_testing_Tuning_TST_VDDA_VREF_CALIB_USED                       (2400uL)

    #define UL_testing_Tuning_TST_VDDA_BAD_RESULT                            (0xFFFFu)


    #define UL_testing_Tuning_TST_VDDA_CONFIG_DEFAULT_CFG                    (UL_testing_Tuning_CONFIG_ENABLE_MASK |\
                                                                             UL_testing_Tuning_CONFIG_SAMPLE_SYNC_MASK |\
                                                                             UL_testing_Tuning_CONFIG_SENSE_EN_MASK |\
                                                                             UL_testing_Tuning_CONFIG_DSI_COUNT_SEL_MASK)

    #define UL_testing_Tuning_TST_VDDA_IDACA_DEFAULT_CFG                     (0x00000000uL)
    #define UL_testing_Tuning_TST_VDDA_IDACB_DEFAULT_CFG                     (UL_testing_Tuning_TST_VDDA_IDAC_DEFAULT |\
                                                                            (UL_testing_Tuning_IDAC_COMP_POL_DYN_DYNAMIC << \
                                                                              CYFLD_CSD_POL_DYN__OFFSET) |\
                                                                              UL_testing_Tuning_IDAC_COMP_LEG3_EN_MASK)

    #define UL_testing_Tuning_TST_CSDCMP_DEFAULT_CFG                         (0x00000000uL)
    #define UL_testing_Tuning_TST_SW_DSI_SEL_DEFAULT_CFG                     (0x00000000uL)
    #define UL_testing_Tuning_TST_SENSE_DUTY_VDDA_CFG                        (0x00000000uL)
    #define UL_testing_Tuning_TST_SEQ_INIT_CNT_DEFAULT_CFG                   (1u)
    #define UL_testing_Tuning_TST_SEQ_NORM_CNT_DEFAULT_CFG                   (UL_testing_Tuning_TST_VDDA_SCAN_DURATION)
    #define UL_testing_Tuning_TST_SW_HS_P_SEL_DEFAULT_CFG                    (UL_testing_Tuning_SW_HS_P_SEL_SW_HMRH_STATIC_CLOSE)
    #define UL_testing_Tuning_TST_SW_HS_N_SEL_DEFAULT_CFG                    (UL_testing_Tuning_SW_HS_N_SEL_SW_HCCD_STATIC_CLOSE)
    #define UL_testing_Tuning_TST_SW_SHIELD_SEL_DEFAULT_CFG                  (0x00000000uL)
    #define UL_testing_Tuning_TST_SW_BYP_SEL_DEFAULT_CFG                     (UL_testing_Tuning_SW_BYP_SEL_SW_BYA_MASK |\
                                                                             UL_testing_Tuning_SW_BYP_SEL_SW_BYB_MASK)
    #define UL_testing_Tuning_TST_SW_CMP_P_SEL_DEFAULT_CFG                   (0x00000000uL)
    #define UL_testing_Tuning_TST_SW_CMP_N_SEL_DEFAULT_CFG                   (0x00000000uL)
    #define UL_testing_Tuning_TST_SW_FW_MOD_SEL_DEFAULT_CFG                  (UL_testing_Tuning_SW_FW_MOD_SEL_SW_C1CC_STATIC_CLOSE |\
                                                                             UL_testing_Tuning_SW_FW_MOD_SEL_SW_C1CD_STATIC_CLOSE)
    #define UL_testing_Tuning_TST_SW_FW_TANK_SEL_DEFAULT_CFG                 (UL_testing_Tuning_SW_FW_TANK_SEL_SW_C2CC_STATIC_CLOSE |\
                                                                             UL_testing_Tuning_SW_FW_TANK_SEL_SW_C2CD_STATIC_CLOSE)
    #define UL_testing_Tuning_TST_SW_REFGEN_SEL_DEFAULT_CFG                  (UL_testing_Tuning_SW_REFGEN_SEL_SW_SGR_MASK)
    #define UL_testing_Tuning_TST_REFGEN_DEFAULT_CFG                         (UL_testing_Tuning_REFGEN_REFGEN_EN_MASK |\
                                                                             UL_testing_Tuning_REFGEN_RES_EN_MASK |\
                                                                            ((uint32)UL_testing_Tuning_TST_VDDA_VREF_GAIN << \
                                                                             CYFLD_CSD_GAIN__OFFSET))
    #define UL_testing_Tuning_TST_SW_AMUXBUF_SEL_DEFAULT_CFG                 (0x00000000uL)
    #define UL_testing_Tuning_TST_HSCMP_DEFAULT_CFG                          (UL_testing_Tuning_HSCMP_EN_MASK |\
                                                                             UL_testing_Tuning_CSD_AZ_EN_MASK)

    #define UL_testing_Tuning_TST_VDDA_SEQ_START_MEASURE_CFG                 (UL_testing_Tuning_SEQ_START_AZ0_SKIP_MASK |\
                                                                            UL_testing_Tuning_SEQ_START_START_MASK)
#else /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2) */

#define UL_testing_Tuning_BIST_CSD_CONFIG                            (UL_testing_Tuning_CONFIG_SENSE_COMP_BW_MASK | \
                                                                     UL_testing_Tuning_CONFIG_SENSE_INSEL_MASK | \
                                                                     UL_testing_Tuning_CONFIG_REFBUF_DRV_MASK)

#define UL_testing_Tuning_BIST_CMOD_PRECHARGE_CONFIG                 (UL_testing_Tuning_BIST_CSD_CONFIG | \
                                                                     UL_testing_Tuning_CONFIG_REFBUF_EN_MASK | \
                                                                     UL_testing_Tuning_CONFIG_COMP_PIN_MASK)

#define UL_testing_Tuning_BIST_CMOD_PRECHARGE_CONFIG_CSD_EN          (UL_testing_Tuning_BIST_CMOD_PRECHARGE_CONFIG | \
                                                                     UL_testing_Tuning_CSD_ENABLE_MASK)

/* Third-generation HW block Ctank pre-charge mode configuration */
#if(UL_testing_Tuning_CSD_CSH_PRECHARGE_SRC == UL_testing_Tuning_CSH_PRECHARGE_VREF)
    #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN)
        #define  UL_testing_Tuning_BIST_CTANK_PRECHARGE_CONFIG       (UL_testing_Tuning_BIST_CSD_CONFIG | \
                                                                     UL_testing_Tuning_CONFIG_REFBUF_EN_MASK | \
                                                                     UL_testing_Tuning_CONFIG_PRS_CLEAR_MASK | \
                                                                     UL_testing_Tuning_CONFIG_REFBUF_OUTSEL_MASK)
    #else
        #define  UL_testing_Tuning_BIST_CTANK_PRECHARGE_CONFIG       (UL_testing_Tuning_BIST_CSD_CONFIG | \
                                                                     UL_testing_Tuning_CONFIG_REFBUF_OUTSEL_MASK | \
                                                                     UL_testing_Tuning_CONFIG_PRS_CLEAR_MASK)
    #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) */
#else
    #define  UL_testing_Tuning_BIST_CTANK_PRECHARGE_CONFIG           (UL_testing_Tuning_BIST_CSD_CONFIG |\
                                                                     UL_testing_Tuning_CONFIG_REFBUF_OUTSEL_MASK |\
                                                                     UL_testing_Tuning_CONFIG_REFBUF_EN_MASK |\
                                                                     UL_testing_Tuning_CONFIG_COMP_MODE_MASK |\
                                                                     UL_testing_Tuning_CONFIG_PRS_CLEAR_MASK |\
                                                                     UL_testing_Tuning_CONFIG_COMP_PIN_MASK)

#endif /* (UL_testing_Tuning_CSD_CSH_PRECHARGE_SRC == UL_testing_Tuning__CSH_PRECHARGE_IO_BUF) */

#define  UL_testing_Tuning_BIST_CTANK_PRECHARGE_CONFIG_CSD_EN        (UL_testing_Tuning_BIST_CTANK_PRECHARGE_CONFIG | \
                                                                     UL_testing_Tuning_CONFIG_ENABLE_MASK | \
                                                                     UL_testing_Tuning_CONFIG_SENSE_COMP_EN_MASK)

#define UL_testing_Tuning_BIST_IDAC_CFG                              (0x00000200uL)
#define UL_testing_Tuning_BIST_IDAC_MOD_MASK                         (0x000003FFuL)
#define UL_testing_Tuning_BIST_IDAC_MOD_VAL_MASK                     (0x000000FFuL)

/* IDAC Gain = 8x that corresponds to 1LSB = 1200 nA */
#define UL_testing_Tuning_TST_SNS_CAP_IDAC_GAIN                      (1200uL)

#define UL_testing_Tuning_BIST_SCAN_DURATION                         (UL_testing_Tuning_BIST_10_BIT_MASK << UL_testing_Tuning_RESOLUTION_OFFSET)
#define UL_testing_Tuning_BIST_SCAN_DURATION_255                     (0x1FFuL << UL_testing_Tuning_RESOLUTION_OFFSET)
#endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2) */

#endif /* #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_SELF_TEST_EN) */

#endif /* End CY_SENSE_UL_testing_Tuning_SELFTEST_H */


/* [] END OF FILE */
