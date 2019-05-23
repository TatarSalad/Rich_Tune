/***************************************************************************//**
* \file UL_testing_Tuning_SensingCSD_LL.h
* \version 6.0
*
* \brief
*   This file provides the headers of APIs specific to CSD sensing implementation.
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

#if !defined(CY_SENSE_UL_testing_Tuning_SENSINGCSD_LL_H)
#define CY_SENSE_UL_testing_Tuning_SENSINGCSD_LL_H

#include <CyLib.h>
#include <cyfitter.h>
#include <cytypes.h>
#include <cypins.h>
#include "UL_testing_Tuning_Structure.h"
#include "UL_testing_Tuning_Configuration.h"
#include "UL_testing_Tuning_Sensing.h"

/****************************************************************************
* Register and mode mask definition
****************************************************************************/

#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2)
    #define UL_testing_Tuning_CSD_CSDCMP_INIT                                (UL_testing_Tuning_CSDCMP_CSDCMP_DISABLED)

    /* SW_HS_P_SEL switches state for Coarse initialization of CMOD (sense path) */
    #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN)
        #if (UL_testing_Tuning_CSD__CMOD_PAD == UL_testing_Tuning_CMOD_CONNECTION)
            #define UL_testing_Tuning_CSD_HS_P_SEL_COARSE_CMOD               (UL_testing_Tuning_SW_HS_P_SEL_SW_HMPM_STATIC_CLOSE)
        #elif (UL_testing_Tuning_CSD__CSHIELD_PAD == UL_testing_Tuning_CMOD_CONNECTION)
            #define UL_testing_Tuning_CSD_HS_P_SEL_COARSE_CMOD               (UL_testing_Tuning_SW_HS_P_SEL_SW_HMPS_STATIC_CLOSE)
        #else
            #define UL_testing_Tuning_CSD_HS_P_SEL_COARSE_CMOD               (UL_testing_Tuning_SW_HS_P_SEL_SW_HMPT_STATIC_CLOSE)
        #endif /* (UL_testing_Tuning_CSD__CMOD_PAD == UL_testing_Tuning_CMOD_CONNECTION) */
    #else
        #define UL_testing_Tuning_CSD_HS_P_SEL_COARSE_CMOD                   (0x00000000uL)
    #endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_CSX_EN) && (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_EN)) */

    #if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_TANK_EN) && \
        (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN))
        /* SW_HS_P_SEL switches state for Coarse initialization of CTANK (sense path) */
        #if (UL_testing_Tuning_CSD__CSH_TANK_PAD == UL_testing_Tuning_CTANK_CONNECTION)
            #define UL_testing_Tuning_CSD_HS_P_SEL_COARSE_TANK               (UL_testing_Tuning_SW_HS_P_SEL_SW_HMPT_STATIC_CLOSE)
        #elif (UL_testing_Tuning_CSD__CSHIELD_PAD == UL_testing_Tuning_CTANK_CONNECTION)
            #define UL_testing_Tuning_CSD_HS_P_SEL_COARSE_TANK               (UL_testing_Tuning_SW_HS_P_SEL_SW_HMPS_STATIC_CLOSE)
        #elif (UL_testing_Tuning_CSD__CMOD_PAD == UL_testing_Tuning_CTANK_CONNECTION)
            #define UL_testing_Tuning_CSD_HS_P_SEL_COARSE_TANK               (UL_testing_Tuning_SW_HS_P_SEL_SW_HMPM_STATIC_CLOSE)
        #else
            #define UL_testing_Tuning_CSD_HS_P_SEL_COARSE_TANK               (UL_testing_Tuning_SW_HS_P_SEL_SW_HMMA_STATIC_CLOSE)
        #endif /* (UL_testing_Tuning_CSD__CSH_TANK_PAD == UL_testing_Tuning_CTANK_CONNECTION) */
    #else
        #define UL_testing_Tuning_CSD_HS_P_SEL_COARSE_TANK                   (0x00000000uL)
    #endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_TANK_EN) && \
               (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN)) */

    #define UL_testing_Tuning_CSD_SW_HS_P_SEL_COARSE                         (UL_testing_Tuning_HS_P_SEL_COARSE_CMOD | UL_testing_Tuning_CSD_HS_P_SEL_COARSE_TANK)

    /* C_mod config */
    #if ((UL_testing_Tuning_CSD__CMOD_PAD == UL_testing_Tuning_CMOD_CONNECTION) || (UL_testing_Tuning_CSD__CMOD_PAD == UL_testing_Tuning_CTANK_CONNECTION))
        #define UL_testing_Tuning_CSD_SW_FW_MOD_SEL_INIT             (UL_testing_Tuning_SW_FW_MOD_SEL_SW_F1PM_STATIC_CLOSE |\
                                                                     UL_testing_Tuning_SW_FW_MOD_SEL_SW_F1MA_STATIC_CLOSE |\
                                                                     UL_testing_Tuning_SW_FW_MOD_SEL_SW_F1CA_STATIC_CLOSE)
        #define UL_testing_Tuning_SW_DSI_SEL_CMODPAD                 (UL_testing_Tuning_SW_DSI_CMOD)
    #else
        #define UL_testing_Tuning_CSD_SW_FW_MOD_SEL_INIT             (0x00000000uL)
        #define UL_testing_Tuning_SW_DSI_SEL_CMODPAD                 (0x00000000uL)
    #endif /* (UL_testing_Tuning_CSD__CMOD_PAD == UL_testing_Tuning_CMOD_CONNECTION) */

    /* C_tank config */
    #if ((UL_testing_Tuning_CSD__CSH_TANK_PAD == UL_testing_Tuning_CMOD_CONNECTION) || (UL_testing_Tuning_CSD__CSH_TANK_PAD == UL_testing_Tuning_CTANK_CONNECTION))
        #define UL_testing_Tuning_CSD_SW_FW_TANK_SEL_INIT            (UL_testing_Tuning_SW_FW_TANK_SEL_SW_F2PT_STATIC_CLOSE |\
                                                                     UL_testing_Tuning_SW_FW_TANK_SEL_SW_F2MA_STATIC_CLOSE |\
                                                                     UL_testing_Tuning_SW_FW_TANK_SEL_SW_F2CA_STATIC_CLOSE)
        #define UL_testing_Tuning_SW_DSI_SEL_TANKPAD                 (UL_testing_Tuning_SW_DSI_CTANK)
    #else
        #define UL_testing_Tuning_CSD_SW_FW_TANK_SEL_INIT            (0x00000000uL)
        #define UL_testing_Tuning_SW_DSI_SEL_TANKPAD                 (0x00000000uL)
    #endif /* (UL_testing_Tuning_CSD__CSH_TANK_PAD == UL_testing_Tuning_CTANK_CONNECTION) */

    #define UL_testing_Tuning_CSD_SW_SHIELD_SEL_INIT                 (UL_testing_Tuning_SW_SHIELD_SEL_SW_HCAV_HSCMP)

    /* Defining default HW configuration according to settings in customizer. */
    #define UL_testing_Tuning_DEFAULT_CSD_CONFIG                 (UL_testing_Tuning_CONFIG_FILTER_DELAY_12MHZ |\
                                                                 UL_testing_Tuning_CONFIG_SAMPLE_SYNC_MASK)
    #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_AUTO_ZERO_EN)
        /* Enable CSDCMP */
        #define UL_testing_Tuning_CSD_CSDCMP_SCAN                (UL_testing_Tuning_CSDCMP_CSDCMP_EN_MASK |\
                                                                 UL_testing_Tuning_CSDCMP_AZ_EN_MASK)
    #else
        /* Enable CSDCMP */
        #define UL_testing_Tuning_CSD_CSDCMP_SCAN                (UL_testing_Tuning_CSDCMP_CSDCMP_EN_MASK)
    #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_AUTO_ZERO_EN) */

    #if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_TANK_EN) && \
        (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN))
        /* SW_HS_P_SEL switches state for Coarse initialization of CTANK (sense path) */
        #if (UL_testing_Tuning_CSD__CSH_TANK_PAD == UL_testing_Tuning_CTANK_CONNECTION)
            #define UL_testing_Tuning_CSD_HS_P_SEL_SCAN_TANK                 (UL_testing_Tuning_SW_HS_P_SEL_SW_HMPT_STATIC_CLOSE)
        #elif (UL_testing_Tuning_CSD__CSHIELD_PAD == UL_testing_Tuning_CTANK_CONNECTION)
            #define UL_testing_Tuning_CSD_HS_P_SEL_SCAN_TANK                 (UL_testing_Tuning_SW_HS_P_SEL_SW_HMPS_STATIC_CLOSE)
        #elif (UL_testing_Tuning_CSD__CMOD_PAD == UL_testing_Tuning_CTANK_CONNECTION)
            #define UL_testing_Tuning_CSD_HS_P_SEL_SCAN_TANK                 (UL_testing_Tuning_SW_HS_P_SEL_SW_HMPM_STATIC_CLOSE)
        #else
            #define UL_testing_Tuning_CSD_HS_P_SEL_SCAN_TANK                 (UL_testing_Tuning_SW_HS_P_SEL_SW_HMMB_STATIC_CLOSE)
        #endif /* (UL_testing_Tuning_CSD__CSH_TANK_PAD == UL_testing_Tuning_CTANK_CONNECTION) */
        #define UL_testing_Tuning_CSD_SW_HS_P_SEL_SCAN                       (UL_testing_Tuning_HS_P_SEL_SCAN_TANK)
    #elif(UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN)
        #define UL_testing_Tuning_CSD_SW_HS_P_SEL_SCAN                       (UL_testing_Tuning_SW_HS_P_SEL_SW_HMMB_STATIC_CLOSE)
    #else
        #define UL_testing_Tuning_CSD_SW_HS_P_SEL_SCAN                       (UL_testing_Tuning_STATIC_OPEN)
    #endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_TANK_EN) && \
               (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN)) */

    /* SW_FW_MOD_SEL switches state for Coarse initialization of CMOD (sense path) */
    #define UL_testing_Tuning_CSD_SW_FW_MOD_SEL_SCAN                 (0x00000000uL)

    #if((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) && \
        (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_TANK_EN) && \
        (UL_testing_Tuning_CSD__CSH_TANK_PAD == UL_testing_Tuning_CTANK_CONNECTION))
        #define UL_testing_Tuning_CSD_SW_FW_TANK_SEL_SCAN            (UL_testing_Tuning_SW_FW_TANK_SEL_SW_F2PT_STATIC_CLOSE | \
                                                                 UL_testing_Tuning_SW_FW_TANK_SEL_SW_F2CB_STATIC_CLOSE)
    #else
        #define UL_testing_Tuning_CSD_SW_FW_TANK_SEL_SCAN            (0x00000000uL)
    #endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) && \
               (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_TANK_EN) && \
               (UL_testing_Tuning_CSD__CSH_TANK_PAD == UL_testing_Tuning_CTANK_CONNECTION)) */

    /* Shield switch default config */
    #if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) && \
         (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_TANK_EN))
        #if (UL_testing_Tuning_IDAC_SINKING == UL_testing_Tuning_CSD_IDAC_CONFIG)
            #define UL_testing_Tuning_CSD_SW_SHIELD_SEL_SCAN          (UL_testing_Tuning_SW_SHIELD_SEL_SW_HCBG_HSCMP)
        #else
            #define UL_testing_Tuning_CSD_SW_SHIELD_SEL_SCAN          (UL_testing_Tuning_SW_SHIELD_SEL_SW_HCBV_HSCMP)
        #endif /* (UL_testing_Tuning_IDAC_SINKING == UL_testing_Tuning_CSD_IDAC_CONFIG) */
    #elif(UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN)
        #if (UL_testing_Tuning_IDAC_SINKING == UL_testing_Tuning_CSD_IDAC_CONFIG)
            #define UL_testing_Tuning_CSD_SW_SHIELD_SEL_SCAN          (UL_testing_Tuning_SW_SHIELD_SEL_SW_HCBV_PHI1 | \
                                                                     UL_testing_Tuning_SW_SHIELD_SEL_SW_HCBG_PHI2_HSCMP)
        #else
            #define UL_testing_Tuning_CSD_SW_SHIELD_SEL_SCAN          (UL_testing_Tuning_SW_SHIELD_SEL_SW_HCBG_PHI1 | \
                                                                     UL_testing_Tuning_SW_SHIELD_SEL_SW_HCBV_PHI2_HSCMP)
        #endif /* (UL_testing_Tuning_IDAC_SINKING == UL_testing_Tuning_CSD_IDAC_CONFIG) */
    #else
        #define UL_testing_Tuning_CSD_SW_SHIELD_SEL_SCAN              (0x00000000uL)
    #endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) && \
               (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_TANK_EN)) */

    #define UL_testing_Tuning_CSD_SW_RES_INIT                        (UL_testing_Tuning_CSD_INIT_SWITCH_RES << CYFLD_CSD_RES_HCAV__OFFSET)
    #define UL_testing_Tuning_CSD_SW_RES_SCAN                        ((UL_testing_Tuning_CSD_SHIELD_SWITCH_RES << CYFLD_CSD_RES_HCBV__OFFSET) |\
                                                                     (UL_testing_Tuning_CSD_SHIELD_SWITCH_RES << CYFLD_CSD_RES_HCBG__OFFSET))

    #define UL_testing_Tuning_CSD_SHIELD_GPIO_DM                 (UL_testing_Tuning_GPIO_STRGDRV)
    #define UL_testing_Tuning_CSD_SENSOR_HSIOM_SEL               (UL_testing_Tuning_HSIOM_SEL_CSD_SENSE)
    #define UL_testing_Tuning_CSD_SHIELD_HSIOM_SEL               (UL_testing_Tuning_HSIOM_SEL_CSD_SHIELD)
    #define UL_testing_Tuning_CSD_CMOD_HSIOM_SEL                 (UL_testing_Tuning_HSIOM_SEL_AMUXA)

    #define UL_testing_Tuning_DEFAULT_IDAC_MOD_BALL_MODE         ((uint32)UL_testing_Tuning_IDAC_MOD_BALL_MODE_FULL <<\
                                                                 CYFLD_CSD_BAL_MODE__OFFSET)
    #define UL_testing_Tuning_DEFAULT_IDAC_COMP_BALL_MODE        ((uint32)UL_testing_Tuning_IDAC_COMP_BALL_MODE_FULL <<\
                                                                 CYFLD_CSD_BAL_MODE__OFFSET)

    #define UL_testing_Tuning_DEFAULT_SENSE_DUTY_SEL             (UL_testing_Tuning_SENSE_DUTY_OVERLAP_PHI1_MASK |\
                                                                 UL_testing_Tuning_SENSE_DUTY_OVERLAP_PHI2_MASK)

    #define UL_testing_Tuning_CSD_IDAC_MOD_BITS_USED                     (7u)
    #define UL_testing_Tuning_CAL_MIDDLE_BIT                             (1uL << (UL_testing_Tuning_CSD_IDAC_MOD_BITS_USED - 1u))

    #define UL_testing_Tuning_DELAY_EXTRACYCLES_NUM                      (9u)

    #if (UL_testing_Tuning_IDAC_GAIN_HIGH == UL_testing_Tuning_CSD_IDAC_GAIN)
        #define UL_testing_Tuning_CSD_IDAC_GAIN_VALUE_NA                 (2400u)
    #elif (UL_testing_Tuning_IDAC_GAIN_MEDIUM == UL_testing_Tuning_CSD_IDAC_GAIN)
        #define UL_testing_Tuning_CSD_IDAC_GAIN_VALUE_NA                 (300u)
    #else
        #define UL_testing_Tuning_CSD_IDAC_GAIN_VALUE_NA                 (37u)
    #endif /* (UL_testing_Tuning_IDAC_GAIN_HIGH == UL_testing_Tuning_CSD_IDAC_GAIN) */

    /* Defining the drive mode of pins depending on the Inactive sensor connection setting in the Component customizer. */
    #if(UL_testing_Tuning_SNS_CONNECTION_GROUND == UL_testing_Tuning_CSD_INACTIVE_SNS_CONNECTION)
        #define UL_testing_Tuning_CSD_INACTIVE_SNS_GPIO_DM               (CY_SYS_PINS_DM_STRONG)
    #elif(UL_testing_Tuning_SNS_CONNECTION_HIGHZ == UL_testing_Tuning_CSD_INACTIVE_SNS_CONNECTION)
        #define UL_testing_Tuning_CSD_INACTIVE_SNS_GPIO_DM               (CY_SYS_PINS_DM_ALG_HIZ)
    #elif(UL_testing_Tuning_SNS_CONNECTION_SHIELD == UL_testing_Tuning_CSD_INACTIVE_SNS_CONNECTION)
        #define UL_testing_Tuning_CSD_INACTIVE_SNS_GPIO_DM               (CY_SYS_PINS_DM_ALG_HIZ)
    #else
        #error "Unsupported inactive connection for the inactive sensors."
    #endif /* (UL_testing_Tuning_SNS_CONNECTION_GROUND == UL_testing_Tuning_CSD_INACTIVE_SNS_CONNECTION) */


    /* Clock Source Mode */
    #if (UL_testing_Tuning_CLK_SOURCE_DIRECT == UL_testing_Tuning_CSD_SNS_CLK_SOURCE)
        #define UL_testing_Tuning_DEFAULT_MODULATION_MODE                (UL_testing_Tuning_CLK_SOURCE_DIRECT)
    #elif (UL_testing_Tuning_CLK_SOURCE_PRSAUTO == UL_testing_Tuning_CSD_SNS_CLK_SOURCE)
        #define UL_testing_Tuning_DEFAULT_MODULATION_MODE                (UL_testing_Tuning_CLK_SOURCE_SSC2)
    #elif ((UL_testing_Tuning_CLK_SOURCE_PRS8) == UL_testing_Tuning_CSD_SNS_CLK_SOURCE)
        #define UL_testing_Tuning_DEFAULT_MODULATION_MODE                (UL_testing_Tuning_CSD_SNS_CLK_SOURCE)
    #elif ((UL_testing_Tuning_CLK_SOURCE_PRS12) == UL_testing_Tuning_CSD_SNS_CLK_SOURCE)
        #define UL_testing_Tuning_DEFAULT_MODULATION_MODE                (UL_testing_Tuning_CSD_SNS_CLK_SOURCE)
    #else
        #define UL_testing_Tuning_DEFAULT_MODULATION_MODE                (UL_testing_Tuning_CSD_SNS_CLK_SOURCE)
    #endif /* (UL_testing_Tuning_CLK_SOURCE_DIRECT != UL_testing_Tuning_CSD_SNS_CLK_SOURCE) */

    /* IDACs Ranges */
    #if (UL_testing_Tuning_IDAC_GAIN_LOW == UL_testing_Tuning_CSD_IDAC_GAIN)
        #define UL_testing_Tuning_DEFAULT_IDAC_MOD_RANGE                 ((uint32)UL_testing_Tuning_IDAC_MOD_RANGE_IDAC_LO << CYFLD_CSD_RANGE__OFFSET)
        #define UL_testing_Tuning_DEFAULT_IDAC_COMP_RANGE                ((uint32)UL_testing_Tuning_IDAC_COMP_RANGE_IDAC_LO << CYFLD_CSD_RANGE__OFFSET)
    #elif (UL_testing_Tuning_IDAC_GAIN_MEDIUM == UL_testing_Tuning_CSD_IDAC_GAIN)
        #define UL_testing_Tuning_DEFAULT_IDAC_MOD_RANGE                 ((uint32)UL_testing_Tuning_IDAC_MOD_RANGE_IDAC_MED << CYFLD_CSD_RANGE__OFFSET)
        #define UL_testing_Tuning_DEFAULT_IDAC_COMP_RANGE                ((uint32)UL_testing_Tuning_IDAC_COMP_RANGE_IDAC_MED << CYFLD_CSD_RANGE__OFFSET)
    #else
        #define UL_testing_Tuning_DEFAULT_IDAC_MOD_RANGE                 ((uint32)UL_testing_Tuning_IDAC_MOD_RANGE_IDAC_HI << CYFLD_CSD_RANGE__OFFSET)
        #define UL_testing_Tuning_DEFAULT_IDAC_COMP_RANGE                ((uint32)UL_testing_Tuning_IDAC_COMP_RANGE_IDAC_HI << CYFLD_CSD_RANGE__OFFSET)
    #endif

    /* IDACs Polarities */
    #if (UL_testing_Tuning_IDAC_SINKING == UL_testing_Tuning_CSD_IDAC_CONFIG)
        #define UL_testing_Tuning_DEFAULT_IDAC_MOD_POLARITY              ((uint32)UL_testing_Tuning_IDAC_MOD_POLARITY_VDDA_SNK << CYFLD_CSD_POLARITY__OFFSET)
        #define UL_testing_Tuning_DEFAULT_IDAC_COMP_POLARITY             ((uint32)UL_testing_Tuning_IDAC_COMP_POLARITY_VDDA_SNK << CYFLD_CSD_POLARITY__OFFSET)
    #else
        #define UL_testing_Tuning_DEFAULT_IDAC_MOD_POLARITY              ((uint32)UL_testing_Tuning_IDAC_MOD_POLARITY_VSSA_SRC << CYFLD_CSD_POLARITY__OFFSET)
        #define UL_testing_Tuning_DEFAULT_IDAC_COMP_POLARITY             ((uint32)UL_testing_Tuning_IDAC_COMP_POLARITY_VSSA_SRC << CYFLD_CSD_POLARITY__OFFSET)
    #endif /* (UL_testing_Tuning_IDAC_SINKING == UL_testing_Tuning_CSD_IDAC_CONFIG) */

    #define UL_testing_Tuning_SW_REFGEN_VREF_SRC                         (UL_testing_Tuning_SW_REFGEN_SEL_SW_SGR_MASK)

    /* IDAC legs configuration */
    #if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_IDAC_COMP_EN) && \
         (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_DEDICATED_IDAC_COMP_EN))
            #define UL_testing_Tuning_DEFAULT_SW_REFGEN_SEL              (UL_testing_Tuning_SW_REFGEN_VREF_SRC | UL_testing_Tuning_SW_REFGEN_SEL_SW_IAIB_MASK)
            #define UL_testing_Tuning_DEFAULT_IDAC_COMP_LEG1_MODE        (UL_testing_Tuning_IDAC_COMP_LEG1_EN_MASK |\
                                                                        ((uint32)UL_testing_Tuning_IDAC_COMP_LEG1_MODE_CSD_STATIC << CYFLD_CSD_LEG1_MODE__OFFSET))
    #else
            #define UL_testing_Tuning_DEFAULT_SW_REFGEN_SEL              (UL_testing_Tuning_SW_REFGEN_VREF_SRC)
            #define UL_testing_Tuning_DEFAULT_IDAC_COMP_LEG1_MODE        ((uint32)UL_testing_Tuning_IDAC_COMP_LEG1_MODE_CSD_STATIC << CYFLD_CSD_LEG1_MODE__OFFSET)
    #endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_IDAC_COMP_EN) && \
               (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_DEDICATED_IDAC_COMP_EN)) */


    #if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_IDAC_COMP_EN) && \
         (UL_testing_Tuning_DISABLE == UL_testing_Tuning_CSD_DEDICATED_IDAC_COMP_EN))
            #define UL_testing_Tuning_DEFAULT_IDAC_COMP_LEG2_MODE        (UL_testing_Tuning_IDAC_COMP_LEG2_EN_MASK |\
                                                                        ((uint32)UL_testing_Tuning_IDAC_COMP_LEG2_MODE_CSD_STATIC << CYFLD_CSD_LEG2_MODE__OFFSET))
    #else
            #define UL_testing_Tuning_DEFAULT_IDAC_COMP_LEG2_MODE        ((uint32)UL_testing_Tuning_IDAC_COMP_LEG2_MODE_GP_STATIC << CYFLD_CSD_LEG2_MODE__OFFSET)
    #endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_IDAC_COMP_EN) && \
            (UL_testing_Tuning_DISABLE == UL_testing_Tuning_CSD_DEDICATED_IDAC_COMP_EN)) */

    /* IDACs register configuration is based on the Component configuration */
    #define UL_testing_Tuning_IDAC_MOD_DEFAULT_CFG                       (UL_testing_Tuning_DEFAULT_IDAC_MOD_RANGE | \
                                                                         UL_testing_Tuning_DEFAULT_IDAC_MOD_POLARITY | \
                                                                         UL_testing_Tuning_DEFAULT_IDAC_MOD_BALL_MODE | \
                                                                        ((uint32)(UL_testing_Tuning_IDAC_MOD_LEG1_MODE_CSD << CYFLD_CSD_LEG1_MODE__OFFSET)) | \
                                                                         UL_testing_Tuning_IDAC_MOD_LEG1_EN_MASK | \
                                                                         UL_testing_Tuning_DEFAULT_IDAC_COMP_LEG2_MODE)

    #define UL_testing_Tuning_IDAC_COMP_DEFAULT_CFG                      (UL_testing_Tuning_DEFAULT_IDAC_COMP_RANGE | \
                                                                         UL_testing_Tuning_DEFAULT_IDAC_COMP_POLARITY | \
                                                                         UL_testing_Tuning_DEFAULT_IDAC_COMP_BALL_MODE | \
                                                                         UL_testing_Tuning_DEFAULT_IDAC_COMP_LEG1_MODE | \
                                                                        ((uint32)(UL_testing_Tuning_IDAC_COMP_LEG2_MODE_GP_STATIC << CYFLD_CSD_LEG2_MODE__OFFSET)))

    #define UL_testing_Tuning_IDAC_MOD_CALIBRATION_CFG                   ((uint32)(UL_testing_Tuning_DEFAULT_IDAC_MOD_RANGE | \
                                                                         UL_testing_Tuning_DEFAULT_IDAC_MOD_POLARITY | \
                                                                         UL_testing_Tuning_DEFAULT_IDAC_MOD_BALL_MODE | \
                                                                        ((uint32)(UL_testing_Tuning_IDAC_MOD_LEG1_MODE_CSD << CYFLD_CSD_LEG1_MODE__OFFSET)) | \
                                                                         UL_testing_Tuning_IDAC_MOD_LEG1_EN_MASK | \
                                                                        ((uint32)((uint32)UL_testing_Tuning_IDAC_MOD_LEG2_MODE_GP_STATIC << CYFLD_CSD_LEG2_MODE__OFFSET))))

    #define UL_testing_Tuning_IDAC_COMP_CALIBRATION_CFG                  ((uint32)(UL_testing_Tuning_DEFAULT_IDAC_COMP_RANGE | \
                                                                         UL_testing_Tuning_DEFAULT_IDAC_COMP_POLARITY | \
                                                                         UL_testing_Tuning_DEFAULT_IDAC_COMP_BALL_MODE | \
                                                                        ((uint32)((uint32)UL_testing_Tuning_IDAC_COMP_LEG1_MODE_GP_STATIC << CYFLD_CSD_LEG1_MODE__OFFSET)) | \
                                                                        ((uint32)((uint32)UL_testing_Tuning_IDAC_COMP_LEG2_MODE_GP_STATIC << CYFLD_CSD_LEG2_MODE__OFFSET))))
#else
    #define UL_testing_Tuning_CSD_IDAC_MOD_BITS_USED                     (8u)

    #define UL_testing_Tuning_IDAC_MOD_CFG_MASK                  (UL_testing_Tuning_IDAC_POLARITY1_MIR_MASK |\
                                                                UL_testing_Tuning_IDAC_MOD_RANGE_MASK |\
                                                                UL_testing_Tuning_IDAC_MOD_MODE_MASK |\
                                                                UL_testing_Tuning_IDAC_MOD_MASK)

    #define UL_testing_Tuning_IDAC_COMP_CFG_MASK                 (UL_testing_Tuning_IDAC_POLARITY2_MIR_MASK |\
                                                                UL_testing_Tuning_IDAC_COMP_RANGE_MASK |\
                                                                UL_testing_Tuning_IDAC_COMP_MODE_MASK |\
                                                                UL_testing_Tuning_IDAC_COMP_MASK)

        #define UL_testing_Tuning_PRS_8_CONFIG                       UL_testing_Tuning_CONFIG_PRS_SELECT_MASK
    #define UL_testing_Tuning_PRS_12_CONFIG                      (UL_testing_Tuning_CONFIG_PRS_12_8_MASK |\
                                                                UL_testing_Tuning_CONFIG_PRS_SELECT_MASK)

    /* Third-generation HW block Initial PRS mode */
    #if (UL_testing_Tuning_CLK_SOURCE_PRS8 == UL_testing_Tuning_CSD_SNS_CLK_SOURCE)
        #define UL_testing_Tuning_DEFAULT_MODULATION_MODE        UL_testing_Tuning_CONFIG_PRS_SELECT_MASK

    #elif (UL_testing_Tuning_CLK_SOURCE_PRS12 == UL_testing_Tuning_CSD_SNS_CLK_SOURCE)
        #define UL_testing_Tuning_DEFAULT_MODULATION_MODE        (UL_testing_Tuning_CONFIG_PRS_12_8_MASK |\
                                                                UL_testing_Tuning_CONFIG_PRS_SELECT_MASK)
    #else
        #define UL_testing_Tuning_DEFAULT_MODULATION_MODE        (0u)
    #endif /* (UL_testing_Tuning_CSD_SNS_CLK_SOURCE == UL_testing_Tuning_PRS_8BITS) */

    /* Third-generation HW block Set IDAC polarity */
    #if (UL_testing_Tuning_CSD_IDAC_CONFIG == UL_testing_Tuning_IDAC_SINKING)
        #define UL_testing_Tuning_DEFAULT_IDAC_POLARITY          UL_testing_Tuning_CONFIG_POLARITY_MASK
        #define UL_testing_Tuning_CSH_DR_CONFIG                  UL_testing_Tuning_CTANK_DR_VDDIO
    #else
        #define UL_testing_Tuning_DEFAULT_IDAC_POLARITY          (0u)
        #define UL_testing_Tuning_CSH_DR_CONFIG                  UL_testing_Tuning_CTANK_DR_VSSIO
    #endif /* (UL_testing_Tuning_CSD_IDAC_CONFIG == UL_testing_Tuning_IDAC_SINKING) */

    /* Defining default CSD configuration according to settings in customizer. */
    #define UL_testing_Tuning_DEFAULT_CSD_CONFIG                 (UL_testing_Tuning_CONFIG_SENSE_COMP_BW_MASK |\
                                                                UL_testing_Tuning_DEFAULT_IDAC_POLARITY |\
                                                                UL_testing_Tuning_CONFIG_SENSE_INSEL_MASK |\
                                                                UL_testing_Tuning_CONFIG_REFBUF_DRV_MASK)

    #define UL_testing_Tuning_CSD_ENABLE_MASK                    (UL_testing_Tuning_CONFIG_ENABLE_MASK |\
                                                                UL_testing_Tuning_CONFIG_SENSE_EN_MASK |\
                                                                UL_testing_Tuning_CONFIG_SENSE_COMP_EN_MASK)

    /* Third-generation HW block Defining mask intended for clearing bits related to pre-charging options. */
    #define UL_testing_Tuning_PRECHARGE_CONFIG_MASK              (UL_testing_Tuning_CONFIG_REFBUF_EN_MASK |\
                                                                UL_testing_Tuning_CONFIG_COMP_MODE_MASK |\
                                                                UL_testing_Tuning_CONFIG_COMP_PIN_MASK  |\
                                                                UL_testing_Tuning_CONFIG_REFBUF_OUTSEL_MASK)

    #define UL_testing_Tuning_CMOD_PRECHARGE_CONFIG              (UL_testing_Tuning_DEFAULT_CSD_CONFIG |\
                                                                UL_testing_Tuning_CONFIG_REFBUF_EN_MASK |\
                                                                UL_testing_Tuning_CONFIG_COMP_PIN_MASK)

    #define UL_testing_Tuning_CMOD_PRECHARGE_CONFIG_CSD_EN       (UL_testing_Tuning_DEFAULT_CSD_CONFIG |\
                                                                UL_testing_Tuning_CSD_ENABLE_MASK |\
                                                                UL_testing_Tuning_CONFIG_REFBUF_EN_MASK |\
                                                                UL_testing_Tuning_CONFIG_COMP_PIN_MASK)


    /* Third-generation HW block Ctank pre-charge mode configuration */
    #if(UL_testing_Tuning_CSD_CSH_PRECHARGE_SRC == UL_testing_Tuning_CSH_PRECHARGE_VREF)
        #if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN)
            #define  UL_testing_Tuning_CTANK_PRECHARGE_CONFIG    (UL_testing_Tuning_DEFAULT_CSD_CONFIG |\
                                                                 UL_testing_Tuning_CONFIG_REFBUF_EN_MASK |\
                                                                 UL_testing_Tuning_CONFIG_PRS_CLEAR_MASK |\
                                                                 UL_testing_Tuning_CONFIG_REFBUF_OUTSEL_MASK)
        #else
            #define  UL_testing_Tuning_CTANK_PRECHARGE_CONFIG    (UL_testing_Tuning_DEFAULT_CSD_CONFIG |\
                                                                 UL_testing_Tuning_CONFIG_REFBUF_OUTSEL_MASK |\
                                                                 UL_testing_Tuning_CONFIG_PRS_CLEAR_MASK)
        #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) */
    #else
        #define  UL_testing_Tuning_CTANK_PRECHARGE_CONFIG        (UL_testing_Tuning_DEFAULT_CSD_CONFIG |\
                                                                UL_testing_Tuning_CONFIG_REFBUF_OUTSEL_MASK |\
                                                                UL_testing_Tuning_CONFIG_REFBUF_EN_MASK |\
                                                                UL_testing_Tuning_CONFIG_COMP_MODE_MASK |\
                                                                UL_testing_Tuning_CONFIG_PRS_CLEAR_MASK |\
                                                                UL_testing_Tuning_CONFIG_COMP_PIN_MASK)
    #endif /* (UL_testing_Tuning_CSD_CSH_PRECHARGE_SRC == UL_testing_Tuning__CSH_PRECHARGE_IO_BUF) */

    #define  UL_testing_Tuning_CTANK_PRECHARGE_CONFIG_CSD_EN     (UL_testing_Tuning_CTANK_PRECHARGE_CONFIG |\
                                                                 UL_testing_Tuning_CONFIG_ENABLE_MASK |\
                                                                 UL_testing_Tuning_CONFIG_SENSE_COMP_EN_MASK)

#endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2) */

/* Calibration constants */
#define UL_testing_Tuning_IDAC_MOD_MAX_CALIB_ERROR                       (10u)
#define UL_testing_Tuning_CAL_MIDDLE_BIT                                 (1uL << (UL_testing_Tuning_CSD_IDAC_MOD_BITS_USED - 1u))

#define UL_testing_Tuning_CSD_AVG_CYCLES_PER_LOOP                   (5u)
#define UL_testing_Tuning_CSD_MEASURE_MAX_TIME_US                   (6000u)
#define UL_testing_Tuning_CSD_PRECHARGE_MAX_TIME_US                 (250u)

#define UL_testing_Tuning_CSD_CALIBR_WATCHDOG_CYCLES_NUM            (((CYDEV_BCLK__SYSCLK__MHZ) * (UL_testing_Tuning_CSD_MEASURE_MAX_TIME_US)) /\
                                                                    (UL_testing_Tuning_CSD_AVG_CYCLES_PER_LOOP))
#define UL_testing_Tuning_CSD_PRECHARGE_WATCHDOG_CYCLES_NUM         (((CYDEV_BCLK__SYSCLK__MHZ) * (UL_testing_Tuning_CSD_MEASURE_MAX_TIME_US)) /\
                                                                    (UL_testing_Tuning_CSD_AVG_CYCLES_PER_LOOP))

/***************************************
* Global software variables
***************************************/

extern uint32 UL_testing_Tuning_configCsd;

#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_NOISE_METRIC_EN)
    extern uint8 UL_testing_Tuning_badConversionsNum;
#endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_NOISE_METRIC_EN) */

#if (UL_testing_Tuning_CSD_SS_DIS != UL_testing_Tuning_CSD_AUTOTUNE)
    /* Stores IDAC and raw count that corresponds to a sensor with maximum Cp within a widget */
    extern uint8 UL_testing_Tuning_calibratedIdac;
    extern uint16 UL_testing_Tuning_calibratedRawcount;
    #if (UL_testing_Tuning_CSD_MATRIX_WIDGET_EN || UL_testing_Tuning_CSD_TOUCHPAD_WIDGET_EN)
        extern uint8 UL_testing_Tuning_calibratedIdacRow;
        extern uint16 UL_testing_Tuning_calibratedRawcountRow;
    #endif /*(UL_testing_Tuning_CSD_MATRIX_WIDGET_EN || UL_testing_Tuning_CSD_TOUCHPAD_WIDGET_EN) */
#endif /* (UL_testing_Tuning_CSD_SS_DIS != UL_testing_Tuning_CSD_AUTOTUNE) */

/***************************************
* Function Prototypes
**************************************/

/**
* \cond SECTION_C_LOW_LEVEL
* \addtogroup group_c_low_level
* \{
*/

void UL_testing_Tuning_CSDSetupWidget(uint32 widgetId);
void UL_testing_Tuning_CSDSetupWidgetExt(uint32 widgetId, uint32 sensorId);
void UL_testing_Tuning_CSDScan(void);
void UL_testing_Tuning_CSDScanExt(void);
#if ((UL_testing_Tuning_CSD_SS_DIS != UL_testing_Tuning_CSD_AUTOTUNE) || \
     (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_IDAC_AUTOCAL_EN))
    cystatus UL_testing_Tuning_CSDCalibrateWidget(uint32 widgetId, uint32 target);
#endif /* ((UL_testing_Tuning_CSD_SS_DIS != UL_testing_Tuning_CSD_AUTOTUNE) || \
           (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_IDAC_AUTOCAL_EN))  */
void UL_testing_Tuning_CSDConnectSns(UL_testing_Tuning_FLASH_IO_STRUCT const *snsAddrPtr);
void UL_testing_Tuning_CSDDisconnectSns(UL_testing_Tuning_FLASH_IO_STRUCT const *snsAddrPtr);

/** \}
* \endcond */

/*****************************************************
* Function Prototypes - internal Low Level functions
*****************************************************/

/**
* \cond SECTION_C_INTERNAL
* \addtogroup group_c_internal
* \{
*/

void UL_testing_Tuning_SsCSDInitialize(void);
void UL_testing_Tuning_SsCSDStartSample(void);
void UL_testing_Tuning_SsCSDSetUpIdacs(UL_testing_Tuning_RAM_WD_BASE_STRUCT const *ptrWdgt);
void UL_testing_Tuning_SsCSDConfigClock(void);
void UL_testing_Tuning_SsCSDElectrodeCheck(void);
#if ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) && \
     (0u != UL_testing_Tuning_CSD_TOTAL_SHIELD_COUNT))
    void UL_testing_Tuning_SsCSDDisableShieldElectrodes(void);
#endif /* ((UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_SHIELD_EN) && \
           (0u != UL_testing_Tuning_CSD_TOTAL_SHIELD_COUNT)) */
#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2)
    uint32 UL_testing_Tuning_SsCSDGetNumberOfConversions(uint32 snsClkDivider, uint32 resolution, uint32 snsClkSrc);
#endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2) */
void UL_testing_Tuning_SsCSDCalculateScanDuration(UL_testing_Tuning_RAM_WD_BASE_STRUCT const *ptrWdgt);
void UL_testing_Tuning_SsCSDConnectSensorExt(uint32 widgetId, uint32 sensorId);
void UL_testing_Tuning_SsCSDDisconnectSnsExt(uint32 widgetId, uint32 sensorId);

#if ((UL_testing_Tuning_CSD_SS_DIS != UL_testing_Tuning_CSD_AUTOTUNE) || \
     (UL_testing_Tuning_ENABLE == UL_testing_Tuning_SELF_TEST_EN) || \
     (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_IDAC_AUTOCAL_EN))
#endif /* ((UL_testing_Tuning_CSD_SS_DIS != UL_testing_Tuning_CSD_AUTOTUNE) || \
           (UL_testing_Tuning_ENABLE == UL_testing_Tuning_SELF_TEST_EN) || \
           (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_IDAC_AUTOCAL_EN)) */

/** \}
* \endcond */

/***************************************
* Global software variables
***************************************/
extern uint32 UL_testing_Tuning_configCsd;

/* Interrupt handler */
extern CY_ISR_PROTO(UL_testing_Tuning_CSDPostSingleScan);
extern CY_ISR_PROTO(UL_testing_Tuning_CSDPostMultiScan);
#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_GANGED_SNS_EN)
extern CY_ISR_PROTO(UL_testing_Tuning_CSDPostMultiScanGanged);
#endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_GANGED_SNS_EN) */
#if (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_NOISE_METRIC_EN)
    extern uint8 UL_testing_Tuning_badConversionsNum;
#endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSD_NOISE_METRIC_EN) */

#endif /* End CY_SENSE_UL_testing_Tuning_SENSINGCSD_LL_H */


/* [] END OF FILE */
