/***************************************************************************//**
* \file UL_testing_Tuning_RegisterMap.h
* \version 6.0
*
* \brief
*   This file provides the definitions for the Component data structure register.
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

#if !defined(CY_SENSE_UL_testing_Tuning_REGISTER_MAP_H)
#define CY_SENSE_UL_testing_Tuning_REGISTER_MAP_H

#include <cytypes.h>
#include "UL_testing_Tuning_Configuration.h"
#include "UL_testing_Tuning_Structure.h"

/*****************************************************************************/
/* RAM Data structure register definitions                                   */
/*****************************************************************************/
#define UL_testing_Tuning_CONFIG_ID_VALUE                   (UL_testing_Tuning_dsRam.configId)
#define UL_testing_Tuning_CONFIG_ID_OFFSET                  (0u)
#define UL_testing_Tuning_CONFIG_ID_SIZE                    (2u)
#define UL_testing_Tuning_CONFIG_ID_PARAM_ID                (0x87000000u)

#define UL_testing_Tuning_DEVICE_ID_VALUE                   (UL_testing_Tuning_dsRam.deviceId)
#define UL_testing_Tuning_DEVICE_ID_OFFSET                  (2u)
#define UL_testing_Tuning_DEVICE_ID_SIZE                    (2u)
#define UL_testing_Tuning_DEVICE_ID_PARAM_ID                (0x8B000002u)

#define UL_testing_Tuning_HW_CLOCK_VALUE                    (UL_testing_Tuning_dsRam.hwClock)
#define UL_testing_Tuning_HW_CLOCK_OFFSET                   (4u)
#define UL_testing_Tuning_HW_CLOCK_SIZE                     (2u)
#define UL_testing_Tuning_HW_CLOCK_PARAM_ID                 (0x86000004u)

#define UL_testing_Tuning_TUNER_CMD_VALUE                   (UL_testing_Tuning_dsRam.tunerCmd)
#define UL_testing_Tuning_TUNER_CMD_OFFSET                  (6u)
#define UL_testing_Tuning_TUNER_CMD_SIZE                    (2u)
#define UL_testing_Tuning_TUNER_CMD_PARAM_ID                (0xA1000006u)

#define UL_testing_Tuning_SCAN_COUNTER_VALUE                (UL_testing_Tuning_dsRam.scanCounter)
#define UL_testing_Tuning_SCAN_COUNTER_OFFSET               (8u)
#define UL_testing_Tuning_SCAN_COUNTER_SIZE                 (2u)
#define UL_testing_Tuning_SCAN_COUNTER_PARAM_ID             (0x85000008u)

#define UL_testing_Tuning_STATUS_VALUE                      (UL_testing_Tuning_dsRam.status)
#define UL_testing_Tuning_STATUS_OFFSET                     (12u)
#define UL_testing_Tuning_STATUS_SIZE                       (4u)
#define UL_testing_Tuning_STATUS_PARAM_ID                   (0xCB00000Cu)

#define UL_testing_Tuning_WDGT_ENABLE0_VALUE                (UL_testing_Tuning_dsRam.wdgtEnable[0u])
#define UL_testing_Tuning_WDGT_ENABLE0_OFFSET               (16u)
#define UL_testing_Tuning_WDGT_ENABLE0_SIZE                 (4u)
#define UL_testing_Tuning_WDGT_ENABLE0_PARAM_ID             (0xE6000010u)

#define UL_testing_Tuning_WDGT_WORKING0_VALUE               (UL_testing_Tuning_dsRam.wdgtWorking[0u])
#define UL_testing_Tuning_WDGT_WORKING0_OFFSET              (20u)
#define UL_testing_Tuning_WDGT_WORKING0_SIZE                (4u)
#define UL_testing_Tuning_WDGT_WORKING0_PARAM_ID            (0xCC000014u)

#define UL_testing_Tuning_WDGT_STATUS0_VALUE                (UL_testing_Tuning_dsRam.wdgtStatus[0u])
#define UL_testing_Tuning_WDGT_STATUS0_OFFSET               (24u)
#define UL_testing_Tuning_WDGT_STATUS0_SIZE                 (4u)
#define UL_testing_Tuning_WDGT_STATUS0_PARAM_ID             (0xCF000018u)

#define UL_testing_Tuning_SNS_STATUS0_VALUE                 (UL_testing_Tuning_dsRam.snsStatus[0u])
#define UL_testing_Tuning_SNS_STATUS0_OFFSET                (28u)
#define UL_testing_Tuning_SNS_STATUS0_SIZE                  (1u)
#define UL_testing_Tuning_SNS_STATUS0_PARAM_ID              (0x4900001Cu)

#define UL_testing_Tuning_SNS_STATUS1_VALUE                 (UL_testing_Tuning_dsRam.snsStatus[1u])
#define UL_testing_Tuning_SNS_STATUS1_OFFSET                (29u)
#define UL_testing_Tuning_SNS_STATUS1_SIZE                  (1u)
#define UL_testing_Tuning_SNS_STATUS1_PARAM_ID              (0x4F00001Du)

#define UL_testing_Tuning_CSD0_CONFIG_VALUE                 (UL_testing_Tuning_dsRam.csd0Config)
#define UL_testing_Tuning_CSD0_CONFIG_OFFSET                (30u)
#define UL_testing_Tuning_CSD0_CONFIG_SIZE                  (2u)
#define UL_testing_Tuning_CSD0_CONFIG_PARAM_ID              (0xAB80001Eu)

#define UL_testing_Tuning_MOD_CSD_CLK_VALUE                 (UL_testing_Tuning_dsRam.modCsdClk)
#define UL_testing_Tuning_MOD_CSD_CLK_OFFSET                (32u)
#define UL_testing_Tuning_MOD_CSD_CLK_SIZE                  (1u)
#define UL_testing_Tuning_MOD_CSD_CLK_PARAM_ID              (0x63800020u)

#define UL_testing_Tuning_GLB_CRC_VALUE                     (UL_testing_Tuning_dsRam.glbCrc)
#define UL_testing_Tuning_GLB_CRC_OFFSET                    (34u)
#define UL_testing_Tuning_GLB_CRC_SIZE                      (2u)
#define UL_testing_Tuning_GLB_CRC_PARAM_ID                  (0x81000022u)

#define UL_testing_Tuning_BUTTON0_CRC_VALUE                 (UL_testing_Tuning_dsRam.wdgtList.button0.crc)
#define UL_testing_Tuning_BUTTON0_CRC_OFFSET                (36u)
#define UL_testing_Tuning_BUTTON0_CRC_SIZE                  (2u)
#define UL_testing_Tuning_BUTTON0_CRC_PARAM_ID              (0x8C000024u)

#define UL_testing_Tuning_BUTTON0_RESOLUTION_VALUE          (UL_testing_Tuning_dsRam.wdgtList.button0.resolution)
#define UL_testing_Tuning_BUTTON0_RESOLUTION_OFFSET         (38u)
#define UL_testing_Tuning_BUTTON0_RESOLUTION_SIZE           (2u)
#define UL_testing_Tuning_BUTTON0_RESOLUTION_PARAM_ID       (0xA6800026u)

#define UL_testing_Tuning_BUTTON0_FINGER_TH_VALUE           (UL_testing_Tuning_dsRam.wdgtList.button0.fingerTh)
#define UL_testing_Tuning_BUTTON0_FINGER_TH_OFFSET          (40u)
#define UL_testing_Tuning_BUTTON0_FINGER_TH_SIZE            (2u)
#define UL_testing_Tuning_BUTTON0_FINGER_TH_PARAM_ID        (0xA9800028u)

#define UL_testing_Tuning_BUTTON0_NOISE_TH_VALUE            (UL_testing_Tuning_dsRam.wdgtList.button0.noiseTh)
#define UL_testing_Tuning_BUTTON0_NOISE_TH_OFFSET           (42u)
#define UL_testing_Tuning_BUTTON0_NOISE_TH_SIZE             (1u)
#define UL_testing_Tuning_BUTTON0_NOISE_TH_PARAM_ID         (0x6D80002Au)

#define UL_testing_Tuning_BUTTON0_NNOISE_TH_VALUE           (UL_testing_Tuning_dsRam.wdgtList.button0.nNoiseTh)
#define UL_testing_Tuning_BUTTON0_NNOISE_TH_OFFSET          (43u)
#define UL_testing_Tuning_BUTTON0_NNOISE_TH_SIZE            (1u)
#define UL_testing_Tuning_BUTTON0_NNOISE_TH_PARAM_ID        (0x6B80002Bu)

#define UL_testing_Tuning_BUTTON0_HYSTERESIS_VALUE          (UL_testing_Tuning_dsRam.wdgtList.button0.hysteresis)
#define UL_testing_Tuning_BUTTON0_HYSTERESIS_OFFSET         (44u)
#define UL_testing_Tuning_BUTTON0_HYSTERESIS_SIZE           (1u)
#define UL_testing_Tuning_BUTTON0_HYSTERESIS_PARAM_ID       (0x6080002Cu)

#define UL_testing_Tuning_BUTTON0_ON_DEBOUNCE_VALUE         (UL_testing_Tuning_dsRam.wdgtList.button0.onDebounce)
#define UL_testing_Tuning_BUTTON0_ON_DEBOUNCE_OFFSET        (45u)
#define UL_testing_Tuning_BUTTON0_ON_DEBOUNCE_SIZE          (1u)
#define UL_testing_Tuning_BUTTON0_ON_DEBOUNCE_PARAM_ID      (0x6680002Du)

#define UL_testing_Tuning_BUTTON0_LOW_BSLN_RST_VALUE        (UL_testing_Tuning_dsRam.wdgtList.button0.lowBslnRst)
#define UL_testing_Tuning_BUTTON0_LOW_BSLN_RST_OFFSET       (46u)
#define UL_testing_Tuning_BUTTON0_LOW_BSLN_RST_SIZE         (1u)
#define UL_testing_Tuning_BUTTON0_LOW_BSLN_RST_PARAM_ID     (0x6C80002Eu)

#define UL_testing_Tuning_BUTTON0_IDAC_MOD0_VALUE           (UL_testing_Tuning_dsRam.wdgtList.button0.idacMod[0u])
#define UL_testing_Tuning_BUTTON0_IDAC_MOD0_OFFSET          (47u)
#define UL_testing_Tuning_BUTTON0_IDAC_MOD0_SIZE            (1u)
#define UL_testing_Tuning_BUTTON0_IDAC_MOD0_PARAM_ID        (0x4C00002Fu)

#define UL_testing_Tuning_BUTTON0_SNS_CLK_VALUE             (UL_testing_Tuning_dsRam.wdgtList.button0.snsClk)
#define UL_testing_Tuning_BUTTON0_SNS_CLK_OFFSET            (48u)
#define UL_testing_Tuning_BUTTON0_SNS_CLK_SIZE              (2u)
#define UL_testing_Tuning_BUTTON0_SNS_CLK_PARAM_ID          (0xAE800030u)

#define UL_testing_Tuning_BUTTON0_SNS_CLK_SOURCE_VALUE      (UL_testing_Tuning_dsRam.wdgtList.button0.snsClkSource)
#define UL_testing_Tuning_BUTTON0_SNS_CLK_SOURCE_OFFSET     (50u)
#define UL_testing_Tuning_BUTTON0_SNS_CLK_SOURCE_SIZE       (1u)
#define UL_testing_Tuning_BUTTON0_SNS_CLK_SOURCE_PARAM_ID   (0x41800032u)

#define UL_testing_Tuning_BUTTON1_CRC_VALUE                 (UL_testing_Tuning_dsRam.wdgtList.button1.crc)
#define UL_testing_Tuning_BUTTON1_CRC_OFFSET                (52u)
#define UL_testing_Tuning_BUTTON1_CRC_SIZE                  (2u)
#define UL_testing_Tuning_BUTTON1_CRC_PARAM_ID              (0x8A010034u)

#define UL_testing_Tuning_BUTTON1_RESOLUTION_VALUE          (UL_testing_Tuning_dsRam.wdgtList.button1.resolution)
#define UL_testing_Tuning_BUTTON1_RESOLUTION_OFFSET         (54u)
#define UL_testing_Tuning_BUTTON1_RESOLUTION_SIZE           (2u)
#define UL_testing_Tuning_BUTTON1_RESOLUTION_PARAM_ID       (0xA0810036u)

#define UL_testing_Tuning_BUTTON1_FINGER_TH_VALUE           (UL_testing_Tuning_dsRam.wdgtList.button1.fingerTh)
#define UL_testing_Tuning_BUTTON1_FINGER_TH_OFFSET          (56u)
#define UL_testing_Tuning_BUTTON1_FINGER_TH_SIZE            (2u)
#define UL_testing_Tuning_BUTTON1_FINGER_TH_PARAM_ID        (0xAF810038u)

#define UL_testing_Tuning_BUTTON1_NOISE_TH_VALUE            (UL_testing_Tuning_dsRam.wdgtList.button1.noiseTh)
#define UL_testing_Tuning_BUTTON1_NOISE_TH_OFFSET           (58u)
#define UL_testing_Tuning_BUTTON1_NOISE_TH_SIZE             (1u)
#define UL_testing_Tuning_BUTTON1_NOISE_TH_PARAM_ID         (0x6B81003Au)

#define UL_testing_Tuning_BUTTON1_NNOISE_TH_VALUE           (UL_testing_Tuning_dsRam.wdgtList.button1.nNoiseTh)
#define UL_testing_Tuning_BUTTON1_NNOISE_TH_OFFSET          (59u)
#define UL_testing_Tuning_BUTTON1_NNOISE_TH_SIZE            (1u)
#define UL_testing_Tuning_BUTTON1_NNOISE_TH_PARAM_ID        (0x6D81003Bu)

#define UL_testing_Tuning_BUTTON1_HYSTERESIS_VALUE          (UL_testing_Tuning_dsRam.wdgtList.button1.hysteresis)
#define UL_testing_Tuning_BUTTON1_HYSTERESIS_OFFSET         (60u)
#define UL_testing_Tuning_BUTTON1_HYSTERESIS_SIZE           (1u)
#define UL_testing_Tuning_BUTTON1_HYSTERESIS_PARAM_ID       (0x6681003Cu)

#define UL_testing_Tuning_BUTTON1_ON_DEBOUNCE_VALUE         (UL_testing_Tuning_dsRam.wdgtList.button1.onDebounce)
#define UL_testing_Tuning_BUTTON1_ON_DEBOUNCE_OFFSET        (61u)
#define UL_testing_Tuning_BUTTON1_ON_DEBOUNCE_SIZE          (1u)
#define UL_testing_Tuning_BUTTON1_ON_DEBOUNCE_PARAM_ID      (0x6081003Du)

#define UL_testing_Tuning_BUTTON1_LOW_BSLN_RST_VALUE        (UL_testing_Tuning_dsRam.wdgtList.button1.lowBslnRst)
#define UL_testing_Tuning_BUTTON1_LOW_BSLN_RST_OFFSET       (62u)
#define UL_testing_Tuning_BUTTON1_LOW_BSLN_RST_SIZE         (1u)
#define UL_testing_Tuning_BUTTON1_LOW_BSLN_RST_PARAM_ID     (0x6A81003Eu)

#define UL_testing_Tuning_BUTTON1_IDAC_MOD0_VALUE           (UL_testing_Tuning_dsRam.wdgtList.button1.idacMod[0u])
#define UL_testing_Tuning_BUTTON1_IDAC_MOD0_OFFSET          (63u)
#define UL_testing_Tuning_BUTTON1_IDAC_MOD0_SIZE            (1u)
#define UL_testing_Tuning_BUTTON1_IDAC_MOD0_PARAM_ID        (0x4A01003Fu)

#define UL_testing_Tuning_BUTTON1_SNS_CLK_VALUE             (UL_testing_Tuning_dsRam.wdgtList.button1.snsClk)
#define UL_testing_Tuning_BUTTON1_SNS_CLK_OFFSET            (64u)
#define UL_testing_Tuning_BUTTON1_SNS_CLK_SIZE              (2u)
#define UL_testing_Tuning_BUTTON1_SNS_CLK_PARAM_ID          (0xAF810040u)

#define UL_testing_Tuning_BUTTON1_SNS_CLK_SOURCE_VALUE      (UL_testing_Tuning_dsRam.wdgtList.button1.snsClkSource)
#define UL_testing_Tuning_BUTTON1_SNS_CLK_SOURCE_OFFSET     (66u)
#define UL_testing_Tuning_BUTTON1_SNS_CLK_SOURCE_SIZE       (1u)
#define UL_testing_Tuning_BUTTON1_SNS_CLK_SOURCE_PARAM_ID   (0x40810042u)

#define UL_testing_Tuning_BUTTON0_SNS0_RAW0_VALUE           (UL_testing_Tuning_dsRam.snsList.button0[0u].raw[0u])
#define UL_testing_Tuning_BUTTON0_SNS0_RAW0_OFFSET          (68u)
#define UL_testing_Tuning_BUTTON0_SNS0_RAW0_SIZE            (2u)
#define UL_testing_Tuning_BUTTON0_SNS0_RAW0_PARAM_ID        (0x8B000044u)

#define UL_testing_Tuning_BUTTON0_SNS0_BSLN0_VALUE          (UL_testing_Tuning_dsRam.snsList.button0[0u].bsln[0u])
#define UL_testing_Tuning_BUTTON0_SNS0_BSLN0_OFFSET         (70u)
#define UL_testing_Tuning_BUTTON0_SNS0_BSLN0_SIZE           (2u)
#define UL_testing_Tuning_BUTTON0_SNS0_BSLN0_PARAM_ID       (0x87000046u)

#define UL_testing_Tuning_BUTTON0_SNS0_BSLN_INV0_VALUE      (UL_testing_Tuning_dsRam.snsList.button0[0u].bslnInv[0u])
#define UL_testing_Tuning_BUTTON0_SNS0_BSLN_INV0_OFFSET     (72u)
#define UL_testing_Tuning_BUTTON0_SNS0_BSLN_INV0_SIZE       (2u)
#define UL_testing_Tuning_BUTTON0_SNS0_BSLN_INV0_PARAM_ID   (0x88000048u)

#define UL_testing_Tuning_BUTTON0_SNS0_BSLN_EXT0_VALUE      (UL_testing_Tuning_dsRam.snsList.button0[0u].bslnExt[0u])
#define UL_testing_Tuning_BUTTON0_SNS0_BSLN_EXT0_OFFSET     (74u)
#define UL_testing_Tuning_BUTTON0_SNS0_BSLN_EXT0_SIZE       (1u)
#define UL_testing_Tuning_BUTTON0_SNS0_BSLN_EXT0_PARAM_ID   (0x4C00004Au)

#define UL_testing_Tuning_BUTTON0_SNS0_DIFF_VALUE           (UL_testing_Tuning_dsRam.snsList.button0[0u].diff)
#define UL_testing_Tuning_BUTTON0_SNS0_DIFF_OFFSET          (76u)
#define UL_testing_Tuning_BUTTON0_SNS0_DIFF_SIZE            (2u)
#define UL_testing_Tuning_BUTTON0_SNS0_DIFF_PARAM_ID        (0x8900004Cu)

#define UL_testing_Tuning_BUTTON0_SNS0_NEG_BSLN_RST_CNT0_VALUE (UL_testing_Tuning_dsRam.snsList.button0[0u].negBslnRstCnt[0u])
#define UL_testing_Tuning_BUTTON0_SNS0_NEG_BSLN_RST_CNT0_OFFSET (78u)
#define UL_testing_Tuning_BUTTON0_SNS0_NEG_BSLN_RST_CNT0_SIZE (1u)
#define UL_testing_Tuning_BUTTON0_SNS0_NEG_BSLN_RST_CNT0_PARAM_ID (0x4D00004Eu)

#define UL_testing_Tuning_BUTTON0_SNS0_IDAC_COMP0_VALUE     (UL_testing_Tuning_dsRam.snsList.button0[0u].idacComp[0u])
#define UL_testing_Tuning_BUTTON0_SNS0_IDAC_COMP0_OFFSET    (79u)
#define UL_testing_Tuning_BUTTON0_SNS0_IDAC_COMP0_SIZE      (1u)
#define UL_testing_Tuning_BUTTON0_SNS0_IDAC_COMP0_PARAM_ID  (0x4B00004Fu)

#define UL_testing_Tuning_BUTTON1_SNS0_RAW0_VALUE           (UL_testing_Tuning_dsRam.snsList.button1[0u].raw[0u])
#define UL_testing_Tuning_BUTTON1_SNS0_RAW0_OFFSET          (80u)
#define UL_testing_Tuning_BUTTON1_SNS0_RAW0_SIZE            (2u)
#define UL_testing_Tuning_BUTTON1_SNS0_RAW0_PARAM_ID        (0x8F000050u)

#define UL_testing_Tuning_BUTTON1_SNS0_BSLN0_VALUE          (UL_testing_Tuning_dsRam.snsList.button1[0u].bsln[0u])
#define UL_testing_Tuning_BUTTON1_SNS0_BSLN0_OFFSET         (82u)
#define UL_testing_Tuning_BUTTON1_SNS0_BSLN0_SIZE           (2u)
#define UL_testing_Tuning_BUTTON1_SNS0_BSLN0_PARAM_ID       (0x83000052u)

#define UL_testing_Tuning_BUTTON1_SNS0_BSLN_INV0_VALUE      (UL_testing_Tuning_dsRam.snsList.button1[0u].bslnInv[0u])
#define UL_testing_Tuning_BUTTON1_SNS0_BSLN_INV0_OFFSET     (84u)
#define UL_testing_Tuning_BUTTON1_SNS0_BSLN_INV0_SIZE       (2u)
#define UL_testing_Tuning_BUTTON1_SNS0_BSLN_INV0_PARAM_ID   (0x8E000054u)

#define UL_testing_Tuning_BUTTON1_SNS0_BSLN_EXT0_VALUE      (UL_testing_Tuning_dsRam.snsList.button1[0u].bslnExt[0u])
#define UL_testing_Tuning_BUTTON1_SNS0_BSLN_EXT0_OFFSET     (86u)
#define UL_testing_Tuning_BUTTON1_SNS0_BSLN_EXT0_SIZE       (1u)
#define UL_testing_Tuning_BUTTON1_SNS0_BSLN_EXT0_PARAM_ID   (0x4A000056u)

#define UL_testing_Tuning_BUTTON1_SNS0_DIFF_VALUE           (UL_testing_Tuning_dsRam.snsList.button1[0u].diff)
#define UL_testing_Tuning_BUTTON1_SNS0_DIFF_OFFSET          (88u)
#define UL_testing_Tuning_BUTTON1_SNS0_DIFF_SIZE            (2u)
#define UL_testing_Tuning_BUTTON1_SNS0_DIFF_PARAM_ID        (0x8D000058u)

#define UL_testing_Tuning_BUTTON1_SNS0_NEG_BSLN_RST_CNT0_VALUE (UL_testing_Tuning_dsRam.snsList.button1[0u].negBslnRstCnt[0u])
#define UL_testing_Tuning_BUTTON1_SNS0_NEG_BSLN_RST_CNT0_OFFSET (90u)
#define UL_testing_Tuning_BUTTON1_SNS0_NEG_BSLN_RST_CNT0_SIZE (1u)
#define UL_testing_Tuning_BUTTON1_SNS0_NEG_BSLN_RST_CNT0_PARAM_ID (0x4900005Au)

#define UL_testing_Tuning_BUTTON1_SNS0_IDAC_COMP0_VALUE     (UL_testing_Tuning_dsRam.snsList.button1[0u].idacComp[0u])
#define UL_testing_Tuning_BUTTON1_SNS0_IDAC_COMP0_OFFSET    (91u)
#define UL_testing_Tuning_BUTTON1_SNS0_IDAC_COMP0_SIZE      (1u)
#define UL_testing_Tuning_BUTTON1_SNS0_IDAC_COMP0_PARAM_ID  (0x4F00005Bu)

#define UL_testing_Tuning_TEST_RESULT_MASK_VALUE            (UL_testing_Tuning_dsRam.selfTest.testResultMask)
#define UL_testing_Tuning_TEST_RESULT_MASK_OFFSET           (92u)
#define UL_testing_Tuning_TEST_RESULT_MASK_SIZE             (4u)
#define UL_testing_Tuning_TEST_RESULT_MASK_PARAM_ID         (0xE800005Cu)

#define UL_testing_Tuning_EXT_CAP0_VALUE                    (UL_testing_Tuning_dsRam.selfTest.extCap[0u])
#define UL_testing_Tuning_EXT_CAP0_OFFSET                   (96u)
#define UL_testing_Tuning_EXT_CAP0_SIZE                     (2u)
#define UL_testing_Tuning_EXT_CAP0_PARAM_ID                 (0xAB000060u)

#define UL_testing_Tuning_EXT_CAP1_VALUE                    (UL_testing_Tuning_dsRam.selfTest.extCap[1u])
#define UL_testing_Tuning_EXT_CAP1_OFFSET                   (98u)
#define UL_testing_Tuning_EXT_CAP1_SIZE                     (2u)
#define UL_testing_Tuning_EXT_CAP1_PARAM_ID                 (0xA7000062u)

#define UL_testing_Tuning_VDDA_VOLTAGE_VALUE                (UL_testing_Tuning_dsRam.selfTest.vddaVoltage)
#define UL_testing_Tuning_VDDA_VOLTAGE_OFFSET               (100u)
#define UL_testing_Tuning_VDDA_VOLTAGE_SIZE                 (2u)
#define UL_testing_Tuning_VDDA_VOLTAGE_PARAM_ID             (0xAA000064u)

#define UL_testing_Tuning_SHIELD_CAP_VALUE                  (UL_testing_Tuning_dsRam.selfTest.shieldCap)
#define UL_testing_Tuning_SHIELD_CAP_OFFSET                 (102u)
#define UL_testing_Tuning_SHIELD_CAP_SIZE                   (2u)
#define UL_testing_Tuning_SHIELD_CAP_PARAM_ID               (0xA6000066u)

#define UL_testing_Tuning_GLB_CRC_CALC_VALUE                (UL_testing_Tuning_dsRam.selfTest.glbCrcCalc)
#define UL_testing_Tuning_GLB_CRC_CALC_OFFSET               (104u)
#define UL_testing_Tuning_GLB_CRC_CALC_SIZE                 (2u)
#define UL_testing_Tuning_GLB_CRC_CALC_PARAM_ID             (0xA9000068u)

#define UL_testing_Tuning_WDGT_CRC_CALC_VALUE               (UL_testing_Tuning_dsRam.selfTest.wdgtCrcCalc)
#define UL_testing_Tuning_WDGT_CRC_CALC_OFFSET              (106u)
#define UL_testing_Tuning_WDGT_CRC_CALC_SIZE                (2u)
#define UL_testing_Tuning_WDGT_CRC_CALC_PARAM_ID            (0xA500006Au)

#define UL_testing_Tuning_WDGT_CRC_ID_VALUE                 (UL_testing_Tuning_dsRam.selfTest.wdgtCrcId)
#define UL_testing_Tuning_WDGT_CRC_ID_OFFSET                (108u)
#define UL_testing_Tuning_WDGT_CRC_ID_SIZE                  (1u)
#define UL_testing_Tuning_WDGT_CRC_ID_PARAM_ID              (0x6000006Cu)

#define UL_testing_Tuning_INV_BSLN_WDGT_ID_VALUE            (UL_testing_Tuning_dsRam.selfTest.invBslnWdgtId)
#define UL_testing_Tuning_INV_BSLN_WDGT_ID_OFFSET           (109u)
#define UL_testing_Tuning_INV_BSLN_WDGT_ID_SIZE             (1u)
#define UL_testing_Tuning_INV_BSLN_WDGT_ID_PARAM_ID         (0x6600006Du)

#define UL_testing_Tuning_INV_BSLN_SNS_ID_VALUE             (UL_testing_Tuning_dsRam.selfTest.invBslnSnsId)
#define UL_testing_Tuning_INV_BSLN_SNS_ID_OFFSET            (110u)
#define UL_testing_Tuning_INV_BSLN_SNS_ID_SIZE              (1u)
#define UL_testing_Tuning_INV_BSLN_SNS_ID_PARAM_ID          (0x6C00006Eu)

#define UL_testing_Tuning_SHORTED_WDGT_ID_VALUE             (UL_testing_Tuning_dsRam.selfTest.shortedWdgtId)
#define UL_testing_Tuning_SHORTED_WDGT_ID_OFFSET            (111u)
#define UL_testing_Tuning_SHORTED_WDGT_ID_SIZE              (1u)
#define UL_testing_Tuning_SHORTED_WDGT_ID_PARAM_ID          (0x6A00006Fu)

#define UL_testing_Tuning_SHORTED_SNS_ID_VALUE              (UL_testing_Tuning_dsRam.selfTest.shortedSnsId)
#define UL_testing_Tuning_SHORTED_SNS_ID_OFFSET             (112u)
#define UL_testing_Tuning_SHORTED_SNS_ID_SIZE               (1u)
#define UL_testing_Tuning_SHORTED_SNS_ID_PARAM_ID           (0x66000070u)

#define UL_testing_Tuning_P2P_WDGT_ID_VALUE                 (UL_testing_Tuning_dsRam.selfTest.p2pWdgtId)
#define UL_testing_Tuning_P2P_WDGT_ID_OFFSET                (113u)
#define UL_testing_Tuning_P2P_WDGT_ID_SIZE                  (1u)
#define UL_testing_Tuning_P2P_WDGT_ID_PARAM_ID              (0x60000071u)

#define UL_testing_Tuning_P2P_SNS_ID_VALUE                  (UL_testing_Tuning_dsRam.selfTest.p2pSnsId)
#define UL_testing_Tuning_P2P_SNS_ID_OFFSET                 (114u)
#define UL_testing_Tuning_P2P_SNS_ID_SIZE                   (1u)
#define UL_testing_Tuning_P2P_SNS_ID_PARAM_ID               (0x6A000072u)

#define UL_testing_Tuning_BUTTON0_SNS_CP0_VALUE             (UL_testing_Tuning_dsRam.snsCp.button0[0u])
#define UL_testing_Tuning_BUTTON0_SNS_CP0_OFFSET            (116u)
#define UL_testing_Tuning_BUTTON0_SNS_CP0_SIZE              (1u)
#define UL_testing_Tuning_BUTTON0_SNS_CP0_PARAM_ID          (0x67000074u)

#define UL_testing_Tuning_BUTTON1_SNS_CP0_VALUE             (UL_testing_Tuning_dsRam.snsCp.button1[0u])
#define UL_testing_Tuning_BUTTON1_SNS_CP0_OFFSET            (117u)
#define UL_testing_Tuning_BUTTON1_SNS_CP0_SIZE              (1u)
#define UL_testing_Tuning_BUTTON1_SNS_CP0_PARAM_ID          (0x61000075u)

#define UL_testing_Tuning_SNR_TEST_WIDGET_ID_VALUE          (UL_testing_Tuning_dsRam.snrTestWidgetId)
#define UL_testing_Tuning_SNR_TEST_WIDGET_ID_OFFSET         (118u)
#define UL_testing_Tuning_SNR_TEST_WIDGET_ID_SIZE           (1u)
#define UL_testing_Tuning_SNR_TEST_WIDGET_ID_PARAM_ID       (0x6B000076u)

#define UL_testing_Tuning_SNR_TEST_SENSOR_ID_VALUE          (UL_testing_Tuning_dsRam.snrTestSensorId)
#define UL_testing_Tuning_SNR_TEST_SENSOR_ID_OFFSET         (119u)
#define UL_testing_Tuning_SNR_TEST_SENSOR_ID_SIZE           (1u)
#define UL_testing_Tuning_SNR_TEST_SENSOR_ID_PARAM_ID       (0x6D000077u)

#define UL_testing_Tuning_SNR_TEST_SCAN_COUNTER_VALUE       (UL_testing_Tuning_dsRam.snrTestScanCounter)
#define UL_testing_Tuning_SNR_TEST_SCAN_COUNTER_OFFSET      (120u)
#define UL_testing_Tuning_SNR_TEST_SCAN_COUNTER_SIZE        (2u)
#define UL_testing_Tuning_SNR_TEST_SCAN_COUNTER_PARAM_ID    (0x87000078u)

#define UL_testing_Tuning_SNR_TEST_RAW_COUNT0_VALUE         (UL_testing_Tuning_dsRam.snrTestRawCount[0u])
#define UL_testing_Tuning_SNR_TEST_RAW_COUNT0_OFFSET        (122u)
#define UL_testing_Tuning_SNR_TEST_RAW_COUNT0_SIZE          (2u)
#define UL_testing_Tuning_SNR_TEST_RAW_COUNT0_PARAM_ID      (0x8B00007Au)


/*****************************************************************************/
/* Flash Data structure register definitions                                 */
/*****************************************************************************/
#define UL_testing_Tuning_BUTTON0_PTR2SNS_FLASH_VALUE       (UL_testing_Tuning_dsFlash.wdgtArray[0].ptr2SnsFlash)
#define UL_testing_Tuning_BUTTON0_PTR2SNS_FLASH_OFFSET      (0u)
#define UL_testing_Tuning_BUTTON0_PTR2SNS_FLASH_SIZE        (4u)
#define UL_testing_Tuning_BUTTON0_PTR2SNS_FLASH_PARAM_ID    (0xD1000000u)

#define UL_testing_Tuning_BUTTON0_PTR2WD_RAM_VALUE          (UL_testing_Tuning_dsFlash.wdgtArray[0].ptr2WdgtRam)
#define UL_testing_Tuning_BUTTON0_PTR2WD_RAM_OFFSET         (4u)
#define UL_testing_Tuning_BUTTON0_PTR2WD_RAM_SIZE           (4u)
#define UL_testing_Tuning_BUTTON0_PTR2WD_RAM_PARAM_ID       (0xD0000004u)

#define UL_testing_Tuning_BUTTON0_PTR2SNS_RAM_VALUE         (UL_testing_Tuning_dsFlash.wdgtArray[0].ptr2SnsRam)
#define UL_testing_Tuning_BUTTON0_PTR2SNS_RAM_OFFSET        (8u)
#define UL_testing_Tuning_BUTTON0_PTR2SNS_RAM_SIZE          (4u)
#define UL_testing_Tuning_BUTTON0_PTR2SNS_RAM_PARAM_ID      (0xD3000008u)

#define UL_testing_Tuning_BUTTON0_PTR2FLTR_HISTORY_VALUE    (UL_testing_Tuning_dsFlash.wdgtArray[0].ptr2FltrHistory)
#define UL_testing_Tuning_BUTTON0_PTR2FLTR_HISTORY_OFFSET   (12u)
#define UL_testing_Tuning_BUTTON0_PTR2FLTR_HISTORY_SIZE     (4u)
#define UL_testing_Tuning_BUTTON0_PTR2FLTR_HISTORY_PARAM_ID (0xD200000Cu)

#define UL_testing_Tuning_BUTTON0_PTR2DEBOUNCE_VALUE        (UL_testing_Tuning_dsFlash.wdgtArray[0].ptr2DebounceArr)
#define UL_testing_Tuning_BUTTON0_PTR2DEBOUNCE_OFFSET       (16u)
#define UL_testing_Tuning_BUTTON0_PTR2DEBOUNCE_SIZE         (4u)
#define UL_testing_Tuning_BUTTON0_PTR2DEBOUNCE_PARAM_ID     (0xD4000010u)

#define UL_testing_Tuning_BUTTON0_STATIC_CONFIG_VALUE       (UL_testing_Tuning_dsFlash.wdgtArray[0].staticConfig)
#define UL_testing_Tuning_BUTTON0_STATIC_CONFIG_OFFSET      (20u)
#define UL_testing_Tuning_BUTTON0_STATIC_CONFIG_SIZE        (4u)
#define UL_testing_Tuning_BUTTON0_STATIC_CONFIG_PARAM_ID    (0xD5000014u)

#define UL_testing_Tuning_BUTTON0_TOTAL_NUM_SNS_VALUE       (UL_testing_Tuning_dsFlash.wdgtArray[0].totalNumSns)
#define UL_testing_Tuning_BUTTON0_TOTAL_NUM_SNS_OFFSET      (24u)
#define UL_testing_Tuning_BUTTON0_TOTAL_NUM_SNS_SIZE        (2u)
#define UL_testing_Tuning_BUTTON0_TOTAL_NUM_SNS_PARAM_ID    (0x99000018u)

#define UL_testing_Tuning_BUTTON0_TYPE_VALUE                (UL_testing_Tuning_dsFlash.wdgtArray[0].wdgtType)
#define UL_testing_Tuning_BUTTON0_TYPE_OFFSET               (26u)
#define UL_testing_Tuning_BUTTON0_TYPE_SIZE                 (1u)
#define UL_testing_Tuning_BUTTON0_TYPE_PARAM_ID             (0x5D00001Au)

#define UL_testing_Tuning_BUTTON0_NUM_COLS_VALUE            (UL_testing_Tuning_dsFlash.wdgtArray[0].numCols)
#define UL_testing_Tuning_BUTTON0_NUM_COLS_OFFSET           (27u)
#define UL_testing_Tuning_BUTTON0_NUM_COLS_SIZE             (1u)
#define UL_testing_Tuning_BUTTON0_NUM_COLS_PARAM_ID         (0x5B00001Bu)

#define UL_testing_Tuning_BUTTON0_PTR2SNS_CP_VALUE          (UL_testing_Tuning_dsFlash.wdgtArray[0].ptr2SnsCpArr)
#define UL_testing_Tuning_BUTTON0_PTR2SNS_CP_OFFSET         (28u)
#define UL_testing_Tuning_BUTTON0_PTR2SNS_CP_SIZE           (4u)
#define UL_testing_Tuning_BUTTON0_PTR2SNS_CP_PARAM_ID       (0xD700001Cu)

#define UL_testing_Tuning_BUTTON1_PTR2SNS_FLASH_VALUE       (UL_testing_Tuning_dsFlash.wdgtArray[1].ptr2SnsFlash)
#define UL_testing_Tuning_BUTTON1_PTR2SNS_FLASH_OFFSET      (32u)
#define UL_testing_Tuning_BUTTON1_PTR2SNS_FLASH_SIZE        (4u)
#define UL_testing_Tuning_BUTTON1_PTR2SNS_FLASH_PARAM_ID    (0xD8010020u)

#define UL_testing_Tuning_BUTTON1_PTR2WD_RAM_VALUE          (UL_testing_Tuning_dsFlash.wdgtArray[1].ptr2WdgtRam)
#define UL_testing_Tuning_BUTTON1_PTR2WD_RAM_OFFSET         (36u)
#define UL_testing_Tuning_BUTTON1_PTR2WD_RAM_SIZE           (4u)
#define UL_testing_Tuning_BUTTON1_PTR2WD_RAM_PARAM_ID       (0xD9010024u)

#define UL_testing_Tuning_BUTTON1_PTR2SNS_RAM_VALUE         (UL_testing_Tuning_dsFlash.wdgtArray[1].ptr2SnsRam)
#define UL_testing_Tuning_BUTTON1_PTR2SNS_RAM_OFFSET        (40u)
#define UL_testing_Tuning_BUTTON1_PTR2SNS_RAM_SIZE          (4u)
#define UL_testing_Tuning_BUTTON1_PTR2SNS_RAM_PARAM_ID      (0xDA010028u)

#define UL_testing_Tuning_BUTTON1_PTR2FLTR_HISTORY_VALUE    (UL_testing_Tuning_dsFlash.wdgtArray[1].ptr2FltrHistory)
#define UL_testing_Tuning_BUTTON1_PTR2FLTR_HISTORY_OFFSET   (44u)
#define UL_testing_Tuning_BUTTON1_PTR2FLTR_HISTORY_SIZE     (4u)
#define UL_testing_Tuning_BUTTON1_PTR2FLTR_HISTORY_PARAM_ID (0xDB01002Cu)

#define UL_testing_Tuning_BUTTON1_PTR2DEBOUNCE_VALUE        (UL_testing_Tuning_dsFlash.wdgtArray[1].ptr2DebounceArr)
#define UL_testing_Tuning_BUTTON1_PTR2DEBOUNCE_OFFSET       (48u)
#define UL_testing_Tuning_BUTTON1_PTR2DEBOUNCE_SIZE         (4u)
#define UL_testing_Tuning_BUTTON1_PTR2DEBOUNCE_PARAM_ID     (0xDD010030u)

#define UL_testing_Tuning_BUTTON1_STATIC_CONFIG_VALUE       (UL_testing_Tuning_dsFlash.wdgtArray[1].staticConfig)
#define UL_testing_Tuning_BUTTON1_STATIC_CONFIG_OFFSET      (52u)
#define UL_testing_Tuning_BUTTON1_STATIC_CONFIG_SIZE        (4u)
#define UL_testing_Tuning_BUTTON1_STATIC_CONFIG_PARAM_ID    (0xDC010034u)

#define UL_testing_Tuning_BUTTON1_TOTAL_NUM_SNS_VALUE       (UL_testing_Tuning_dsFlash.wdgtArray[1].totalNumSns)
#define UL_testing_Tuning_BUTTON1_TOTAL_NUM_SNS_OFFSET      (56u)
#define UL_testing_Tuning_BUTTON1_TOTAL_NUM_SNS_SIZE        (2u)
#define UL_testing_Tuning_BUTTON1_TOTAL_NUM_SNS_PARAM_ID    (0x90010038u)

#define UL_testing_Tuning_BUTTON1_TYPE_VALUE                (UL_testing_Tuning_dsFlash.wdgtArray[1].wdgtType)
#define UL_testing_Tuning_BUTTON1_TYPE_OFFSET               (58u)
#define UL_testing_Tuning_BUTTON1_TYPE_SIZE                 (1u)
#define UL_testing_Tuning_BUTTON1_TYPE_PARAM_ID             (0x5401003Au)

#define UL_testing_Tuning_BUTTON1_NUM_COLS_VALUE            (UL_testing_Tuning_dsFlash.wdgtArray[1].numCols)
#define UL_testing_Tuning_BUTTON1_NUM_COLS_OFFSET           (59u)
#define UL_testing_Tuning_BUTTON1_NUM_COLS_SIZE             (1u)
#define UL_testing_Tuning_BUTTON1_NUM_COLS_PARAM_ID         (0x5201003Bu)

#define UL_testing_Tuning_BUTTON1_PTR2SNS_CP_VALUE          (UL_testing_Tuning_dsFlash.wdgtArray[1].ptr2SnsCpArr)
#define UL_testing_Tuning_BUTTON1_PTR2SNS_CP_OFFSET         (60u)
#define UL_testing_Tuning_BUTTON1_PTR2SNS_CP_SIZE           (4u)
#define UL_testing_Tuning_BUTTON1_PTR2SNS_CP_PARAM_ID       (0xDE01003Cu)


#endif /* End CY_SENSE_UL_testing_Tuning_REGISTER_MAP_H */


/* [] END OF FILE */
