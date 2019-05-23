/***************************************************************************//**
* \file CapSense_RegisterMap.h
* \version 6.0
*
* \brief
*   This file provides the definitions for the Component data structure register.
*
* \see CapSense v6.0 Datasheet
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

#if !defined(CY_SENSE_CapSense_REGISTER_MAP_H)
#define CY_SENSE_CapSense_REGISTER_MAP_H

#include <cytypes.h>
#include "CapSense_Configuration.h"
#include "CapSense_Structure.h"

/*****************************************************************************/
/* RAM Data structure register definitions                                   */
/*****************************************************************************/
#define CapSense_CONFIG_ID_VALUE                            (CapSense_dsRam.configId)
#define CapSense_CONFIG_ID_OFFSET                           (0u)
#define CapSense_CONFIG_ID_SIZE                             (2u)
#define CapSense_CONFIG_ID_PARAM_ID                         (0x87000000u)

#define CapSense_DEVICE_ID_VALUE                            (CapSense_dsRam.deviceId)
#define CapSense_DEVICE_ID_OFFSET                           (2u)
#define CapSense_DEVICE_ID_SIZE                             (2u)
#define CapSense_DEVICE_ID_PARAM_ID                         (0x8B000002u)

#define CapSense_HW_CLOCK_VALUE                             (CapSense_dsRam.hwClock)
#define CapSense_HW_CLOCK_OFFSET                            (4u)
#define CapSense_HW_CLOCK_SIZE                              (2u)
#define CapSense_HW_CLOCK_PARAM_ID                          (0x86000004u)

#define CapSense_TUNER_CMD_VALUE                            (CapSense_dsRam.tunerCmd)
#define CapSense_TUNER_CMD_OFFSET                           (6u)
#define CapSense_TUNER_CMD_SIZE                             (2u)
#define CapSense_TUNER_CMD_PARAM_ID                         (0xA1000006u)

#define CapSense_SCAN_COUNTER_VALUE                         (CapSense_dsRam.scanCounter)
#define CapSense_SCAN_COUNTER_OFFSET                        (8u)
#define CapSense_SCAN_COUNTER_SIZE                          (2u)
#define CapSense_SCAN_COUNTER_PARAM_ID                      (0x85000008u)

#define CapSense_STATUS_VALUE                               (CapSense_dsRam.status)
#define CapSense_STATUS_OFFSET                              (12u)
#define CapSense_STATUS_SIZE                                (4u)
#define CapSense_STATUS_PARAM_ID                            (0xCB00000Cu)

#define CapSense_WDGT_ENABLE0_VALUE                         (CapSense_dsRam.wdgtEnable[0u])
#define CapSense_WDGT_ENABLE0_OFFSET                        (16u)
#define CapSense_WDGT_ENABLE0_SIZE                          (4u)
#define CapSense_WDGT_ENABLE0_PARAM_ID                      (0xE6000010u)

#define CapSense_WDGT_WORKING0_VALUE                        (CapSense_dsRam.wdgtWorking[0u])
#define CapSense_WDGT_WORKING0_OFFSET                       (20u)
#define CapSense_WDGT_WORKING0_SIZE                         (4u)
#define CapSense_WDGT_WORKING0_PARAM_ID                     (0xCC000014u)

#define CapSense_WDGT_STATUS0_VALUE                         (CapSense_dsRam.wdgtStatus[0u])
#define CapSense_WDGT_STATUS0_OFFSET                        (24u)
#define CapSense_WDGT_STATUS0_SIZE                          (4u)
#define CapSense_WDGT_STATUS0_PARAM_ID                      (0xCF000018u)

#define CapSense_SNS_STATUS0_VALUE                          (CapSense_dsRam.snsStatus[0u])
#define CapSense_SNS_STATUS0_OFFSET                         (28u)
#define CapSense_SNS_STATUS0_SIZE                           (1u)
#define CapSense_SNS_STATUS0_PARAM_ID                       (0x4900001Cu)

#define CapSense_SNS_STATUS1_VALUE                          (CapSense_dsRam.snsStatus[1u])
#define CapSense_SNS_STATUS1_OFFSET                         (29u)
#define CapSense_SNS_STATUS1_SIZE                           (1u)
#define CapSense_SNS_STATUS1_PARAM_ID                       (0x4F00001Du)

#define CapSense_CSD0_CONFIG_VALUE                          (CapSense_dsRam.csd0Config)
#define CapSense_CSD0_CONFIG_OFFSET                         (30u)
#define CapSense_CSD0_CONFIG_SIZE                           (2u)
#define CapSense_CSD0_CONFIG_PARAM_ID                       (0xAB80001Eu)

#define CapSense_MOD_CSD_CLK_VALUE                          (CapSense_dsRam.modCsdClk)
#define CapSense_MOD_CSD_CLK_OFFSET                         (32u)
#define CapSense_MOD_CSD_CLK_SIZE                           (1u)
#define CapSense_MOD_CSD_CLK_PARAM_ID                       (0x63800020u)

#define CapSense_GLB_CRC_VALUE                              (CapSense_dsRam.glbCrc)
#define CapSense_GLB_CRC_OFFSET                             (34u)
#define CapSense_GLB_CRC_SIZE                               (2u)
#define CapSense_GLB_CRC_PARAM_ID                           (0x81000022u)

#define CapSense_START_CRC_VALUE                            (CapSense_dsRam.wdgtList.start.crc)
#define CapSense_START_CRC_OFFSET                           (36u)
#define CapSense_START_CRC_SIZE                             (2u)
#define CapSense_START_CRC_PARAM_ID                         (0x8C000024u)

#define CapSense_START_RESOLUTION_VALUE                     (CapSense_dsRam.wdgtList.start.resolution)
#define CapSense_START_RESOLUTION_OFFSET                    (38u)
#define CapSense_START_RESOLUTION_SIZE                      (2u)
#define CapSense_START_RESOLUTION_PARAM_ID                  (0xA6800026u)

#define CapSense_START_FINGER_TH_VALUE                      (CapSense_dsRam.wdgtList.start.fingerTh)
#define CapSense_START_FINGER_TH_OFFSET                     (40u)
#define CapSense_START_FINGER_TH_SIZE                       (2u)
#define CapSense_START_FINGER_TH_PARAM_ID                   (0xA9800028u)

#define CapSense_START_NOISE_TH_VALUE                       (CapSense_dsRam.wdgtList.start.noiseTh)
#define CapSense_START_NOISE_TH_OFFSET                      (42u)
#define CapSense_START_NOISE_TH_SIZE                        (1u)
#define CapSense_START_NOISE_TH_PARAM_ID                    (0x6D80002Au)

#define CapSense_START_NNOISE_TH_VALUE                      (CapSense_dsRam.wdgtList.start.nNoiseTh)
#define CapSense_START_NNOISE_TH_OFFSET                     (43u)
#define CapSense_START_NNOISE_TH_SIZE                       (1u)
#define CapSense_START_NNOISE_TH_PARAM_ID                   (0x6B80002Bu)

#define CapSense_START_HYSTERESIS_VALUE                     (CapSense_dsRam.wdgtList.start.hysteresis)
#define CapSense_START_HYSTERESIS_OFFSET                    (44u)
#define CapSense_START_HYSTERESIS_SIZE                      (1u)
#define CapSense_START_HYSTERESIS_PARAM_ID                  (0x6080002Cu)

#define CapSense_START_ON_DEBOUNCE_VALUE                    (CapSense_dsRam.wdgtList.start.onDebounce)
#define CapSense_START_ON_DEBOUNCE_OFFSET                   (45u)
#define CapSense_START_ON_DEBOUNCE_SIZE                     (1u)
#define CapSense_START_ON_DEBOUNCE_PARAM_ID                 (0x6680002Du)

#define CapSense_START_LOW_BSLN_RST_VALUE                   (CapSense_dsRam.wdgtList.start.lowBslnRst)
#define CapSense_START_LOW_BSLN_RST_OFFSET                  (46u)
#define CapSense_START_LOW_BSLN_RST_SIZE                    (1u)
#define CapSense_START_LOW_BSLN_RST_PARAM_ID                (0x6C80002Eu)

#define CapSense_START_IDAC_MOD0_VALUE                      (CapSense_dsRam.wdgtList.start.idacMod[0u])
#define CapSense_START_IDAC_MOD0_OFFSET                     (47u)
#define CapSense_START_IDAC_MOD0_SIZE                       (1u)
#define CapSense_START_IDAC_MOD0_PARAM_ID                   (0x4C00002Fu)

#define CapSense_START_SNS_CLK_VALUE                        (CapSense_dsRam.wdgtList.start.snsClk)
#define CapSense_START_SNS_CLK_OFFSET                       (48u)
#define CapSense_START_SNS_CLK_SIZE                         (2u)
#define CapSense_START_SNS_CLK_PARAM_ID                     (0xAE800030u)

#define CapSense_START_SNS_CLK_SOURCE_VALUE                 (CapSense_dsRam.wdgtList.start.snsClkSource)
#define CapSense_START_SNS_CLK_SOURCE_OFFSET                (50u)
#define CapSense_START_SNS_CLK_SOURCE_SIZE                  (1u)
#define CapSense_START_SNS_CLK_SOURCE_PARAM_ID              (0x41800032u)

#define CapSense_POWER_CRC_VALUE                            (CapSense_dsRam.wdgtList.power.crc)
#define CapSense_POWER_CRC_OFFSET                           (52u)
#define CapSense_POWER_CRC_SIZE                             (2u)
#define CapSense_POWER_CRC_PARAM_ID                         (0x8A010034u)

#define CapSense_POWER_RESOLUTION_VALUE                     (CapSense_dsRam.wdgtList.power.resolution)
#define CapSense_POWER_RESOLUTION_OFFSET                    (54u)
#define CapSense_POWER_RESOLUTION_SIZE                      (2u)
#define CapSense_POWER_RESOLUTION_PARAM_ID                  (0xA0810036u)

#define CapSense_POWER_FINGER_TH_VALUE                      (CapSense_dsRam.wdgtList.power.fingerTh)
#define CapSense_POWER_FINGER_TH_OFFSET                     (56u)
#define CapSense_POWER_FINGER_TH_SIZE                       (2u)
#define CapSense_POWER_FINGER_TH_PARAM_ID                   (0xAF810038u)

#define CapSense_POWER_NOISE_TH_VALUE                       (CapSense_dsRam.wdgtList.power.noiseTh)
#define CapSense_POWER_NOISE_TH_OFFSET                      (58u)
#define CapSense_POWER_NOISE_TH_SIZE                        (1u)
#define CapSense_POWER_NOISE_TH_PARAM_ID                    (0x6B81003Au)

#define CapSense_POWER_NNOISE_TH_VALUE                      (CapSense_dsRam.wdgtList.power.nNoiseTh)
#define CapSense_POWER_NNOISE_TH_OFFSET                     (59u)
#define CapSense_POWER_NNOISE_TH_SIZE                       (1u)
#define CapSense_POWER_NNOISE_TH_PARAM_ID                   (0x6D81003Bu)

#define CapSense_POWER_HYSTERESIS_VALUE                     (CapSense_dsRam.wdgtList.power.hysteresis)
#define CapSense_POWER_HYSTERESIS_OFFSET                    (60u)
#define CapSense_POWER_HYSTERESIS_SIZE                      (1u)
#define CapSense_POWER_HYSTERESIS_PARAM_ID                  (0x6681003Cu)

#define CapSense_POWER_ON_DEBOUNCE_VALUE                    (CapSense_dsRam.wdgtList.power.onDebounce)
#define CapSense_POWER_ON_DEBOUNCE_OFFSET                   (61u)
#define CapSense_POWER_ON_DEBOUNCE_SIZE                     (1u)
#define CapSense_POWER_ON_DEBOUNCE_PARAM_ID                 (0x6081003Du)

#define CapSense_POWER_LOW_BSLN_RST_VALUE                   (CapSense_dsRam.wdgtList.power.lowBslnRst)
#define CapSense_POWER_LOW_BSLN_RST_OFFSET                  (62u)
#define CapSense_POWER_LOW_BSLN_RST_SIZE                    (1u)
#define CapSense_POWER_LOW_BSLN_RST_PARAM_ID                (0x6A81003Eu)

#define CapSense_POWER_IDAC_MOD0_VALUE                      (CapSense_dsRam.wdgtList.power.idacMod[0u])
#define CapSense_POWER_IDAC_MOD0_OFFSET                     (63u)
#define CapSense_POWER_IDAC_MOD0_SIZE                       (1u)
#define CapSense_POWER_IDAC_MOD0_PARAM_ID                   (0x4A01003Fu)

#define CapSense_POWER_SNS_CLK_VALUE                        (CapSense_dsRam.wdgtList.power.snsClk)
#define CapSense_POWER_SNS_CLK_OFFSET                       (64u)
#define CapSense_POWER_SNS_CLK_SIZE                         (2u)
#define CapSense_POWER_SNS_CLK_PARAM_ID                     (0xAF810040u)

#define CapSense_POWER_SNS_CLK_SOURCE_VALUE                 (CapSense_dsRam.wdgtList.power.snsClkSource)
#define CapSense_POWER_SNS_CLK_SOURCE_OFFSET                (66u)
#define CapSense_POWER_SNS_CLK_SOURCE_SIZE                  (1u)
#define CapSense_POWER_SNS_CLK_SOURCE_PARAM_ID              (0x40810042u)

#define CapSense_START_SNS0_RAW0_VALUE                      (CapSense_dsRam.snsList.start[0u].raw[0u])
#define CapSense_START_SNS0_RAW0_OFFSET                     (68u)
#define CapSense_START_SNS0_RAW0_SIZE                       (2u)
#define CapSense_START_SNS0_RAW0_PARAM_ID                   (0x8B000044u)

#define CapSense_START_SNS0_BSLN0_VALUE                     (CapSense_dsRam.snsList.start[0u].bsln[0u])
#define CapSense_START_SNS0_BSLN0_OFFSET                    (70u)
#define CapSense_START_SNS0_BSLN0_SIZE                      (2u)
#define CapSense_START_SNS0_BSLN0_PARAM_ID                  (0x87000046u)

#define CapSense_START_SNS0_BSLN_INV0_VALUE                 (CapSense_dsRam.snsList.start[0u].bslnInv[0u])
#define CapSense_START_SNS0_BSLN_INV0_OFFSET                (72u)
#define CapSense_START_SNS0_BSLN_INV0_SIZE                  (2u)
#define CapSense_START_SNS0_BSLN_INV0_PARAM_ID              (0x88000048u)

#define CapSense_START_SNS0_BSLN_EXT0_VALUE                 (CapSense_dsRam.snsList.start[0u].bslnExt[0u])
#define CapSense_START_SNS0_BSLN_EXT0_OFFSET                (74u)
#define CapSense_START_SNS0_BSLN_EXT0_SIZE                  (1u)
#define CapSense_START_SNS0_BSLN_EXT0_PARAM_ID              (0x4C00004Au)

#define CapSense_START_SNS0_DIFF_VALUE                      (CapSense_dsRam.snsList.start[0u].diff)
#define CapSense_START_SNS0_DIFF_OFFSET                     (76u)
#define CapSense_START_SNS0_DIFF_SIZE                       (2u)
#define CapSense_START_SNS0_DIFF_PARAM_ID                   (0x8900004Cu)

#define CapSense_START_SNS0_NEG_BSLN_RST_CNT0_VALUE         (CapSense_dsRam.snsList.start[0u].negBslnRstCnt[0u])
#define CapSense_START_SNS0_NEG_BSLN_RST_CNT0_OFFSET        (78u)
#define CapSense_START_SNS0_NEG_BSLN_RST_CNT0_SIZE          (1u)
#define CapSense_START_SNS0_NEG_BSLN_RST_CNT0_PARAM_ID      (0x4D00004Eu)

#define CapSense_START_SNS0_IDAC_COMP0_VALUE                (CapSense_dsRam.snsList.start[0u].idacComp[0u])
#define CapSense_START_SNS0_IDAC_COMP0_OFFSET               (79u)
#define CapSense_START_SNS0_IDAC_COMP0_SIZE                 (1u)
#define CapSense_START_SNS0_IDAC_COMP0_PARAM_ID             (0x4B00004Fu)

#define CapSense_POWER_SNS0_RAW0_VALUE                      (CapSense_dsRam.snsList.power[0u].raw[0u])
#define CapSense_POWER_SNS0_RAW0_OFFSET                     (80u)
#define CapSense_POWER_SNS0_RAW0_SIZE                       (2u)
#define CapSense_POWER_SNS0_RAW0_PARAM_ID                   (0x8F000050u)

#define CapSense_POWER_SNS0_BSLN0_VALUE                     (CapSense_dsRam.snsList.power[0u].bsln[0u])
#define CapSense_POWER_SNS0_BSLN0_OFFSET                    (82u)
#define CapSense_POWER_SNS0_BSLN0_SIZE                      (2u)
#define CapSense_POWER_SNS0_BSLN0_PARAM_ID                  (0x83000052u)

#define CapSense_POWER_SNS0_BSLN_INV0_VALUE                 (CapSense_dsRam.snsList.power[0u].bslnInv[0u])
#define CapSense_POWER_SNS0_BSLN_INV0_OFFSET                (84u)
#define CapSense_POWER_SNS0_BSLN_INV0_SIZE                  (2u)
#define CapSense_POWER_SNS0_BSLN_INV0_PARAM_ID              (0x8E000054u)

#define CapSense_POWER_SNS0_BSLN_EXT0_VALUE                 (CapSense_dsRam.snsList.power[0u].bslnExt[0u])
#define CapSense_POWER_SNS0_BSLN_EXT0_OFFSET                (86u)
#define CapSense_POWER_SNS0_BSLN_EXT0_SIZE                  (1u)
#define CapSense_POWER_SNS0_BSLN_EXT0_PARAM_ID              (0x4A000056u)

#define CapSense_POWER_SNS0_DIFF_VALUE                      (CapSense_dsRam.snsList.power[0u].diff)
#define CapSense_POWER_SNS0_DIFF_OFFSET                     (88u)
#define CapSense_POWER_SNS0_DIFF_SIZE                       (2u)
#define CapSense_POWER_SNS0_DIFF_PARAM_ID                   (0x8D000058u)

#define CapSense_POWER_SNS0_NEG_BSLN_RST_CNT0_VALUE         (CapSense_dsRam.snsList.power[0u].negBslnRstCnt[0u])
#define CapSense_POWER_SNS0_NEG_BSLN_RST_CNT0_OFFSET        (90u)
#define CapSense_POWER_SNS0_NEG_BSLN_RST_CNT0_SIZE          (1u)
#define CapSense_POWER_SNS0_NEG_BSLN_RST_CNT0_PARAM_ID      (0x4900005Au)

#define CapSense_POWER_SNS0_IDAC_COMP0_VALUE                (CapSense_dsRam.snsList.power[0u].idacComp[0u])
#define CapSense_POWER_SNS0_IDAC_COMP0_OFFSET               (91u)
#define CapSense_POWER_SNS0_IDAC_COMP0_SIZE                 (1u)
#define CapSense_POWER_SNS0_IDAC_COMP0_PARAM_ID             (0x4F00005Bu)

#define CapSense_TEST_RESULT_MASK_VALUE                     (CapSense_dsRam.selfTest.testResultMask)
#define CapSense_TEST_RESULT_MASK_OFFSET                    (92u)
#define CapSense_TEST_RESULT_MASK_SIZE                      (4u)
#define CapSense_TEST_RESULT_MASK_PARAM_ID                  (0xE800005Cu)

#define CapSense_EXT_CAP0_VALUE                             (CapSense_dsRam.selfTest.extCap[0u])
#define CapSense_EXT_CAP0_OFFSET                            (96u)
#define CapSense_EXT_CAP0_SIZE                              (2u)
#define CapSense_EXT_CAP0_PARAM_ID                          (0xAB000060u)

#define CapSense_EXT_CAP1_VALUE                             (CapSense_dsRam.selfTest.extCap[1u])
#define CapSense_EXT_CAP1_OFFSET                            (98u)
#define CapSense_EXT_CAP1_SIZE                              (2u)
#define CapSense_EXT_CAP1_PARAM_ID                          (0xA7000062u)

#define CapSense_VDDA_VOLTAGE_VALUE                         (CapSense_dsRam.selfTest.vddaVoltage)
#define CapSense_VDDA_VOLTAGE_OFFSET                        (100u)
#define CapSense_VDDA_VOLTAGE_SIZE                          (2u)
#define CapSense_VDDA_VOLTAGE_PARAM_ID                      (0xAA000064u)

#define CapSense_SHIELD_CAP_VALUE                           (CapSense_dsRam.selfTest.shieldCap)
#define CapSense_SHIELD_CAP_OFFSET                          (102u)
#define CapSense_SHIELD_CAP_SIZE                            (2u)
#define CapSense_SHIELD_CAP_PARAM_ID                        (0xA6000066u)

#define CapSense_GLB_CRC_CALC_VALUE                         (CapSense_dsRam.selfTest.glbCrcCalc)
#define CapSense_GLB_CRC_CALC_OFFSET                        (104u)
#define CapSense_GLB_CRC_CALC_SIZE                          (2u)
#define CapSense_GLB_CRC_CALC_PARAM_ID                      (0xA9000068u)

#define CapSense_WDGT_CRC_CALC_VALUE                        (CapSense_dsRam.selfTest.wdgtCrcCalc)
#define CapSense_WDGT_CRC_CALC_OFFSET                       (106u)
#define CapSense_WDGT_CRC_CALC_SIZE                         (2u)
#define CapSense_WDGT_CRC_CALC_PARAM_ID                     (0xA500006Au)

#define CapSense_WDGT_CRC_ID_VALUE                          (CapSense_dsRam.selfTest.wdgtCrcId)
#define CapSense_WDGT_CRC_ID_OFFSET                         (108u)
#define CapSense_WDGT_CRC_ID_SIZE                           (1u)
#define CapSense_WDGT_CRC_ID_PARAM_ID                       (0x6000006Cu)

#define CapSense_INV_BSLN_WDGT_ID_VALUE                     (CapSense_dsRam.selfTest.invBslnWdgtId)
#define CapSense_INV_BSLN_WDGT_ID_OFFSET                    (109u)
#define CapSense_INV_BSLN_WDGT_ID_SIZE                      (1u)
#define CapSense_INV_BSLN_WDGT_ID_PARAM_ID                  (0x6600006Du)

#define CapSense_INV_BSLN_SNS_ID_VALUE                      (CapSense_dsRam.selfTest.invBslnSnsId)
#define CapSense_INV_BSLN_SNS_ID_OFFSET                     (110u)
#define CapSense_INV_BSLN_SNS_ID_SIZE                       (1u)
#define CapSense_INV_BSLN_SNS_ID_PARAM_ID                   (0x6C00006Eu)

#define CapSense_SHORTED_WDGT_ID_VALUE                      (CapSense_dsRam.selfTest.shortedWdgtId)
#define CapSense_SHORTED_WDGT_ID_OFFSET                     (111u)
#define CapSense_SHORTED_WDGT_ID_SIZE                       (1u)
#define CapSense_SHORTED_WDGT_ID_PARAM_ID                   (0x6A00006Fu)

#define CapSense_SHORTED_SNS_ID_VALUE                       (CapSense_dsRam.selfTest.shortedSnsId)
#define CapSense_SHORTED_SNS_ID_OFFSET                      (112u)
#define CapSense_SHORTED_SNS_ID_SIZE                        (1u)
#define CapSense_SHORTED_SNS_ID_PARAM_ID                    (0x66000070u)

#define CapSense_P2P_WDGT_ID_VALUE                          (CapSense_dsRam.selfTest.p2pWdgtId)
#define CapSense_P2P_WDGT_ID_OFFSET                         (113u)
#define CapSense_P2P_WDGT_ID_SIZE                           (1u)
#define CapSense_P2P_WDGT_ID_PARAM_ID                       (0x60000071u)

#define CapSense_P2P_SNS_ID_VALUE                           (CapSense_dsRam.selfTest.p2pSnsId)
#define CapSense_P2P_SNS_ID_OFFSET                          (114u)
#define CapSense_P2P_SNS_ID_SIZE                            (1u)
#define CapSense_P2P_SNS_ID_PARAM_ID                        (0x6A000072u)

#define CapSense_START_SNS_CP0_VALUE                        (CapSense_dsRam.snsCp.start[0u])
#define CapSense_START_SNS_CP0_OFFSET                       (116u)
#define CapSense_START_SNS_CP0_SIZE                         (1u)
#define CapSense_START_SNS_CP0_PARAM_ID                     (0x67000074u)

#define CapSense_POWER_SNS_CP0_VALUE                        (CapSense_dsRam.snsCp.power[0u])
#define CapSense_POWER_SNS_CP0_OFFSET                       (117u)
#define CapSense_POWER_SNS_CP0_SIZE                         (1u)
#define CapSense_POWER_SNS_CP0_PARAM_ID                     (0x61000075u)

#define CapSense_SNR_TEST_WIDGET_ID_VALUE                   (CapSense_dsRam.snrTestWidgetId)
#define CapSense_SNR_TEST_WIDGET_ID_OFFSET                  (118u)
#define CapSense_SNR_TEST_WIDGET_ID_SIZE                    (1u)
#define CapSense_SNR_TEST_WIDGET_ID_PARAM_ID                (0x6B000076u)

#define CapSense_SNR_TEST_SENSOR_ID_VALUE                   (CapSense_dsRam.snrTestSensorId)
#define CapSense_SNR_TEST_SENSOR_ID_OFFSET                  (119u)
#define CapSense_SNR_TEST_SENSOR_ID_SIZE                    (1u)
#define CapSense_SNR_TEST_SENSOR_ID_PARAM_ID                (0x6D000077u)

#define CapSense_SNR_TEST_SCAN_COUNTER_VALUE                (CapSense_dsRam.snrTestScanCounter)
#define CapSense_SNR_TEST_SCAN_COUNTER_OFFSET               (120u)
#define CapSense_SNR_TEST_SCAN_COUNTER_SIZE                 (2u)
#define CapSense_SNR_TEST_SCAN_COUNTER_PARAM_ID             (0x87000078u)

#define CapSense_SNR_TEST_RAW_COUNT0_VALUE                  (CapSense_dsRam.snrTestRawCount[0u])
#define CapSense_SNR_TEST_RAW_COUNT0_OFFSET                 (122u)
#define CapSense_SNR_TEST_RAW_COUNT0_SIZE                   (2u)
#define CapSense_SNR_TEST_RAW_COUNT0_PARAM_ID               (0x8B00007Au)


/*****************************************************************************/
/* Flash Data structure register definitions                                 */
/*****************************************************************************/
#define CapSense_START_PTR2SNS_FLASH_VALUE                  (CapSense_dsFlash.wdgtArray[0].ptr2SnsFlash)
#define CapSense_START_PTR2SNS_FLASH_OFFSET                 (0u)
#define CapSense_START_PTR2SNS_FLASH_SIZE                   (4u)
#define CapSense_START_PTR2SNS_FLASH_PARAM_ID               (0xD1000000u)

#define CapSense_START_PTR2WD_RAM_VALUE                     (CapSense_dsFlash.wdgtArray[0].ptr2WdgtRam)
#define CapSense_START_PTR2WD_RAM_OFFSET                    (4u)
#define CapSense_START_PTR2WD_RAM_SIZE                      (4u)
#define CapSense_START_PTR2WD_RAM_PARAM_ID                  (0xD0000004u)

#define CapSense_START_PTR2SNS_RAM_VALUE                    (CapSense_dsFlash.wdgtArray[0].ptr2SnsRam)
#define CapSense_START_PTR2SNS_RAM_OFFSET                   (8u)
#define CapSense_START_PTR2SNS_RAM_SIZE                     (4u)
#define CapSense_START_PTR2SNS_RAM_PARAM_ID                 (0xD3000008u)

#define CapSense_START_PTR2FLTR_HISTORY_VALUE               (CapSense_dsFlash.wdgtArray[0].ptr2FltrHistory)
#define CapSense_START_PTR2FLTR_HISTORY_OFFSET              (12u)
#define CapSense_START_PTR2FLTR_HISTORY_SIZE                (4u)
#define CapSense_START_PTR2FLTR_HISTORY_PARAM_ID            (0xD200000Cu)

#define CapSense_START_PTR2DEBOUNCE_VALUE                   (CapSense_dsFlash.wdgtArray[0].ptr2DebounceArr)
#define CapSense_START_PTR2DEBOUNCE_OFFSET                  (16u)
#define CapSense_START_PTR2DEBOUNCE_SIZE                    (4u)
#define CapSense_START_PTR2DEBOUNCE_PARAM_ID                (0xD4000010u)

#define CapSense_START_STATIC_CONFIG_VALUE                  (CapSense_dsFlash.wdgtArray[0].staticConfig)
#define CapSense_START_STATIC_CONFIG_OFFSET                 (20u)
#define CapSense_START_STATIC_CONFIG_SIZE                   (4u)
#define CapSense_START_STATIC_CONFIG_PARAM_ID               (0xD5000014u)

#define CapSense_START_TOTAL_NUM_SNS_VALUE                  (CapSense_dsFlash.wdgtArray[0].totalNumSns)
#define CapSense_START_TOTAL_NUM_SNS_OFFSET                 (24u)
#define CapSense_START_TOTAL_NUM_SNS_SIZE                   (2u)
#define CapSense_START_TOTAL_NUM_SNS_PARAM_ID               (0x99000018u)

#define CapSense_START_TYPE_VALUE                           (CapSense_dsFlash.wdgtArray[0].wdgtType)
#define CapSense_START_TYPE_OFFSET                          (26u)
#define CapSense_START_TYPE_SIZE                            (1u)
#define CapSense_START_TYPE_PARAM_ID                        (0x5D00001Au)

#define CapSense_START_NUM_COLS_VALUE                       (CapSense_dsFlash.wdgtArray[0].numCols)
#define CapSense_START_NUM_COLS_OFFSET                      (27u)
#define CapSense_START_NUM_COLS_SIZE                        (1u)
#define CapSense_START_NUM_COLS_PARAM_ID                    (0x5B00001Bu)

#define CapSense_START_PTR2SNS_CP_VALUE                     (CapSense_dsFlash.wdgtArray[0].ptr2SnsCpArr)
#define CapSense_START_PTR2SNS_CP_OFFSET                    (28u)
#define CapSense_START_PTR2SNS_CP_SIZE                      (4u)
#define CapSense_START_PTR2SNS_CP_PARAM_ID                  (0xD700001Cu)

#define CapSense_POWER_PTR2SNS_FLASH_VALUE                  (CapSense_dsFlash.wdgtArray[1].ptr2SnsFlash)
#define CapSense_POWER_PTR2SNS_FLASH_OFFSET                 (32u)
#define CapSense_POWER_PTR2SNS_FLASH_SIZE                   (4u)
#define CapSense_POWER_PTR2SNS_FLASH_PARAM_ID               (0xD8010020u)

#define CapSense_POWER_PTR2WD_RAM_VALUE                     (CapSense_dsFlash.wdgtArray[1].ptr2WdgtRam)
#define CapSense_POWER_PTR2WD_RAM_OFFSET                    (36u)
#define CapSense_POWER_PTR2WD_RAM_SIZE                      (4u)
#define CapSense_POWER_PTR2WD_RAM_PARAM_ID                  (0xD9010024u)

#define CapSense_POWER_PTR2SNS_RAM_VALUE                    (CapSense_dsFlash.wdgtArray[1].ptr2SnsRam)
#define CapSense_POWER_PTR2SNS_RAM_OFFSET                   (40u)
#define CapSense_POWER_PTR2SNS_RAM_SIZE                     (4u)
#define CapSense_POWER_PTR2SNS_RAM_PARAM_ID                 (0xDA010028u)

#define CapSense_POWER_PTR2FLTR_HISTORY_VALUE               (CapSense_dsFlash.wdgtArray[1].ptr2FltrHistory)
#define CapSense_POWER_PTR2FLTR_HISTORY_OFFSET              (44u)
#define CapSense_POWER_PTR2FLTR_HISTORY_SIZE                (4u)
#define CapSense_POWER_PTR2FLTR_HISTORY_PARAM_ID            (0xDB01002Cu)

#define CapSense_POWER_PTR2DEBOUNCE_VALUE                   (CapSense_dsFlash.wdgtArray[1].ptr2DebounceArr)
#define CapSense_POWER_PTR2DEBOUNCE_OFFSET                  (48u)
#define CapSense_POWER_PTR2DEBOUNCE_SIZE                    (4u)
#define CapSense_POWER_PTR2DEBOUNCE_PARAM_ID                (0xDD010030u)

#define CapSense_POWER_STATIC_CONFIG_VALUE                  (CapSense_dsFlash.wdgtArray[1].staticConfig)
#define CapSense_POWER_STATIC_CONFIG_OFFSET                 (52u)
#define CapSense_POWER_STATIC_CONFIG_SIZE                   (4u)
#define CapSense_POWER_STATIC_CONFIG_PARAM_ID               (0xDC010034u)

#define CapSense_POWER_TOTAL_NUM_SNS_VALUE                  (CapSense_dsFlash.wdgtArray[1].totalNumSns)
#define CapSense_POWER_TOTAL_NUM_SNS_OFFSET                 (56u)
#define CapSense_POWER_TOTAL_NUM_SNS_SIZE                   (2u)
#define CapSense_POWER_TOTAL_NUM_SNS_PARAM_ID               (0x90010038u)

#define CapSense_POWER_TYPE_VALUE                           (CapSense_dsFlash.wdgtArray[1].wdgtType)
#define CapSense_POWER_TYPE_OFFSET                          (58u)
#define CapSense_POWER_TYPE_SIZE                            (1u)
#define CapSense_POWER_TYPE_PARAM_ID                        (0x5401003Au)

#define CapSense_POWER_NUM_COLS_VALUE                       (CapSense_dsFlash.wdgtArray[1].numCols)
#define CapSense_POWER_NUM_COLS_OFFSET                      (59u)
#define CapSense_POWER_NUM_COLS_SIZE                        (1u)
#define CapSense_POWER_NUM_COLS_PARAM_ID                    (0x5201003Bu)

#define CapSense_POWER_PTR2SNS_CP_VALUE                     (CapSense_dsFlash.wdgtArray[1].ptr2SnsCpArr)
#define CapSense_POWER_PTR2SNS_CP_OFFSET                    (60u)
#define CapSense_POWER_PTR2SNS_CP_SIZE                      (4u)
#define CapSense_POWER_PTR2SNS_CP_PARAM_ID                  (0xDE01003Cu)


#endif /* End CY_SENSE_CapSense_REGISTER_MAP_H */


/* [] END OF FILE */
