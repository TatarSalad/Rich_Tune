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

#define CapSense_SNS_STATUS2_VALUE                          (CapSense_dsRam.snsStatus[2u])
#define CapSense_SNS_STATUS2_OFFSET                         (30u)
#define CapSense_SNS_STATUS2_SIZE                           (1u)
#define CapSense_SNS_STATUS2_PARAM_ID                       (0x4500001Eu)

#define CapSense_SNS_STATUS3_VALUE                          (CapSense_dsRam.snsStatus[3u])
#define CapSense_SNS_STATUS3_OFFSET                         (31u)
#define CapSense_SNS_STATUS3_SIZE                           (1u)
#define CapSense_SNS_STATUS3_PARAM_ID                       (0x4300001Fu)

#define CapSense_SNS_STATUS4_VALUE                          (CapSense_dsRam.snsStatus[4u])
#define CapSense_SNS_STATUS4_OFFSET                         (32u)
#define CapSense_SNS_STATUS4_SIZE                           (1u)
#define CapSense_SNS_STATUS4_PARAM_ID                       (0x45000020u)

#define CapSense_SNS_STATUS5_VALUE                          (CapSense_dsRam.snsStatus[5u])
#define CapSense_SNS_STATUS5_OFFSET                         (33u)
#define CapSense_SNS_STATUS5_SIZE                           (1u)
#define CapSense_SNS_STATUS5_PARAM_ID                       (0x43000021u)

#define CapSense_SNS_STATUS6_VALUE                          (CapSense_dsRam.snsStatus[6u])
#define CapSense_SNS_STATUS6_OFFSET                         (34u)
#define CapSense_SNS_STATUS6_SIZE                           (1u)
#define CapSense_SNS_STATUS6_PARAM_ID                       (0x49000022u)

#define CapSense_SNS_STATUS7_VALUE                          (CapSense_dsRam.snsStatus[7u])
#define CapSense_SNS_STATUS7_OFFSET                         (35u)
#define CapSense_SNS_STATUS7_SIZE                           (1u)
#define CapSense_SNS_STATUS7_PARAM_ID                       (0x4F000023u)

#define CapSense_SNS_STATUS8_VALUE                          (CapSense_dsRam.snsStatus[8u])
#define CapSense_SNS_STATUS8_OFFSET                         (36u)
#define CapSense_SNS_STATUS8_SIZE                           (1u)
#define CapSense_SNS_STATUS8_PARAM_ID                       (0x44000024u)

#define CapSense_SNS_STATUS9_VALUE                          (CapSense_dsRam.snsStatus[9u])
#define CapSense_SNS_STATUS9_OFFSET                         (37u)
#define CapSense_SNS_STATUS9_SIZE                           (1u)
#define CapSense_SNS_STATUS9_PARAM_ID                       (0x42000025u)

#define CapSense_SNS_STATUS10_VALUE                         (CapSense_dsRam.snsStatus[10u])
#define CapSense_SNS_STATUS10_OFFSET                        (38u)
#define CapSense_SNS_STATUS10_SIZE                          (1u)
#define CapSense_SNS_STATUS10_PARAM_ID                      (0x48000026u)

#define CapSense_SNS_STATUS11_VALUE                         (CapSense_dsRam.snsStatus[11u])
#define CapSense_SNS_STATUS11_OFFSET                        (39u)
#define CapSense_SNS_STATUS11_SIZE                          (1u)
#define CapSense_SNS_STATUS11_PARAM_ID                      (0x4E000027u)

#define CapSense_SNS_STATUS12_VALUE                         (CapSense_dsRam.snsStatus[12u])
#define CapSense_SNS_STATUS12_OFFSET                        (40u)
#define CapSense_SNS_STATUS12_SIZE                          (1u)
#define CapSense_SNS_STATUS12_PARAM_ID                      (0x47000028u)

#define CapSense_CSD0_CONFIG_VALUE                          (CapSense_dsRam.csd0Config)
#define CapSense_CSD0_CONFIG_OFFSET                         (42u)
#define CapSense_CSD0_CONFIG_SIZE                           (2u)
#define CapSense_CSD0_CONFIG_PARAM_ID                       (0xA580002Au)

#define CapSense_MOD_CSD_CLK_VALUE                          (CapSense_dsRam.modCsdClk)
#define CapSense_MOD_CSD_CLK_OFFSET                         (44u)
#define CapSense_MOD_CSD_CLK_SIZE                           (1u)
#define CapSense_MOD_CSD_CLK_PARAM_ID                       (0x6080002Cu)

#define CapSense_GLB_CRC_VALUE                              (CapSense_dsRam.glbCrc)
#define CapSense_GLB_CRC_OFFSET                             (46u)
#define CapSense_GLB_CRC_SIZE                               (2u)
#define CapSense_GLB_CRC_PARAM_ID                           (0x8200002Eu)

#define CapSense_KEY00_CRC_VALUE                            (CapSense_dsRam.wdgtList.key00.crc)
#define CapSense_KEY00_CRC_OFFSET                           (48u)
#define CapSense_KEY00_CRC_SIZE                             (2u)
#define CapSense_KEY00_CRC_PARAM_ID                         (0x88000030u)

#define CapSense_KEY00_RESOLUTION_VALUE                     (CapSense_dsRam.wdgtList.key00.resolution)
#define CapSense_KEY00_RESOLUTION_OFFSET                    (50u)
#define CapSense_KEY00_RESOLUTION_SIZE                      (2u)
#define CapSense_KEY00_RESOLUTION_PARAM_ID                  (0xA2800032u)

#define CapSense_KEY00_FINGER_TH_VALUE                      (CapSense_dsRam.wdgtList.key00.fingerTh)
#define CapSense_KEY00_FINGER_TH_OFFSET                     (52u)
#define CapSense_KEY00_FINGER_TH_SIZE                       (2u)
#define CapSense_KEY00_FINGER_TH_PARAM_ID                   (0xAF800034u)

#define CapSense_KEY00_NOISE_TH_VALUE                       (CapSense_dsRam.wdgtList.key00.noiseTh)
#define CapSense_KEY00_NOISE_TH_OFFSET                      (54u)
#define CapSense_KEY00_NOISE_TH_SIZE                        (1u)
#define CapSense_KEY00_NOISE_TH_PARAM_ID                    (0x6B800036u)

#define CapSense_KEY00_NNOISE_TH_VALUE                      (CapSense_dsRam.wdgtList.key00.nNoiseTh)
#define CapSense_KEY00_NNOISE_TH_OFFSET                     (55u)
#define CapSense_KEY00_NNOISE_TH_SIZE                       (1u)
#define CapSense_KEY00_NNOISE_TH_PARAM_ID                   (0x6D800037u)

#define CapSense_KEY00_HYSTERESIS_VALUE                     (CapSense_dsRam.wdgtList.key00.hysteresis)
#define CapSense_KEY00_HYSTERESIS_OFFSET                    (56u)
#define CapSense_KEY00_HYSTERESIS_SIZE                      (1u)
#define CapSense_KEY00_HYSTERESIS_PARAM_ID                  (0x64800038u)

#define CapSense_KEY00_ON_DEBOUNCE_VALUE                    (CapSense_dsRam.wdgtList.key00.onDebounce)
#define CapSense_KEY00_ON_DEBOUNCE_OFFSET                   (57u)
#define CapSense_KEY00_ON_DEBOUNCE_SIZE                     (1u)
#define CapSense_KEY00_ON_DEBOUNCE_PARAM_ID                 (0x62800039u)

#define CapSense_KEY00_LOW_BSLN_RST_VALUE                   (CapSense_dsRam.wdgtList.key00.lowBslnRst)
#define CapSense_KEY00_LOW_BSLN_RST_OFFSET                  (58u)
#define CapSense_KEY00_LOW_BSLN_RST_SIZE                    (1u)
#define CapSense_KEY00_LOW_BSLN_RST_PARAM_ID                (0x6880003Au)

#define CapSense_KEY00_IDAC_MOD0_VALUE                      (CapSense_dsRam.wdgtList.key00.idacMod[0u])
#define CapSense_KEY00_IDAC_MOD0_OFFSET                     (59u)
#define CapSense_KEY00_IDAC_MOD0_SIZE                       (1u)
#define CapSense_KEY00_IDAC_MOD0_PARAM_ID                   (0x4800003Bu)

#define CapSense_KEY00_SNS_CLK_VALUE                        (CapSense_dsRam.wdgtList.key00.snsClk)
#define CapSense_KEY00_SNS_CLK_OFFSET                       (60u)
#define CapSense_KEY00_SNS_CLK_SIZE                         (2u)
#define CapSense_KEY00_SNS_CLK_PARAM_ID                     (0xAD80003Cu)

#define CapSense_KEY00_SNS_CLK_SOURCE_VALUE                 (CapSense_dsRam.wdgtList.key00.snsClkSource)
#define CapSense_KEY00_SNS_CLK_SOURCE_OFFSET                (62u)
#define CapSense_KEY00_SNS_CLK_SOURCE_SIZE                  (1u)
#define CapSense_KEY00_SNS_CLK_SOURCE_PARAM_ID              (0x4280003Eu)

#define CapSense_KEY01_CRC_VALUE                            (CapSense_dsRam.wdgtList.key01.crc)
#define CapSense_KEY01_CRC_OFFSET                           (64u)
#define CapSense_KEY01_CRC_SIZE                             (2u)
#define CapSense_KEY01_CRC_PARAM_ID                         (0x89010040u)

#define CapSense_KEY01_RESOLUTION_VALUE                     (CapSense_dsRam.wdgtList.key01.resolution)
#define CapSense_KEY01_RESOLUTION_OFFSET                    (66u)
#define CapSense_KEY01_RESOLUTION_SIZE                      (2u)
#define CapSense_KEY01_RESOLUTION_PARAM_ID                  (0xA3810042u)

#define CapSense_KEY01_FINGER_TH_VALUE                      (CapSense_dsRam.wdgtList.key01.fingerTh)
#define CapSense_KEY01_FINGER_TH_OFFSET                     (68u)
#define CapSense_KEY01_FINGER_TH_SIZE                       (2u)
#define CapSense_KEY01_FINGER_TH_PARAM_ID                   (0xAE810044u)

#define CapSense_KEY01_NOISE_TH_VALUE                       (CapSense_dsRam.wdgtList.key01.noiseTh)
#define CapSense_KEY01_NOISE_TH_OFFSET                      (70u)
#define CapSense_KEY01_NOISE_TH_SIZE                        (1u)
#define CapSense_KEY01_NOISE_TH_PARAM_ID                    (0x6A810046u)

#define CapSense_KEY01_NNOISE_TH_VALUE                      (CapSense_dsRam.wdgtList.key01.nNoiseTh)
#define CapSense_KEY01_NNOISE_TH_OFFSET                     (71u)
#define CapSense_KEY01_NNOISE_TH_SIZE                       (1u)
#define CapSense_KEY01_NNOISE_TH_PARAM_ID                   (0x6C810047u)

#define CapSense_KEY01_HYSTERESIS_VALUE                     (CapSense_dsRam.wdgtList.key01.hysteresis)
#define CapSense_KEY01_HYSTERESIS_OFFSET                    (72u)
#define CapSense_KEY01_HYSTERESIS_SIZE                      (1u)
#define CapSense_KEY01_HYSTERESIS_PARAM_ID                  (0x65810048u)

#define CapSense_KEY01_ON_DEBOUNCE_VALUE                    (CapSense_dsRam.wdgtList.key01.onDebounce)
#define CapSense_KEY01_ON_DEBOUNCE_OFFSET                   (73u)
#define CapSense_KEY01_ON_DEBOUNCE_SIZE                     (1u)
#define CapSense_KEY01_ON_DEBOUNCE_PARAM_ID                 (0x63810049u)

#define CapSense_KEY01_LOW_BSLN_RST_VALUE                   (CapSense_dsRam.wdgtList.key01.lowBslnRst)
#define CapSense_KEY01_LOW_BSLN_RST_OFFSET                  (74u)
#define CapSense_KEY01_LOW_BSLN_RST_SIZE                    (1u)
#define CapSense_KEY01_LOW_BSLN_RST_PARAM_ID                (0x6981004Au)

#define CapSense_KEY01_IDAC_MOD0_VALUE                      (CapSense_dsRam.wdgtList.key01.idacMod[0u])
#define CapSense_KEY01_IDAC_MOD0_OFFSET                     (75u)
#define CapSense_KEY01_IDAC_MOD0_SIZE                       (1u)
#define CapSense_KEY01_IDAC_MOD0_PARAM_ID                   (0x4901004Bu)

#define CapSense_KEY01_SNS_CLK_VALUE                        (CapSense_dsRam.wdgtList.key01.snsClk)
#define CapSense_KEY01_SNS_CLK_OFFSET                       (76u)
#define CapSense_KEY01_SNS_CLK_SIZE                         (2u)
#define CapSense_KEY01_SNS_CLK_PARAM_ID                     (0xAC81004Cu)

#define CapSense_KEY01_SNS_CLK_SOURCE_VALUE                 (CapSense_dsRam.wdgtList.key01.snsClkSource)
#define CapSense_KEY01_SNS_CLK_SOURCE_OFFSET                (78u)
#define CapSense_KEY01_SNS_CLK_SOURCE_SIZE                  (1u)
#define CapSense_KEY01_SNS_CLK_SOURCE_PARAM_ID              (0x4381004Eu)

#define CapSense_KEY02_CRC_VALUE                            (CapSense_dsRam.wdgtList.key02.crc)
#define CapSense_KEY02_CRC_OFFSET                           (80u)
#define CapSense_KEY02_CRC_SIZE                             (2u)
#define CapSense_KEY02_CRC_PARAM_ID                         (0x89020050u)

#define CapSense_KEY02_RESOLUTION_VALUE                     (CapSense_dsRam.wdgtList.key02.resolution)
#define CapSense_KEY02_RESOLUTION_OFFSET                    (82u)
#define CapSense_KEY02_RESOLUTION_SIZE                      (2u)
#define CapSense_KEY02_RESOLUTION_PARAM_ID                  (0xA3820052u)

#define CapSense_KEY02_FINGER_TH_VALUE                      (CapSense_dsRam.wdgtList.key02.fingerTh)
#define CapSense_KEY02_FINGER_TH_OFFSET                     (84u)
#define CapSense_KEY02_FINGER_TH_SIZE                       (2u)
#define CapSense_KEY02_FINGER_TH_PARAM_ID                   (0xAE820054u)

#define CapSense_KEY02_NOISE_TH_VALUE                       (CapSense_dsRam.wdgtList.key02.noiseTh)
#define CapSense_KEY02_NOISE_TH_OFFSET                      (86u)
#define CapSense_KEY02_NOISE_TH_SIZE                        (1u)
#define CapSense_KEY02_NOISE_TH_PARAM_ID                    (0x6A820056u)

#define CapSense_KEY02_NNOISE_TH_VALUE                      (CapSense_dsRam.wdgtList.key02.nNoiseTh)
#define CapSense_KEY02_NNOISE_TH_OFFSET                     (87u)
#define CapSense_KEY02_NNOISE_TH_SIZE                       (1u)
#define CapSense_KEY02_NNOISE_TH_PARAM_ID                   (0x6C820057u)

#define CapSense_KEY02_HYSTERESIS_VALUE                     (CapSense_dsRam.wdgtList.key02.hysteresis)
#define CapSense_KEY02_HYSTERESIS_OFFSET                    (88u)
#define CapSense_KEY02_HYSTERESIS_SIZE                      (1u)
#define CapSense_KEY02_HYSTERESIS_PARAM_ID                  (0x65820058u)

#define CapSense_KEY02_ON_DEBOUNCE_VALUE                    (CapSense_dsRam.wdgtList.key02.onDebounce)
#define CapSense_KEY02_ON_DEBOUNCE_OFFSET                   (89u)
#define CapSense_KEY02_ON_DEBOUNCE_SIZE                     (1u)
#define CapSense_KEY02_ON_DEBOUNCE_PARAM_ID                 (0x63820059u)

#define CapSense_KEY02_LOW_BSLN_RST_VALUE                   (CapSense_dsRam.wdgtList.key02.lowBslnRst)
#define CapSense_KEY02_LOW_BSLN_RST_OFFSET                  (90u)
#define CapSense_KEY02_LOW_BSLN_RST_SIZE                    (1u)
#define CapSense_KEY02_LOW_BSLN_RST_PARAM_ID                (0x6982005Au)

#define CapSense_KEY02_IDAC_MOD0_VALUE                      (CapSense_dsRam.wdgtList.key02.idacMod[0u])
#define CapSense_KEY02_IDAC_MOD0_OFFSET                     (91u)
#define CapSense_KEY02_IDAC_MOD0_SIZE                       (1u)
#define CapSense_KEY02_IDAC_MOD0_PARAM_ID                   (0x4902005Bu)

#define CapSense_KEY02_SNS_CLK_VALUE                        (CapSense_dsRam.wdgtList.key02.snsClk)
#define CapSense_KEY02_SNS_CLK_OFFSET                       (92u)
#define CapSense_KEY02_SNS_CLK_SIZE                         (2u)
#define CapSense_KEY02_SNS_CLK_PARAM_ID                     (0xAC82005Cu)

#define CapSense_KEY02_SNS_CLK_SOURCE_VALUE                 (CapSense_dsRam.wdgtList.key02.snsClkSource)
#define CapSense_KEY02_SNS_CLK_SOURCE_OFFSET                (94u)
#define CapSense_KEY02_SNS_CLK_SOURCE_SIZE                  (1u)
#define CapSense_KEY02_SNS_CLK_SOURCE_PARAM_ID              (0x4382005Eu)

#define CapSense_KEY03_CRC_VALUE                            (CapSense_dsRam.wdgtList.key03.crc)
#define CapSense_KEY03_CRC_OFFSET                           (96u)
#define CapSense_KEY03_CRC_SIZE                             (2u)
#define CapSense_KEY03_CRC_PARAM_ID                         (0x85030060u)

#define CapSense_KEY03_RESOLUTION_VALUE                     (CapSense_dsRam.wdgtList.key03.resolution)
#define CapSense_KEY03_RESOLUTION_OFFSET                    (98u)
#define CapSense_KEY03_RESOLUTION_SIZE                      (2u)
#define CapSense_KEY03_RESOLUTION_PARAM_ID                  (0xAF830062u)

#define CapSense_KEY03_FINGER_TH_VALUE                      (CapSense_dsRam.wdgtList.key03.fingerTh)
#define CapSense_KEY03_FINGER_TH_OFFSET                     (100u)
#define CapSense_KEY03_FINGER_TH_SIZE                       (2u)
#define CapSense_KEY03_FINGER_TH_PARAM_ID                   (0xA2830064u)

#define CapSense_KEY03_NOISE_TH_VALUE                       (CapSense_dsRam.wdgtList.key03.noiseTh)
#define CapSense_KEY03_NOISE_TH_OFFSET                      (102u)
#define CapSense_KEY03_NOISE_TH_SIZE                        (1u)
#define CapSense_KEY03_NOISE_TH_PARAM_ID                    (0x66830066u)

#define CapSense_KEY03_NNOISE_TH_VALUE                      (CapSense_dsRam.wdgtList.key03.nNoiseTh)
#define CapSense_KEY03_NNOISE_TH_OFFSET                     (103u)
#define CapSense_KEY03_NNOISE_TH_SIZE                       (1u)
#define CapSense_KEY03_NNOISE_TH_PARAM_ID                   (0x60830067u)

#define CapSense_KEY03_HYSTERESIS_VALUE                     (CapSense_dsRam.wdgtList.key03.hysteresis)
#define CapSense_KEY03_HYSTERESIS_OFFSET                    (104u)
#define CapSense_KEY03_HYSTERESIS_SIZE                      (1u)
#define CapSense_KEY03_HYSTERESIS_PARAM_ID                  (0x69830068u)

#define CapSense_KEY03_ON_DEBOUNCE_VALUE                    (CapSense_dsRam.wdgtList.key03.onDebounce)
#define CapSense_KEY03_ON_DEBOUNCE_OFFSET                   (105u)
#define CapSense_KEY03_ON_DEBOUNCE_SIZE                     (1u)
#define CapSense_KEY03_ON_DEBOUNCE_PARAM_ID                 (0x6F830069u)

#define CapSense_KEY03_LOW_BSLN_RST_VALUE                   (CapSense_dsRam.wdgtList.key03.lowBslnRst)
#define CapSense_KEY03_LOW_BSLN_RST_OFFSET                  (106u)
#define CapSense_KEY03_LOW_BSLN_RST_SIZE                    (1u)
#define CapSense_KEY03_LOW_BSLN_RST_PARAM_ID                (0x6583006Au)

#define CapSense_KEY03_IDAC_MOD0_VALUE                      (CapSense_dsRam.wdgtList.key03.idacMod[0u])
#define CapSense_KEY03_IDAC_MOD0_OFFSET                     (107u)
#define CapSense_KEY03_IDAC_MOD0_SIZE                       (1u)
#define CapSense_KEY03_IDAC_MOD0_PARAM_ID                   (0x4503006Bu)

#define CapSense_KEY03_SNS_CLK_VALUE                        (CapSense_dsRam.wdgtList.key03.snsClk)
#define CapSense_KEY03_SNS_CLK_OFFSET                       (108u)
#define CapSense_KEY03_SNS_CLK_SIZE                         (2u)
#define CapSense_KEY03_SNS_CLK_PARAM_ID                     (0xA083006Cu)

#define CapSense_KEY03_SNS_CLK_SOURCE_VALUE                 (CapSense_dsRam.wdgtList.key03.snsClkSource)
#define CapSense_KEY03_SNS_CLK_SOURCE_OFFSET                (110u)
#define CapSense_KEY03_SNS_CLK_SOURCE_SIZE                  (1u)
#define CapSense_KEY03_SNS_CLK_SOURCE_PARAM_ID              (0x4F83006Eu)

#define CapSense_KEY04_CRC_VALUE                            (CapSense_dsRam.wdgtList.key04.crc)
#define CapSense_KEY04_CRC_OFFSET                           (112u)
#define CapSense_KEY04_CRC_SIZE                             (2u)
#define CapSense_KEY04_CRC_PARAM_ID                         (0x89040070u)

#define CapSense_KEY04_RESOLUTION_VALUE                     (CapSense_dsRam.wdgtList.key04.resolution)
#define CapSense_KEY04_RESOLUTION_OFFSET                    (114u)
#define CapSense_KEY04_RESOLUTION_SIZE                      (2u)
#define CapSense_KEY04_RESOLUTION_PARAM_ID                  (0xA3840072u)

#define CapSense_KEY04_FINGER_TH_VALUE                      (CapSense_dsRam.wdgtList.key04.fingerTh)
#define CapSense_KEY04_FINGER_TH_OFFSET                     (116u)
#define CapSense_KEY04_FINGER_TH_SIZE                       (2u)
#define CapSense_KEY04_FINGER_TH_PARAM_ID                   (0xAE840074u)

#define CapSense_KEY04_NOISE_TH_VALUE                       (CapSense_dsRam.wdgtList.key04.noiseTh)
#define CapSense_KEY04_NOISE_TH_OFFSET                      (118u)
#define CapSense_KEY04_NOISE_TH_SIZE                        (1u)
#define CapSense_KEY04_NOISE_TH_PARAM_ID                    (0x6A840076u)

#define CapSense_KEY04_NNOISE_TH_VALUE                      (CapSense_dsRam.wdgtList.key04.nNoiseTh)
#define CapSense_KEY04_NNOISE_TH_OFFSET                     (119u)
#define CapSense_KEY04_NNOISE_TH_SIZE                       (1u)
#define CapSense_KEY04_NNOISE_TH_PARAM_ID                   (0x6C840077u)

#define CapSense_KEY04_HYSTERESIS_VALUE                     (CapSense_dsRam.wdgtList.key04.hysteresis)
#define CapSense_KEY04_HYSTERESIS_OFFSET                    (120u)
#define CapSense_KEY04_HYSTERESIS_SIZE                      (1u)
#define CapSense_KEY04_HYSTERESIS_PARAM_ID                  (0x65840078u)

#define CapSense_KEY04_ON_DEBOUNCE_VALUE                    (CapSense_dsRam.wdgtList.key04.onDebounce)
#define CapSense_KEY04_ON_DEBOUNCE_OFFSET                   (121u)
#define CapSense_KEY04_ON_DEBOUNCE_SIZE                     (1u)
#define CapSense_KEY04_ON_DEBOUNCE_PARAM_ID                 (0x63840079u)

#define CapSense_KEY04_LOW_BSLN_RST_VALUE                   (CapSense_dsRam.wdgtList.key04.lowBslnRst)
#define CapSense_KEY04_LOW_BSLN_RST_OFFSET                  (122u)
#define CapSense_KEY04_LOW_BSLN_RST_SIZE                    (1u)
#define CapSense_KEY04_LOW_BSLN_RST_PARAM_ID                (0x6984007Au)

#define CapSense_KEY04_IDAC_MOD0_VALUE                      (CapSense_dsRam.wdgtList.key04.idacMod[0u])
#define CapSense_KEY04_IDAC_MOD0_OFFSET                     (123u)
#define CapSense_KEY04_IDAC_MOD0_SIZE                       (1u)
#define CapSense_KEY04_IDAC_MOD0_PARAM_ID                   (0x4904007Bu)

#define CapSense_KEY04_SNS_CLK_VALUE                        (CapSense_dsRam.wdgtList.key04.snsClk)
#define CapSense_KEY04_SNS_CLK_OFFSET                       (124u)
#define CapSense_KEY04_SNS_CLK_SIZE                         (2u)
#define CapSense_KEY04_SNS_CLK_PARAM_ID                     (0xAC84007Cu)

#define CapSense_KEY04_SNS_CLK_SOURCE_VALUE                 (CapSense_dsRam.wdgtList.key04.snsClkSource)
#define CapSense_KEY04_SNS_CLK_SOURCE_OFFSET                (126u)
#define CapSense_KEY04_SNS_CLK_SOURCE_SIZE                  (1u)
#define CapSense_KEY04_SNS_CLK_SOURCE_PARAM_ID              (0x4384007Eu)

#define CapSense_KEY05_CRC_VALUE                            (CapSense_dsRam.wdgtList.key05.crc)
#define CapSense_KEY05_CRC_OFFSET                           (128u)
#define CapSense_KEY05_CRC_SIZE                             (2u)
#define CapSense_KEY05_CRC_PARAM_ID                         (0x8B050080u)

#define CapSense_KEY05_RESOLUTION_VALUE                     (CapSense_dsRam.wdgtList.key05.resolution)
#define CapSense_KEY05_RESOLUTION_OFFSET                    (130u)
#define CapSense_KEY05_RESOLUTION_SIZE                      (2u)
#define CapSense_KEY05_RESOLUTION_PARAM_ID                  (0xA1850082u)

#define CapSense_KEY05_FINGER_TH_VALUE                      (CapSense_dsRam.wdgtList.key05.fingerTh)
#define CapSense_KEY05_FINGER_TH_OFFSET                     (132u)
#define CapSense_KEY05_FINGER_TH_SIZE                       (2u)
#define CapSense_KEY05_FINGER_TH_PARAM_ID                   (0xAC850084u)

#define CapSense_KEY05_NOISE_TH_VALUE                       (CapSense_dsRam.wdgtList.key05.noiseTh)
#define CapSense_KEY05_NOISE_TH_OFFSET                      (134u)
#define CapSense_KEY05_NOISE_TH_SIZE                        (1u)
#define CapSense_KEY05_NOISE_TH_PARAM_ID                    (0x68850086u)

#define CapSense_KEY05_NNOISE_TH_VALUE                      (CapSense_dsRam.wdgtList.key05.nNoiseTh)
#define CapSense_KEY05_NNOISE_TH_OFFSET                     (135u)
#define CapSense_KEY05_NNOISE_TH_SIZE                       (1u)
#define CapSense_KEY05_NNOISE_TH_PARAM_ID                   (0x6E850087u)

#define CapSense_KEY05_HYSTERESIS_VALUE                     (CapSense_dsRam.wdgtList.key05.hysteresis)
#define CapSense_KEY05_HYSTERESIS_OFFSET                    (136u)
#define CapSense_KEY05_HYSTERESIS_SIZE                      (1u)
#define CapSense_KEY05_HYSTERESIS_PARAM_ID                  (0x67850088u)

#define CapSense_KEY05_ON_DEBOUNCE_VALUE                    (CapSense_dsRam.wdgtList.key05.onDebounce)
#define CapSense_KEY05_ON_DEBOUNCE_OFFSET                   (137u)
#define CapSense_KEY05_ON_DEBOUNCE_SIZE                     (1u)
#define CapSense_KEY05_ON_DEBOUNCE_PARAM_ID                 (0x61850089u)

#define CapSense_KEY05_LOW_BSLN_RST_VALUE                   (CapSense_dsRam.wdgtList.key05.lowBslnRst)
#define CapSense_KEY05_LOW_BSLN_RST_OFFSET                  (138u)
#define CapSense_KEY05_LOW_BSLN_RST_SIZE                    (1u)
#define CapSense_KEY05_LOW_BSLN_RST_PARAM_ID                (0x6B85008Au)

#define CapSense_KEY05_IDAC_MOD0_VALUE                      (CapSense_dsRam.wdgtList.key05.idacMod[0u])
#define CapSense_KEY05_IDAC_MOD0_OFFSET                     (139u)
#define CapSense_KEY05_IDAC_MOD0_SIZE                       (1u)
#define CapSense_KEY05_IDAC_MOD0_PARAM_ID                   (0x4B05008Bu)

#define CapSense_KEY05_SNS_CLK_VALUE                        (CapSense_dsRam.wdgtList.key05.snsClk)
#define CapSense_KEY05_SNS_CLK_OFFSET                       (140u)
#define CapSense_KEY05_SNS_CLK_SIZE                         (2u)
#define CapSense_KEY05_SNS_CLK_PARAM_ID                     (0xAE85008Cu)

#define CapSense_KEY05_SNS_CLK_SOURCE_VALUE                 (CapSense_dsRam.wdgtList.key05.snsClkSource)
#define CapSense_KEY05_SNS_CLK_SOURCE_OFFSET                (142u)
#define CapSense_KEY05_SNS_CLK_SOURCE_SIZE                  (1u)
#define CapSense_KEY05_SNS_CLK_SOURCE_PARAM_ID              (0x4185008Eu)

#define CapSense_KEY06_CRC_VALUE                            (CapSense_dsRam.wdgtList.key06.crc)
#define CapSense_KEY06_CRC_OFFSET                           (144u)
#define CapSense_KEY06_CRC_SIZE                             (2u)
#define CapSense_KEY06_CRC_PARAM_ID                         (0x8B060090u)

#define CapSense_KEY06_RESOLUTION_VALUE                     (CapSense_dsRam.wdgtList.key06.resolution)
#define CapSense_KEY06_RESOLUTION_OFFSET                    (146u)
#define CapSense_KEY06_RESOLUTION_SIZE                      (2u)
#define CapSense_KEY06_RESOLUTION_PARAM_ID                  (0xA1860092u)

#define CapSense_KEY06_FINGER_TH_VALUE                      (CapSense_dsRam.wdgtList.key06.fingerTh)
#define CapSense_KEY06_FINGER_TH_OFFSET                     (148u)
#define CapSense_KEY06_FINGER_TH_SIZE                       (2u)
#define CapSense_KEY06_FINGER_TH_PARAM_ID                   (0xAC860094u)

#define CapSense_KEY06_NOISE_TH_VALUE                       (CapSense_dsRam.wdgtList.key06.noiseTh)
#define CapSense_KEY06_NOISE_TH_OFFSET                      (150u)
#define CapSense_KEY06_NOISE_TH_SIZE                        (1u)
#define CapSense_KEY06_NOISE_TH_PARAM_ID                    (0x68860096u)

#define CapSense_KEY06_NNOISE_TH_VALUE                      (CapSense_dsRam.wdgtList.key06.nNoiseTh)
#define CapSense_KEY06_NNOISE_TH_OFFSET                     (151u)
#define CapSense_KEY06_NNOISE_TH_SIZE                       (1u)
#define CapSense_KEY06_NNOISE_TH_PARAM_ID                   (0x6E860097u)

#define CapSense_KEY06_HYSTERESIS_VALUE                     (CapSense_dsRam.wdgtList.key06.hysteresis)
#define CapSense_KEY06_HYSTERESIS_OFFSET                    (152u)
#define CapSense_KEY06_HYSTERESIS_SIZE                      (1u)
#define CapSense_KEY06_HYSTERESIS_PARAM_ID                  (0x67860098u)

#define CapSense_KEY06_ON_DEBOUNCE_VALUE                    (CapSense_dsRam.wdgtList.key06.onDebounce)
#define CapSense_KEY06_ON_DEBOUNCE_OFFSET                   (153u)
#define CapSense_KEY06_ON_DEBOUNCE_SIZE                     (1u)
#define CapSense_KEY06_ON_DEBOUNCE_PARAM_ID                 (0x61860099u)

#define CapSense_KEY06_LOW_BSLN_RST_VALUE                   (CapSense_dsRam.wdgtList.key06.lowBslnRst)
#define CapSense_KEY06_LOW_BSLN_RST_OFFSET                  (154u)
#define CapSense_KEY06_LOW_BSLN_RST_SIZE                    (1u)
#define CapSense_KEY06_LOW_BSLN_RST_PARAM_ID                (0x6B86009Au)

#define CapSense_KEY06_IDAC_MOD0_VALUE                      (CapSense_dsRam.wdgtList.key06.idacMod[0u])
#define CapSense_KEY06_IDAC_MOD0_OFFSET                     (155u)
#define CapSense_KEY06_IDAC_MOD0_SIZE                       (1u)
#define CapSense_KEY06_IDAC_MOD0_PARAM_ID                   (0x4B06009Bu)

#define CapSense_KEY06_SNS_CLK_VALUE                        (CapSense_dsRam.wdgtList.key06.snsClk)
#define CapSense_KEY06_SNS_CLK_OFFSET                       (156u)
#define CapSense_KEY06_SNS_CLK_SIZE                         (2u)
#define CapSense_KEY06_SNS_CLK_PARAM_ID                     (0xAE86009Cu)

#define CapSense_KEY06_SNS_CLK_SOURCE_VALUE                 (CapSense_dsRam.wdgtList.key06.snsClkSource)
#define CapSense_KEY06_SNS_CLK_SOURCE_OFFSET                (158u)
#define CapSense_KEY06_SNS_CLK_SOURCE_SIZE                  (1u)
#define CapSense_KEY06_SNS_CLK_SOURCE_PARAM_ID              (0x4186009Eu)

#define CapSense_KEY07_CRC_VALUE                            (CapSense_dsRam.wdgtList.key07.crc)
#define CapSense_KEY07_CRC_OFFSET                           (160u)
#define CapSense_KEY07_CRC_SIZE                             (2u)
#define CapSense_KEY07_CRC_PARAM_ID                         (0x870700A0u)

#define CapSense_KEY07_RESOLUTION_VALUE                     (CapSense_dsRam.wdgtList.key07.resolution)
#define CapSense_KEY07_RESOLUTION_OFFSET                    (162u)
#define CapSense_KEY07_RESOLUTION_SIZE                      (2u)
#define CapSense_KEY07_RESOLUTION_PARAM_ID                  (0xAD8700A2u)

#define CapSense_KEY07_FINGER_TH_VALUE                      (CapSense_dsRam.wdgtList.key07.fingerTh)
#define CapSense_KEY07_FINGER_TH_OFFSET                     (164u)
#define CapSense_KEY07_FINGER_TH_SIZE                       (2u)
#define CapSense_KEY07_FINGER_TH_PARAM_ID                   (0xA08700A4u)

#define CapSense_KEY07_NOISE_TH_VALUE                       (CapSense_dsRam.wdgtList.key07.noiseTh)
#define CapSense_KEY07_NOISE_TH_OFFSET                      (166u)
#define CapSense_KEY07_NOISE_TH_SIZE                        (1u)
#define CapSense_KEY07_NOISE_TH_PARAM_ID                    (0x648700A6u)

#define CapSense_KEY07_NNOISE_TH_VALUE                      (CapSense_dsRam.wdgtList.key07.nNoiseTh)
#define CapSense_KEY07_NNOISE_TH_OFFSET                     (167u)
#define CapSense_KEY07_NNOISE_TH_SIZE                       (1u)
#define CapSense_KEY07_NNOISE_TH_PARAM_ID                   (0x628700A7u)

#define CapSense_KEY07_HYSTERESIS_VALUE                     (CapSense_dsRam.wdgtList.key07.hysteresis)
#define CapSense_KEY07_HYSTERESIS_OFFSET                    (168u)
#define CapSense_KEY07_HYSTERESIS_SIZE                      (1u)
#define CapSense_KEY07_HYSTERESIS_PARAM_ID                  (0x6B8700A8u)

#define CapSense_KEY07_ON_DEBOUNCE_VALUE                    (CapSense_dsRam.wdgtList.key07.onDebounce)
#define CapSense_KEY07_ON_DEBOUNCE_OFFSET                   (169u)
#define CapSense_KEY07_ON_DEBOUNCE_SIZE                     (1u)
#define CapSense_KEY07_ON_DEBOUNCE_PARAM_ID                 (0x6D8700A9u)

#define CapSense_KEY07_LOW_BSLN_RST_VALUE                   (CapSense_dsRam.wdgtList.key07.lowBslnRst)
#define CapSense_KEY07_LOW_BSLN_RST_OFFSET                  (170u)
#define CapSense_KEY07_LOW_BSLN_RST_SIZE                    (1u)
#define CapSense_KEY07_LOW_BSLN_RST_PARAM_ID                (0x678700AAu)

#define CapSense_KEY07_IDAC_MOD0_VALUE                      (CapSense_dsRam.wdgtList.key07.idacMod[0u])
#define CapSense_KEY07_IDAC_MOD0_OFFSET                     (171u)
#define CapSense_KEY07_IDAC_MOD0_SIZE                       (1u)
#define CapSense_KEY07_IDAC_MOD0_PARAM_ID                   (0x470700ABu)

#define CapSense_KEY07_SNS_CLK_VALUE                        (CapSense_dsRam.wdgtList.key07.snsClk)
#define CapSense_KEY07_SNS_CLK_OFFSET                       (172u)
#define CapSense_KEY07_SNS_CLK_SIZE                         (2u)
#define CapSense_KEY07_SNS_CLK_PARAM_ID                     (0xA28700ACu)

#define CapSense_KEY07_SNS_CLK_SOURCE_VALUE                 (CapSense_dsRam.wdgtList.key07.snsClkSource)
#define CapSense_KEY07_SNS_CLK_SOURCE_OFFSET                (174u)
#define CapSense_KEY07_SNS_CLK_SOURCE_SIZE                  (1u)
#define CapSense_KEY07_SNS_CLK_SOURCE_PARAM_ID              (0x4D8700AEu)

#define CapSense_KEY08_CRC_VALUE                            (CapSense_dsRam.wdgtList.key08.crc)
#define CapSense_KEY08_CRC_OFFSET                           (176u)
#define CapSense_KEY08_CRC_SIZE                             (2u)
#define CapSense_KEY08_CRC_PARAM_ID                         (0x8A0800B0u)

#define CapSense_KEY08_RESOLUTION_VALUE                     (CapSense_dsRam.wdgtList.key08.resolution)
#define CapSense_KEY08_RESOLUTION_OFFSET                    (178u)
#define CapSense_KEY08_RESOLUTION_SIZE                      (2u)
#define CapSense_KEY08_RESOLUTION_PARAM_ID                  (0xA08800B2u)

#define CapSense_KEY08_FINGER_TH_VALUE                      (CapSense_dsRam.wdgtList.key08.fingerTh)
#define CapSense_KEY08_FINGER_TH_OFFSET                     (180u)
#define CapSense_KEY08_FINGER_TH_SIZE                       (2u)
#define CapSense_KEY08_FINGER_TH_PARAM_ID                   (0xAD8800B4u)

#define CapSense_KEY08_NOISE_TH_VALUE                       (CapSense_dsRam.wdgtList.key08.noiseTh)
#define CapSense_KEY08_NOISE_TH_OFFSET                      (182u)
#define CapSense_KEY08_NOISE_TH_SIZE                        (1u)
#define CapSense_KEY08_NOISE_TH_PARAM_ID                    (0x698800B6u)

#define CapSense_KEY08_NNOISE_TH_VALUE                      (CapSense_dsRam.wdgtList.key08.nNoiseTh)
#define CapSense_KEY08_NNOISE_TH_OFFSET                     (183u)
#define CapSense_KEY08_NNOISE_TH_SIZE                       (1u)
#define CapSense_KEY08_NNOISE_TH_PARAM_ID                   (0x6F8800B7u)

#define CapSense_KEY08_HYSTERESIS_VALUE                     (CapSense_dsRam.wdgtList.key08.hysteresis)
#define CapSense_KEY08_HYSTERESIS_OFFSET                    (184u)
#define CapSense_KEY08_HYSTERESIS_SIZE                      (1u)
#define CapSense_KEY08_HYSTERESIS_PARAM_ID                  (0x668800B8u)

#define CapSense_KEY08_ON_DEBOUNCE_VALUE                    (CapSense_dsRam.wdgtList.key08.onDebounce)
#define CapSense_KEY08_ON_DEBOUNCE_OFFSET                   (185u)
#define CapSense_KEY08_ON_DEBOUNCE_SIZE                     (1u)
#define CapSense_KEY08_ON_DEBOUNCE_PARAM_ID                 (0x608800B9u)

#define CapSense_KEY08_LOW_BSLN_RST_VALUE                   (CapSense_dsRam.wdgtList.key08.lowBslnRst)
#define CapSense_KEY08_LOW_BSLN_RST_OFFSET                  (186u)
#define CapSense_KEY08_LOW_BSLN_RST_SIZE                    (1u)
#define CapSense_KEY08_LOW_BSLN_RST_PARAM_ID                (0x6A8800BAu)

#define CapSense_KEY08_IDAC_MOD0_VALUE                      (CapSense_dsRam.wdgtList.key08.idacMod[0u])
#define CapSense_KEY08_IDAC_MOD0_OFFSET                     (187u)
#define CapSense_KEY08_IDAC_MOD0_SIZE                       (1u)
#define CapSense_KEY08_IDAC_MOD0_PARAM_ID                   (0x4A0800BBu)

#define CapSense_KEY08_SNS_CLK_VALUE                        (CapSense_dsRam.wdgtList.key08.snsClk)
#define CapSense_KEY08_SNS_CLK_OFFSET                       (188u)
#define CapSense_KEY08_SNS_CLK_SIZE                         (2u)
#define CapSense_KEY08_SNS_CLK_PARAM_ID                     (0xAF8800BCu)

#define CapSense_KEY08_SNS_CLK_SOURCE_VALUE                 (CapSense_dsRam.wdgtList.key08.snsClkSource)
#define CapSense_KEY08_SNS_CLK_SOURCE_OFFSET                (190u)
#define CapSense_KEY08_SNS_CLK_SOURCE_SIZE                  (1u)
#define CapSense_KEY08_SNS_CLK_SOURCE_PARAM_ID              (0x408800BEu)

#define CapSense_KEY09_CRC_VALUE                            (CapSense_dsRam.wdgtList.key09.crc)
#define CapSense_KEY09_CRC_OFFSET                           (192u)
#define CapSense_KEY09_CRC_SIZE                             (2u)
#define CapSense_KEY09_CRC_PARAM_ID                         (0x8B0900C0u)

#define CapSense_KEY09_RESOLUTION_VALUE                     (CapSense_dsRam.wdgtList.key09.resolution)
#define CapSense_KEY09_RESOLUTION_OFFSET                    (194u)
#define CapSense_KEY09_RESOLUTION_SIZE                      (2u)
#define CapSense_KEY09_RESOLUTION_PARAM_ID                  (0xA18900C2u)

#define CapSense_KEY09_FINGER_TH_VALUE                      (CapSense_dsRam.wdgtList.key09.fingerTh)
#define CapSense_KEY09_FINGER_TH_OFFSET                     (196u)
#define CapSense_KEY09_FINGER_TH_SIZE                       (2u)
#define CapSense_KEY09_FINGER_TH_PARAM_ID                   (0xAC8900C4u)

#define CapSense_KEY09_NOISE_TH_VALUE                       (CapSense_dsRam.wdgtList.key09.noiseTh)
#define CapSense_KEY09_NOISE_TH_OFFSET                      (198u)
#define CapSense_KEY09_NOISE_TH_SIZE                        (1u)
#define CapSense_KEY09_NOISE_TH_PARAM_ID                    (0x688900C6u)

#define CapSense_KEY09_NNOISE_TH_VALUE                      (CapSense_dsRam.wdgtList.key09.nNoiseTh)
#define CapSense_KEY09_NNOISE_TH_OFFSET                     (199u)
#define CapSense_KEY09_NNOISE_TH_SIZE                       (1u)
#define CapSense_KEY09_NNOISE_TH_PARAM_ID                   (0x6E8900C7u)

#define CapSense_KEY09_HYSTERESIS_VALUE                     (CapSense_dsRam.wdgtList.key09.hysteresis)
#define CapSense_KEY09_HYSTERESIS_OFFSET                    (200u)
#define CapSense_KEY09_HYSTERESIS_SIZE                      (1u)
#define CapSense_KEY09_HYSTERESIS_PARAM_ID                  (0x678900C8u)

#define CapSense_KEY09_ON_DEBOUNCE_VALUE                    (CapSense_dsRam.wdgtList.key09.onDebounce)
#define CapSense_KEY09_ON_DEBOUNCE_OFFSET                   (201u)
#define CapSense_KEY09_ON_DEBOUNCE_SIZE                     (1u)
#define CapSense_KEY09_ON_DEBOUNCE_PARAM_ID                 (0x618900C9u)

#define CapSense_KEY09_LOW_BSLN_RST_VALUE                   (CapSense_dsRam.wdgtList.key09.lowBslnRst)
#define CapSense_KEY09_LOW_BSLN_RST_OFFSET                  (202u)
#define CapSense_KEY09_LOW_BSLN_RST_SIZE                    (1u)
#define CapSense_KEY09_LOW_BSLN_RST_PARAM_ID                (0x6B8900CAu)

#define CapSense_KEY09_IDAC_MOD0_VALUE                      (CapSense_dsRam.wdgtList.key09.idacMod[0u])
#define CapSense_KEY09_IDAC_MOD0_OFFSET                     (203u)
#define CapSense_KEY09_IDAC_MOD0_SIZE                       (1u)
#define CapSense_KEY09_IDAC_MOD0_PARAM_ID                   (0x4B0900CBu)

#define CapSense_KEY09_SNS_CLK_VALUE                        (CapSense_dsRam.wdgtList.key09.snsClk)
#define CapSense_KEY09_SNS_CLK_OFFSET                       (204u)
#define CapSense_KEY09_SNS_CLK_SIZE                         (2u)
#define CapSense_KEY09_SNS_CLK_PARAM_ID                     (0xAE8900CCu)

#define CapSense_KEY09_SNS_CLK_SOURCE_VALUE                 (CapSense_dsRam.wdgtList.key09.snsClkSource)
#define CapSense_KEY09_SNS_CLK_SOURCE_OFFSET                (206u)
#define CapSense_KEY09_SNS_CLK_SOURCE_SIZE                  (1u)
#define CapSense_KEY09_SNS_CLK_SOURCE_PARAM_ID              (0x418900CEu)

#define CapSense_KEY10_CRC_VALUE                            (CapSense_dsRam.wdgtList.key10.crc)
#define CapSense_KEY10_CRC_OFFSET                           (208u)
#define CapSense_KEY10_CRC_SIZE                             (2u)
#define CapSense_KEY10_CRC_PARAM_ID                         (0x8B0A00D0u)

#define CapSense_KEY10_RESOLUTION_VALUE                     (CapSense_dsRam.wdgtList.key10.resolution)
#define CapSense_KEY10_RESOLUTION_OFFSET                    (210u)
#define CapSense_KEY10_RESOLUTION_SIZE                      (2u)
#define CapSense_KEY10_RESOLUTION_PARAM_ID                  (0xA18A00D2u)

#define CapSense_KEY10_FINGER_TH_VALUE                      (CapSense_dsRam.wdgtList.key10.fingerTh)
#define CapSense_KEY10_FINGER_TH_OFFSET                     (212u)
#define CapSense_KEY10_FINGER_TH_SIZE                       (2u)
#define CapSense_KEY10_FINGER_TH_PARAM_ID                   (0xAC8A00D4u)

#define CapSense_KEY10_NOISE_TH_VALUE                       (CapSense_dsRam.wdgtList.key10.noiseTh)
#define CapSense_KEY10_NOISE_TH_OFFSET                      (214u)
#define CapSense_KEY10_NOISE_TH_SIZE                        (1u)
#define CapSense_KEY10_NOISE_TH_PARAM_ID                    (0x688A00D6u)

#define CapSense_KEY10_NNOISE_TH_VALUE                      (CapSense_dsRam.wdgtList.key10.nNoiseTh)
#define CapSense_KEY10_NNOISE_TH_OFFSET                     (215u)
#define CapSense_KEY10_NNOISE_TH_SIZE                       (1u)
#define CapSense_KEY10_NNOISE_TH_PARAM_ID                   (0x6E8A00D7u)

#define CapSense_KEY10_HYSTERESIS_VALUE                     (CapSense_dsRam.wdgtList.key10.hysteresis)
#define CapSense_KEY10_HYSTERESIS_OFFSET                    (216u)
#define CapSense_KEY10_HYSTERESIS_SIZE                      (1u)
#define CapSense_KEY10_HYSTERESIS_PARAM_ID                  (0x678A00D8u)

#define CapSense_KEY10_ON_DEBOUNCE_VALUE                    (CapSense_dsRam.wdgtList.key10.onDebounce)
#define CapSense_KEY10_ON_DEBOUNCE_OFFSET                   (217u)
#define CapSense_KEY10_ON_DEBOUNCE_SIZE                     (1u)
#define CapSense_KEY10_ON_DEBOUNCE_PARAM_ID                 (0x618A00D9u)

#define CapSense_KEY10_LOW_BSLN_RST_VALUE                   (CapSense_dsRam.wdgtList.key10.lowBslnRst)
#define CapSense_KEY10_LOW_BSLN_RST_OFFSET                  (218u)
#define CapSense_KEY10_LOW_BSLN_RST_SIZE                    (1u)
#define CapSense_KEY10_LOW_BSLN_RST_PARAM_ID                (0x6B8A00DAu)

#define CapSense_KEY10_IDAC_MOD0_VALUE                      (CapSense_dsRam.wdgtList.key10.idacMod[0u])
#define CapSense_KEY10_IDAC_MOD0_OFFSET                     (219u)
#define CapSense_KEY10_IDAC_MOD0_SIZE                       (1u)
#define CapSense_KEY10_IDAC_MOD0_PARAM_ID                   (0x4B0A00DBu)

#define CapSense_KEY10_SNS_CLK_VALUE                        (CapSense_dsRam.wdgtList.key10.snsClk)
#define CapSense_KEY10_SNS_CLK_OFFSET                       (220u)
#define CapSense_KEY10_SNS_CLK_SIZE                         (2u)
#define CapSense_KEY10_SNS_CLK_PARAM_ID                     (0xAE8A00DCu)

#define CapSense_KEY10_SNS_CLK_SOURCE_VALUE                 (CapSense_dsRam.wdgtList.key10.snsClkSource)
#define CapSense_KEY10_SNS_CLK_SOURCE_OFFSET                (222u)
#define CapSense_KEY10_SNS_CLK_SOURCE_SIZE                  (1u)
#define CapSense_KEY10_SNS_CLK_SOURCE_PARAM_ID              (0x418A00DEu)

#define CapSense_KEY11_CRC_VALUE                            (CapSense_dsRam.wdgtList.key11.crc)
#define CapSense_KEY11_CRC_OFFSET                           (224u)
#define CapSense_KEY11_CRC_SIZE                             (2u)
#define CapSense_KEY11_CRC_PARAM_ID                         (0x870B00E0u)

#define CapSense_KEY11_RESOLUTION_VALUE                     (CapSense_dsRam.wdgtList.key11.resolution)
#define CapSense_KEY11_RESOLUTION_OFFSET                    (226u)
#define CapSense_KEY11_RESOLUTION_SIZE                      (2u)
#define CapSense_KEY11_RESOLUTION_PARAM_ID                  (0xAD8B00E2u)

#define CapSense_KEY11_FINGER_TH_VALUE                      (CapSense_dsRam.wdgtList.key11.fingerTh)
#define CapSense_KEY11_FINGER_TH_OFFSET                     (228u)
#define CapSense_KEY11_FINGER_TH_SIZE                       (2u)
#define CapSense_KEY11_FINGER_TH_PARAM_ID                   (0xA08B00E4u)

#define CapSense_KEY11_NOISE_TH_VALUE                       (CapSense_dsRam.wdgtList.key11.noiseTh)
#define CapSense_KEY11_NOISE_TH_OFFSET                      (230u)
#define CapSense_KEY11_NOISE_TH_SIZE                        (1u)
#define CapSense_KEY11_NOISE_TH_PARAM_ID                    (0x648B00E6u)

#define CapSense_KEY11_NNOISE_TH_VALUE                      (CapSense_dsRam.wdgtList.key11.nNoiseTh)
#define CapSense_KEY11_NNOISE_TH_OFFSET                     (231u)
#define CapSense_KEY11_NNOISE_TH_SIZE                       (1u)
#define CapSense_KEY11_NNOISE_TH_PARAM_ID                   (0x628B00E7u)

#define CapSense_KEY11_HYSTERESIS_VALUE                     (CapSense_dsRam.wdgtList.key11.hysteresis)
#define CapSense_KEY11_HYSTERESIS_OFFSET                    (232u)
#define CapSense_KEY11_HYSTERESIS_SIZE                      (1u)
#define CapSense_KEY11_HYSTERESIS_PARAM_ID                  (0x6B8B00E8u)

#define CapSense_KEY11_ON_DEBOUNCE_VALUE                    (CapSense_dsRam.wdgtList.key11.onDebounce)
#define CapSense_KEY11_ON_DEBOUNCE_OFFSET                   (233u)
#define CapSense_KEY11_ON_DEBOUNCE_SIZE                     (1u)
#define CapSense_KEY11_ON_DEBOUNCE_PARAM_ID                 (0x6D8B00E9u)

#define CapSense_KEY11_LOW_BSLN_RST_VALUE                   (CapSense_dsRam.wdgtList.key11.lowBslnRst)
#define CapSense_KEY11_LOW_BSLN_RST_OFFSET                  (234u)
#define CapSense_KEY11_LOW_BSLN_RST_SIZE                    (1u)
#define CapSense_KEY11_LOW_BSLN_RST_PARAM_ID                (0x678B00EAu)

#define CapSense_KEY11_IDAC_MOD0_VALUE                      (CapSense_dsRam.wdgtList.key11.idacMod[0u])
#define CapSense_KEY11_IDAC_MOD0_OFFSET                     (235u)
#define CapSense_KEY11_IDAC_MOD0_SIZE                       (1u)
#define CapSense_KEY11_IDAC_MOD0_PARAM_ID                   (0x470B00EBu)

#define CapSense_KEY11_SNS_CLK_VALUE                        (CapSense_dsRam.wdgtList.key11.snsClk)
#define CapSense_KEY11_SNS_CLK_OFFSET                       (236u)
#define CapSense_KEY11_SNS_CLK_SIZE                         (2u)
#define CapSense_KEY11_SNS_CLK_PARAM_ID                     (0xA28B00ECu)

#define CapSense_KEY11_SNS_CLK_SOURCE_VALUE                 (CapSense_dsRam.wdgtList.key11.snsClkSource)
#define CapSense_KEY11_SNS_CLK_SOURCE_OFFSET                (238u)
#define CapSense_KEY11_SNS_CLK_SOURCE_SIZE                  (1u)
#define CapSense_KEY11_SNS_CLK_SOURCE_PARAM_ID              (0x4D8B00EEu)

#define CapSense_KEY12_CRC_VALUE                            (CapSense_dsRam.wdgtList.key12.crc)
#define CapSense_KEY12_CRC_OFFSET                           (240u)
#define CapSense_KEY12_CRC_SIZE                             (2u)
#define CapSense_KEY12_CRC_PARAM_ID                         (0x8B0C00F0u)

#define CapSense_KEY12_RESOLUTION_VALUE                     (CapSense_dsRam.wdgtList.key12.resolution)
#define CapSense_KEY12_RESOLUTION_OFFSET                    (242u)
#define CapSense_KEY12_RESOLUTION_SIZE                      (2u)
#define CapSense_KEY12_RESOLUTION_PARAM_ID                  (0xA18C00F2u)

#define CapSense_KEY12_FINGER_TH_VALUE                      (CapSense_dsRam.wdgtList.key12.fingerTh)
#define CapSense_KEY12_FINGER_TH_OFFSET                     (244u)
#define CapSense_KEY12_FINGER_TH_SIZE                       (2u)
#define CapSense_KEY12_FINGER_TH_PARAM_ID                   (0xAC8C00F4u)

#define CapSense_KEY12_NOISE_TH_VALUE                       (CapSense_dsRam.wdgtList.key12.noiseTh)
#define CapSense_KEY12_NOISE_TH_OFFSET                      (246u)
#define CapSense_KEY12_NOISE_TH_SIZE                        (1u)
#define CapSense_KEY12_NOISE_TH_PARAM_ID                    (0x688C00F6u)

#define CapSense_KEY12_NNOISE_TH_VALUE                      (CapSense_dsRam.wdgtList.key12.nNoiseTh)
#define CapSense_KEY12_NNOISE_TH_OFFSET                     (247u)
#define CapSense_KEY12_NNOISE_TH_SIZE                       (1u)
#define CapSense_KEY12_NNOISE_TH_PARAM_ID                   (0x6E8C00F7u)

#define CapSense_KEY12_HYSTERESIS_VALUE                     (CapSense_dsRam.wdgtList.key12.hysteresis)
#define CapSense_KEY12_HYSTERESIS_OFFSET                    (248u)
#define CapSense_KEY12_HYSTERESIS_SIZE                      (1u)
#define CapSense_KEY12_HYSTERESIS_PARAM_ID                  (0x678C00F8u)

#define CapSense_KEY12_ON_DEBOUNCE_VALUE                    (CapSense_dsRam.wdgtList.key12.onDebounce)
#define CapSense_KEY12_ON_DEBOUNCE_OFFSET                   (249u)
#define CapSense_KEY12_ON_DEBOUNCE_SIZE                     (1u)
#define CapSense_KEY12_ON_DEBOUNCE_PARAM_ID                 (0x618C00F9u)

#define CapSense_KEY12_LOW_BSLN_RST_VALUE                   (CapSense_dsRam.wdgtList.key12.lowBslnRst)
#define CapSense_KEY12_LOW_BSLN_RST_OFFSET                  (250u)
#define CapSense_KEY12_LOW_BSLN_RST_SIZE                    (1u)
#define CapSense_KEY12_LOW_BSLN_RST_PARAM_ID                (0x6B8C00FAu)

#define CapSense_KEY12_IDAC_MOD0_VALUE                      (CapSense_dsRam.wdgtList.key12.idacMod[0u])
#define CapSense_KEY12_IDAC_MOD0_OFFSET                     (251u)
#define CapSense_KEY12_IDAC_MOD0_SIZE                       (1u)
#define CapSense_KEY12_IDAC_MOD0_PARAM_ID                   (0x4B0C00FBu)

#define CapSense_KEY12_SNS_CLK_VALUE                        (CapSense_dsRam.wdgtList.key12.snsClk)
#define CapSense_KEY12_SNS_CLK_OFFSET                       (252u)
#define CapSense_KEY12_SNS_CLK_SIZE                         (2u)
#define CapSense_KEY12_SNS_CLK_PARAM_ID                     (0xAE8C00FCu)

#define CapSense_KEY12_SNS_CLK_SOURCE_VALUE                 (CapSense_dsRam.wdgtList.key12.snsClkSource)
#define CapSense_KEY12_SNS_CLK_SOURCE_OFFSET                (254u)
#define CapSense_KEY12_SNS_CLK_SOURCE_SIZE                  (1u)
#define CapSense_KEY12_SNS_CLK_SOURCE_PARAM_ID              (0x418C00FEu)

#define CapSense_KEY00_SNS0_RAW0_VALUE                      (CapSense_dsRam.snsList.key00[0u].raw[0u])
#define CapSense_KEY00_SNS0_RAW0_OFFSET                     (256u)
#define CapSense_KEY00_SNS0_RAW0_SIZE                       (2u)
#define CapSense_KEY00_SNS0_RAW0_PARAM_ID                   (0x8C000100u)

#define CapSense_KEY00_SNS0_BSLN0_VALUE                     (CapSense_dsRam.snsList.key00[0u].bsln[0u])
#define CapSense_KEY00_SNS0_BSLN0_OFFSET                    (258u)
#define CapSense_KEY00_SNS0_BSLN0_SIZE                      (2u)
#define CapSense_KEY00_SNS0_BSLN0_PARAM_ID                  (0x80000102u)

#define CapSense_KEY00_SNS0_BSLN_INV0_VALUE                 (CapSense_dsRam.snsList.key00[0u].bslnInv[0u])
#define CapSense_KEY00_SNS0_BSLN_INV0_OFFSET                (260u)
#define CapSense_KEY00_SNS0_BSLN_INV0_SIZE                  (2u)
#define CapSense_KEY00_SNS0_BSLN_INV0_PARAM_ID              (0x8D000104u)

#define CapSense_KEY00_SNS0_BSLN_EXT0_VALUE                 (CapSense_dsRam.snsList.key00[0u].bslnExt[0u])
#define CapSense_KEY00_SNS0_BSLN_EXT0_OFFSET                (262u)
#define CapSense_KEY00_SNS0_BSLN_EXT0_SIZE                  (1u)
#define CapSense_KEY00_SNS0_BSLN_EXT0_PARAM_ID              (0x49000106u)

#define CapSense_KEY00_SNS0_DIFF_VALUE                      (CapSense_dsRam.snsList.key00[0u].diff)
#define CapSense_KEY00_SNS0_DIFF_OFFSET                     (264u)
#define CapSense_KEY00_SNS0_DIFF_SIZE                       (2u)
#define CapSense_KEY00_SNS0_DIFF_PARAM_ID                   (0x8E000108u)

#define CapSense_KEY00_SNS0_NEG_BSLN_RST_CNT0_VALUE         (CapSense_dsRam.snsList.key00[0u].negBslnRstCnt[0u])
#define CapSense_KEY00_SNS0_NEG_BSLN_RST_CNT0_OFFSET        (266u)
#define CapSense_KEY00_SNS0_NEG_BSLN_RST_CNT0_SIZE          (1u)
#define CapSense_KEY00_SNS0_NEG_BSLN_RST_CNT0_PARAM_ID      (0x4A00010Au)

#define CapSense_KEY00_SNS0_IDAC_COMP0_VALUE                (CapSense_dsRam.snsList.key00[0u].idacComp[0u])
#define CapSense_KEY00_SNS0_IDAC_COMP0_OFFSET               (267u)
#define CapSense_KEY00_SNS0_IDAC_COMP0_SIZE                 (1u)
#define CapSense_KEY00_SNS0_IDAC_COMP0_PARAM_ID             (0x4C00010Bu)

#define CapSense_KEY01_SNS0_RAW0_VALUE                      (CapSense_dsRam.snsList.key01[0u].raw[0u])
#define CapSense_KEY01_SNS0_RAW0_OFFSET                     (268u)
#define CapSense_KEY01_SNS0_RAW0_SIZE                       (2u)
#define CapSense_KEY01_SNS0_RAW0_PARAM_ID                   (0x8F00010Cu)

#define CapSense_KEY01_SNS0_BSLN0_VALUE                     (CapSense_dsRam.snsList.key01[0u].bsln[0u])
#define CapSense_KEY01_SNS0_BSLN0_OFFSET                    (270u)
#define CapSense_KEY01_SNS0_BSLN0_SIZE                      (2u)
#define CapSense_KEY01_SNS0_BSLN0_PARAM_ID                  (0x8300010Eu)

#define CapSense_KEY01_SNS0_BSLN_INV0_VALUE                 (CapSense_dsRam.snsList.key01[0u].bslnInv[0u])
#define CapSense_KEY01_SNS0_BSLN_INV0_OFFSET                (272u)
#define CapSense_KEY01_SNS0_BSLN_INV0_SIZE                  (2u)
#define CapSense_KEY01_SNS0_BSLN_INV0_PARAM_ID              (0x89000110u)

#define CapSense_KEY01_SNS0_BSLN_EXT0_VALUE                 (CapSense_dsRam.snsList.key01[0u].bslnExt[0u])
#define CapSense_KEY01_SNS0_BSLN_EXT0_OFFSET                (274u)
#define CapSense_KEY01_SNS0_BSLN_EXT0_SIZE                  (1u)
#define CapSense_KEY01_SNS0_BSLN_EXT0_PARAM_ID              (0x4D000112u)

#define CapSense_KEY01_SNS0_DIFF_VALUE                      (CapSense_dsRam.snsList.key01[0u].diff)
#define CapSense_KEY01_SNS0_DIFF_OFFSET                     (276u)
#define CapSense_KEY01_SNS0_DIFF_SIZE                       (2u)
#define CapSense_KEY01_SNS0_DIFF_PARAM_ID                   (0x88000114u)

#define CapSense_KEY01_SNS0_NEG_BSLN_RST_CNT0_VALUE         (CapSense_dsRam.snsList.key01[0u].negBslnRstCnt[0u])
#define CapSense_KEY01_SNS0_NEG_BSLN_RST_CNT0_OFFSET        (278u)
#define CapSense_KEY01_SNS0_NEG_BSLN_RST_CNT0_SIZE          (1u)
#define CapSense_KEY01_SNS0_NEG_BSLN_RST_CNT0_PARAM_ID      (0x4C000116u)

#define CapSense_KEY01_SNS0_IDAC_COMP0_VALUE                (CapSense_dsRam.snsList.key01[0u].idacComp[0u])
#define CapSense_KEY01_SNS0_IDAC_COMP0_OFFSET               (279u)
#define CapSense_KEY01_SNS0_IDAC_COMP0_SIZE                 (1u)
#define CapSense_KEY01_SNS0_IDAC_COMP0_PARAM_ID             (0x4A000117u)

#define CapSense_KEY02_SNS0_RAW0_VALUE                      (CapSense_dsRam.snsList.key02[0u].raw[0u])
#define CapSense_KEY02_SNS0_RAW0_OFFSET                     (280u)
#define CapSense_KEY02_SNS0_RAW0_SIZE                       (2u)
#define CapSense_KEY02_SNS0_RAW0_PARAM_ID                   (0x8B000118u)

#define CapSense_KEY02_SNS0_BSLN0_VALUE                     (CapSense_dsRam.snsList.key02[0u].bsln[0u])
#define CapSense_KEY02_SNS0_BSLN0_OFFSET                    (282u)
#define CapSense_KEY02_SNS0_BSLN0_SIZE                      (2u)
#define CapSense_KEY02_SNS0_BSLN0_PARAM_ID                  (0x8700011Au)

#define CapSense_KEY02_SNS0_BSLN_INV0_VALUE                 (CapSense_dsRam.snsList.key02[0u].bslnInv[0u])
#define CapSense_KEY02_SNS0_BSLN_INV0_OFFSET                (284u)
#define CapSense_KEY02_SNS0_BSLN_INV0_SIZE                  (2u)
#define CapSense_KEY02_SNS0_BSLN_INV0_PARAM_ID              (0x8A00011Cu)

#define CapSense_KEY02_SNS0_BSLN_EXT0_VALUE                 (CapSense_dsRam.snsList.key02[0u].bslnExt[0u])
#define CapSense_KEY02_SNS0_BSLN_EXT0_OFFSET                (286u)
#define CapSense_KEY02_SNS0_BSLN_EXT0_SIZE                  (1u)
#define CapSense_KEY02_SNS0_BSLN_EXT0_PARAM_ID              (0x4E00011Eu)

#define CapSense_KEY02_SNS0_DIFF_VALUE                      (CapSense_dsRam.snsList.key02[0u].diff)
#define CapSense_KEY02_SNS0_DIFF_OFFSET                     (288u)
#define CapSense_KEY02_SNS0_DIFF_SIZE                       (2u)
#define CapSense_KEY02_SNS0_DIFF_PARAM_ID                   (0x86000120u)

#define CapSense_KEY02_SNS0_NEG_BSLN_RST_CNT0_VALUE         (CapSense_dsRam.snsList.key02[0u].negBslnRstCnt[0u])
#define CapSense_KEY02_SNS0_NEG_BSLN_RST_CNT0_OFFSET        (290u)
#define CapSense_KEY02_SNS0_NEG_BSLN_RST_CNT0_SIZE          (1u)
#define CapSense_KEY02_SNS0_NEG_BSLN_RST_CNT0_PARAM_ID      (0x42000122u)

#define CapSense_KEY02_SNS0_IDAC_COMP0_VALUE                (CapSense_dsRam.snsList.key02[0u].idacComp[0u])
#define CapSense_KEY02_SNS0_IDAC_COMP0_OFFSET               (291u)
#define CapSense_KEY02_SNS0_IDAC_COMP0_SIZE                 (1u)
#define CapSense_KEY02_SNS0_IDAC_COMP0_PARAM_ID             (0x44000123u)

#define CapSense_KEY03_SNS0_RAW0_VALUE                      (CapSense_dsRam.snsList.key03[0u].raw[0u])
#define CapSense_KEY03_SNS0_RAW0_OFFSET                     (292u)
#define CapSense_KEY03_SNS0_RAW0_SIZE                       (2u)
#define CapSense_KEY03_SNS0_RAW0_PARAM_ID                   (0x87000124u)

#define CapSense_KEY03_SNS0_BSLN0_VALUE                     (CapSense_dsRam.snsList.key03[0u].bsln[0u])
#define CapSense_KEY03_SNS0_BSLN0_OFFSET                    (294u)
#define CapSense_KEY03_SNS0_BSLN0_SIZE                      (2u)
#define CapSense_KEY03_SNS0_BSLN0_PARAM_ID                  (0x8B000126u)

#define CapSense_KEY03_SNS0_BSLN_INV0_VALUE                 (CapSense_dsRam.snsList.key03[0u].bslnInv[0u])
#define CapSense_KEY03_SNS0_BSLN_INV0_OFFSET                (296u)
#define CapSense_KEY03_SNS0_BSLN_INV0_SIZE                  (2u)
#define CapSense_KEY03_SNS0_BSLN_INV0_PARAM_ID              (0x84000128u)

#define CapSense_KEY03_SNS0_BSLN_EXT0_VALUE                 (CapSense_dsRam.snsList.key03[0u].bslnExt[0u])
#define CapSense_KEY03_SNS0_BSLN_EXT0_OFFSET                (298u)
#define CapSense_KEY03_SNS0_BSLN_EXT0_SIZE                  (1u)
#define CapSense_KEY03_SNS0_BSLN_EXT0_PARAM_ID              (0x4000012Au)

#define CapSense_KEY03_SNS0_DIFF_VALUE                      (CapSense_dsRam.snsList.key03[0u].diff)
#define CapSense_KEY03_SNS0_DIFF_OFFSET                     (300u)
#define CapSense_KEY03_SNS0_DIFF_SIZE                       (2u)
#define CapSense_KEY03_SNS0_DIFF_PARAM_ID                   (0x8500012Cu)

#define CapSense_KEY03_SNS0_NEG_BSLN_RST_CNT0_VALUE         (CapSense_dsRam.snsList.key03[0u].negBslnRstCnt[0u])
#define CapSense_KEY03_SNS0_NEG_BSLN_RST_CNT0_OFFSET        (302u)
#define CapSense_KEY03_SNS0_NEG_BSLN_RST_CNT0_SIZE          (1u)
#define CapSense_KEY03_SNS0_NEG_BSLN_RST_CNT0_PARAM_ID      (0x4100012Eu)

#define CapSense_KEY03_SNS0_IDAC_COMP0_VALUE                (CapSense_dsRam.snsList.key03[0u].idacComp[0u])
#define CapSense_KEY03_SNS0_IDAC_COMP0_OFFSET               (303u)
#define CapSense_KEY03_SNS0_IDAC_COMP0_SIZE                 (1u)
#define CapSense_KEY03_SNS0_IDAC_COMP0_PARAM_ID             (0x4700012Fu)

#define CapSense_KEY04_SNS0_RAW0_VALUE                      (CapSense_dsRam.snsList.key04[0u].raw[0u])
#define CapSense_KEY04_SNS0_RAW0_OFFSET                     (304u)
#define CapSense_KEY04_SNS0_RAW0_SIZE                       (2u)
#define CapSense_KEY04_SNS0_RAW0_PARAM_ID                   (0x83000130u)

#define CapSense_KEY04_SNS0_BSLN0_VALUE                     (CapSense_dsRam.snsList.key04[0u].bsln[0u])
#define CapSense_KEY04_SNS0_BSLN0_OFFSET                    (306u)
#define CapSense_KEY04_SNS0_BSLN0_SIZE                      (2u)
#define CapSense_KEY04_SNS0_BSLN0_PARAM_ID                  (0x8F000132u)

#define CapSense_KEY04_SNS0_BSLN_INV0_VALUE                 (CapSense_dsRam.snsList.key04[0u].bslnInv[0u])
#define CapSense_KEY04_SNS0_BSLN_INV0_OFFSET                (308u)
#define CapSense_KEY04_SNS0_BSLN_INV0_SIZE                  (2u)
#define CapSense_KEY04_SNS0_BSLN_INV0_PARAM_ID              (0x82000134u)

#define CapSense_KEY04_SNS0_BSLN_EXT0_VALUE                 (CapSense_dsRam.snsList.key04[0u].bslnExt[0u])
#define CapSense_KEY04_SNS0_BSLN_EXT0_OFFSET                (310u)
#define CapSense_KEY04_SNS0_BSLN_EXT0_SIZE                  (1u)
#define CapSense_KEY04_SNS0_BSLN_EXT0_PARAM_ID              (0x46000136u)

#define CapSense_KEY04_SNS0_DIFF_VALUE                      (CapSense_dsRam.snsList.key04[0u].diff)
#define CapSense_KEY04_SNS0_DIFF_OFFSET                     (312u)
#define CapSense_KEY04_SNS0_DIFF_SIZE                       (2u)
#define CapSense_KEY04_SNS0_DIFF_PARAM_ID                   (0x81000138u)

#define CapSense_KEY04_SNS0_NEG_BSLN_RST_CNT0_VALUE         (CapSense_dsRam.snsList.key04[0u].negBslnRstCnt[0u])
#define CapSense_KEY04_SNS0_NEG_BSLN_RST_CNT0_OFFSET        (314u)
#define CapSense_KEY04_SNS0_NEG_BSLN_RST_CNT0_SIZE          (1u)
#define CapSense_KEY04_SNS0_NEG_BSLN_RST_CNT0_PARAM_ID      (0x4500013Au)

#define CapSense_KEY04_SNS0_IDAC_COMP0_VALUE                (CapSense_dsRam.snsList.key04[0u].idacComp[0u])
#define CapSense_KEY04_SNS0_IDAC_COMP0_OFFSET               (315u)
#define CapSense_KEY04_SNS0_IDAC_COMP0_SIZE                 (1u)
#define CapSense_KEY04_SNS0_IDAC_COMP0_PARAM_ID             (0x4300013Bu)

#define CapSense_KEY05_SNS0_RAW0_VALUE                      (CapSense_dsRam.snsList.key05[0u].raw[0u])
#define CapSense_KEY05_SNS0_RAW0_OFFSET                     (316u)
#define CapSense_KEY05_SNS0_RAW0_SIZE                       (2u)
#define CapSense_KEY05_SNS0_RAW0_PARAM_ID                   (0x8000013Cu)

#define CapSense_KEY05_SNS0_BSLN0_VALUE                     (CapSense_dsRam.snsList.key05[0u].bsln[0u])
#define CapSense_KEY05_SNS0_BSLN0_OFFSET                    (318u)
#define CapSense_KEY05_SNS0_BSLN0_SIZE                      (2u)
#define CapSense_KEY05_SNS0_BSLN0_PARAM_ID                  (0x8C00013Eu)

#define CapSense_KEY05_SNS0_BSLN_INV0_VALUE                 (CapSense_dsRam.snsList.key05[0u].bslnInv[0u])
#define CapSense_KEY05_SNS0_BSLN_INV0_OFFSET                (320u)
#define CapSense_KEY05_SNS0_BSLN_INV0_SIZE                  (2u)
#define CapSense_KEY05_SNS0_BSLN_INV0_PARAM_ID              (0x81000140u)

#define CapSense_KEY05_SNS0_BSLN_EXT0_VALUE                 (CapSense_dsRam.snsList.key05[0u].bslnExt[0u])
#define CapSense_KEY05_SNS0_BSLN_EXT0_OFFSET                (322u)
#define CapSense_KEY05_SNS0_BSLN_EXT0_SIZE                  (1u)
#define CapSense_KEY05_SNS0_BSLN_EXT0_PARAM_ID              (0x45000142u)

#define CapSense_KEY05_SNS0_DIFF_VALUE                      (CapSense_dsRam.snsList.key05[0u].diff)
#define CapSense_KEY05_SNS0_DIFF_OFFSET                     (324u)
#define CapSense_KEY05_SNS0_DIFF_SIZE                       (2u)
#define CapSense_KEY05_SNS0_DIFF_PARAM_ID                   (0x80000144u)

#define CapSense_KEY05_SNS0_NEG_BSLN_RST_CNT0_VALUE         (CapSense_dsRam.snsList.key05[0u].negBslnRstCnt[0u])
#define CapSense_KEY05_SNS0_NEG_BSLN_RST_CNT0_OFFSET        (326u)
#define CapSense_KEY05_SNS0_NEG_BSLN_RST_CNT0_SIZE          (1u)
#define CapSense_KEY05_SNS0_NEG_BSLN_RST_CNT0_PARAM_ID      (0x44000146u)

#define CapSense_KEY05_SNS0_IDAC_COMP0_VALUE                (CapSense_dsRam.snsList.key05[0u].idacComp[0u])
#define CapSense_KEY05_SNS0_IDAC_COMP0_OFFSET               (327u)
#define CapSense_KEY05_SNS0_IDAC_COMP0_SIZE                 (1u)
#define CapSense_KEY05_SNS0_IDAC_COMP0_PARAM_ID             (0x42000147u)

#define CapSense_KEY06_SNS0_RAW0_VALUE                      (CapSense_dsRam.snsList.key06[0u].raw[0u])
#define CapSense_KEY06_SNS0_RAW0_OFFSET                     (328u)
#define CapSense_KEY06_SNS0_RAW0_SIZE                       (2u)
#define CapSense_KEY06_SNS0_RAW0_PARAM_ID                   (0x83000148u)

#define CapSense_KEY06_SNS0_BSLN0_VALUE                     (CapSense_dsRam.snsList.key06[0u].bsln[0u])
#define CapSense_KEY06_SNS0_BSLN0_OFFSET                    (330u)
#define CapSense_KEY06_SNS0_BSLN0_SIZE                      (2u)
#define CapSense_KEY06_SNS0_BSLN0_PARAM_ID                  (0x8F00014Au)

#define CapSense_KEY06_SNS0_BSLN_INV0_VALUE                 (CapSense_dsRam.snsList.key06[0u].bslnInv[0u])
#define CapSense_KEY06_SNS0_BSLN_INV0_OFFSET                (332u)
#define CapSense_KEY06_SNS0_BSLN_INV0_SIZE                  (2u)
#define CapSense_KEY06_SNS0_BSLN_INV0_PARAM_ID              (0x8200014Cu)

#define CapSense_KEY06_SNS0_BSLN_EXT0_VALUE                 (CapSense_dsRam.snsList.key06[0u].bslnExt[0u])
#define CapSense_KEY06_SNS0_BSLN_EXT0_OFFSET                (334u)
#define CapSense_KEY06_SNS0_BSLN_EXT0_SIZE                  (1u)
#define CapSense_KEY06_SNS0_BSLN_EXT0_PARAM_ID              (0x4600014Eu)

#define CapSense_KEY06_SNS0_DIFF_VALUE                      (CapSense_dsRam.snsList.key06[0u].diff)
#define CapSense_KEY06_SNS0_DIFF_OFFSET                     (336u)
#define CapSense_KEY06_SNS0_DIFF_SIZE                       (2u)
#define CapSense_KEY06_SNS0_DIFF_PARAM_ID                   (0x84000150u)

#define CapSense_KEY06_SNS0_NEG_BSLN_RST_CNT0_VALUE         (CapSense_dsRam.snsList.key06[0u].negBslnRstCnt[0u])
#define CapSense_KEY06_SNS0_NEG_BSLN_RST_CNT0_OFFSET        (338u)
#define CapSense_KEY06_SNS0_NEG_BSLN_RST_CNT0_SIZE          (1u)
#define CapSense_KEY06_SNS0_NEG_BSLN_RST_CNT0_PARAM_ID      (0x40000152u)

#define CapSense_KEY06_SNS0_IDAC_COMP0_VALUE                (CapSense_dsRam.snsList.key06[0u].idacComp[0u])
#define CapSense_KEY06_SNS0_IDAC_COMP0_OFFSET               (339u)
#define CapSense_KEY06_SNS0_IDAC_COMP0_SIZE                 (1u)
#define CapSense_KEY06_SNS0_IDAC_COMP0_PARAM_ID             (0x46000153u)

#define CapSense_KEY07_SNS0_RAW0_VALUE                      (CapSense_dsRam.snsList.key07[0u].raw[0u])
#define CapSense_KEY07_SNS0_RAW0_OFFSET                     (340u)
#define CapSense_KEY07_SNS0_RAW0_SIZE                       (2u)
#define CapSense_KEY07_SNS0_RAW0_PARAM_ID                   (0x85000154u)

#define CapSense_KEY07_SNS0_BSLN0_VALUE                     (CapSense_dsRam.snsList.key07[0u].bsln[0u])
#define CapSense_KEY07_SNS0_BSLN0_OFFSET                    (342u)
#define CapSense_KEY07_SNS0_BSLN0_SIZE                      (2u)
#define CapSense_KEY07_SNS0_BSLN0_PARAM_ID                  (0x89000156u)

#define CapSense_KEY07_SNS0_BSLN_INV0_VALUE                 (CapSense_dsRam.snsList.key07[0u].bslnInv[0u])
#define CapSense_KEY07_SNS0_BSLN_INV0_OFFSET                (344u)
#define CapSense_KEY07_SNS0_BSLN_INV0_SIZE                  (2u)
#define CapSense_KEY07_SNS0_BSLN_INV0_PARAM_ID              (0x86000158u)

#define CapSense_KEY07_SNS0_BSLN_EXT0_VALUE                 (CapSense_dsRam.snsList.key07[0u].bslnExt[0u])
#define CapSense_KEY07_SNS0_BSLN_EXT0_OFFSET                (346u)
#define CapSense_KEY07_SNS0_BSLN_EXT0_SIZE                  (1u)
#define CapSense_KEY07_SNS0_BSLN_EXT0_PARAM_ID              (0x4200015Au)

#define CapSense_KEY07_SNS0_DIFF_VALUE                      (CapSense_dsRam.snsList.key07[0u].diff)
#define CapSense_KEY07_SNS0_DIFF_OFFSET                     (348u)
#define CapSense_KEY07_SNS0_DIFF_SIZE                       (2u)
#define CapSense_KEY07_SNS0_DIFF_PARAM_ID                   (0x8700015Cu)

#define CapSense_KEY07_SNS0_NEG_BSLN_RST_CNT0_VALUE         (CapSense_dsRam.snsList.key07[0u].negBslnRstCnt[0u])
#define CapSense_KEY07_SNS0_NEG_BSLN_RST_CNT0_OFFSET        (350u)
#define CapSense_KEY07_SNS0_NEG_BSLN_RST_CNT0_SIZE          (1u)
#define CapSense_KEY07_SNS0_NEG_BSLN_RST_CNT0_PARAM_ID      (0x4300015Eu)

#define CapSense_KEY07_SNS0_IDAC_COMP0_VALUE                (CapSense_dsRam.snsList.key07[0u].idacComp[0u])
#define CapSense_KEY07_SNS0_IDAC_COMP0_OFFSET               (351u)
#define CapSense_KEY07_SNS0_IDAC_COMP0_SIZE                 (1u)
#define CapSense_KEY07_SNS0_IDAC_COMP0_PARAM_ID             (0x4500015Fu)

#define CapSense_KEY08_SNS0_RAW0_VALUE                      (CapSense_dsRam.snsList.key08[0u].raw[0u])
#define CapSense_KEY08_SNS0_RAW0_OFFSET                     (352u)
#define CapSense_KEY08_SNS0_RAW0_SIZE                       (2u)
#define CapSense_KEY08_SNS0_RAW0_PARAM_ID                   (0x8B000160u)

#define CapSense_KEY08_SNS0_BSLN0_VALUE                     (CapSense_dsRam.snsList.key08[0u].bsln[0u])
#define CapSense_KEY08_SNS0_BSLN0_OFFSET                    (354u)
#define CapSense_KEY08_SNS0_BSLN0_SIZE                      (2u)
#define CapSense_KEY08_SNS0_BSLN0_PARAM_ID                  (0x87000162u)

#define CapSense_KEY08_SNS0_BSLN_INV0_VALUE                 (CapSense_dsRam.snsList.key08[0u].bslnInv[0u])
#define CapSense_KEY08_SNS0_BSLN_INV0_OFFSET                (356u)
#define CapSense_KEY08_SNS0_BSLN_INV0_SIZE                  (2u)
#define CapSense_KEY08_SNS0_BSLN_INV0_PARAM_ID              (0x8A000164u)

#define CapSense_KEY08_SNS0_BSLN_EXT0_VALUE                 (CapSense_dsRam.snsList.key08[0u].bslnExt[0u])
#define CapSense_KEY08_SNS0_BSLN_EXT0_OFFSET                (358u)
#define CapSense_KEY08_SNS0_BSLN_EXT0_SIZE                  (1u)
#define CapSense_KEY08_SNS0_BSLN_EXT0_PARAM_ID              (0x4E000166u)

#define CapSense_KEY08_SNS0_DIFF_VALUE                      (CapSense_dsRam.snsList.key08[0u].diff)
#define CapSense_KEY08_SNS0_DIFF_OFFSET                     (360u)
#define CapSense_KEY08_SNS0_DIFF_SIZE                       (2u)
#define CapSense_KEY08_SNS0_DIFF_PARAM_ID                   (0x89000168u)

#define CapSense_KEY08_SNS0_NEG_BSLN_RST_CNT0_VALUE         (CapSense_dsRam.snsList.key08[0u].negBslnRstCnt[0u])
#define CapSense_KEY08_SNS0_NEG_BSLN_RST_CNT0_OFFSET        (362u)
#define CapSense_KEY08_SNS0_NEG_BSLN_RST_CNT0_SIZE          (1u)
#define CapSense_KEY08_SNS0_NEG_BSLN_RST_CNT0_PARAM_ID      (0x4D00016Au)

#define CapSense_KEY08_SNS0_IDAC_COMP0_VALUE                (CapSense_dsRam.snsList.key08[0u].idacComp[0u])
#define CapSense_KEY08_SNS0_IDAC_COMP0_OFFSET               (363u)
#define CapSense_KEY08_SNS0_IDAC_COMP0_SIZE                 (1u)
#define CapSense_KEY08_SNS0_IDAC_COMP0_PARAM_ID             (0x4B00016Bu)

#define CapSense_KEY09_SNS0_RAW0_VALUE                      (CapSense_dsRam.snsList.key09[0u].raw[0u])
#define CapSense_KEY09_SNS0_RAW0_OFFSET                     (364u)
#define CapSense_KEY09_SNS0_RAW0_SIZE                       (2u)
#define CapSense_KEY09_SNS0_RAW0_PARAM_ID                   (0x8800016Cu)

#define CapSense_KEY09_SNS0_BSLN0_VALUE                     (CapSense_dsRam.snsList.key09[0u].bsln[0u])
#define CapSense_KEY09_SNS0_BSLN0_OFFSET                    (366u)
#define CapSense_KEY09_SNS0_BSLN0_SIZE                      (2u)
#define CapSense_KEY09_SNS0_BSLN0_PARAM_ID                  (0x8400016Eu)

#define CapSense_KEY09_SNS0_BSLN_INV0_VALUE                 (CapSense_dsRam.snsList.key09[0u].bslnInv[0u])
#define CapSense_KEY09_SNS0_BSLN_INV0_OFFSET                (368u)
#define CapSense_KEY09_SNS0_BSLN_INV0_SIZE                  (2u)
#define CapSense_KEY09_SNS0_BSLN_INV0_PARAM_ID              (0x8E000170u)

#define CapSense_KEY09_SNS0_BSLN_EXT0_VALUE                 (CapSense_dsRam.snsList.key09[0u].bslnExt[0u])
#define CapSense_KEY09_SNS0_BSLN_EXT0_OFFSET                (370u)
#define CapSense_KEY09_SNS0_BSLN_EXT0_SIZE                  (1u)
#define CapSense_KEY09_SNS0_BSLN_EXT0_PARAM_ID              (0x4A000172u)

#define CapSense_KEY09_SNS0_DIFF_VALUE                      (CapSense_dsRam.snsList.key09[0u].diff)
#define CapSense_KEY09_SNS0_DIFF_OFFSET                     (372u)
#define CapSense_KEY09_SNS0_DIFF_SIZE                       (2u)
#define CapSense_KEY09_SNS0_DIFF_PARAM_ID                   (0x8F000174u)

#define CapSense_KEY09_SNS0_NEG_BSLN_RST_CNT0_VALUE         (CapSense_dsRam.snsList.key09[0u].negBslnRstCnt[0u])
#define CapSense_KEY09_SNS0_NEG_BSLN_RST_CNT0_OFFSET        (374u)
#define CapSense_KEY09_SNS0_NEG_BSLN_RST_CNT0_SIZE          (1u)
#define CapSense_KEY09_SNS0_NEG_BSLN_RST_CNT0_PARAM_ID      (0x4B000176u)

#define CapSense_KEY09_SNS0_IDAC_COMP0_VALUE                (CapSense_dsRam.snsList.key09[0u].idacComp[0u])
#define CapSense_KEY09_SNS0_IDAC_COMP0_OFFSET               (375u)
#define CapSense_KEY09_SNS0_IDAC_COMP0_SIZE                 (1u)
#define CapSense_KEY09_SNS0_IDAC_COMP0_PARAM_ID             (0x4D000177u)

#define CapSense_KEY10_SNS0_RAW0_VALUE                      (CapSense_dsRam.snsList.key10[0u].raw[0u])
#define CapSense_KEY10_SNS0_RAW0_OFFSET                     (376u)
#define CapSense_KEY10_SNS0_RAW0_SIZE                       (2u)
#define CapSense_KEY10_SNS0_RAW0_PARAM_ID                   (0x8C000178u)

#define CapSense_KEY10_SNS0_BSLN0_VALUE                     (CapSense_dsRam.snsList.key10[0u].bsln[0u])
#define CapSense_KEY10_SNS0_BSLN0_OFFSET                    (378u)
#define CapSense_KEY10_SNS0_BSLN0_SIZE                      (2u)
#define CapSense_KEY10_SNS0_BSLN0_PARAM_ID                  (0x8000017Au)

#define CapSense_KEY10_SNS0_BSLN_INV0_VALUE                 (CapSense_dsRam.snsList.key10[0u].bslnInv[0u])
#define CapSense_KEY10_SNS0_BSLN_INV0_OFFSET                (380u)
#define CapSense_KEY10_SNS0_BSLN_INV0_SIZE                  (2u)
#define CapSense_KEY10_SNS0_BSLN_INV0_PARAM_ID              (0x8D00017Cu)

#define CapSense_KEY10_SNS0_BSLN_EXT0_VALUE                 (CapSense_dsRam.snsList.key10[0u].bslnExt[0u])
#define CapSense_KEY10_SNS0_BSLN_EXT0_OFFSET                (382u)
#define CapSense_KEY10_SNS0_BSLN_EXT0_SIZE                  (1u)
#define CapSense_KEY10_SNS0_BSLN_EXT0_PARAM_ID              (0x4900017Eu)

#define CapSense_KEY10_SNS0_DIFF_VALUE                      (CapSense_dsRam.snsList.key10[0u].diff)
#define CapSense_KEY10_SNS0_DIFF_OFFSET                     (384u)
#define CapSense_KEY10_SNS0_DIFF_SIZE                       (2u)
#define CapSense_KEY10_SNS0_DIFF_PARAM_ID                   (0x8F000180u)

#define CapSense_KEY10_SNS0_NEG_BSLN_RST_CNT0_VALUE         (CapSense_dsRam.snsList.key10[0u].negBslnRstCnt[0u])
#define CapSense_KEY10_SNS0_NEG_BSLN_RST_CNT0_OFFSET        (386u)
#define CapSense_KEY10_SNS0_NEG_BSLN_RST_CNT0_SIZE          (1u)
#define CapSense_KEY10_SNS0_NEG_BSLN_RST_CNT0_PARAM_ID      (0x4B000182u)

#define CapSense_KEY10_SNS0_IDAC_COMP0_VALUE                (CapSense_dsRam.snsList.key10[0u].idacComp[0u])
#define CapSense_KEY10_SNS0_IDAC_COMP0_OFFSET               (387u)
#define CapSense_KEY10_SNS0_IDAC_COMP0_SIZE                 (1u)
#define CapSense_KEY10_SNS0_IDAC_COMP0_PARAM_ID             (0x4D000183u)

#define CapSense_KEY11_SNS0_RAW0_VALUE                      (CapSense_dsRam.snsList.key11[0u].raw[0u])
#define CapSense_KEY11_SNS0_RAW0_OFFSET                     (388u)
#define CapSense_KEY11_SNS0_RAW0_SIZE                       (2u)
#define CapSense_KEY11_SNS0_RAW0_PARAM_ID                   (0x8E000184u)

#define CapSense_KEY11_SNS0_BSLN0_VALUE                     (CapSense_dsRam.snsList.key11[0u].bsln[0u])
#define CapSense_KEY11_SNS0_BSLN0_OFFSET                    (390u)
#define CapSense_KEY11_SNS0_BSLN0_SIZE                      (2u)
#define CapSense_KEY11_SNS0_BSLN0_PARAM_ID                  (0x82000186u)

#define CapSense_KEY11_SNS0_BSLN_INV0_VALUE                 (CapSense_dsRam.snsList.key11[0u].bslnInv[0u])
#define CapSense_KEY11_SNS0_BSLN_INV0_OFFSET                (392u)
#define CapSense_KEY11_SNS0_BSLN_INV0_SIZE                  (2u)
#define CapSense_KEY11_SNS0_BSLN_INV0_PARAM_ID              (0x8D000188u)

#define CapSense_KEY11_SNS0_BSLN_EXT0_VALUE                 (CapSense_dsRam.snsList.key11[0u].bslnExt[0u])
#define CapSense_KEY11_SNS0_BSLN_EXT0_OFFSET                (394u)
#define CapSense_KEY11_SNS0_BSLN_EXT0_SIZE                  (1u)
#define CapSense_KEY11_SNS0_BSLN_EXT0_PARAM_ID              (0x4900018Au)

#define CapSense_KEY11_SNS0_DIFF_VALUE                      (CapSense_dsRam.snsList.key11[0u].diff)
#define CapSense_KEY11_SNS0_DIFF_OFFSET                     (396u)
#define CapSense_KEY11_SNS0_DIFF_SIZE                       (2u)
#define CapSense_KEY11_SNS0_DIFF_PARAM_ID                   (0x8C00018Cu)

#define CapSense_KEY11_SNS0_NEG_BSLN_RST_CNT0_VALUE         (CapSense_dsRam.snsList.key11[0u].negBslnRstCnt[0u])
#define CapSense_KEY11_SNS0_NEG_BSLN_RST_CNT0_OFFSET        (398u)
#define CapSense_KEY11_SNS0_NEG_BSLN_RST_CNT0_SIZE          (1u)
#define CapSense_KEY11_SNS0_NEG_BSLN_RST_CNT0_PARAM_ID      (0x4800018Eu)

#define CapSense_KEY11_SNS0_IDAC_COMP0_VALUE                (CapSense_dsRam.snsList.key11[0u].idacComp[0u])
#define CapSense_KEY11_SNS0_IDAC_COMP0_OFFSET               (399u)
#define CapSense_KEY11_SNS0_IDAC_COMP0_SIZE                 (1u)
#define CapSense_KEY11_SNS0_IDAC_COMP0_PARAM_ID             (0x4E00018Fu)

#define CapSense_KEY12_SNS0_RAW0_VALUE                      (CapSense_dsRam.snsList.key12[0u].raw[0u])
#define CapSense_KEY12_SNS0_RAW0_OFFSET                     (400u)
#define CapSense_KEY12_SNS0_RAW0_SIZE                       (2u)
#define CapSense_KEY12_SNS0_RAW0_PARAM_ID                   (0x8A000190u)

#define CapSense_KEY12_SNS0_BSLN0_VALUE                     (CapSense_dsRam.snsList.key12[0u].bsln[0u])
#define CapSense_KEY12_SNS0_BSLN0_OFFSET                    (402u)
#define CapSense_KEY12_SNS0_BSLN0_SIZE                      (2u)
#define CapSense_KEY12_SNS0_BSLN0_PARAM_ID                  (0x86000192u)

#define CapSense_KEY12_SNS0_BSLN_INV0_VALUE                 (CapSense_dsRam.snsList.key12[0u].bslnInv[0u])
#define CapSense_KEY12_SNS0_BSLN_INV0_OFFSET                (404u)
#define CapSense_KEY12_SNS0_BSLN_INV0_SIZE                  (2u)
#define CapSense_KEY12_SNS0_BSLN_INV0_PARAM_ID              (0x8B000194u)

#define CapSense_KEY12_SNS0_BSLN_EXT0_VALUE                 (CapSense_dsRam.snsList.key12[0u].bslnExt[0u])
#define CapSense_KEY12_SNS0_BSLN_EXT0_OFFSET                (406u)
#define CapSense_KEY12_SNS0_BSLN_EXT0_SIZE                  (1u)
#define CapSense_KEY12_SNS0_BSLN_EXT0_PARAM_ID              (0x4F000196u)

#define CapSense_KEY12_SNS0_DIFF_VALUE                      (CapSense_dsRam.snsList.key12[0u].diff)
#define CapSense_KEY12_SNS0_DIFF_OFFSET                     (408u)
#define CapSense_KEY12_SNS0_DIFF_SIZE                       (2u)
#define CapSense_KEY12_SNS0_DIFF_PARAM_ID                   (0x88000198u)

#define CapSense_KEY12_SNS0_NEG_BSLN_RST_CNT0_VALUE         (CapSense_dsRam.snsList.key12[0u].negBslnRstCnt[0u])
#define CapSense_KEY12_SNS0_NEG_BSLN_RST_CNT0_OFFSET        (410u)
#define CapSense_KEY12_SNS0_NEG_BSLN_RST_CNT0_SIZE          (1u)
#define CapSense_KEY12_SNS0_NEG_BSLN_RST_CNT0_PARAM_ID      (0x4C00019Au)

#define CapSense_KEY12_SNS0_IDAC_COMP0_VALUE                (CapSense_dsRam.snsList.key12[0u].idacComp[0u])
#define CapSense_KEY12_SNS0_IDAC_COMP0_OFFSET               (411u)
#define CapSense_KEY12_SNS0_IDAC_COMP0_SIZE                 (1u)
#define CapSense_KEY12_SNS0_IDAC_COMP0_PARAM_ID             (0x4A00019Bu)

#define CapSense_TEST_RESULT_MASK_VALUE                     (CapSense_dsRam.selfTest.testResultMask)
#define CapSense_TEST_RESULT_MASK_OFFSET                    (412u)
#define CapSense_TEST_RESULT_MASK_SIZE                      (4u)
#define CapSense_TEST_RESULT_MASK_PARAM_ID                  (0xED00019Cu)

#define CapSense_EXT_CAP0_VALUE                             (CapSense_dsRam.selfTest.extCap[0u])
#define CapSense_EXT_CAP0_OFFSET                            (416u)
#define CapSense_EXT_CAP0_SIZE                              (2u)
#define CapSense_EXT_CAP0_PARAM_ID                          (0xAE0001A0u)

#define CapSense_EXT_CAP1_VALUE                             (CapSense_dsRam.selfTest.extCap[1u])
#define CapSense_EXT_CAP1_OFFSET                            (418u)
#define CapSense_EXT_CAP1_SIZE                              (2u)
#define CapSense_EXT_CAP1_PARAM_ID                          (0xA20001A2u)

#define CapSense_VDDA_VOLTAGE_VALUE                         (CapSense_dsRam.selfTest.vddaVoltage)
#define CapSense_VDDA_VOLTAGE_OFFSET                        (420u)
#define CapSense_VDDA_VOLTAGE_SIZE                          (2u)
#define CapSense_VDDA_VOLTAGE_PARAM_ID                      (0xAF0001A4u)

#define CapSense_SHIELD_CAP_VALUE                           (CapSense_dsRam.selfTest.shieldCap)
#define CapSense_SHIELD_CAP_OFFSET                          (422u)
#define CapSense_SHIELD_CAP_SIZE                            (2u)
#define CapSense_SHIELD_CAP_PARAM_ID                        (0xA30001A6u)

#define CapSense_GLB_CRC_CALC_VALUE                         (CapSense_dsRam.selfTest.glbCrcCalc)
#define CapSense_GLB_CRC_CALC_OFFSET                        (424u)
#define CapSense_GLB_CRC_CALC_SIZE                          (2u)
#define CapSense_GLB_CRC_CALC_PARAM_ID                      (0xAC0001A8u)

#define CapSense_WDGT_CRC_CALC_VALUE                        (CapSense_dsRam.selfTest.wdgtCrcCalc)
#define CapSense_WDGT_CRC_CALC_OFFSET                       (426u)
#define CapSense_WDGT_CRC_CALC_SIZE                         (2u)
#define CapSense_WDGT_CRC_CALC_PARAM_ID                     (0xA00001AAu)

#define CapSense_WDGT_CRC_ID_VALUE                          (CapSense_dsRam.selfTest.wdgtCrcId)
#define CapSense_WDGT_CRC_ID_OFFSET                         (428u)
#define CapSense_WDGT_CRC_ID_SIZE                           (1u)
#define CapSense_WDGT_CRC_ID_PARAM_ID                       (0x650001ACu)

#define CapSense_INV_BSLN_WDGT_ID_VALUE                     (CapSense_dsRam.selfTest.invBslnWdgtId)
#define CapSense_INV_BSLN_WDGT_ID_OFFSET                    (429u)
#define CapSense_INV_BSLN_WDGT_ID_SIZE                      (1u)
#define CapSense_INV_BSLN_WDGT_ID_PARAM_ID                  (0x630001ADu)

#define CapSense_INV_BSLN_SNS_ID_VALUE                      (CapSense_dsRam.selfTest.invBslnSnsId)
#define CapSense_INV_BSLN_SNS_ID_OFFSET                     (430u)
#define CapSense_INV_BSLN_SNS_ID_SIZE                       (1u)
#define CapSense_INV_BSLN_SNS_ID_PARAM_ID                   (0x690001AEu)

#define CapSense_SHORTED_WDGT_ID_VALUE                      (CapSense_dsRam.selfTest.shortedWdgtId)
#define CapSense_SHORTED_WDGT_ID_OFFSET                     (431u)
#define CapSense_SHORTED_WDGT_ID_SIZE                       (1u)
#define CapSense_SHORTED_WDGT_ID_PARAM_ID                   (0x6F0001AFu)

#define CapSense_SHORTED_SNS_ID_VALUE                       (CapSense_dsRam.selfTest.shortedSnsId)
#define CapSense_SHORTED_SNS_ID_OFFSET                      (432u)
#define CapSense_SHORTED_SNS_ID_SIZE                        (1u)
#define CapSense_SHORTED_SNS_ID_PARAM_ID                    (0x630001B0u)

#define CapSense_P2P_WDGT_ID_VALUE                          (CapSense_dsRam.selfTest.p2pWdgtId)
#define CapSense_P2P_WDGT_ID_OFFSET                         (433u)
#define CapSense_P2P_WDGT_ID_SIZE                           (1u)
#define CapSense_P2P_WDGT_ID_PARAM_ID                       (0x650001B1u)

#define CapSense_P2P_SNS_ID_VALUE                           (CapSense_dsRam.selfTest.p2pSnsId)
#define CapSense_P2P_SNS_ID_OFFSET                          (434u)
#define CapSense_P2P_SNS_ID_SIZE                            (1u)
#define CapSense_P2P_SNS_ID_PARAM_ID                        (0x6F0001B2u)

#define CapSense_KEY00_SNS_CP0_VALUE                        (CapSense_dsRam.snsCp.key00[0u])
#define CapSense_KEY00_SNS_CP0_OFFSET                       (436u)
#define CapSense_KEY00_SNS_CP0_SIZE                         (1u)
#define CapSense_KEY00_SNS_CP0_PARAM_ID                     (0x620001B4u)

#define CapSense_KEY01_SNS_CP0_VALUE                        (CapSense_dsRam.snsCp.key01[0u])
#define CapSense_KEY01_SNS_CP0_OFFSET                       (437u)
#define CapSense_KEY01_SNS_CP0_SIZE                         (1u)
#define CapSense_KEY01_SNS_CP0_PARAM_ID                     (0x640001B5u)

#define CapSense_KEY02_SNS_CP0_VALUE                        (CapSense_dsRam.snsCp.key02[0u])
#define CapSense_KEY02_SNS_CP0_OFFSET                       (438u)
#define CapSense_KEY02_SNS_CP0_SIZE                         (1u)
#define CapSense_KEY02_SNS_CP0_PARAM_ID                     (0x6E0001B6u)

#define CapSense_KEY03_SNS_CP0_VALUE                        (CapSense_dsRam.snsCp.key03[0u])
#define CapSense_KEY03_SNS_CP0_OFFSET                       (439u)
#define CapSense_KEY03_SNS_CP0_SIZE                         (1u)
#define CapSense_KEY03_SNS_CP0_PARAM_ID                     (0x680001B7u)

#define CapSense_KEY04_SNS_CP0_VALUE                        (CapSense_dsRam.snsCp.key04[0u])
#define CapSense_KEY04_SNS_CP0_OFFSET                       (440u)
#define CapSense_KEY04_SNS_CP0_SIZE                         (1u)
#define CapSense_KEY04_SNS_CP0_PARAM_ID                     (0x610001B8u)

#define CapSense_KEY05_SNS_CP0_VALUE                        (CapSense_dsRam.snsCp.key05[0u])
#define CapSense_KEY05_SNS_CP0_OFFSET                       (441u)
#define CapSense_KEY05_SNS_CP0_SIZE                         (1u)
#define CapSense_KEY05_SNS_CP0_PARAM_ID                     (0x670001B9u)

#define CapSense_KEY06_SNS_CP0_VALUE                        (CapSense_dsRam.snsCp.key06[0u])
#define CapSense_KEY06_SNS_CP0_OFFSET                       (442u)
#define CapSense_KEY06_SNS_CP0_SIZE                         (1u)
#define CapSense_KEY06_SNS_CP0_PARAM_ID                     (0x6D0001BAu)

#define CapSense_KEY07_SNS_CP0_VALUE                        (CapSense_dsRam.snsCp.key07[0u])
#define CapSense_KEY07_SNS_CP0_OFFSET                       (443u)
#define CapSense_KEY07_SNS_CP0_SIZE                         (1u)
#define CapSense_KEY07_SNS_CP0_PARAM_ID                     (0x6B0001BBu)

#define CapSense_KEY08_SNS_CP0_VALUE                        (CapSense_dsRam.snsCp.key08[0u])
#define CapSense_KEY08_SNS_CP0_OFFSET                       (444u)
#define CapSense_KEY08_SNS_CP0_SIZE                         (1u)
#define CapSense_KEY08_SNS_CP0_PARAM_ID                     (0x600001BCu)

#define CapSense_KEY09_SNS_CP0_VALUE                        (CapSense_dsRam.snsCp.key09[0u])
#define CapSense_KEY09_SNS_CP0_OFFSET                       (445u)
#define CapSense_KEY09_SNS_CP0_SIZE                         (1u)
#define CapSense_KEY09_SNS_CP0_PARAM_ID                     (0x660001BDu)

#define CapSense_KEY10_SNS_CP0_VALUE                        (CapSense_dsRam.snsCp.key10[0u])
#define CapSense_KEY10_SNS_CP0_OFFSET                       (446u)
#define CapSense_KEY10_SNS_CP0_SIZE                         (1u)
#define CapSense_KEY10_SNS_CP0_PARAM_ID                     (0x6C0001BEu)

#define CapSense_KEY11_SNS_CP0_VALUE                        (CapSense_dsRam.snsCp.key11[0u])
#define CapSense_KEY11_SNS_CP0_OFFSET                       (447u)
#define CapSense_KEY11_SNS_CP0_SIZE                         (1u)
#define CapSense_KEY11_SNS_CP0_PARAM_ID                     (0x6A0001BFu)

#define CapSense_KEY12_SNS_CP0_VALUE                        (CapSense_dsRam.snsCp.key12[0u])
#define CapSense_KEY12_SNS_CP0_OFFSET                       (448u)
#define CapSense_KEY12_SNS_CP0_SIZE                         (1u)
#define CapSense_KEY12_SNS_CP0_PARAM_ID                     (0x610001C0u)

#define CapSense_SNR_TEST_WIDGET_ID_VALUE                   (CapSense_dsRam.snrTestWidgetId)
#define CapSense_SNR_TEST_WIDGET_ID_OFFSET                  (449u)
#define CapSense_SNR_TEST_WIDGET_ID_SIZE                    (1u)
#define CapSense_SNR_TEST_WIDGET_ID_PARAM_ID                (0x670001C1u)

#define CapSense_SNR_TEST_SENSOR_ID_VALUE                   (CapSense_dsRam.snrTestSensorId)
#define CapSense_SNR_TEST_SENSOR_ID_OFFSET                  (450u)
#define CapSense_SNR_TEST_SENSOR_ID_SIZE                    (1u)
#define CapSense_SNR_TEST_SENSOR_ID_PARAM_ID                (0x6D0001C2u)

#define CapSense_SNR_TEST_SCAN_COUNTER_VALUE                (CapSense_dsRam.snrTestScanCounter)
#define CapSense_SNR_TEST_SCAN_COUNTER_OFFSET               (452u)
#define CapSense_SNR_TEST_SCAN_COUNTER_SIZE                 (2u)
#define CapSense_SNR_TEST_SCAN_COUNTER_PARAM_ID             (0x830001C4u)

#define CapSense_SNR_TEST_RAW_COUNT0_VALUE                  (CapSense_dsRam.snrTestRawCount[0u])
#define CapSense_SNR_TEST_RAW_COUNT0_OFFSET                 (454u)
#define CapSense_SNR_TEST_RAW_COUNT0_SIZE                   (2u)
#define CapSense_SNR_TEST_RAW_COUNT0_PARAM_ID               (0x8F0001C6u)


/*****************************************************************************/
/* Flash Data structure register definitions                                 */
/*****************************************************************************/
#define CapSense_KEY00_PTR2SNS_FLASH_VALUE                  (CapSense_dsFlash.wdgtArray[0].ptr2SnsFlash)
#define CapSense_KEY00_PTR2SNS_FLASH_OFFSET                 (0u)
#define CapSense_KEY00_PTR2SNS_FLASH_SIZE                   (4u)
#define CapSense_KEY00_PTR2SNS_FLASH_PARAM_ID               (0xD1000000u)

#define CapSense_KEY00_PTR2WD_RAM_VALUE                     (CapSense_dsFlash.wdgtArray[0].ptr2WdgtRam)
#define CapSense_KEY00_PTR2WD_RAM_OFFSET                    (4u)
#define CapSense_KEY00_PTR2WD_RAM_SIZE                      (4u)
#define CapSense_KEY00_PTR2WD_RAM_PARAM_ID                  (0xD0000004u)

#define CapSense_KEY00_PTR2SNS_RAM_VALUE                    (CapSense_dsFlash.wdgtArray[0].ptr2SnsRam)
#define CapSense_KEY00_PTR2SNS_RAM_OFFSET                   (8u)
#define CapSense_KEY00_PTR2SNS_RAM_SIZE                     (4u)
#define CapSense_KEY00_PTR2SNS_RAM_PARAM_ID                 (0xD3000008u)

#define CapSense_KEY00_PTR2FLTR_HISTORY_VALUE               (CapSense_dsFlash.wdgtArray[0].ptr2FltrHistory)
#define CapSense_KEY00_PTR2FLTR_HISTORY_OFFSET              (12u)
#define CapSense_KEY00_PTR2FLTR_HISTORY_SIZE                (4u)
#define CapSense_KEY00_PTR2FLTR_HISTORY_PARAM_ID            (0xD200000Cu)

#define CapSense_KEY00_PTR2DEBOUNCE_VALUE                   (CapSense_dsFlash.wdgtArray[0].ptr2DebounceArr)
#define CapSense_KEY00_PTR2DEBOUNCE_OFFSET                  (16u)
#define CapSense_KEY00_PTR2DEBOUNCE_SIZE                    (4u)
#define CapSense_KEY00_PTR2DEBOUNCE_PARAM_ID                (0xD4000010u)

#define CapSense_KEY00_STATIC_CONFIG_VALUE                  (CapSense_dsFlash.wdgtArray[0].staticConfig)
#define CapSense_KEY00_STATIC_CONFIG_OFFSET                 (20u)
#define CapSense_KEY00_STATIC_CONFIG_SIZE                   (4u)
#define CapSense_KEY00_STATIC_CONFIG_PARAM_ID               (0xD5000014u)

#define CapSense_KEY00_TOTAL_NUM_SNS_VALUE                  (CapSense_dsFlash.wdgtArray[0].totalNumSns)
#define CapSense_KEY00_TOTAL_NUM_SNS_OFFSET                 (24u)
#define CapSense_KEY00_TOTAL_NUM_SNS_SIZE                   (2u)
#define CapSense_KEY00_TOTAL_NUM_SNS_PARAM_ID               (0x99000018u)

#define CapSense_KEY00_TYPE_VALUE                           (CapSense_dsFlash.wdgtArray[0].wdgtType)
#define CapSense_KEY00_TYPE_OFFSET                          (26u)
#define CapSense_KEY00_TYPE_SIZE                            (1u)
#define CapSense_KEY00_TYPE_PARAM_ID                        (0x5D00001Au)

#define CapSense_KEY00_NUM_COLS_VALUE                       (CapSense_dsFlash.wdgtArray[0].numCols)
#define CapSense_KEY00_NUM_COLS_OFFSET                      (27u)
#define CapSense_KEY00_NUM_COLS_SIZE                        (1u)
#define CapSense_KEY00_NUM_COLS_PARAM_ID                    (0x5B00001Bu)

#define CapSense_KEY00_PTR2SNS_CP_VALUE                     (CapSense_dsFlash.wdgtArray[0].ptr2SnsCpArr)
#define CapSense_KEY00_PTR2SNS_CP_OFFSET                    (28u)
#define CapSense_KEY00_PTR2SNS_CP_SIZE                      (4u)
#define CapSense_KEY00_PTR2SNS_CP_PARAM_ID                  (0xD700001Cu)

#define CapSense_KEY01_PTR2SNS_FLASH_VALUE                  (CapSense_dsFlash.wdgtArray[1].ptr2SnsFlash)
#define CapSense_KEY01_PTR2SNS_FLASH_OFFSET                 (32u)
#define CapSense_KEY01_PTR2SNS_FLASH_SIZE                   (4u)
#define CapSense_KEY01_PTR2SNS_FLASH_PARAM_ID               (0xD8010020u)

#define CapSense_KEY01_PTR2WD_RAM_VALUE                     (CapSense_dsFlash.wdgtArray[1].ptr2WdgtRam)
#define CapSense_KEY01_PTR2WD_RAM_OFFSET                    (36u)
#define CapSense_KEY01_PTR2WD_RAM_SIZE                      (4u)
#define CapSense_KEY01_PTR2WD_RAM_PARAM_ID                  (0xD9010024u)

#define CapSense_KEY01_PTR2SNS_RAM_VALUE                    (CapSense_dsFlash.wdgtArray[1].ptr2SnsRam)
#define CapSense_KEY01_PTR2SNS_RAM_OFFSET                   (40u)
#define CapSense_KEY01_PTR2SNS_RAM_SIZE                     (4u)
#define CapSense_KEY01_PTR2SNS_RAM_PARAM_ID                 (0xDA010028u)

#define CapSense_KEY01_PTR2FLTR_HISTORY_VALUE               (CapSense_dsFlash.wdgtArray[1].ptr2FltrHistory)
#define CapSense_KEY01_PTR2FLTR_HISTORY_OFFSET              (44u)
#define CapSense_KEY01_PTR2FLTR_HISTORY_SIZE                (4u)
#define CapSense_KEY01_PTR2FLTR_HISTORY_PARAM_ID            (0xDB01002Cu)

#define CapSense_KEY01_PTR2DEBOUNCE_VALUE                   (CapSense_dsFlash.wdgtArray[1].ptr2DebounceArr)
#define CapSense_KEY01_PTR2DEBOUNCE_OFFSET                  (48u)
#define CapSense_KEY01_PTR2DEBOUNCE_SIZE                    (4u)
#define CapSense_KEY01_PTR2DEBOUNCE_PARAM_ID                (0xDD010030u)

#define CapSense_KEY01_STATIC_CONFIG_VALUE                  (CapSense_dsFlash.wdgtArray[1].staticConfig)
#define CapSense_KEY01_STATIC_CONFIG_OFFSET                 (52u)
#define CapSense_KEY01_STATIC_CONFIG_SIZE                   (4u)
#define CapSense_KEY01_STATIC_CONFIG_PARAM_ID               (0xDC010034u)

#define CapSense_KEY01_TOTAL_NUM_SNS_VALUE                  (CapSense_dsFlash.wdgtArray[1].totalNumSns)
#define CapSense_KEY01_TOTAL_NUM_SNS_OFFSET                 (56u)
#define CapSense_KEY01_TOTAL_NUM_SNS_SIZE                   (2u)
#define CapSense_KEY01_TOTAL_NUM_SNS_PARAM_ID               (0x90010038u)

#define CapSense_KEY01_TYPE_VALUE                           (CapSense_dsFlash.wdgtArray[1].wdgtType)
#define CapSense_KEY01_TYPE_OFFSET                          (58u)
#define CapSense_KEY01_TYPE_SIZE                            (1u)
#define CapSense_KEY01_TYPE_PARAM_ID                        (0x5401003Au)

#define CapSense_KEY01_NUM_COLS_VALUE                       (CapSense_dsFlash.wdgtArray[1].numCols)
#define CapSense_KEY01_NUM_COLS_OFFSET                      (59u)
#define CapSense_KEY01_NUM_COLS_SIZE                        (1u)
#define CapSense_KEY01_NUM_COLS_PARAM_ID                    (0x5201003Bu)

#define CapSense_KEY01_PTR2SNS_CP_VALUE                     (CapSense_dsFlash.wdgtArray[1].ptr2SnsCpArr)
#define CapSense_KEY01_PTR2SNS_CP_OFFSET                    (60u)
#define CapSense_KEY01_PTR2SNS_CP_SIZE                      (4u)
#define CapSense_KEY01_PTR2SNS_CP_PARAM_ID                  (0xDE01003Cu)

#define CapSense_KEY02_PTR2SNS_FLASH_VALUE                  (CapSense_dsFlash.wdgtArray[2].ptr2SnsFlash)
#define CapSense_KEY02_PTR2SNS_FLASH_OFFSET                 (64u)
#define CapSense_KEY02_PTR2SNS_FLASH_SIZE                   (4u)
#define CapSense_KEY02_PTR2SNS_FLASH_PARAM_ID               (0xDA020040u)

#define CapSense_KEY02_PTR2WD_RAM_VALUE                     (CapSense_dsFlash.wdgtArray[2].ptr2WdgtRam)
#define CapSense_KEY02_PTR2WD_RAM_OFFSET                    (68u)
#define CapSense_KEY02_PTR2WD_RAM_SIZE                      (4u)
#define CapSense_KEY02_PTR2WD_RAM_PARAM_ID                  (0xDB020044u)

#define CapSense_KEY02_PTR2SNS_RAM_VALUE                    (CapSense_dsFlash.wdgtArray[2].ptr2SnsRam)
#define CapSense_KEY02_PTR2SNS_RAM_OFFSET                   (72u)
#define CapSense_KEY02_PTR2SNS_RAM_SIZE                     (4u)
#define CapSense_KEY02_PTR2SNS_RAM_PARAM_ID                 (0xD8020048u)

#define CapSense_KEY02_PTR2FLTR_HISTORY_VALUE               (CapSense_dsFlash.wdgtArray[2].ptr2FltrHistory)
#define CapSense_KEY02_PTR2FLTR_HISTORY_OFFSET              (76u)
#define CapSense_KEY02_PTR2FLTR_HISTORY_SIZE                (4u)
#define CapSense_KEY02_PTR2FLTR_HISTORY_PARAM_ID            (0xD902004Cu)

#define CapSense_KEY02_PTR2DEBOUNCE_VALUE                   (CapSense_dsFlash.wdgtArray[2].ptr2DebounceArr)
#define CapSense_KEY02_PTR2DEBOUNCE_OFFSET                  (80u)
#define CapSense_KEY02_PTR2DEBOUNCE_SIZE                    (4u)
#define CapSense_KEY02_PTR2DEBOUNCE_PARAM_ID                (0xDF020050u)

#define CapSense_KEY02_STATIC_CONFIG_VALUE                  (CapSense_dsFlash.wdgtArray[2].staticConfig)
#define CapSense_KEY02_STATIC_CONFIG_OFFSET                 (84u)
#define CapSense_KEY02_STATIC_CONFIG_SIZE                   (4u)
#define CapSense_KEY02_STATIC_CONFIG_PARAM_ID               (0xDE020054u)

#define CapSense_KEY02_TOTAL_NUM_SNS_VALUE                  (CapSense_dsFlash.wdgtArray[2].totalNumSns)
#define CapSense_KEY02_TOTAL_NUM_SNS_OFFSET                 (88u)
#define CapSense_KEY02_TOTAL_NUM_SNS_SIZE                   (2u)
#define CapSense_KEY02_TOTAL_NUM_SNS_PARAM_ID               (0x92020058u)

#define CapSense_KEY02_TYPE_VALUE                           (CapSense_dsFlash.wdgtArray[2].wdgtType)
#define CapSense_KEY02_TYPE_OFFSET                          (90u)
#define CapSense_KEY02_TYPE_SIZE                            (1u)
#define CapSense_KEY02_TYPE_PARAM_ID                        (0x5602005Au)

#define CapSense_KEY02_NUM_COLS_VALUE                       (CapSense_dsFlash.wdgtArray[2].numCols)
#define CapSense_KEY02_NUM_COLS_OFFSET                      (91u)
#define CapSense_KEY02_NUM_COLS_SIZE                        (1u)
#define CapSense_KEY02_NUM_COLS_PARAM_ID                    (0x5002005Bu)

#define CapSense_KEY02_PTR2SNS_CP_VALUE                     (CapSense_dsFlash.wdgtArray[2].ptr2SnsCpArr)
#define CapSense_KEY02_PTR2SNS_CP_OFFSET                    (92u)
#define CapSense_KEY02_PTR2SNS_CP_SIZE                      (4u)
#define CapSense_KEY02_PTR2SNS_CP_PARAM_ID                  (0xDC02005Cu)

#define CapSense_KEY03_PTR2SNS_FLASH_VALUE                  (CapSense_dsFlash.wdgtArray[3].ptr2SnsFlash)
#define CapSense_KEY03_PTR2SNS_FLASH_OFFSET                 (96u)
#define CapSense_KEY03_PTR2SNS_FLASH_SIZE                   (4u)
#define CapSense_KEY03_PTR2SNS_FLASH_PARAM_ID               (0xD3030060u)

#define CapSense_KEY03_PTR2WD_RAM_VALUE                     (CapSense_dsFlash.wdgtArray[3].ptr2WdgtRam)
#define CapSense_KEY03_PTR2WD_RAM_OFFSET                    (100u)
#define CapSense_KEY03_PTR2WD_RAM_SIZE                      (4u)
#define CapSense_KEY03_PTR2WD_RAM_PARAM_ID                  (0xD2030064u)

#define CapSense_KEY03_PTR2SNS_RAM_VALUE                    (CapSense_dsFlash.wdgtArray[3].ptr2SnsRam)
#define CapSense_KEY03_PTR2SNS_RAM_OFFSET                   (104u)
#define CapSense_KEY03_PTR2SNS_RAM_SIZE                     (4u)
#define CapSense_KEY03_PTR2SNS_RAM_PARAM_ID                 (0xD1030068u)

#define CapSense_KEY03_PTR2FLTR_HISTORY_VALUE               (CapSense_dsFlash.wdgtArray[3].ptr2FltrHistory)
#define CapSense_KEY03_PTR2FLTR_HISTORY_OFFSET              (108u)
#define CapSense_KEY03_PTR2FLTR_HISTORY_SIZE                (4u)
#define CapSense_KEY03_PTR2FLTR_HISTORY_PARAM_ID            (0xD003006Cu)

#define CapSense_KEY03_PTR2DEBOUNCE_VALUE                   (CapSense_dsFlash.wdgtArray[3].ptr2DebounceArr)
#define CapSense_KEY03_PTR2DEBOUNCE_OFFSET                  (112u)
#define CapSense_KEY03_PTR2DEBOUNCE_SIZE                    (4u)
#define CapSense_KEY03_PTR2DEBOUNCE_PARAM_ID                (0xD6030070u)

#define CapSense_KEY03_STATIC_CONFIG_VALUE                  (CapSense_dsFlash.wdgtArray[3].staticConfig)
#define CapSense_KEY03_STATIC_CONFIG_OFFSET                 (116u)
#define CapSense_KEY03_STATIC_CONFIG_SIZE                   (4u)
#define CapSense_KEY03_STATIC_CONFIG_PARAM_ID               (0xD7030074u)

#define CapSense_KEY03_TOTAL_NUM_SNS_VALUE                  (CapSense_dsFlash.wdgtArray[3].totalNumSns)
#define CapSense_KEY03_TOTAL_NUM_SNS_OFFSET                 (120u)
#define CapSense_KEY03_TOTAL_NUM_SNS_SIZE                   (2u)
#define CapSense_KEY03_TOTAL_NUM_SNS_PARAM_ID               (0x9B030078u)

#define CapSense_KEY03_TYPE_VALUE                           (CapSense_dsFlash.wdgtArray[3].wdgtType)
#define CapSense_KEY03_TYPE_OFFSET                          (122u)
#define CapSense_KEY03_TYPE_SIZE                            (1u)
#define CapSense_KEY03_TYPE_PARAM_ID                        (0x5F03007Au)

#define CapSense_KEY03_NUM_COLS_VALUE                       (CapSense_dsFlash.wdgtArray[3].numCols)
#define CapSense_KEY03_NUM_COLS_OFFSET                      (123u)
#define CapSense_KEY03_NUM_COLS_SIZE                        (1u)
#define CapSense_KEY03_NUM_COLS_PARAM_ID                    (0x5903007Bu)

#define CapSense_KEY03_PTR2SNS_CP_VALUE                     (CapSense_dsFlash.wdgtArray[3].ptr2SnsCpArr)
#define CapSense_KEY03_PTR2SNS_CP_OFFSET                    (124u)
#define CapSense_KEY03_PTR2SNS_CP_SIZE                      (4u)
#define CapSense_KEY03_PTR2SNS_CP_PARAM_ID                  (0xD503007Cu)

#define CapSense_KEY04_PTR2SNS_FLASH_VALUE                  (CapSense_dsFlash.wdgtArray[4].ptr2SnsFlash)
#define CapSense_KEY04_PTR2SNS_FLASH_OFFSET                 (128u)
#define CapSense_KEY04_PTR2SNS_FLASH_SIZE                   (4u)
#define CapSense_KEY04_PTR2SNS_FLASH_PARAM_ID               (0xDE040080u)

#define CapSense_KEY04_PTR2WD_RAM_VALUE                     (CapSense_dsFlash.wdgtArray[4].ptr2WdgtRam)
#define CapSense_KEY04_PTR2WD_RAM_OFFSET                    (132u)
#define CapSense_KEY04_PTR2WD_RAM_SIZE                      (4u)
#define CapSense_KEY04_PTR2WD_RAM_PARAM_ID                  (0xDF040084u)

#define CapSense_KEY04_PTR2SNS_RAM_VALUE                    (CapSense_dsFlash.wdgtArray[4].ptr2SnsRam)
#define CapSense_KEY04_PTR2SNS_RAM_OFFSET                   (136u)
#define CapSense_KEY04_PTR2SNS_RAM_SIZE                     (4u)
#define CapSense_KEY04_PTR2SNS_RAM_PARAM_ID                 (0xDC040088u)

#define CapSense_KEY04_PTR2FLTR_HISTORY_VALUE               (CapSense_dsFlash.wdgtArray[4].ptr2FltrHistory)
#define CapSense_KEY04_PTR2FLTR_HISTORY_OFFSET              (140u)
#define CapSense_KEY04_PTR2FLTR_HISTORY_SIZE                (4u)
#define CapSense_KEY04_PTR2FLTR_HISTORY_PARAM_ID            (0xDD04008Cu)

#define CapSense_KEY04_PTR2DEBOUNCE_VALUE                   (CapSense_dsFlash.wdgtArray[4].ptr2DebounceArr)
#define CapSense_KEY04_PTR2DEBOUNCE_OFFSET                  (144u)
#define CapSense_KEY04_PTR2DEBOUNCE_SIZE                    (4u)
#define CapSense_KEY04_PTR2DEBOUNCE_PARAM_ID                (0xDB040090u)

#define CapSense_KEY04_STATIC_CONFIG_VALUE                  (CapSense_dsFlash.wdgtArray[4].staticConfig)
#define CapSense_KEY04_STATIC_CONFIG_OFFSET                 (148u)
#define CapSense_KEY04_STATIC_CONFIG_SIZE                   (4u)
#define CapSense_KEY04_STATIC_CONFIG_PARAM_ID               (0xDA040094u)

#define CapSense_KEY04_TOTAL_NUM_SNS_VALUE                  (CapSense_dsFlash.wdgtArray[4].totalNumSns)
#define CapSense_KEY04_TOTAL_NUM_SNS_OFFSET                 (152u)
#define CapSense_KEY04_TOTAL_NUM_SNS_SIZE                   (2u)
#define CapSense_KEY04_TOTAL_NUM_SNS_PARAM_ID               (0x96040098u)

#define CapSense_KEY04_TYPE_VALUE                           (CapSense_dsFlash.wdgtArray[4].wdgtType)
#define CapSense_KEY04_TYPE_OFFSET                          (154u)
#define CapSense_KEY04_TYPE_SIZE                            (1u)
#define CapSense_KEY04_TYPE_PARAM_ID                        (0x5204009Au)

#define CapSense_KEY04_NUM_COLS_VALUE                       (CapSense_dsFlash.wdgtArray[4].numCols)
#define CapSense_KEY04_NUM_COLS_OFFSET                      (155u)
#define CapSense_KEY04_NUM_COLS_SIZE                        (1u)
#define CapSense_KEY04_NUM_COLS_PARAM_ID                    (0x5404009Bu)

#define CapSense_KEY04_PTR2SNS_CP_VALUE                     (CapSense_dsFlash.wdgtArray[4].ptr2SnsCpArr)
#define CapSense_KEY04_PTR2SNS_CP_OFFSET                    (156u)
#define CapSense_KEY04_PTR2SNS_CP_SIZE                      (4u)
#define CapSense_KEY04_PTR2SNS_CP_PARAM_ID                  (0xD804009Cu)

#define CapSense_KEY05_PTR2SNS_FLASH_VALUE                  (CapSense_dsFlash.wdgtArray[5].ptr2SnsFlash)
#define CapSense_KEY05_PTR2SNS_FLASH_OFFSET                 (160u)
#define CapSense_KEY05_PTR2SNS_FLASH_SIZE                   (4u)
#define CapSense_KEY05_PTR2SNS_FLASH_PARAM_ID               (0xD70500A0u)

#define CapSense_KEY05_PTR2WD_RAM_VALUE                     (CapSense_dsFlash.wdgtArray[5].ptr2WdgtRam)
#define CapSense_KEY05_PTR2WD_RAM_OFFSET                    (164u)
#define CapSense_KEY05_PTR2WD_RAM_SIZE                      (4u)
#define CapSense_KEY05_PTR2WD_RAM_PARAM_ID                  (0xD60500A4u)

#define CapSense_KEY05_PTR2SNS_RAM_VALUE                    (CapSense_dsFlash.wdgtArray[5].ptr2SnsRam)
#define CapSense_KEY05_PTR2SNS_RAM_OFFSET                   (168u)
#define CapSense_KEY05_PTR2SNS_RAM_SIZE                     (4u)
#define CapSense_KEY05_PTR2SNS_RAM_PARAM_ID                 (0xD50500A8u)

#define CapSense_KEY05_PTR2FLTR_HISTORY_VALUE               (CapSense_dsFlash.wdgtArray[5].ptr2FltrHistory)
#define CapSense_KEY05_PTR2FLTR_HISTORY_OFFSET              (172u)
#define CapSense_KEY05_PTR2FLTR_HISTORY_SIZE                (4u)
#define CapSense_KEY05_PTR2FLTR_HISTORY_PARAM_ID            (0xD40500ACu)

#define CapSense_KEY05_PTR2DEBOUNCE_VALUE                   (CapSense_dsFlash.wdgtArray[5].ptr2DebounceArr)
#define CapSense_KEY05_PTR2DEBOUNCE_OFFSET                  (176u)
#define CapSense_KEY05_PTR2DEBOUNCE_SIZE                    (4u)
#define CapSense_KEY05_PTR2DEBOUNCE_PARAM_ID                (0xD20500B0u)

#define CapSense_KEY05_STATIC_CONFIG_VALUE                  (CapSense_dsFlash.wdgtArray[5].staticConfig)
#define CapSense_KEY05_STATIC_CONFIG_OFFSET                 (180u)
#define CapSense_KEY05_STATIC_CONFIG_SIZE                   (4u)
#define CapSense_KEY05_STATIC_CONFIG_PARAM_ID               (0xD30500B4u)

#define CapSense_KEY05_TOTAL_NUM_SNS_VALUE                  (CapSense_dsFlash.wdgtArray[5].totalNumSns)
#define CapSense_KEY05_TOTAL_NUM_SNS_OFFSET                 (184u)
#define CapSense_KEY05_TOTAL_NUM_SNS_SIZE                   (2u)
#define CapSense_KEY05_TOTAL_NUM_SNS_PARAM_ID               (0x9F0500B8u)

#define CapSense_KEY05_TYPE_VALUE                           (CapSense_dsFlash.wdgtArray[5].wdgtType)
#define CapSense_KEY05_TYPE_OFFSET                          (186u)
#define CapSense_KEY05_TYPE_SIZE                            (1u)
#define CapSense_KEY05_TYPE_PARAM_ID                        (0x5B0500BAu)

#define CapSense_KEY05_NUM_COLS_VALUE                       (CapSense_dsFlash.wdgtArray[5].numCols)
#define CapSense_KEY05_NUM_COLS_OFFSET                      (187u)
#define CapSense_KEY05_NUM_COLS_SIZE                        (1u)
#define CapSense_KEY05_NUM_COLS_PARAM_ID                    (0x5D0500BBu)

#define CapSense_KEY05_PTR2SNS_CP_VALUE                     (CapSense_dsFlash.wdgtArray[5].ptr2SnsCpArr)
#define CapSense_KEY05_PTR2SNS_CP_OFFSET                    (188u)
#define CapSense_KEY05_PTR2SNS_CP_SIZE                      (4u)
#define CapSense_KEY05_PTR2SNS_CP_PARAM_ID                  (0xD10500BCu)

#define CapSense_KEY06_PTR2SNS_FLASH_VALUE                  (CapSense_dsFlash.wdgtArray[6].ptr2SnsFlash)
#define CapSense_KEY06_PTR2SNS_FLASH_OFFSET                 (192u)
#define CapSense_KEY06_PTR2SNS_FLASH_SIZE                   (4u)
#define CapSense_KEY06_PTR2SNS_FLASH_PARAM_ID               (0xD50600C0u)

#define CapSense_KEY06_PTR2WD_RAM_VALUE                     (CapSense_dsFlash.wdgtArray[6].ptr2WdgtRam)
#define CapSense_KEY06_PTR2WD_RAM_OFFSET                    (196u)
#define CapSense_KEY06_PTR2WD_RAM_SIZE                      (4u)
#define CapSense_KEY06_PTR2WD_RAM_PARAM_ID                  (0xD40600C4u)

#define CapSense_KEY06_PTR2SNS_RAM_VALUE                    (CapSense_dsFlash.wdgtArray[6].ptr2SnsRam)
#define CapSense_KEY06_PTR2SNS_RAM_OFFSET                   (200u)
#define CapSense_KEY06_PTR2SNS_RAM_SIZE                     (4u)
#define CapSense_KEY06_PTR2SNS_RAM_PARAM_ID                 (0xD70600C8u)

#define CapSense_KEY06_PTR2FLTR_HISTORY_VALUE               (CapSense_dsFlash.wdgtArray[6].ptr2FltrHistory)
#define CapSense_KEY06_PTR2FLTR_HISTORY_OFFSET              (204u)
#define CapSense_KEY06_PTR2FLTR_HISTORY_SIZE                (4u)
#define CapSense_KEY06_PTR2FLTR_HISTORY_PARAM_ID            (0xD60600CCu)

#define CapSense_KEY06_PTR2DEBOUNCE_VALUE                   (CapSense_dsFlash.wdgtArray[6].ptr2DebounceArr)
#define CapSense_KEY06_PTR2DEBOUNCE_OFFSET                  (208u)
#define CapSense_KEY06_PTR2DEBOUNCE_SIZE                    (4u)
#define CapSense_KEY06_PTR2DEBOUNCE_PARAM_ID                (0xD00600D0u)

#define CapSense_KEY06_STATIC_CONFIG_VALUE                  (CapSense_dsFlash.wdgtArray[6].staticConfig)
#define CapSense_KEY06_STATIC_CONFIG_OFFSET                 (212u)
#define CapSense_KEY06_STATIC_CONFIG_SIZE                   (4u)
#define CapSense_KEY06_STATIC_CONFIG_PARAM_ID               (0xD10600D4u)

#define CapSense_KEY06_TOTAL_NUM_SNS_VALUE                  (CapSense_dsFlash.wdgtArray[6].totalNumSns)
#define CapSense_KEY06_TOTAL_NUM_SNS_OFFSET                 (216u)
#define CapSense_KEY06_TOTAL_NUM_SNS_SIZE                   (2u)
#define CapSense_KEY06_TOTAL_NUM_SNS_PARAM_ID               (0x9D0600D8u)

#define CapSense_KEY06_TYPE_VALUE                           (CapSense_dsFlash.wdgtArray[6].wdgtType)
#define CapSense_KEY06_TYPE_OFFSET                          (218u)
#define CapSense_KEY06_TYPE_SIZE                            (1u)
#define CapSense_KEY06_TYPE_PARAM_ID                        (0x590600DAu)

#define CapSense_KEY06_NUM_COLS_VALUE                       (CapSense_dsFlash.wdgtArray[6].numCols)
#define CapSense_KEY06_NUM_COLS_OFFSET                      (219u)
#define CapSense_KEY06_NUM_COLS_SIZE                        (1u)
#define CapSense_KEY06_NUM_COLS_PARAM_ID                    (0x5F0600DBu)

#define CapSense_KEY06_PTR2SNS_CP_VALUE                     (CapSense_dsFlash.wdgtArray[6].ptr2SnsCpArr)
#define CapSense_KEY06_PTR2SNS_CP_OFFSET                    (220u)
#define CapSense_KEY06_PTR2SNS_CP_SIZE                      (4u)
#define CapSense_KEY06_PTR2SNS_CP_PARAM_ID                  (0xD30600DCu)

#define CapSense_KEY07_PTR2SNS_FLASH_VALUE                  (CapSense_dsFlash.wdgtArray[7].ptr2SnsFlash)
#define CapSense_KEY07_PTR2SNS_FLASH_OFFSET                 (224u)
#define CapSense_KEY07_PTR2SNS_FLASH_SIZE                   (4u)
#define CapSense_KEY07_PTR2SNS_FLASH_PARAM_ID               (0xDC0700E0u)

#define CapSense_KEY07_PTR2WD_RAM_VALUE                     (CapSense_dsFlash.wdgtArray[7].ptr2WdgtRam)
#define CapSense_KEY07_PTR2WD_RAM_OFFSET                    (228u)
#define CapSense_KEY07_PTR2WD_RAM_SIZE                      (4u)
#define CapSense_KEY07_PTR2WD_RAM_PARAM_ID                  (0xDD0700E4u)

#define CapSense_KEY07_PTR2SNS_RAM_VALUE                    (CapSense_dsFlash.wdgtArray[7].ptr2SnsRam)
#define CapSense_KEY07_PTR2SNS_RAM_OFFSET                   (232u)
#define CapSense_KEY07_PTR2SNS_RAM_SIZE                     (4u)
#define CapSense_KEY07_PTR2SNS_RAM_PARAM_ID                 (0xDE0700E8u)

#define CapSense_KEY07_PTR2FLTR_HISTORY_VALUE               (CapSense_dsFlash.wdgtArray[7].ptr2FltrHistory)
#define CapSense_KEY07_PTR2FLTR_HISTORY_OFFSET              (236u)
#define CapSense_KEY07_PTR2FLTR_HISTORY_SIZE                (4u)
#define CapSense_KEY07_PTR2FLTR_HISTORY_PARAM_ID            (0xDF0700ECu)

#define CapSense_KEY07_PTR2DEBOUNCE_VALUE                   (CapSense_dsFlash.wdgtArray[7].ptr2DebounceArr)
#define CapSense_KEY07_PTR2DEBOUNCE_OFFSET                  (240u)
#define CapSense_KEY07_PTR2DEBOUNCE_SIZE                    (4u)
#define CapSense_KEY07_PTR2DEBOUNCE_PARAM_ID                (0xD90700F0u)

#define CapSense_KEY07_STATIC_CONFIG_VALUE                  (CapSense_dsFlash.wdgtArray[7].staticConfig)
#define CapSense_KEY07_STATIC_CONFIG_OFFSET                 (244u)
#define CapSense_KEY07_STATIC_CONFIG_SIZE                   (4u)
#define CapSense_KEY07_STATIC_CONFIG_PARAM_ID               (0xD80700F4u)

#define CapSense_KEY07_TOTAL_NUM_SNS_VALUE                  (CapSense_dsFlash.wdgtArray[7].totalNumSns)
#define CapSense_KEY07_TOTAL_NUM_SNS_OFFSET                 (248u)
#define CapSense_KEY07_TOTAL_NUM_SNS_SIZE                   (2u)
#define CapSense_KEY07_TOTAL_NUM_SNS_PARAM_ID               (0x940700F8u)

#define CapSense_KEY07_TYPE_VALUE                           (CapSense_dsFlash.wdgtArray[7].wdgtType)
#define CapSense_KEY07_TYPE_OFFSET                          (250u)
#define CapSense_KEY07_TYPE_SIZE                            (1u)
#define CapSense_KEY07_TYPE_PARAM_ID                        (0x500700FAu)

#define CapSense_KEY07_NUM_COLS_VALUE                       (CapSense_dsFlash.wdgtArray[7].numCols)
#define CapSense_KEY07_NUM_COLS_OFFSET                      (251u)
#define CapSense_KEY07_NUM_COLS_SIZE                        (1u)
#define CapSense_KEY07_NUM_COLS_PARAM_ID                    (0x560700FBu)

#define CapSense_KEY07_PTR2SNS_CP_VALUE                     (CapSense_dsFlash.wdgtArray[7].ptr2SnsCpArr)
#define CapSense_KEY07_PTR2SNS_CP_OFFSET                    (252u)
#define CapSense_KEY07_PTR2SNS_CP_SIZE                      (4u)
#define CapSense_KEY07_PTR2SNS_CP_PARAM_ID                  (0xDA0700FCu)

#define CapSense_KEY08_PTR2SNS_FLASH_VALUE                  (CapSense_dsFlash.wdgtArray[8].ptr2SnsFlash)
#define CapSense_KEY08_PTR2SNS_FLASH_OFFSET                 (256u)
#define CapSense_KEY08_PTR2SNS_FLASH_SIZE                   (4u)
#define CapSense_KEY08_PTR2SNS_FLASH_PARAM_ID               (0xDB080100u)

#define CapSense_KEY08_PTR2WD_RAM_VALUE                     (CapSense_dsFlash.wdgtArray[8].ptr2WdgtRam)
#define CapSense_KEY08_PTR2WD_RAM_OFFSET                    (260u)
#define CapSense_KEY08_PTR2WD_RAM_SIZE                      (4u)
#define CapSense_KEY08_PTR2WD_RAM_PARAM_ID                  (0xDA080104u)

#define CapSense_KEY08_PTR2SNS_RAM_VALUE                    (CapSense_dsFlash.wdgtArray[8].ptr2SnsRam)
#define CapSense_KEY08_PTR2SNS_RAM_OFFSET                   (264u)
#define CapSense_KEY08_PTR2SNS_RAM_SIZE                     (4u)
#define CapSense_KEY08_PTR2SNS_RAM_PARAM_ID                 (0xD9080108u)

#define CapSense_KEY08_PTR2FLTR_HISTORY_VALUE               (CapSense_dsFlash.wdgtArray[8].ptr2FltrHistory)
#define CapSense_KEY08_PTR2FLTR_HISTORY_OFFSET              (268u)
#define CapSense_KEY08_PTR2FLTR_HISTORY_SIZE                (4u)
#define CapSense_KEY08_PTR2FLTR_HISTORY_PARAM_ID            (0xD808010Cu)

#define CapSense_KEY08_PTR2DEBOUNCE_VALUE                   (CapSense_dsFlash.wdgtArray[8].ptr2DebounceArr)
#define CapSense_KEY08_PTR2DEBOUNCE_OFFSET                  (272u)
#define CapSense_KEY08_PTR2DEBOUNCE_SIZE                    (4u)
#define CapSense_KEY08_PTR2DEBOUNCE_PARAM_ID                (0xDE080110u)

#define CapSense_KEY08_STATIC_CONFIG_VALUE                  (CapSense_dsFlash.wdgtArray[8].staticConfig)
#define CapSense_KEY08_STATIC_CONFIG_OFFSET                 (276u)
#define CapSense_KEY08_STATIC_CONFIG_SIZE                   (4u)
#define CapSense_KEY08_STATIC_CONFIG_PARAM_ID               (0xDF080114u)

#define CapSense_KEY08_TOTAL_NUM_SNS_VALUE                  (CapSense_dsFlash.wdgtArray[8].totalNumSns)
#define CapSense_KEY08_TOTAL_NUM_SNS_OFFSET                 (280u)
#define CapSense_KEY08_TOTAL_NUM_SNS_SIZE                   (2u)
#define CapSense_KEY08_TOTAL_NUM_SNS_PARAM_ID               (0x93080118u)

#define CapSense_KEY08_TYPE_VALUE                           (CapSense_dsFlash.wdgtArray[8].wdgtType)
#define CapSense_KEY08_TYPE_OFFSET                          (282u)
#define CapSense_KEY08_TYPE_SIZE                            (1u)
#define CapSense_KEY08_TYPE_PARAM_ID                        (0x5708011Au)

#define CapSense_KEY08_NUM_COLS_VALUE                       (CapSense_dsFlash.wdgtArray[8].numCols)
#define CapSense_KEY08_NUM_COLS_OFFSET                      (283u)
#define CapSense_KEY08_NUM_COLS_SIZE                        (1u)
#define CapSense_KEY08_NUM_COLS_PARAM_ID                    (0x5108011Bu)

#define CapSense_KEY08_PTR2SNS_CP_VALUE                     (CapSense_dsFlash.wdgtArray[8].ptr2SnsCpArr)
#define CapSense_KEY08_PTR2SNS_CP_OFFSET                    (284u)
#define CapSense_KEY08_PTR2SNS_CP_SIZE                      (4u)
#define CapSense_KEY08_PTR2SNS_CP_PARAM_ID                  (0xDD08011Cu)

#define CapSense_KEY09_PTR2SNS_FLASH_VALUE                  (CapSense_dsFlash.wdgtArray[9].ptr2SnsFlash)
#define CapSense_KEY09_PTR2SNS_FLASH_OFFSET                 (288u)
#define CapSense_KEY09_PTR2SNS_FLASH_SIZE                   (4u)
#define CapSense_KEY09_PTR2SNS_FLASH_PARAM_ID               (0xD2090120u)

#define CapSense_KEY09_PTR2WD_RAM_VALUE                     (CapSense_dsFlash.wdgtArray[9].ptr2WdgtRam)
#define CapSense_KEY09_PTR2WD_RAM_OFFSET                    (292u)
#define CapSense_KEY09_PTR2WD_RAM_SIZE                      (4u)
#define CapSense_KEY09_PTR2WD_RAM_PARAM_ID                  (0xD3090124u)

#define CapSense_KEY09_PTR2SNS_RAM_VALUE                    (CapSense_dsFlash.wdgtArray[9].ptr2SnsRam)
#define CapSense_KEY09_PTR2SNS_RAM_OFFSET                   (296u)
#define CapSense_KEY09_PTR2SNS_RAM_SIZE                     (4u)
#define CapSense_KEY09_PTR2SNS_RAM_PARAM_ID                 (0xD0090128u)

#define CapSense_KEY09_PTR2FLTR_HISTORY_VALUE               (CapSense_dsFlash.wdgtArray[9].ptr2FltrHistory)
#define CapSense_KEY09_PTR2FLTR_HISTORY_OFFSET              (300u)
#define CapSense_KEY09_PTR2FLTR_HISTORY_SIZE                (4u)
#define CapSense_KEY09_PTR2FLTR_HISTORY_PARAM_ID            (0xD109012Cu)

#define CapSense_KEY09_PTR2DEBOUNCE_VALUE                   (CapSense_dsFlash.wdgtArray[9].ptr2DebounceArr)
#define CapSense_KEY09_PTR2DEBOUNCE_OFFSET                  (304u)
#define CapSense_KEY09_PTR2DEBOUNCE_SIZE                    (4u)
#define CapSense_KEY09_PTR2DEBOUNCE_PARAM_ID                (0xD7090130u)

#define CapSense_KEY09_STATIC_CONFIG_VALUE                  (CapSense_dsFlash.wdgtArray[9].staticConfig)
#define CapSense_KEY09_STATIC_CONFIG_OFFSET                 (308u)
#define CapSense_KEY09_STATIC_CONFIG_SIZE                   (4u)
#define CapSense_KEY09_STATIC_CONFIG_PARAM_ID               (0xD6090134u)

#define CapSense_KEY09_TOTAL_NUM_SNS_VALUE                  (CapSense_dsFlash.wdgtArray[9].totalNumSns)
#define CapSense_KEY09_TOTAL_NUM_SNS_OFFSET                 (312u)
#define CapSense_KEY09_TOTAL_NUM_SNS_SIZE                   (2u)
#define CapSense_KEY09_TOTAL_NUM_SNS_PARAM_ID               (0x9A090138u)

#define CapSense_KEY09_TYPE_VALUE                           (CapSense_dsFlash.wdgtArray[9].wdgtType)
#define CapSense_KEY09_TYPE_OFFSET                          (314u)
#define CapSense_KEY09_TYPE_SIZE                            (1u)
#define CapSense_KEY09_TYPE_PARAM_ID                        (0x5E09013Au)

#define CapSense_KEY09_NUM_COLS_VALUE                       (CapSense_dsFlash.wdgtArray[9].numCols)
#define CapSense_KEY09_NUM_COLS_OFFSET                      (315u)
#define CapSense_KEY09_NUM_COLS_SIZE                        (1u)
#define CapSense_KEY09_NUM_COLS_PARAM_ID                    (0x5809013Bu)

#define CapSense_KEY09_PTR2SNS_CP_VALUE                     (CapSense_dsFlash.wdgtArray[9].ptr2SnsCpArr)
#define CapSense_KEY09_PTR2SNS_CP_OFFSET                    (316u)
#define CapSense_KEY09_PTR2SNS_CP_SIZE                      (4u)
#define CapSense_KEY09_PTR2SNS_CP_PARAM_ID                  (0xD409013Cu)

#define CapSense_KEY10_PTR2SNS_FLASH_VALUE                  (CapSense_dsFlash.wdgtArray[10].ptr2SnsFlash)
#define CapSense_KEY10_PTR2SNS_FLASH_OFFSET                 (320u)
#define CapSense_KEY10_PTR2SNS_FLASH_SIZE                   (4u)
#define CapSense_KEY10_PTR2SNS_FLASH_PARAM_ID               (0xD00A0140u)

#define CapSense_KEY10_PTR2WD_RAM_VALUE                     (CapSense_dsFlash.wdgtArray[10].ptr2WdgtRam)
#define CapSense_KEY10_PTR2WD_RAM_OFFSET                    (324u)
#define CapSense_KEY10_PTR2WD_RAM_SIZE                      (4u)
#define CapSense_KEY10_PTR2WD_RAM_PARAM_ID                  (0xD10A0144u)

#define CapSense_KEY10_PTR2SNS_RAM_VALUE                    (CapSense_dsFlash.wdgtArray[10].ptr2SnsRam)
#define CapSense_KEY10_PTR2SNS_RAM_OFFSET                   (328u)
#define CapSense_KEY10_PTR2SNS_RAM_SIZE                     (4u)
#define CapSense_KEY10_PTR2SNS_RAM_PARAM_ID                 (0xD20A0148u)

#define CapSense_KEY10_PTR2FLTR_HISTORY_VALUE               (CapSense_dsFlash.wdgtArray[10].ptr2FltrHistory)
#define CapSense_KEY10_PTR2FLTR_HISTORY_OFFSET              (332u)
#define CapSense_KEY10_PTR2FLTR_HISTORY_SIZE                (4u)
#define CapSense_KEY10_PTR2FLTR_HISTORY_PARAM_ID            (0xD30A014Cu)

#define CapSense_KEY10_PTR2DEBOUNCE_VALUE                   (CapSense_dsFlash.wdgtArray[10].ptr2DebounceArr)
#define CapSense_KEY10_PTR2DEBOUNCE_OFFSET                  (336u)
#define CapSense_KEY10_PTR2DEBOUNCE_SIZE                    (4u)
#define CapSense_KEY10_PTR2DEBOUNCE_PARAM_ID                (0xD50A0150u)

#define CapSense_KEY10_STATIC_CONFIG_VALUE                  (CapSense_dsFlash.wdgtArray[10].staticConfig)
#define CapSense_KEY10_STATIC_CONFIG_OFFSET                 (340u)
#define CapSense_KEY10_STATIC_CONFIG_SIZE                   (4u)
#define CapSense_KEY10_STATIC_CONFIG_PARAM_ID               (0xD40A0154u)

#define CapSense_KEY10_TOTAL_NUM_SNS_VALUE                  (CapSense_dsFlash.wdgtArray[10].totalNumSns)
#define CapSense_KEY10_TOTAL_NUM_SNS_OFFSET                 (344u)
#define CapSense_KEY10_TOTAL_NUM_SNS_SIZE                   (2u)
#define CapSense_KEY10_TOTAL_NUM_SNS_PARAM_ID               (0x980A0158u)

#define CapSense_KEY10_TYPE_VALUE                           (CapSense_dsFlash.wdgtArray[10].wdgtType)
#define CapSense_KEY10_TYPE_OFFSET                          (346u)
#define CapSense_KEY10_TYPE_SIZE                            (1u)
#define CapSense_KEY10_TYPE_PARAM_ID                        (0x5C0A015Au)

#define CapSense_KEY10_NUM_COLS_VALUE                       (CapSense_dsFlash.wdgtArray[10].numCols)
#define CapSense_KEY10_NUM_COLS_OFFSET                      (347u)
#define CapSense_KEY10_NUM_COLS_SIZE                        (1u)
#define CapSense_KEY10_NUM_COLS_PARAM_ID                    (0x5A0A015Bu)

#define CapSense_KEY10_PTR2SNS_CP_VALUE                     (CapSense_dsFlash.wdgtArray[10].ptr2SnsCpArr)
#define CapSense_KEY10_PTR2SNS_CP_OFFSET                    (348u)
#define CapSense_KEY10_PTR2SNS_CP_SIZE                      (4u)
#define CapSense_KEY10_PTR2SNS_CP_PARAM_ID                  (0xD60A015Cu)

#define CapSense_KEY11_PTR2SNS_FLASH_VALUE                  (CapSense_dsFlash.wdgtArray[11].ptr2SnsFlash)
#define CapSense_KEY11_PTR2SNS_FLASH_OFFSET                 (352u)
#define CapSense_KEY11_PTR2SNS_FLASH_SIZE                   (4u)
#define CapSense_KEY11_PTR2SNS_FLASH_PARAM_ID               (0xD90B0160u)

#define CapSense_KEY11_PTR2WD_RAM_VALUE                     (CapSense_dsFlash.wdgtArray[11].ptr2WdgtRam)
#define CapSense_KEY11_PTR2WD_RAM_OFFSET                    (356u)
#define CapSense_KEY11_PTR2WD_RAM_SIZE                      (4u)
#define CapSense_KEY11_PTR2WD_RAM_PARAM_ID                  (0xD80B0164u)

#define CapSense_KEY11_PTR2SNS_RAM_VALUE                    (CapSense_dsFlash.wdgtArray[11].ptr2SnsRam)
#define CapSense_KEY11_PTR2SNS_RAM_OFFSET                   (360u)
#define CapSense_KEY11_PTR2SNS_RAM_SIZE                     (4u)
#define CapSense_KEY11_PTR2SNS_RAM_PARAM_ID                 (0xDB0B0168u)

#define CapSense_KEY11_PTR2FLTR_HISTORY_VALUE               (CapSense_dsFlash.wdgtArray[11].ptr2FltrHistory)
#define CapSense_KEY11_PTR2FLTR_HISTORY_OFFSET              (364u)
#define CapSense_KEY11_PTR2FLTR_HISTORY_SIZE                (4u)
#define CapSense_KEY11_PTR2FLTR_HISTORY_PARAM_ID            (0xDA0B016Cu)

#define CapSense_KEY11_PTR2DEBOUNCE_VALUE                   (CapSense_dsFlash.wdgtArray[11].ptr2DebounceArr)
#define CapSense_KEY11_PTR2DEBOUNCE_OFFSET                  (368u)
#define CapSense_KEY11_PTR2DEBOUNCE_SIZE                    (4u)
#define CapSense_KEY11_PTR2DEBOUNCE_PARAM_ID                (0xDC0B0170u)

#define CapSense_KEY11_STATIC_CONFIG_VALUE                  (CapSense_dsFlash.wdgtArray[11].staticConfig)
#define CapSense_KEY11_STATIC_CONFIG_OFFSET                 (372u)
#define CapSense_KEY11_STATIC_CONFIG_SIZE                   (4u)
#define CapSense_KEY11_STATIC_CONFIG_PARAM_ID               (0xDD0B0174u)

#define CapSense_KEY11_TOTAL_NUM_SNS_VALUE                  (CapSense_dsFlash.wdgtArray[11].totalNumSns)
#define CapSense_KEY11_TOTAL_NUM_SNS_OFFSET                 (376u)
#define CapSense_KEY11_TOTAL_NUM_SNS_SIZE                   (2u)
#define CapSense_KEY11_TOTAL_NUM_SNS_PARAM_ID               (0x910B0178u)

#define CapSense_KEY11_TYPE_VALUE                           (CapSense_dsFlash.wdgtArray[11].wdgtType)
#define CapSense_KEY11_TYPE_OFFSET                          (378u)
#define CapSense_KEY11_TYPE_SIZE                            (1u)
#define CapSense_KEY11_TYPE_PARAM_ID                        (0x550B017Au)

#define CapSense_KEY11_NUM_COLS_VALUE                       (CapSense_dsFlash.wdgtArray[11].numCols)
#define CapSense_KEY11_NUM_COLS_OFFSET                      (379u)
#define CapSense_KEY11_NUM_COLS_SIZE                        (1u)
#define CapSense_KEY11_NUM_COLS_PARAM_ID                    (0x530B017Bu)

#define CapSense_KEY11_PTR2SNS_CP_VALUE                     (CapSense_dsFlash.wdgtArray[11].ptr2SnsCpArr)
#define CapSense_KEY11_PTR2SNS_CP_OFFSET                    (380u)
#define CapSense_KEY11_PTR2SNS_CP_SIZE                      (4u)
#define CapSense_KEY11_PTR2SNS_CP_PARAM_ID                  (0xDF0B017Cu)

#define CapSense_KEY12_PTR2SNS_FLASH_VALUE                  (CapSense_dsFlash.wdgtArray[12].ptr2SnsFlash)
#define CapSense_KEY12_PTR2SNS_FLASH_OFFSET                 (384u)
#define CapSense_KEY12_PTR2SNS_FLASH_SIZE                   (4u)
#define CapSense_KEY12_PTR2SNS_FLASH_PARAM_ID               (0xD40C0180u)

#define CapSense_KEY12_PTR2WD_RAM_VALUE                     (CapSense_dsFlash.wdgtArray[12].ptr2WdgtRam)
#define CapSense_KEY12_PTR2WD_RAM_OFFSET                    (388u)
#define CapSense_KEY12_PTR2WD_RAM_SIZE                      (4u)
#define CapSense_KEY12_PTR2WD_RAM_PARAM_ID                  (0xD50C0184u)

#define CapSense_KEY12_PTR2SNS_RAM_VALUE                    (CapSense_dsFlash.wdgtArray[12].ptr2SnsRam)
#define CapSense_KEY12_PTR2SNS_RAM_OFFSET                   (392u)
#define CapSense_KEY12_PTR2SNS_RAM_SIZE                     (4u)
#define CapSense_KEY12_PTR2SNS_RAM_PARAM_ID                 (0xD60C0188u)

#define CapSense_KEY12_PTR2FLTR_HISTORY_VALUE               (CapSense_dsFlash.wdgtArray[12].ptr2FltrHistory)
#define CapSense_KEY12_PTR2FLTR_HISTORY_OFFSET              (396u)
#define CapSense_KEY12_PTR2FLTR_HISTORY_SIZE                (4u)
#define CapSense_KEY12_PTR2FLTR_HISTORY_PARAM_ID            (0xD70C018Cu)

#define CapSense_KEY12_PTR2DEBOUNCE_VALUE                   (CapSense_dsFlash.wdgtArray[12].ptr2DebounceArr)
#define CapSense_KEY12_PTR2DEBOUNCE_OFFSET                  (400u)
#define CapSense_KEY12_PTR2DEBOUNCE_SIZE                    (4u)
#define CapSense_KEY12_PTR2DEBOUNCE_PARAM_ID                (0xD10C0190u)

#define CapSense_KEY12_STATIC_CONFIG_VALUE                  (CapSense_dsFlash.wdgtArray[12].staticConfig)
#define CapSense_KEY12_STATIC_CONFIG_OFFSET                 (404u)
#define CapSense_KEY12_STATIC_CONFIG_SIZE                   (4u)
#define CapSense_KEY12_STATIC_CONFIG_PARAM_ID               (0xD00C0194u)

#define CapSense_KEY12_TOTAL_NUM_SNS_VALUE                  (CapSense_dsFlash.wdgtArray[12].totalNumSns)
#define CapSense_KEY12_TOTAL_NUM_SNS_OFFSET                 (408u)
#define CapSense_KEY12_TOTAL_NUM_SNS_SIZE                   (2u)
#define CapSense_KEY12_TOTAL_NUM_SNS_PARAM_ID               (0x9C0C0198u)

#define CapSense_KEY12_TYPE_VALUE                           (CapSense_dsFlash.wdgtArray[12].wdgtType)
#define CapSense_KEY12_TYPE_OFFSET                          (410u)
#define CapSense_KEY12_TYPE_SIZE                            (1u)
#define CapSense_KEY12_TYPE_PARAM_ID                        (0x580C019Au)

#define CapSense_KEY12_NUM_COLS_VALUE                       (CapSense_dsFlash.wdgtArray[12].numCols)
#define CapSense_KEY12_NUM_COLS_OFFSET                      (411u)
#define CapSense_KEY12_NUM_COLS_SIZE                        (1u)
#define CapSense_KEY12_NUM_COLS_PARAM_ID                    (0x5E0C019Bu)

#define CapSense_KEY12_PTR2SNS_CP_VALUE                     (CapSense_dsFlash.wdgtArray[12].ptr2SnsCpArr)
#define CapSense_KEY12_PTR2SNS_CP_OFFSET                    (412u)
#define CapSense_KEY12_PTR2SNS_CP_SIZE                      (4u)
#define CapSense_KEY12_PTR2SNS_CP_PARAM_ID                  (0xD20C019Cu)


#endif /* End CY_SENSE_CapSense_REGISTER_MAP_H */


/* [] END OF FILE */
