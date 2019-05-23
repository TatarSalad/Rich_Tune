/*******************************************************************************
* \file UL_testing_Tuning_Configuration.h
* \version 6.0
*
* \brief
*   This file provides the customizer parameters definitions.
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

#if !defined(CY_SENSE_UL_testing_Tuning_CONFIGURATION_H)
#define CY_SENSE_UL_testing_Tuning_CONFIGURATION_H

#include <cytypes.h>

/*******************************************************************************
* Customizer-generated defines
*******************************************************************************/
#define UL_testing_Tuning_ENABLE                             (1u)
#define UL_testing_Tuning_DISABLE                            (0u)

#define UL_testing_Tuning_THIRD_GENERATION_BLOCK             (1u)
#define UL_testing_Tuning_FOURTH_GENERATION_BLOCK            (2u)

#define UL_testing_Tuning_GENERATION_BLOCK_VERSION           (2u)

/*******************************************************************************
* HW IP block global defines
*******************************************************************************/

#if (UL_testing_Tuning_GENERATION_BLOCK_VERSION == UL_testing_Tuning_THIRD_GENERATION_BLOCK)
    #define UL_testing_Tuning_CSDV1                          (UL_testing_Tuning_ENABLE)
    
    #ifdef CYIPBLOCK_m0s8csd_VERSION
        #if (0u == CYIPBLOCK_m0s8csd_VERSION)
            #define UL_testing_Tuning_CSDV1_VER1             (UL_testing_Tuning_ENABLE)
            #define UL_testing_Tuning_CSDV1_VER2             (UL_testing_Tuning_DISABLE)
        #else
            #define UL_testing_Tuning_CSDV1_VER1             (UL_testing_Tuning_DISABLE)
            #define UL_testing_Tuning_CSDV1_VER2             (UL_testing_Tuning_ENABLE)
        #endif
    #else
        #error "HW IP block version is not specified"
    #endif
#else
    #define UL_testing_Tuning_CSDV1                          (UL_testing_Tuning_DISABLE)
    #define UL_testing_Tuning_CSDV1_VER1                     (UL_testing_Tuning_DISABLE)
    #define UL_testing_Tuning_CSDV1_VER2                     (UL_testing_Tuning_DISABLE)
#endif

#if (UL_testing_Tuning_GENERATION_BLOCK_VERSION == UL_testing_Tuning_FOURTH_GENERATION_BLOCK)
    #define UL_testing_Tuning_CSDV2                          (UL_testing_Tuning_ENABLE)
    
    #ifdef CYIPBLOCK_m0s8csdv2_VERSION
        #if (1u == CYIPBLOCK_m0s8csdv2_VERSION)
            #define UL_testing_Tuning_CSDV2_VER1             (UL_testing_Tuning_ENABLE)
        #else
            #define UL_testing_Tuning_CSDV2_VER1             (UL_testing_Tuning_DISABLE)
        #endif
        #if (2u == CYIPBLOCK_m0s8csdv2_VERSION)
            #define UL_testing_Tuning_CSDV2_VER2             (UL_testing_Tuning_ENABLE)
        #else
            #define UL_testing_Tuning_CSDV2_VER2             (UL_testing_Tuning_DISABLE)
        #endif
    #else
        #error "HW IP block version is not specified"
    #endif
#else
    #define UL_testing_Tuning_CSDV2                          (UL_testing_Tuning_DISABLE)
    #define UL_testing_Tuning_CSDV2_VER1                     (UL_testing_Tuning_DISABLE)
    #define UL_testing_Tuning_CSDV2_VER2                     (UL_testing_Tuning_DISABLE)
#endif

/*******************************************************************************
* Project-global defines
*******************************************************************************/

#define UL_testing_Tuning_2000_MV                            (2000u)

#ifdef CYDEV_VDDA_MV
    #define UL_testing_Tuning_CYDEV_VDDA_MV                  (CYDEV_VDDA_MV)
#else
    #ifdef CYDEV_VDD_MV
        #define UL_testing_Tuning_CYDEV_VDDA_MV              (CYDEV_VDD_MV)
    #endif
#endif

#define UL_testing_Tuning_BAD_CONVERSIONS_NUM                (1u)
#define UL_testing_Tuning_RESAMPLING_CYCLES_MAX_NUMBER       (1u)


/*******************************************************************************
* Enabled Scan Methods
*******************************************************************************/
#define UL_testing_Tuning_CSD_EN                             (1u)
#define UL_testing_Tuning_CSX_EN                             (0u)
#define UL_testing_Tuning_ISX_EN                             (0u)
#define UL_testing_Tuning_CSD_CSX_EN                         (UL_testing_Tuning_CSD_EN && UL_testing_Tuning_CSX_EN)

#define UL_testing_Tuning_MANY_SENSE_MODES_EN                ((UL_testing_Tuning_CSD_EN && UL_testing_Tuning_CSX_EN) || \
                                                             (UL_testing_Tuning_CSD_EN && UL_testing_Tuning_ISX_EN) || \
                                                             (UL_testing_Tuning_CSX_EN && UL_testing_Tuning_ISX_EN) || \
                                                             (UL_testing_Tuning_SELF_TEST_EN))

#define UL_testing_Tuning_MANY_WIDGET_METHODS_EN             ((UL_testing_Tuning_CSD_EN && UL_testing_Tuning_CSX_EN) || \
                                                             (UL_testing_Tuning_CSD_EN && UL_testing_Tuning_ISX_EN) || \
                                                             (UL_testing_Tuning_CSX_EN && UL_testing_Tuning_ISX_EN))

#define UL_testing_Tuning_CSD2X_EN                           (0u)
#define UL_testing_Tuning_CSX2X_EN                           (0u)

/*******************************************************************************
* Definitions for number of widgets and sensors
*******************************************************************************/
#define UL_testing_Tuning_TOTAL_WIDGETS                      (2u)
#define UL_testing_Tuning_TOTAL_CSD_WIDGETS                  (2u)
#define UL_testing_Tuning_TOTAL_CSD_SENSORS                  (2u)
#define UL_testing_Tuning_TOTAL_CSX_WIDGETS                  (0u)
#define UL_testing_Tuning_TOTAL_ISX_WIDGETS                  (0u)
#define UL_testing_Tuning_TOTAL_CSX_NODES                    (0u)
#define UL_testing_Tuning_TOTAL_ISX_NODES                    (0u)

/*******************************************************************************
* Total number of CSD sensors + CSX nodes
*******************************************************************************/
#define UL_testing_Tuning_TOTAL_SENSORS            (UL_testing_Tuning_TOTAL_CSD_SENSORS + \
                                                   UL_testing_Tuning_TOTAL_CSX_NODES+ \
                                                   UL_testing_Tuning_TOTAL_ISX_NODES)

/*******************************************************************************
* Total number of scan slots (used only when dual-channel scan is enabled)
*******************************************************************************/
#define UL_testing_Tuning_TOTAL_SCAN_SLOTS         (2u)

/*******************************************************************************
* Defines widget IDs
*******************************************************************************/
#define UL_testing_Tuning_BUTTON0_WDGT_ID       (0u)
#define UL_testing_Tuning_BUTTON1_WDGT_ID       (1u)

/*******************************************************************************
* Defines sensor IDs
*******************************************************************************/

/* Button0 sensor names */
#define UL_testing_Tuning_BUTTON0_SNS0_ID       (0u)

/* Button1 sensor names */
#define UL_testing_Tuning_BUTTON1_SNS0_ID       (0u)



/*******************************************************************************
* Enabled widget types
*******************************************************************************/
#define UL_testing_Tuning_BUTTON_WIDGET_EN         (1u)
#define UL_testing_Tuning_SLIDER_WIDGET_EN         (0u)
#define UL_testing_Tuning_MATRIX_WIDGET_EN         (0u)
#define UL_testing_Tuning_PROXIMITY_WIDGET_EN      (0u)
#define UL_testing_Tuning_TOUCHPAD_WIDGET_EN       (0u)
#define UL_testing_Tuning_ENCODERDIAL_WIDGET_EN    (0u)

#define UL_testing_Tuning_CSD_MATRIX_WIDGET_EN     (0u)
#define UL_testing_Tuning_CSD_TOUCHPAD_WIDGET_EN   (0u)

#define UL_testing_Tuning_CSX_MATRIX_WIDGET_EN     (0u)
#define UL_testing_Tuning_CSX_TOUCHPAD_WIDGET_EN   (0u)

/*******************************************************************************
* Centroid APIs
*******************************************************************************/
#define UL_testing_Tuning_CENTROID_EN              (0u)
#define UL_testing_Tuning_TOTAL_DIPLEXED_SLIDERS   (0u)
#define UL_testing_Tuning_TOTAL_LINEAR_SLIDERS     (0u)
#define UL_testing_Tuning_TOTAL_RADIAL_SLIDERS     (0u)
#define UL_testing_Tuning_TOTAL_TOUCHPADS          (0u)
#define UL_testing_Tuning_MAX_CENTROID_LENGTH      (0u)
#define UL_testing_Tuning_SLIDER_MULT_METHOD       (0u)
#define UL_testing_Tuning_TOUCHPAD_MULT_METHOD     (0u)

/*******************************************************************************
* Enabled sensor types
*******************************************************************************/
#define UL_testing_Tuning_REGULAR_SENSOR_EN        (1u)
#define UL_testing_Tuning_PROXIMITY_SENSOR_EN      (0u)

/*******************************************************************************
* Sensor ganging
*******************************************************************************/
#define UL_testing_Tuning_GANGED_SNS_EN            (0u)
#define UL_testing_Tuning_CSD_GANGED_SNS_EN        (0u)
#define UL_testing_Tuning_CSX_GANGED_SNS_EN        (0u)

/*******************************************************************************
* Max number of sensors used among all the widgets
*******************************************************************************/
#define UL_testing_Tuning_MAX_SENSORS_PER_WIDGET   (1u)
#define UL_testing_Tuning_MAX_SENSORS_PER_5X5_TOUCHPAD (1u)

/*******************************************************************************
* Total number of all used electrodes (NOT unique)
*******************************************************************************/
#define UL_testing_Tuning_TOTAL_ELECTRODES         (2u)
/* Obsolete */
#define UL_testing_Tuning_TOTAL_SENSOR_IOS         UL_testing_Tuning_TOTAL_ELECTRODES

/*******************************************************************************
* Total number of used physical IOs (unique)
*******************************************************************************/
#define UL_testing_Tuning_TOTAL_IO_CNT             (2u)

/*******************************************************************************
* Array length for widget status registers
*******************************************************************************/
#define UL_testing_Tuning_WDGT_STATUS_WORDS        \
                        (((uint8)((UL_testing_Tuning_TOTAL_WIDGETS - 1u) / 32u)) + 1u)


/*******************************************************************************
* Auto-tuning mode selection
*******************************************************************************/
#define UL_testing_Tuning_CSD_SS_DIS         (0x00ul)
#define UL_testing_Tuning_CSD_SS_HW_EN       (0x01ul)
#define UL_testing_Tuning_CSD_SS_TH_EN       (0x02ul)
#define UL_testing_Tuning_CSD_SS_HWTH_EN     (UL_testing_Tuning_CSD_SS_HW_EN | \
                                             UL_testing_Tuning_CSD_SS_TH_EN)

#define UL_testing_Tuning_CSD_AUTOTUNE       UL_testing_Tuning_CSD_SS_DIS


/*******************************************************************************
* General settings
*******************************************************************************/

#define UL_testing_Tuning_AUTO_RESET_METHOD_LEGACY (0u)
#define UL_testing_Tuning_AUTO_RESET_METHOD_SAMPLE (1u)

#define UL_testing_Tuning_MULTI_FREQ_SCAN_EN       (0u)
#define UL_testing_Tuning_SENSOR_AUTO_RESET_EN     (0u)
#define UL_testing_Tuning_SENSOR_AUTO_RESET_METHOD (0u)
#define UL_testing_Tuning_NUM_CENTROIDS            (1u)
#define UL_testing_Tuning_4PTS_LOCAL_MAX_EN        (0u)
#define UL_testing_Tuning_OFF_DEBOUNCE_EN          (0u)
#define UL_testing_Tuning_CUSTOM_DS_RAM_SIZE       (0u)

/* Defines power status of HW block after scanning */
#define UL_testing_Tuning_BLOCK_OFF_AFTER_SCAN_EN  (0u)

/* Defines number of scan frequencies */
#if (UL_testing_Tuning_DISABLE != UL_testing_Tuning_MULTI_FREQ_SCAN_EN)
    #define UL_testing_Tuning_NUM_SCAN_FREQS       (3u)
#else
    #define UL_testing_Tuning_NUM_SCAN_FREQS       (1u)
#endif /* #if (UL_testing_Tuning_DISABLE != UL_testing_Tuning_MULTI_FREQ_SCAN_EN) */

/* Data size for thresholds / low baseline reset */
#define UL_testing_Tuning_SIZE_8BITS               (8u)
#define UL_testing_Tuning_SIZE_16BITS              (16u)

#define UL_testing_Tuning_THRESHOLD_SIZE           UL_testing_Tuning_SIZE_16BITS
typedef uint16 UL_testing_Tuning_THRESHOLD_TYPE;

#if (UL_testing_Tuning_AUTO_RESET_METHOD_LEGACY == UL_testing_Tuning_SENSOR_AUTO_RESET_METHOD)
    #define UL_testing_Tuning_LOW_BSLN_RST_SIZE        UL_testing_Tuning_SIZE_8BITS
    typedef uint8 UL_testing_Tuning_LOW_BSLN_RST_TYPE;
#else
    #define UL_testing_Tuning_LOW_BSLN_RST_SIZE    (16u)
    typedef uint16 UL_testing_Tuning_LOW_BSLN_RST_TYPE;
#endif /* #if (UL_testing_Tuning_AUTO_RESET_METHOD_LEGACY == UL_testing_Tuning_SENSOR_AUTO_RESET_METHOD) */

/* Coefficient to define touch threshold for proximity sensors */
#define UL_testing_Tuning_PROX_TOUCH_COEFF         (300u)

/*******************************************************************************
* General Filter Constants
*******************************************************************************/

/* Baseline algorithm options */
#define UL_testing_Tuning_IIR_BASELINE                 (0u)
#define UL_testing_Tuning_BUCKET_BASELINE              (1u)

#define UL_testing_Tuning_BASELINE_TYPE                UL_testing_Tuning_IIR_BASELINE

/* IIR baseline filter algorithm for regular sensors*/
#define UL_testing_Tuning_REGULAR_IIR_BL_TYPE          UL_testing_Tuning_IIR_FILTER_PERFORMANCE

/* IIR baseline coefficients for regular sensors */
#define UL_testing_Tuning_REGULAR_IIR_BL_N             (1u)
#define UL_testing_Tuning_REGULAR_IIR_BL_SHIFT         (8u)

/* IIR baseline filter algorithm for proximity sensors*/
#define UL_testing_Tuning_PROX_IIR_BL_TYPE             UL_testing_Tuning_IIR_FILTER_PERFORMANCE

/* IIR baseline coefficients for proximity sensors */
#define UL_testing_Tuning_PROX_IIR_BL_N                (1u)
#define UL_testing_Tuning_PROX_IIR_BL_SHIFT            (8u)


/* IIR filter constants */
#define UL_testing_Tuning_IIR_COEFFICIENT_K            (256u)

/* IIR filter type */
#define UL_testing_Tuning_IIR_FILTER_STANDARD          (1u)
#define UL_testing_Tuning_IIR_FILTER_PERFORMANCE       (2u)
#define UL_testing_Tuning_IIR_FILTER_MEMORY            (3u)

/* Regular sensor raw count filters */
#define UL_testing_Tuning_REGULAR_RC_FILTER_EN         (0u)
#define UL_testing_Tuning_REGULAR_RC_IIR_FILTER_EN     (0u)
#define UL_testing_Tuning_REGULAR_RC_MEDIAN_FILTER_EN  (0u)
#define UL_testing_Tuning_REGULAR_RC_AVERAGE_FILTER_EN (0u)
#define UL_testing_Tuning_REGULAR_RC_CUSTOM_FILTER_EN  (0u)
#define UL_testing_Tuning_REGULAR_RC_ALP_FILTER_EN     (0u)

/* Proximity sensor raw count filters */
#define UL_testing_Tuning_PROX_RC_FILTER_EN            (0u)
#define UL_testing_Tuning_PROX_RC_IIR_FILTER_EN        (0u)
#define UL_testing_Tuning_PROX_RC_MEDIAN_FILTER_EN     (0u)
#define UL_testing_Tuning_PROX_RC_AVERAGE_FILTER_EN    (0u)
#define UL_testing_Tuning_PROX_RC_CUSTOM_FILTER_EN     (0u)
#define UL_testing_Tuning_PROX_RC_ALP_FILTER_EN        (0u)

#define UL_testing_Tuning_ALP_FILTER_EN                (0u)
#define UL_testing_Tuning_REGULAR_RC_ALP_FILTER_COEFF  (2u)
#define UL_testing_Tuning_PROX_RC_ALP_FILTER_COEFF     (2u)

/* Raw count filters */
#define UL_testing_Tuning_RC_FILTER_EN                 (UL_testing_Tuning_REGULAR_RC_FILTER_EN || UL_testing_Tuning_PROX_RC_FILTER_EN)

/* IIR raw count filter algorithm for regular sensors */
#define UL_testing_Tuning_REGULAR_IIR_RC_TYPE          (UL_testing_Tuning_IIR_FILTER_STANDARD)

/* IIR raw count filter coefficients for regular sensors */
#define UL_testing_Tuning_REGULAR_IIR_RC_N             (128u)
#define UL_testing_Tuning_REGULAR_IIR_RC_SHIFT         (0u)

/* IIR raw count filter algorithm for proximity sensors*/
#define UL_testing_Tuning_PROX_IIR_RC_TYPE             (UL_testing_Tuning_IIR_FILTER_STANDARD)

/* IIR raw count filter coefficients for proximity sensors */
#define UL_testing_Tuning_PROX_IIR_RC_N                (128u)
#define UL_testing_Tuning_PROX_IIR_RC_SHIFT            (0u)

/* Median filter constants */

/* Order of regular sensor median filter */
#define UL_testing_Tuning_REGULAR_MEDIAN_LEN           (2u)

/* Order of proximity sensor median filter */
#define UL_testing_Tuning_PROX_MEDIAN_LEN              (2u)

/* Average filter constants*/
#define UL_testing_Tuning_AVERAGE_FILTER_LEN_2         (1u)
#define UL_testing_Tuning_AVERAGE_FILTER_LEN_4         (3u)

/* Order of regular sensor average filter */
#define UL_testing_Tuning_REGULAR_AVERAGE_LEN          (UL_testing_Tuning_AVERAGE_FILTER_LEN_4)

/* Order of proximity sensor average filter */
#define UL_testing_Tuning_PROX_AVERAGE_LEN             (UL_testing_Tuning_AVERAGE_FILTER_LEN_4)

/* Widget baseline coefficient enable */
#define UL_testing_Tuning_WD_BSLN_COEFF_EN             (0u)

/* Centroid position filters */
#define UL_testing_Tuning_POSITION_FILTER_EN           (0u)
#define UL_testing_Tuning_POS_MEDIAN_FILTER_EN         (0u)
#define UL_testing_Tuning_POS_IIR_FILTER_EN            (0u)
#define UL_testing_Tuning_POS_ADAPTIVE_IIR_FILTER_EN   (0u)
#define UL_testing_Tuning_POS_AVERAGE_FILTER_EN        (0u)
#define UL_testing_Tuning_POS_JITTER_FILTER_EN         (0u)
#define UL_testing_Tuning_BALLISTIC_MULTIPLIER_EN      (0u)
#define UL_testing_Tuning_CENTROID_3X3_CSD_EN          (0u)
#define UL_testing_Tuning_CENTROID_5X5_CSD_EN          (0u)
#define UL_testing_Tuning_CSD_5X5_MAX_FINGERS          (1u)

#define UL_testing_Tuning_POS_IIR_COEFF                (128u)
#define UL_testing_Tuning_POS_IIR_RESET_RADIAL_SLIDER  (35u)

#define UL_testing_Tuning_CSX_TOUCHPAD_UNDEFINED       (40u)

/* IDAC options */

/* Third-generation HW block IDAC gain */
#define UL_testing_Tuning_IDAC_GAIN_4X                 (4u)
#define UL_testing_Tuning_IDAC_GAIN_8X                 (8u)

/* Fourth-generation HW block IDAC gain */
#define UL_testing_Tuning_IDAC_GAIN_LOW                (0uL)
#define UL_testing_Tuning_IDAC_GAIN_MEDIUM             (1uL)
#define UL_testing_Tuning_IDAC_GAIN_HIGH               (2uL)

#define UL_testing_Tuning_IDAC_SOURCING                (0u)
#define UL_testing_Tuning_IDAC_SINKING                 (1u)

/* Shield tank capacitor precharge source */
#define UL_testing_Tuning_CSH_PRECHARGE_VREF           (0u)
#define UL_testing_Tuning_CSH_PRECHARGE_IO_BUF         (1u)

/* Shield electrode delay */
#define UL_testing_Tuning_NO_DELAY                     (0u)

#if(UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2)
    #define UL_testing_Tuning_SH_DELAY_5NS             (1u)
    #define UL_testing_Tuning_SH_DELAY_10NS            (2u)
    #define UL_testing_Tuning_SH_DELAY_20NS            (3u)
#else
    #if(UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV1_VER2)
        #define UL_testing_Tuning_SH_DELAY_10NS        (3u)
        #define UL_testing_Tuning_SH_DELAY_50NS        (2u)
    #else
        #define UL_testing_Tuning_SH_DELAY_1CYCLES     (1u)
        #define UL_testing_Tuning_SH_DELAY_2CYCLES     (2u)
    #endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV1_VER2) */
#endif /* (UL_testing_Tuning_ENABLE == UL_testing_Tuning_CSDV2) */

/* Inactive sensor connection options */
#define UL_testing_Tuning_SNS_CONNECTION_GROUND        (0x00000006Lu)
#define UL_testing_Tuning_SNS_CONNECTION_HIGHZ         (0x00000000Lu)
#define UL_testing_Tuning_SNS_CONNECTION_SHIELD        (0x00000002Lu)

/* Sense clock selection options */
#if defined(UL_testing_Tuning_TAPEOUT_STAR_USED)
    #define UL_testing_Tuning_CSDV2_REF9P6UA_EN            (0u)
#else
    #define UL_testing_Tuning_CSDV2_REF9P6UA_EN            (1u)
#endif /* defined(UL_testing_Tuning_TAPEOUT_STAR_USED) */

#define UL_testing_Tuning_CLK_SOURCE_DIRECT            (0x00000000Lu)

#define UL_testing_Tuning_CLK_SOURCE_SSC1              (0x01u)
#define UL_testing_Tuning_CLK_SOURCE_SSC2              (0x02u)
#define UL_testing_Tuning_CLK_SOURCE_SSC3              (0x03u)
#define UL_testing_Tuning_CLK_SOURCE_SSC4              (0x04u)

#define UL_testing_Tuning_CLK_SOURCE_PRS8              (0x05u)
#define UL_testing_Tuning_CLK_SOURCE_PRS12             (0x06u)
#define UL_testing_Tuning_CLK_SOURCE_PRSAUTO           (0xFFu)

#define UL_testing_Tuning_MFS_IMO                      (0u)
#define UL_testing_Tuning_MFS_SNS_CLK                  (1u)

/* Defines scan resolutions */
#define UL_testing_Tuning_RES6BIT                      (6u)
#define UL_testing_Tuning_RES7BIT                      (7u)
#define UL_testing_Tuning_RES8BIT                      (8u)
#define UL_testing_Tuning_RES9BIT                      (9u)
#define UL_testing_Tuning_RES10BIT                     (10u)
#define UL_testing_Tuning_RES11BIT                     (11u)
#define UL_testing_Tuning_RES12BIT                     (12u)
#define UL_testing_Tuning_RES13BIT                     (13u)
#define UL_testing_Tuning_RES14BIT                     (14u)
#define UL_testing_Tuning_RES15BIT                     (15u)
#define UL_testing_Tuning_RES16BIT                     (16u)

/* Fourth-generation HW block: Initialization switch resistance */
#define UL_testing_Tuning_INIT_SW_RES_LOW              (0x00000000Lu)
#define UL_testing_Tuning_INIT_SW_RES_MEDIUM           (0x00000001Lu)
#define UL_testing_Tuning_INIT_SW_RES_HIGH             (0x00000002Lu)

/* Fourth-generation HW block: Initialization switch resistance */
#define UL_testing_Tuning_SCAN_SW_RES_LOW              (0x00000000Lu)
#define UL_testing_Tuning_SCAN_SW_RES_MEDIUM           (0x00000001Lu)
#define UL_testing_Tuning_SCAN_SW_RES_HIGH             (0x00000002Lu)

/* Fourth-generation HW block: CSD shield switch resistance */
#define UL_testing_Tuning_SHIELD_SW_RES_LOW            (0x00000000Lu)
#define UL_testing_Tuning_SHIELD_SW_RES_MEDIUM         (0x00000001Lu)
#define UL_testing_Tuning_SHIELD_SW_RES_HIGH           (0x00000002Lu)
#define UL_testing_Tuning_SHIELD_SW_RES_LOW_EMI        (0x00000003Lu)

/* Fourth-generation HW block: CSD shield switch resistance */
#define UL_testing_Tuning_INIT_SHIELD_SW_RES_LOW       (0x00000000Lu)
#define UL_testing_Tuning_INIT_SHIELD_SW_RES_MEDIUM    (0x00000001Lu)
#define UL_testing_Tuning_INIT_SHIELD_SW_RES_HIGH      (0x00000002Lu)
#define UL_testing_Tuning_INIT_SHIELD_SW_RES_LOW_EMI   (0x00000003Lu)

/* Fourth-generation HW block: CSD shield switch resistance */
#define UL_testing_Tuning_SCAN_SHIELD_SW_RES_LOW       (0x00000000Lu)
#define UL_testing_Tuning_SCAN_SHIELD_SW_RES_MEDIUM    (0x00000001Lu)
#define UL_testing_Tuning_SCAN_SHIELD_SW_RES_HIGH      (0x00000002Lu)
#define UL_testing_Tuning_SCAN_SHIELD_SW_RES_LOW_EMI   (0x00000003Lu)

/* Sensing method */
#define UL_testing_Tuning_SENSING_LEGACY               (0x00000000Lu)
#define UL_testing_Tuning_SENSING_LOW_EMI              (0x00000001Lu)
#define UL_testing_Tuning_SENSING_FULL_WAVE            (0x00000002Lu)


/*******************************************************************************
* CSD/CSX Common settings
*******************************************************************************/

#define UL_testing_Tuning_BLOCK_ANALOG_WAKEUP_DELAY_US (0u)

#define UL_testing_Tuning_MFS_METHOD                   (0u)
#define UL_testing_Tuning_IMO_FREQUENCY_OFFSET_F1      (20u)
#define UL_testing_Tuning_IMO_FREQUENCY_OFFSET_F2      (20u)

/*******************************************************************************
* CSD Specific settings
*******************************************************************************/

/* CSD scan method settings */
#define UL_testing_Tuning_CSD_IDAC_AUTOCAL_EN          (1u)
#define UL_testing_Tuning_CSD_IDAC_GAIN                (UL_testing_Tuning_IDAC_GAIN_HIGH)
#define UL_testing_Tuning_CSD_SHIELD_EN                (1u)
#define UL_testing_Tuning_CSD_SHIELD_TANK_EN           (1u)
#define UL_testing_Tuning_CSD_CSH_PRECHARGE_SRC        (UL_testing_Tuning_CSH_PRECHARGE_VREF)
#define UL_testing_Tuning_CSD_SHIELD_DELAY             (UL_testing_Tuning_NO_DELAY)
#define UL_testing_Tuning_CSD_TOTAL_SHIELD_COUNT       (1u)
#define UL_testing_Tuning_CSD_SCANSPEED_DIVIDER        (1u)
#define UL_testing_Tuning_CSD_COMMON_SNS_CLK_EN        (0u)
#define UL_testing_Tuning_CSD_SNS_CLK_SOURCE           (UL_testing_Tuning_CLK_SOURCE_PRSAUTO)
#define UL_testing_Tuning_CSD_SNS_CLK_DIVIDER          (8u)
#define UL_testing_Tuning_CSD_INACTIVE_SNS_CONNECTION  (UL_testing_Tuning_SNS_CONNECTION_SHIELD)
#define UL_testing_Tuning_CSD_IDAC_COMP_EN             (1u)
#define UL_testing_Tuning_CSD_IDAC_CONFIG              (UL_testing_Tuning_IDAC_SOURCING)
#define UL_testing_Tuning_CSD_RAWCOUNT_CAL_LEVEL       (70u)
#define UL_testing_Tuning_CSD_DUALIDAC_LEVEL           (50u)
#define UL_testing_Tuning_CSD_PRESCAN_SETTLING_TIME    (5u)
#define UL_testing_Tuning_CSD_SNSCLK_R_CONST           (1000u)
#define UL_testing_Tuning_CSD_VREF_MV                  (2743u)

/* CSD settings - Fourth-generation HW block */
#define UL_testing_Tuning_CSD_ANALOG_STARTUP_DELAY_US  (10u)
#define UL_testing_Tuning_CSD_FINE_INIT_TIME           (10u)
#define UL_testing_Tuning_CSD_DEDICATED_IDAC_COMP_EN   (1u)
#define UL_testing_Tuning_CSD_AUTO_ZERO_EN             (0u)
#define UL_testing_Tuning_CSD_AUTO_ZERO_TIME           (15Lu)
#define UL_testing_Tuning_CSD_NOISE_METRIC_EN          (0u)
#define UL_testing_Tuning_CSD_NOISE_METRIC_TH          (1Lu)
#define UL_testing_Tuning_CSD_INIT_SWITCH_RES          (UL_testing_Tuning_INIT_SW_RES_MEDIUM)
#define UL_testing_Tuning_CSD_SENSING_METHOD           (0)
#define UL_testing_Tuning_CSD_SHIELD_SWITCH_RES        (UL_testing_Tuning_SHIELD_SW_RES_MEDIUM)
#define UL_testing_Tuning_CSD_GAIN                     (13Lu)

#define UL_testing_Tuning_CSD_MFS_METHOD               (u)
#define UL_testing_Tuning_CSD_MFS_DIVIDER_OFFSET_F1    (1u)
#define UL_testing_Tuning_CSD_MFS_DIVIDER_OFFSET_F2    (2u)

/*******************************************************************************
* CSX Specific settings
*******************************************************************************/

/* CSX scan method settings */
#define UL_testing_Tuning_CSX_SCANSPEED_DIVIDER        (1u)
#define UL_testing_Tuning_CSX_COMMON_TX_CLK_EN         (0u)
#define UL_testing_Tuning_CSX_TX_CLK_SOURCE            (UL_testing_Tuning_CLK_SOURCE_PRSAUTO)
#define UL_testing_Tuning_CSX_TX_CLK_DIVIDER           (80u)
#define UL_testing_Tuning_CSX_MAX_FINGERS              (1u)
#define UL_testing_Tuning_CSX_MAX_LOCAL_PEAKS          (5u)
#define UL_testing_Tuning_CSX_IDAC_AUTOCAL_EN          (0u)
#define UL_testing_Tuning_CSX_IDAC_BITS_USED           (7u)
#define UL_testing_Tuning_CSX_RAWCOUNT_CAL_LEVEL       (40u)
#define UL_testing_Tuning_CSX_IDAC_GAIN                (UL_testing_Tuning_IDAC_GAIN_MEDIUM)
#define UL_testing_Tuning_CSX_SKIP_OVSMPL_SPECIFIC_NODES (0u)
#define UL_testing_Tuning_CSX_MULTIPHASE_TX_EN         (0u)
#define UL_testing_Tuning_CSX_MAX_TX_PHASE_LENGTH      (0u)

/* CSX settings - Fourth-generation HW block */
#define UL_testing_Tuning_CSX_ANALOG_STARTUP_DELAY_US  (10u)
#define UL_testing_Tuning_CSX_AUTO_ZERO_EN             (0u)
#define UL_testing_Tuning_CSX_AUTO_ZERO_TIME           (15u)
#define UL_testing_Tuning_CSX_FINE_INIT_TIME           (4u)
#define UL_testing_Tuning_CSX_NOISE_METRIC_EN          (0u)
#define UL_testing_Tuning_CSX_NOISE_METRIC_TH          (1u)
#define UL_testing_Tuning_CSX_INIT_SWITCH_RES          (UL_testing_Tuning_INIT_SW_RES_MEDIUM)
#define UL_testing_Tuning_CSX_SCAN_SWITCH_RES          (UL_testing_Tuning_SCAN_SW_RES_LOW)
#define UL_testing_Tuning_CSX_INIT_SHIELD_SWITCH_RES   (UL_testing_Tuning_INIT_SHIELD_SW_RES_HIGH)
#define UL_testing_Tuning_CSX_SCAN_SHIELD_SWITCH_RES   (UL_testing_Tuning_SCAN_SHIELD_SW_RES_LOW)

#define UL_testing_Tuning_CSX_MFS_METHOD               (u)
#define UL_testing_Tuning_CSX_MFS_DIVIDER_OFFSET_F1    (1u)
#define UL_testing_Tuning_CSX_MFS_DIVIDER_OFFSET_F2    (2u)

/* Gesture parameters */
#define UL_testing_Tuning_GES_GLOBAL_EN                (0u)

/*******************************************************************************
* ISX Specific settings
*******************************************************************************/

/* ISX scan method settings */
#define UL_testing_Tuning_ISX_SCANSPEED_DIVIDER        (1u)
#define UL_testing_Tuning_ISX_LX_CLK_DIVIDER           (80u)
#define UL_testing_Tuning_ISX_IDAC_AUTOCAL_EN          (0u)
#define UL_testing_Tuning_ISX_IDAC_BITS_USED           (7u)
#define UL_testing_Tuning_ISX_RAWCOUNT_CAL_LEVEL       (30u)
#define UL_testing_Tuning_ISX_IDAC_GAIN                (UL_testing_Tuning_IDAC_GAIN_MEDIUM)
#define UL_testing_Tuning_ISX_SKIP_OVSMPL_SPECIFIC_NODES (0u)
#define UL_testing_Tuning_ISX_MAX_TX_PHASE_LENGTH      (0u)
#define UL_testing_Tuning_ISX_PIN_COUNT_LX             (u)
/* ISX settings - Fourth-generation HW block */
#define UL_testing_Tuning_ISX_AUTO_ZERO_EN             (0u)
#define UL_testing_Tuning_ISX_AUTO_ZERO_TIME           (15u)
#define UL_testing_Tuning_ISX_FINE_INIT_TIME           (20u)
#define UL_testing_Tuning_ISX_NOISE_METRIC_EN          (0u)
#define UL_testing_Tuning_ISX_NOISE_METRIC_TH          (1u)
#define UL_testing_Tuning_ISX_INIT_SWITCH_RES          (UL_testing_Tuning_INIT_SW_RES_MEDIUM)
#define UL_testing_Tuning_ISX_SCAN_SWITCH_RES          (UL_testing_Tuning_SCAN_SW_RES_LOW)
#define UL_testing_Tuning_ISX_INIT_SHIELD_SWITCH_RES   (UL_testing_Tuning_INIT_SHIELD_SW_RES_HIGH)
#define UL_testing_Tuning_ISX_SCAN_SHIELD_SWITCH_RES   (UL_testing_Tuning_SCAN_SHIELD_SW_RES_LOW)
#define UL_testing_Tuning_ISX_SAMPLE_PHASE_DEG         (30u)

/*******************************************************************************
* Global Parameter Definitions
*******************************************************************************/

/* Compound section definitions */
#define UL_testing_Tuning_ANY_NONSS_AUTOCAL ((0u != UL_testing_Tuning_CSX_IDAC_AUTOCAL_EN) || \
                                       (0u != UL_testing_Tuning_ISX_IDAC_AUTOCAL_EN) || \
                                      ((UL_testing_Tuning_CSD_AUTOTUNE == UL_testing_Tuning_CSD_SS_DIS) && (0u != UL_testing_Tuning_CSD_IDAC_AUTOCAL_EN)))
#define UL_testing_Tuning_ANYMODE_AUTOCAL (((0u != UL_testing_Tuning_CSX_IDAC_AUTOCAL_EN) \
                                       || (0u != UL_testing_Tuning_ISX_IDAC_AUTOCAL_EN)) \
                                       || (0u != UL_testing_Tuning_CSD_IDAC_AUTOCAL_EN))
/* RAM Global Parameters Definitions */
#define UL_testing_Tuning_CONFIG_ID             (0x2384u)
#define UL_testing_Tuning_DEVICE_ID             (0x0100u)
#define UL_testing_Tuning_HW_CLOCK              (0x0BB8u)
#define UL_testing_Tuning_CSD0_CONFIG           (0x0108u)
#define UL_testing_Tuning_GLB_CRC               (0x56D4u)

/*******************************************************************************
* Button0 initialization values for FLASH data structure
*******************************************************************************/
#define UL_testing_Tuning_BUTTON0_STATIC_CONFIG (10241u)
#define UL_testing_Tuning_BUTTON0_NUM_SENSORS   (1u)

/*******************************************************************************
* Button0 initialization values for RAM data structure
*******************************************************************************/
#define UL_testing_Tuning_BUTTON0_CRC           (0x1818u)
#define UL_testing_Tuning_BUTTON0_RESOLUTION    (UL_testing_Tuning_RES13BIT)
#define UL_testing_Tuning_BUTTON0_FINGER_TH     (1800u)
#define UL_testing_Tuning_BUTTON0_NOISE_TH      (30u)
#define UL_testing_Tuning_BUTTON0_NNOISE_TH     (30u)
#define UL_testing_Tuning_BUTTON0_HYSTERESIS    (200u)
#define UL_testing_Tuning_BUTTON0_ON_DEBOUNCE   (3u)
#define UL_testing_Tuning_BUTTON0_LOW_BSLN_RST  (30u)
#define UL_testing_Tuning_BUTTON0_IDAC_MOD0     (32u)
#define UL_testing_Tuning_BUTTON0_SNS_CLK       (8u)
#define UL_testing_Tuning_BUTTON0_SNS_CLK_SOURCE (0u)

/*******************************************************************************
* Button1 initialization values for FLASH data structure
*******************************************************************************/
#define UL_testing_Tuning_BUTTON1_STATIC_CONFIG (10241u)
#define UL_testing_Tuning_BUTTON1_NUM_SENSORS   (1u)

/*******************************************************************************
* Button1 initialization values for RAM data structure
*******************************************************************************/
#define UL_testing_Tuning_BUTTON1_CRC           (0x1E28u)
#define UL_testing_Tuning_BUTTON1_RESOLUTION    (UL_testing_Tuning_RES13BIT)
#define UL_testing_Tuning_BUTTON1_FINGER_TH     (1800u)
#define UL_testing_Tuning_BUTTON1_NOISE_TH      (40u)
#define UL_testing_Tuning_BUTTON1_NNOISE_TH     (40u)
#define UL_testing_Tuning_BUTTON1_HYSTERESIS    (200u)
#define UL_testing_Tuning_BUTTON1_ON_DEBOUNCE   (3u)
#define UL_testing_Tuning_BUTTON1_LOW_BSLN_RST  (30u)
#define UL_testing_Tuning_BUTTON1_IDAC_MOD0     (32u)
#define UL_testing_Tuning_BUTTON1_SNS_CLK       (8u)
#define UL_testing_Tuning_BUTTON1_SNS_CLK_SOURCE (0u)

/* RAM Sensor Parameters Definitions */
#define UL_testing_Tuning_BUTTON0_SNS0_IDAC_COMP0 (32u)
#define UL_testing_Tuning_BUTTON1_SNS0_IDAC_COMP0 (32u)



/*******************************************************************************
* ADC Specific Macros
*******************************************************************************/
#define UL_testing_Tuning_ADC_RES8BIT                  (8u)
#define UL_testing_Tuning_ADC_RES10BIT                 (10u)

#define UL_testing_Tuning_ADC_FULLRANGE_MODE           (0u)
#define UL_testing_Tuning_ADC_VREF_MODE                (1u)

#define UL_testing_Tuning_ADC_MIN_CHANNELS             (1u)
#define UL_testing_Tuning_ADC_EN                       (0u)
#define UL_testing_Tuning_ADC_STANDALONE_EN            (0u)
#define UL_testing_Tuning_ADC_TOTAL_CHANNELS           (1u)
#define UL_testing_Tuning_ADC_RESOLUTION               (UL_testing_Tuning_ADC_RES10BIT)
#define UL_testing_Tuning_ADC_AMUXB_INPUT_EN           (0u)
#define UL_testing_Tuning_ADC_SELECT_AMUXB_CH          (0u)
#define UL_testing_Tuning_ADC_AZ_EN                    (1Lu)
#define UL_testing_Tuning_ADC_AZ_TIME                  (5u)
#define UL_testing_Tuning_ADC_VREF_MV                  (3840u)
#define UL_testing_Tuning_ADC_GAIN                     (9Lu)
#define UL_testing_Tuning_ADC_IDAC_DEFAULT             (27u)
#define UL_testing_Tuning_ADC_MODCLK_DIV_DEFAULT       (1u)
#define UL_testing_Tuning_ADC_MEASURE_MODE             (UL_testing_Tuning_ADC_FULLRANGE_MODE)
#define UL_testing_Tuning_ADC_ANALOG_STARTUP_DELAY_US  (5u)
#define UL_testing_Tuning_ADC_ACQUISITION_TIME_US      (10u)

/*******************************************************************************
* Built-In Self-Test Configuration
*******************************************************************************/
#define UL_testing_Tuning_SELF_TEST_EN                   (1Lu)
#define UL_testing_Tuning_TST_GLOBAL_CRC_EN              (1Lu)
#define UL_testing_Tuning_TST_WDGT_CRC_EN                (1Lu)
#define UL_testing_Tuning_TST_BSLN_DUPLICATION_EN        (1Lu)
#define UL_testing_Tuning_TST_BSLN_RAW_OUT_RANGE_EN      (1Lu)
#define UL_testing_Tuning_TST_SNS_SHORT_EN               (1Lu)
#define UL_testing_Tuning_TST_SNS2SNS_SHORT_EN           (1Lu)
#define UL_testing_Tuning_TST_SNS_CAP_EN                 (1Lu)
#define UL_testing_Tuning_TST_SH_CAP_EN                  (1Lu)
#define UL_testing_Tuning_TST_EXTERNAL_CAP_EN            (1Lu)
#define UL_testing_Tuning_TST_INTERNAL_CAP_EN            (0Lu)
#define UL_testing_Tuning_TST_VDDA_EN                    (1Lu)
#define UL_testing_Tuning_TST_FINE_INIT_TIME             (10Lu)

#define UL_testing_Tuning_TST_GLOBAL_CRC (1Lu << 0u)
#define UL_testing_Tuning_TST_WDGT_CRC (1Lu << 1u)
#define UL_testing_Tuning_TST_BSLN_DUPLICATION (1Lu << 4u)
#define UL_testing_Tuning_TST_BSLN_RAW_OUT_RANGE (1Lu << 5u)
#define UL_testing_Tuning_TST_SNS_SHORT (1Lu << 8u)
#define UL_testing_Tuning_TST_SNS2SNS_SHORT (1Lu << 9u)
#define UL_testing_Tuning_TST_SNS_CAP  (1Lu << 12u)
#define UL_testing_Tuning_TST_SH_CAP   (1Lu << 13u)
#define UL_testing_Tuning_TST_EXTERNAL_CAP (1Lu << 16u)
#define UL_testing_Tuning_TST_VDDA     (1Lu << 20u)
#define UL_testing_Tuning_TST_RUN_SELF_TEST_MASK (UL_testing_Tuning_TST_GLOBAL_CRC | \
                                        UL_testing_Tuning_TST_WDGT_CRC | \
                                        UL_testing_Tuning_TST_BSLN_DUPLICATION | \
                                        UL_testing_Tuning_TST_SNS_SHORT | \
                                        UL_testing_Tuning_TST_SNS2SNS_SHORT | \
                                        UL_testing_Tuning_TST_SNS_CAP | \
                                        UL_testing_Tuning_TST_SH_CAP | \
                                        UL_testing_Tuning_TST_EXTERNAL_CAP | \
                                        UL_testing_Tuning_TST_VDDA)
#define UL_testing_Tuning_TST_RUN_SELF_TEST_START_MASK (UL_testing_Tuning_TST_GLOBAL_CRC | \
                                        UL_testing_Tuning_TST_WDGT_CRC | \
                                        UL_testing_Tuning_TST_SNS_SHORT | \
                                        UL_testing_Tuning_TST_SNS2SNS_SHORT)

#define UL_testing_Tuning_GLOBAL_CRC_AREA_START (30u)
#define UL_testing_Tuning_GLOBAL_CRC_AREA_SIZE (4u)
#define UL_testing_Tuning_WIDGET_CRC_AREA_START (2u)
#define UL_testing_Tuning_WIDGET_CRC_AREA_SIZE (14u)


#define UL_testing_Tuning_TST_CMOD_ID  (0u)
#define UL_testing_Tuning_TST_CSH_ID   (1u)
#define UL_testing_Tuning_TST_EXT_CAPS_NUM (2u)

/* The divider that provides highest possible modulator clock frequency */
#define UL_testing_Tuning_TST_MOD_CLK_DIVIDER (1u)
/* The highest possible sense clock frequency to satisfy equation:
 * SnsClkFreq < 1 / (10 * R * 500pF) */
#define UL_testing_Tuning_TST_BASE_SNS_CLK_KHZ (187u)
/* The sense clock divider for UL_testing_Tuning_TST_BASE_SNS_CLK_KHZ */
#define UL_testing_Tuning_TST_BASE_SNS_CLK_DIVIDER (128u)
/* The minimum valid sense clock divider */
#define UL_testing_Tuning_TST_MIN_SNS_CLK_DIVIDER (4u)
/* VDDA measurement test configuration */
#define UL_testing_Tuning_TST_VDDA_VREF_MV (3840u)
#define UL_testing_Tuning_TST_VDDA_VREF_GAIN (9u)
#define UL_testing_Tuning_TST_VDDA_IDAC_DEFAULT (27u)

#define UL_testing_Tuning_TST_ANALOG_STARTUP_DELAY_US    (23u)

/*******************************************************************************
* Gesture Configuration
*******************************************************************************/
#define UL_testing_Tuning_TIMESTAMP_INTERVAL             (1Lu)
#define UL_testing_Tuning_GESTURE_EN_WIDGET_ID           (0Lu)
#define UL_testing_Tuning_BALLISTIC_EN_WIDGET_ID         (0Lu)


#endif /* CY_SENSE_UL_testing_Tuning_CONFIGURATION_H */


/* [] END OF FILE */
