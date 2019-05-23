/*******************************************************************************
* File Name: Buzzer_Volume.h
* Version 2.10
*
* Description:
*  This file provides constants and parameter values for the Buzzer_Volume
*  component.
*
* Note:
*  None
*
********************************************************************************
* Copyright 2013-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_TCPWM_Buzzer_Volume_H)
#define CY_TCPWM_Buzzer_Volume_H


#include "CyLib.h"
#include "cytypes.h"
#include "cyfitter.h"


/*******************************************************************************
* Internal Type defines
*******************************************************************************/

/* Structure to save state before go to sleep */
typedef struct
{
    uint8  enableState;
} Buzzer_Volume_BACKUP_STRUCT;


/*******************************************************************************
* Variables
*******************************************************************************/
extern uint8  Buzzer_Volume_initVar;


/***************************************
*   Conditional Compilation Parameters
****************************************/

#define Buzzer_Volume_CY_TCPWM_V2                    (CYIPBLOCK_m0s8tcpwm_VERSION == 2u)
#define Buzzer_Volume_CY_TCPWM_4000                  (CY_PSOC4_4000)

/* TCPWM Configuration */
#define Buzzer_Volume_CONFIG                         (7lu)

/* Quad Mode */
/* Parameters */
#define Buzzer_Volume_QUAD_ENCODING_MODES            (0lu)
#define Buzzer_Volume_QUAD_AUTO_START                (1lu)

/* Signal modes */
#define Buzzer_Volume_QUAD_INDEX_SIGNAL_MODE         (0lu)
#define Buzzer_Volume_QUAD_PHIA_SIGNAL_MODE          (3lu)
#define Buzzer_Volume_QUAD_PHIB_SIGNAL_MODE          (3lu)
#define Buzzer_Volume_QUAD_STOP_SIGNAL_MODE          (0lu)

/* Signal present */
#define Buzzer_Volume_QUAD_INDEX_SIGNAL_PRESENT      (0lu)
#define Buzzer_Volume_QUAD_STOP_SIGNAL_PRESENT       (0lu)

/* Interrupt Mask */
#define Buzzer_Volume_QUAD_INTERRUPT_MASK            (1lu)

/* Timer/Counter Mode */
/* Parameters */
#define Buzzer_Volume_TC_RUN_MODE                    (0lu)
#define Buzzer_Volume_TC_COUNTER_MODE                (0lu)
#define Buzzer_Volume_TC_COMP_CAP_MODE               (2lu)
#define Buzzer_Volume_TC_PRESCALER                   (0lu)

/* Signal modes */
#define Buzzer_Volume_TC_RELOAD_SIGNAL_MODE          (0lu)
#define Buzzer_Volume_TC_COUNT_SIGNAL_MODE           (3lu)
#define Buzzer_Volume_TC_START_SIGNAL_MODE           (0lu)
#define Buzzer_Volume_TC_STOP_SIGNAL_MODE            (0lu)
#define Buzzer_Volume_TC_CAPTURE_SIGNAL_MODE         (0lu)

/* Signal present */
#define Buzzer_Volume_TC_RELOAD_SIGNAL_PRESENT       (0lu)
#define Buzzer_Volume_TC_COUNT_SIGNAL_PRESENT        (0lu)
#define Buzzer_Volume_TC_START_SIGNAL_PRESENT        (0lu)
#define Buzzer_Volume_TC_STOP_SIGNAL_PRESENT         (0lu)
#define Buzzer_Volume_TC_CAPTURE_SIGNAL_PRESENT      (0lu)

/* Interrupt Mask */
#define Buzzer_Volume_TC_INTERRUPT_MASK              (1lu)

/* PWM Mode */
/* Parameters */
#define Buzzer_Volume_PWM_KILL_EVENT                 (0lu)
#define Buzzer_Volume_PWM_STOP_EVENT                 (0lu)
#define Buzzer_Volume_PWM_MODE                       (4lu)
#define Buzzer_Volume_PWM_OUT_N_INVERT               (0lu)
#define Buzzer_Volume_PWM_OUT_INVERT                 (0lu)
#define Buzzer_Volume_PWM_ALIGN                      (0lu)
#define Buzzer_Volume_PWM_RUN_MODE                   (0lu)
#define Buzzer_Volume_PWM_DEAD_TIME_CYCLE            (0lu)
#define Buzzer_Volume_PWM_PRESCALER                  (0lu)

/* Signal modes */
#define Buzzer_Volume_PWM_RELOAD_SIGNAL_MODE         (0lu)
#define Buzzer_Volume_PWM_COUNT_SIGNAL_MODE          (3lu)
#define Buzzer_Volume_PWM_START_SIGNAL_MODE          (0lu)
#define Buzzer_Volume_PWM_STOP_SIGNAL_MODE           (0lu)
#define Buzzer_Volume_PWM_SWITCH_SIGNAL_MODE         (0lu)

/* Signal present */
#define Buzzer_Volume_PWM_RELOAD_SIGNAL_PRESENT      (0lu)
#define Buzzer_Volume_PWM_COUNT_SIGNAL_PRESENT       (0lu)
#define Buzzer_Volume_PWM_START_SIGNAL_PRESENT       (0lu)
#define Buzzer_Volume_PWM_STOP_SIGNAL_PRESENT        (0lu)
#define Buzzer_Volume_PWM_SWITCH_SIGNAL_PRESENT      (0lu)

/* Interrupt Mask */
#define Buzzer_Volume_PWM_INTERRUPT_MASK             (1lu)


/***************************************
*    Initial Parameter Constants
***************************************/

/* Timer/Counter Mode */
#define Buzzer_Volume_TC_PERIOD_VALUE                (65535lu)
#define Buzzer_Volume_TC_COMPARE_VALUE               (65535lu)
#define Buzzer_Volume_TC_COMPARE_BUF_VALUE           (65535lu)
#define Buzzer_Volume_TC_COMPARE_SWAP                (0lu)

/* PWM Mode */
#define Buzzer_Volume_PWM_PERIOD_VALUE               (20lu)
#define Buzzer_Volume_PWM_PERIOD_BUF_VALUE           (65535lu)
#define Buzzer_Volume_PWM_PERIOD_SWAP                (0lu)
#define Buzzer_Volume_PWM_COMPARE_VALUE              (7lu)
#define Buzzer_Volume_PWM_COMPARE_BUF_VALUE          (65535lu)
#define Buzzer_Volume_PWM_COMPARE_SWAP               (0lu)


/***************************************
*    Enumerated Types and Parameters
***************************************/

#define Buzzer_Volume__LEFT 0
#define Buzzer_Volume__RIGHT 1
#define Buzzer_Volume__CENTER 2
#define Buzzer_Volume__ASYMMETRIC 3

#define Buzzer_Volume__X1 0
#define Buzzer_Volume__X2 1
#define Buzzer_Volume__X4 2

#define Buzzer_Volume__PWM 4
#define Buzzer_Volume__PWM_DT 5
#define Buzzer_Volume__PWM_PR 6

#define Buzzer_Volume__INVERSE 1
#define Buzzer_Volume__DIRECT 0

#define Buzzer_Volume__CAPTURE 2
#define Buzzer_Volume__COMPARE 0

#define Buzzer_Volume__TRIG_LEVEL 3
#define Buzzer_Volume__TRIG_RISING 0
#define Buzzer_Volume__TRIG_FALLING 1
#define Buzzer_Volume__TRIG_BOTH 2

#define Buzzer_Volume__INTR_MASK_TC 1
#define Buzzer_Volume__INTR_MASK_CC_MATCH 2
#define Buzzer_Volume__INTR_MASK_NONE 0
#define Buzzer_Volume__INTR_MASK_TC_CC 3

#define Buzzer_Volume__UNCONFIG 8
#define Buzzer_Volume__TIMER 1
#define Buzzer_Volume__QUAD 3
#define Buzzer_Volume__PWM_SEL 7

#define Buzzer_Volume__COUNT_UP 0
#define Buzzer_Volume__COUNT_DOWN 1
#define Buzzer_Volume__COUNT_UPDOWN0 2
#define Buzzer_Volume__COUNT_UPDOWN1 3


/* Prescaler */
#define Buzzer_Volume_PRESCALE_DIVBY1                ((uint32)(0u << Buzzer_Volume_PRESCALER_SHIFT))
#define Buzzer_Volume_PRESCALE_DIVBY2                ((uint32)(1u << Buzzer_Volume_PRESCALER_SHIFT))
#define Buzzer_Volume_PRESCALE_DIVBY4                ((uint32)(2u << Buzzer_Volume_PRESCALER_SHIFT))
#define Buzzer_Volume_PRESCALE_DIVBY8                ((uint32)(3u << Buzzer_Volume_PRESCALER_SHIFT))
#define Buzzer_Volume_PRESCALE_DIVBY16               ((uint32)(4u << Buzzer_Volume_PRESCALER_SHIFT))
#define Buzzer_Volume_PRESCALE_DIVBY32               ((uint32)(5u << Buzzer_Volume_PRESCALER_SHIFT))
#define Buzzer_Volume_PRESCALE_DIVBY64               ((uint32)(6u << Buzzer_Volume_PRESCALER_SHIFT))
#define Buzzer_Volume_PRESCALE_DIVBY128              ((uint32)(7u << Buzzer_Volume_PRESCALER_SHIFT))

/* TCPWM set modes */
#define Buzzer_Volume_MODE_TIMER_COMPARE             ((uint32)(Buzzer_Volume__COMPARE         <<  \
                                                                  Buzzer_Volume_MODE_SHIFT))
#define Buzzer_Volume_MODE_TIMER_CAPTURE             ((uint32)(Buzzer_Volume__CAPTURE         <<  \
                                                                  Buzzer_Volume_MODE_SHIFT))
#define Buzzer_Volume_MODE_QUAD                      ((uint32)(Buzzer_Volume__QUAD            <<  \
                                                                  Buzzer_Volume_MODE_SHIFT))
#define Buzzer_Volume_MODE_PWM                       ((uint32)(Buzzer_Volume__PWM             <<  \
                                                                  Buzzer_Volume_MODE_SHIFT))
#define Buzzer_Volume_MODE_PWM_DT                    ((uint32)(Buzzer_Volume__PWM_DT          <<  \
                                                                  Buzzer_Volume_MODE_SHIFT))
#define Buzzer_Volume_MODE_PWM_PR                    ((uint32)(Buzzer_Volume__PWM_PR          <<  \
                                                                  Buzzer_Volume_MODE_SHIFT))

/* Quad Modes */
#define Buzzer_Volume_MODE_X1                        ((uint32)(Buzzer_Volume__X1              <<  \
                                                                  Buzzer_Volume_QUAD_MODE_SHIFT))
#define Buzzer_Volume_MODE_X2                        ((uint32)(Buzzer_Volume__X2              <<  \
                                                                  Buzzer_Volume_QUAD_MODE_SHIFT))
#define Buzzer_Volume_MODE_X4                        ((uint32)(Buzzer_Volume__X4              <<  \
                                                                  Buzzer_Volume_QUAD_MODE_SHIFT))

/* Counter modes */
#define Buzzer_Volume_COUNT_UP                       ((uint32)(Buzzer_Volume__COUNT_UP        <<  \
                                                                  Buzzer_Volume_UPDOWN_SHIFT))
#define Buzzer_Volume_COUNT_DOWN                     ((uint32)(Buzzer_Volume__COUNT_DOWN      <<  \
                                                                  Buzzer_Volume_UPDOWN_SHIFT))
#define Buzzer_Volume_COUNT_UPDOWN0                  ((uint32)(Buzzer_Volume__COUNT_UPDOWN0   <<  \
                                                                  Buzzer_Volume_UPDOWN_SHIFT))
#define Buzzer_Volume_COUNT_UPDOWN1                  ((uint32)(Buzzer_Volume__COUNT_UPDOWN1   <<  \
                                                                  Buzzer_Volume_UPDOWN_SHIFT))

/* PWM output invert */
#define Buzzer_Volume_INVERT_LINE                    ((uint32)(Buzzer_Volume__INVERSE         <<  \
                                                                  Buzzer_Volume_INV_OUT_SHIFT))
#define Buzzer_Volume_INVERT_LINE_N                  ((uint32)(Buzzer_Volume__INVERSE         <<  \
                                                                  Buzzer_Volume_INV_COMPL_OUT_SHIFT))

/* Trigger modes */
#define Buzzer_Volume_TRIG_RISING                    ((uint32)Buzzer_Volume__TRIG_RISING)
#define Buzzer_Volume_TRIG_FALLING                   ((uint32)Buzzer_Volume__TRIG_FALLING)
#define Buzzer_Volume_TRIG_BOTH                      ((uint32)Buzzer_Volume__TRIG_BOTH)
#define Buzzer_Volume_TRIG_LEVEL                     ((uint32)Buzzer_Volume__TRIG_LEVEL)

/* Interrupt mask */
#define Buzzer_Volume_INTR_MASK_TC                   ((uint32)Buzzer_Volume__INTR_MASK_TC)
#define Buzzer_Volume_INTR_MASK_CC_MATCH             ((uint32)Buzzer_Volume__INTR_MASK_CC_MATCH)

/* PWM Output Controls */
#define Buzzer_Volume_CC_MATCH_SET                   (0x00u)
#define Buzzer_Volume_CC_MATCH_CLEAR                 (0x01u)
#define Buzzer_Volume_CC_MATCH_INVERT                (0x02u)
#define Buzzer_Volume_CC_MATCH_NO_CHANGE             (0x03u)
#define Buzzer_Volume_OVERLOW_SET                    (0x00u)
#define Buzzer_Volume_OVERLOW_CLEAR                  (0x04u)
#define Buzzer_Volume_OVERLOW_INVERT                 (0x08u)
#define Buzzer_Volume_OVERLOW_NO_CHANGE              (0x0Cu)
#define Buzzer_Volume_UNDERFLOW_SET                  (0x00u)
#define Buzzer_Volume_UNDERFLOW_CLEAR                (0x10u)
#define Buzzer_Volume_UNDERFLOW_INVERT               (0x20u)
#define Buzzer_Volume_UNDERFLOW_NO_CHANGE            (0x30u)

/* PWM Align */
#define Buzzer_Volume_PWM_MODE_LEFT                  (Buzzer_Volume_CC_MATCH_CLEAR        |   \
                                                         Buzzer_Volume_OVERLOW_SET           |   \
                                                         Buzzer_Volume_UNDERFLOW_NO_CHANGE)
#define Buzzer_Volume_PWM_MODE_RIGHT                 (Buzzer_Volume_CC_MATCH_SET          |   \
                                                         Buzzer_Volume_OVERLOW_NO_CHANGE     |   \
                                                         Buzzer_Volume_UNDERFLOW_CLEAR)
#define Buzzer_Volume_PWM_MODE_ASYM                  (Buzzer_Volume_CC_MATCH_INVERT       |   \
                                                         Buzzer_Volume_OVERLOW_SET           |   \
                                                         Buzzer_Volume_UNDERFLOW_CLEAR)

#if (Buzzer_Volume_CY_TCPWM_V2)
    #if(Buzzer_Volume_CY_TCPWM_4000)
        #define Buzzer_Volume_PWM_MODE_CENTER                (Buzzer_Volume_CC_MATCH_INVERT       |   \
                                                                 Buzzer_Volume_OVERLOW_NO_CHANGE     |   \
                                                                 Buzzer_Volume_UNDERFLOW_CLEAR)
    #else
        #define Buzzer_Volume_PWM_MODE_CENTER                (Buzzer_Volume_CC_MATCH_INVERT       |   \
                                                                 Buzzer_Volume_OVERLOW_SET           |   \
                                                                 Buzzer_Volume_UNDERFLOW_CLEAR)
    #endif /* (Buzzer_Volume_CY_TCPWM_4000) */
#else
    #define Buzzer_Volume_PWM_MODE_CENTER                (Buzzer_Volume_CC_MATCH_INVERT       |   \
                                                             Buzzer_Volume_OVERLOW_NO_CHANGE     |   \
                                                             Buzzer_Volume_UNDERFLOW_CLEAR)
#endif /* (Buzzer_Volume_CY_TCPWM_NEW) */

/* Command operations without condition */
#define Buzzer_Volume_CMD_CAPTURE                    (0u)
#define Buzzer_Volume_CMD_RELOAD                     (8u)
#define Buzzer_Volume_CMD_STOP                       (16u)
#define Buzzer_Volume_CMD_START                      (24u)

/* Status */
#define Buzzer_Volume_STATUS_DOWN                    (1u)
#define Buzzer_Volume_STATUS_RUNNING                 (2u)


/***************************************
*        Function Prototypes
****************************************/

void   Buzzer_Volume_Init(void);
void   Buzzer_Volume_Enable(void);
void   Buzzer_Volume_Start(void);
void   Buzzer_Volume_Stop(void);

void   Buzzer_Volume_SetMode(uint32 mode);
void   Buzzer_Volume_SetCounterMode(uint32 counterMode);
void   Buzzer_Volume_SetPWMMode(uint32 modeMask);
void   Buzzer_Volume_SetQDMode(uint32 qdMode);

void   Buzzer_Volume_SetPrescaler(uint32 prescaler);
void   Buzzer_Volume_TriggerCommand(uint32 mask, uint32 command);
void   Buzzer_Volume_SetOneShot(uint32 oneShotEnable);
uint32 Buzzer_Volume_ReadStatus(void);

void   Buzzer_Volume_SetPWMSyncKill(uint32 syncKillEnable);
void   Buzzer_Volume_SetPWMStopOnKill(uint32 stopOnKillEnable);
void   Buzzer_Volume_SetPWMDeadTime(uint32 deadTime);
void   Buzzer_Volume_SetPWMInvert(uint32 mask);

void   Buzzer_Volume_SetInterruptMode(uint32 interruptMask);
uint32 Buzzer_Volume_GetInterruptSourceMasked(void);
uint32 Buzzer_Volume_GetInterruptSource(void);
void   Buzzer_Volume_ClearInterrupt(uint32 interruptMask);
void   Buzzer_Volume_SetInterrupt(uint32 interruptMask);

void   Buzzer_Volume_WriteCounter(uint32 count);
uint32 Buzzer_Volume_ReadCounter(void);

uint32 Buzzer_Volume_ReadCapture(void);
uint32 Buzzer_Volume_ReadCaptureBuf(void);

void   Buzzer_Volume_WritePeriod(uint32 period);
uint32 Buzzer_Volume_ReadPeriod(void);
void   Buzzer_Volume_WritePeriodBuf(uint32 periodBuf);
uint32 Buzzer_Volume_ReadPeriodBuf(void);

void   Buzzer_Volume_WriteCompare(uint32 compare);
uint32 Buzzer_Volume_ReadCompare(void);
void   Buzzer_Volume_WriteCompareBuf(uint32 compareBuf);
uint32 Buzzer_Volume_ReadCompareBuf(void);

void   Buzzer_Volume_SetPeriodSwap(uint32 swapEnable);
void   Buzzer_Volume_SetCompareSwap(uint32 swapEnable);

void   Buzzer_Volume_SetCaptureMode(uint32 triggerMode);
void   Buzzer_Volume_SetReloadMode(uint32 triggerMode);
void   Buzzer_Volume_SetStartMode(uint32 triggerMode);
void   Buzzer_Volume_SetStopMode(uint32 triggerMode);
void   Buzzer_Volume_SetCountMode(uint32 triggerMode);

void   Buzzer_Volume_SaveConfig(void);
void   Buzzer_Volume_RestoreConfig(void);
void   Buzzer_Volume_Sleep(void);
void   Buzzer_Volume_Wakeup(void);


/***************************************
*             Registers
***************************************/

#define Buzzer_Volume_BLOCK_CONTROL_REG              (*(reg32 *) Buzzer_Volume_cy_m0s8_tcpwm_1__TCPWM_CTRL )
#define Buzzer_Volume_BLOCK_CONTROL_PTR              ( (reg32 *) Buzzer_Volume_cy_m0s8_tcpwm_1__TCPWM_CTRL )
#define Buzzer_Volume_COMMAND_REG                    (*(reg32 *) Buzzer_Volume_cy_m0s8_tcpwm_1__TCPWM_CMD )
#define Buzzer_Volume_COMMAND_PTR                    ( (reg32 *) Buzzer_Volume_cy_m0s8_tcpwm_1__TCPWM_CMD )
#define Buzzer_Volume_INTRRUPT_CAUSE_REG             (*(reg32 *) Buzzer_Volume_cy_m0s8_tcpwm_1__TCPWM_INTR_CAUSE )
#define Buzzer_Volume_INTRRUPT_CAUSE_PTR             ( (reg32 *) Buzzer_Volume_cy_m0s8_tcpwm_1__TCPWM_INTR_CAUSE )
#define Buzzer_Volume_CONTROL_REG                    (*(reg32 *) Buzzer_Volume_cy_m0s8_tcpwm_1__CTRL )
#define Buzzer_Volume_CONTROL_PTR                    ( (reg32 *) Buzzer_Volume_cy_m0s8_tcpwm_1__CTRL )
#define Buzzer_Volume_STATUS_REG                     (*(reg32 *) Buzzer_Volume_cy_m0s8_tcpwm_1__STATUS )
#define Buzzer_Volume_STATUS_PTR                     ( (reg32 *) Buzzer_Volume_cy_m0s8_tcpwm_1__STATUS )
#define Buzzer_Volume_COUNTER_REG                    (*(reg32 *) Buzzer_Volume_cy_m0s8_tcpwm_1__COUNTER )
#define Buzzer_Volume_COUNTER_PTR                    ( (reg32 *) Buzzer_Volume_cy_m0s8_tcpwm_1__COUNTER )
#define Buzzer_Volume_COMP_CAP_REG                   (*(reg32 *) Buzzer_Volume_cy_m0s8_tcpwm_1__CC )
#define Buzzer_Volume_COMP_CAP_PTR                   ( (reg32 *) Buzzer_Volume_cy_m0s8_tcpwm_1__CC )
#define Buzzer_Volume_COMP_CAP_BUF_REG               (*(reg32 *) Buzzer_Volume_cy_m0s8_tcpwm_1__CC_BUFF )
#define Buzzer_Volume_COMP_CAP_BUF_PTR               ( (reg32 *) Buzzer_Volume_cy_m0s8_tcpwm_1__CC_BUFF )
#define Buzzer_Volume_PERIOD_REG                     (*(reg32 *) Buzzer_Volume_cy_m0s8_tcpwm_1__PERIOD )
#define Buzzer_Volume_PERIOD_PTR                     ( (reg32 *) Buzzer_Volume_cy_m0s8_tcpwm_1__PERIOD )
#define Buzzer_Volume_PERIOD_BUF_REG                 (*(reg32 *) Buzzer_Volume_cy_m0s8_tcpwm_1__PERIOD_BUFF )
#define Buzzer_Volume_PERIOD_BUF_PTR                 ( (reg32 *) Buzzer_Volume_cy_m0s8_tcpwm_1__PERIOD_BUFF )
#define Buzzer_Volume_TRIG_CONTROL0_REG              (*(reg32 *) Buzzer_Volume_cy_m0s8_tcpwm_1__TR_CTRL0 )
#define Buzzer_Volume_TRIG_CONTROL0_PTR              ( (reg32 *) Buzzer_Volume_cy_m0s8_tcpwm_1__TR_CTRL0 )
#define Buzzer_Volume_TRIG_CONTROL1_REG              (*(reg32 *) Buzzer_Volume_cy_m0s8_tcpwm_1__TR_CTRL1 )
#define Buzzer_Volume_TRIG_CONTROL1_PTR              ( (reg32 *) Buzzer_Volume_cy_m0s8_tcpwm_1__TR_CTRL1 )
#define Buzzer_Volume_TRIG_CONTROL2_REG              (*(reg32 *) Buzzer_Volume_cy_m0s8_tcpwm_1__TR_CTRL2 )
#define Buzzer_Volume_TRIG_CONTROL2_PTR              ( (reg32 *) Buzzer_Volume_cy_m0s8_tcpwm_1__TR_CTRL2 )
#define Buzzer_Volume_INTERRUPT_REQ_REG              (*(reg32 *) Buzzer_Volume_cy_m0s8_tcpwm_1__INTR )
#define Buzzer_Volume_INTERRUPT_REQ_PTR              ( (reg32 *) Buzzer_Volume_cy_m0s8_tcpwm_1__INTR )
#define Buzzer_Volume_INTERRUPT_SET_REG              (*(reg32 *) Buzzer_Volume_cy_m0s8_tcpwm_1__INTR_SET )
#define Buzzer_Volume_INTERRUPT_SET_PTR              ( (reg32 *) Buzzer_Volume_cy_m0s8_tcpwm_1__INTR_SET )
#define Buzzer_Volume_INTERRUPT_MASK_REG             (*(reg32 *) Buzzer_Volume_cy_m0s8_tcpwm_1__INTR_MASK )
#define Buzzer_Volume_INTERRUPT_MASK_PTR             ( (reg32 *) Buzzer_Volume_cy_m0s8_tcpwm_1__INTR_MASK )
#define Buzzer_Volume_INTERRUPT_MASKED_REG           (*(reg32 *) Buzzer_Volume_cy_m0s8_tcpwm_1__INTR_MASKED )
#define Buzzer_Volume_INTERRUPT_MASKED_PTR           ( (reg32 *) Buzzer_Volume_cy_m0s8_tcpwm_1__INTR_MASKED )


/***************************************
*       Registers Constants
***************************************/

/* Mask */
#define Buzzer_Volume_MASK                           ((uint32)Buzzer_Volume_cy_m0s8_tcpwm_1__TCPWM_CTRL_MASK)

/* Shift constants for control register */
#define Buzzer_Volume_RELOAD_CC_SHIFT                (0u)
#define Buzzer_Volume_RELOAD_PERIOD_SHIFT            (1u)
#define Buzzer_Volume_PWM_SYNC_KILL_SHIFT            (2u)
#define Buzzer_Volume_PWM_STOP_KILL_SHIFT            (3u)
#define Buzzer_Volume_PRESCALER_SHIFT                (8u)
#define Buzzer_Volume_UPDOWN_SHIFT                   (16u)
#define Buzzer_Volume_ONESHOT_SHIFT                  (18u)
#define Buzzer_Volume_QUAD_MODE_SHIFT                (20u)
#define Buzzer_Volume_INV_OUT_SHIFT                  (20u)
#define Buzzer_Volume_INV_COMPL_OUT_SHIFT            (21u)
#define Buzzer_Volume_MODE_SHIFT                     (24u)

/* Mask constants for control register */
#define Buzzer_Volume_RELOAD_CC_MASK                 ((uint32)(Buzzer_Volume_1BIT_MASK        <<  \
                                                                            Buzzer_Volume_RELOAD_CC_SHIFT))
#define Buzzer_Volume_RELOAD_PERIOD_MASK             ((uint32)(Buzzer_Volume_1BIT_MASK        <<  \
                                                                            Buzzer_Volume_RELOAD_PERIOD_SHIFT))
#define Buzzer_Volume_PWM_SYNC_KILL_MASK             ((uint32)(Buzzer_Volume_1BIT_MASK        <<  \
                                                                            Buzzer_Volume_PWM_SYNC_KILL_SHIFT))
#define Buzzer_Volume_PWM_STOP_KILL_MASK             ((uint32)(Buzzer_Volume_1BIT_MASK        <<  \
                                                                            Buzzer_Volume_PWM_STOP_KILL_SHIFT))
#define Buzzer_Volume_PRESCALER_MASK                 ((uint32)(Buzzer_Volume_8BIT_MASK        <<  \
                                                                            Buzzer_Volume_PRESCALER_SHIFT))
#define Buzzer_Volume_UPDOWN_MASK                    ((uint32)(Buzzer_Volume_2BIT_MASK        <<  \
                                                                            Buzzer_Volume_UPDOWN_SHIFT))
#define Buzzer_Volume_ONESHOT_MASK                   ((uint32)(Buzzer_Volume_1BIT_MASK        <<  \
                                                                            Buzzer_Volume_ONESHOT_SHIFT))
#define Buzzer_Volume_QUAD_MODE_MASK                 ((uint32)(Buzzer_Volume_3BIT_MASK        <<  \
                                                                            Buzzer_Volume_QUAD_MODE_SHIFT))
#define Buzzer_Volume_INV_OUT_MASK                   ((uint32)(Buzzer_Volume_2BIT_MASK        <<  \
                                                                            Buzzer_Volume_INV_OUT_SHIFT))
#define Buzzer_Volume_MODE_MASK                      ((uint32)(Buzzer_Volume_3BIT_MASK        <<  \
                                                                            Buzzer_Volume_MODE_SHIFT))

/* Shift constants for trigger control register 1 */
#define Buzzer_Volume_CAPTURE_SHIFT                  (0u)
#define Buzzer_Volume_COUNT_SHIFT                    (2u)
#define Buzzer_Volume_RELOAD_SHIFT                   (4u)
#define Buzzer_Volume_STOP_SHIFT                     (6u)
#define Buzzer_Volume_START_SHIFT                    (8u)

/* Mask constants for trigger control register 1 */
#define Buzzer_Volume_CAPTURE_MASK                   ((uint32)(Buzzer_Volume_2BIT_MASK        <<  \
                                                                  Buzzer_Volume_CAPTURE_SHIFT))
#define Buzzer_Volume_COUNT_MASK                     ((uint32)(Buzzer_Volume_2BIT_MASK        <<  \
                                                                  Buzzer_Volume_COUNT_SHIFT))
#define Buzzer_Volume_RELOAD_MASK                    ((uint32)(Buzzer_Volume_2BIT_MASK        <<  \
                                                                  Buzzer_Volume_RELOAD_SHIFT))
#define Buzzer_Volume_STOP_MASK                      ((uint32)(Buzzer_Volume_2BIT_MASK        <<  \
                                                                  Buzzer_Volume_STOP_SHIFT))
#define Buzzer_Volume_START_MASK                     ((uint32)(Buzzer_Volume_2BIT_MASK        <<  \
                                                                  Buzzer_Volume_START_SHIFT))

/* MASK */
#define Buzzer_Volume_1BIT_MASK                      ((uint32)0x01u)
#define Buzzer_Volume_2BIT_MASK                      ((uint32)0x03u)
#define Buzzer_Volume_3BIT_MASK                      ((uint32)0x07u)
#define Buzzer_Volume_6BIT_MASK                      ((uint32)0x3Fu)
#define Buzzer_Volume_8BIT_MASK                      ((uint32)0xFFu)
#define Buzzer_Volume_16BIT_MASK                     ((uint32)0xFFFFu)

/* Shift constant for status register */
#define Buzzer_Volume_RUNNING_STATUS_SHIFT           (30u)


/***************************************
*    Initial Constants
***************************************/

#define Buzzer_Volume_CTRL_QUAD_BASE_CONFIG                                                          \
        (((uint32)(Buzzer_Volume_QUAD_ENCODING_MODES     << Buzzer_Volume_QUAD_MODE_SHIFT))       |\
         ((uint32)(Buzzer_Volume_CONFIG                  << Buzzer_Volume_MODE_SHIFT)))

#define Buzzer_Volume_CTRL_PWM_BASE_CONFIG                                                           \
        (((uint32)(Buzzer_Volume_PWM_STOP_EVENT          << Buzzer_Volume_PWM_STOP_KILL_SHIFT))   |\
         ((uint32)(Buzzer_Volume_PWM_OUT_INVERT          << Buzzer_Volume_INV_OUT_SHIFT))         |\
         ((uint32)(Buzzer_Volume_PWM_OUT_N_INVERT        << Buzzer_Volume_INV_COMPL_OUT_SHIFT))   |\
         ((uint32)(Buzzer_Volume_PWM_MODE                << Buzzer_Volume_MODE_SHIFT)))

#define Buzzer_Volume_CTRL_PWM_RUN_MODE                                                              \
            ((uint32)(Buzzer_Volume_PWM_RUN_MODE         << Buzzer_Volume_ONESHOT_SHIFT))
            
#define Buzzer_Volume_CTRL_PWM_ALIGN                                                                 \
            ((uint32)(Buzzer_Volume_PWM_ALIGN            << Buzzer_Volume_UPDOWN_SHIFT))

#define Buzzer_Volume_CTRL_PWM_KILL_EVENT                                                            \
             ((uint32)(Buzzer_Volume_PWM_KILL_EVENT      << Buzzer_Volume_PWM_SYNC_KILL_SHIFT))

#define Buzzer_Volume_CTRL_PWM_DEAD_TIME_CYCLE                                                       \
            ((uint32)(Buzzer_Volume_PWM_DEAD_TIME_CYCLE  << Buzzer_Volume_PRESCALER_SHIFT))

#define Buzzer_Volume_CTRL_PWM_PRESCALER                                                             \
            ((uint32)(Buzzer_Volume_PWM_PRESCALER        << Buzzer_Volume_PRESCALER_SHIFT))

#define Buzzer_Volume_CTRL_TIMER_BASE_CONFIG                                                         \
        (((uint32)(Buzzer_Volume_TC_PRESCALER            << Buzzer_Volume_PRESCALER_SHIFT))       |\
         ((uint32)(Buzzer_Volume_TC_COUNTER_MODE         << Buzzer_Volume_UPDOWN_SHIFT))          |\
         ((uint32)(Buzzer_Volume_TC_RUN_MODE             << Buzzer_Volume_ONESHOT_SHIFT))         |\
         ((uint32)(Buzzer_Volume_TC_COMP_CAP_MODE        << Buzzer_Volume_MODE_SHIFT)))
        
#define Buzzer_Volume_QUAD_SIGNALS_MODES                                                             \
        (((uint32)(Buzzer_Volume_QUAD_PHIA_SIGNAL_MODE   << Buzzer_Volume_COUNT_SHIFT))           |\
         ((uint32)(Buzzer_Volume_QUAD_INDEX_SIGNAL_MODE  << Buzzer_Volume_RELOAD_SHIFT))          |\
         ((uint32)(Buzzer_Volume_QUAD_STOP_SIGNAL_MODE   << Buzzer_Volume_STOP_SHIFT))            |\
         ((uint32)(Buzzer_Volume_QUAD_PHIB_SIGNAL_MODE   << Buzzer_Volume_START_SHIFT)))

#define Buzzer_Volume_PWM_SIGNALS_MODES                                                              \
        (((uint32)(Buzzer_Volume_PWM_SWITCH_SIGNAL_MODE  << Buzzer_Volume_CAPTURE_SHIFT))         |\
         ((uint32)(Buzzer_Volume_PWM_COUNT_SIGNAL_MODE   << Buzzer_Volume_COUNT_SHIFT))           |\
         ((uint32)(Buzzer_Volume_PWM_RELOAD_SIGNAL_MODE  << Buzzer_Volume_RELOAD_SHIFT))          |\
         ((uint32)(Buzzer_Volume_PWM_STOP_SIGNAL_MODE    << Buzzer_Volume_STOP_SHIFT))            |\
         ((uint32)(Buzzer_Volume_PWM_START_SIGNAL_MODE   << Buzzer_Volume_START_SHIFT)))

#define Buzzer_Volume_TIMER_SIGNALS_MODES                                                            \
        (((uint32)(Buzzer_Volume_TC_CAPTURE_SIGNAL_MODE  << Buzzer_Volume_CAPTURE_SHIFT))         |\
         ((uint32)(Buzzer_Volume_TC_COUNT_SIGNAL_MODE    << Buzzer_Volume_COUNT_SHIFT))           |\
         ((uint32)(Buzzer_Volume_TC_RELOAD_SIGNAL_MODE   << Buzzer_Volume_RELOAD_SHIFT))          |\
         ((uint32)(Buzzer_Volume_TC_STOP_SIGNAL_MODE     << Buzzer_Volume_STOP_SHIFT))            |\
         ((uint32)(Buzzer_Volume_TC_START_SIGNAL_MODE    << Buzzer_Volume_START_SHIFT)))
        
#define Buzzer_Volume_TIMER_UPDOWN_CNT_USED                                                          \
                ((Buzzer_Volume__COUNT_UPDOWN0 == Buzzer_Volume_TC_COUNTER_MODE)                  ||\
                 (Buzzer_Volume__COUNT_UPDOWN1 == Buzzer_Volume_TC_COUNTER_MODE))

#define Buzzer_Volume_PWM_UPDOWN_CNT_USED                                                            \
                ((Buzzer_Volume__CENTER == Buzzer_Volume_PWM_ALIGN)                               ||\
                 (Buzzer_Volume__ASYMMETRIC == Buzzer_Volume_PWM_ALIGN))               
        
#define Buzzer_Volume_PWM_PR_INIT_VALUE              (1u)
#define Buzzer_Volume_QUAD_PERIOD_INIT_VALUE         (0x8000u)



#endif /* End CY_TCPWM_Buzzer_Volume_H */

/* [] END OF FILE */
