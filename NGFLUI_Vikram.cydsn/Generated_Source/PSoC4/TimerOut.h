/*******************************************************************************
* File Name: TimerOut.h
* Version 2.10
*
* Description:
*  This file provides constants and parameter values for the TimerOut
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

#if !defined(CY_TCPWM_TimerOut_H)
#define CY_TCPWM_TimerOut_H


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
} TimerOut_BACKUP_STRUCT;


/*******************************************************************************
* Variables
*******************************************************************************/
extern uint8  TimerOut_initVar;


/***************************************
*   Conditional Compilation Parameters
****************************************/

#define TimerOut_CY_TCPWM_V2                    (CYIPBLOCK_m0s8tcpwm_VERSION == 2u)
#define TimerOut_CY_TCPWM_4000                  (CY_PSOC4_4000)

/* TCPWM Configuration */
#define TimerOut_CONFIG                         (1lu)

/* Quad Mode */
/* Parameters */
#define TimerOut_QUAD_ENCODING_MODES            (0lu)
#define TimerOut_QUAD_AUTO_START                (1lu)

/* Signal modes */
#define TimerOut_QUAD_INDEX_SIGNAL_MODE         (0lu)
#define TimerOut_QUAD_PHIA_SIGNAL_MODE          (3lu)
#define TimerOut_QUAD_PHIB_SIGNAL_MODE          (3lu)
#define TimerOut_QUAD_STOP_SIGNAL_MODE          (0lu)

/* Signal present */
#define TimerOut_QUAD_INDEX_SIGNAL_PRESENT      (0lu)
#define TimerOut_QUAD_STOP_SIGNAL_PRESENT       (0lu)

/* Interrupt Mask */
#define TimerOut_QUAD_INTERRUPT_MASK            (1lu)

/* Timer/Counter Mode */
/* Parameters */
#define TimerOut_TC_RUN_MODE                    (1lu)
#define TimerOut_TC_COUNTER_MODE                (0lu)
#define TimerOut_TC_COMP_CAP_MODE               (2lu)
#define TimerOut_TC_PRESCALER                   (7lu)

/* Signal modes */
#define TimerOut_TC_RELOAD_SIGNAL_MODE          (0lu)
#define TimerOut_TC_COUNT_SIGNAL_MODE           (3lu)
#define TimerOut_TC_START_SIGNAL_MODE           (0lu)
#define TimerOut_TC_STOP_SIGNAL_MODE            (0lu)
#define TimerOut_TC_CAPTURE_SIGNAL_MODE         (0lu)

/* Signal present */
#define TimerOut_TC_RELOAD_SIGNAL_PRESENT       (0lu)
#define TimerOut_TC_COUNT_SIGNAL_PRESENT        (0lu)
#define TimerOut_TC_START_SIGNAL_PRESENT        (0lu)
#define TimerOut_TC_STOP_SIGNAL_PRESENT         (0lu)
#define TimerOut_TC_CAPTURE_SIGNAL_PRESENT      (0lu)

/* Interrupt Mask */
#define TimerOut_TC_INTERRUPT_MASK              (1lu)

/* PWM Mode */
/* Parameters */
#define TimerOut_PWM_KILL_EVENT                 (0lu)
#define TimerOut_PWM_STOP_EVENT                 (0lu)
#define TimerOut_PWM_MODE                       (4lu)
#define TimerOut_PWM_OUT_N_INVERT               (0lu)
#define TimerOut_PWM_OUT_INVERT                 (0lu)
#define TimerOut_PWM_ALIGN                      (0lu)
#define TimerOut_PWM_RUN_MODE                   (0lu)
#define TimerOut_PWM_DEAD_TIME_CYCLE            (0lu)
#define TimerOut_PWM_PRESCALER                  (0lu)

/* Signal modes */
#define TimerOut_PWM_RELOAD_SIGNAL_MODE         (0lu)
#define TimerOut_PWM_COUNT_SIGNAL_MODE          (3lu)
#define TimerOut_PWM_START_SIGNAL_MODE          (0lu)
#define TimerOut_PWM_STOP_SIGNAL_MODE           (0lu)
#define TimerOut_PWM_SWITCH_SIGNAL_MODE         (0lu)

/* Signal present */
#define TimerOut_PWM_RELOAD_SIGNAL_PRESENT      (0lu)
#define TimerOut_PWM_COUNT_SIGNAL_PRESENT       (0lu)
#define TimerOut_PWM_START_SIGNAL_PRESENT       (0lu)
#define TimerOut_PWM_STOP_SIGNAL_PRESENT        (0lu)
#define TimerOut_PWM_SWITCH_SIGNAL_PRESENT      (0lu)

/* Interrupt Mask */
#define TimerOut_PWM_INTERRUPT_MASK             (1lu)


/***************************************
*    Initial Parameter Constants
***************************************/

/* Timer/Counter Mode */
#define TimerOut_TC_PERIOD_VALUE                (700lu)
#define TimerOut_TC_COMPARE_VALUE               (65535lu)
#define TimerOut_TC_COMPARE_BUF_VALUE           (65535lu)
#define TimerOut_TC_COMPARE_SWAP                (0lu)

/* PWM Mode */
#define TimerOut_PWM_PERIOD_VALUE               (65535lu)
#define TimerOut_PWM_PERIOD_BUF_VALUE           (65535lu)
#define TimerOut_PWM_PERIOD_SWAP                (0lu)
#define TimerOut_PWM_COMPARE_VALUE              (65535lu)
#define TimerOut_PWM_COMPARE_BUF_VALUE          (65535lu)
#define TimerOut_PWM_COMPARE_SWAP               (0lu)


/***************************************
*    Enumerated Types and Parameters
***************************************/

#define TimerOut__LEFT 0
#define TimerOut__RIGHT 1
#define TimerOut__CENTER 2
#define TimerOut__ASYMMETRIC 3

#define TimerOut__X1 0
#define TimerOut__X2 1
#define TimerOut__X4 2

#define TimerOut__PWM 4
#define TimerOut__PWM_DT 5
#define TimerOut__PWM_PR 6

#define TimerOut__INVERSE 1
#define TimerOut__DIRECT 0

#define TimerOut__CAPTURE 2
#define TimerOut__COMPARE 0

#define TimerOut__TRIG_LEVEL 3
#define TimerOut__TRIG_RISING 0
#define TimerOut__TRIG_FALLING 1
#define TimerOut__TRIG_BOTH 2

#define TimerOut__INTR_MASK_TC 1
#define TimerOut__INTR_MASK_CC_MATCH 2
#define TimerOut__INTR_MASK_NONE 0
#define TimerOut__INTR_MASK_TC_CC 3

#define TimerOut__UNCONFIG 8
#define TimerOut__TIMER 1
#define TimerOut__QUAD 3
#define TimerOut__PWM_SEL 7

#define TimerOut__COUNT_UP 0
#define TimerOut__COUNT_DOWN 1
#define TimerOut__COUNT_UPDOWN0 2
#define TimerOut__COUNT_UPDOWN1 3


/* Prescaler */
#define TimerOut_PRESCALE_DIVBY1                ((uint32)(0u << TimerOut_PRESCALER_SHIFT))
#define TimerOut_PRESCALE_DIVBY2                ((uint32)(1u << TimerOut_PRESCALER_SHIFT))
#define TimerOut_PRESCALE_DIVBY4                ((uint32)(2u << TimerOut_PRESCALER_SHIFT))
#define TimerOut_PRESCALE_DIVBY8                ((uint32)(3u << TimerOut_PRESCALER_SHIFT))
#define TimerOut_PRESCALE_DIVBY16               ((uint32)(4u << TimerOut_PRESCALER_SHIFT))
#define TimerOut_PRESCALE_DIVBY32               ((uint32)(5u << TimerOut_PRESCALER_SHIFT))
#define TimerOut_PRESCALE_DIVBY64               ((uint32)(6u << TimerOut_PRESCALER_SHIFT))
#define TimerOut_PRESCALE_DIVBY128              ((uint32)(7u << TimerOut_PRESCALER_SHIFT))

/* TCPWM set modes */
#define TimerOut_MODE_TIMER_COMPARE             ((uint32)(TimerOut__COMPARE         <<  \
                                                                  TimerOut_MODE_SHIFT))
#define TimerOut_MODE_TIMER_CAPTURE             ((uint32)(TimerOut__CAPTURE         <<  \
                                                                  TimerOut_MODE_SHIFT))
#define TimerOut_MODE_QUAD                      ((uint32)(TimerOut__QUAD            <<  \
                                                                  TimerOut_MODE_SHIFT))
#define TimerOut_MODE_PWM                       ((uint32)(TimerOut__PWM             <<  \
                                                                  TimerOut_MODE_SHIFT))
#define TimerOut_MODE_PWM_DT                    ((uint32)(TimerOut__PWM_DT          <<  \
                                                                  TimerOut_MODE_SHIFT))
#define TimerOut_MODE_PWM_PR                    ((uint32)(TimerOut__PWM_PR          <<  \
                                                                  TimerOut_MODE_SHIFT))

/* Quad Modes */
#define TimerOut_MODE_X1                        ((uint32)(TimerOut__X1              <<  \
                                                                  TimerOut_QUAD_MODE_SHIFT))
#define TimerOut_MODE_X2                        ((uint32)(TimerOut__X2              <<  \
                                                                  TimerOut_QUAD_MODE_SHIFT))
#define TimerOut_MODE_X4                        ((uint32)(TimerOut__X4              <<  \
                                                                  TimerOut_QUAD_MODE_SHIFT))

/* Counter modes */
#define TimerOut_COUNT_UP                       ((uint32)(TimerOut__COUNT_UP        <<  \
                                                                  TimerOut_UPDOWN_SHIFT))
#define TimerOut_COUNT_DOWN                     ((uint32)(TimerOut__COUNT_DOWN      <<  \
                                                                  TimerOut_UPDOWN_SHIFT))
#define TimerOut_COUNT_UPDOWN0                  ((uint32)(TimerOut__COUNT_UPDOWN0   <<  \
                                                                  TimerOut_UPDOWN_SHIFT))
#define TimerOut_COUNT_UPDOWN1                  ((uint32)(TimerOut__COUNT_UPDOWN1   <<  \
                                                                  TimerOut_UPDOWN_SHIFT))

/* PWM output invert */
#define TimerOut_INVERT_LINE                    ((uint32)(TimerOut__INVERSE         <<  \
                                                                  TimerOut_INV_OUT_SHIFT))
#define TimerOut_INVERT_LINE_N                  ((uint32)(TimerOut__INVERSE         <<  \
                                                                  TimerOut_INV_COMPL_OUT_SHIFT))

/* Trigger modes */
#define TimerOut_TRIG_RISING                    ((uint32)TimerOut__TRIG_RISING)
#define TimerOut_TRIG_FALLING                   ((uint32)TimerOut__TRIG_FALLING)
#define TimerOut_TRIG_BOTH                      ((uint32)TimerOut__TRIG_BOTH)
#define TimerOut_TRIG_LEVEL                     ((uint32)TimerOut__TRIG_LEVEL)

/* Interrupt mask */
#define TimerOut_INTR_MASK_TC                   ((uint32)TimerOut__INTR_MASK_TC)
#define TimerOut_INTR_MASK_CC_MATCH             ((uint32)TimerOut__INTR_MASK_CC_MATCH)

/* PWM Output Controls */
#define TimerOut_CC_MATCH_SET                   (0x00u)
#define TimerOut_CC_MATCH_CLEAR                 (0x01u)
#define TimerOut_CC_MATCH_INVERT                (0x02u)
#define TimerOut_CC_MATCH_NO_CHANGE             (0x03u)
#define TimerOut_OVERLOW_SET                    (0x00u)
#define TimerOut_OVERLOW_CLEAR                  (0x04u)
#define TimerOut_OVERLOW_INVERT                 (0x08u)
#define TimerOut_OVERLOW_NO_CHANGE              (0x0Cu)
#define TimerOut_UNDERFLOW_SET                  (0x00u)
#define TimerOut_UNDERFLOW_CLEAR                (0x10u)
#define TimerOut_UNDERFLOW_INVERT               (0x20u)
#define TimerOut_UNDERFLOW_NO_CHANGE            (0x30u)

/* PWM Align */
#define TimerOut_PWM_MODE_LEFT                  (TimerOut_CC_MATCH_CLEAR        |   \
                                                         TimerOut_OVERLOW_SET           |   \
                                                         TimerOut_UNDERFLOW_NO_CHANGE)
#define TimerOut_PWM_MODE_RIGHT                 (TimerOut_CC_MATCH_SET          |   \
                                                         TimerOut_OVERLOW_NO_CHANGE     |   \
                                                         TimerOut_UNDERFLOW_CLEAR)
#define TimerOut_PWM_MODE_ASYM                  (TimerOut_CC_MATCH_INVERT       |   \
                                                         TimerOut_OVERLOW_SET           |   \
                                                         TimerOut_UNDERFLOW_CLEAR)

#if (TimerOut_CY_TCPWM_V2)
    #if(TimerOut_CY_TCPWM_4000)
        #define TimerOut_PWM_MODE_CENTER                (TimerOut_CC_MATCH_INVERT       |   \
                                                                 TimerOut_OVERLOW_NO_CHANGE     |   \
                                                                 TimerOut_UNDERFLOW_CLEAR)
    #else
        #define TimerOut_PWM_MODE_CENTER                (TimerOut_CC_MATCH_INVERT       |   \
                                                                 TimerOut_OVERLOW_SET           |   \
                                                                 TimerOut_UNDERFLOW_CLEAR)
    #endif /* (TimerOut_CY_TCPWM_4000) */
#else
    #define TimerOut_PWM_MODE_CENTER                (TimerOut_CC_MATCH_INVERT       |   \
                                                             TimerOut_OVERLOW_NO_CHANGE     |   \
                                                             TimerOut_UNDERFLOW_CLEAR)
#endif /* (TimerOut_CY_TCPWM_NEW) */

/* Command operations without condition */
#define TimerOut_CMD_CAPTURE                    (0u)
#define TimerOut_CMD_RELOAD                     (8u)
#define TimerOut_CMD_STOP                       (16u)
#define TimerOut_CMD_START                      (24u)

/* Status */
#define TimerOut_STATUS_DOWN                    (1u)
#define TimerOut_STATUS_RUNNING                 (2u)


/***************************************
*        Function Prototypes
****************************************/

void   TimerOut_Init(void);
void   TimerOut_Enable(void);
void   TimerOut_Start(void);
void   TimerOut_Stop(void);

void   TimerOut_SetMode(uint32 mode);
void   TimerOut_SetCounterMode(uint32 counterMode);
void   TimerOut_SetPWMMode(uint32 modeMask);
void   TimerOut_SetQDMode(uint32 qdMode);

void   TimerOut_SetPrescaler(uint32 prescaler);
void   TimerOut_TriggerCommand(uint32 mask, uint32 command);
void   TimerOut_SetOneShot(uint32 oneShotEnable);
uint32 TimerOut_ReadStatus(void);

void   TimerOut_SetPWMSyncKill(uint32 syncKillEnable);
void   TimerOut_SetPWMStopOnKill(uint32 stopOnKillEnable);
void   TimerOut_SetPWMDeadTime(uint32 deadTime);
void   TimerOut_SetPWMInvert(uint32 mask);

void   TimerOut_SetInterruptMode(uint32 interruptMask);
uint32 TimerOut_GetInterruptSourceMasked(void);
uint32 TimerOut_GetInterruptSource(void);
void   TimerOut_ClearInterrupt(uint32 interruptMask);
void   TimerOut_SetInterrupt(uint32 interruptMask);

void   TimerOut_WriteCounter(uint32 count);
uint32 TimerOut_ReadCounter(void);

uint32 TimerOut_ReadCapture(void);
uint32 TimerOut_ReadCaptureBuf(void);

void   TimerOut_WritePeriod(uint32 period);
uint32 TimerOut_ReadPeriod(void);
void   TimerOut_WritePeriodBuf(uint32 periodBuf);
uint32 TimerOut_ReadPeriodBuf(void);

void   TimerOut_WriteCompare(uint32 compare);
uint32 TimerOut_ReadCompare(void);
void   TimerOut_WriteCompareBuf(uint32 compareBuf);
uint32 TimerOut_ReadCompareBuf(void);

void   TimerOut_SetPeriodSwap(uint32 swapEnable);
void   TimerOut_SetCompareSwap(uint32 swapEnable);

void   TimerOut_SetCaptureMode(uint32 triggerMode);
void   TimerOut_SetReloadMode(uint32 triggerMode);
void   TimerOut_SetStartMode(uint32 triggerMode);
void   TimerOut_SetStopMode(uint32 triggerMode);
void   TimerOut_SetCountMode(uint32 triggerMode);

void   TimerOut_SaveConfig(void);
void   TimerOut_RestoreConfig(void);
void   TimerOut_Sleep(void);
void   TimerOut_Wakeup(void);


/***************************************
*             Registers
***************************************/

#define TimerOut_BLOCK_CONTROL_REG              (*(reg32 *) TimerOut_cy_m0s8_tcpwm_1__TCPWM_CTRL )
#define TimerOut_BLOCK_CONTROL_PTR              ( (reg32 *) TimerOut_cy_m0s8_tcpwm_1__TCPWM_CTRL )
#define TimerOut_COMMAND_REG                    (*(reg32 *) TimerOut_cy_m0s8_tcpwm_1__TCPWM_CMD )
#define TimerOut_COMMAND_PTR                    ( (reg32 *) TimerOut_cy_m0s8_tcpwm_1__TCPWM_CMD )
#define TimerOut_INTRRUPT_CAUSE_REG             (*(reg32 *) TimerOut_cy_m0s8_tcpwm_1__TCPWM_INTR_CAUSE )
#define TimerOut_INTRRUPT_CAUSE_PTR             ( (reg32 *) TimerOut_cy_m0s8_tcpwm_1__TCPWM_INTR_CAUSE )
#define TimerOut_CONTROL_REG                    (*(reg32 *) TimerOut_cy_m0s8_tcpwm_1__CTRL )
#define TimerOut_CONTROL_PTR                    ( (reg32 *) TimerOut_cy_m0s8_tcpwm_1__CTRL )
#define TimerOut_STATUS_REG                     (*(reg32 *) TimerOut_cy_m0s8_tcpwm_1__STATUS )
#define TimerOut_STATUS_PTR                     ( (reg32 *) TimerOut_cy_m0s8_tcpwm_1__STATUS )
#define TimerOut_COUNTER_REG                    (*(reg32 *) TimerOut_cy_m0s8_tcpwm_1__COUNTER )
#define TimerOut_COUNTER_PTR                    ( (reg32 *) TimerOut_cy_m0s8_tcpwm_1__COUNTER )
#define TimerOut_COMP_CAP_REG                   (*(reg32 *) TimerOut_cy_m0s8_tcpwm_1__CC )
#define TimerOut_COMP_CAP_PTR                   ( (reg32 *) TimerOut_cy_m0s8_tcpwm_1__CC )
#define TimerOut_COMP_CAP_BUF_REG               (*(reg32 *) TimerOut_cy_m0s8_tcpwm_1__CC_BUFF )
#define TimerOut_COMP_CAP_BUF_PTR               ( (reg32 *) TimerOut_cy_m0s8_tcpwm_1__CC_BUFF )
#define TimerOut_PERIOD_REG                     (*(reg32 *) TimerOut_cy_m0s8_tcpwm_1__PERIOD )
#define TimerOut_PERIOD_PTR                     ( (reg32 *) TimerOut_cy_m0s8_tcpwm_1__PERIOD )
#define TimerOut_PERIOD_BUF_REG                 (*(reg32 *) TimerOut_cy_m0s8_tcpwm_1__PERIOD_BUFF )
#define TimerOut_PERIOD_BUF_PTR                 ( (reg32 *) TimerOut_cy_m0s8_tcpwm_1__PERIOD_BUFF )
#define TimerOut_TRIG_CONTROL0_REG              (*(reg32 *) TimerOut_cy_m0s8_tcpwm_1__TR_CTRL0 )
#define TimerOut_TRIG_CONTROL0_PTR              ( (reg32 *) TimerOut_cy_m0s8_tcpwm_1__TR_CTRL0 )
#define TimerOut_TRIG_CONTROL1_REG              (*(reg32 *) TimerOut_cy_m0s8_tcpwm_1__TR_CTRL1 )
#define TimerOut_TRIG_CONTROL1_PTR              ( (reg32 *) TimerOut_cy_m0s8_tcpwm_1__TR_CTRL1 )
#define TimerOut_TRIG_CONTROL2_REG              (*(reg32 *) TimerOut_cy_m0s8_tcpwm_1__TR_CTRL2 )
#define TimerOut_TRIG_CONTROL2_PTR              ( (reg32 *) TimerOut_cy_m0s8_tcpwm_1__TR_CTRL2 )
#define TimerOut_INTERRUPT_REQ_REG              (*(reg32 *) TimerOut_cy_m0s8_tcpwm_1__INTR )
#define TimerOut_INTERRUPT_REQ_PTR              ( (reg32 *) TimerOut_cy_m0s8_tcpwm_1__INTR )
#define TimerOut_INTERRUPT_SET_REG              (*(reg32 *) TimerOut_cy_m0s8_tcpwm_1__INTR_SET )
#define TimerOut_INTERRUPT_SET_PTR              ( (reg32 *) TimerOut_cy_m0s8_tcpwm_1__INTR_SET )
#define TimerOut_INTERRUPT_MASK_REG             (*(reg32 *) TimerOut_cy_m0s8_tcpwm_1__INTR_MASK )
#define TimerOut_INTERRUPT_MASK_PTR             ( (reg32 *) TimerOut_cy_m0s8_tcpwm_1__INTR_MASK )
#define TimerOut_INTERRUPT_MASKED_REG           (*(reg32 *) TimerOut_cy_m0s8_tcpwm_1__INTR_MASKED )
#define TimerOut_INTERRUPT_MASKED_PTR           ( (reg32 *) TimerOut_cy_m0s8_tcpwm_1__INTR_MASKED )


/***************************************
*       Registers Constants
***************************************/

/* Mask */
#define TimerOut_MASK                           ((uint32)TimerOut_cy_m0s8_tcpwm_1__TCPWM_CTRL_MASK)

/* Shift constants for control register */
#define TimerOut_RELOAD_CC_SHIFT                (0u)
#define TimerOut_RELOAD_PERIOD_SHIFT            (1u)
#define TimerOut_PWM_SYNC_KILL_SHIFT            (2u)
#define TimerOut_PWM_STOP_KILL_SHIFT            (3u)
#define TimerOut_PRESCALER_SHIFT                (8u)
#define TimerOut_UPDOWN_SHIFT                   (16u)
#define TimerOut_ONESHOT_SHIFT                  (18u)
#define TimerOut_QUAD_MODE_SHIFT                (20u)
#define TimerOut_INV_OUT_SHIFT                  (20u)
#define TimerOut_INV_COMPL_OUT_SHIFT            (21u)
#define TimerOut_MODE_SHIFT                     (24u)

/* Mask constants for control register */
#define TimerOut_RELOAD_CC_MASK                 ((uint32)(TimerOut_1BIT_MASK        <<  \
                                                                            TimerOut_RELOAD_CC_SHIFT))
#define TimerOut_RELOAD_PERIOD_MASK             ((uint32)(TimerOut_1BIT_MASK        <<  \
                                                                            TimerOut_RELOAD_PERIOD_SHIFT))
#define TimerOut_PWM_SYNC_KILL_MASK             ((uint32)(TimerOut_1BIT_MASK        <<  \
                                                                            TimerOut_PWM_SYNC_KILL_SHIFT))
#define TimerOut_PWM_STOP_KILL_MASK             ((uint32)(TimerOut_1BIT_MASK        <<  \
                                                                            TimerOut_PWM_STOP_KILL_SHIFT))
#define TimerOut_PRESCALER_MASK                 ((uint32)(TimerOut_8BIT_MASK        <<  \
                                                                            TimerOut_PRESCALER_SHIFT))
#define TimerOut_UPDOWN_MASK                    ((uint32)(TimerOut_2BIT_MASK        <<  \
                                                                            TimerOut_UPDOWN_SHIFT))
#define TimerOut_ONESHOT_MASK                   ((uint32)(TimerOut_1BIT_MASK        <<  \
                                                                            TimerOut_ONESHOT_SHIFT))
#define TimerOut_QUAD_MODE_MASK                 ((uint32)(TimerOut_3BIT_MASK        <<  \
                                                                            TimerOut_QUAD_MODE_SHIFT))
#define TimerOut_INV_OUT_MASK                   ((uint32)(TimerOut_2BIT_MASK        <<  \
                                                                            TimerOut_INV_OUT_SHIFT))
#define TimerOut_MODE_MASK                      ((uint32)(TimerOut_3BIT_MASK        <<  \
                                                                            TimerOut_MODE_SHIFT))

/* Shift constants for trigger control register 1 */
#define TimerOut_CAPTURE_SHIFT                  (0u)
#define TimerOut_COUNT_SHIFT                    (2u)
#define TimerOut_RELOAD_SHIFT                   (4u)
#define TimerOut_STOP_SHIFT                     (6u)
#define TimerOut_START_SHIFT                    (8u)

/* Mask constants for trigger control register 1 */
#define TimerOut_CAPTURE_MASK                   ((uint32)(TimerOut_2BIT_MASK        <<  \
                                                                  TimerOut_CAPTURE_SHIFT))
#define TimerOut_COUNT_MASK                     ((uint32)(TimerOut_2BIT_MASK        <<  \
                                                                  TimerOut_COUNT_SHIFT))
#define TimerOut_RELOAD_MASK                    ((uint32)(TimerOut_2BIT_MASK        <<  \
                                                                  TimerOut_RELOAD_SHIFT))
#define TimerOut_STOP_MASK                      ((uint32)(TimerOut_2BIT_MASK        <<  \
                                                                  TimerOut_STOP_SHIFT))
#define TimerOut_START_MASK                     ((uint32)(TimerOut_2BIT_MASK        <<  \
                                                                  TimerOut_START_SHIFT))

/* MASK */
#define TimerOut_1BIT_MASK                      ((uint32)0x01u)
#define TimerOut_2BIT_MASK                      ((uint32)0x03u)
#define TimerOut_3BIT_MASK                      ((uint32)0x07u)
#define TimerOut_6BIT_MASK                      ((uint32)0x3Fu)
#define TimerOut_8BIT_MASK                      ((uint32)0xFFu)
#define TimerOut_16BIT_MASK                     ((uint32)0xFFFFu)

/* Shift constant for status register */
#define TimerOut_RUNNING_STATUS_SHIFT           (30u)


/***************************************
*    Initial Constants
***************************************/

#define TimerOut_CTRL_QUAD_BASE_CONFIG                                                          \
        (((uint32)(TimerOut_QUAD_ENCODING_MODES     << TimerOut_QUAD_MODE_SHIFT))       |\
         ((uint32)(TimerOut_CONFIG                  << TimerOut_MODE_SHIFT)))

#define TimerOut_CTRL_PWM_BASE_CONFIG                                                           \
        (((uint32)(TimerOut_PWM_STOP_EVENT          << TimerOut_PWM_STOP_KILL_SHIFT))   |\
         ((uint32)(TimerOut_PWM_OUT_INVERT          << TimerOut_INV_OUT_SHIFT))         |\
         ((uint32)(TimerOut_PWM_OUT_N_INVERT        << TimerOut_INV_COMPL_OUT_SHIFT))   |\
         ((uint32)(TimerOut_PWM_MODE                << TimerOut_MODE_SHIFT)))

#define TimerOut_CTRL_PWM_RUN_MODE                                                              \
            ((uint32)(TimerOut_PWM_RUN_MODE         << TimerOut_ONESHOT_SHIFT))
            
#define TimerOut_CTRL_PWM_ALIGN                                                                 \
            ((uint32)(TimerOut_PWM_ALIGN            << TimerOut_UPDOWN_SHIFT))

#define TimerOut_CTRL_PWM_KILL_EVENT                                                            \
             ((uint32)(TimerOut_PWM_KILL_EVENT      << TimerOut_PWM_SYNC_KILL_SHIFT))

#define TimerOut_CTRL_PWM_DEAD_TIME_CYCLE                                                       \
            ((uint32)(TimerOut_PWM_DEAD_TIME_CYCLE  << TimerOut_PRESCALER_SHIFT))

#define TimerOut_CTRL_PWM_PRESCALER                                                             \
            ((uint32)(TimerOut_PWM_PRESCALER        << TimerOut_PRESCALER_SHIFT))

#define TimerOut_CTRL_TIMER_BASE_CONFIG                                                         \
        (((uint32)(TimerOut_TC_PRESCALER            << TimerOut_PRESCALER_SHIFT))       |\
         ((uint32)(TimerOut_TC_COUNTER_MODE         << TimerOut_UPDOWN_SHIFT))          |\
         ((uint32)(TimerOut_TC_RUN_MODE             << TimerOut_ONESHOT_SHIFT))         |\
         ((uint32)(TimerOut_TC_COMP_CAP_MODE        << TimerOut_MODE_SHIFT)))
        
#define TimerOut_QUAD_SIGNALS_MODES                                                             \
        (((uint32)(TimerOut_QUAD_PHIA_SIGNAL_MODE   << TimerOut_COUNT_SHIFT))           |\
         ((uint32)(TimerOut_QUAD_INDEX_SIGNAL_MODE  << TimerOut_RELOAD_SHIFT))          |\
         ((uint32)(TimerOut_QUAD_STOP_SIGNAL_MODE   << TimerOut_STOP_SHIFT))            |\
         ((uint32)(TimerOut_QUAD_PHIB_SIGNAL_MODE   << TimerOut_START_SHIFT)))

#define TimerOut_PWM_SIGNALS_MODES                                                              \
        (((uint32)(TimerOut_PWM_SWITCH_SIGNAL_MODE  << TimerOut_CAPTURE_SHIFT))         |\
         ((uint32)(TimerOut_PWM_COUNT_SIGNAL_MODE   << TimerOut_COUNT_SHIFT))           |\
         ((uint32)(TimerOut_PWM_RELOAD_SIGNAL_MODE  << TimerOut_RELOAD_SHIFT))          |\
         ((uint32)(TimerOut_PWM_STOP_SIGNAL_MODE    << TimerOut_STOP_SHIFT))            |\
         ((uint32)(TimerOut_PWM_START_SIGNAL_MODE   << TimerOut_START_SHIFT)))

#define TimerOut_TIMER_SIGNALS_MODES                                                            \
        (((uint32)(TimerOut_TC_CAPTURE_SIGNAL_MODE  << TimerOut_CAPTURE_SHIFT))         |\
         ((uint32)(TimerOut_TC_COUNT_SIGNAL_MODE    << TimerOut_COUNT_SHIFT))           |\
         ((uint32)(TimerOut_TC_RELOAD_SIGNAL_MODE   << TimerOut_RELOAD_SHIFT))          |\
         ((uint32)(TimerOut_TC_STOP_SIGNAL_MODE     << TimerOut_STOP_SHIFT))            |\
         ((uint32)(TimerOut_TC_START_SIGNAL_MODE    << TimerOut_START_SHIFT)))
        
#define TimerOut_TIMER_UPDOWN_CNT_USED                                                          \
                ((TimerOut__COUNT_UPDOWN0 == TimerOut_TC_COUNTER_MODE)                  ||\
                 (TimerOut__COUNT_UPDOWN1 == TimerOut_TC_COUNTER_MODE))

#define TimerOut_PWM_UPDOWN_CNT_USED                                                            \
                ((TimerOut__CENTER == TimerOut_PWM_ALIGN)                               ||\
                 (TimerOut__ASYMMETRIC == TimerOut_PWM_ALIGN))               
        
#define TimerOut_PWM_PR_INIT_VALUE              (1u)
#define TimerOut_QUAD_PERIOD_INIT_VALUE         (0x8000u)



#endif /* End CY_TCPWM_TimerOut_H */

/* [] END OF FILE */
