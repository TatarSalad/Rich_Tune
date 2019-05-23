/*******************************************************************************
* File Name: MatrixTimer.h
* Version 2.10
*
* Description:
*  This file provides constants and parameter values for the MatrixTimer
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

#if !defined(CY_TCPWM_MatrixTimer_H)
#define CY_TCPWM_MatrixTimer_H


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
} MatrixTimer_BACKUP_STRUCT;


/*******************************************************************************
* Variables
*******************************************************************************/
extern uint8  MatrixTimer_initVar;


/***************************************
*   Conditional Compilation Parameters
****************************************/

#define MatrixTimer_CY_TCPWM_V2                    (CYIPBLOCK_m0s8tcpwm_VERSION == 2u)
#define MatrixTimer_CY_TCPWM_4000                  (CY_PSOC4_4000)

/* TCPWM Configuration */
#define MatrixTimer_CONFIG                         (1lu)

/* Quad Mode */
/* Parameters */
#define MatrixTimer_QUAD_ENCODING_MODES            (0lu)
#define MatrixTimer_QUAD_AUTO_START                (1lu)

/* Signal modes */
#define MatrixTimer_QUAD_INDEX_SIGNAL_MODE         (0lu)
#define MatrixTimer_QUAD_PHIA_SIGNAL_MODE          (3lu)
#define MatrixTimer_QUAD_PHIB_SIGNAL_MODE          (3lu)
#define MatrixTimer_QUAD_STOP_SIGNAL_MODE          (0lu)

/* Signal present */
#define MatrixTimer_QUAD_INDEX_SIGNAL_PRESENT      (0lu)
#define MatrixTimer_QUAD_STOP_SIGNAL_PRESENT       (0lu)

/* Interrupt Mask */
#define MatrixTimer_QUAD_INTERRUPT_MASK            (1lu)

/* Timer/Counter Mode */
/* Parameters */
#define MatrixTimer_TC_RUN_MODE                    (0lu)
#define MatrixTimer_TC_COUNTER_MODE                (0lu)
#define MatrixTimer_TC_COMP_CAP_MODE               (2lu)
#define MatrixTimer_TC_PRESCALER                   (0lu)

/* Signal modes */
#define MatrixTimer_TC_RELOAD_SIGNAL_MODE          (0lu)
#define MatrixTimer_TC_COUNT_SIGNAL_MODE           (3lu)
#define MatrixTimer_TC_START_SIGNAL_MODE           (0lu)
#define MatrixTimer_TC_STOP_SIGNAL_MODE            (0lu)
#define MatrixTimer_TC_CAPTURE_SIGNAL_MODE         (0lu)

/* Signal present */
#define MatrixTimer_TC_RELOAD_SIGNAL_PRESENT       (0lu)
#define MatrixTimer_TC_COUNT_SIGNAL_PRESENT        (0lu)
#define MatrixTimer_TC_START_SIGNAL_PRESENT        (0lu)
#define MatrixTimer_TC_STOP_SIGNAL_PRESENT         (0lu)
#define MatrixTimer_TC_CAPTURE_SIGNAL_PRESENT      (0lu)

/* Interrupt Mask */
#define MatrixTimer_TC_INTERRUPT_MASK              (1lu)

/* PWM Mode */
/* Parameters */
#define MatrixTimer_PWM_KILL_EVENT                 (0lu)
#define MatrixTimer_PWM_STOP_EVENT                 (0lu)
#define MatrixTimer_PWM_MODE                       (4lu)
#define MatrixTimer_PWM_OUT_N_INVERT               (0lu)
#define MatrixTimer_PWM_OUT_INVERT                 (0lu)
#define MatrixTimer_PWM_ALIGN                      (0lu)
#define MatrixTimer_PWM_RUN_MODE                   (0lu)
#define MatrixTimer_PWM_DEAD_TIME_CYCLE            (0lu)
#define MatrixTimer_PWM_PRESCALER                  (0lu)

/* Signal modes */
#define MatrixTimer_PWM_RELOAD_SIGNAL_MODE         (0lu)
#define MatrixTimer_PWM_COUNT_SIGNAL_MODE          (3lu)
#define MatrixTimer_PWM_START_SIGNAL_MODE          (0lu)
#define MatrixTimer_PWM_STOP_SIGNAL_MODE           (0lu)
#define MatrixTimer_PWM_SWITCH_SIGNAL_MODE         (0lu)

/* Signal present */
#define MatrixTimer_PWM_RELOAD_SIGNAL_PRESENT      (0lu)
#define MatrixTimer_PWM_COUNT_SIGNAL_PRESENT       (0lu)
#define MatrixTimer_PWM_START_SIGNAL_PRESENT       (0lu)
#define MatrixTimer_PWM_STOP_SIGNAL_PRESENT        (0lu)
#define MatrixTimer_PWM_SWITCH_SIGNAL_PRESENT      (0lu)

/* Interrupt Mask */
#define MatrixTimer_PWM_INTERRUPT_MASK             (1lu)


/***************************************
*    Initial Parameter Constants
***************************************/

/* Timer/Counter Mode */
#define MatrixTimer_TC_PERIOD_VALUE                (500lu)
#define MatrixTimer_TC_COMPARE_VALUE               (65535lu)
#define MatrixTimer_TC_COMPARE_BUF_VALUE           (65535lu)
#define MatrixTimer_TC_COMPARE_SWAP                (0lu)

/* PWM Mode */
#define MatrixTimer_PWM_PERIOD_VALUE               (65535lu)
#define MatrixTimer_PWM_PERIOD_BUF_VALUE           (65535lu)
#define MatrixTimer_PWM_PERIOD_SWAP                (0lu)
#define MatrixTimer_PWM_COMPARE_VALUE              (65535lu)
#define MatrixTimer_PWM_COMPARE_BUF_VALUE          (65535lu)
#define MatrixTimer_PWM_COMPARE_SWAP               (0lu)


/***************************************
*    Enumerated Types and Parameters
***************************************/

#define MatrixTimer__LEFT 0
#define MatrixTimer__RIGHT 1
#define MatrixTimer__CENTER 2
#define MatrixTimer__ASYMMETRIC 3

#define MatrixTimer__X1 0
#define MatrixTimer__X2 1
#define MatrixTimer__X4 2

#define MatrixTimer__PWM 4
#define MatrixTimer__PWM_DT 5
#define MatrixTimer__PWM_PR 6

#define MatrixTimer__INVERSE 1
#define MatrixTimer__DIRECT 0

#define MatrixTimer__CAPTURE 2
#define MatrixTimer__COMPARE 0

#define MatrixTimer__TRIG_LEVEL 3
#define MatrixTimer__TRIG_RISING 0
#define MatrixTimer__TRIG_FALLING 1
#define MatrixTimer__TRIG_BOTH 2

#define MatrixTimer__INTR_MASK_TC 1
#define MatrixTimer__INTR_MASK_CC_MATCH 2
#define MatrixTimer__INTR_MASK_NONE 0
#define MatrixTimer__INTR_MASK_TC_CC 3

#define MatrixTimer__UNCONFIG 8
#define MatrixTimer__TIMER 1
#define MatrixTimer__QUAD 3
#define MatrixTimer__PWM_SEL 7

#define MatrixTimer__COUNT_UP 0
#define MatrixTimer__COUNT_DOWN 1
#define MatrixTimer__COUNT_UPDOWN0 2
#define MatrixTimer__COUNT_UPDOWN1 3


/* Prescaler */
#define MatrixTimer_PRESCALE_DIVBY1                ((uint32)(0u << MatrixTimer_PRESCALER_SHIFT))
#define MatrixTimer_PRESCALE_DIVBY2                ((uint32)(1u << MatrixTimer_PRESCALER_SHIFT))
#define MatrixTimer_PRESCALE_DIVBY4                ((uint32)(2u << MatrixTimer_PRESCALER_SHIFT))
#define MatrixTimer_PRESCALE_DIVBY8                ((uint32)(3u << MatrixTimer_PRESCALER_SHIFT))
#define MatrixTimer_PRESCALE_DIVBY16               ((uint32)(4u << MatrixTimer_PRESCALER_SHIFT))
#define MatrixTimer_PRESCALE_DIVBY32               ((uint32)(5u << MatrixTimer_PRESCALER_SHIFT))
#define MatrixTimer_PRESCALE_DIVBY64               ((uint32)(6u << MatrixTimer_PRESCALER_SHIFT))
#define MatrixTimer_PRESCALE_DIVBY128              ((uint32)(7u << MatrixTimer_PRESCALER_SHIFT))

/* TCPWM set modes */
#define MatrixTimer_MODE_TIMER_COMPARE             ((uint32)(MatrixTimer__COMPARE         <<  \
                                                                  MatrixTimer_MODE_SHIFT))
#define MatrixTimer_MODE_TIMER_CAPTURE             ((uint32)(MatrixTimer__CAPTURE         <<  \
                                                                  MatrixTimer_MODE_SHIFT))
#define MatrixTimer_MODE_QUAD                      ((uint32)(MatrixTimer__QUAD            <<  \
                                                                  MatrixTimer_MODE_SHIFT))
#define MatrixTimer_MODE_PWM                       ((uint32)(MatrixTimer__PWM             <<  \
                                                                  MatrixTimer_MODE_SHIFT))
#define MatrixTimer_MODE_PWM_DT                    ((uint32)(MatrixTimer__PWM_DT          <<  \
                                                                  MatrixTimer_MODE_SHIFT))
#define MatrixTimer_MODE_PWM_PR                    ((uint32)(MatrixTimer__PWM_PR          <<  \
                                                                  MatrixTimer_MODE_SHIFT))

/* Quad Modes */
#define MatrixTimer_MODE_X1                        ((uint32)(MatrixTimer__X1              <<  \
                                                                  MatrixTimer_QUAD_MODE_SHIFT))
#define MatrixTimer_MODE_X2                        ((uint32)(MatrixTimer__X2              <<  \
                                                                  MatrixTimer_QUAD_MODE_SHIFT))
#define MatrixTimer_MODE_X4                        ((uint32)(MatrixTimer__X4              <<  \
                                                                  MatrixTimer_QUAD_MODE_SHIFT))

/* Counter modes */
#define MatrixTimer_COUNT_UP                       ((uint32)(MatrixTimer__COUNT_UP        <<  \
                                                                  MatrixTimer_UPDOWN_SHIFT))
#define MatrixTimer_COUNT_DOWN                     ((uint32)(MatrixTimer__COUNT_DOWN      <<  \
                                                                  MatrixTimer_UPDOWN_SHIFT))
#define MatrixTimer_COUNT_UPDOWN0                  ((uint32)(MatrixTimer__COUNT_UPDOWN0   <<  \
                                                                  MatrixTimer_UPDOWN_SHIFT))
#define MatrixTimer_COUNT_UPDOWN1                  ((uint32)(MatrixTimer__COUNT_UPDOWN1   <<  \
                                                                  MatrixTimer_UPDOWN_SHIFT))

/* PWM output invert */
#define MatrixTimer_INVERT_LINE                    ((uint32)(MatrixTimer__INVERSE         <<  \
                                                                  MatrixTimer_INV_OUT_SHIFT))
#define MatrixTimer_INVERT_LINE_N                  ((uint32)(MatrixTimer__INVERSE         <<  \
                                                                  MatrixTimer_INV_COMPL_OUT_SHIFT))

/* Trigger modes */
#define MatrixTimer_TRIG_RISING                    ((uint32)MatrixTimer__TRIG_RISING)
#define MatrixTimer_TRIG_FALLING                   ((uint32)MatrixTimer__TRIG_FALLING)
#define MatrixTimer_TRIG_BOTH                      ((uint32)MatrixTimer__TRIG_BOTH)
#define MatrixTimer_TRIG_LEVEL                     ((uint32)MatrixTimer__TRIG_LEVEL)

/* Interrupt mask */
#define MatrixTimer_INTR_MASK_TC                   ((uint32)MatrixTimer__INTR_MASK_TC)
#define MatrixTimer_INTR_MASK_CC_MATCH             ((uint32)MatrixTimer__INTR_MASK_CC_MATCH)

/* PWM Output Controls */
#define MatrixTimer_CC_MATCH_SET                   (0x00u)
#define MatrixTimer_CC_MATCH_CLEAR                 (0x01u)
#define MatrixTimer_CC_MATCH_INVERT                (0x02u)
#define MatrixTimer_CC_MATCH_NO_CHANGE             (0x03u)
#define MatrixTimer_OVERLOW_SET                    (0x00u)
#define MatrixTimer_OVERLOW_CLEAR                  (0x04u)
#define MatrixTimer_OVERLOW_INVERT                 (0x08u)
#define MatrixTimer_OVERLOW_NO_CHANGE              (0x0Cu)
#define MatrixTimer_UNDERFLOW_SET                  (0x00u)
#define MatrixTimer_UNDERFLOW_CLEAR                (0x10u)
#define MatrixTimer_UNDERFLOW_INVERT               (0x20u)
#define MatrixTimer_UNDERFLOW_NO_CHANGE            (0x30u)

/* PWM Align */
#define MatrixTimer_PWM_MODE_LEFT                  (MatrixTimer_CC_MATCH_CLEAR        |   \
                                                         MatrixTimer_OVERLOW_SET           |   \
                                                         MatrixTimer_UNDERFLOW_NO_CHANGE)
#define MatrixTimer_PWM_MODE_RIGHT                 (MatrixTimer_CC_MATCH_SET          |   \
                                                         MatrixTimer_OVERLOW_NO_CHANGE     |   \
                                                         MatrixTimer_UNDERFLOW_CLEAR)
#define MatrixTimer_PWM_MODE_ASYM                  (MatrixTimer_CC_MATCH_INVERT       |   \
                                                         MatrixTimer_OVERLOW_SET           |   \
                                                         MatrixTimer_UNDERFLOW_CLEAR)

#if (MatrixTimer_CY_TCPWM_V2)
    #if(MatrixTimer_CY_TCPWM_4000)
        #define MatrixTimer_PWM_MODE_CENTER                (MatrixTimer_CC_MATCH_INVERT       |   \
                                                                 MatrixTimer_OVERLOW_NO_CHANGE     |   \
                                                                 MatrixTimer_UNDERFLOW_CLEAR)
    #else
        #define MatrixTimer_PWM_MODE_CENTER                (MatrixTimer_CC_MATCH_INVERT       |   \
                                                                 MatrixTimer_OVERLOW_SET           |   \
                                                                 MatrixTimer_UNDERFLOW_CLEAR)
    #endif /* (MatrixTimer_CY_TCPWM_4000) */
#else
    #define MatrixTimer_PWM_MODE_CENTER                (MatrixTimer_CC_MATCH_INVERT       |   \
                                                             MatrixTimer_OVERLOW_NO_CHANGE     |   \
                                                             MatrixTimer_UNDERFLOW_CLEAR)
#endif /* (MatrixTimer_CY_TCPWM_NEW) */

/* Command operations without condition */
#define MatrixTimer_CMD_CAPTURE                    (0u)
#define MatrixTimer_CMD_RELOAD                     (8u)
#define MatrixTimer_CMD_STOP                       (16u)
#define MatrixTimer_CMD_START                      (24u)

/* Status */
#define MatrixTimer_STATUS_DOWN                    (1u)
#define MatrixTimer_STATUS_RUNNING                 (2u)


/***************************************
*        Function Prototypes
****************************************/

void   MatrixTimer_Init(void);
void   MatrixTimer_Enable(void);
void   MatrixTimer_Start(void);
void   MatrixTimer_Stop(void);

void   MatrixTimer_SetMode(uint32 mode);
void   MatrixTimer_SetCounterMode(uint32 counterMode);
void   MatrixTimer_SetPWMMode(uint32 modeMask);
void   MatrixTimer_SetQDMode(uint32 qdMode);

void   MatrixTimer_SetPrescaler(uint32 prescaler);
void   MatrixTimer_TriggerCommand(uint32 mask, uint32 command);
void   MatrixTimer_SetOneShot(uint32 oneShotEnable);
uint32 MatrixTimer_ReadStatus(void);

void   MatrixTimer_SetPWMSyncKill(uint32 syncKillEnable);
void   MatrixTimer_SetPWMStopOnKill(uint32 stopOnKillEnable);
void   MatrixTimer_SetPWMDeadTime(uint32 deadTime);
void   MatrixTimer_SetPWMInvert(uint32 mask);

void   MatrixTimer_SetInterruptMode(uint32 interruptMask);
uint32 MatrixTimer_GetInterruptSourceMasked(void);
uint32 MatrixTimer_GetInterruptSource(void);
void   MatrixTimer_ClearInterrupt(uint32 interruptMask);
void   MatrixTimer_SetInterrupt(uint32 interruptMask);

void   MatrixTimer_WriteCounter(uint32 count);
uint32 MatrixTimer_ReadCounter(void);

uint32 MatrixTimer_ReadCapture(void);
uint32 MatrixTimer_ReadCaptureBuf(void);

void   MatrixTimer_WritePeriod(uint32 period);
uint32 MatrixTimer_ReadPeriod(void);
void   MatrixTimer_WritePeriodBuf(uint32 periodBuf);
uint32 MatrixTimer_ReadPeriodBuf(void);

void   MatrixTimer_WriteCompare(uint32 compare);
uint32 MatrixTimer_ReadCompare(void);
void   MatrixTimer_WriteCompareBuf(uint32 compareBuf);
uint32 MatrixTimer_ReadCompareBuf(void);

void   MatrixTimer_SetPeriodSwap(uint32 swapEnable);
void   MatrixTimer_SetCompareSwap(uint32 swapEnable);

void   MatrixTimer_SetCaptureMode(uint32 triggerMode);
void   MatrixTimer_SetReloadMode(uint32 triggerMode);
void   MatrixTimer_SetStartMode(uint32 triggerMode);
void   MatrixTimer_SetStopMode(uint32 triggerMode);
void   MatrixTimer_SetCountMode(uint32 triggerMode);

void   MatrixTimer_SaveConfig(void);
void   MatrixTimer_RestoreConfig(void);
void   MatrixTimer_Sleep(void);
void   MatrixTimer_Wakeup(void);


/***************************************
*             Registers
***************************************/

#define MatrixTimer_BLOCK_CONTROL_REG              (*(reg32 *) MatrixTimer_cy_m0s8_tcpwm_1__TCPWM_CTRL )
#define MatrixTimer_BLOCK_CONTROL_PTR              ( (reg32 *) MatrixTimer_cy_m0s8_tcpwm_1__TCPWM_CTRL )
#define MatrixTimer_COMMAND_REG                    (*(reg32 *) MatrixTimer_cy_m0s8_tcpwm_1__TCPWM_CMD )
#define MatrixTimer_COMMAND_PTR                    ( (reg32 *) MatrixTimer_cy_m0s8_tcpwm_1__TCPWM_CMD )
#define MatrixTimer_INTRRUPT_CAUSE_REG             (*(reg32 *) MatrixTimer_cy_m0s8_tcpwm_1__TCPWM_INTR_CAUSE )
#define MatrixTimer_INTRRUPT_CAUSE_PTR             ( (reg32 *) MatrixTimer_cy_m0s8_tcpwm_1__TCPWM_INTR_CAUSE )
#define MatrixTimer_CONTROL_REG                    (*(reg32 *) MatrixTimer_cy_m0s8_tcpwm_1__CTRL )
#define MatrixTimer_CONTROL_PTR                    ( (reg32 *) MatrixTimer_cy_m0s8_tcpwm_1__CTRL )
#define MatrixTimer_STATUS_REG                     (*(reg32 *) MatrixTimer_cy_m0s8_tcpwm_1__STATUS )
#define MatrixTimer_STATUS_PTR                     ( (reg32 *) MatrixTimer_cy_m0s8_tcpwm_1__STATUS )
#define MatrixTimer_COUNTER_REG                    (*(reg32 *) MatrixTimer_cy_m0s8_tcpwm_1__COUNTER )
#define MatrixTimer_COUNTER_PTR                    ( (reg32 *) MatrixTimer_cy_m0s8_tcpwm_1__COUNTER )
#define MatrixTimer_COMP_CAP_REG                   (*(reg32 *) MatrixTimer_cy_m0s8_tcpwm_1__CC )
#define MatrixTimer_COMP_CAP_PTR                   ( (reg32 *) MatrixTimer_cy_m0s8_tcpwm_1__CC )
#define MatrixTimer_COMP_CAP_BUF_REG               (*(reg32 *) MatrixTimer_cy_m0s8_tcpwm_1__CC_BUFF )
#define MatrixTimer_COMP_CAP_BUF_PTR               ( (reg32 *) MatrixTimer_cy_m0s8_tcpwm_1__CC_BUFF )
#define MatrixTimer_PERIOD_REG                     (*(reg32 *) MatrixTimer_cy_m0s8_tcpwm_1__PERIOD )
#define MatrixTimer_PERIOD_PTR                     ( (reg32 *) MatrixTimer_cy_m0s8_tcpwm_1__PERIOD )
#define MatrixTimer_PERIOD_BUF_REG                 (*(reg32 *) MatrixTimer_cy_m0s8_tcpwm_1__PERIOD_BUFF )
#define MatrixTimer_PERIOD_BUF_PTR                 ( (reg32 *) MatrixTimer_cy_m0s8_tcpwm_1__PERIOD_BUFF )
#define MatrixTimer_TRIG_CONTROL0_REG              (*(reg32 *) MatrixTimer_cy_m0s8_tcpwm_1__TR_CTRL0 )
#define MatrixTimer_TRIG_CONTROL0_PTR              ( (reg32 *) MatrixTimer_cy_m0s8_tcpwm_1__TR_CTRL0 )
#define MatrixTimer_TRIG_CONTROL1_REG              (*(reg32 *) MatrixTimer_cy_m0s8_tcpwm_1__TR_CTRL1 )
#define MatrixTimer_TRIG_CONTROL1_PTR              ( (reg32 *) MatrixTimer_cy_m0s8_tcpwm_1__TR_CTRL1 )
#define MatrixTimer_TRIG_CONTROL2_REG              (*(reg32 *) MatrixTimer_cy_m0s8_tcpwm_1__TR_CTRL2 )
#define MatrixTimer_TRIG_CONTROL2_PTR              ( (reg32 *) MatrixTimer_cy_m0s8_tcpwm_1__TR_CTRL2 )
#define MatrixTimer_INTERRUPT_REQ_REG              (*(reg32 *) MatrixTimer_cy_m0s8_tcpwm_1__INTR )
#define MatrixTimer_INTERRUPT_REQ_PTR              ( (reg32 *) MatrixTimer_cy_m0s8_tcpwm_1__INTR )
#define MatrixTimer_INTERRUPT_SET_REG              (*(reg32 *) MatrixTimer_cy_m0s8_tcpwm_1__INTR_SET )
#define MatrixTimer_INTERRUPT_SET_PTR              ( (reg32 *) MatrixTimer_cy_m0s8_tcpwm_1__INTR_SET )
#define MatrixTimer_INTERRUPT_MASK_REG             (*(reg32 *) MatrixTimer_cy_m0s8_tcpwm_1__INTR_MASK )
#define MatrixTimer_INTERRUPT_MASK_PTR             ( (reg32 *) MatrixTimer_cy_m0s8_tcpwm_1__INTR_MASK )
#define MatrixTimer_INTERRUPT_MASKED_REG           (*(reg32 *) MatrixTimer_cy_m0s8_tcpwm_1__INTR_MASKED )
#define MatrixTimer_INTERRUPT_MASKED_PTR           ( (reg32 *) MatrixTimer_cy_m0s8_tcpwm_1__INTR_MASKED )


/***************************************
*       Registers Constants
***************************************/

/* Mask */
#define MatrixTimer_MASK                           ((uint32)MatrixTimer_cy_m0s8_tcpwm_1__TCPWM_CTRL_MASK)

/* Shift constants for control register */
#define MatrixTimer_RELOAD_CC_SHIFT                (0u)
#define MatrixTimer_RELOAD_PERIOD_SHIFT            (1u)
#define MatrixTimer_PWM_SYNC_KILL_SHIFT            (2u)
#define MatrixTimer_PWM_STOP_KILL_SHIFT            (3u)
#define MatrixTimer_PRESCALER_SHIFT                (8u)
#define MatrixTimer_UPDOWN_SHIFT                   (16u)
#define MatrixTimer_ONESHOT_SHIFT                  (18u)
#define MatrixTimer_QUAD_MODE_SHIFT                (20u)
#define MatrixTimer_INV_OUT_SHIFT                  (20u)
#define MatrixTimer_INV_COMPL_OUT_SHIFT            (21u)
#define MatrixTimer_MODE_SHIFT                     (24u)

/* Mask constants for control register */
#define MatrixTimer_RELOAD_CC_MASK                 ((uint32)(MatrixTimer_1BIT_MASK        <<  \
                                                                            MatrixTimer_RELOAD_CC_SHIFT))
#define MatrixTimer_RELOAD_PERIOD_MASK             ((uint32)(MatrixTimer_1BIT_MASK        <<  \
                                                                            MatrixTimer_RELOAD_PERIOD_SHIFT))
#define MatrixTimer_PWM_SYNC_KILL_MASK             ((uint32)(MatrixTimer_1BIT_MASK        <<  \
                                                                            MatrixTimer_PWM_SYNC_KILL_SHIFT))
#define MatrixTimer_PWM_STOP_KILL_MASK             ((uint32)(MatrixTimer_1BIT_MASK        <<  \
                                                                            MatrixTimer_PWM_STOP_KILL_SHIFT))
#define MatrixTimer_PRESCALER_MASK                 ((uint32)(MatrixTimer_8BIT_MASK        <<  \
                                                                            MatrixTimer_PRESCALER_SHIFT))
#define MatrixTimer_UPDOWN_MASK                    ((uint32)(MatrixTimer_2BIT_MASK        <<  \
                                                                            MatrixTimer_UPDOWN_SHIFT))
#define MatrixTimer_ONESHOT_MASK                   ((uint32)(MatrixTimer_1BIT_MASK        <<  \
                                                                            MatrixTimer_ONESHOT_SHIFT))
#define MatrixTimer_QUAD_MODE_MASK                 ((uint32)(MatrixTimer_3BIT_MASK        <<  \
                                                                            MatrixTimer_QUAD_MODE_SHIFT))
#define MatrixTimer_INV_OUT_MASK                   ((uint32)(MatrixTimer_2BIT_MASK        <<  \
                                                                            MatrixTimer_INV_OUT_SHIFT))
#define MatrixTimer_MODE_MASK                      ((uint32)(MatrixTimer_3BIT_MASK        <<  \
                                                                            MatrixTimer_MODE_SHIFT))

/* Shift constants for trigger control register 1 */
#define MatrixTimer_CAPTURE_SHIFT                  (0u)
#define MatrixTimer_COUNT_SHIFT                    (2u)
#define MatrixTimer_RELOAD_SHIFT                   (4u)
#define MatrixTimer_STOP_SHIFT                     (6u)
#define MatrixTimer_START_SHIFT                    (8u)

/* Mask constants for trigger control register 1 */
#define MatrixTimer_CAPTURE_MASK                   ((uint32)(MatrixTimer_2BIT_MASK        <<  \
                                                                  MatrixTimer_CAPTURE_SHIFT))
#define MatrixTimer_COUNT_MASK                     ((uint32)(MatrixTimer_2BIT_MASK        <<  \
                                                                  MatrixTimer_COUNT_SHIFT))
#define MatrixTimer_RELOAD_MASK                    ((uint32)(MatrixTimer_2BIT_MASK        <<  \
                                                                  MatrixTimer_RELOAD_SHIFT))
#define MatrixTimer_STOP_MASK                      ((uint32)(MatrixTimer_2BIT_MASK        <<  \
                                                                  MatrixTimer_STOP_SHIFT))
#define MatrixTimer_START_MASK                     ((uint32)(MatrixTimer_2BIT_MASK        <<  \
                                                                  MatrixTimer_START_SHIFT))

/* MASK */
#define MatrixTimer_1BIT_MASK                      ((uint32)0x01u)
#define MatrixTimer_2BIT_MASK                      ((uint32)0x03u)
#define MatrixTimer_3BIT_MASK                      ((uint32)0x07u)
#define MatrixTimer_6BIT_MASK                      ((uint32)0x3Fu)
#define MatrixTimer_8BIT_MASK                      ((uint32)0xFFu)
#define MatrixTimer_16BIT_MASK                     ((uint32)0xFFFFu)

/* Shift constant for status register */
#define MatrixTimer_RUNNING_STATUS_SHIFT           (30u)


/***************************************
*    Initial Constants
***************************************/

#define MatrixTimer_CTRL_QUAD_BASE_CONFIG                                                          \
        (((uint32)(MatrixTimer_QUAD_ENCODING_MODES     << MatrixTimer_QUAD_MODE_SHIFT))       |\
         ((uint32)(MatrixTimer_CONFIG                  << MatrixTimer_MODE_SHIFT)))

#define MatrixTimer_CTRL_PWM_BASE_CONFIG                                                           \
        (((uint32)(MatrixTimer_PWM_STOP_EVENT          << MatrixTimer_PWM_STOP_KILL_SHIFT))   |\
         ((uint32)(MatrixTimer_PWM_OUT_INVERT          << MatrixTimer_INV_OUT_SHIFT))         |\
         ((uint32)(MatrixTimer_PWM_OUT_N_INVERT        << MatrixTimer_INV_COMPL_OUT_SHIFT))   |\
         ((uint32)(MatrixTimer_PWM_MODE                << MatrixTimer_MODE_SHIFT)))

#define MatrixTimer_CTRL_PWM_RUN_MODE                                                              \
            ((uint32)(MatrixTimer_PWM_RUN_MODE         << MatrixTimer_ONESHOT_SHIFT))
            
#define MatrixTimer_CTRL_PWM_ALIGN                                                                 \
            ((uint32)(MatrixTimer_PWM_ALIGN            << MatrixTimer_UPDOWN_SHIFT))

#define MatrixTimer_CTRL_PWM_KILL_EVENT                                                            \
             ((uint32)(MatrixTimer_PWM_KILL_EVENT      << MatrixTimer_PWM_SYNC_KILL_SHIFT))

#define MatrixTimer_CTRL_PWM_DEAD_TIME_CYCLE                                                       \
            ((uint32)(MatrixTimer_PWM_DEAD_TIME_CYCLE  << MatrixTimer_PRESCALER_SHIFT))

#define MatrixTimer_CTRL_PWM_PRESCALER                                                             \
            ((uint32)(MatrixTimer_PWM_PRESCALER        << MatrixTimer_PRESCALER_SHIFT))

#define MatrixTimer_CTRL_TIMER_BASE_CONFIG                                                         \
        (((uint32)(MatrixTimer_TC_PRESCALER            << MatrixTimer_PRESCALER_SHIFT))       |\
         ((uint32)(MatrixTimer_TC_COUNTER_MODE         << MatrixTimer_UPDOWN_SHIFT))          |\
         ((uint32)(MatrixTimer_TC_RUN_MODE             << MatrixTimer_ONESHOT_SHIFT))         |\
         ((uint32)(MatrixTimer_TC_COMP_CAP_MODE        << MatrixTimer_MODE_SHIFT)))
        
#define MatrixTimer_QUAD_SIGNALS_MODES                                                             \
        (((uint32)(MatrixTimer_QUAD_PHIA_SIGNAL_MODE   << MatrixTimer_COUNT_SHIFT))           |\
         ((uint32)(MatrixTimer_QUAD_INDEX_SIGNAL_MODE  << MatrixTimer_RELOAD_SHIFT))          |\
         ((uint32)(MatrixTimer_QUAD_STOP_SIGNAL_MODE   << MatrixTimer_STOP_SHIFT))            |\
         ((uint32)(MatrixTimer_QUAD_PHIB_SIGNAL_MODE   << MatrixTimer_START_SHIFT)))

#define MatrixTimer_PWM_SIGNALS_MODES                                                              \
        (((uint32)(MatrixTimer_PWM_SWITCH_SIGNAL_MODE  << MatrixTimer_CAPTURE_SHIFT))         |\
         ((uint32)(MatrixTimer_PWM_COUNT_SIGNAL_MODE   << MatrixTimer_COUNT_SHIFT))           |\
         ((uint32)(MatrixTimer_PWM_RELOAD_SIGNAL_MODE  << MatrixTimer_RELOAD_SHIFT))          |\
         ((uint32)(MatrixTimer_PWM_STOP_SIGNAL_MODE    << MatrixTimer_STOP_SHIFT))            |\
         ((uint32)(MatrixTimer_PWM_START_SIGNAL_MODE   << MatrixTimer_START_SHIFT)))

#define MatrixTimer_TIMER_SIGNALS_MODES                                                            \
        (((uint32)(MatrixTimer_TC_CAPTURE_SIGNAL_MODE  << MatrixTimer_CAPTURE_SHIFT))         |\
         ((uint32)(MatrixTimer_TC_COUNT_SIGNAL_MODE    << MatrixTimer_COUNT_SHIFT))           |\
         ((uint32)(MatrixTimer_TC_RELOAD_SIGNAL_MODE   << MatrixTimer_RELOAD_SHIFT))          |\
         ((uint32)(MatrixTimer_TC_STOP_SIGNAL_MODE     << MatrixTimer_STOP_SHIFT))            |\
         ((uint32)(MatrixTimer_TC_START_SIGNAL_MODE    << MatrixTimer_START_SHIFT)))
        
#define MatrixTimer_TIMER_UPDOWN_CNT_USED                                                          \
                ((MatrixTimer__COUNT_UPDOWN0 == MatrixTimer_TC_COUNTER_MODE)                  ||\
                 (MatrixTimer__COUNT_UPDOWN1 == MatrixTimer_TC_COUNTER_MODE))

#define MatrixTimer_PWM_UPDOWN_CNT_USED                                                            \
                ((MatrixTimer__CENTER == MatrixTimer_PWM_ALIGN)                               ||\
                 (MatrixTimer__ASYMMETRIC == MatrixTimer_PWM_ALIGN))               
        
#define MatrixTimer_PWM_PR_INIT_VALUE              (1u)
#define MatrixTimer_QUAD_PERIOD_INIT_VALUE         (0x8000u)



#endif /* End CY_TCPWM_MatrixTimer_H */

/* [] END OF FILE */
