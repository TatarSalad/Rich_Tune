/*******************************************************************************
* File Name: TimeOut.h
* Version 2.10
*
* Description:
*  This file provides constants and parameter values for the TimeOut
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

#if !defined(CY_TCPWM_TimeOut_H)
#define CY_TCPWM_TimeOut_H


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
} TimeOut_BACKUP_STRUCT;


/*******************************************************************************
* Variables
*******************************************************************************/
extern uint8  TimeOut_initVar;


/***************************************
*   Conditional Compilation Parameters
****************************************/

#define TimeOut_CY_TCPWM_V2                    (CYIPBLOCK_m0s8tcpwm_VERSION == 2u)
#define TimeOut_CY_TCPWM_4000                  (CY_PSOC4_4000)

/* TCPWM Configuration */
#define TimeOut_CONFIG                         (1lu)

/* Quad Mode */
/* Parameters */
#define TimeOut_QUAD_ENCODING_MODES            (0lu)
#define TimeOut_QUAD_AUTO_START                (1lu)

/* Signal modes */
#define TimeOut_QUAD_INDEX_SIGNAL_MODE         (0lu)
#define TimeOut_QUAD_PHIA_SIGNAL_MODE          (3lu)
#define TimeOut_QUAD_PHIB_SIGNAL_MODE          (3lu)
#define TimeOut_QUAD_STOP_SIGNAL_MODE          (0lu)

/* Signal present */
#define TimeOut_QUAD_INDEX_SIGNAL_PRESENT      (0lu)
#define TimeOut_QUAD_STOP_SIGNAL_PRESENT       (0lu)

/* Interrupt Mask */
#define TimeOut_QUAD_INTERRUPT_MASK            (1lu)

/* Timer/Counter Mode */
/* Parameters */
#define TimeOut_TC_RUN_MODE                    (1lu)
#define TimeOut_TC_COUNTER_MODE                (0lu)
#define TimeOut_TC_COMP_CAP_MODE               (2lu)
#define TimeOut_TC_PRESCALER                   (7lu)

/* Signal modes */
#define TimeOut_TC_RELOAD_SIGNAL_MODE          (0lu)
#define TimeOut_TC_COUNT_SIGNAL_MODE           (3lu)
#define TimeOut_TC_START_SIGNAL_MODE           (0lu)
#define TimeOut_TC_STOP_SIGNAL_MODE            (0lu)
#define TimeOut_TC_CAPTURE_SIGNAL_MODE         (0lu)

/* Signal present */
#define TimeOut_TC_RELOAD_SIGNAL_PRESENT       (0lu)
#define TimeOut_TC_COUNT_SIGNAL_PRESENT        (0lu)
#define TimeOut_TC_START_SIGNAL_PRESENT        (0lu)
#define TimeOut_TC_STOP_SIGNAL_PRESENT         (0lu)
#define TimeOut_TC_CAPTURE_SIGNAL_PRESENT      (0lu)

/* Interrupt Mask */
#define TimeOut_TC_INTERRUPT_MASK              (1lu)

/* PWM Mode */
/* Parameters */
#define TimeOut_PWM_KILL_EVENT                 (0lu)
#define TimeOut_PWM_STOP_EVENT                 (0lu)
#define TimeOut_PWM_MODE                       (4lu)
#define TimeOut_PWM_OUT_N_INVERT               (0lu)
#define TimeOut_PWM_OUT_INVERT                 (0lu)
#define TimeOut_PWM_ALIGN                      (0lu)
#define TimeOut_PWM_RUN_MODE                   (0lu)
#define TimeOut_PWM_DEAD_TIME_CYCLE            (0lu)
#define TimeOut_PWM_PRESCALER                  (0lu)

/* Signal modes */
#define TimeOut_PWM_RELOAD_SIGNAL_MODE         (0lu)
#define TimeOut_PWM_COUNT_SIGNAL_MODE          (3lu)
#define TimeOut_PWM_START_SIGNAL_MODE          (0lu)
#define TimeOut_PWM_STOP_SIGNAL_MODE           (0lu)
#define TimeOut_PWM_SWITCH_SIGNAL_MODE         (0lu)

/* Signal present */
#define TimeOut_PWM_RELOAD_SIGNAL_PRESENT      (0lu)
#define TimeOut_PWM_COUNT_SIGNAL_PRESENT       (0lu)
#define TimeOut_PWM_START_SIGNAL_PRESENT       (0lu)
#define TimeOut_PWM_STOP_SIGNAL_PRESENT        (0lu)
#define TimeOut_PWM_SWITCH_SIGNAL_PRESENT      (0lu)

/* Interrupt Mask */
#define TimeOut_PWM_INTERRUPT_MASK             (1lu)


/***************************************
*    Initial Parameter Constants
***************************************/

/* Timer/Counter Mode */
#define TimeOut_TC_PERIOD_VALUE                (700lu)
#define TimeOut_TC_COMPARE_VALUE               (65535lu)
#define TimeOut_TC_COMPARE_BUF_VALUE           (65535lu)
#define TimeOut_TC_COMPARE_SWAP                (0lu)

/* PWM Mode */
#define TimeOut_PWM_PERIOD_VALUE               (65535lu)
#define TimeOut_PWM_PERIOD_BUF_VALUE           (65535lu)
#define TimeOut_PWM_PERIOD_SWAP                (0lu)
#define TimeOut_PWM_COMPARE_VALUE              (65535lu)
#define TimeOut_PWM_COMPARE_BUF_VALUE          (65535lu)
#define TimeOut_PWM_COMPARE_SWAP               (0lu)


/***************************************
*    Enumerated Types and Parameters
***************************************/

#define TimeOut__LEFT 0
#define TimeOut__RIGHT 1
#define TimeOut__CENTER 2
#define TimeOut__ASYMMETRIC 3

#define TimeOut__X1 0
#define TimeOut__X2 1
#define TimeOut__X4 2

#define TimeOut__PWM 4
#define TimeOut__PWM_DT 5
#define TimeOut__PWM_PR 6

#define TimeOut__INVERSE 1
#define TimeOut__DIRECT 0

#define TimeOut__CAPTURE 2
#define TimeOut__COMPARE 0

#define TimeOut__TRIG_LEVEL 3
#define TimeOut__TRIG_RISING 0
#define TimeOut__TRIG_FALLING 1
#define TimeOut__TRIG_BOTH 2

#define TimeOut__INTR_MASK_TC 1
#define TimeOut__INTR_MASK_CC_MATCH 2
#define TimeOut__INTR_MASK_NONE 0
#define TimeOut__INTR_MASK_TC_CC 3

#define TimeOut__UNCONFIG 8
#define TimeOut__TIMER 1
#define TimeOut__QUAD 3
#define TimeOut__PWM_SEL 7

#define TimeOut__COUNT_UP 0
#define TimeOut__COUNT_DOWN 1
#define TimeOut__COUNT_UPDOWN0 2
#define TimeOut__COUNT_UPDOWN1 3


/* Prescaler */
#define TimeOut_PRESCALE_DIVBY1                ((uint32)(0u << TimeOut_PRESCALER_SHIFT))
#define TimeOut_PRESCALE_DIVBY2                ((uint32)(1u << TimeOut_PRESCALER_SHIFT))
#define TimeOut_PRESCALE_DIVBY4                ((uint32)(2u << TimeOut_PRESCALER_SHIFT))
#define TimeOut_PRESCALE_DIVBY8                ((uint32)(3u << TimeOut_PRESCALER_SHIFT))
#define TimeOut_PRESCALE_DIVBY16               ((uint32)(4u << TimeOut_PRESCALER_SHIFT))
#define TimeOut_PRESCALE_DIVBY32               ((uint32)(5u << TimeOut_PRESCALER_SHIFT))
#define TimeOut_PRESCALE_DIVBY64               ((uint32)(6u << TimeOut_PRESCALER_SHIFT))
#define TimeOut_PRESCALE_DIVBY128              ((uint32)(7u << TimeOut_PRESCALER_SHIFT))

/* TCPWM set modes */
#define TimeOut_MODE_TIMER_COMPARE             ((uint32)(TimeOut__COMPARE         <<  \
                                                                  TimeOut_MODE_SHIFT))
#define TimeOut_MODE_TIMER_CAPTURE             ((uint32)(TimeOut__CAPTURE         <<  \
                                                                  TimeOut_MODE_SHIFT))
#define TimeOut_MODE_QUAD                      ((uint32)(TimeOut__QUAD            <<  \
                                                                  TimeOut_MODE_SHIFT))
#define TimeOut_MODE_PWM                       ((uint32)(TimeOut__PWM             <<  \
                                                                  TimeOut_MODE_SHIFT))
#define TimeOut_MODE_PWM_DT                    ((uint32)(TimeOut__PWM_DT          <<  \
                                                                  TimeOut_MODE_SHIFT))
#define TimeOut_MODE_PWM_PR                    ((uint32)(TimeOut__PWM_PR          <<  \
                                                                  TimeOut_MODE_SHIFT))

/* Quad Modes */
#define TimeOut_MODE_X1                        ((uint32)(TimeOut__X1              <<  \
                                                                  TimeOut_QUAD_MODE_SHIFT))
#define TimeOut_MODE_X2                        ((uint32)(TimeOut__X2              <<  \
                                                                  TimeOut_QUAD_MODE_SHIFT))
#define TimeOut_MODE_X4                        ((uint32)(TimeOut__X4              <<  \
                                                                  TimeOut_QUAD_MODE_SHIFT))

/* Counter modes */
#define TimeOut_COUNT_UP                       ((uint32)(TimeOut__COUNT_UP        <<  \
                                                                  TimeOut_UPDOWN_SHIFT))
#define TimeOut_COUNT_DOWN                     ((uint32)(TimeOut__COUNT_DOWN      <<  \
                                                                  TimeOut_UPDOWN_SHIFT))
#define TimeOut_COUNT_UPDOWN0                  ((uint32)(TimeOut__COUNT_UPDOWN0   <<  \
                                                                  TimeOut_UPDOWN_SHIFT))
#define TimeOut_COUNT_UPDOWN1                  ((uint32)(TimeOut__COUNT_UPDOWN1   <<  \
                                                                  TimeOut_UPDOWN_SHIFT))

/* PWM output invert */
#define TimeOut_INVERT_LINE                    ((uint32)(TimeOut__INVERSE         <<  \
                                                                  TimeOut_INV_OUT_SHIFT))
#define TimeOut_INVERT_LINE_N                  ((uint32)(TimeOut__INVERSE         <<  \
                                                                  TimeOut_INV_COMPL_OUT_SHIFT))

/* Trigger modes */
#define TimeOut_TRIG_RISING                    ((uint32)TimeOut__TRIG_RISING)
#define TimeOut_TRIG_FALLING                   ((uint32)TimeOut__TRIG_FALLING)
#define TimeOut_TRIG_BOTH                      ((uint32)TimeOut__TRIG_BOTH)
#define TimeOut_TRIG_LEVEL                     ((uint32)TimeOut__TRIG_LEVEL)

/* Interrupt mask */
#define TimeOut_INTR_MASK_TC                   ((uint32)TimeOut__INTR_MASK_TC)
#define TimeOut_INTR_MASK_CC_MATCH             ((uint32)TimeOut__INTR_MASK_CC_MATCH)

/* PWM Output Controls */
#define TimeOut_CC_MATCH_SET                   (0x00u)
#define TimeOut_CC_MATCH_CLEAR                 (0x01u)
#define TimeOut_CC_MATCH_INVERT                (0x02u)
#define TimeOut_CC_MATCH_NO_CHANGE             (0x03u)
#define TimeOut_OVERLOW_SET                    (0x00u)
#define TimeOut_OVERLOW_CLEAR                  (0x04u)
#define TimeOut_OVERLOW_INVERT                 (0x08u)
#define TimeOut_OVERLOW_NO_CHANGE              (0x0Cu)
#define TimeOut_UNDERFLOW_SET                  (0x00u)
#define TimeOut_UNDERFLOW_CLEAR                (0x10u)
#define TimeOut_UNDERFLOW_INVERT               (0x20u)
#define TimeOut_UNDERFLOW_NO_CHANGE            (0x30u)

/* PWM Align */
#define TimeOut_PWM_MODE_LEFT                  (TimeOut_CC_MATCH_CLEAR        |   \
                                                         TimeOut_OVERLOW_SET           |   \
                                                         TimeOut_UNDERFLOW_NO_CHANGE)
#define TimeOut_PWM_MODE_RIGHT                 (TimeOut_CC_MATCH_SET          |   \
                                                         TimeOut_OVERLOW_NO_CHANGE     |   \
                                                         TimeOut_UNDERFLOW_CLEAR)
#define TimeOut_PWM_MODE_ASYM                  (TimeOut_CC_MATCH_INVERT       |   \
                                                         TimeOut_OVERLOW_SET           |   \
                                                         TimeOut_UNDERFLOW_CLEAR)

#if (TimeOut_CY_TCPWM_V2)
    #if(TimeOut_CY_TCPWM_4000)
        #define TimeOut_PWM_MODE_CENTER                (TimeOut_CC_MATCH_INVERT       |   \
                                                                 TimeOut_OVERLOW_NO_CHANGE     |   \
                                                                 TimeOut_UNDERFLOW_CLEAR)
    #else
        #define TimeOut_PWM_MODE_CENTER                (TimeOut_CC_MATCH_INVERT       |   \
                                                                 TimeOut_OVERLOW_SET           |   \
                                                                 TimeOut_UNDERFLOW_CLEAR)
    #endif /* (TimeOut_CY_TCPWM_4000) */
#else
    #define TimeOut_PWM_MODE_CENTER                (TimeOut_CC_MATCH_INVERT       |   \
                                                             TimeOut_OVERLOW_NO_CHANGE     |   \
                                                             TimeOut_UNDERFLOW_CLEAR)
#endif /* (TimeOut_CY_TCPWM_NEW) */

/* Command operations without condition */
#define TimeOut_CMD_CAPTURE                    (0u)
#define TimeOut_CMD_RELOAD                     (8u)
#define TimeOut_CMD_STOP                       (16u)
#define TimeOut_CMD_START                      (24u)

/* Status */
#define TimeOut_STATUS_DOWN                    (1u)
#define TimeOut_STATUS_RUNNING                 (2u)


/***************************************
*        Function Prototypes
****************************************/

void   TimeOut_Init(void);
void   TimeOut_Enable(void);
void   TimeOut_Start(void);
void   TimeOut_Stop(void);

void   TimeOut_SetMode(uint32 mode);
void   TimeOut_SetCounterMode(uint32 counterMode);
void   TimeOut_SetPWMMode(uint32 modeMask);
void   TimeOut_SetQDMode(uint32 qdMode);

void   TimeOut_SetPrescaler(uint32 prescaler);
void   TimeOut_TriggerCommand(uint32 mask, uint32 command);
void   TimeOut_SetOneShot(uint32 oneShotEnable);
uint32 TimeOut_ReadStatus(void);

void   TimeOut_SetPWMSyncKill(uint32 syncKillEnable);
void   TimeOut_SetPWMStopOnKill(uint32 stopOnKillEnable);
void   TimeOut_SetPWMDeadTime(uint32 deadTime);
void   TimeOut_SetPWMInvert(uint32 mask);

void   TimeOut_SetInterruptMode(uint32 interruptMask);
uint32 TimeOut_GetInterruptSourceMasked(void);
uint32 TimeOut_GetInterruptSource(void);
void   TimeOut_ClearInterrupt(uint32 interruptMask);
void   TimeOut_SetInterrupt(uint32 interruptMask);

void   TimeOut_WriteCounter(uint32 count);
uint32 TimeOut_ReadCounter(void);

uint32 TimeOut_ReadCapture(void);
uint32 TimeOut_ReadCaptureBuf(void);

void   TimeOut_WritePeriod(uint32 period);
uint32 TimeOut_ReadPeriod(void);
void   TimeOut_WritePeriodBuf(uint32 periodBuf);
uint32 TimeOut_ReadPeriodBuf(void);

void   TimeOut_WriteCompare(uint32 compare);
uint32 TimeOut_ReadCompare(void);
void   TimeOut_WriteCompareBuf(uint32 compareBuf);
uint32 TimeOut_ReadCompareBuf(void);

void   TimeOut_SetPeriodSwap(uint32 swapEnable);
void   TimeOut_SetCompareSwap(uint32 swapEnable);

void   TimeOut_SetCaptureMode(uint32 triggerMode);
void   TimeOut_SetReloadMode(uint32 triggerMode);
void   TimeOut_SetStartMode(uint32 triggerMode);
void   TimeOut_SetStopMode(uint32 triggerMode);
void   TimeOut_SetCountMode(uint32 triggerMode);

void   TimeOut_SaveConfig(void);
void   TimeOut_RestoreConfig(void);
void   TimeOut_Sleep(void);
void   TimeOut_Wakeup(void);


/***************************************
*             Registers
***************************************/

#define TimeOut_BLOCK_CONTROL_REG              (*(reg32 *) TimeOut_cy_m0s8_tcpwm_1__TCPWM_CTRL )
#define TimeOut_BLOCK_CONTROL_PTR              ( (reg32 *) TimeOut_cy_m0s8_tcpwm_1__TCPWM_CTRL )
#define TimeOut_COMMAND_REG                    (*(reg32 *) TimeOut_cy_m0s8_tcpwm_1__TCPWM_CMD )
#define TimeOut_COMMAND_PTR                    ( (reg32 *) TimeOut_cy_m0s8_tcpwm_1__TCPWM_CMD )
#define TimeOut_INTRRUPT_CAUSE_REG             (*(reg32 *) TimeOut_cy_m0s8_tcpwm_1__TCPWM_INTR_CAUSE )
#define TimeOut_INTRRUPT_CAUSE_PTR             ( (reg32 *) TimeOut_cy_m0s8_tcpwm_1__TCPWM_INTR_CAUSE )
#define TimeOut_CONTROL_REG                    (*(reg32 *) TimeOut_cy_m0s8_tcpwm_1__CTRL )
#define TimeOut_CONTROL_PTR                    ( (reg32 *) TimeOut_cy_m0s8_tcpwm_1__CTRL )
#define TimeOut_STATUS_REG                     (*(reg32 *) TimeOut_cy_m0s8_tcpwm_1__STATUS )
#define TimeOut_STATUS_PTR                     ( (reg32 *) TimeOut_cy_m0s8_tcpwm_1__STATUS )
#define TimeOut_COUNTER_REG                    (*(reg32 *) TimeOut_cy_m0s8_tcpwm_1__COUNTER )
#define TimeOut_COUNTER_PTR                    ( (reg32 *) TimeOut_cy_m0s8_tcpwm_1__COUNTER )
#define TimeOut_COMP_CAP_REG                   (*(reg32 *) TimeOut_cy_m0s8_tcpwm_1__CC )
#define TimeOut_COMP_CAP_PTR                   ( (reg32 *) TimeOut_cy_m0s8_tcpwm_1__CC )
#define TimeOut_COMP_CAP_BUF_REG               (*(reg32 *) TimeOut_cy_m0s8_tcpwm_1__CC_BUFF )
#define TimeOut_COMP_CAP_BUF_PTR               ( (reg32 *) TimeOut_cy_m0s8_tcpwm_1__CC_BUFF )
#define TimeOut_PERIOD_REG                     (*(reg32 *) TimeOut_cy_m0s8_tcpwm_1__PERIOD )
#define TimeOut_PERIOD_PTR                     ( (reg32 *) TimeOut_cy_m0s8_tcpwm_1__PERIOD )
#define TimeOut_PERIOD_BUF_REG                 (*(reg32 *) TimeOut_cy_m0s8_tcpwm_1__PERIOD_BUFF )
#define TimeOut_PERIOD_BUF_PTR                 ( (reg32 *) TimeOut_cy_m0s8_tcpwm_1__PERIOD_BUFF )
#define TimeOut_TRIG_CONTROL0_REG              (*(reg32 *) TimeOut_cy_m0s8_tcpwm_1__TR_CTRL0 )
#define TimeOut_TRIG_CONTROL0_PTR              ( (reg32 *) TimeOut_cy_m0s8_tcpwm_1__TR_CTRL0 )
#define TimeOut_TRIG_CONTROL1_REG              (*(reg32 *) TimeOut_cy_m0s8_tcpwm_1__TR_CTRL1 )
#define TimeOut_TRIG_CONTROL1_PTR              ( (reg32 *) TimeOut_cy_m0s8_tcpwm_1__TR_CTRL1 )
#define TimeOut_TRIG_CONTROL2_REG              (*(reg32 *) TimeOut_cy_m0s8_tcpwm_1__TR_CTRL2 )
#define TimeOut_TRIG_CONTROL2_PTR              ( (reg32 *) TimeOut_cy_m0s8_tcpwm_1__TR_CTRL2 )
#define TimeOut_INTERRUPT_REQ_REG              (*(reg32 *) TimeOut_cy_m0s8_tcpwm_1__INTR )
#define TimeOut_INTERRUPT_REQ_PTR              ( (reg32 *) TimeOut_cy_m0s8_tcpwm_1__INTR )
#define TimeOut_INTERRUPT_SET_REG              (*(reg32 *) TimeOut_cy_m0s8_tcpwm_1__INTR_SET )
#define TimeOut_INTERRUPT_SET_PTR              ( (reg32 *) TimeOut_cy_m0s8_tcpwm_1__INTR_SET )
#define TimeOut_INTERRUPT_MASK_REG             (*(reg32 *) TimeOut_cy_m0s8_tcpwm_1__INTR_MASK )
#define TimeOut_INTERRUPT_MASK_PTR             ( (reg32 *) TimeOut_cy_m0s8_tcpwm_1__INTR_MASK )
#define TimeOut_INTERRUPT_MASKED_REG           (*(reg32 *) TimeOut_cy_m0s8_tcpwm_1__INTR_MASKED )
#define TimeOut_INTERRUPT_MASKED_PTR           ( (reg32 *) TimeOut_cy_m0s8_tcpwm_1__INTR_MASKED )


/***************************************
*       Registers Constants
***************************************/

/* Mask */
#define TimeOut_MASK                           ((uint32)TimeOut_cy_m0s8_tcpwm_1__TCPWM_CTRL_MASK)

/* Shift constants for control register */
#define TimeOut_RELOAD_CC_SHIFT                (0u)
#define TimeOut_RELOAD_PERIOD_SHIFT            (1u)
#define TimeOut_PWM_SYNC_KILL_SHIFT            (2u)
#define TimeOut_PWM_STOP_KILL_SHIFT            (3u)
#define TimeOut_PRESCALER_SHIFT                (8u)
#define TimeOut_UPDOWN_SHIFT                   (16u)
#define TimeOut_ONESHOT_SHIFT                  (18u)
#define TimeOut_QUAD_MODE_SHIFT                (20u)
#define TimeOut_INV_OUT_SHIFT                  (20u)
#define TimeOut_INV_COMPL_OUT_SHIFT            (21u)
#define TimeOut_MODE_SHIFT                     (24u)

/* Mask constants for control register */
#define TimeOut_RELOAD_CC_MASK                 ((uint32)(TimeOut_1BIT_MASK        <<  \
                                                                            TimeOut_RELOAD_CC_SHIFT))
#define TimeOut_RELOAD_PERIOD_MASK             ((uint32)(TimeOut_1BIT_MASK        <<  \
                                                                            TimeOut_RELOAD_PERIOD_SHIFT))
#define TimeOut_PWM_SYNC_KILL_MASK             ((uint32)(TimeOut_1BIT_MASK        <<  \
                                                                            TimeOut_PWM_SYNC_KILL_SHIFT))
#define TimeOut_PWM_STOP_KILL_MASK             ((uint32)(TimeOut_1BIT_MASK        <<  \
                                                                            TimeOut_PWM_STOP_KILL_SHIFT))
#define TimeOut_PRESCALER_MASK                 ((uint32)(TimeOut_8BIT_MASK        <<  \
                                                                            TimeOut_PRESCALER_SHIFT))
#define TimeOut_UPDOWN_MASK                    ((uint32)(TimeOut_2BIT_MASK        <<  \
                                                                            TimeOut_UPDOWN_SHIFT))
#define TimeOut_ONESHOT_MASK                   ((uint32)(TimeOut_1BIT_MASK        <<  \
                                                                            TimeOut_ONESHOT_SHIFT))
#define TimeOut_QUAD_MODE_MASK                 ((uint32)(TimeOut_3BIT_MASK        <<  \
                                                                            TimeOut_QUAD_MODE_SHIFT))
#define TimeOut_INV_OUT_MASK                   ((uint32)(TimeOut_2BIT_MASK        <<  \
                                                                            TimeOut_INV_OUT_SHIFT))
#define TimeOut_MODE_MASK                      ((uint32)(TimeOut_3BIT_MASK        <<  \
                                                                            TimeOut_MODE_SHIFT))

/* Shift constants for trigger control register 1 */
#define TimeOut_CAPTURE_SHIFT                  (0u)
#define TimeOut_COUNT_SHIFT                    (2u)
#define TimeOut_RELOAD_SHIFT                   (4u)
#define TimeOut_STOP_SHIFT                     (6u)
#define TimeOut_START_SHIFT                    (8u)

/* Mask constants for trigger control register 1 */
#define TimeOut_CAPTURE_MASK                   ((uint32)(TimeOut_2BIT_MASK        <<  \
                                                                  TimeOut_CAPTURE_SHIFT))
#define TimeOut_COUNT_MASK                     ((uint32)(TimeOut_2BIT_MASK        <<  \
                                                                  TimeOut_COUNT_SHIFT))
#define TimeOut_RELOAD_MASK                    ((uint32)(TimeOut_2BIT_MASK        <<  \
                                                                  TimeOut_RELOAD_SHIFT))
#define TimeOut_STOP_MASK                      ((uint32)(TimeOut_2BIT_MASK        <<  \
                                                                  TimeOut_STOP_SHIFT))
#define TimeOut_START_MASK                     ((uint32)(TimeOut_2BIT_MASK        <<  \
                                                                  TimeOut_START_SHIFT))

/* MASK */
#define TimeOut_1BIT_MASK                      ((uint32)0x01u)
#define TimeOut_2BIT_MASK                      ((uint32)0x03u)
#define TimeOut_3BIT_MASK                      ((uint32)0x07u)
#define TimeOut_6BIT_MASK                      ((uint32)0x3Fu)
#define TimeOut_8BIT_MASK                      ((uint32)0xFFu)
#define TimeOut_16BIT_MASK                     ((uint32)0xFFFFu)

/* Shift constant for status register */
#define TimeOut_RUNNING_STATUS_SHIFT           (30u)


/***************************************
*    Initial Constants
***************************************/

#define TimeOut_CTRL_QUAD_BASE_CONFIG                                                          \
        (((uint32)(TimeOut_QUAD_ENCODING_MODES     << TimeOut_QUAD_MODE_SHIFT))       |\
         ((uint32)(TimeOut_CONFIG                  << TimeOut_MODE_SHIFT)))

#define TimeOut_CTRL_PWM_BASE_CONFIG                                                           \
        (((uint32)(TimeOut_PWM_STOP_EVENT          << TimeOut_PWM_STOP_KILL_SHIFT))   |\
         ((uint32)(TimeOut_PWM_OUT_INVERT          << TimeOut_INV_OUT_SHIFT))         |\
         ((uint32)(TimeOut_PWM_OUT_N_INVERT        << TimeOut_INV_COMPL_OUT_SHIFT))   |\
         ((uint32)(TimeOut_PWM_MODE                << TimeOut_MODE_SHIFT)))

#define TimeOut_CTRL_PWM_RUN_MODE                                                              \
            ((uint32)(TimeOut_PWM_RUN_MODE         << TimeOut_ONESHOT_SHIFT))
            
#define TimeOut_CTRL_PWM_ALIGN                                                                 \
            ((uint32)(TimeOut_PWM_ALIGN            << TimeOut_UPDOWN_SHIFT))

#define TimeOut_CTRL_PWM_KILL_EVENT                                                            \
             ((uint32)(TimeOut_PWM_KILL_EVENT      << TimeOut_PWM_SYNC_KILL_SHIFT))

#define TimeOut_CTRL_PWM_DEAD_TIME_CYCLE                                                       \
            ((uint32)(TimeOut_PWM_DEAD_TIME_CYCLE  << TimeOut_PRESCALER_SHIFT))

#define TimeOut_CTRL_PWM_PRESCALER                                                             \
            ((uint32)(TimeOut_PWM_PRESCALER        << TimeOut_PRESCALER_SHIFT))

#define TimeOut_CTRL_TIMER_BASE_CONFIG                                                         \
        (((uint32)(TimeOut_TC_PRESCALER            << TimeOut_PRESCALER_SHIFT))       |\
         ((uint32)(TimeOut_TC_COUNTER_MODE         << TimeOut_UPDOWN_SHIFT))          |\
         ((uint32)(TimeOut_TC_RUN_MODE             << TimeOut_ONESHOT_SHIFT))         |\
         ((uint32)(TimeOut_TC_COMP_CAP_MODE        << TimeOut_MODE_SHIFT)))
        
#define TimeOut_QUAD_SIGNALS_MODES                                                             \
        (((uint32)(TimeOut_QUAD_PHIA_SIGNAL_MODE   << TimeOut_COUNT_SHIFT))           |\
         ((uint32)(TimeOut_QUAD_INDEX_SIGNAL_MODE  << TimeOut_RELOAD_SHIFT))          |\
         ((uint32)(TimeOut_QUAD_STOP_SIGNAL_MODE   << TimeOut_STOP_SHIFT))            |\
         ((uint32)(TimeOut_QUAD_PHIB_SIGNAL_MODE   << TimeOut_START_SHIFT)))

#define TimeOut_PWM_SIGNALS_MODES                                                              \
        (((uint32)(TimeOut_PWM_SWITCH_SIGNAL_MODE  << TimeOut_CAPTURE_SHIFT))         |\
         ((uint32)(TimeOut_PWM_COUNT_SIGNAL_MODE   << TimeOut_COUNT_SHIFT))           |\
         ((uint32)(TimeOut_PWM_RELOAD_SIGNAL_MODE  << TimeOut_RELOAD_SHIFT))          |\
         ((uint32)(TimeOut_PWM_STOP_SIGNAL_MODE    << TimeOut_STOP_SHIFT))            |\
         ((uint32)(TimeOut_PWM_START_SIGNAL_MODE   << TimeOut_START_SHIFT)))

#define TimeOut_TIMER_SIGNALS_MODES                                                            \
        (((uint32)(TimeOut_TC_CAPTURE_SIGNAL_MODE  << TimeOut_CAPTURE_SHIFT))         |\
         ((uint32)(TimeOut_TC_COUNT_SIGNAL_MODE    << TimeOut_COUNT_SHIFT))           |\
         ((uint32)(TimeOut_TC_RELOAD_SIGNAL_MODE   << TimeOut_RELOAD_SHIFT))          |\
         ((uint32)(TimeOut_TC_STOP_SIGNAL_MODE     << TimeOut_STOP_SHIFT))            |\
         ((uint32)(TimeOut_TC_START_SIGNAL_MODE    << TimeOut_START_SHIFT)))
        
#define TimeOut_TIMER_UPDOWN_CNT_USED                                                          \
                ((TimeOut__COUNT_UPDOWN0 == TimeOut_TC_COUNTER_MODE)                  ||\
                 (TimeOut__COUNT_UPDOWN1 == TimeOut_TC_COUNTER_MODE))

#define TimeOut_PWM_UPDOWN_CNT_USED                                                            \
                ((TimeOut__CENTER == TimeOut_PWM_ALIGN)                               ||\
                 (TimeOut__ASYMMETRIC == TimeOut_PWM_ALIGN))               
        
#define TimeOut_PWM_PR_INIT_VALUE              (1u)
#define TimeOut_QUAD_PERIOD_INIT_VALUE         (0x8000u)



#endif /* End CY_TCPWM_TimeOut_H */

/* [] END OF FILE */
