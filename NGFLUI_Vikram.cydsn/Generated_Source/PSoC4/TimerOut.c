/*******************************************************************************
* File Name: TimerOut.c
* Version 2.10
*
* Description:
*  This file provides the source code to the API for the TimerOut
*  component
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

#include "TimerOut.h"

uint8 TimerOut_initVar = 0u;


/*******************************************************************************
* Function Name: TimerOut_Init
********************************************************************************
*
* Summary:
*  Initialize/Restore default TimerOut configuration.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void TimerOut_Init(void)
{

    /* Set values from customizer to CTRL */
    #if (TimerOut__QUAD == TimerOut_CONFIG)
        TimerOut_CONTROL_REG = TimerOut_CTRL_QUAD_BASE_CONFIG;
        
        /* Set values from customizer to CTRL1 */
        TimerOut_TRIG_CONTROL1_REG  = TimerOut_QUAD_SIGNALS_MODES;

        /* Set values from customizer to INTR */
        TimerOut_SetInterruptMode(TimerOut_QUAD_INTERRUPT_MASK);
        
         /* Set other values */
        TimerOut_SetCounterMode(TimerOut_COUNT_DOWN);
        TimerOut_WritePeriod(TimerOut_QUAD_PERIOD_INIT_VALUE);
        TimerOut_WriteCounter(TimerOut_QUAD_PERIOD_INIT_VALUE);
    #endif  /* (TimerOut__QUAD == TimerOut_CONFIG) */

    #if (TimerOut__TIMER == TimerOut_CONFIG)
        TimerOut_CONTROL_REG = TimerOut_CTRL_TIMER_BASE_CONFIG;
        
        /* Set values from customizer to CTRL1 */
        TimerOut_TRIG_CONTROL1_REG  = TimerOut_TIMER_SIGNALS_MODES;
    
        /* Set values from customizer to INTR */
        TimerOut_SetInterruptMode(TimerOut_TC_INTERRUPT_MASK);
        
        /* Set other values from customizer */
        TimerOut_WritePeriod(TimerOut_TC_PERIOD_VALUE );

        #if (TimerOut__COMPARE == TimerOut_TC_COMP_CAP_MODE)
            TimerOut_WriteCompare(TimerOut_TC_COMPARE_VALUE);

            #if (1u == TimerOut_TC_COMPARE_SWAP)
                TimerOut_SetCompareSwap(1u);
                TimerOut_WriteCompareBuf(TimerOut_TC_COMPARE_BUF_VALUE);
            #endif  /* (1u == TimerOut_TC_COMPARE_SWAP) */
        #endif  /* (TimerOut__COMPARE == TimerOut_TC_COMP_CAP_MODE) */

        /* Initialize counter value */
        #if (TimerOut_CY_TCPWM_V2 && TimerOut_TIMER_UPDOWN_CNT_USED && !TimerOut_CY_TCPWM_4000)
            TimerOut_WriteCounter(1u);
        #elif(TimerOut__COUNT_DOWN == TimerOut_TC_COUNTER_MODE)
            TimerOut_WriteCounter(TimerOut_TC_PERIOD_VALUE);
        #else
            TimerOut_WriteCounter(0u);
        #endif /* (TimerOut_CY_TCPWM_V2 && TimerOut_TIMER_UPDOWN_CNT_USED && !TimerOut_CY_TCPWM_4000) */
    #endif  /* (TimerOut__TIMER == TimerOut_CONFIG) */

    #if (TimerOut__PWM_SEL == TimerOut_CONFIG)
        TimerOut_CONTROL_REG = TimerOut_CTRL_PWM_BASE_CONFIG;

        #if (TimerOut__PWM_PR == TimerOut_PWM_MODE)
            TimerOut_CONTROL_REG |= TimerOut_CTRL_PWM_RUN_MODE;
            TimerOut_WriteCounter(TimerOut_PWM_PR_INIT_VALUE);
        #else
            TimerOut_CONTROL_REG |= TimerOut_CTRL_PWM_ALIGN | TimerOut_CTRL_PWM_KILL_EVENT;
            
            /* Initialize counter value */
            #if (TimerOut_CY_TCPWM_V2 && TimerOut_PWM_UPDOWN_CNT_USED && !TimerOut_CY_TCPWM_4000)
                TimerOut_WriteCounter(1u);
            #elif (TimerOut__RIGHT == TimerOut_PWM_ALIGN)
                TimerOut_WriteCounter(TimerOut_PWM_PERIOD_VALUE);
            #else 
                TimerOut_WriteCounter(0u);
            #endif  /* (TimerOut_CY_TCPWM_V2 && TimerOut_PWM_UPDOWN_CNT_USED && !TimerOut_CY_TCPWM_4000) */
        #endif  /* (TimerOut__PWM_PR == TimerOut_PWM_MODE) */

        #if (TimerOut__PWM_DT == TimerOut_PWM_MODE)
            TimerOut_CONTROL_REG |= TimerOut_CTRL_PWM_DEAD_TIME_CYCLE;
        #endif  /* (TimerOut__PWM_DT == TimerOut_PWM_MODE) */

        #if (TimerOut__PWM == TimerOut_PWM_MODE)
            TimerOut_CONTROL_REG |= TimerOut_CTRL_PWM_PRESCALER;
        #endif  /* (TimerOut__PWM == TimerOut_PWM_MODE) */

        /* Set values from customizer to CTRL1 */
        TimerOut_TRIG_CONTROL1_REG  = TimerOut_PWM_SIGNALS_MODES;
    
        /* Set values from customizer to INTR */
        TimerOut_SetInterruptMode(TimerOut_PWM_INTERRUPT_MASK);

        /* Set values from customizer to CTRL2 */
        #if (TimerOut__PWM_PR == TimerOut_PWM_MODE)
            TimerOut_TRIG_CONTROL2_REG =
                    (TimerOut_CC_MATCH_NO_CHANGE    |
                    TimerOut_OVERLOW_NO_CHANGE      |
                    TimerOut_UNDERFLOW_NO_CHANGE);
        #else
            #if (TimerOut__LEFT == TimerOut_PWM_ALIGN)
                TimerOut_TRIG_CONTROL2_REG = TimerOut_PWM_MODE_LEFT;
            #endif  /* ( TimerOut_PWM_LEFT == TimerOut_PWM_ALIGN) */

            #if (TimerOut__RIGHT == TimerOut_PWM_ALIGN)
                TimerOut_TRIG_CONTROL2_REG = TimerOut_PWM_MODE_RIGHT;
            #endif  /* ( TimerOut_PWM_RIGHT == TimerOut_PWM_ALIGN) */

            #if (TimerOut__CENTER == TimerOut_PWM_ALIGN)
                TimerOut_TRIG_CONTROL2_REG = TimerOut_PWM_MODE_CENTER;
            #endif  /* ( TimerOut_PWM_CENTER == TimerOut_PWM_ALIGN) */

            #if (TimerOut__ASYMMETRIC == TimerOut_PWM_ALIGN)
                TimerOut_TRIG_CONTROL2_REG = TimerOut_PWM_MODE_ASYM;
            #endif  /* (TimerOut__ASYMMETRIC == TimerOut_PWM_ALIGN) */
        #endif  /* (TimerOut__PWM_PR == TimerOut_PWM_MODE) */

        /* Set other values from customizer */
        TimerOut_WritePeriod(TimerOut_PWM_PERIOD_VALUE );
        TimerOut_WriteCompare(TimerOut_PWM_COMPARE_VALUE);

        #if (1u == TimerOut_PWM_COMPARE_SWAP)
            TimerOut_SetCompareSwap(1u);
            TimerOut_WriteCompareBuf(TimerOut_PWM_COMPARE_BUF_VALUE);
        #endif  /* (1u == TimerOut_PWM_COMPARE_SWAP) */

        #if (1u == TimerOut_PWM_PERIOD_SWAP)
            TimerOut_SetPeriodSwap(1u);
            TimerOut_WritePeriodBuf(TimerOut_PWM_PERIOD_BUF_VALUE);
        #endif  /* (1u == TimerOut_PWM_PERIOD_SWAP) */
    #endif  /* (TimerOut__PWM_SEL == TimerOut_CONFIG) */
    
}


/*******************************************************************************
* Function Name: TimerOut_Enable
********************************************************************************
*
* Summary:
*  Enables the TimerOut.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void TimerOut_Enable(void)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();
    TimerOut_BLOCK_CONTROL_REG |= TimerOut_MASK;
    CyExitCriticalSection(enableInterrupts);

    /* Start Timer or PWM if start input is absent */
    #if (TimerOut__PWM_SEL == TimerOut_CONFIG)
        #if (0u == TimerOut_PWM_START_SIGNAL_PRESENT)
            TimerOut_TriggerCommand(TimerOut_MASK, TimerOut_CMD_START);
        #endif /* (0u == TimerOut_PWM_START_SIGNAL_PRESENT) */
    #endif /* (TimerOut__PWM_SEL == TimerOut_CONFIG) */

    #if (TimerOut__TIMER == TimerOut_CONFIG)
        #if (0u == TimerOut_TC_START_SIGNAL_PRESENT)
            TimerOut_TriggerCommand(TimerOut_MASK, TimerOut_CMD_START);
        #endif /* (0u == TimerOut_TC_START_SIGNAL_PRESENT) */
    #endif /* (TimerOut__TIMER == TimerOut_CONFIG) */
    
    #if (TimerOut__QUAD == TimerOut_CONFIG)
        #if (0u != TimerOut_QUAD_AUTO_START)
            TimerOut_TriggerCommand(TimerOut_MASK, TimerOut_CMD_RELOAD);
        #endif /* (0u != TimerOut_QUAD_AUTO_START) */
    #endif  /* (TimerOut__QUAD == TimerOut_CONFIG) */
}


/*******************************************************************************
* Function Name: TimerOut_Start
********************************************************************************
*
* Summary:
*  Initializes the TimerOut with default customizer
*  values when called the first time and enables the TimerOut.
*  For subsequent calls the configuration is left unchanged and the component is
*  just enabled.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Global variables:
*  TimerOut_initVar: global variable is used to indicate initial
*  configuration of this component.  The variable is initialized to zero and set
*  to 1 the first time TimerOut_Start() is called. This allows
*  enabling/disabling a component without re-initialization in all subsequent
*  calls to the TimerOut_Start() routine.
*
*******************************************************************************/
void TimerOut_Start(void)
{
    if (0u == TimerOut_initVar)
    {
        TimerOut_Init();
        TimerOut_initVar = 1u;
    }

    TimerOut_Enable();
}


/*******************************************************************************
* Function Name: TimerOut_Stop
********************************************************************************
*
* Summary:
*  Disables the TimerOut.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void TimerOut_Stop(void)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    TimerOut_BLOCK_CONTROL_REG &= (uint32)~TimerOut_MASK;

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: TimerOut_SetMode
********************************************************************************
*
* Summary:
*  Sets the operation mode of the TimerOut. This function is used when
*  configured as a generic TimerOut and the actual mode of operation is
*  set at runtime. The mode must be set while the component is disabled.
*
* Parameters:
*  mode: Mode for the TimerOut to operate in
*   Values:
*   - TimerOut_MODE_TIMER_COMPARE - Timer / Counter with
*                                                 compare capability
*         - TimerOut_MODE_TIMER_CAPTURE - Timer / Counter with
*                                                 capture capability
*         - TimerOut_MODE_QUAD - Quadrature decoder
*         - TimerOut_MODE_PWM - PWM
*         - TimerOut_MODE_PWM_DT - PWM with dead time
*         - TimerOut_MODE_PWM_PR - PWM with pseudo random capability
*
* Return:
*  None
*
*******************************************************************************/
void TimerOut_SetMode(uint32 mode)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    TimerOut_CONTROL_REG &= (uint32)~TimerOut_MODE_MASK;
    TimerOut_CONTROL_REG |= mode;

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: TimerOut_SetQDMode
********************************************************************************
*
* Summary:
*  Sets the the Quadrature Decoder to one of the 3 supported modes.
*  Its functionality is only applicable to Quadrature Decoder operation.
*
* Parameters:
*  qdMode: Quadrature Decoder mode
*   Values:
*         - TimerOut_MODE_X1 - Counts on phi 1 rising
*         - TimerOut_MODE_X2 - Counts on both edges of phi1 (2x faster)
*         - TimerOut_MODE_X4 - Counts on both edges of phi1 and phi2
*                                        (4x faster)
*
* Return:
*  None
*
*******************************************************************************/
void TimerOut_SetQDMode(uint32 qdMode)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    TimerOut_CONTROL_REG &= (uint32)~TimerOut_QUAD_MODE_MASK;
    TimerOut_CONTROL_REG |= qdMode;

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: TimerOut_SetPrescaler
********************************************************************************
*
* Summary:
*  Sets the prescaler value that is applied to the clock input.  Not applicable
*  to a PWM with the dead time mode or Quadrature Decoder mode.
*
* Parameters:
*  prescaler: Prescaler divider value
*   Values:
*         - TimerOut_PRESCALE_DIVBY1    - Divide by 1 (no prescaling)
*         - TimerOut_PRESCALE_DIVBY2    - Divide by 2
*         - TimerOut_PRESCALE_DIVBY4    - Divide by 4
*         - TimerOut_PRESCALE_DIVBY8    - Divide by 8
*         - TimerOut_PRESCALE_DIVBY16   - Divide by 16
*         - TimerOut_PRESCALE_DIVBY32   - Divide by 32
*         - TimerOut_PRESCALE_DIVBY64   - Divide by 64
*         - TimerOut_PRESCALE_DIVBY128  - Divide by 128
*
* Return:
*  None
*
*******************************************************************************/
void TimerOut_SetPrescaler(uint32 prescaler)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    TimerOut_CONTROL_REG &= (uint32)~TimerOut_PRESCALER_MASK;
    TimerOut_CONTROL_REG |= prescaler;

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: TimerOut_SetOneShot
********************************************************************************
*
* Summary:
*  Writes the register that controls whether the TimerOut runs
*  continuously or stops when terminal count is reached.  By default the
*  TimerOut operates in the continuous mode.
*
* Parameters:
*  oneShotEnable
*   Values:
*     - 0 - Continuous
*     - 1 - Enable One Shot
*
* Return:
*  None
*
*******************************************************************************/
void TimerOut_SetOneShot(uint32 oneShotEnable)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    TimerOut_CONTROL_REG &= (uint32)~TimerOut_ONESHOT_MASK;
    TimerOut_CONTROL_REG |= ((uint32)((oneShotEnable & TimerOut_1BIT_MASK) <<
                                                               TimerOut_ONESHOT_SHIFT));

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: TimerOut_SetPWMMode
********************************************************************************
*
* Summary:
*  Writes the control register that determines what mode of operation the PWM
*  output lines are driven in.  There is a setting for what to do on a
*  comparison match (CC_MATCH), on an overflow (OVERFLOW) and on an underflow
*  (UNDERFLOW).  The value for each of the three must be ORed together to form
*  the mode.
*
* Parameters:
*  modeMask: A combination of three mode settings.  Mask must include a value
*  for each of the three or use one of the preconfigured PWM settings.
*   Values:
*     - CC_MATCH_SET        - Set on comparison match
*     - CC_MATCH_CLEAR      - Clear on comparison match
*     - CC_MATCH_INVERT     - Invert on comparison match
*     - CC_MATCH_NO_CHANGE  - No change on comparison match
*     - OVERLOW_SET         - Set on overflow
*     - OVERLOW_CLEAR       - Clear on  overflow
*     - OVERLOW_INVERT      - Invert on overflow
*     - OVERLOW_NO_CHANGE   - No change on overflow
*     - UNDERFLOW_SET       - Set on underflow
*     - UNDERFLOW_CLEAR     - Clear on underflow
*     - UNDERFLOW_INVERT    - Invert on underflow
*     - UNDERFLOW_NO_CHANGE - No change on underflow
*     - PWM_MODE_LEFT       - Setting for left aligned PWM.  Should be combined
*                             with up counting mode
*     - PWM_MODE_RIGHT      - Setting for right aligned PWM.  Should be combined
*                             with down counting mode
*     - PWM_MODE_CENTER     - Setting for center aligned PWM.  Should be
*                             combined with up/down 0 mode
*     - PWM_MODE_ASYM       - Setting for asymmetric PWM.  Should be combined
*                             with up/down 1 mode
*
* Return:
*  None
*
*******************************************************************************/
void TimerOut_SetPWMMode(uint32 modeMask)
{
    TimerOut_TRIG_CONTROL2_REG = (modeMask & TimerOut_6BIT_MASK);
}



/*******************************************************************************
* Function Name: TimerOut_SetPWMSyncKill
********************************************************************************
*
* Summary:
*  Writes the register that controls whether the PWM kill signal (stop input)
*  causes asynchronous or synchronous kill operation.  By default the kill
*  operation is asynchronous.  This functionality is only applicable to the PWM
*  and PWM with dead time modes.
*
*  For Synchronous mode the kill signal disables both the line and line_n
*  signals until the next terminal count.
*
*  For Asynchronous mode the kill signal disables both the line and line_n
*  signals when the kill signal is present.  This mode should only be used
*  when the kill signal (stop input) is configured in the pass through mode
*  (Level sensitive signal).

*
* Parameters:
*  syncKillEnable
*   Values:
*     - 0 - Asynchronous
*     - 1 - Synchronous
*
* Return:
*  None
*
*******************************************************************************/
void TimerOut_SetPWMSyncKill(uint32 syncKillEnable)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    TimerOut_CONTROL_REG &= (uint32)~TimerOut_PWM_SYNC_KILL_MASK;
    TimerOut_CONTROL_REG |= ((uint32)((syncKillEnable & TimerOut_1BIT_MASK)  <<
                                               TimerOut_PWM_SYNC_KILL_SHIFT));

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: TimerOut_SetPWMStopOnKill
********************************************************************************
*
* Summary:
*  Writes the register that controls whether the PWM kill signal (stop input)
*  causes the PWM counter to stop.  By default the kill operation does not stop
*  the counter.  This functionality is only applicable to the three PWM modes.
*
*
* Parameters:
*  stopOnKillEnable
*   Values:
*     - 0 - Don't stop
*     - 1 - Stop
*
* Return:
*  None
*
*******************************************************************************/
void TimerOut_SetPWMStopOnKill(uint32 stopOnKillEnable)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    TimerOut_CONTROL_REG &= (uint32)~TimerOut_PWM_STOP_KILL_MASK;
    TimerOut_CONTROL_REG |= ((uint32)((stopOnKillEnable & TimerOut_1BIT_MASK)  <<
                                                         TimerOut_PWM_STOP_KILL_SHIFT));

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: TimerOut_SetPWMDeadTime
********************************************************************************
*
* Summary:
*  Writes the dead time control value.  This value delays the rising edge of
*  both the line and line_n signals the designated number of cycles resulting
*  in both signals being inactive for that many cycles.  This functionality is
*  only applicable to the PWM in the dead time mode.

*
* Parameters:
*  Dead time to insert
*   Values: 0 to 255
*
* Return:
*  None
*
*******************************************************************************/
void TimerOut_SetPWMDeadTime(uint32 deadTime)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    TimerOut_CONTROL_REG &= (uint32)~TimerOut_PRESCALER_MASK;
    TimerOut_CONTROL_REG |= ((uint32)((deadTime & TimerOut_8BIT_MASK) <<
                                                          TimerOut_PRESCALER_SHIFT));

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: TimerOut_SetPWMInvert
********************************************************************************
*
* Summary:
*  Writes the bits that control whether the line and line_n outputs are
*  inverted from their normal output values.  This functionality is only
*  applicable to the three PWM modes.
*
* Parameters:
*  mask: Mask of outputs to invert.
*   Values:
*         - TimerOut_INVERT_LINE   - Inverts the line output
*         - TimerOut_INVERT_LINE_N - Inverts the line_n output
*
* Return:
*  None
*
*******************************************************************************/
void TimerOut_SetPWMInvert(uint32 mask)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    TimerOut_CONTROL_REG &= (uint32)~TimerOut_INV_OUT_MASK;
    TimerOut_CONTROL_REG |= mask;

    CyExitCriticalSection(enableInterrupts);
}



/*******************************************************************************
* Function Name: TimerOut_WriteCounter
********************************************************************************
*
* Summary:
*  Writes a new 16bit counter value directly into the counter register, thus
*  setting the counter (not the period) to the value written. It is not
*  advised to write to this field when the counter is running.
*
* Parameters:
*  count: value to write
*
* Return:
*  None
*
*******************************************************************************/
void TimerOut_WriteCounter(uint32 count)
{
    TimerOut_COUNTER_REG = (count & TimerOut_16BIT_MASK);
}


/*******************************************************************************
* Function Name: TimerOut_ReadCounter
********************************************************************************
*
* Summary:
*  Reads the current counter value.
*
* Parameters:
*  None
*
* Return:
*  Current counter value
*
*******************************************************************************/
uint32 TimerOut_ReadCounter(void)
{
    return (TimerOut_COUNTER_REG & TimerOut_16BIT_MASK);
}


/*******************************************************************************
* Function Name: TimerOut_SetCounterMode
********************************************************************************
*
* Summary:
*  Sets the counter mode.  Applicable to all modes except Quadrature Decoder
*  and the PWM with a pseudo random output.
*
* Parameters:
*  counterMode: Enumerated counter type values
*   Values:
*     - TimerOut_COUNT_UP       - Counts up
*     - TimerOut_COUNT_DOWN     - Counts down
*     - TimerOut_COUNT_UPDOWN0  - Counts up and down. Terminal count
*                                         generated when counter reaches 0
*     - TimerOut_COUNT_UPDOWN1  - Counts up and down. Terminal count
*                                         generated both when counter reaches 0
*                                         and period
*
* Return:
*  None
*
*******************************************************************************/
void TimerOut_SetCounterMode(uint32 counterMode)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    TimerOut_CONTROL_REG &= (uint32)~TimerOut_UPDOWN_MASK;
    TimerOut_CONTROL_REG |= counterMode;

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: TimerOut_WritePeriod
********************************************************************************
*
* Summary:
*  Writes the 16 bit period register with the new period value.
*  To cause the counter to count for N cycles this register should be written
*  with N-1 (counts from 0 to period inclusive).
*
* Parameters:
*  period: Period value
*
* Return:
*  None
*
*******************************************************************************/
void TimerOut_WritePeriod(uint32 period)
{
    TimerOut_PERIOD_REG = (period & TimerOut_16BIT_MASK);
}


/*******************************************************************************
* Function Name: TimerOut_ReadPeriod
********************************************************************************
*
* Summary:
*  Reads the 16 bit period register.
*
* Parameters:
*  None
*
* Return:
*  Period value
*
*******************************************************************************/
uint32 TimerOut_ReadPeriod(void)
{
    return (TimerOut_PERIOD_REG & TimerOut_16BIT_MASK);
}


/*******************************************************************************
* Function Name: TimerOut_SetCompareSwap
********************************************************************************
*
* Summary:
*  Writes the register that controls whether the compare registers are
*  swapped. When enabled in the Timer/Counter mode(without capture) the swap
*  occurs at a TC event. In the PWM mode the swap occurs at the next TC event
*  following a hardware switch event.
*
* Parameters:
*  swapEnable
*   Values:
*     - 0 - Disable swap
*     - 1 - Enable swap
*
* Return:
*  None
*
*******************************************************************************/
void TimerOut_SetCompareSwap(uint32 swapEnable)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    TimerOut_CONTROL_REG &= (uint32)~TimerOut_RELOAD_CC_MASK;
    TimerOut_CONTROL_REG |= (swapEnable & TimerOut_1BIT_MASK);

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: TimerOut_WritePeriodBuf
********************************************************************************
*
* Summary:
*  Writes the 16 bit period buf register with the new period value.
*
* Parameters:
*  periodBuf: Period value
*
* Return:
*  None
*
*******************************************************************************/
void TimerOut_WritePeriodBuf(uint32 periodBuf)
{
    TimerOut_PERIOD_BUF_REG = (periodBuf & TimerOut_16BIT_MASK);
}


/*******************************************************************************
* Function Name: TimerOut_ReadPeriodBuf
********************************************************************************
*
* Summary:
*  Reads the 16 bit period buf register.
*
* Parameters:
*  None
*
* Return:
*  Period value
*
*******************************************************************************/
uint32 TimerOut_ReadPeriodBuf(void)
{
    return (TimerOut_PERIOD_BUF_REG & TimerOut_16BIT_MASK);
}


/*******************************************************************************
* Function Name: TimerOut_SetPeriodSwap
********************************************************************************
*
* Summary:
*  Writes the register that controls whether the period registers are
*  swapped. When enabled in Timer/Counter mode the swap occurs at a TC event.
*  In the PWM mode the swap occurs at the next TC event following a hardware
*  switch event.
*
* Parameters:
*  swapEnable
*   Values:
*     - 0 - Disable swap
*     - 1 - Enable swap
*
* Return:
*  None
*
*******************************************************************************/
void TimerOut_SetPeriodSwap(uint32 swapEnable)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    TimerOut_CONTROL_REG &= (uint32)~TimerOut_RELOAD_PERIOD_MASK;
    TimerOut_CONTROL_REG |= ((uint32)((swapEnable & TimerOut_1BIT_MASK) <<
                                                            TimerOut_RELOAD_PERIOD_SHIFT));

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: TimerOut_WriteCompare
********************************************************************************
*
* Summary:
*  Writes the 16 bit compare register with the new compare value. Not
*  applicable for Timer/Counter with Capture or in Quadrature Decoder modes.
*
* Parameters:
*  compare: Compare value
*
* Return:
*  None
*
* Note:
*  It is not recommended to use the value equal to "0" or equal to 
*  "period value" in Center or Asymmetric align PWM modes on the 
*  PSoC 4100/PSoC 4200 devices.
*  PSoC 4000 devices write the 16 bit compare register with the decremented 
*  compare value in the Up counting mode (except 0x0u), and the incremented 
*  compare value in the Down counting mode (except 0xFFFFu).
*
*******************************************************************************/
void TimerOut_WriteCompare(uint32 compare)
{
    #if (TimerOut_CY_TCPWM_4000)
        uint32 currentMode;
    #endif /* (TimerOut_CY_TCPWM_4000) */

    #if (TimerOut_CY_TCPWM_4000)
        currentMode = ((TimerOut_CONTROL_REG & TimerOut_UPDOWN_MASK) >> TimerOut_UPDOWN_SHIFT);

        if (((uint32)TimerOut__COUNT_DOWN == currentMode) && (0xFFFFu != compare))
        {
            compare++;
        }
        else if (((uint32)TimerOut__COUNT_UP == currentMode) && (0u != compare))
        {
            compare--;
        }
        else
        {
        }
        
    
    #endif /* (TimerOut_CY_TCPWM_4000) */
    
    TimerOut_COMP_CAP_REG = (compare & TimerOut_16BIT_MASK);
}


/*******************************************************************************
* Function Name: TimerOut_ReadCompare
********************************************************************************
*
* Summary:
*  Reads the compare register. Not applicable for Timer/Counter with Capture
*  or in Quadrature Decoder modes.
*  PSoC 4000 devices read the incremented compare register value in the 
*  Up counting mode (except 0xFFFFu), and the decremented value in the 
*  Down counting mode (except 0x0u).
*
* Parameters:
*  None
*
* Return:
*  Compare value
*
* Note:
*  PSoC 4000 devices read the incremented compare register value in the 
*  Up counting mode (except 0xFFFFu), and the decremented value in the 
*  Down counting mode (except 0x0u).
*
*******************************************************************************/
uint32 TimerOut_ReadCompare(void)
{
    #if (TimerOut_CY_TCPWM_4000)
        uint32 currentMode;
        uint32 regVal;
    #endif /* (TimerOut_CY_TCPWM_4000) */

    #if (TimerOut_CY_TCPWM_4000)
        currentMode = ((TimerOut_CONTROL_REG & TimerOut_UPDOWN_MASK) >> TimerOut_UPDOWN_SHIFT);
        
        regVal = TimerOut_COMP_CAP_REG;
        
        if (((uint32)TimerOut__COUNT_DOWN == currentMode) && (0u != regVal))
        {
            regVal--;
        }
        else if (((uint32)TimerOut__COUNT_UP == currentMode) && (0xFFFFu != regVal))
        {
            regVal++;
        }
        else
        {
        }

        return (regVal & TimerOut_16BIT_MASK);
    #else
        return (TimerOut_COMP_CAP_REG & TimerOut_16BIT_MASK);
    #endif /* (TimerOut_CY_TCPWM_4000) */
}


/*******************************************************************************
* Function Name: TimerOut_WriteCompareBuf
********************************************************************************
*
* Summary:
*  Writes the 16 bit compare buffer register with the new compare value. Not
*  applicable for Timer/Counter with Capture or in Quadrature Decoder modes.
*
* Parameters:
*  compareBuf: Compare value
*
* Return:
*  None
*
* Note:
*  It is not recommended to use the value equal to "0" or equal to 
*  "period value" in Center or Asymmetric align PWM modes on the 
*  PSoC 4100/PSoC 4200 devices.
*  PSoC 4000 devices write the 16 bit compare register with the decremented 
*  compare value in the Up counting mode (except 0x0u), and the incremented 
*  compare value in the Down counting mode (except 0xFFFFu).
*
*******************************************************************************/
void TimerOut_WriteCompareBuf(uint32 compareBuf)
{
    #if (TimerOut_CY_TCPWM_4000)
        uint32 currentMode;
    #endif /* (TimerOut_CY_TCPWM_4000) */

    #if (TimerOut_CY_TCPWM_4000)
        currentMode = ((TimerOut_CONTROL_REG & TimerOut_UPDOWN_MASK) >> TimerOut_UPDOWN_SHIFT);

        if (((uint32)TimerOut__COUNT_DOWN == currentMode) && (0xFFFFu != compareBuf))
        {
            compareBuf++;
        }
        else if (((uint32)TimerOut__COUNT_UP == currentMode) && (0u != compareBuf))
        {
            compareBuf --;
        }
        else
        {
        }
    #endif /* (TimerOut_CY_TCPWM_4000) */
    
    TimerOut_COMP_CAP_BUF_REG = (compareBuf & TimerOut_16BIT_MASK);
}


/*******************************************************************************
* Function Name: TimerOut_ReadCompareBuf
********************************************************************************
*
* Summary:
*  Reads the compare buffer register. Not applicable for Timer/Counter with
*  Capture or in Quadrature Decoder modes.
*
* Parameters:
*  None
*
* Return:
*  Compare buffer value
*
* Note:
*  PSoC 4000 devices read the incremented compare register value in the 
*  Up counting mode (except 0xFFFFu), and the decremented value in the 
*  Down counting mode (except 0x0u).
*
*******************************************************************************/
uint32 TimerOut_ReadCompareBuf(void)
{
    #if (TimerOut_CY_TCPWM_4000)
        uint32 currentMode;
        uint32 regVal;
    #endif /* (TimerOut_CY_TCPWM_4000) */

    #if (TimerOut_CY_TCPWM_4000)
        currentMode = ((TimerOut_CONTROL_REG & TimerOut_UPDOWN_MASK) >> TimerOut_UPDOWN_SHIFT);

        regVal = TimerOut_COMP_CAP_BUF_REG;
        
        if (((uint32)TimerOut__COUNT_DOWN == currentMode) && (0u != regVal))
        {
            regVal--;
        }
        else if (((uint32)TimerOut__COUNT_UP == currentMode) && (0xFFFFu != regVal))
        {
            regVal++;
        }
        else
        {
        }

        return (regVal & TimerOut_16BIT_MASK);
    #else
        return (TimerOut_COMP_CAP_BUF_REG & TimerOut_16BIT_MASK);
    #endif /* (TimerOut_CY_TCPWM_4000) */
}


/*******************************************************************************
* Function Name: TimerOut_ReadCapture
********************************************************************************
*
* Summary:
*  Reads the captured counter value. This API is applicable only for
*  Timer/Counter with the capture mode and Quadrature Decoder modes.
*
* Parameters:
*  None
*
* Return:
*  Capture value
*
*******************************************************************************/
uint32 TimerOut_ReadCapture(void)
{
    return (TimerOut_COMP_CAP_REG & TimerOut_16BIT_MASK);
}


/*******************************************************************************
* Function Name: TimerOut_ReadCaptureBuf
********************************************************************************
*
* Summary:
*  Reads the capture buffer register. This API is applicable only for
*  Timer/Counter with the capture mode and Quadrature Decoder modes.
*
* Parameters:
*  None
*
* Return:
*  Capture buffer value
*
*******************************************************************************/
uint32 TimerOut_ReadCaptureBuf(void)
{
    return (TimerOut_COMP_CAP_BUF_REG & TimerOut_16BIT_MASK);
}


/*******************************************************************************
* Function Name: TimerOut_SetCaptureMode
********************************************************************************
*
* Summary:
*  Sets the capture trigger mode. For PWM mode this is the switch input.
*  This input is not applicable to the Timer/Counter without Capture and
*  Quadrature Decoder modes.
*
* Parameters:
*  triggerMode: Enumerated trigger mode value
*   Values:
*     - TimerOut_TRIG_LEVEL     - Level
*     - TimerOut_TRIG_RISING    - Rising edge
*     - TimerOut_TRIG_FALLING   - Falling edge
*     - TimerOut_TRIG_BOTH      - Both rising and falling edge
*
* Return:
*  None
*
*******************************************************************************/
void TimerOut_SetCaptureMode(uint32 triggerMode)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    TimerOut_TRIG_CONTROL1_REG &= (uint32)~TimerOut_CAPTURE_MASK;
    TimerOut_TRIG_CONTROL1_REG |= triggerMode;

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: TimerOut_SetReloadMode
********************************************************************************
*
* Summary:
*  Sets the reload trigger mode. For Quadrature Decoder mode this is the index
*  input.
*
* Parameters:
*  triggerMode: Enumerated trigger mode value
*   Values:
*     - TimerOut_TRIG_LEVEL     - Level
*     - TimerOut_TRIG_RISING    - Rising edge
*     - TimerOut_TRIG_FALLING   - Falling edge
*     - TimerOut_TRIG_BOTH      - Both rising and falling edge
*
* Return:
*  None
*
*******************************************************************************/
void TimerOut_SetReloadMode(uint32 triggerMode)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    TimerOut_TRIG_CONTROL1_REG &= (uint32)~TimerOut_RELOAD_MASK;
    TimerOut_TRIG_CONTROL1_REG |= ((uint32)(triggerMode << TimerOut_RELOAD_SHIFT));

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: TimerOut_SetStartMode
********************************************************************************
*
* Summary:
*  Sets the start trigger mode. For Quadrature Decoder mode this is the
*  phiB input.
*
* Parameters:
*  triggerMode: Enumerated trigger mode value
*   Values:
*     - TimerOut_TRIG_LEVEL     - Level
*     - TimerOut_TRIG_RISING    - Rising edge
*     - TimerOut_TRIG_FALLING   - Falling edge
*     - TimerOut_TRIG_BOTH      - Both rising and falling edge
*
* Return:
*  None
*
*******************************************************************************/
void TimerOut_SetStartMode(uint32 triggerMode)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    TimerOut_TRIG_CONTROL1_REG &= (uint32)~TimerOut_START_MASK;
    TimerOut_TRIG_CONTROL1_REG |= ((uint32)(triggerMode << TimerOut_START_SHIFT));

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: TimerOut_SetStopMode
********************************************************************************
*
* Summary:
*  Sets the stop trigger mode. For PWM mode this is the kill input.
*
* Parameters:
*  triggerMode: Enumerated trigger mode value
*   Values:
*     - TimerOut_TRIG_LEVEL     - Level
*     - TimerOut_TRIG_RISING    - Rising edge
*     - TimerOut_TRIG_FALLING   - Falling edge
*     - TimerOut_TRIG_BOTH      - Both rising and falling edge
*
* Return:
*  None
*
*******************************************************************************/
void TimerOut_SetStopMode(uint32 triggerMode)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    TimerOut_TRIG_CONTROL1_REG &= (uint32)~TimerOut_STOP_MASK;
    TimerOut_TRIG_CONTROL1_REG |= ((uint32)(triggerMode << TimerOut_STOP_SHIFT));

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: TimerOut_SetCountMode
********************************************************************************
*
* Summary:
*  Sets the count trigger mode. For Quadrature Decoder mode this is the phiA
*  input.
*
* Parameters:
*  triggerMode: Enumerated trigger mode value
*   Values:
*     - TimerOut_TRIG_LEVEL     - Level
*     - TimerOut_TRIG_RISING    - Rising edge
*     - TimerOut_TRIG_FALLING   - Falling edge
*     - TimerOut_TRIG_BOTH      - Both rising and falling edge
*
* Return:
*  None
*
*******************************************************************************/
void TimerOut_SetCountMode(uint32 triggerMode)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    TimerOut_TRIG_CONTROL1_REG &= (uint32)~TimerOut_COUNT_MASK;
    TimerOut_TRIG_CONTROL1_REG |= ((uint32)(triggerMode << TimerOut_COUNT_SHIFT));

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: TimerOut_TriggerCommand
********************************************************************************
*
* Summary:
*  Triggers the designated command to occur on the designated TCPWM instances.
*  The mask can be used to apply this command simultaneously to more than one
*  instance.  This allows multiple TCPWM instances to be synchronized.
*
* Parameters:
*  mask: A combination of mask bits for each instance of the TCPWM that the
*        command should apply to.  This function from one instance can be used
*        to apply the command to any of the instances in the design.
*        The mask value for a specific instance is available with the MASK
*        define.
*  command: Enumerated command values. Capture command only applicable for
*           Timer/Counter with Capture and PWM modes.
*   Values:
*     - TimerOut_CMD_CAPTURE    - Trigger Capture/Switch command
*     - TimerOut_CMD_RELOAD     - Trigger Reload/Index command
*     - TimerOut_CMD_STOP       - Trigger Stop/Kill command
*     - TimerOut_CMD_START      - Trigger Start/phiB command
*
* Return:
*  None
*
*******************************************************************************/
void TimerOut_TriggerCommand(uint32 mask, uint32 command)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    TimerOut_COMMAND_REG = ((uint32)(mask << command));

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: TimerOut_ReadStatus
********************************************************************************
*
* Summary:
*  Reads the status of the TimerOut.
*
* Parameters:
*  None
*
* Return:
*  Status
*   Values:
*     - TimerOut_STATUS_DOWN    - Set if counting down
*     - TimerOut_STATUS_RUNNING - Set if counter is running
*
*******************************************************************************/
uint32 TimerOut_ReadStatus(void)
{
    return ((TimerOut_STATUS_REG >> TimerOut_RUNNING_STATUS_SHIFT) |
            (TimerOut_STATUS_REG & TimerOut_STATUS_DOWN));
}


/*******************************************************************************
* Function Name: TimerOut_SetInterruptMode
********************************************************************************
*
* Summary:
*  Sets the interrupt mask to control which interrupt
*  requests generate the interrupt signal.
*
* Parameters:
*   interruptMask: Mask of bits to be enabled
*   Values:
*     - TimerOut_INTR_MASK_TC       - Terminal count mask
*     - TimerOut_INTR_MASK_CC_MATCH - Compare count / capture mask
*
* Return:
*  None
*
*******************************************************************************/
void TimerOut_SetInterruptMode(uint32 interruptMask)
{
    TimerOut_INTERRUPT_MASK_REG =  interruptMask;
}


/*******************************************************************************
* Function Name: TimerOut_GetInterruptSourceMasked
********************************************************************************
*
* Summary:
*  Gets the interrupt requests masked by the interrupt mask.
*
* Parameters:
*   None
*
* Return:
*  Masked interrupt source
*   Values:
*     - TimerOut_INTR_MASK_TC       - Terminal count mask
*     - TimerOut_INTR_MASK_CC_MATCH - Compare count / capture mask
*
*******************************************************************************/
uint32 TimerOut_GetInterruptSourceMasked(void)
{
    return (TimerOut_INTERRUPT_MASKED_REG);
}


/*******************************************************************************
* Function Name: TimerOut_GetInterruptSource
********************************************************************************
*
* Summary:
*  Gets the interrupt requests (without masking).
*
* Parameters:
*  None
*
* Return:
*  Interrupt request value
*   Values:
*     - TimerOut_INTR_MASK_TC       - Terminal count mask
*     - TimerOut_INTR_MASK_CC_MATCH - Compare count / capture mask
*
*******************************************************************************/
uint32 TimerOut_GetInterruptSource(void)
{
    return (TimerOut_INTERRUPT_REQ_REG);
}


/*******************************************************************************
* Function Name: TimerOut_ClearInterrupt
********************************************************************************
*
* Summary:
*  Clears the interrupt request.
*
* Parameters:
*   interruptMask: Mask of interrupts to clear
*   Values:
*     - TimerOut_INTR_MASK_TC       - Terminal count mask
*     - TimerOut_INTR_MASK_CC_MATCH - Compare count / capture mask
*
* Return:
*  None
*
*******************************************************************************/
void TimerOut_ClearInterrupt(uint32 interruptMask)
{
    TimerOut_INTERRUPT_REQ_REG = interruptMask;
}


/*******************************************************************************
* Function Name: TimerOut_SetInterrupt
********************************************************************************
*
* Summary:
*  Sets a software interrupt request.
*
* Parameters:
*   interruptMask: Mask of interrupts to set
*   Values:
*     - TimerOut_INTR_MASK_TC       - Terminal count mask
*     - TimerOut_INTR_MASK_CC_MATCH - Compare count / capture mask
*
* Return:
*  None
*
*******************************************************************************/
void TimerOut_SetInterrupt(uint32 interruptMask)
{
    TimerOut_INTERRUPT_SET_REG = interruptMask;
}


/* [] END OF FILE */
