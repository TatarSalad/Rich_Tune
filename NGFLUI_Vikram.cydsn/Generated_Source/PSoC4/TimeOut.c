/*******************************************************************************
* File Name: TimeOut.c
* Version 2.10
*
* Description:
*  This file provides the source code to the API for the TimeOut
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

#include "TimeOut.h"

uint8 TimeOut_initVar = 0u;


/*******************************************************************************
* Function Name: TimeOut_Init
********************************************************************************
*
* Summary:
*  Initialize/Restore default TimeOut configuration.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void TimeOut_Init(void)
{

    /* Set values from customizer to CTRL */
    #if (TimeOut__QUAD == TimeOut_CONFIG)
        TimeOut_CONTROL_REG = TimeOut_CTRL_QUAD_BASE_CONFIG;
        
        /* Set values from customizer to CTRL1 */
        TimeOut_TRIG_CONTROL1_REG  = TimeOut_QUAD_SIGNALS_MODES;

        /* Set values from customizer to INTR */
        TimeOut_SetInterruptMode(TimeOut_QUAD_INTERRUPT_MASK);
        
         /* Set other values */
        TimeOut_SetCounterMode(TimeOut_COUNT_DOWN);
        TimeOut_WritePeriod(TimeOut_QUAD_PERIOD_INIT_VALUE);
        TimeOut_WriteCounter(TimeOut_QUAD_PERIOD_INIT_VALUE);
    #endif  /* (TimeOut__QUAD == TimeOut_CONFIG) */

    #if (TimeOut__TIMER == TimeOut_CONFIG)
        TimeOut_CONTROL_REG = TimeOut_CTRL_TIMER_BASE_CONFIG;
        
        /* Set values from customizer to CTRL1 */
        TimeOut_TRIG_CONTROL1_REG  = TimeOut_TIMER_SIGNALS_MODES;
    
        /* Set values from customizer to INTR */
        TimeOut_SetInterruptMode(TimeOut_TC_INTERRUPT_MASK);
        
        /* Set other values from customizer */
        TimeOut_WritePeriod(TimeOut_TC_PERIOD_VALUE );

        #if (TimeOut__COMPARE == TimeOut_TC_COMP_CAP_MODE)
            TimeOut_WriteCompare(TimeOut_TC_COMPARE_VALUE);

            #if (1u == TimeOut_TC_COMPARE_SWAP)
                TimeOut_SetCompareSwap(1u);
                TimeOut_WriteCompareBuf(TimeOut_TC_COMPARE_BUF_VALUE);
            #endif  /* (1u == TimeOut_TC_COMPARE_SWAP) */
        #endif  /* (TimeOut__COMPARE == TimeOut_TC_COMP_CAP_MODE) */

        /* Initialize counter value */
        #if (TimeOut_CY_TCPWM_V2 && TimeOut_TIMER_UPDOWN_CNT_USED && !TimeOut_CY_TCPWM_4000)
            TimeOut_WriteCounter(1u);
        #elif(TimeOut__COUNT_DOWN == TimeOut_TC_COUNTER_MODE)
            TimeOut_WriteCounter(TimeOut_TC_PERIOD_VALUE);
        #else
            TimeOut_WriteCounter(0u);
        #endif /* (TimeOut_CY_TCPWM_V2 && TimeOut_TIMER_UPDOWN_CNT_USED && !TimeOut_CY_TCPWM_4000) */
    #endif  /* (TimeOut__TIMER == TimeOut_CONFIG) */

    #if (TimeOut__PWM_SEL == TimeOut_CONFIG)
        TimeOut_CONTROL_REG = TimeOut_CTRL_PWM_BASE_CONFIG;

        #if (TimeOut__PWM_PR == TimeOut_PWM_MODE)
            TimeOut_CONTROL_REG |= TimeOut_CTRL_PWM_RUN_MODE;
            TimeOut_WriteCounter(TimeOut_PWM_PR_INIT_VALUE);
        #else
            TimeOut_CONTROL_REG |= TimeOut_CTRL_PWM_ALIGN | TimeOut_CTRL_PWM_KILL_EVENT;
            
            /* Initialize counter value */
            #if (TimeOut_CY_TCPWM_V2 && TimeOut_PWM_UPDOWN_CNT_USED && !TimeOut_CY_TCPWM_4000)
                TimeOut_WriteCounter(1u);
            #elif (TimeOut__RIGHT == TimeOut_PWM_ALIGN)
                TimeOut_WriteCounter(TimeOut_PWM_PERIOD_VALUE);
            #else 
                TimeOut_WriteCounter(0u);
            #endif  /* (TimeOut_CY_TCPWM_V2 && TimeOut_PWM_UPDOWN_CNT_USED && !TimeOut_CY_TCPWM_4000) */
        #endif  /* (TimeOut__PWM_PR == TimeOut_PWM_MODE) */

        #if (TimeOut__PWM_DT == TimeOut_PWM_MODE)
            TimeOut_CONTROL_REG |= TimeOut_CTRL_PWM_DEAD_TIME_CYCLE;
        #endif  /* (TimeOut__PWM_DT == TimeOut_PWM_MODE) */

        #if (TimeOut__PWM == TimeOut_PWM_MODE)
            TimeOut_CONTROL_REG |= TimeOut_CTRL_PWM_PRESCALER;
        #endif  /* (TimeOut__PWM == TimeOut_PWM_MODE) */

        /* Set values from customizer to CTRL1 */
        TimeOut_TRIG_CONTROL1_REG  = TimeOut_PWM_SIGNALS_MODES;
    
        /* Set values from customizer to INTR */
        TimeOut_SetInterruptMode(TimeOut_PWM_INTERRUPT_MASK);

        /* Set values from customizer to CTRL2 */
        #if (TimeOut__PWM_PR == TimeOut_PWM_MODE)
            TimeOut_TRIG_CONTROL2_REG =
                    (TimeOut_CC_MATCH_NO_CHANGE    |
                    TimeOut_OVERLOW_NO_CHANGE      |
                    TimeOut_UNDERFLOW_NO_CHANGE);
        #else
            #if (TimeOut__LEFT == TimeOut_PWM_ALIGN)
                TimeOut_TRIG_CONTROL2_REG = TimeOut_PWM_MODE_LEFT;
            #endif  /* ( TimeOut_PWM_LEFT == TimeOut_PWM_ALIGN) */

            #if (TimeOut__RIGHT == TimeOut_PWM_ALIGN)
                TimeOut_TRIG_CONTROL2_REG = TimeOut_PWM_MODE_RIGHT;
            #endif  /* ( TimeOut_PWM_RIGHT == TimeOut_PWM_ALIGN) */

            #if (TimeOut__CENTER == TimeOut_PWM_ALIGN)
                TimeOut_TRIG_CONTROL2_REG = TimeOut_PWM_MODE_CENTER;
            #endif  /* ( TimeOut_PWM_CENTER == TimeOut_PWM_ALIGN) */

            #if (TimeOut__ASYMMETRIC == TimeOut_PWM_ALIGN)
                TimeOut_TRIG_CONTROL2_REG = TimeOut_PWM_MODE_ASYM;
            #endif  /* (TimeOut__ASYMMETRIC == TimeOut_PWM_ALIGN) */
        #endif  /* (TimeOut__PWM_PR == TimeOut_PWM_MODE) */

        /* Set other values from customizer */
        TimeOut_WritePeriod(TimeOut_PWM_PERIOD_VALUE );
        TimeOut_WriteCompare(TimeOut_PWM_COMPARE_VALUE);

        #if (1u == TimeOut_PWM_COMPARE_SWAP)
            TimeOut_SetCompareSwap(1u);
            TimeOut_WriteCompareBuf(TimeOut_PWM_COMPARE_BUF_VALUE);
        #endif  /* (1u == TimeOut_PWM_COMPARE_SWAP) */

        #if (1u == TimeOut_PWM_PERIOD_SWAP)
            TimeOut_SetPeriodSwap(1u);
            TimeOut_WritePeriodBuf(TimeOut_PWM_PERIOD_BUF_VALUE);
        #endif  /* (1u == TimeOut_PWM_PERIOD_SWAP) */
    #endif  /* (TimeOut__PWM_SEL == TimeOut_CONFIG) */
    
}


/*******************************************************************************
* Function Name: TimeOut_Enable
********************************************************************************
*
* Summary:
*  Enables the TimeOut.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void TimeOut_Enable(void)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();
    TimeOut_BLOCK_CONTROL_REG |= TimeOut_MASK;
    CyExitCriticalSection(enableInterrupts);

    /* Start Timer or PWM if start input is absent */
    #if (TimeOut__PWM_SEL == TimeOut_CONFIG)
        #if (0u == TimeOut_PWM_START_SIGNAL_PRESENT)
            TimeOut_TriggerCommand(TimeOut_MASK, TimeOut_CMD_START);
        #endif /* (0u == TimeOut_PWM_START_SIGNAL_PRESENT) */
    #endif /* (TimeOut__PWM_SEL == TimeOut_CONFIG) */

    #if (TimeOut__TIMER == TimeOut_CONFIG)
        #if (0u == TimeOut_TC_START_SIGNAL_PRESENT)
            TimeOut_TriggerCommand(TimeOut_MASK, TimeOut_CMD_START);
        #endif /* (0u == TimeOut_TC_START_SIGNAL_PRESENT) */
    #endif /* (TimeOut__TIMER == TimeOut_CONFIG) */
    
    #if (TimeOut__QUAD == TimeOut_CONFIG)
        #if (0u != TimeOut_QUAD_AUTO_START)
            TimeOut_TriggerCommand(TimeOut_MASK, TimeOut_CMD_RELOAD);
        #endif /* (0u != TimeOut_QUAD_AUTO_START) */
    #endif  /* (TimeOut__QUAD == TimeOut_CONFIG) */
}


/*******************************************************************************
* Function Name: TimeOut_Start
********************************************************************************
*
* Summary:
*  Initializes the TimeOut with default customizer
*  values when called the first time and enables the TimeOut.
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
*  TimeOut_initVar: global variable is used to indicate initial
*  configuration of this component.  The variable is initialized to zero and set
*  to 1 the first time TimeOut_Start() is called. This allows
*  enabling/disabling a component without re-initialization in all subsequent
*  calls to the TimeOut_Start() routine.
*
*******************************************************************************/
void TimeOut_Start(void)
{
    if (0u == TimeOut_initVar)
    {
        TimeOut_Init();
        TimeOut_initVar = 1u;
    }

    TimeOut_Enable();
}


/*******************************************************************************
* Function Name: TimeOut_Stop
********************************************************************************
*
* Summary:
*  Disables the TimeOut.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void TimeOut_Stop(void)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    TimeOut_BLOCK_CONTROL_REG &= (uint32)~TimeOut_MASK;

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: TimeOut_SetMode
********************************************************************************
*
* Summary:
*  Sets the operation mode of the TimeOut. This function is used when
*  configured as a generic TimeOut and the actual mode of operation is
*  set at runtime. The mode must be set while the component is disabled.
*
* Parameters:
*  mode: Mode for the TimeOut to operate in
*   Values:
*   - TimeOut_MODE_TIMER_COMPARE - Timer / Counter with
*                                                 compare capability
*         - TimeOut_MODE_TIMER_CAPTURE - Timer / Counter with
*                                                 capture capability
*         - TimeOut_MODE_QUAD - Quadrature decoder
*         - TimeOut_MODE_PWM - PWM
*         - TimeOut_MODE_PWM_DT - PWM with dead time
*         - TimeOut_MODE_PWM_PR - PWM with pseudo random capability
*
* Return:
*  None
*
*******************************************************************************/
void TimeOut_SetMode(uint32 mode)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    TimeOut_CONTROL_REG &= (uint32)~TimeOut_MODE_MASK;
    TimeOut_CONTROL_REG |= mode;

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: TimeOut_SetQDMode
********************************************************************************
*
* Summary:
*  Sets the the Quadrature Decoder to one of the 3 supported modes.
*  Its functionality is only applicable to Quadrature Decoder operation.
*
* Parameters:
*  qdMode: Quadrature Decoder mode
*   Values:
*         - TimeOut_MODE_X1 - Counts on phi 1 rising
*         - TimeOut_MODE_X2 - Counts on both edges of phi1 (2x faster)
*         - TimeOut_MODE_X4 - Counts on both edges of phi1 and phi2
*                                        (4x faster)
*
* Return:
*  None
*
*******************************************************************************/
void TimeOut_SetQDMode(uint32 qdMode)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    TimeOut_CONTROL_REG &= (uint32)~TimeOut_QUAD_MODE_MASK;
    TimeOut_CONTROL_REG |= qdMode;

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: TimeOut_SetPrescaler
********************************************************************************
*
* Summary:
*  Sets the prescaler value that is applied to the clock input.  Not applicable
*  to a PWM with the dead time mode or Quadrature Decoder mode.
*
* Parameters:
*  prescaler: Prescaler divider value
*   Values:
*         - TimeOut_PRESCALE_DIVBY1    - Divide by 1 (no prescaling)
*         - TimeOut_PRESCALE_DIVBY2    - Divide by 2
*         - TimeOut_PRESCALE_DIVBY4    - Divide by 4
*         - TimeOut_PRESCALE_DIVBY8    - Divide by 8
*         - TimeOut_PRESCALE_DIVBY16   - Divide by 16
*         - TimeOut_PRESCALE_DIVBY32   - Divide by 32
*         - TimeOut_PRESCALE_DIVBY64   - Divide by 64
*         - TimeOut_PRESCALE_DIVBY128  - Divide by 128
*
* Return:
*  None
*
*******************************************************************************/
void TimeOut_SetPrescaler(uint32 prescaler)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    TimeOut_CONTROL_REG &= (uint32)~TimeOut_PRESCALER_MASK;
    TimeOut_CONTROL_REG |= prescaler;

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: TimeOut_SetOneShot
********************************************************************************
*
* Summary:
*  Writes the register that controls whether the TimeOut runs
*  continuously or stops when terminal count is reached.  By default the
*  TimeOut operates in the continuous mode.
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
void TimeOut_SetOneShot(uint32 oneShotEnable)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    TimeOut_CONTROL_REG &= (uint32)~TimeOut_ONESHOT_MASK;
    TimeOut_CONTROL_REG |= ((uint32)((oneShotEnable & TimeOut_1BIT_MASK) <<
                                                               TimeOut_ONESHOT_SHIFT));

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: TimeOut_SetPWMMode
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
void TimeOut_SetPWMMode(uint32 modeMask)
{
    TimeOut_TRIG_CONTROL2_REG = (modeMask & TimeOut_6BIT_MASK);
}



/*******************************************************************************
* Function Name: TimeOut_SetPWMSyncKill
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
void TimeOut_SetPWMSyncKill(uint32 syncKillEnable)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    TimeOut_CONTROL_REG &= (uint32)~TimeOut_PWM_SYNC_KILL_MASK;
    TimeOut_CONTROL_REG |= ((uint32)((syncKillEnable & TimeOut_1BIT_MASK)  <<
                                               TimeOut_PWM_SYNC_KILL_SHIFT));

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: TimeOut_SetPWMStopOnKill
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
void TimeOut_SetPWMStopOnKill(uint32 stopOnKillEnable)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    TimeOut_CONTROL_REG &= (uint32)~TimeOut_PWM_STOP_KILL_MASK;
    TimeOut_CONTROL_REG |= ((uint32)((stopOnKillEnable & TimeOut_1BIT_MASK)  <<
                                                         TimeOut_PWM_STOP_KILL_SHIFT));

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: TimeOut_SetPWMDeadTime
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
void TimeOut_SetPWMDeadTime(uint32 deadTime)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    TimeOut_CONTROL_REG &= (uint32)~TimeOut_PRESCALER_MASK;
    TimeOut_CONTROL_REG |= ((uint32)((deadTime & TimeOut_8BIT_MASK) <<
                                                          TimeOut_PRESCALER_SHIFT));

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: TimeOut_SetPWMInvert
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
*         - TimeOut_INVERT_LINE   - Inverts the line output
*         - TimeOut_INVERT_LINE_N - Inverts the line_n output
*
* Return:
*  None
*
*******************************************************************************/
void TimeOut_SetPWMInvert(uint32 mask)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    TimeOut_CONTROL_REG &= (uint32)~TimeOut_INV_OUT_MASK;
    TimeOut_CONTROL_REG |= mask;

    CyExitCriticalSection(enableInterrupts);
}



/*******************************************************************************
* Function Name: TimeOut_WriteCounter
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
void TimeOut_WriteCounter(uint32 count)
{
    TimeOut_COUNTER_REG = (count & TimeOut_16BIT_MASK);
}


/*******************************************************************************
* Function Name: TimeOut_ReadCounter
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
uint32 TimeOut_ReadCounter(void)
{
    return (TimeOut_COUNTER_REG & TimeOut_16BIT_MASK);
}


/*******************************************************************************
* Function Name: TimeOut_SetCounterMode
********************************************************************************
*
* Summary:
*  Sets the counter mode.  Applicable to all modes except Quadrature Decoder
*  and the PWM with a pseudo random output.
*
* Parameters:
*  counterMode: Enumerated counter type values
*   Values:
*     - TimeOut_COUNT_UP       - Counts up
*     - TimeOut_COUNT_DOWN     - Counts down
*     - TimeOut_COUNT_UPDOWN0  - Counts up and down. Terminal count
*                                         generated when counter reaches 0
*     - TimeOut_COUNT_UPDOWN1  - Counts up and down. Terminal count
*                                         generated both when counter reaches 0
*                                         and period
*
* Return:
*  None
*
*******************************************************************************/
void TimeOut_SetCounterMode(uint32 counterMode)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    TimeOut_CONTROL_REG &= (uint32)~TimeOut_UPDOWN_MASK;
    TimeOut_CONTROL_REG |= counterMode;

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: TimeOut_WritePeriod
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
void TimeOut_WritePeriod(uint32 period)
{
    TimeOut_PERIOD_REG = (period & TimeOut_16BIT_MASK);
}


/*******************************************************************************
* Function Name: TimeOut_ReadPeriod
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
uint32 TimeOut_ReadPeriod(void)
{
    return (TimeOut_PERIOD_REG & TimeOut_16BIT_MASK);
}


/*******************************************************************************
* Function Name: TimeOut_SetCompareSwap
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
void TimeOut_SetCompareSwap(uint32 swapEnable)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    TimeOut_CONTROL_REG &= (uint32)~TimeOut_RELOAD_CC_MASK;
    TimeOut_CONTROL_REG |= (swapEnable & TimeOut_1BIT_MASK);

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: TimeOut_WritePeriodBuf
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
void TimeOut_WritePeriodBuf(uint32 periodBuf)
{
    TimeOut_PERIOD_BUF_REG = (periodBuf & TimeOut_16BIT_MASK);
}


/*******************************************************************************
* Function Name: TimeOut_ReadPeriodBuf
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
uint32 TimeOut_ReadPeriodBuf(void)
{
    return (TimeOut_PERIOD_BUF_REG & TimeOut_16BIT_MASK);
}


/*******************************************************************************
* Function Name: TimeOut_SetPeriodSwap
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
void TimeOut_SetPeriodSwap(uint32 swapEnable)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    TimeOut_CONTROL_REG &= (uint32)~TimeOut_RELOAD_PERIOD_MASK;
    TimeOut_CONTROL_REG |= ((uint32)((swapEnable & TimeOut_1BIT_MASK) <<
                                                            TimeOut_RELOAD_PERIOD_SHIFT));

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: TimeOut_WriteCompare
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
void TimeOut_WriteCompare(uint32 compare)
{
    #if (TimeOut_CY_TCPWM_4000)
        uint32 currentMode;
    #endif /* (TimeOut_CY_TCPWM_4000) */

    #if (TimeOut_CY_TCPWM_4000)
        currentMode = ((TimeOut_CONTROL_REG & TimeOut_UPDOWN_MASK) >> TimeOut_UPDOWN_SHIFT);

        if (((uint32)TimeOut__COUNT_DOWN == currentMode) && (0xFFFFu != compare))
        {
            compare++;
        }
        else if (((uint32)TimeOut__COUNT_UP == currentMode) && (0u != compare))
        {
            compare--;
        }
        else
        {
        }
        
    
    #endif /* (TimeOut_CY_TCPWM_4000) */
    
    TimeOut_COMP_CAP_REG = (compare & TimeOut_16BIT_MASK);
}


/*******************************************************************************
* Function Name: TimeOut_ReadCompare
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
uint32 TimeOut_ReadCompare(void)
{
    #if (TimeOut_CY_TCPWM_4000)
        uint32 currentMode;
        uint32 regVal;
    #endif /* (TimeOut_CY_TCPWM_4000) */

    #if (TimeOut_CY_TCPWM_4000)
        currentMode = ((TimeOut_CONTROL_REG & TimeOut_UPDOWN_MASK) >> TimeOut_UPDOWN_SHIFT);
        
        regVal = TimeOut_COMP_CAP_REG;
        
        if (((uint32)TimeOut__COUNT_DOWN == currentMode) && (0u != regVal))
        {
            regVal--;
        }
        else if (((uint32)TimeOut__COUNT_UP == currentMode) && (0xFFFFu != regVal))
        {
            regVal++;
        }
        else
        {
        }

        return (regVal & TimeOut_16BIT_MASK);
    #else
        return (TimeOut_COMP_CAP_REG & TimeOut_16BIT_MASK);
    #endif /* (TimeOut_CY_TCPWM_4000) */
}


/*******************************************************************************
* Function Name: TimeOut_WriteCompareBuf
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
void TimeOut_WriteCompareBuf(uint32 compareBuf)
{
    #if (TimeOut_CY_TCPWM_4000)
        uint32 currentMode;
    #endif /* (TimeOut_CY_TCPWM_4000) */

    #if (TimeOut_CY_TCPWM_4000)
        currentMode = ((TimeOut_CONTROL_REG & TimeOut_UPDOWN_MASK) >> TimeOut_UPDOWN_SHIFT);

        if (((uint32)TimeOut__COUNT_DOWN == currentMode) && (0xFFFFu != compareBuf))
        {
            compareBuf++;
        }
        else if (((uint32)TimeOut__COUNT_UP == currentMode) && (0u != compareBuf))
        {
            compareBuf --;
        }
        else
        {
        }
    #endif /* (TimeOut_CY_TCPWM_4000) */
    
    TimeOut_COMP_CAP_BUF_REG = (compareBuf & TimeOut_16BIT_MASK);
}


/*******************************************************************************
* Function Name: TimeOut_ReadCompareBuf
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
uint32 TimeOut_ReadCompareBuf(void)
{
    #if (TimeOut_CY_TCPWM_4000)
        uint32 currentMode;
        uint32 regVal;
    #endif /* (TimeOut_CY_TCPWM_4000) */

    #if (TimeOut_CY_TCPWM_4000)
        currentMode = ((TimeOut_CONTROL_REG & TimeOut_UPDOWN_MASK) >> TimeOut_UPDOWN_SHIFT);

        regVal = TimeOut_COMP_CAP_BUF_REG;
        
        if (((uint32)TimeOut__COUNT_DOWN == currentMode) && (0u != regVal))
        {
            regVal--;
        }
        else if (((uint32)TimeOut__COUNT_UP == currentMode) && (0xFFFFu != regVal))
        {
            regVal++;
        }
        else
        {
        }

        return (regVal & TimeOut_16BIT_MASK);
    #else
        return (TimeOut_COMP_CAP_BUF_REG & TimeOut_16BIT_MASK);
    #endif /* (TimeOut_CY_TCPWM_4000) */
}


/*******************************************************************************
* Function Name: TimeOut_ReadCapture
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
uint32 TimeOut_ReadCapture(void)
{
    return (TimeOut_COMP_CAP_REG & TimeOut_16BIT_MASK);
}


/*******************************************************************************
* Function Name: TimeOut_ReadCaptureBuf
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
uint32 TimeOut_ReadCaptureBuf(void)
{
    return (TimeOut_COMP_CAP_BUF_REG & TimeOut_16BIT_MASK);
}


/*******************************************************************************
* Function Name: TimeOut_SetCaptureMode
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
*     - TimeOut_TRIG_LEVEL     - Level
*     - TimeOut_TRIG_RISING    - Rising edge
*     - TimeOut_TRIG_FALLING   - Falling edge
*     - TimeOut_TRIG_BOTH      - Both rising and falling edge
*
* Return:
*  None
*
*******************************************************************************/
void TimeOut_SetCaptureMode(uint32 triggerMode)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    TimeOut_TRIG_CONTROL1_REG &= (uint32)~TimeOut_CAPTURE_MASK;
    TimeOut_TRIG_CONTROL1_REG |= triggerMode;

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: TimeOut_SetReloadMode
********************************************************************************
*
* Summary:
*  Sets the reload trigger mode. For Quadrature Decoder mode this is the index
*  input.
*
* Parameters:
*  triggerMode: Enumerated trigger mode value
*   Values:
*     - TimeOut_TRIG_LEVEL     - Level
*     - TimeOut_TRIG_RISING    - Rising edge
*     - TimeOut_TRIG_FALLING   - Falling edge
*     - TimeOut_TRIG_BOTH      - Both rising and falling edge
*
* Return:
*  None
*
*******************************************************************************/
void TimeOut_SetReloadMode(uint32 triggerMode)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    TimeOut_TRIG_CONTROL1_REG &= (uint32)~TimeOut_RELOAD_MASK;
    TimeOut_TRIG_CONTROL1_REG |= ((uint32)(triggerMode << TimeOut_RELOAD_SHIFT));

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: TimeOut_SetStartMode
********************************************************************************
*
* Summary:
*  Sets the start trigger mode. For Quadrature Decoder mode this is the
*  phiB input.
*
* Parameters:
*  triggerMode: Enumerated trigger mode value
*   Values:
*     - TimeOut_TRIG_LEVEL     - Level
*     - TimeOut_TRIG_RISING    - Rising edge
*     - TimeOut_TRIG_FALLING   - Falling edge
*     - TimeOut_TRIG_BOTH      - Both rising and falling edge
*
* Return:
*  None
*
*******************************************************************************/
void TimeOut_SetStartMode(uint32 triggerMode)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    TimeOut_TRIG_CONTROL1_REG &= (uint32)~TimeOut_START_MASK;
    TimeOut_TRIG_CONTROL1_REG |= ((uint32)(triggerMode << TimeOut_START_SHIFT));

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: TimeOut_SetStopMode
********************************************************************************
*
* Summary:
*  Sets the stop trigger mode. For PWM mode this is the kill input.
*
* Parameters:
*  triggerMode: Enumerated trigger mode value
*   Values:
*     - TimeOut_TRIG_LEVEL     - Level
*     - TimeOut_TRIG_RISING    - Rising edge
*     - TimeOut_TRIG_FALLING   - Falling edge
*     - TimeOut_TRIG_BOTH      - Both rising and falling edge
*
* Return:
*  None
*
*******************************************************************************/
void TimeOut_SetStopMode(uint32 triggerMode)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    TimeOut_TRIG_CONTROL1_REG &= (uint32)~TimeOut_STOP_MASK;
    TimeOut_TRIG_CONTROL1_REG |= ((uint32)(triggerMode << TimeOut_STOP_SHIFT));

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: TimeOut_SetCountMode
********************************************************************************
*
* Summary:
*  Sets the count trigger mode. For Quadrature Decoder mode this is the phiA
*  input.
*
* Parameters:
*  triggerMode: Enumerated trigger mode value
*   Values:
*     - TimeOut_TRIG_LEVEL     - Level
*     - TimeOut_TRIG_RISING    - Rising edge
*     - TimeOut_TRIG_FALLING   - Falling edge
*     - TimeOut_TRIG_BOTH      - Both rising and falling edge
*
* Return:
*  None
*
*******************************************************************************/
void TimeOut_SetCountMode(uint32 triggerMode)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    TimeOut_TRIG_CONTROL1_REG &= (uint32)~TimeOut_COUNT_MASK;
    TimeOut_TRIG_CONTROL1_REG |= ((uint32)(triggerMode << TimeOut_COUNT_SHIFT));

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: TimeOut_TriggerCommand
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
*     - TimeOut_CMD_CAPTURE    - Trigger Capture/Switch command
*     - TimeOut_CMD_RELOAD     - Trigger Reload/Index command
*     - TimeOut_CMD_STOP       - Trigger Stop/Kill command
*     - TimeOut_CMD_START      - Trigger Start/phiB command
*
* Return:
*  None
*
*******************************************************************************/
void TimeOut_TriggerCommand(uint32 mask, uint32 command)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    TimeOut_COMMAND_REG = ((uint32)(mask << command));

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: TimeOut_ReadStatus
********************************************************************************
*
* Summary:
*  Reads the status of the TimeOut.
*
* Parameters:
*  None
*
* Return:
*  Status
*   Values:
*     - TimeOut_STATUS_DOWN    - Set if counting down
*     - TimeOut_STATUS_RUNNING - Set if counter is running
*
*******************************************************************************/
uint32 TimeOut_ReadStatus(void)
{
    return ((TimeOut_STATUS_REG >> TimeOut_RUNNING_STATUS_SHIFT) |
            (TimeOut_STATUS_REG & TimeOut_STATUS_DOWN));
}


/*******************************************************************************
* Function Name: TimeOut_SetInterruptMode
********************************************************************************
*
* Summary:
*  Sets the interrupt mask to control which interrupt
*  requests generate the interrupt signal.
*
* Parameters:
*   interruptMask: Mask of bits to be enabled
*   Values:
*     - TimeOut_INTR_MASK_TC       - Terminal count mask
*     - TimeOut_INTR_MASK_CC_MATCH - Compare count / capture mask
*
* Return:
*  None
*
*******************************************************************************/
void TimeOut_SetInterruptMode(uint32 interruptMask)
{
    TimeOut_INTERRUPT_MASK_REG =  interruptMask;
}


/*******************************************************************************
* Function Name: TimeOut_GetInterruptSourceMasked
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
*     - TimeOut_INTR_MASK_TC       - Terminal count mask
*     - TimeOut_INTR_MASK_CC_MATCH - Compare count / capture mask
*
*******************************************************************************/
uint32 TimeOut_GetInterruptSourceMasked(void)
{
    return (TimeOut_INTERRUPT_MASKED_REG);
}


/*******************************************************************************
* Function Name: TimeOut_GetInterruptSource
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
*     - TimeOut_INTR_MASK_TC       - Terminal count mask
*     - TimeOut_INTR_MASK_CC_MATCH - Compare count / capture mask
*
*******************************************************************************/
uint32 TimeOut_GetInterruptSource(void)
{
    return (TimeOut_INTERRUPT_REQ_REG);
}


/*******************************************************************************
* Function Name: TimeOut_ClearInterrupt
********************************************************************************
*
* Summary:
*  Clears the interrupt request.
*
* Parameters:
*   interruptMask: Mask of interrupts to clear
*   Values:
*     - TimeOut_INTR_MASK_TC       - Terminal count mask
*     - TimeOut_INTR_MASK_CC_MATCH - Compare count / capture mask
*
* Return:
*  None
*
*******************************************************************************/
void TimeOut_ClearInterrupt(uint32 interruptMask)
{
    TimeOut_INTERRUPT_REQ_REG = interruptMask;
}


/*******************************************************************************
* Function Name: TimeOut_SetInterrupt
********************************************************************************
*
* Summary:
*  Sets a software interrupt request.
*
* Parameters:
*   interruptMask: Mask of interrupts to set
*   Values:
*     - TimeOut_INTR_MASK_TC       - Terminal count mask
*     - TimeOut_INTR_MASK_CC_MATCH - Compare count / capture mask
*
* Return:
*  None
*
*******************************************************************************/
void TimeOut_SetInterrupt(uint32 interruptMask)
{
    TimeOut_INTERRUPT_SET_REG = interruptMask;
}


/* [] END OF FILE */
