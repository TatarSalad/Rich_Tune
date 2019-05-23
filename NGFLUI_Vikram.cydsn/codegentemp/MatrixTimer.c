/*******************************************************************************
* File Name: MatrixTimer.c
* Version 2.10
*
* Description:
*  This file provides the source code to the API for the MatrixTimer
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

#include "MatrixTimer.h"

uint8 MatrixTimer_initVar = 0u;


/*******************************************************************************
* Function Name: MatrixTimer_Init
********************************************************************************
*
* Summary:
*  Initialize/Restore default MatrixTimer configuration.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void MatrixTimer_Init(void)
{

    /* Set values from customizer to CTRL */
    #if (MatrixTimer__QUAD == MatrixTimer_CONFIG)
        MatrixTimer_CONTROL_REG = MatrixTimer_CTRL_QUAD_BASE_CONFIG;
        
        /* Set values from customizer to CTRL1 */
        MatrixTimer_TRIG_CONTROL1_REG  = MatrixTimer_QUAD_SIGNALS_MODES;

        /* Set values from customizer to INTR */
        MatrixTimer_SetInterruptMode(MatrixTimer_QUAD_INTERRUPT_MASK);
        
         /* Set other values */
        MatrixTimer_SetCounterMode(MatrixTimer_COUNT_DOWN);
        MatrixTimer_WritePeriod(MatrixTimer_QUAD_PERIOD_INIT_VALUE);
        MatrixTimer_WriteCounter(MatrixTimer_QUAD_PERIOD_INIT_VALUE);
    #endif  /* (MatrixTimer__QUAD == MatrixTimer_CONFIG) */

    #if (MatrixTimer__TIMER == MatrixTimer_CONFIG)
        MatrixTimer_CONTROL_REG = MatrixTimer_CTRL_TIMER_BASE_CONFIG;
        
        /* Set values from customizer to CTRL1 */
        MatrixTimer_TRIG_CONTROL1_REG  = MatrixTimer_TIMER_SIGNALS_MODES;
    
        /* Set values from customizer to INTR */
        MatrixTimer_SetInterruptMode(MatrixTimer_TC_INTERRUPT_MASK);
        
        /* Set other values from customizer */
        MatrixTimer_WritePeriod(MatrixTimer_TC_PERIOD_VALUE );

        #if (MatrixTimer__COMPARE == MatrixTimer_TC_COMP_CAP_MODE)
            MatrixTimer_WriteCompare(MatrixTimer_TC_COMPARE_VALUE);

            #if (1u == MatrixTimer_TC_COMPARE_SWAP)
                MatrixTimer_SetCompareSwap(1u);
                MatrixTimer_WriteCompareBuf(MatrixTimer_TC_COMPARE_BUF_VALUE);
            #endif  /* (1u == MatrixTimer_TC_COMPARE_SWAP) */
        #endif  /* (MatrixTimer__COMPARE == MatrixTimer_TC_COMP_CAP_MODE) */

        /* Initialize counter value */
        #if (MatrixTimer_CY_TCPWM_V2 && MatrixTimer_TIMER_UPDOWN_CNT_USED && !MatrixTimer_CY_TCPWM_4000)
            MatrixTimer_WriteCounter(1u);
        #elif(MatrixTimer__COUNT_DOWN == MatrixTimer_TC_COUNTER_MODE)
            MatrixTimer_WriteCounter(MatrixTimer_TC_PERIOD_VALUE);
        #else
            MatrixTimer_WriteCounter(0u);
        #endif /* (MatrixTimer_CY_TCPWM_V2 && MatrixTimer_TIMER_UPDOWN_CNT_USED && !MatrixTimer_CY_TCPWM_4000) */
    #endif  /* (MatrixTimer__TIMER == MatrixTimer_CONFIG) */

    #if (MatrixTimer__PWM_SEL == MatrixTimer_CONFIG)
        MatrixTimer_CONTROL_REG = MatrixTimer_CTRL_PWM_BASE_CONFIG;

        #if (MatrixTimer__PWM_PR == MatrixTimer_PWM_MODE)
            MatrixTimer_CONTROL_REG |= MatrixTimer_CTRL_PWM_RUN_MODE;
            MatrixTimer_WriteCounter(MatrixTimer_PWM_PR_INIT_VALUE);
        #else
            MatrixTimer_CONTROL_REG |= MatrixTimer_CTRL_PWM_ALIGN | MatrixTimer_CTRL_PWM_KILL_EVENT;
            
            /* Initialize counter value */
            #if (MatrixTimer_CY_TCPWM_V2 && MatrixTimer_PWM_UPDOWN_CNT_USED && !MatrixTimer_CY_TCPWM_4000)
                MatrixTimer_WriteCounter(1u);
            #elif (MatrixTimer__RIGHT == MatrixTimer_PWM_ALIGN)
                MatrixTimer_WriteCounter(MatrixTimer_PWM_PERIOD_VALUE);
            #else 
                MatrixTimer_WriteCounter(0u);
            #endif  /* (MatrixTimer_CY_TCPWM_V2 && MatrixTimer_PWM_UPDOWN_CNT_USED && !MatrixTimer_CY_TCPWM_4000) */
        #endif  /* (MatrixTimer__PWM_PR == MatrixTimer_PWM_MODE) */

        #if (MatrixTimer__PWM_DT == MatrixTimer_PWM_MODE)
            MatrixTimer_CONTROL_REG |= MatrixTimer_CTRL_PWM_DEAD_TIME_CYCLE;
        #endif  /* (MatrixTimer__PWM_DT == MatrixTimer_PWM_MODE) */

        #if (MatrixTimer__PWM == MatrixTimer_PWM_MODE)
            MatrixTimer_CONTROL_REG |= MatrixTimer_CTRL_PWM_PRESCALER;
        #endif  /* (MatrixTimer__PWM == MatrixTimer_PWM_MODE) */

        /* Set values from customizer to CTRL1 */
        MatrixTimer_TRIG_CONTROL1_REG  = MatrixTimer_PWM_SIGNALS_MODES;
    
        /* Set values from customizer to INTR */
        MatrixTimer_SetInterruptMode(MatrixTimer_PWM_INTERRUPT_MASK);

        /* Set values from customizer to CTRL2 */
        #if (MatrixTimer__PWM_PR == MatrixTimer_PWM_MODE)
            MatrixTimer_TRIG_CONTROL2_REG =
                    (MatrixTimer_CC_MATCH_NO_CHANGE    |
                    MatrixTimer_OVERLOW_NO_CHANGE      |
                    MatrixTimer_UNDERFLOW_NO_CHANGE);
        #else
            #if (MatrixTimer__LEFT == MatrixTimer_PWM_ALIGN)
                MatrixTimer_TRIG_CONTROL2_REG = MatrixTimer_PWM_MODE_LEFT;
            #endif  /* ( MatrixTimer_PWM_LEFT == MatrixTimer_PWM_ALIGN) */

            #if (MatrixTimer__RIGHT == MatrixTimer_PWM_ALIGN)
                MatrixTimer_TRIG_CONTROL2_REG = MatrixTimer_PWM_MODE_RIGHT;
            #endif  /* ( MatrixTimer_PWM_RIGHT == MatrixTimer_PWM_ALIGN) */

            #if (MatrixTimer__CENTER == MatrixTimer_PWM_ALIGN)
                MatrixTimer_TRIG_CONTROL2_REG = MatrixTimer_PWM_MODE_CENTER;
            #endif  /* ( MatrixTimer_PWM_CENTER == MatrixTimer_PWM_ALIGN) */

            #if (MatrixTimer__ASYMMETRIC == MatrixTimer_PWM_ALIGN)
                MatrixTimer_TRIG_CONTROL2_REG = MatrixTimer_PWM_MODE_ASYM;
            #endif  /* (MatrixTimer__ASYMMETRIC == MatrixTimer_PWM_ALIGN) */
        #endif  /* (MatrixTimer__PWM_PR == MatrixTimer_PWM_MODE) */

        /* Set other values from customizer */
        MatrixTimer_WritePeriod(MatrixTimer_PWM_PERIOD_VALUE );
        MatrixTimer_WriteCompare(MatrixTimer_PWM_COMPARE_VALUE);

        #if (1u == MatrixTimer_PWM_COMPARE_SWAP)
            MatrixTimer_SetCompareSwap(1u);
            MatrixTimer_WriteCompareBuf(MatrixTimer_PWM_COMPARE_BUF_VALUE);
        #endif  /* (1u == MatrixTimer_PWM_COMPARE_SWAP) */

        #if (1u == MatrixTimer_PWM_PERIOD_SWAP)
            MatrixTimer_SetPeriodSwap(1u);
            MatrixTimer_WritePeriodBuf(MatrixTimer_PWM_PERIOD_BUF_VALUE);
        #endif  /* (1u == MatrixTimer_PWM_PERIOD_SWAP) */
    #endif  /* (MatrixTimer__PWM_SEL == MatrixTimer_CONFIG) */
    
}


/*******************************************************************************
* Function Name: MatrixTimer_Enable
********************************************************************************
*
* Summary:
*  Enables the MatrixTimer.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void MatrixTimer_Enable(void)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();
    MatrixTimer_BLOCK_CONTROL_REG |= MatrixTimer_MASK;
    CyExitCriticalSection(enableInterrupts);

    /* Start Timer or PWM if start input is absent */
    #if (MatrixTimer__PWM_SEL == MatrixTimer_CONFIG)
        #if (0u == MatrixTimer_PWM_START_SIGNAL_PRESENT)
            MatrixTimer_TriggerCommand(MatrixTimer_MASK, MatrixTimer_CMD_START);
        #endif /* (0u == MatrixTimer_PWM_START_SIGNAL_PRESENT) */
    #endif /* (MatrixTimer__PWM_SEL == MatrixTimer_CONFIG) */

    #if (MatrixTimer__TIMER == MatrixTimer_CONFIG)
        #if (0u == MatrixTimer_TC_START_SIGNAL_PRESENT)
            MatrixTimer_TriggerCommand(MatrixTimer_MASK, MatrixTimer_CMD_START);
        #endif /* (0u == MatrixTimer_TC_START_SIGNAL_PRESENT) */
    #endif /* (MatrixTimer__TIMER == MatrixTimer_CONFIG) */
    
    #if (MatrixTimer__QUAD == MatrixTimer_CONFIG)
        #if (0u != MatrixTimer_QUAD_AUTO_START)
            MatrixTimer_TriggerCommand(MatrixTimer_MASK, MatrixTimer_CMD_RELOAD);
        #endif /* (0u != MatrixTimer_QUAD_AUTO_START) */
    #endif  /* (MatrixTimer__QUAD == MatrixTimer_CONFIG) */
}


/*******************************************************************************
* Function Name: MatrixTimer_Start
********************************************************************************
*
* Summary:
*  Initializes the MatrixTimer with default customizer
*  values when called the first time and enables the MatrixTimer.
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
*  MatrixTimer_initVar: global variable is used to indicate initial
*  configuration of this component.  The variable is initialized to zero and set
*  to 1 the first time MatrixTimer_Start() is called. This allows
*  enabling/disabling a component without re-initialization in all subsequent
*  calls to the MatrixTimer_Start() routine.
*
*******************************************************************************/
void MatrixTimer_Start(void)
{
    if (0u == MatrixTimer_initVar)
    {
        MatrixTimer_Init();
        MatrixTimer_initVar = 1u;
    }

    MatrixTimer_Enable();
}


/*******************************************************************************
* Function Name: MatrixTimer_Stop
********************************************************************************
*
* Summary:
*  Disables the MatrixTimer.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void MatrixTimer_Stop(void)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    MatrixTimer_BLOCK_CONTROL_REG &= (uint32)~MatrixTimer_MASK;

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: MatrixTimer_SetMode
********************************************************************************
*
* Summary:
*  Sets the operation mode of the MatrixTimer. This function is used when
*  configured as a generic MatrixTimer and the actual mode of operation is
*  set at runtime. The mode must be set while the component is disabled.
*
* Parameters:
*  mode: Mode for the MatrixTimer to operate in
*   Values:
*   - MatrixTimer_MODE_TIMER_COMPARE - Timer / Counter with
*                                                 compare capability
*         - MatrixTimer_MODE_TIMER_CAPTURE - Timer / Counter with
*                                                 capture capability
*         - MatrixTimer_MODE_QUAD - Quadrature decoder
*         - MatrixTimer_MODE_PWM - PWM
*         - MatrixTimer_MODE_PWM_DT - PWM with dead time
*         - MatrixTimer_MODE_PWM_PR - PWM with pseudo random capability
*
* Return:
*  None
*
*******************************************************************************/
void MatrixTimer_SetMode(uint32 mode)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    MatrixTimer_CONTROL_REG &= (uint32)~MatrixTimer_MODE_MASK;
    MatrixTimer_CONTROL_REG |= mode;

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: MatrixTimer_SetQDMode
********************************************************************************
*
* Summary:
*  Sets the the Quadrature Decoder to one of the 3 supported modes.
*  Its functionality is only applicable to Quadrature Decoder operation.
*
* Parameters:
*  qdMode: Quadrature Decoder mode
*   Values:
*         - MatrixTimer_MODE_X1 - Counts on phi 1 rising
*         - MatrixTimer_MODE_X2 - Counts on both edges of phi1 (2x faster)
*         - MatrixTimer_MODE_X4 - Counts on both edges of phi1 and phi2
*                                        (4x faster)
*
* Return:
*  None
*
*******************************************************************************/
void MatrixTimer_SetQDMode(uint32 qdMode)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    MatrixTimer_CONTROL_REG &= (uint32)~MatrixTimer_QUAD_MODE_MASK;
    MatrixTimer_CONTROL_REG |= qdMode;

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: MatrixTimer_SetPrescaler
********************************************************************************
*
* Summary:
*  Sets the prescaler value that is applied to the clock input.  Not applicable
*  to a PWM with the dead time mode or Quadrature Decoder mode.
*
* Parameters:
*  prescaler: Prescaler divider value
*   Values:
*         - MatrixTimer_PRESCALE_DIVBY1    - Divide by 1 (no prescaling)
*         - MatrixTimer_PRESCALE_DIVBY2    - Divide by 2
*         - MatrixTimer_PRESCALE_DIVBY4    - Divide by 4
*         - MatrixTimer_PRESCALE_DIVBY8    - Divide by 8
*         - MatrixTimer_PRESCALE_DIVBY16   - Divide by 16
*         - MatrixTimer_PRESCALE_DIVBY32   - Divide by 32
*         - MatrixTimer_PRESCALE_DIVBY64   - Divide by 64
*         - MatrixTimer_PRESCALE_DIVBY128  - Divide by 128
*
* Return:
*  None
*
*******************************************************************************/
void MatrixTimer_SetPrescaler(uint32 prescaler)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    MatrixTimer_CONTROL_REG &= (uint32)~MatrixTimer_PRESCALER_MASK;
    MatrixTimer_CONTROL_REG |= prescaler;

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: MatrixTimer_SetOneShot
********************************************************************************
*
* Summary:
*  Writes the register that controls whether the MatrixTimer runs
*  continuously or stops when terminal count is reached.  By default the
*  MatrixTimer operates in the continuous mode.
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
void MatrixTimer_SetOneShot(uint32 oneShotEnable)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    MatrixTimer_CONTROL_REG &= (uint32)~MatrixTimer_ONESHOT_MASK;
    MatrixTimer_CONTROL_REG |= ((uint32)((oneShotEnable & MatrixTimer_1BIT_MASK) <<
                                                               MatrixTimer_ONESHOT_SHIFT));

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: MatrixTimer_SetPWMMode
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
void MatrixTimer_SetPWMMode(uint32 modeMask)
{
    MatrixTimer_TRIG_CONTROL2_REG = (modeMask & MatrixTimer_6BIT_MASK);
}



/*******************************************************************************
* Function Name: MatrixTimer_SetPWMSyncKill
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
void MatrixTimer_SetPWMSyncKill(uint32 syncKillEnable)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    MatrixTimer_CONTROL_REG &= (uint32)~MatrixTimer_PWM_SYNC_KILL_MASK;
    MatrixTimer_CONTROL_REG |= ((uint32)((syncKillEnable & MatrixTimer_1BIT_MASK)  <<
                                               MatrixTimer_PWM_SYNC_KILL_SHIFT));

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: MatrixTimer_SetPWMStopOnKill
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
void MatrixTimer_SetPWMStopOnKill(uint32 stopOnKillEnable)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    MatrixTimer_CONTROL_REG &= (uint32)~MatrixTimer_PWM_STOP_KILL_MASK;
    MatrixTimer_CONTROL_REG |= ((uint32)((stopOnKillEnable & MatrixTimer_1BIT_MASK)  <<
                                                         MatrixTimer_PWM_STOP_KILL_SHIFT));

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: MatrixTimer_SetPWMDeadTime
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
void MatrixTimer_SetPWMDeadTime(uint32 deadTime)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    MatrixTimer_CONTROL_REG &= (uint32)~MatrixTimer_PRESCALER_MASK;
    MatrixTimer_CONTROL_REG |= ((uint32)((deadTime & MatrixTimer_8BIT_MASK) <<
                                                          MatrixTimer_PRESCALER_SHIFT));

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: MatrixTimer_SetPWMInvert
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
*         - MatrixTimer_INVERT_LINE   - Inverts the line output
*         - MatrixTimer_INVERT_LINE_N - Inverts the line_n output
*
* Return:
*  None
*
*******************************************************************************/
void MatrixTimer_SetPWMInvert(uint32 mask)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    MatrixTimer_CONTROL_REG &= (uint32)~MatrixTimer_INV_OUT_MASK;
    MatrixTimer_CONTROL_REG |= mask;

    CyExitCriticalSection(enableInterrupts);
}



/*******************************************************************************
* Function Name: MatrixTimer_WriteCounter
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
void MatrixTimer_WriteCounter(uint32 count)
{
    MatrixTimer_COUNTER_REG = (count & MatrixTimer_16BIT_MASK);
}


/*******************************************************************************
* Function Name: MatrixTimer_ReadCounter
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
uint32 MatrixTimer_ReadCounter(void)
{
    return (MatrixTimer_COUNTER_REG & MatrixTimer_16BIT_MASK);
}


/*******************************************************************************
* Function Name: MatrixTimer_SetCounterMode
********************************************************************************
*
* Summary:
*  Sets the counter mode.  Applicable to all modes except Quadrature Decoder
*  and the PWM with a pseudo random output.
*
* Parameters:
*  counterMode: Enumerated counter type values
*   Values:
*     - MatrixTimer_COUNT_UP       - Counts up
*     - MatrixTimer_COUNT_DOWN     - Counts down
*     - MatrixTimer_COUNT_UPDOWN0  - Counts up and down. Terminal count
*                                         generated when counter reaches 0
*     - MatrixTimer_COUNT_UPDOWN1  - Counts up and down. Terminal count
*                                         generated both when counter reaches 0
*                                         and period
*
* Return:
*  None
*
*******************************************************************************/
void MatrixTimer_SetCounterMode(uint32 counterMode)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    MatrixTimer_CONTROL_REG &= (uint32)~MatrixTimer_UPDOWN_MASK;
    MatrixTimer_CONTROL_REG |= counterMode;

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: MatrixTimer_WritePeriod
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
void MatrixTimer_WritePeriod(uint32 period)
{
    MatrixTimer_PERIOD_REG = (period & MatrixTimer_16BIT_MASK);
}


/*******************************************************************************
* Function Name: MatrixTimer_ReadPeriod
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
uint32 MatrixTimer_ReadPeriod(void)
{
    return (MatrixTimer_PERIOD_REG & MatrixTimer_16BIT_MASK);
}


/*******************************************************************************
* Function Name: MatrixTimer_SetCompareSwap
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
void MatrixTimer_SetCompareSwap(uint32 swapEnable)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    MatrixTimer_CONTROL_REG &= (uint32)~MatrixTimer_RELOAD_CC_MASK;
    MatrixTimer_CONTROL_REG |= (swapEnable & MatrixTimer_1BIT_MASK);

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: MatrixTimer_WritePeriodBuf
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
void MatrixTimer_WritePeriodBuf(uint32 periodBuf)
{
    MatrixTimer_PERIOD_BUF_REG = (periodBuf & MatrixTimer_16BIT_MASK);
}


/*******************************************************************************
* Function Name: MatrixTimer_ReadPeriodBuf
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
uint32 MatrixTimer_ReadPeriodBuf(void)
{
    return (MatrixTimer_PERIOD_BUF_REG & MatrixTimer_16BIT_MASK);
}


/*******************************************************************************
* Function Name: MatrixTimer_SetPeriodSwap
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
void MatrixTimer_SetPeriodSwap(uint32 swapEnable)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    MatrixTimer_CONTROL_REG &= (uint32)~MatrixTimer_RELOAD_PERIOD_MASK;
    MatrixTimer_CONTROL_REG |= ((uint32)((swapEnable & MatrixTimer_1BIT_MASK) <<
                                                            MatrixTimer_RELOAD_PERIOD_SHIFT));

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: MatrixTimer_WriteCompare
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
void MatrixTimer_WriteCompare(uint32 compare)
{
    #if (MatrixTimer_CY_TCPWM_4000)
        uint32 currentMode;
    #endif /* (MatrixTimer_CY_TCPWM_4000) */

    #if (MatrixTimer_CY_TCPWM_4000)
        currentMode = ((MatrixTimer_CONTROL_REG & MatrixTimer_UPDOWN_MASK) >> MatrixTimer_UPDOWN_SHIFT);

        if (((uint32)MatrixTimer__COUNT_DOWN == currentMode) && (0xFFFFu != compare))
        {
            compare++;
        }
        else if (((uint32)MatrixTimer__COUNT_UP == currentMode) && (0u != compare))
        {
            compare--;
        }
        else
        {
        }
        
    
    #endif /* (MatrixTimer_CY_TCPWM_4000) */
    
    MatrixTimer_COMP_CAP_REG = (compare & MatrixTimer_16BIT_MASK);
}


/*******************************************************************************
* Function Name: MatrixTimer_ReadCompare
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
uint32 MatrixTimer_ReadCompare(void)
{
    #if (MatrixTimer_CY_TCPWM_4000)
        uint32 currentMode;
        uint32 regVal;
    #endif /* (MatrixTimer_CY_TCPWM_4000) */

    #if (MatrixTimer_CY_TCPWM_4000)
        currentMode = ((MatrixTimer_CONTROL_REG & MatrixTimer_UPDOWN_MASK) >> MatrixTimer_UPDOWN_SHIFT);
        
        regVal = MatrixTimer_COMP_CAP_REG;
        
        if (((uint32)MatrixTimer__COUNT_DOWN == currentMode) && (0u != regVal))
        {
            regVal--;
        }
        else if (((uint32)MatrixTimer__COUNT_UP == currentMode) && (0xFFFFu != regVal))
        {
            regVal++;
        }
        else
        {
        }

        return (regVal & MatrixTimer_16BIT_MASK);
    #else
        return (MatrixTimer_COMP_CAP_REG & MatrixTimer_16BIT_MASK);
    #endif /* (MatrixTimer_CY_TCPWM_4000) */
}


/*******************************************************************************
* Function Name: MatrixTimer_WriteCompareBuf
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
void MatrixTimer_WriteCompareBuf(uint32 compareBuf)
{
    #if (MatrixTimer_CY_TCPWM_4000)
        uint32 currentMode;
    #endif /* (MatrixTimer_CY_TCPWM_4000) */

    #if (MatrixTimer_CY_TCPWM_4000)
        currentMode = ((MatrixTimer_CONTROL_REG & MatrixTimer_UPDOWN_MASK) >> MatrixTimer_UPDOWN_SHIFT);

        if (((uint32)MatrixTimer__COUNT_DOWN == currentMode) && (0xFFFFu != compareBuf))
        {
            compareBuf++;
        }
        else if (((uint32)MatrixTimer__COUNT_UP == currentMode) && (0u != compareBuf))
        {
            compareBuf --;
        }
        else
        {
        }
    #endif /* (MatrixTimer_CY_TCPWM_4000) */
    
    MatrixTimer_COMP_CAP_BUF_REG = (compareBuf & MatrixTimer_16BIT_MASK);
}


/*******************************************************************************
* Function Name: MatrixTimer_ReadCompareBuf
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
uint32 MatrixTimer_ReadCompareBuf(void)
{
    #if (MatrixTimer_CY_TCPWM_4000)
        uint32 currentMode;
        uint32 regVal;
    #endif /* (MatrixTimer_CY_TCPWM_4000) */

    #if (MatrixTimer_CY_TCPWM_4000)
        currentMode = ((MatrixTimer_CONTROL_REG & MatrixTimer_UPDOWN_MASK) >> MatrixTimer_UPDOWN_SHIFT);

        regVal = MatrixTimer_COMP_CAP_BUF_REG;
        
        if (((uint32)MatrixTimer__COUNT_DOWN == currentMode) && (0u != regVal))
        {
            regVal--;
        }
        else if (((uint32)MatrixTimer__COUNT_UP == currentMode) && (0xFFFFu != regVal))
        {
            regVal++;
        }
        else
        {
        }

        return (regVal & MatrixTimer_16BIT_MASK);
    #else
        return (MatrixTimer_COMP_CAP_BUF_REG & MatrixTimer_16BIT_MASK);
    #endif /* (MatrixTimer_CY_TCPWM_4000) */
}


/*******************************************************************************
* Function Name: MatrixTimer_ReadCapture
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
uint32 MatrixTimer_ReadCapture(void)
{
    return (MatrixTimer_COMP_CAP_REG & MatrixTimer_16BIT_MASK);
}


/*******************************************************************************
* Function Name: MatrixTimer_ReadCaptureBuf
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
uint32 MatrixTimer_ReadCaptureBuf(void)
{
    return (MatrixTimer_COMP_CAP_BUF_REG & MatrixTimer_16BIT_MASK);
}


/*******************************************************************************
* Function Name: MatrixTimer_SetCaptureMode
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
*     - MatrixTimer_TRIG_LEVEL     - Level
*     - MatrixTimer_TRIG_RISING    - Rising edge
*     - MatrixTimer_TRIG_FALLING   - Falling edge
*     - MatrixTimer_TRIG_BOTH      - Both rising and falling edge
*
* Return:
*  None
*
*******************************************************************************/
void MatrixTimer_SetCaptureMode(uint32 triggerMode)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    MatrixTimer_TRIG_CONTROL1_REG &= (uint32)~MatrixTimer_CAPTURE_MASK;
    MatrixTimer_TRIG_CONTROL1_REG |= triggerMode;

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: MatrixTimer_SetReloadMode
********************************************************************************
*
* Summary:
*  Sets the reload trigger mode. For Quadrature Decoder mode this is the index
*  input.
*
* Parameters:
*  triggerMode: Enumerated trigger mode value
*   Values:
*     - MatrixTimer_TRIG_LEVEL     - Level
*     - MatrixTimer_TRIG_RISING    - Rising edge
*     - MatrixTimer_TRIG_FALLING   - Falling edge
*     - MatrixTimer_TRIG_BOTH      - Both rising and falling edge
*
* Return:
*  None
*
*******************************************************************************/
void MatrixTimer_SetReloadMode(uint32 triggerMode)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    MatrixTimer_TRIG_CONTROL1_REG &= (uint32)~MatrixTimer_RELOAD_MASK;
    MatrixTimer_TRIG_CONTROL1_REG |= ((uint32)(triggerMode << MatrixTimer_RELOAD_SHIFT));

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: MatrixTimer_SetStartMode
********************************************************************************
*
* Summary:
*  Sets the start trigger mode. For Quadrature Decoder mode this is the
*  phiB input.
*
* Parameters:
*  triggerMode: Enumerated trigger mode value
*   Values:
*     - MatrixTimer_TRIG_LEVEL     - Level
*     - MatrixTimer_TRIG_RISING    - Rising edge
*     - MatrixTimer_TRIG_FALLING   - Falling edge
*     - MatrixTimer_TRIG_BOTH      - Both rising and falling edge
*
* Return:
*  None
*
*******************************************************************************/
void MatrixTimer_SetStartMode(uint32 triggerMode)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    MatrixTimer_TRIG_CONTROL1_REG &= (uint32)~MatrixTimer_START_MASK;
    MatrixTimer_TRIG_CONTROL1_REG |= ((uint32)(triggerMode << MatrixTimer_START_SHIFT));

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: MatrixTimer_SetStopMode
********************************************************************************
*
* Summary:
*  Sets the stop trigger mode. For PWM mode this is the kill input.
*
* Parameters:
*  triggerMode: Enumerated trigger mode value
*   Values:
*     - MatrixTimer_TRIG_LEVEL     - Level
*     - MatrixTimer_TRIG_RISING    - Rising edge
*     - MatrixTimer_TRIG_FALLING   - Falling edge
*     - MatrixTimer_TRIG_BOTH      - Both rising and falling edge
*
* Return:
*  None
*
*******************************************************************************/
void MatrixTimer_SetStopMode(uint32 triggerMode)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    MatrixTimer_TRIG_CONTROL1_REG &= (uint32)~MatrixTimer_STOP_MASK;
    MatrixTimer_TRIG_CONTROL1_REG |= ((uint32)(triggerMode << MatrixTimer_STOP_SHIFT));

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: MatrixTimer_SetCountMode
********************************************************************************
*
* Summary:
*  Sets the count trigger mode. For Quadrature Decoder mode this is the phiA
*  input.
*
* Parameters:
*  triggerMode: Enumerated trigger mode value
*   Values:
*     - MatrixTimer_TRIG_LEVEL     - Level
*     - MatrixTimer_TRIG_RISING    - Rising edge
*     - MatrixTimer_TRIG_FALLING   - Falling edge
*     - MatrixTimer_TRIG_BOTH      - Both rising and falling edge
*
* Return:
*  None
*
*******************************************************************************/
void MatrixTimer_SetCountMode(uint32 triggerMode)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    MatrixTimer_TRIG_CONTROL1_REG &= (uint32)~MatrixTimer_COUNT_MASK;
    MatrixTimer_TRIG_CONTROL1_REG |= ((uint32)(triggerMode << MatrixTimer_COUNT_SHIFT));

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: MatrixTimer_TriggerCommand
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
*     - MatrixTimer_CMD_CAPTURE    - Trigger Capture/Switch command
*     - MatrixTimer_CMD_RELOAD     - Trigger Reload/Index command
*     - MatrixTimer_CMD_STOP       - Trigger Stop/Kill command
*     - MatrixTimer_CMD_START      - Trigger Start/phiB command
*
* Return:
*  None
*
*******************************************************************************/
void MatrixTimer_TriggerCommand(uint32 mask, uint32 command)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    MatrixTimer_COMMAND_REG = ((uint32)(mask << command));

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: MatrixTimer_ReadStatus
********************************************************************************
*
* Summary:
*  Reads the status of the MatrixTimer.
*
* Parameters:
*  None
*
* Return:
*  Status
*   Values:
*     - MatrixTimer_STATUS_DOWN    - Set if counting down
*     - MatrixTimer_STATUS_RUNNING - Set if counter is running
*
*******************************************************************************/
uint32 MatrixTimer_ReadStatus(void)
{
    return ((MatrixTimer_STATUS_REG >> MatrixTimer_RUNNING_STATUS_SHIFT) |
            (MatrixTimer_STATUS_REG & MatrixTimer_STATUS_DOWN));
}


/*******************************************************************************
* Function Name: MatrixTimer_SetInterruptMode
********************************************************************************
*
* Summary:
*  Sets the interrupt mask to control which interrupt
*  requests generate the interrupt signal.
*
* Parameters:
*   interruptMask: Mask of bits to be enabled
*   Values:
*     - MatrixTimer_INTR_MASK_TC       - Terminal count mask
*     - MatrixTimer_INTR_MASK_CC_MATCH - Compare count / capture mask
*
* Return:
*  None
*
*******************************************************************************/
void MatrixTimer_SetInterruptMode(uint32 interruptMask)
{
    MatrixTimer_INTERRUPT_MASK_REG =  interruptMask;
}


/*******************************************************************************
* Function Name: MatrixTimer_GetInterruptSourceMasked
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
*     - MatrixTimer_INTR_MASK_TC       - Terminal count mask
*     - MatrixTimer_INTR_MASK_CC_MATCH - Compare count / capture mask
*
*******************************************************************************/
uint32 MatrixTimer_GetInterruptSourceMasked(void)
{
    return (MatrixTimer_INTERRUPT_MASKED_REG);
}


/*******************************************************************************
* Function Name: MatrixTimer_GetInterruptSource
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
*     - MatrixTimer_INTR_MASK_TC       - Terminal count mask
*     - MatrixTimer_INTR_MASK_CC_MATCH - Compare count / capture mask
*
*******************************************************************************/
uint32 MatrixTimer_GetInterruptSource(void)
{
    return (MatrixTimer_INTERRUPT_REQ_REG);
}


/*******************************************************************************
* Function Name: MatrixTimer_ClearInterrupt
********************************************************************************
*
* Summary:
*  Clears the interrupt request.
*
* Parameters:
*   interruptMask: Mask of interrupts to clear
*   Values:
*     - MatrixTimer_INTR_MASK_TC       - Terminal count mask
*     - MatrixTimer_INTR_MASK_CC_MATCH - Compare count / capture mask
*
* Return:
*  None
*
*******************************************************************************/
void MatrixTimer_ClearInterrupt(uint32 interruptMask)
{
    MatrixTimer_INTERRUPT_REQ_REG = interruptMask;
}


/*******************************************************************************
* Function Name: MatrixTimer_SetInterrupt
********************************************************************************
*
* Summary:
*  Sets a software interrupt request.
*
* Parameters:
*   interruptMask: Mask of interrupts to set
*   Values:
*     - MatrixTimer_INTR_MASK_TC       - Terminal count mask
*     - MatrixTimer_INTR_MASK_CC_MATCH - Compare count / capture mask
*
* Return:
*  None
*
*******************************************************************************/
void MatrixTimer_SetInterrupt(uint32 interruptMask)
{
    MatrixTimer_INTERRUPT_SET_REG = interruptMask;
}


/* [] END OF FILE */
