/*******************************************************************************
* File Name: TimerOut_PM.c
* Version 2.10
*
* Description:
*  This file contains the setup, control, and status commands to support
*  the component operations in the low power mode.
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

static TimerOut_BACKUP_STRUCT TimerOut_backup;


/*******************************************************************************
* Function Name: TimerOut_SaveConfig
********************************************************************************
*
* Summary:
*  All configuration registers are retention. Nothing to save here.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void TimerOut_SaveConfig(void)
{

}


/*******************************************************************************
* Function Name: TimerOut_Sleep
********************************************************************************
*
* Summary:
*  Stops the component operation and saves the user configuration.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void TimerOut_Sleep(void)
{
    if(0u != (TimerOut_BLOCK_CONTROL_REG & TimerOut_MASK))
    {
        TimerOut_backup.enableState = 1u;
    }
    else
    {
        TimerOut_backup.enableState = 0u;
    }

    TimerOut_Stop();
    TimerOut_SaveConfig();
}


/*******************************************************************************
* Function Name: TimerOut_RestoreConfig
********************************************************************************
*
* Summary:
*  All configuration registers are retention. Nothing to restore here.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void TimerOut_RestoreConfig(void)
{

}


/*******************************************************************************
* Function Name: TimerOut_Wakeup
********************************************************************************
*
* Summary:
*  Restores the user configuration and restores the enable state.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void TimerOut_Wakeup(void)
{
    TimerOut_RestoreConfig();

    if(0u != TimerOut_backup.enableState)
    {
        TimerOut_Enable();
    }
}


/* [] END OF FILE */
