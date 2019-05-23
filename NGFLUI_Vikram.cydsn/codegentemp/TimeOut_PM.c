/*******************************************************************************
* File Name: TimeOut_PM.c
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

#include "TimeOut.h"

static TimeOut_BACKUP_STRUCT TimeOut_backup;


/*******************************************************************************
* Function Name: TimeOut_SaveConfig
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
void TimeOut_SaveConfig(void)
{

}


/*******************************************************************************
* Function Name: TimeOut_Sleep
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
void TimeOut_Sleep(void)
{
    if(0u != (TimeOut_BLOCK_CONTROL_REG & TimeOut_MASK))
    {
        TimeOut_backup.enableState = 1u;
    }
    else
    {
        TimeOut_backup.enableState = 0u;
    }

    TimeOut_Stop();
    TimeOut_SaveConfig();
}


/*******************************************************************************
* Function Name: TimeOut_RestoreConfig
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
void TimeOut_RestoreConfig(void)
{

}


/*******************************************************************************
* Function Name: TimeOut_Wakeup
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
void TimeOut_Wakeup(void)
{
    TimeOut_RestoreConfig();

    if(0u != TimeOut_backup.enableState)
    {
        TimeOut_Enable();
    }
}


/* [] END OF FILE */
