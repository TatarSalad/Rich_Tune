/*******************************************************************************
* File Name: PWM_Freq_PM.c
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

#include "PWM_Freq.h"

static PWM_Freq_BACKUP_STRUCT PWM_Freq_backup;


/*******************************************************************************
* Function Name: PWM_Freq_SaveConfig
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
void PWM_Freq_SaveConfig(void)
{

}


/*******************************************************************************
* Function Name: PWM_Freq_Sleep
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
void PWM_Freq_Sleep(void)
{
    if(0u != (PWM_Freq_BLOCK_CONTROL_REG & PWM_Freq_MASK))
    {
        PWM_Freq_backup.enableState = 1u;
    }
    else
    {
        PWM_Freq_backup.enableState = 0u;
    }

    PWM_Freq_Stop();
    PWM_Freq_SaveConfig();
}


/*******************************************************************************
* Function Name: PWM_Freq_RestoreConfig
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
void PWM_Freq_RestoreConfig(void)
{

}


/*******************************************************************************
* Function Name: PWM_Freq_Wakeup
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
void PWM_Freq_Wakeup(void)
{
    PWM_Freq_RestoreConfig();

    if(0u != PWM_Freq_backup.enableState)
    {
        PWM_Freq_Enable();
    }
}


/* [] END OF FILE */
