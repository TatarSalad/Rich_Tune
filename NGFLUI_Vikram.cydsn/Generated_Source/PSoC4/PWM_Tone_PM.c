/*******************************************************************************
* File Name: PWM_Tone_PM.c
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

#include "PWM_Tone.h"

static PWM_Tone_BACKUP_STRUCT PWM_Tone_backup;


/*******************************************************************************
* Function Name: PWM_Tone_SaveConfig
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
void PWM_Tone_SaveConfig(void)
{

}


/*******************************************************************************
* Function Name: PWM_Tone_Sleep
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
void PWM_Tone_Sleep(void)
{
    if(0u != (PWM_Tone_BLOCK_CONTROL_REG & PWM_Tone_MASK))
    {
        PWM_Tone_backup.enableState = 1u;
    }
    else
    {
        PWM_Tone_backup.enableState = 0u;
    }

    PWM_Tone_Stop();
    PWM_Tone_SaveConfig();
}


/*******************************************************************************
* Function Name: PWM_Tone_RestoreConfig
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
void PWM_Tone_RestoreConfig(void)
{

}


/*******************************************************************************
* Function Name: PWM_Tone_Wakeup
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
void PWM_Tone_Wakeup(void)
{
    PWM_Tone_RestoreConfig();

    if(0u != PWM_Tone_backup.enableState)
    {
        PWM_Tone_Enable();
    }
}


/* [] END OF FILE */
