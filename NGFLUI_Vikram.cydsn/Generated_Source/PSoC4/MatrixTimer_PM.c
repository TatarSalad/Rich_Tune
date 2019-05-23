/*******************************************************************************
* File Name: MatrixTimer_PM.c
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

#include "MatrixTimer.h"

static MatrixTimer_BACKUP_STRUCT MatrixTimer_backup;


/*******************************************************************************
* Function Name: MatrixTimer_SaveConfig
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
void MatrixTimer_SaveConfig(void)
{

}


/*******************************************************************************
* Function Name: MatrixTimer_Sleep
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
void MatrixTimer_Sleep(void)
{
    if(0u != (MatrixTimer_BLOCK_CONTROL_REG & MatrixTimer_MASK))
    {
        MatrixTimer_backup.enableState = 1u;
    }
    else
    {
        MatrixTimer_backup.enableState = 0u;
    }

    MatrixTimer_Stop();
    MatrixTimer_SaveConfig();
}


/*******************************************************************************
* Function Name: MatrixTimer_RestoreConfig
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
void MatrixTimer_RestoreConfig(void)
{

}


/*******************************************************************************
* Function Name: MatrixTimer_Wakeup
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
void MatrixTimer_Wakeup(void)
{
    MatrixTimer_RestoreConfig();

    if(0u != MatrixTimer_backup.enableState)
    {
        MatrixTimer_Enable();
    }
}


/* [] END OF FILE */
