/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include "project.h"
#include "stdio.h"

/* uncomment following statement to enable CapSense tuner */
#define TUNER_ENABLED               (1u)

#define CMOD (1u)
#define SHIELD_ENABLED (1u)
#define SHIELD_TANK_CAPACITOR (1u)

#define SLAVE_BUFFER_FIXED ((CMOD + SHIELD_ENABLED + SHIELD_TANK_CAPACITOR) * 2)

#define LED_OFF 0u
#define LED_ON 1u

#define LOW 0u
#define HI 1u

#define FALSE 0u
#define TRUE 1u

#define NO_KEY_PRESSED 0xff

enum
{
	KEY00,
	KEY01,
};


int main(void)
{
    uint16 i;
	uint8 activeKey = NO_KEY_PRESSED;	/* number of the most recent key pressed */
	uint8 previousKey = NO_KEY_PRESSED;	/* keep track of last key pressed ... to be used in "new event" test */
	uint8 multipleKeyFlag = FALSE;
	uint8 numActiveSensors;	/* number of active sensors in a given scan cycle */
	char printBuf[60]; /* buffer used for sprintf */
	
	CyGlobalIntEnable; /* Enable global interrupts. */

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
	
	HB_LED_Write(LED_ON);
	CyDelay(500);
	HB_LED_Write(LED_OFF);
	CyDelay(500);	
	
	#if TUNER_ENABLED || CapSense_SELF_TEST_EN
		#ifdef CY_SCB_EZI2C_EZI2C_H
			/* Start EZI2C block */
		    EZI2C_Start();
		#endif
	#endif
    
    #if TUNER_ENABLED
		#ifdef CY_SCB_EZI2C_EZI2C_H
			/* Set up I2C communication data buffer with CapSense data structure 
	        to be exposed to I2C master on a primary slave address request
	        */
	        EZI2C_EzI2CSetBuffer1(sizeof(CapSense_dsRam), sizeof(CapSense_dsRam),(uint8 *)&CapSense_dsRam);
		#endif
    #endif
	
	#ifdef CY_SCB_UART_H
		UART_Start();
		UART_UartPutString("\nSlave UART started\r\n");
	#endif
	
	CapSense_Start(); /* Initialize component */
	
	#if CapSense_SELF_TEST_EN
		uint16 wdgtNumber, snsNumber, totalNumSensors = 0;
		uint32 capacitanceValue;
		uint8 slaveBuffer[SLAVE_BUFFER_FIXED + CapSense_TOTAL_CSD_SENSORS];
		CapSense_FLASH_WD_STRUCT const *ptrFlashWidget;
		CapSense_RAM_SNS_STRUCT *ptrSns;
		
		#if EZI2C_EZI2C_NUMBER_OF_ADDRESSES
			EZI2C_EzI2CSetBuffer2(sizeof(slaveBuffer), 0, (uint8 *)&slaveBuffer);
		#endif
		
		capacitanceValue = CapSense_GetExtCapCapacitance(CapSense_TST_CMOD_ID);
		slaveBuffer[0] = (uint8) (capacitanceValue >> 8);
		slaveBuffer[1] = (uint8) (capacitanceValue >> 0);
		#ifdef CY_SCB_UART_H
			sprintf(printBuf, "Cmod value = %lu pF\r\n", capacitanceValue);
			UART_UartPutString(printBuf);
		#endif
		
		capacitanceValue = CapSense_GetExtCapCapacitance(CapSense_TST_CSH_ID);
		slaveBuffer[2] = (uint8) (capacitanceValue >> 8);
		slaveBuffer[3] = (uint8) (capacitanceValue >> 0);
		#ifdef CY_SCB_UART_H
			sprintf(printBuf, "Csh_tank value = %lu pF\r\n", capacitanceValue);
			UART_UartPutString(printBuf);
		#endif
		
		capacitanceValue = CapSense_GetShieldCapacitance();
		slaveBuffer[4] = (uint8) (capacitanceValue >> 8);
		slaveBuffer[5] = (uint8) (capacitanceValue >> 0);
		#ifdef CY_SCB_UART_H
			sprintf(printBuf, "Shield capacitance = %lu pF\r\n\n", capacitanceValue);
			UART_UartPutString(printBuf);
		#endif
					
		for(wdgtNumber = 0; wdgtNumber < CapSense_TOTAL_CSD_WIDGETS; wdgtNumber++)
		{
			ptrFlashWidget = &CapSense_dsFlash.wdgtArray[wdgtNumber];
			for(snsNumber = 0; snsNumber < ptrFlashWidget->totalNumSns; snsNumber++)
			{
				capacitanceValue = CapSense_GetSensorCapacitance(wdgtNumber, snsNumber);
				slaveBuffer[SLAVE_BUFFER_FIXED + totalNumSensors] = (uint8) capacitanceValue;
				#ifdef CY_SCB_UART_H
					sprintf(printBuf, "Sensor %2d Cp = %2lu pF\r\n", totalNumSensors, capacitanceValue);
					UART_UartPutString(printBuf);
				#endif
				totalNumSensors++;
			}
		}
		
		#ifdef CY_SCB_UART_H
			UART_UartPutString("\r\n\n");
		#endif
				
	#endif // CapSense_SELF_TEST_EN
	
	CapSense_ScanAllWidgets(); /* Scan all widgets */
	
	while(CapSense_IsBusy()); /* wait for scan to complete */
	
	CyDelay(500);
	
	CapSense_InitializeAllBaselines(); /* re-initialize baselines before starting forever loop */

    for(;;)
    {
        /* Place your application code here. */
		
		/* Scan all widgets. */
	    CapSense_ScanAllWidgets(); 
		
		/* Wait for scan to complete. */
        while(!(CapSense_NOT_BUSY == CapSense_IsBusy()))
			;

        /* Process all widgets. */
        CapSense_ProcessAllWidgets();
		
		#ifdef TUNER_ENABLED
			CapSense_RunTuner(); /* To sync with Tuner application */
		#endif
		
		/* Check if any widget is active. */
		if(CapSense_IsAnyWidgetActive())
		{
			HB_LED_Write(LED_ON);
			
			numActiveSensors = 0;	/* clear sensor count */
			
			/* update active key and count */
			for(i = CapSense_START_WDGT_ID; i < CapSense_TOTAL_CSD_WIDGETS; i++)
			{
				if(CapSense_IsWidgetActive(i))
				{
					numActiveSensors++;	/* will be used to test whether one key or multiple keys are active */
					activeKey = i - CapSense_START_WDGT_ID;	/* stores the active key from this scan */
				}
			}
			
			/* process keys from this scan */
			if(numActiveSensors == 1)	/* only one key ... CapSense key # is reported to console via UART */
			{
				if(activeKey != previousKey)	/* new event if true */
				{
					multipleKeyFlag = FALSE;
					previousKey = activeKey;	/* remember new key */
					
					#ifdef CY_SCB_UART_H
						sprintf(printBuf, "Key pressed ........ SW%d\r\n", (activeKey + 100));
						UART_UartPutString(printBuf);
					#endif
					
					/* Add key press actions here. */					
					switch(activeKey)
					{
						case KEY00: // SW100
							Start_Button_Write(HI);
							break;
						case KEY01: // SW101
							
							break;
						
						default:
							break;
					}
				}
			}
			else if(numActiveSensors > 1)	/* multiple keys are active */
			{
				if(FALSE == multipleKeyFlag)
				{
					#ifdef CY_SCB_UART_H
						UART_UartPutString("Multiple sensors detected\r\n");
					#endif
					multipleKeyFlag = TRUE;
					previousKey = NO_KEY_PRESSED;
				}
			}			
			else
			{
				/* nothing for now */
			}
				
		}
		else /* no keys active */
		{
			if(previousKey != NO_KEY_PRESSED)	/* if true, this is a "key released" event */
			{
				HB_LED_Write(LED_OFF);
				Start_Button_Write(LOW);
				
				previousKey = NO_KEY_PRESSED; /* update previousKey to show that no keys are presently active */
				
				#ifdef CY_SCB_UART_H
					/* issue UART message here */
					UART_UartPutString("Key(s) released\r\n");
				#endif
			}
		}
    }
}

/* [] END OF FILE */
