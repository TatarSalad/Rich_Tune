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
#include "main.h"
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

#define HIGH_SIDE_MASK 0x003f

/* Macros to simplify operations on a single/specific LED. Note - use schematic reference designator (i.e. Z201). */
#define MATRIX_LED_ON(a) LED_Matrix[ledMatrixScanAndPins[a].scan] |= 1 << ledMatrixScanAndPins[a].pin;
#define MATRIX_LED_OFF(a) LED_Matrix[ledMatrixScanAndPins[a].scan] &= ~(1 << ledMatrixScanAndPins[a].pin);
#define MATRIX_LED_TOGGLE(a) LED_Matrix[ledMatrixScanAndPins[a].scan] ^= 1 << ledMatrixScanAndPins[a].pin;

const uint16 ScanMask[NumberOfMatrixScans] = {Scan0Mask, Scan1Mask, Scan2Mask, Scan3Mask, Scan4Mask, Scan5Mask};

uint16 LED_Matrix[NumberOfMatrixScans] = {0, 0, 0, 0, 0, 0};

enum
{
	SW100,
	SW101,
	SW102,
	SW103,
	SW104,
	SW105,
	SW106,
	SW107,
	SW108,
	SW109,
	SW110,
	SW111,
	SW112
};

/* LED matrix ISR - Timer interrupt set to 600 KHz/500 = 1.2 kHz (6 columns = each LED updated @ 200 Hz) */
CY_ISR(LED_MatrixISR)
{
	uint16 SPI_data;
	static uint8 active_column;
	
	if(active_column >= NumberOfMatrixScans)
		active_column = 0;
	
	SPI_OE_Write(HI);	/* blank display to prevent ghosting while updating next column */
	
	SPI_data = ((LED_Matrix[active_column] & ~HIGH_SIDE_MASK) | ScanMask[active_column]);
	
	/* update SPI LED driver ... display will be released by SPI_Done ISR */
	SPI_SpiUartWriteTxData(SPI_data); /* send bit pattern to SPI LED driver */

	active_column++;

	MatrixTimer_ClearInterrupt(MatrixTimer_INTR_MASK_TC);
	isr_MatrixTimer_ClearPending();
}

CY_ISR(TimeOutISR)
{
	PWM_Freq_Stop(); /* kill the buzzer */
	PWM_Volume_Stop();
	TimeOut_ClearInterrupt(TimeOut_INTR_MASK_TC);
	isr_TimeOut_ClearPending();
}

CY_ISR(SPI_DoneISR)
{
	SPI_OE_Write(LOW);	/* SPI transaction complete ... release display, no ghosting */
	SPI_ClearMasterInterruptSource(SPI_INTR_MASTER_SPI_DONE);
	isr_SPI_Done_ClearPending();
}

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
	
	isr_TimeOut_StartEx(TimeOutISR);
	isr_MatrixTimer_StartEx(LED_MatrixISR);
	
	SPI_Start();
	isr_SPI_Done_StartEx(SPI_DoneISR);
	MatrixTimer_Start();
	
	PWM_Freq_Start(); /* start the buzzer */
	PWM_Volume_Start();
	CyDelay(350);
	PWM_Freq_Stop(); /* kill the buzzer */
	PWM_Volume_Stop();
	
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
	
	//UART_Start();
	
	//UART_UartPutString("\nUART started\r\n");
	
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
		
		//UART_UartPutString("\r\n\n");
				
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
			for(i = CapSense_KEY00_WDGT_ID; i < CapSense_TOTAL_CSD_WIDGETS; i++)
			{
				if(CapSense_IsWidgetActive(i))
				{
					numActiveSensors++;	/* will be used to test whether one key or multiple keys are active */
					activeKey = i - CapSense_KEY00_WDGT_ID;	/* stores the active key from this scan */
				}
			}
			
			/* process keys from this scan */
			if(numActiveSensors == 1)	/* only one key ... CapSense key # is reported to console via UART */
			{
				if(activeKey != previousKey)	/* new event if true */
				{
					multipleKeyFlag = FALSE;
					previousKey = activeKey;	/* remember new key */
					
					PWM_Freq_Start(); /* start the buzzer */
					PWM_Volume_Start();
					TimeOut_Start(); /* start oneshot timer ... will kill the buzzer in 150 msec unless key is released sooner */
					
					sprintf(printBuf, "Key pressed ........ SW%d\r\n", (activeKey + 100));
					//UART_UartPutString(printBuf);
					
					/* Add key press actions here. */					
					switch(activeKey)
					{
						case SW100: // SW100
							MATRIX_LED_TOGGLE(Z201)
							break;
						case SW101: // SW101
							MATRIX_LED_TOGGLE(Z202)
							break;
						case SW102: // SW102
							MATRIX_LED_TOGGLE(Z210)
							break;
						case SW103: // SW103
							MATRIX_LED_TOGGLE(Z209)
							break;
						case SW104: // SW104
							MATRIX_LED_TOGGLE(Z212)
							break;
						case SW105: // SW105
							MATRIX_LED_TOGGLE(Z211)
							break;
						case SW106: // SW106
							MATRIX_LED_TOGGLE(Z203)
							break;
						case SW107: // SW107
							MATRIX_LED_TOGGLE(Z204)
							break;
						case SW108: // SW108
							MATRIX_LED_TOGGLE(Z205)
							break;
						case SW109: // SW109
							MATRIX_LED_TOGGLE(Z206)
							break;
						case SW110: // SW110
							MATRIX_LED_TOGGLE(Z207)
							break;
						case SW111: // SW111
							MATRIX_LED_TOGGLE(Z208)
							break;
						case SW112: // SW112
							MATRIX_LED_TOGGLE(Z213)
							MATRIX_LED_TOGGLE(Z214)
							MATRIX_LED_TOGGLE(Z216)
							MATRIX_LED_TOGGLE(Z217)
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
//					UART_UartPutString("Multiple sensors detected\r\n");
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
				
				PWM_Freq_Stop();
				PWM_Volume_Stop();	/* kill the buzzer */
				
				previousKey = NO_KEY_PRESSED; /* update previousKey to show that no keys are presently active */
									
				/* issue UART message here */
//				UART_UartPutString("Key(s) released\r\n");

			}
		}
		
    }
}

/* [] END OF FILE */
