/********************************
* File Name: PCRTask.c
* Date: 2015.07.16
* Author: Jun Yeon
********************************/

#include "HardwareProfile.h"
#include "./CONFIG/Compiler.h"
#include "./PCR/PCRTask.h"
#include "./DEFINE/UserDefs.h"
#include "./DEFINE/GlobalTypeVars.h"
#include "./PCR/Temp_Table.h"
#include "./USB/usb.h"
#include <math.h>

// in UsbTask.c
extern unsigned char ReceivedDataBuffer[RX_BUFSIZE];
extern unsigned char ToSendDataBuffer[TX_BUFSIZE];

BYTE prevState = STATE_READY;
BYTE currentState = STATE_READY;

BYTE chamber_h = 0x00;
BYTE chamber_l = 0x00;

BYTE photodiode_h = 0x00;
BYTE photodiode_l = 0x00;

BYTE currentError = 0x00;
BYTE request_data = 0x00;

// For calculating the temperature.
float currentTemp = 0x00;
float temp_buffer[5], temp_buffer2[5];

// For pid controls
BYTE prevTargetTemp = 25;
BYTE currentTargetTemp = 25;
double lastIntegral = 0;
double lastError = 0;

float kp = 0, ki = 0, kd = 0;
float integralMax = 2600.0;

BYTE fanFlag = 0;

/**********************************
* Function : void PCR_Task(void)
* This function is overall routine for microPCR
**********************************/
void PCR_Task(void)
{
	// Check the cmd buffer, performing the specific task.
	Command_Setting();

	// Sensing the adc values(for photodiode, chamber, heatsink)
	Sensor_Task();
	
	if( rxBuffer.ledControl ){
		//LED_WG = rxBuffer.led_wg;
		//LED_R = rxBuffer.led_r;
		//LED_G = rxBuffer.led_g;
		LED_B = rxBuffer.led_b;
	}

	// Setting the tx buffer by structed buffer.
	TxBuffer_Setting();
}

/**********************************
* Function : WORD ReadTemperature(BYTE sensor)
* This function have a reading adc values.
* The parameter sensor's type shows in below.
	* ADC_PHOTODIODE(0x01) : photodiode adc value
	* ADC_CHAMBER(0x02)    : chamber temperature adc value
	* ADC_HEATSINK(0x03)   : heatsink temperature adc value
* The returned value is sampled in SAMPLING_COUNT times.
**********************************/
WORD ReadTemperature(BYTE sensor)
{
	WORD w;
	BYTE low=0x00;
	BYTE high=0x00;
	BYTE counter = SAMPLING_COUNT;   //multiple adc sampling
	WORD sum=0;

	// Select the ADC Channel by parameter.
	// The ADC Channel information shows in HardwareProfile -PICDEM FSUSB.h file.
	switch(sensor)
	{
		case ADC_PHOTODIODE:
			SetADCChannel(Sensor_Photodiode);
			break;
		case ADC_CHAMBER:			
			SetADCChannel(Sensor_Chamber);
			break;
		case ADC_HEATSINK:			
			SetADCChannel(Sensor_Heatsink);
			break;
	}

	while(counter--)
	{
		while(ADCON0bits.NOT_DONE);     // Wait for busy
    	ADCON0bits.GO = 1;              // Start AD conversion
    	while(ADCON0bits.NOT_DONE);     // Wait for conversion

		low = ADRESL;
		high = ADRESH;
    	w = (WORD)high*256 + (WORD)low;
		sum += w;
	}    

	w = sum/SAMPLING_COUNT;

    return w;
}

/**********************************
* Function : WORD ReadPhotodiode(void)
* This function is overriding for reading photodiode.
* The developer use this function very easier.
**********************************/
WORD ReadPhotodiode(void)
{
	return ReadTemperature(ADC_PHOTODIODE);
}

/**********************************
* Function : double quickSort(float *d, int n)
* Implementation quicksort for double type
* The parameter 'd' is array of double types
* And 'n' parameter is count of array 'd'.
**********************************/
double quickSort(float *d, int n)
{
	int left, right;
	double pivot;
	double temp;

	if( n > 1 )
	{
		pivot = d[n-1];
		left = -1;
		right = n-1;

		while(TRUE)
		{
			while( d[++left] < pivot );
			while( d[--right] > pivot );

			if( left >= right ) break;

			temp = d[left];
			d[left] = d[right];
		}

		temp = d[left];
		d[left] = d[n-1];
		d[n-1] = temp;
		quickSort(d, left);
		quickSort(d + left + 1, n - left - 1);
	}
	return d[2];
}

/**********************************
* Function : void Sensor_Task(void)
* reading some essential sensor data functions
* and save the data to variables.
* essential sensor : photodiode, chamber(also temperature)
* chamber_h, chamber_l are adc value of chamber.
* photodiode_h, photodiode_l are adc valoe of photodiode.
* currentTemp is chip's temperature by calculated from chamber value.
**********************************/
extern struct{BYTE ntc_chamber[NTC_CHAMBER_SIZE];}NTC_CHAMBER_TABLE;

ROM BYTE *pTemp_Chamber = (ROM BYTE *)&NTC_CHAMBER_TABLE;

void Sensor_Task(void)
{
	double r, InRs, tmp, adc;
	WORD chamber = ReadTemperature(ADC_CHAMBER);
	WORD photodiode = ReadPhotodiode();
	WORD index = 0;

	// save the adc value by high low type
	chamber_h = (BYTE)(chamber>>8);
	chamber_l = (BYTE)(chamber);
	photodiode_h = (BYTE)(photodiode>>8);
	photodiode_l = (BYTE)(photodiode);

	// temperature calculation
	index = (WORD)((chamber/4) * 2.);
	currentTemp = (float)(pTemp_Chamber[index]) + (float)(pTemp_Chamber[index+1] * 0.1);
	
	// for median filtering
	temp_buffer[0] = temp_buffer[1];
	temp_buffer[1] = temp_buffer[2];
	temp_buffer[2] = temp_buffer[3];
	temp_buffer[3] = temp_buffer[4];
	temp_buffer[4] = currentTemp;

	memcpy(temp_buffer2, temp_buffer, 5*sizeof(float));

	currentTemp = (float)quickSort(temp_buffer2, 5);
}

/**********************************
* Function : void Command_Setting(void)
* The command list was listed in below.
	* CMD_READY = 0x00,
	* CMD_PCR_RUN,
	* CMD_PCR_STOP,
	* CMD_REQUEST_LINE,
	* CMD_BOOTLOADER = 0x55
 - The 'CMD_READY' command is common operation.
 - The 'CMD_PCR_RUN' command is used to run the PCR, but, the pid value must exist.
 - The 'CMD_PCR_STOP' command is used to stop the PCR.
 - The 'CMD_REQUEST_LINE' command is not used.
 - The 'CMD_BOOTLOADER' command is not working that maybe the board is different.

All of command is checking the pc command flow for assert.
**********************************/
void Command_Setting(void)
{
	int i = 0;

	switch( rxBuffer.cmd )
	{
		case CMD_READY:
			if( currentState == STATE_RUNNING )
				Run_Task();
			break;
		case CMD_PCR_RUN:
			if( currentState == STATE_READY )
			{
				Init_PWM_MODE();
				currentState = STATE_RUNNING;
				lastIntegral = 0;
				lastError = 0;
				prevTargetTemp = currentTargetTemp = 25;

				Run_Task();
			}
			else if( currentState == STATE_RUNNING )
			{
				Run_Task();
			}
			else
				currentError = ERROR_ASSERT;
			break;
		case CMD_PCR_STOP:
			if( currentState == STATE_RUNNING )
			{
				currentState = STATE_READY;
				Stop_Task();
			}
			else
				currentError = ERROR_ASSERT;
			break;
		case CMD_FAN_ON:
			if( currentState == STATE_READY )
			{
				currentState = STATE_PAN_RUNNING;
				Fan_ON();
			}
			else if( currentState != STATE_PAN_RUNNING )
				currentError = ERROR_ASSERT;
			break;
		case CMD_FAN_OFF:
			if( currentState == STATE_PAN_RUNNING )
			{
				currentState = STATE_READY;
				Fan_OFF();
			}
			else
			{
				currentError = ERROR_ASSERT;
			}
			break;
	}
}

void Run_Task(void)
{
	if( rxBuffer.cmd == CMD_PCR_RUN && 
		currentTargetTemp != rxBuffer.currentTargetTemp )
	{
		prevTargetTemp = currentTargetTemp;
		currentTargetTemp = rxBuffer.currentTargetTemp;

		if ( !(fabs(prevTargetTemp - currentTargetTemp) < .5) ) 
		{
			lastIntegral = 0;
			lastError = 0;
		}
	}
	
	//in Run_Task, 150901 kimjd
	if(prevTargetTemp>currentTargetTemp ) 
	{
		if( currentTemp>prevTargetTemp-1 ) 
		{
			Fan_ON();
			fanFlag = 1;
		}

		if( (fanFlag == 1) && (currentTemp-currentTargetTemp<= FAN_STOP_TEMPDIF) ) 
		{
			Fan_OFF();
			fanFlag = 2;
		}
		
		if( (fanFlag == 2) && (currentTemp-currentTargetTemp<= FAN_STOP_ARRIVAL) )
		{
			fanFlag = 0;
			T2S_Flag = TRUE;
		}
	}
	else
	{
		targetArrival = 0;
		fanFlag = 0;
		Fan_OFF();
	}


	PID_Control();
}

void PID_Control(void)
{
	double currentErr = 0, proportional = 0, integral = 0;
	double derivative = 0;
	int pwmValue = 0xffff;

	// read pid values from buffer
	if( rxBuffer.cmd == CMD_PCR_RUN )
	{
		memcpy(&kp, &(rxBuffer.pid_p1), 4);
		memcpy(&ki, &(rxBuffer.pid_i1), 4);
		memcpy(&kd, &(rxBuffer.pid_d1), 4);
		memcpy(&integralMax, &(rxBuffer.integralMax_1), 4);
	}

	currentErr = currentTargetTemp - currentTemp;
	proportional = currentErr;
	integral = currentErr + lastIntegral;

	if( integral > integralMax )
		integral = integralMax;
	else if( integral < -integralMax )
		integral = -integralMax;

	derivative = currentErr - lastError;
	pwmValue = 	kp * proportional + 
				ki * integral +
				kd * derivative;

	if( pwmValue > 1023 )
		pwmValue = 1023;
	else if( pwmValue < 0 )
		pwmValue = 0;

	lastError = currentErr;
	lastIntegral = integral;

	//PID Control, 150901 kimjd
	if(	prevTargetTemp > currentTargetTemp )
	{
		if ( fanFlag == 2 || T2S_Flag ) 
		{
			pwmValue = 0x0;
			lastIntegral = 0;
			lastError = 0;
     	}
	} 

	CCPR1L = (BYTE)(pwmValue>>2);
	CCP1CON = ((CCP1CON&0xCF) | (BYTE)((pwmValue&0x03)<<4));
}

void Stop_Task(void)
{
	Stop_PWM_MODE();
	Fan_OFF();
}

void TxBuffer_Setting(void)
{
	BYTE *tempBuf;

	txBuffer.state = currentState;

	txBuffer.chamber_h = chamber_h;
	txBuffer.chamber_l = chamber_l;

	// Convert float type to BYTE pointer
	tempBuf = (BYTE*)&(currentTemp);
	memcpy(&(txBuffer.chamber_temp_1), tempBuf, sizeof(float));

	txBuffer.photodiode_h = photodiode_h;
	txBuffer.photodiode_l = photodiode_l;

	// Checking the PC Software error & firmware error.
	txBuffer.currentError = currentError;

	// For request
	txBuffer.request_data = request_data;

	// For setting the arrival
	// 150904 YJ
	txBuffer.targetArrival = targetArrival;

	// Copy the txBuffer struct to rawBuffer
	// and clear previous buffer
	USBMaskInterrupts();
	memset(&rxBuffer, 0, sizeof(RxBuffer));
	memcpy(&ToSendDataBuffer, &txBuffer, sizeof(TxBuffer));
	USBUnmaskInterrupts();
}