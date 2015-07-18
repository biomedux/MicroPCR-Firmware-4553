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
#include <math.h>

// in UsbTask.c
extern unsigned char rxRawBuffer[RX_BUFSIZE];
extern unsigned char txRawBuffer[TX_BUFSIZE];

PID pids[MAX_PID_COUNT];

BYTE prevState = STATE_READY;
BYTE currentState = STATE_READY;

BYTE pidIndex = 0x00;
BYTE totalPidCount = 0x00; 

BYTE chamber_h = 0x00;
BYTE chamber_l = 0x00;

BYTE photodiode_h = 0x00;
BYTE photodiode_l = 0x00;

BYTE currentError = 0x00;
BYTE request_data = 0x00;

// For calculating the temperature.
float currentTemp = 0x00;
double temp_buffer[5], temp_buffer2[5];

// For pid controls
BYTE prevTargetTemp = 25;
BYTE currentTargetTemp = 25;
double lastIntegral = 0;
double lastError = 0;
double kp = 0x00, ki = 0x00, kd = 0x00;

/**********************************
* Function : void PCR_Task(void)
* This function is overall routine for microPCR
**********************************/
void PCR_Task(void)
{
	// copy the raw buffer to structed buffer.
	// and clear previous raw buffer(important)
	memcpy(&rxBuffer, rxRawBuffer, RX_BUFSIZE);
	memset(rxRawBuffer, 0, RX_BUFSIZE);

	// Check the cmd buffer, performing the specific task.
	Command_Setting();

	// Sensing the adc values(for photodiode, chamber, heatsink)
	Sensor_Task();

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
void Sensor_Task(void)
{
	double r, InRs, tmp, adc;
	WORD chamber = ReadTemperature(ADC_CHAMBER);
	WORD photodiode = ReadPhotodiode();

	// save the adc value by high low type
	chamber_h = (BYTE)(chamber>>8);
	chamber_l = (BYTE)(chamber);
	photodiode_h = (BYTE)(photodiode>>8);
	photodiode_l = (BYTE)(photodiode);

	// temperature calculation
	adc = ((double)(chamber_h & 0x0f)*255. + (double)(chamber_l))/4.0;
	currentTemp = 0;

	// the Rref, A_VAL, B_VAL, C_VAL, K are defined in UserDefs.h file
	if( adc != 0 )
	{
		r = Rref * (1024.0 / adc - 1.0);
		InRs = log(r);
		tmp = A_VAL + B_VAL *InRs + C_VAL * pow(InRs,3.);

		currentTemp = 1./tmp-K;
	}

	// for median filtering
	temp_buffer[0] = temp_buffer[1];
	temp_buffer[1] = temp_buffer[2];
	temp_buffer[2] = temp_buffer[3];
	temp_buffer[3] = temp_buffer[4];
	temp_buffer[4] = currentTemp;

	memcpy(temp_buffer2, temp_buffer, 5*sizeof(double));
	
	currentTemp = (float)quickSort(temp_buffer2, 5);
}

/**********************************
* Function : void Command_Setting(void)
* The command list was listed in below.
	* CMD_READY = 0x00,
	* CMD_PID_WRITE,
	* CMD_PID_END,
	* CMD_PCR_RUN,
	* CMD_PCR_STOP,
	* CMD_REQUEST_LINE,
	* CMD_BOOTLOADER = 0x55
 - The 'CMD_READY' command is common operation.
 - The 'CMD_PID_WRITE' command is used to store PID values.
 - The 'CMD_PID_END' command is used to change the state of pid write mode.
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
			break;
		// PID write mode enable in STATE_READY.
		case CMD_PID_WRITE:
			// if the first of this state, reset all of the variables.
			if( currentState == STATE_READY )
			{
				// Change the state
				currentState = STATE_PID_READING;

				for(i=0; i<MAX_PID_COUNT; ++i)
					memset(&pids[i], 0, sizeof(PID));

				pidIndex = 0;
				totalPidCount = 0;
				PIDRead_Task();
			}
			// already STATE_PID_READING mode, go PIDRead_Task()
			else if( currentState == STATE_PID_READING )	
				PIDRead_Task();
			else
				currentError = ERROR_ASSERT;
			break;
		case CMD_PID_END:
			if( currentState == STATE_PID_READING )
			{
				currentState = STATE_READY;
				totalPidCount = pidIndex;
			}
			else
				currentError = ERROR_ASSERT;
			break;
		case CMD_PCR_RUN:
			if( currentState == STATE_READY && 
				totalPidCount != 0 )
			{
				Init_PWM_MODE();
				currentState = STATE_RUNNING;
				kp = 0x00;	ki = 0x00;	kd = 0x00;
				lastIntegral = 0;
				lastError = 0;
				prevTargetTemp = currentTargetTemp = 25;

				Run_Task();
			}
			else if( currentState == STATE_RUNNING &&
					 totalPidCount != 0 )
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
			else if( currentState == STATE_PID_READING )
				currentError = ERROR_ASSERT;
			break;
	}
}

void PIDRead_Task(void)
{
	// assume the pc is not sending the pid that over the maximum count of pids.
	// so, we need not exception this problem.
	pids[pidIndex].startTemp = rxBuffer.startTemp;
	pids[pidIndex].targetTemp = rxBuffer.targetTemp;
	
	memcpy(&pids[pidIndex].p, &rxBuffer.pid_p1, sizeof(float));
	memcpy(&pids[pidIndex].i, &rxBuffer.pid_i1, sizeof(float));
	memcpy(&pids[pidIndex].d, &rxBuffer.pid_d1, sizeof(float));

	pidIndex++;
}

void Run_Task(void)
{
	double currentErr = 0, proportional = 0, integral = 0;
	double derivative = 0;
	WORD pwmValue = 0xffff;
	int paramIdx = 0;

	if( currentTargetTemp != rxBuffer.currentTargetTemp )
	{
		prevTargetTemp = currentTargetTemp;
		currentTargetTemp = rxBuffer.currentTargetTemp;

		if( !fabs(prevTargetTemp - currentTargetTemp) < .5 )
		{
			if( currentTargetTemp > 94.5 )
				paramIdx = 0;
			else if( currentTargetTemp > 71.5 )
				paramIdx = 2;
			else if( currentTargetTemp > 59.5 )
				paramIdx = 3;
	
			kp = pids[paramIdx].p;
			ki = pids[paramIdx].i;
			kd = pids[paramIdx].d;
	
			lastIntegral = 0;
			lastError = 0;
		}
	}
	
	if( prevTargetTemp > currentTargetTemp )
	{
		if( currentTemp-currentTargetTemp <= FAN_STOP_TEMPDIF )
		{
			Fan_OFF()
		}
		else
		{
			Fan_ON();
		}
	}
	else
	{
		Fan_OFF();
	}
	
	currentErr = currentTargetTemp - currentTemp;
	proportional = currentErr;
	integral = currentErr + lastIntegral;

	if( integral > INTGRALMAX )
		integral = INTGRALMAX;
	else if( integral < 0 )
		integral = 0;

	derivative = currentErr - lastError;
	pwmValue = 	kp * proportional + 
				ki * integral +
				kd * derivative;

	if( pwmValue > 255 )
		pwmValue = 255;
	else if( pwmValue < 0 )
		pwmValue = 0;

	pwmValue = fabs(pwmValue);

	if( pwmValue == 255 )
		pwmValue = 1023;
	else
	{
		pwmValue = pwmValue / 100 * 1023.;	
		if( pwmValue >= 1023 )
			pwmValue = 1023;
		else if( pwmValue <= 0 )
			pwmValue = 0;
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

	// Copy the txBuffer struct to rawBuffer
	memcpy(&txRawBuffer, &txBuffer, sizeof(TxBuffer));
}