/****************************************************
*	FileName 	:	Timer.c
*	Date	 	:	2012.07.27
*	Author		:	Yeon June
****************************************************/

/** Includes ***************************************/
#include "HardwareProfile.h"
#include "./CONFIG/Compiler.h"
#include "./DEFINE/GenericTypeDefs.h"
#include "./DEFINE/GlobalTypeVars.h"
#include "./DEFINE/UserDefs.h"
#include "./PCR/Timer.h"


/***************************************************
 * Function:        void TIMR1_init(void)
 *
 * OverView:		This routine will use high interrupt service routine configuration.
 *
 * Note:			None
 ***************************************************/
void TIMR1_init(void)
{
	T1CONbits.TMR1ON = 0; //stop timer
	T1CONbits.RD16 = 1; //0:two 8bit , 1:one 16bit		
	T1CONbits.TMR1CS = 0; //Internal clock	(Fosc/4) = 12Mhz
	T1CONbits.T1OSCEN=0;  //timer 1 oscillator shut off
	//Timer1 Prescaler Select bits 	
	T1CONbits.T1CKPS1=1;	//12 / 8 => 1/1.5Mhz * 1500 = 1ms at every timer 1 interrupt
	T1CONbits.T1CKPS0=1;	

	//16bit setting	  65536-1500 = 64035 = 0xFA23
	TMR1H = 0xFA; //set duration high byte first
	TMR1L = 0x23;  //set duration low byte

	// Enable interrupts 
	//INTCON2bits.TMR1IP = 1;// timer 1 interrupt Low priority
	INTCONbits.GIEH=1;//Enables all unmasked interrupts
	INTCONbits.GIEL=1;//Enables all unmasked peripheral interrupts		
	PIE1bits.TMR1IE =1;	

	T1CONbits.TMR1ON = 1;  	//start timer	
}

/***************************************************
 * Function:        timer1_isr(void)
 *
 * OverView:		If high interrupt actived, called this function.
 *
 * Note:			reference BootLoader.h
 ***************************************************/
void timer1_isr(void)
{
	if(PIR1bits.TMR1IF)
	{
		PIR1bits.TMR1IF=0;	// set off timer1 flag.

		TMR1H = 0xFA; 	//	set duration high first
		TMR1L = 0x23;  	//	set duration low

		if( T2MS_Counter >= 2 )
		{
			T2MS_Flag = TRUE;
			T2MS_Counter = 0;	
			T30MS_Counter++;
		}
		else
		{
			T2MS_Counter++;
		}

		if( T30MS_Counter >= 15 )
		{
			T30MS_Flag = TRUE;
			T30MS_Counter = 0;
		}

		// 150904 YJ
		// counting 2sec
		if( freeRunning )
		{
			freeRunningCounter++;
			if( freeRunningCounter >= 2000 )
			{
				freeRunning = FALSE;
				freeRunningCounter = 0;
				fallingTargetArrival = 1;
			}	
		}

		// 150930 YJ For PWM Power Test
		powerCounter++;
		if( powerCounter <= PWM_POWER_DUTY ){
			Set_PWM_plus_Out();
			Set_PWM_minus_Out();
		}
		else{
			Set_PWM_plus_In();
			Set_PWM_minus_In();
		}

		if( powerCounter >= PWM_POWER_PERIOD )
			powerCounter = 0;
	}
}

