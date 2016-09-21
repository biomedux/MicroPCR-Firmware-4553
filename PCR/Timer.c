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
 * Function:        void TIMR0_init(void)
 *
 * OverView:		This routine will use low interrupt service routine configuration.
 *
 * Note:			None
 ***************************************************/
void TIMR0_init(void)
{
	//Timer0
	T0CONbits.TMR0ON = 0;	// stop timer

	T0CONbits.T08BIT = 1;	// 1:8bit , 0:16bit
	T0CONbits.T0CS = 0;		// Internal instruction cycle clock (CLKO)
	T0CONbits.PSA=0;		// Timer0 prescaler is assigned. Timer0 clock input comes from prescaler output.
	
//	T0CON = 0xC5;
	T0CON = 0xC4;

	//8bit setting
	TMR0L = 0x06;
	tmr0l_temp = 0x06;

	// Enable interrupts 
	INTCON2bits.TMR0IP = 0;	//Low priority
	INTCONbits.GIEH=1;		//Enables all unmasked interrupts
	INTCONbits.GIEL=1;		//Enables all unmasked peripheral interrupts
	INTCONbits.TMR0IE=1;	//Enables the TMR0 overflow interrupt

	T0CONbits.TMR0ON = 1;	//start timer
}

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
 * Function:        void timer0_isr(void)
 *
 * OverView:		If low interrupt actived, called this function.
 *					Control LED
 *
 * Note:			reference BootLoader.h
 ***************************************************/
void timer0_isr(void)
{
	if(INTCONbits.TMR0IF)
	{
		INTCONbits.TMR0IF = 0;
		TMR0L = tmr0l_temp;

		if(!wg_ledCtrl)
		{
			if(wg_ledPwmDuty <= 0)
			{
				TMR0L = wg_ledPwmDuty;
				LED_WG_OFF();
			}

			else if(TMR0L == wg_ledPwmDuty)
			{
				TMR0L = 255 - wg_ledPwmDuty;
				LED_WG_ON();
			}
			else
			{
				TMR0L = wg_ledPwmDuty;
				LED_WG_OFF();
			}
		}
		else if(!r_ledCtrl)
		{
			if(r_ledPwmDuty <= 0)
			{
				TMR0L = r_ledPwmDuty;
				LED_R_OFF();
			}

			else if(TMR0L == r_ledPwmDuty)
			{
				TMR0L = 255 - r_ledPwmDuty;
				LED_R_ON();
			}
			else
			{
				TMR0L = r_ledPwmDuty;
				LED_R_OFF();
			}
		}
		else if(!g_ledCtrl)
		{
			if(g_ledPwmDuty <= 0)
			{
				TMR0L = g_ledPwmDuty;
				LED_G_OFF();
			}

			else if(TMR0L == g_ledPwmDuty)
			{
				TMR0L = 255 - g_ledPwmDuty;
				LED_G_ON();
			}
			else
			{
				TMR0L = g_ledPwmDuty;
				LED_G_OFF();
			}
		}
		else if(!b_ledCtrl)
		{
			if(b_ledPwmDuty <= 0)
			{
				TMR0L = b_ledPwmDuty;
				LED_B_OFF();
			}

			else if(TMR0L == b_ledPwmDuty)
			{
				TMR0L = 255 - b_ledPwmDuty;
				LED_B_ON();
			}
			else
			{
				TMR0L = b_ledPwmDuty;
				LED_B_OFF();
			}
		}
		else
		{
			TMR0L = tmr0l_temp;
			LED_WG_OFF();
			LED_R_OFF();
			LED_G_OFF();
			LED_B_OFF();
		}
		tmr0l_temp = TMR0L;
	}
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
		if( currentState == STATE_RUNNING ){
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
		else{
			Set_PWM_plus_In();
			Set_PWM_minus_In();
			powerCounter = 0;
		}
	}
}

