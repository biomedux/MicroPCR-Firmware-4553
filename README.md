# MicroPCR_Firmware
# Photodiode 와 temperature sensor 간섭 문제 #

**2016-02-02 김종대**

**Experimenter: 이승철, 이득주**

1. 간섭문제의 원인으로 예상되는 것은 Photodiode input이 Vref=3.3V를 넘게 input으로 들어가서 이것으로 보임.

	A. Analog input에 5V를 직접 연결하면 5.6도 올라간다

	B. 3.3V 를 연결하면 온도의 변화가 없다

	C. Temperature ADC 시작전에 한번 ADC를 하면 5V 입력일 때보다 0.7도 낮아진다.


2. 결론: Photodiode가 4096를 넘기 전까지는 문제가 없을 것으로 생각되며, 차후 3.3V를 넘지 못하게 하는 회로가 필요할 것으로 생각된다.

3. 추가로 발견된 SW flaw

	**A.	ADC conversion clock setting 오류**

		HardwareProfile - PICDEM FSUSB.h를 보면 아래 2line으로 ADC conversion clock을 만드는 데 파란색 부분이 111이 되어야 한다.
		이와 같은 오류로 해서 Fosc/64로 했으나 사실은 Fosc/32가 된다.
		즉, Tad=1/(48M/32)=0.67us (Table 28-29: A/D conversion requirement에 어긋남)

		#define SetADCConvClock(CLOCK)	{ADCON2= (ADCON2&0b11111000) | (CLOCK & 0b00000011);}
		#define ADC_ADCS_FREQD64		(0b00000110)

		한편 sampling time 인 Tacq는 아래에 의하면 6*Tad 이므로 4us 이다
		
		#define SetADCAcqTime(TIME)	{ADCON2= (ADCON2&0b11000111) | (TIME&0b00111000);}
		#define ADC_ACQT_06TAD			(0b00011000)

	**B.	수정**

		#define SetADCConvClock(CLOCK)	{ADCON2= (ADCON2&0b11111000) | (CLOCK & 0b00000111);}

		Tad=1/(48M/64)=1.33 us
		Tacq=8us > 2.35us (channel change후 requirement)
		Tc = 6Tad+12Tad=18Tad=24us

		10번 ADC하는 데 걸리는 시간 240us

	**C.	ADC 시간 실험 (README.docx 파일 참고)**

	![ADC는 두번 실행되고 약 260us정도 됨](http://imgur.com/XbCMSUg)
		
	ADC는 두번 실행되고 약 260us정도 됨
	
	**D.	과거 실험 (README.docx 파일 참고)**

		Tad=1/(48M/32)=0.67us 이고, ADC_ACQT_20TAD 일 때 10번 ADC loop 결과
		Tc=32Tad=21.4us

		10번 ADC 하는 데 걸리는 시간 214us
	![](http://imgur.com/DwTRvIa)

	**E.	Main loop period가 3ms인 문제**

		이번 측정에서 전체 주기가 3ms인 것은 아래 code문제임.
		Counter가 ‘0’이 된 후에 1ms후에 1이 되고 다시 increment한다.
		다시 1ms 후에 2가 되고 1ms후에 if condition으로 check되어 0으로 reset됨.

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
		
		2ms 마다 reset하려면 아래와 같이 수정해야 함.
		
		T2MS_Counter++;
		if( T2MS_Counter >= 2 )
		{
			T2MS_Flag = TRUE;
			T2MS_Counter = 0;	
			T30MS_Counter++;
		}
