# MicroPCR_Firmware

# Photodiode and temperature sensor interference 문제 #

2016-01-29 김 종 대

### 1.	현상 ###
	Photodiode를 막거나 열었을 때 0~5V까지 변하고 이로 인하여 측정 온도 값이 변한다.
	5V일 때와 0V 일 때 약 상온에서 약 7도 정도 차이가 난다.

### 2.	문제 분석 ###
	HW를 검토한 결과 PIC에서 interference가 일어나는 것으로 확인되어 ADC 동작을 분석하기 시작함.


 
**\#define InitADC()**

	{
		Set_ADC_INPUT();\
		SetVREF();\
		SetADCSampleFormat(RIGHT_ADJUST);\
		SetADCChannel(ADC_CHAMBER);\
		SetADCAcqTime(ADC_ACQT_06TAD);\
		SetADCConvClock(ADC_ADCS_FREQD64);\
		ADC_ADON();
	}

Conversion time: tc=6TAD+12TAD=18TAD

Sampling clock: TAD = FREQD64=48M/64=0.75M=1.3us

그러므로 **conversion time = 24us** 이다.

**21.1 A/D Acquisition Requirements**

	… After the analog input channel is selected (changed), the channel must be sampled for at least the minimum acquisition time before starting a conversion …

	TACQ = Amplifier Settling Time + Holding Capacitor Charging Time + Temperature Coefficient 
		 = TAMP + TC + TCOFF = 2.45 us

	Instruction execution: Tcyc=12Mhz

PCRtask()는 T2MS_Flag가 True일 때 동작하므로 2ms 마다 동작할 것이다. (init.c main_looper)
PCR_Task() 안에서는 Sensor_Task()가 매번 불리워지므로 2ms 마다 불리울 것이다.
Sensor_Task)()에서는 ReadTemperature(ADC_CHAMBER)를 부르고 바로 ReadPhotodiode()를 부른다.
 ReadPhotodiode()에서는 ReadTemperatgure(ADC_PHOTODIODE)를 부르므로 다음과 같이 계속해서 수행될 것이다.

Do every 2m:

	Sensor_chamber설정
	ADC 10번 // 여기까지 약 250us
	Sensor_photodiode설정
	ADC 10번 // 여기까지 약 500us

여기서 문제는 sensor_chamber를 설정하자마자 ADC에 들어가는 것이 문제이다.

	첫번째 ADC를 하는 것은 Photodiode전압일 것이다.
	그러므로 chamber 온도 전압이 0.5v이면 (5+9*0.5)/10=0.95V까지 올라갈 수 있다.
	물론 decay하기 때문에 그 정도는 아니겠지만 상당히 많이 전압이 올라갈 수 있다.

### 3. 해결방안 ###

A.	PIC18F4550에서 recommend한 대로 ADC channel switching하고 최소 2.45usec의 delay를 주도록한다.

PCRTask.c 안에 ReadTemperature(BYTE sensor) 안에

	switch(sensor)
	{
		…
	}

	//여기에 2.45us 이상 delay 추가
	WORD delay_cnt;
	for (delay_cnt=0;delay_cnt<10;delay_cnt++); //약 5us예상

	do { delay_cnt--;} while(delay_cnt);

	while(counter--)
	{
		…
	}
