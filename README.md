# Micro PCR Firmware - LED PWM 제어 기능 추가 #
**2016-09-21 이승철**
##

MicroPCR은 4개의 LED가 digital port를 사용하므로, 일반적인 PWM 제어를 할 수 없다.

따라서 타이머를 이용한 Software PWM을 구현해 4개의 LED를 PWM 제어를 한다.

##
#주의#
본 Firmware는 최신 MicroPCR_MFC (v2.6) 또는 Labgenius_ver2 를 사용해야 합니다. 