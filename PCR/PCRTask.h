/*****************************
* File Name: PCRTask.h
* Date: 2015.07.16
* Author: Jun Yeon
*****************************/

#ifndef __PCRTASK_H__
#define __PCRTASK_H__

#include "./DEFINE/GenericTypeDefs.h"

void PCR_Task(void);
WORD ReadTemperature(BYTE sensor);
WORD ReadPhotodiode(void);
double quickSort(float *d, int n);

void Sensor_Task(void);
void Command_Setting(void);
void Run_Task(void);
void Stop_Task(void);
void TxBuffer_Setting(void);


#endif

