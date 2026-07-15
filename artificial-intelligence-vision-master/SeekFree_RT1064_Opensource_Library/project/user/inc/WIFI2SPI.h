#ifndef _WIFI2SPI_h
#define _WIFI2SPI_h

#include "zf_common_headfile.h"

void myWIFI2SPI_Init();
void SendDataToAssistant(seekfree_assistant_oscilloscope_struct* sendDataStructure,char num);
void ReceiveData();
void wifi_task();

extern uint8_t lost;
#endif