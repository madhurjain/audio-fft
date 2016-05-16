
#ifndef __MIC_H
#define __MIC_H

#include <stdio.h>
#include <string.h>
#include "stm32f4_discovery_audio.h"
#include "cmsis_os.h"

#define WR_BUFFER_SIZE           4096

void StartAudioCapture(xQueueHandle *q);
uint8_t StartAudioIn(uint16_t* pBuf, uint32_t wSize);
uint32_t StopAudioIn(void);

#endif
