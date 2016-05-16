#include "microphone.h"

uint16_t WrBuffer[WR_BUFFER_SIZE];
static uint16_t RecBuf[PCM_OUT_SIZE*2];	/* PCM stereo samples are saved in RecBuf */
static uint16_t InternalBuffer[INTERNAL_BUFF_SIZE]; /* PDM is stored in InternalBuffer */
__IO uint32_t ITCounter = 0;
__IO uint32_t AudioDataReady = 0, AudioBuffOffset = 0;

// PCM_OUT_SIZE = 16
// INTERNAL_BUFF_SIZE = 128
// RecBuf = 32 (16bit)
// InternalBuffer = 128 (16bit)
// WrBuffer = 4096 (16bit)

void StartAudioCapture(xQueueHandle *q)
{
	uint16_t *ptr;
	ITCounter = 0;
	/* 8000 - Sampling Freq, 16 - Bits/Sample, 1 - Mono Channel */	
	BSP_AUDIO_IN_Init(DEFAULT_AUDIO_IN_FREQ, DEFAULT_AUDIO_IN_BIT_RESOLUTION, DEFAULT_AUDIO_IN_CHANNEL_NBR);
	BSP_AUDIO_IN_Record((uint16_t*)&InternalBuffer[0], INTERNAL_BUFF_SIZE);

	for(;;) {
		// TODO: Using semaphore instead of flag would've been better
		if(AudioDataReady == 1) {			
			ptr = WrBuffer + AudioBuffOffset;
			// Send captured 2048 words for FFT processing			
			if(xQueueSend(*q, &ptr, 0) != pdPASS) {
				BSP_LED_Toggle(LED5);
			}
			AudioDataReady = 0;
		}
	}
}

/**
  * @brief  Start Audio recording.
  * @param  pBuf: pointer to a buffer
  *         wSize: Buffer size
  * @retval None
  */
uint8_t StartAudioIn(uint16_t* pBuf, uint32_t wSize)
{
  return (BSP_AUDIO_IN_Record(pBuf, wSize));
}

/**
  * @brief  Stop Audio recording.
  * @param  None
  * @retval None
  */
uint32_t StopAudioIn(void)
{
  return BSP_AUDIO_IN_Stop();
}

/**
  * @brief  Manages the DMA Half Transfer complete interrupt.
  * @param  None
  * @retval None
  */
// First 64 words to RecBuf
void BSP_AUDIO_IN_HalfTransfer_CallBack(void)
{
  /* PDM to PCM data convert */
  BSP_AUDIO_IN_PDMToPCM((uint16_t*)&InternalBuffer[0], (uint16_t*)&RecBuf[0]);
  
  /* Copy PCM data in internal buffer */
	// 64 bytes copied
  memcpy((uint16_t*)&WrBuffer[ITCounter * (PCM_OUT_SIZE*2)], RecBuf, PCM_OUT_SIZE*4);
  	
  if(ITCounter == (WR_BUFFER_SIZE/(PCM_OUT_SIZE*4))-1)
  {
		AudioDataReady = 1;
    AudioBuffOffset = 0;
    ITCounter++;
  }
  else if(ITCounter == (WR_BUFFER_SIZE/(PCM_OUT_SIZE*2))-1)
  {    
		AudioDataReady = 1;
    AudioBuffOffset = WR_BUFFER_SIZE/2;
    ITCounter = 0;
  }
  else
  {		
    ITCounter++;
  }
}

/**
  * @brief  Calculates the remaining file size and new position of the pointer.
  * @param  None
  * @retval None
  */
// Next 64 words to RecBuf
void BSP_AUDIO_IN_TransferComplete_CallBack(void)
{
  /* PDM to PCM data convert */
  BSP_AUDIO_IN_PDMToPCM((uint16_t*)&InternalBuffer[INTERNAL_BUFF_SIZE/2], (uint16_t*)&RecBuf[0]);
  
  /* Copy PCM data in internal buffer */
	// 64 bytes copied
  memcpy((uint16_t*)&WrBuffer[ITCounter * (PCM_OUT_SIZE*2)], RecBuf, PCM_OUT_SIZE*4);
  
  if(ITCounter == (WR_BUFFER_SIZE/(PCM_OUT_SIZE*4))-1)
  {
		// 63 (4096 bytes / 2048 words (16bit) copied to WrBuffer
		AudioDataReady = 1;
    AudioBuffOffset = 0;
    ITCounter++;
		
  }
  else if(ITCounter == (WR_BUFFER_SIZE/(PCM_OUT_SIZE*2))-1)
  {
		// 127 (next 4096 bytes / 2048 words (16bit) copied to WrBuffer
		AudioDataReady = 1;
    AudioBuffOffset = WR_BUFFER_SIZE/2;
    ITCounter = 0;
  }
  else
  {
    ITCounter++;
  }
}
