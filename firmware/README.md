Real-Time Audio FFT (firmware)
==============================

Open the project in Keil uVision 5 (MDK-ARM). Code is slightly above the 32Kb limit thus would require Keil Professional to compile. You can use the 7 day evaluation.
STM32Cube MX tool was used to generate the base source with FreeRTOS and USB CDC.

Adding Board Support Packages (BSP)
-----------------------------------

- Copy Drivers/BSP for STM32F4 Discovery
- Create Groups in Keil Project for `Drivers/BSP/STM32F4-Discovery`, `Drivers/BSP/Components` and `Middlewares/STM32_Audio` and add relevant files
- Open `stm32f4xx_hal_conf.h` and comment out the #define for required HAL driver includes
- Open `Options for Target..` -> `C/C++` and add suitable directory to include path
- Build Project
  
Refer `STM32Cube_FW_F4_V1.11.0\Projects\STM32F4-Discovery\Examples\BSP`

Fix for USB CDC VCP (Exclamation mark in Device Manager)
--------------------------------------------------------

- In usbd_def.h change USB_HS_MAX_PACKET_SIZE from `512` to `256`
- In usbd_cdc.h change CDC_DATA_HS_MAX_PACKET_SIZE from `512` to `256`