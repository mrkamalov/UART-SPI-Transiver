/**
  * @file           : spiTransiver.c
  * @author         : Kamalov Marat
  * @date			: 06.05.2024
*/

#include "cmsis_os.h"
#include "main.h"
#include "string.h"

#define SPI_TIMEOUT 25000
static SPI_HandleTypeDef* pSPI;
static uint16_t spiBuffer[BUFFER_SIZE];
static uint8_t receiveBuffer[10];
extern osMessageQueueId_t spiSendQueueHandle;
extern osMessageQueueId_t uartSendQueueHandle;
extern IWDG_HandleTypeDef hiwdg;


void spiTransiverTask(void *argument)
{
  uint8_t uartMsg[10];
  pSPI = (SPI_HandleTypeDef*)argument;

  HAL_GPIO_WritePin(SPI2_EN_GPIO_Port, SPI2_EN_Pin, GPIO_PIN_RESET);
  memset(spiBuffer, stringEndSymbol, BUFFER_SIZE);
  HAL_SPI_Receive_IT(pSPI, (uint8_t*)spiBuffer, 1);//(pSPI, spiBuffer, receiveBuffer, 1);
  for(;;)
  {
	HAL_IWDG_Refresh(&hiwdg);
  	if(osMessageQueueGet (spiSendQueueHandle, &uartMsg, 0, SPI_TIMEOUT) != osOK)
      HAL_SPI_Transmit_IT(pSPI, uartMsg, 1);
  }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if(hspi != pSPI) return;
  if(pSPI->Instance->SR&SPI_FLAG_RXNE){
    if(spiBuffer[0]==stringEndSymbol) osMessageQueuePut(uartSendQueueHandle,stringEndSymbol,0,1000);
    else osMessageQueuePut(uartSendQueueHandle,spiBuffer,0,1000);
  }
  memset(spiBuffer, stringEndSymbol, 1);
  HAL_SPI_Receive_IT(pSPI, (uint8_t*)spiBuffer, 1);
}
