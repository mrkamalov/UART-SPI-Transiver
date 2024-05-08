/**
  * @file           : uartTransiver.c
  * @author         : Kamalov Marat
  * @date			: 06.05.2024
*/

#include "cmsis_os.h"
#include "main.h"
#include "uartTransiver.h"

static UART_HandleTypeDef* pUART;
static uint8_t uartBuffer[BUFFER_SIZE];
extern osMessageQueueId_t spiSendQueueHandle;
extern osMessageQueueId_t uartSendQueueHandle;
extern osSemaphoreId_t uartSendSemHandle;
static uint8_t lastByte = 0;
void uartTransiverTask(void *argument)
{
  uint16_t spiMsg[10];
  pUART = (UART_HandleTypeDef *)argument;

  HAL_UART_Receive_IT(pUART, uartBuffer, 1);
  for(;;)
  {
	if(osMessageQueueGet(uartSendQueueHandle, &spiMsg, 0, osWaitForever) != osOK) continue;
	if(osSemaphoreAcquire(uartSendSemHandle, SECOND_TIMEOUT)==osOK)
      HAL_UART_Transmit_IT(pUART, (uint8_t*)spiMsg, 2);
  }
}

void processBytes(uint8_t byte){
  static uint8_t prevByte = 0;
  static uint8_t isFirstByte = 1;
  static uint16_t value = 0;

  lastByte = byte;
  if(isFirstByte){
	if(byte==stringEndSymbol){
	  value = 0;
	  osMessageQueuePut(spiSendQueueHandle,&value,0,0);
	  return;
	}
	prevByte = byte;
	isFirstByte = 0;
  }
  else {
	value = (prevByte << 8) | byte;
	isFirstByte = 1;
	osMessageQueuePut(spiSendQueueHandle,&value,0,0);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart == pUART){
	processBytes(uartBuffer[0]);
    HAL_UART_Receive_IT(pUART, uartBuffer, 1);
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  osSemaphoreRelease(uartSendSemHandle);
}

uint8_t getUARTLastByte(void){
  return lastByte;
}

