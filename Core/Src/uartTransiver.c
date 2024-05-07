/**
  * @file           : uartTransiver.c
  * @author         : Kamalov Marat
  * @date			: 06.05.2024
*/

#include "cmsis_os.h"
#include "main.h"

static UART_HandleTypeDef* pUART;
static uint8_t uartBuffer[BUFFER_SIZE];
extern osMessageQueueId_t spiSendQueueHandle;
extern osMessageQueueId_t uartSendQueueHandle;

void uartTransiverTask(void *argument)
{
  uint8_t spiMsg[10];
  pUART = (UART_HandleTypeDef *)argument;

  HAL_UART_Receive_IT(pUART, uartBuffer, 1);
  for(;;)
  {
	if(osMessageQueueGet (uartSendQueueHandle, &spiMsg, 0, osWaitForever) == osOK)
	  HAL_UART_Transmit_IT(pUART, spiMsg, 1);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart == pUART){
    if(uartBuffer[0]==stringEndSymbol) osMessageQueuePut(spiSendQueueHandle,stringEndSymbol,0,1000);
    else osMessageQueuePut(spiSendQueueHandle,uartBuffer,0,1000);
    HAL_UART_Receive_IT(pUART, uartBuffer, 1);
  }
}
