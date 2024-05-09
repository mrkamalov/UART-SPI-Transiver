/**
  * @file           : spiTransiver.c
  * @author         : Kamalov Marat
  * @date			: 06.05.2024
*/

#include "cmsis_os.h"
#include "main.h"
#include "string.h"
#include "uartTransiver.h"

#define SPI_TIMEOUT 25000
static SPI_HandleTypeDef* pSPI;
static uint16_t transmitBuffer[BUFFER_SIZE];
static uint16_t receiveBuffer[BUFFER_SIZE];
extern osMessageQueueId_t spiSendQueueHandle;
extern osMessageQueueId_t uartSendQueueHandle;
extern IWDG_HandleTypeDef hiwdg;
volatile uint8_t spiSendFlag = 0;

void spiTransiverTask(void *argument)
{
  uint16_t uartMsg[10]={0xaa};
  pSPI = (SPI_HandleTypeDef*)argument;

  HAL_GPIO_WritePin(SPI2_EN_GPIO_Port, SPI2_EN_Pin, GPIO_PIN_RESET);
  memset(transmitBuffer, stringEndSymbol, BUFFER_SIZE);
  HAL_SPI_TransmitReceive_IT(pSPI, (uint8_t*)transmitBuffer, (uint8_t*)receiveBuffer, 1);
  for(;;)
  {
	HAL_IWDG_Refresh(&hiwdg);
  	if(osMessageQueueGet (spiSendQueueHandle, &uartMsg, 0, SECOND_TIMEOUT) == osOK){
  	  if(getUARTLastByte()==stringEndSymbol) spiSendFlag = 0;
  	  else spiSendFlag = 1;
  	  transmitBuffer[0] = uartMsg[0];
      HAL_SPI_TransmitReceive_IT(pSPI, (uint8_t*)transmitBuffer, (uint8_t*)receiveBuffer, 1);
  	}
  	else{
  	  spiSendFlag = 0;
  	  HAL_SPI_TransmitReceive_IT(pSPI, (uint8_t*)transmitBuffer, (uint8_t*)receiveBuffer, 1);
  	}
  }
}

uint8_t isDataTransmitted(uint16_t data){
  static uint8_t transmissionFlag = 0;

  if(transmissionFlag==0){
    if((data&0xff00)>>8==stringEndSymbol){
    	transmissionFlag = 1;
      return 1;
    }
	else return 0;
  }
  else{
    if((data&0xff)==stringEndSymbol){
      transmissionFlag = 0;
	  return 1;
	}
	else return 1;
  }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if(hspi != pSPI) return;
  if(pSPI->Instance->SR&SPI_FLAG_RXNE){
	if(isDataTransmitted(receiveBuffer[0])) osMessageQueuePut(uartSendQueueHandle,receiveBuffer,0,0);
  }
  if(spiSendFlag == 0){
    memset(transmitBuffer, stringEndSymbol, BUFFER_SIZE);
    HAL_SPI_TransmitReceive_IT(pSPI, (uint8_t*)transmitBuffer, (uint8_t*)receiveBuffer, 1);
  }
}
