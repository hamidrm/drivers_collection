#include "stm32f0xx_hal.h"
#include "string.h"
#include "nrf24l01_hal.h"
#include "util.h"



static Nrf24_LL_CB Nrf24Int;
SPI_HandleTypeDef Nrf24SpiHandler;
static uint8_t Nrf24_LL_Init(void);






uint8_t Nrf24_HAL_Init(Nrf24_LL_CB CB){
  uint8_t Status;
  Status = Nrf24_LL_Init();
  Nrf24Int = CB;
  return Status;
}



static uint8_t Nrf24_LL_Init(void){
  uint8_t res;
  
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /* Enable GPIOA Clock  */
  ClockEnable(GPIOA);

  /* Enable GPIOB Clock  */
  ClockEnable(GPIOB);
  
  /* Enable SPI Clock  */
  ClockEnable(SPI1);
  
  /* SPI SCK GPIO pin configuration  */
  GPIO_InitStruct.Pin       = NRF24_SCK_PIN | NRF24_MISO_PIN | NRF24_MOSI_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLDOWN;
  GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(NRF24_SCK_PORT, &GPIO_InitStruct);
  
  /* NRF24 CE GPIO pin configuration  */
  
  GPIO_InitStruct.Pin = NRF24_CE_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLDOWN;
  HAL_GPIO_Init(NRF24_CE_PORT, &GPIO_InitStruct);

  
  /* NRF24 CS GPIO pin configuration  */
  GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pin = NRF24_CS_PIN;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  HAL_GPIO_Init(NRF24_CS_PORT, &GPIO_InitStruct);
  
  
  GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
    
  /* NRF24 IRQ GPIO pin configuration  */
  //GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pin = NRF24_IRQ_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull      = GPIO_PULLDOWN;
  HAL_GPIO_Init(NRF24_IRQ_PORT, &GPIO_InitStruct);

  /* NRF24 External Interrupt IRQ */
  
  HAL_NVIC_SetPriority(NRF24_EXTI_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(NRF24_EXTI_IRQn);
  
  
  /* Clear SPI handle parameters */
  memset(&Nrf24SpiHandler,0,sizeof(Nrf24SpiHandler));
  
  /* Set CS pin*/
  NRF24_SPI_END();

  /* Set the SPI parameters */
  Nrf24SpiHandler.Instance               = SPIx;
  Nrf24SpiHandler.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  Nrf24SpiHandler.Init.Direction         = SPI_DIRECTION_2LINES;
  Nrf24SpiHandler.Init.CLKPhase          = SPI_PHASE_1EDGE;
  Nrf24SpiHandler.Init.CLKPolarity       = SPI_POLARITY_LOW;
  Nrf24SpiHandler.Init.DataSize          = SPI_DATASIZE_8BIT;
  Nrf24SpiHandler.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  Nrf24SpiHandler.Init.TIMode            = SPI_TIMODE_DISABLE;
  Nrf24SpiHandler.Init.NSS               = SPI_NSS_SOFT;
  Nrf24SpiHandler.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
  Nrf24SpiHandler.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  Nrf24SpiHandler.Init.CRCPolynomial     = 7;
  Nrf24SpiHandler.Init.CRCLength         = SPI_CRC_LENGTH_8BIT;
  Nrf24SpiHandler.Init.Mode = SPI_MODE_MASTER;
  
  HAL_SPI_DeInit(&Nrf24SpiHandler);
    
  __HAL_SPI_DISABLE(&Nrf24SpiHandler);
  /* Init the SPI */
  res = HAL_SPI_Init(&Nrf24SpiHandler);

  __HAL_SPI_ENABLE(&Nrf24SpiHandler);
  
  return res;
}

uint8_t AAA = 0xFF;
void EXTI4_15_IRQHandler(void)
{
  
  if(__HAL_GPIO_EXTI_GET_IT(NRF24_IRQ_PIN) != RESET) 
  { 
    __HAL_GPIO_EXTI_CLEAR_IT(NRF24_IRQ_PIN);
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,AAA);
    AAA = ~AAA;
    if(Nrf24Int != NULL)
      Nrf24Int();
  }
  HAL_NVIC_ClearPendingIRQ(NRF24_EXTI_IRQn);
}


uint8_t Nrf24_Write(uint8_t reg,uint8_t * data,uint8_t len){
  uint8_t status;
  
  volatile uint8_t dummy = 0;
  HAL_NVIC_DisableIRQ(NRF24_EXTI_IRQn);

  len = (len == 0)?1:len;
  
  NRF24_SPI_BEGIN();

  NRF24_SPI_TX_WAIT();

  NRF24_SPI_TXD(reg);

  NRF24_SPI_RX_WAIT();

  status = NRF24_SPI_RXD();

  while(len--){
    
    NRF24_SPI_TX_WAIT();

    NRF24_SPI_TXD(*data++);

    NRF24_SPI_RX_WAIT();

    dummy = NRF24_SPI_RXD();

  }
  
  
  NRF24_SPI_END();
  
  sleep(800);
  HAL_NVIC_EnableIRQ(NRF24_EXTI_IRQn);
  return status;
}

uint8_t Nrf24_Read(uint8_t reg,uint8_t * data,uint8_t len){

  uint8_t status;
  HAL_NVIC_DisableIRQ(NRF24_EXTI_IRQn);
  volatile uint8_t dummy = 0;
  
  len = (len == 0)?1:len;
  
  NRF24_SPI_BEGIN();

  NRF24_SPI_TX_WAIT();

  NRF24_SPI_TXD(reg);

  NRF24_SPI_RX_WAIT();

  status = NRF24_SPI_RXD();

  while(len--){
    
    NRF24_SPI_TX_WAIT();

    NRF24_SPI_TXD(dummy);

    NRF24_SPI_RX_WAIT();

    *data++ = NRF24_SPI_RXD();

  }
  
  
  NRF24_SPI_END();
  
  sleep(800);
  HAL_NVIC_EnableIRQ(NRF24_EXTI_IRQn);
  return status;
}