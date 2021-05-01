#ifndef NRF24L01_HAL_H

#define SPIx    SPI1

#define NRF24_SCK_PIN           GPIO_PIN_5
#define NRF24_MISO_PIN          GPIO_PIN_6
#define NRF24_MOSI_PIN          GPIO_PIN_7
#define NRF24_CS_PIN            GPIO_PIN_9
#define NRF24_IRQ_PIN           GPIO_PIN_10
#define NRF24_EXTI_IRQn         EXTI4_15_IRQn
#define NRF24_CE_PIN            GPIO_PIN_1

#define NRF24_SCK_PORT          GPIOA
#define NRF24_MISO_PORT         GPIOA
#define NRF24_MOSI_PORT         GPIOA
#define NRF24_CS_PORT           GPIOA
#define NRF24_CE_PORT           GPIOB
#define NRF24_IRQ_PORT          GPIOA



#define NRF24_HAL_CE_SET()      HAL_GPIO_WritePin(NRF24_CE_PORT,NRF24_CE_PIN,GPIO_PIN_SET)
#define NRF24_HAL_CE_CLEAR()    HAL_GPIO_WritePin(NRF24_CE_PORT,NRF24_CE_PIN,GPIO_PIN_RESET)
#define NRF24_HAL_CE_PULSE()    do{\
                                  NRF24_SPI_CE_SET();\
                                  SLEEP(50);\
                                  NRF24_SPI_CE_CLEAR();\
                                }while(0)
#define NRF24_SPI_BEGIN()      HAL_GPIO_WritePin(NRF24_CS_PORT,NRF24_CS_PIN,GPIO_PIN_RESET)
#define NRF24_SPI_END()        HAL_GPIO_WritePin(NRF24_CS_PORT,NRF24_CS_PIN,GPIO_PIN_SET)
#define NRF24_SPI_TX_WAIT()    do{uint32_t TimeOut = NRF24_COMM_RETRY;while (TimeOut-- > 0 && ((SPIx->SR & SPI_FLAG_TXE) == 0 || (SPIx->SR & SPI_FLAG_BSY))){};}while(0)
#define NRF24_SPI_RX_WAIT()    do{uint32_t TimeOut = NRF24_COMM_RETRY;while (TimeOut-- > 0 && ((SPIx->SR & SPI_FLAG_RXNE) == 0 || (SPIx->SR & SPI_FLAG_BSY))){};}while(0)
#define NRF24_SPI_RXD()        (*(__IO uint8_t *)&SPIx->DR)
#define NRF24_SPI_TXD(data)    (*(__IO uint8_t *)&SPIx->DR) = data

#define NRF24_COMM_RETRY      10000


typedef void (*Nrf24_LL_CB)(void);




uint8_t Nrf24_HAL_Init(Nrf24_LL_CB CB);
uint8_t Nrf24_Write(uint8_t reg,uint8_t * data,uint8_t len);
uint8_t Nrf24_Read(uint8_t reg,uint8_t * data,uint8_t len);


#endif