#ifndef NRF24L01_H
#define NRF24L01_H



typedef enum {
  Nrf24Config_RXInt = 1<<2,
  Nrf24Config_TXInt = 1<<1,
  Nrf24Config_MaxRetrasmitsInt = 1,
} Nrf24ConfigIntMode;

typedef enum {
  Nrf24Config_CrcDisable = 0,
  Nrf24Config_Crc1Byte   = 2,
  Nrf24Config_Crc2Byte   = 3,
} Nrf24ConfigCrcMode;

typedef enum {
  Nrf24Config_PwrDown = 0,
  Nrf24Config_PwrUp   = 1,
} Nrf24ConfigPowerMode;

typedef enum {
  Nrf24Config_PTX = 0,
  Nrf24Config_PRX = 1,
} Nrf24ConfigPrimRxTxMode;

typedef struct {
  uint8_t RxTxControl : 1;
  uint8_t Power : 1;
  uint8_t Crc : 2;
  uint8_t Interrupt : 3;
  uint8_t Reserved : 1;
} Nrf24Config;


typedef enum {
  Nrf24ConfigRf_CarrierCont_Disable = 0,
  Nrf24ConfigRf_CarrierCont_Enable = 1,
} Nrf24ConfigRfCarrierCont;

typedef enum {
  Nrf24ConfigRf_DataRate_250Kbps = 4,
  Nrf24ConfigRf_DataRate_1Mbps = 0,
  Nrf24ConfigRf_DataRate_2Mbps = 1,
} Nrf24ConfigRfDataRate;

typedef enum {
  Nrf24ConfigRf_Power_n18dBm = 0,
  Nrf24ConfigRf_Power_n12dBm = 1,
  Nrf24ConfigRf_Power_n6dBm = 2,
  Nrf24ConfigRf_Power_0dBm = 3,
} Nrf24ConfigRfPower;

typedef struct {
  uint8_t Reserved1 : 1;
  uint8_t RfPower : 2;
  uint8_t DataRate : 3;
  uint8_t Reserved2 : 1;
  uint8_t CarrierCont : 1;
} Nrf24RfConfig;


typedef struct {
  uint8_t IsTxBuffFull : 1;
  uint8_t RxPipeNumber : 3;
  uint8_t IsMaxRetrasmitsInt : 1;
  uint8_t IsTxDataInt : 1;
  uint8_t IsRxDataInt : 1;
  uint8_t Reserved : 1;
} Nrf24Status;

typedef enum {
  Nrf24Status_RXInt = 1<<2,
  Nrf24Status_TXInt = 1<<1,
  Nrf24Status_MaxRetrasmitsInt = 1,
} Nrf24StatusInt;


typedef struct {
  uint8_t IsRxEmpety : 1;
  uint8_t IsRxFull : 1;
  uint8_t Reserved1 : 2;
  uint8_t IsTxEmpety : 1;
  uint8_t IsTxFull : 1;
  uint8_t TxReuse : 1;
  uint8_t Reserved2 : 1;
} Nrf24FifoStatus;

typedef struct {
  uint8_t EnableDynAck : 1;
  uint8_t EnablePayloadWithAck : 1;
  uint8_t EnableDynamicPayloadLen : 1;
  uint8_t Reserved : 5;
} Nrf24Feature;

typedef enum {
  Nrf24EventType_RXD = 1,
  Nrf24EventType_TXD = 2,
  Nrf24EventType_MaxRetransmission = 4,
} Nrf24EventType;

typedef enum {
  Nrf24BrustMode_None = 0,
  Nrf24BrustMode_Tx = 1,
  Nrf24BrustMode_Rx = 2,
  Nrf24BrustMode_Both = 3,
} Nrf24BrustMode;

typedef enum {
  Nrf24MachineState_RxMode,
  Nrf24MachineState_TxAllMode,
  Nrf24MachineState_Tx1Mode,
  Nrf24MachineState_Standby2,
  Nrf24MachineState_Standby1,
  Nrf24MachineState_PowerDown,
} Nrf24MachineStates;


typedef enum {
  NRF24_CONFIG      =   0x00,
  NRF24_EN_AA       =   0x01,
  NRF24_EN_RXADDR   =   0x02,
  NRF24_SETUP_AW    =	0x03,
  NRF24_SETUP_RETR  =	0x04,
  NRF24_RF_CH       =	0x05,
  NRF24_RF_SETUP    =	0x06,
  NRF24_STATUS      =	0x07,
  NRF24_OBSERVE_TX  =	0x08,
  NRF24_CD          =	0x09,
  NRF24_RX_ADDR_P0  =	0x0A,
  NRF24_RX_ADDR_P1  =	0x0B,
  NRF24_RX_ADDR_P2  =	0x0C,
  NRF24_RX_ADDR_P3  =	0x0D,
  NRF24_RX_ADDR_P4  =	0x0E,
  NRF24_RX_ADDR_P5  =	0x0F,
  NRF24_TX_ADDR     =	0x10,
  NRF24_RX_PW_P0    =	0x11,
  NRF24_RX_PW_P1    =	0x12,
  NRF24_RX_PW_P2    =	0x13,
  NRF24_RX_PW_P3    =	0x14,
  NRF24_RX_PW_P4    =	0x15,
  NRF24_RX_PW_P5    =	0x16,
  NRF24_FIFO_STATUS =	0x17,
  NRF24_DYNPD	    =	0x1C,
  NRF24_FEATURE	    =	0x1D,
} Nrf24Registers;


typedef void (*Nrf24IntCB) (Nrf24EventType);


#define NRF24_READ_REG_ADDR(x)    (x | NRF24_R_REGISTER)
#define NRF24_WRITE_REG_ADDR(x)    (x | NRF24_W_REGISTER)

#define Nrf24_WriteRegister(reg,data,len)       Nrf24_Write(NRF24_WRITE_REG_ADDR(reg),data,len)
#define Nrf24_ReadRegister(reg,data,len)       Nrf24_Read(NRF24_READ_REG_ADDR(reg),data,len)


/* Bit Mnemonics */
#define NRF24_MASK_RX_DR  6
#define NRF24_MASK_TX_DS  5
#define NRF24_MASK_MAX_RT 4
#define NRF24_EN_CRC      3
#define NRF24_CRCO        2
#define NRF24_PWR_UP      1
#define NRF24_PRIM_RX     0
#define NRF24_ENAA_P5     5
#define NRF24_ENAA_P4     4
#define NRF24_ENAA_P3     3
#define NRF24_ENAA_P2     2
#define NRF24_ENAA_P1     1
#define NRF24_ENAA_P0     0
#define NRF24_ERX_P5      5
#define NRF24_ERX_P4      4
#define NRF24_ERX_P3      3
#define NRF24_ERX_P2      2
#define NRF24_ERX_P1      1
#define NRF24_ERX_P0      0

#define NRF24_EN_RX_P5      (1<<NRF24_ERX_P5)
#define NRF24_EN_RX_P4      (1<<NRF24_ERX_P4)
#define NRF24_EN_RX_P3      (1<<NRF24_ERX_P3)
#define NRF24_EN_RX_P2      (1<<NRF24_ERX_P2)
#define NRF24_EN_RX_P1      (1<<NRF24_ERX_P1)
#define NRF24_EN_RX_P0      (1<<NRF24_ERX_P0)

#define NRF24_AW          0
#define NRF24_ARD         4
#define NRF24_ARC         0
#define NRF24_PLL_LOCK    4
#define NRF24_RF_DR       3
#define NRF24_RF_PWR      1
#define NRF24_RX_DR       6
#define NRF24_TX_DS       5
#define NRF24_MAX_RT      4
#define NRF24_RX_P_NO     1
#define NRF24_TX_FULL     0
#define NRF24_PLOS_CNT    4
#define NRF24_ARC_CNT     0
#define NRF24_TX_REUSE    6
#define NRF24_FIFO_FULL   5
#define NRF24_TX_EMPTY    4
#define NRF24_RX_FULL     1
#define NRF24_RX_EMPTY    0
#define NRF24_DPL_P5	    5
#define NRF24_DPL_P4	    4
#define NRF24_DPL_P3	    3
#define NRF24_DPL_P2	    2
#define NRF24_DPL_P1	    1
#define NRF24_DPL_P0	    0
#define NRF24_EN_DPL	    2
#define NRF24_EN_ACK_PAY        1
#define NRF24_EN_DYN_ACK        0

/* Instruction Mnemonics */
#define NRF24_R_REGISTER    0x00
#define NRF24_W_REGISTER    0x20
#define NRF24_REGISTER_MASK 0x1F
#define NRF24_ACTIVATE      0x50
#define NRF24_R_RX_PL_WID   0x60
#define NRF24_R_RX_PAYLOAD  0x61
#define NRF24_W_TX_PAYLOAD  0xA0
#define NRF24_W_ACK_PAYLOAD 0xA8
#define NRF24_W_TX_PAYLOAD_NOACK 0xB0
#define NRF24_FLUSH_TX      0xE1
#define NRF24_FLUSH_RX      0xE2
#define NRF24_REUSE_TX_PL   0xE3
#define NRF24_NOP           0xFF



uint8_t Nrf24_Init(Nrf24IntCB cb);
void Nrf24_PwrUp(void);
void Nrf24_Sleep(void);
void Nrf24_OpenInputStream(void);
void Nrf24_CloseStream(void);
void Nrf24_OpenOutputStream(void);
uint8_t Nrf24_GetRxFifoLen(uint8_t  * len);
uint8_t Nrf24_WriteAckPayload(uint8_t pipe,uint8_t  * data,uint8_t len);
uint8_t Nrf24_WritePayloadNoAck(uint8_t  * data,uint8_t len);
uint8_t Nrf24_ReadPayload(uint8_t * data,uint8_t len);
uint8_t Nrf24_WritePayload(uint8_t * data,uint8_t  len);
uint8_t Nrf24_GetRxFifoLen(uint8_t  * len);
uint8_t Nrf24_ReuseTxData(void);
uint8_t Nrf24_FlushTxFifo(void);
uint8_t Nrf24_FlushRxFifo(void);
uint8_t Nrf24_SetCrcMode(Nrf24ConfigCrcMode CrcMode);
Nrf24ConfigCrcMode Nrf24_GetCrcMode(void);
uint8_t Nrf24_SetPwrMode(Nrf24ConfigPowerMode PwrMode);
Nrf24ConfigPowerMode Nrf24_GetPwrMode(void);
uint8_t Nrf24_SetSystemMode(Nrf24ConfigPrimRxTxMode SysMode);
Nrf24ConfigPrimRxTxMode Nrf24_GetSystemMode(void);
uint8_t Nrf24_SetConfig(Nrf24Config * Nrf24Cfg);
uint8_t Nrf24_GetConfig(Nrf24Config * Nrf24Cfg);
uint8_t Nrf24_SetAAStatus(uint8_t status);
uint8_t Nrf24_GetAAStatus(uint8_t * status);
uint8_t Nrf24_SetRXAddrStatus(uint8_t status);
uint8_t Nrf24_GetRXAddrStatus(uint8_t * status);
uint8_t Nrf24_SetAddrWidths(uint8_t width);
uint8_t Nrf24_GetAddrWidths(uint8_t * width);
uint8_t Nrf24_SetRetrasmissionCfg(uint16_t delay,uint8_t count);
uint8_t Nrf24_GetRetrasmissionCfg(uint16_t * delay,uint8_t * count);
uint8_t Nrf24_SetChannel(uint8_t ch);
uint8_t Nrf24_GetChannel(uint8_t * ch);
uint8_t Nrf24_SetRfConfig(Nrf24RfConfig * Nrf24RfCfg);
uint8_t Nrf24_GetRfConfig(Nrf24RfConfig * Nrf24RfCfg);
uint8_t Nrf24_SetDataRate(Nrf24ConfigRfDataRate RfDr);
uint8_t Nrf24_GetDataRate(Nrf24ConfigRfDataRate * RfDr);
uint8_t Nrf24_SetRfPower(Nrf24ConfigRfPower RfPwr);
uint8_t Nrf24_GetRfPower(Nrf24ConfigRfPower * RfPwr);
Nrf24Status Nrf24_GetStatus(void);
uint8_t Nrf24_TransmitObserve(uint8_t * PacketsLostCount,uint8_t * PacketsRetransmittedCount);
uint8_t Nrf24_IsRssiInRange(void);
uint8_t Nrf24_GetReceiveAddress(uint8_t Pipe,uint8_t * Addr,uint8_t * AddrLen);
uint8_t Nrf24_SetReceiveAddress(uint8_t Pipe,uint8_t * Addr);
uint8_t Nrf24_SetTxAddress(uint8_t * Addr);
uint8_t Nrf24_GetTxAddress(uint8_t * Addr);
uint8_t Nrf24_SetRxBytesCount(uint8_t Pipe,uint8_t Count);
uint8_t Nrf24_GetRxBytesCount(uint8_t Pipe);
Nrf24FifoStatus Nrf24_GetFifoStatus(void);
uint8_t Nrf24_GetDynPayloadLenStatus(uint8_t * Status);
uint8_t Nrf24_SetDynPayloadLenStatus(uint8_t * Status);
uint8_t Nrf24_GetFeature(Nrf24Feature * Feature);
uint8_t Nrf24_SetFeature(Nrf24Feature * Feature);
uint8_t Nrf24_SetIntMode(uint8_t IntMode);
uint8_t Nrf24_GetIntMode(void);
void Nrf24_SetBrustMode(Nrf24BrustMode Mode);

#endif
