#include "stm32f0xx_hal.h"
#include "string.h"
#include "nrf24l01_hal.h"
#include "nrf24l01.h"
#include "util.h"

static Nrf24IntCB IntCallBack;
static Nrf24BrustMode  BrustMode = Nrf24BrustMode_None;
static Nrf24MachineStates CurrentState = Nrf24MachineState_PowerDown;
static void Nrf24LL_Int(void);
static void Nrf24_ClearIntFlag(uint8_t flags);
static void Nrf24_SetState(Nrf24MachineStates s);


uint8_t Nrf24_Init(Nrf24IntCB cb){
  uint8_t status;
  IntCallBack = cb;
  
  __disable_interrupt();
  status = Nrf24_HAL_Init(Nrf24LL_Int);
  sleep(200000);
  Nrf24_SetPwrMode(Nrf24Config_PwrDown);
  Nrf24_ClearIntFlag(Nrf24Status_RXInt | Nrf24Status_TXInt | Nrf24Status_MaxRetrasmitsInt);
  Nrf24_SetIntMode(Nrf24Config_RXInt | Nrf24Config_TXInt | Nrf24Config_MaxRetrasmitsInt);
  
  Nrf24_FlushTxFifo();
  Nrf24_FlushRxFifo();
  
  __enable_interrupt();
  return status;
}

void Nrf24_SetBrustMode(Nrf24BrustMode Mode){
  BrustMode = Mode;
}

void Nrf24_PwrUp(void){
  Nrf24_FlushRxFifo();
  Nrf24_FlushTxFifo();
  Nrf24_SetState(Nrf24MachineState_Standby1);
}


void Nrf24_Sleep(void){
  Nrf24_FlushRxFifo();
  Nrf24_FlushTxFifo();
  Nrf24_SetState(Nrf24MachineState_PowerDown);
}


void Nrf24_OpenInputStream(void){
  Nrf24_SetState(Nrf24MachineState_RxMode);
}

void Nrf24_CloseStream(void){
  Nrf24_SetState(Nrf24MachineState_Standby1);
}

void Nrf24_OpenOutputStream(void){
  Nrf24_SetState(Nrf24MachineState_TxAllMode);
}

static void Nrf24LL_Int(void){
    Nrf24Status Status = Nrf24_GetStatus();
    Nrf24FifoStatus FifoStatus = Nrf24_GetFifoStatus();

    if(Status.IsTxDataInt && FifoStatus.IsTxEmpety){
      if((BrustMode & Nrf24BrustMode_Tx)==0)
        Nrf24_SetState(Nrf24MachineState_Standby1);
    }
    if(Status.IsRxDataInt && (!FifoStatus.IsRxEmpety)){
      if((BrustMode & Nrf24BrustMode_Rx)==0)
        Nrf24_SetState(Nrf24MachineState_Standby1);
    }
    if(IntCallBack)
      IntCallBack((Nrf24EventType)((Status.IsRxDataInt) | (Status.IsTxDataInt << 1) | (Status.IsMaxRetrasmitsInt << 2)));
    Nrf24_ClearIntFlag(7);
}


static void Nrf24_SetState(Nrf24MachineStates s){
  switch(s){
  case Nrf24MachineState_RxMode:
    {
      if(CurrentState == Nrf24MachineState_RxMode)
        return;
      if(CurrentState != Nrf24MachineState_Standby1)
        Nrf24_SetState(Nrf24MachineState_Standby1);
      NRF24_HAL_CE_SET();
      Nrf24_SetSystemMode(Nrf24Config_PRX);
      
      sleep(350); //Wait for PLL locking
      
      CurrentState = Nrf24MachineState_RxMode;
    }
    break;
  case Nrf24MachineState_TxAllMode:
    {
      if(CurrentState == Nrf24MachineState_TxAllMode)
        return;
      Nrf24FifoStatus Status = Nrf24_GetFifoStatus();
      if(CurrentState != Nrf24MachineState_Standby1)
        Nrf24_SetState(Nrf24MachineState_Standby1);
      NRF24_HAL_CE_SET();
      Nrf24_SetSystemMode(Nrf24Config_PTX);
      sleep(350); //Wait for PLL locking
      
      if(Status.IsTxEmpety)
        CurrentState = Nrf24MachineState_Standby2;
      else
        CurrentState = Nrf24MachineState_TxAllMode;
    }
    break;
  case Nrf24MachineState_Tx1Mode:
    {
      if(CurrentState == Nrf24MachineState_Tx1Mode)
        return;
      Nrf24FifoStatus Status = Nrf24_GetFifoStatus();
      if(CurrentState != Nrf24MachineState_Standby1)
        Nrf24_SetState(Nrf24MachineState_Standby1);
      NRF24_HAL_CE_SET();
      Nrf24_SetSystemMode(Nrf24Config_PTX);
      sleep(15);
      NRF24_HAL_CE_CLEAR();
      sleep(180); //Wait for PLL locking + Sending data
      if(Status.IsTxEmpety)
        CurrentState = Nrf24MachineState_Standby2;
      else
        CurrentState = Nrf24MachineState_Standby1;
    }
    break;
  case Nrf24MachineState_Standby2:
    if(CurrentState == Nrf24MachineState_Standby2)
      return;
    break;
  case Nrf24MachineState_Standby1:
    {
      if(CurrentState == Nrf24MachineState_Standby1)
        return;
      if(CurrentState == Nrf24MachineState_PowerDown){
        Nrf24_SetPwrMode(Nrf24Config_PwrUp);
        sleep(2000);
      }else{
        NRF24_HAL_CE_CLEAR();
      }
      CurrentState = Nrf24MachineState_Standby1;
    }
    break;
  case Nrf24MachineState_PowerDown:
    {
      if(CurrentState == Nrf24MachineState_PowerDown)
        return;
      NRF24_HAL_CE_CLEAR();
      Nrf24_SetPwrMode(Nrf24Config_PwrDown);
    }
    break;
  }
}

uint8_t Nrf24_GetRxFifoLen(uint8_t  * len){
  return Nrf24_Read(NRF24_R_RX_PL_WID,len,1);
}

uint8_t Nrf24_WriteAckPayload(uint8_t pipe,uint8_t  * data,uint8_t len){
  return Nrf24_Write(NRF24_W_ACK_PAYLOAD + pipe,data,len);
}

uint8_t Nrf24_WritePayloadNoAck(uint8_t  * data,uint8_t len){
  return Nrf24_Write(NRF24_W_TX_PAYLOAD_NOACK,data,len);
}


uint8_t Nrf24_ReadPayload(uint8_t * data,uint8_t len){
  return Nrf24_Read(NRF24_R_RX_PAYLOAD,data,len);
}

uint8_t Nrf24_WritePayload(uint8_t * data,uint8_t  len){
  return Nrf24_Write(NRF24_W_TX_PAYLOAD,data,len);
}

uint8_t Nrf24_ReuseTxData(void){
  return Nrf24_Write(NRF24_REUSE_TX_PL,NULL,0);
}

uint8_t Nrf24_FlushTxFifo(void){
  return Nrf24_Write(NRF24_FLUSH_TX,NULL,0);
}

uint8_t Nrf24_FlushRxFifo(void){
  return Nrf24_Write(NRF24_FLUSH_RX,NULL,0);
}


uint8_t Nrf24_SetPwrMode(Nrf24ConfigPowerMode PwrMode){
  uint8_t  CurrentCfg;
  Nrf24_GetConfig((Nrf24Config *)&CurrentCfg);
  CurrentCfg &= ~(1<<NRF24_PWR_UP);
  CurrentCfg |= ((uint8_t)(PwrMode))<<NRF24_PWR_UP;
  return Nrf24_SetConfig((Nrf24Config *)&CurrentCfg);
}

Nrf24ConfigPowerMode Nrf24_GetPwrMode(void){
  Nrf24Config  CurrentCfg;
  Nrf24_GetConfig(&CurrentCfg);
  return (Nrf24ConfigPowerMode)(CurrentCfg.Power);
}


uint8_t Nrf24_SetCrcMode(Nrf24ConfigCrcMode CrcMode){
  uint8_t  CurrentCfg;
  Nrf24_GetConfig((Nrf24Config *)&CurrentCfg);
  CurrentCfg &= ~(Nrf24Config_Crc2Byte<<NRF24_CRCO);
  CurrentCfg |= ((uint8_t)(CrcMode))<<NRF24_CRCO;
  return Nrf24_SetConfig((Nrf24Config *)&CurrentCfg);
}

Nrf24ConfigCrcMode Nrf24_GetCrcMode(void){
  Nrf24Config  CurrentCfg;
  Nrf24_GetConfig(&CurrentCfg);
  return (Nrf24ConfigCrcMode)(CurrentCfg.Crc);
}


uint8_t Nrf24_SetIntMode(uint8_t IntMode){
  uint8_t  CurrentCfg;
  Nrf24_GetConfig((Nrf24Config *)&CurrentCfg);
  CurrentCfg &= ~(7<<NRF24_MASK_MAX_RT);
  CurrentCfg |= ((uint8_t)((~IntMode)&0x7))<<NRF24_MASK_MAX_RT;
  return Nrf24_SetConfig((Nrf24Config *)&CurrentCfg);
}

uint8_t Nrf24_GetIntMode(void){
  Nrf24Config  CurrentCfg;
  Nrf24_GetConfig(&CurrentCfg);
  return (Nrf24ConfigCrcMode)(CurrentCfg.Interrupt);
}

uint8_t Nrf24_SetSystemMode(Nrf24ConfigPrimRxTxMode SysMode){
  uint8_t  CurrentCfg;
  Nrf24_GetConfig((Nrf24Config *)&CurrentCfg);
  CurrentCfg &= ~(1<<NRF24_PRIM_RX);
  CurrentCfg |= ((uint8_t)(SysMode))<<NRF24_PRIM_RX;
  return Nrf24_SetConfig((Nrf24Config *)&CurrentCfg);
}

Nrf24ConfigPrimRxTxMode Nrf24_GetSystemMode(void){
  Nrf24Config  CurrentCfg;
  Nrf24_GetConfig(&CurrentCfg);
  return (Nrf24ConfigPrimRxTxMode)(CurrentCfg.RxTxControl);
}


uint8_t Nrf24_SetConfig(Nrf24Config * Nrf24Cfg){
  return Nrf24_WriteRegister(NRF24_CONFIG,(uint8_t *)Nrf24Cfg,1);
}

uint8_t Nrf24_GetConfig(Nrf24Config * Nrf24Cfg){
  uint8_t CurrentCfg;
  uint8_t status = Nrf24_ReadRegister(NRF24_CONFIG,&CurrentCfg,1);
  *Nrf24Cfg = *(Nrf24Config *)&CurrentCfg;
  return status;
}




uint8_t Nrf24_SetAAStatus(uint8_t status){
  //assert(status & 0xC0 == 0)
  return Nrf24_WriteRegister(NRF24_EN_AA,&status,1);
}

uint8_t Nrf24_GetAAStatus(uint8_t * status){
  return Nrf24_ReadRegister(NRF24_EN_AA,status,1);
}



uint8_t Nrf24_SetRXAddrStatus(uint8_t status){
  //assert(status & 0xC0 == 0)
  return Nrf24_WriteRegister(NRF24_EN_RXADDR,&status,1);
}

uint8_t Nrf24_GetRXAddrStatus(uint8_t * status){
  return Nrf24_ReadRegister(NRF24_EN_RXADDR,status,1);
}



uint8_t Nrf24_SetAddrWidths(uint8_t width){
  //assert(2 < width < 6)
  width = width - 2;
  return Nrf24_WriteRegister(NRF24_SETUP_AW,&width,1);
}

uint8_t Nrf24_GetAddrWidths(uint8_t * width){
  uint8_t status;
  status = Nrf24_ReadRegister(NRF24_SETUP_AW,width,1);
  *width = *width + 2;
  return status;
}


uint8_t Nrf24_SetRetrasmissionCfg(uint16_t delay,uint8_t count){
  //assert(0 < delay =< 4000)
  //assert(0 =< count =< 15)
  uint8_t _delay = ((delay - 1) / 250);
  uint8_t _v = (_delay << NRF24_ARD) | ((count  << NRF24_ARC) & 0x0F);
  return Nrf24_WriteRegister(NRF24_SETUP_RETR,&_v,1);
}

uint8_t Nrf24_GetRetrasmissionCfg(uint16_t * delay,uint8_t * count){
  uint8_t _v;
  uint8_t status = Nrf24_ReadRegister(NRF24_SETUP_RETR,&_v,1);
  *delay = ((_v >> NRF24_ARD) + 1) * 250;
  *count = (_v >> NRF24_ARC) & 0x0F;
  return status;
}

uint8_t Nrf24_SetChannel(uint8_t ch){
  //assert(0 =< ch =< 0x7F)
  return Nrf24_WriteRegister(NRF24_RF_CH,&ch,1);
}

uint8_t Nrf24_GetChannel(uint8_t * ch){
  return Nrf24_ReadRegister(NRF24_RF_CH,ch,1);
}


uint8_t Nrf24_SetRfConfig(Nrf24RfConfig * Nrf24RfCfg){
  return Nrf24_WriteRegister(NRF24_RF_SETUP,(uint8_t *)Nrf24RfCfg,1);
}

uint8_t Nrf24_GetRfConfig(Nrf24RfConfig * Nrf24RfCfg){
  uint8_t CurrentCfg;
  uint8_t status = Nrf24_ReadRegister(NRF24_RF_SETUP,&CurrentCfg,1);
  *Nrf24RfCfg = *(Nrf24RfConfig *)&CurrentCfg;
  return status;
}


uint8_t Nrf24_SetDataRate(Nrf24ConfigRfDataRate RfDr){
  uint8_t  CurrentRfCfg;
  Nrf24_GetRfConfig((Nrf24RfConfig *)&CurrentRfCfg);
  CurrentRfCfg &= ~(7<<NRF24_RF_DR);
  CurrentRfCfg |= ((uint8_t)(RfDr))<<NRF24_RF_DR;
  return Nrf24_SetRfConfig((Nrf24RfConfig *)&CurrentRfCfg);
}

uint8_t Nrf24_GetDataRate(Nrf24ConfigRfDataRate * RfDr){
  Nrf24RfConfig  CurrentRfCfg;
  uint8_t status = Nrf24_GetRfConfig(&CurrentRfCfg);
  *RfDr = (Nrf24ConfigRfDataRate)CurrentRfCfg.DataRate;
  return status;
}

uint8_t Nrf24_SetRfPower(Nrf24ConfigRfPower RfPwr){
  uint8_t  CurrentRfCfg;
  Nrf24_GetRfConfig((Nrf24RfConfig *)&CurrentRfCfg);
  CurrentRfCfg &= ~(3<<NRF24_RF_PWR);
  CurrentRfCfg |= ((uint8_t)(RfPwr))<<NRF24_RF_PWR;
  return Nrf24_SetRfConfig((Nrf24RfConfig *)&CurrentRfCfg);
}

uint8_t Nrf24_GetRfPower(Nrf24ConfigRfPower * RfPwr){
  Nrf24RfConfig  CurrentRfCfg;
  uint8_t status = Nrf24_GetRfConfig(&CurrentRfCfg);
  *RfPwr = (Nrf24ConfigRfPower)CurrentRfCfg.RfPower;
  return status;
}


static void Nrf24_ClearIntFlag(uint8_t flags){
  uint8_t status;
  Nrf24_ReadRegister(NRF24_STATUS,&status,1);
  status |= flags<<NRF24_MAX_RT;
  Nrf24_WriteRegister(NRF24_STATUS,&status,1);
}

Nrf24Status Nrf24_GetStatus(void){
  uint8_t status,dummy;
  status = Nrf24_Read(NRF24_NOP,&dummy,1);
  return *((Nrf24Status *)&status);
}



uint8_t Nrf24_TransmitObserve(uint8_t * PacketsLostCount,uint8_t * PacketsRetransmittedCount){
  uint8_t status;
  uint8_t TO;
  status = Nrf24_ReadRegister(NRF24_OBSERVE_TX,&TO,1);
  *PacketsLostCount = TO >> NRF24_PLOS_CNT;
  *PacketsRetransmittedCount = (TO >> NRF24_ARC_CNT) & 0x0F;
  return status;
}

uint8_t Nrf24_IsRssiInRange(void){
  uint8_t RCD;
  Nrf24_ReadRegister(NRF24_CD,&RCD,1);
  return RCD;
}

uint8_t Nrf24_GetReceiveAddress(uint8_t Pipe,uint8_t * Addr,uint8_t * AddrLen){
  Nrf24_GetAddrWidths(AddrLen);
  if(Pipe == 0){
    return Nrf24_ReadRegister(NRF24_RX_ADDR_P0,Addr,*AddrLen);
  }else if(Pipe == 1){
    return Nrf24_ReadRegister(NRF24_RX_ADDR_P1,Addr,*AddrLen);
  }else if(Pipe < 6){
    uint8_t PipeAddress = Pipe + NRF24_RX_ADDR_P0;
    Nrf24_ReadRegister(NRF24_RX_ADDR_P1,Addr,*AddrLen);
    return Nrf24_ReadRegister(*((Nrf24Registers *)&PipeAddress),Addr,1);
  }
  return 0;
}

uint8_t Nrf24_SetReceiveAddress(uint8_t Pipe,uint8_t * Addr){
  uint8_t AddrLen;
  Nrf24_GetAddrWidths(&AddrLen);
  if(Pipe == 0){
    return Nrf24_WriteRegister(NRF24_RX_ADDR_P0,Addr,AddrLen);
  }else if(Pipe == 1){
    return Nrf24_WriteRegister(NRF24_RX_ADDR_P1,Addr,AddrLen);
  }else if(Pipe < 6){
    uint8_t PipeAddress = Pipe + NRF24_RX_ADDR_P0;
    return Nrf24_WriteRegister(*((Nrf24Registers *)&PipeAddress),Addr,1);
  }
  return 0;
}

uint8_t Nrf24_SetTxAddress(uint8_t * Addr){
  uint8_t AddrLen;
  Nrf24_GetAddrWidths(&AddrLen);
  return Nrf24_WriteRegister(NRF24_TX_ADDR,Addr,AddrLen);
}

uint8_t Nrf24_GetTxAddress(uint8_t * Addr){
  uint8_t AddrLen;
  Nrf24_GetAddrWidths(&AddrLen);
  return Nrf24_ReadRegister(NRF24_TX_ADDR,Addr,AddrLen);
}

uint8_t Nrf24_SetRxBytesCount(uint8_t Pipe,uint8_t Count){
  uint8_t PipeAddress = Pipe + NRF24_RX_PW_P0;
  Nrf24_WriteRegister(*((Nrf24Registers *)&PipeAddress),&Count,1);
  return Count;
}

uint8_t Nrf24_GetRxBytesCount(uint8_t Pipe){
  uint8_t Count , PipeAddress = Pipe + NRF24_RX_PW_P0;
  Nrf24_ReadRegister(*((Nrf24Registers *)&PipeAddress),&Count,1);
  return Count;
}


Nrf24FifoStatus Nrf24_GetFifoStatus(void){
  Nrf24FifoStatus status;
  Nrf24_ReadRegister(NRF24_FIFO_STATUS,(uint8_t *)&status,1);
  return status;
}

uint8_t Nrf24_GetDynPayloadLenStatus(uint8_t * Status){
  return Nrf24_ReadRegister(NRF24_DYNPD,Status,1);
}

uint8_t Nrf24_SetDynPayloadLenStatus(uint8_t * Status){
  return Nrf24_WriteRegister(NRF24_DYNPD,Status,1);
}

uint8_t Nrf24_GetFeature(Nrf24Feature * Feature){
  return Nrf24_WriteRegister(NRF24_FEATURE,(uint8_t *)Feature,1);
}

uint8_t Nrf24_SetFeature(Nrf24Feature * Feature){
  return Nrf24_WriteRegister(NRF24_FEATURE,(uint8_t *)Feature,1);
}