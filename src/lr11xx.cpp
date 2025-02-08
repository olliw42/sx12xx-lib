//*******************************************************
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// LR11XX standard interface
//*******************************************************
// contributed by JLP
//*******************************************************

#include "lr11xx.h"
#include <string.h>  // for memcpy

// spi

void Lr11xxDriverBase::SpiRead(uint8_t* datain, uint8_t len)
{
uint8_t dummy = 0; // NOP

    while (len) {
        SpiTransfer(dummy, datain);
        datain++;
        len--;
    }
}


void Lr11xxDriverBase::SpiWrite(uint8_t* dataout, uint8_t len)
{
uint8_t dummy;

    while (len) {
        SpiTransfer(*dataout, &dummy);
        dataout++;
        len--;
    }
}


// low level methods

void Lr11xxDriverBase::WriteCommand(uint16_t opcode, uint8_t* data, uint8_t len)
{
ALIGNED uint8_t txData[WORD_PAD(len + 2)] = {0};
ALIGNED uint8_t rxData[WORD_PAD(len + 2)];

    txData[0] = (uint8_t)((opcode & 0xFF00) >> 8);  // opcode high byte
    txData[1] = (uint8_t)(opcode & 0x00FF);         // opcode low byte

    memcpy(&txData[2], data, len);  // copy opcode + data

    WaitOnBusy();
    SpiSelect();
    SpiTransfer(txData, rxData, len + 2);
    SpiDeselect();

    _status1 = rxData[0];
    _status2 = rxData[1];
}


void Lr11xxDriverBase::ReadStatus(uint8_t* data)  // status is unique, so provide a special function
{
ALIGNED uint8_t txData[8] = {0};
ALIGNED uint8_t rxData[8];

    txData[0] = (uint8_t)((LR11XX_CMD_GET_STATUS & 0xFF00) >> 8);  // opcode high byte
    txData[1] = (uint8_t)(LR11XX_CMD_GET_STATUS & 0x00FF);         // opcode low byte
        
    WaitOnBusy();
    SpiSelect();
    SpiTransfer(txData, rxData, 6);  // two byte opcode + four byte irq status
    SpiDeselect();

    _status1 = rxData[0];
    _status2 = rxData[1];

    memcpy(data, &rxData[2], 4);
}


void Lr11xxDriverBase::ReadCommand(uint16_t opcode, uint8_t* data, uint8_t len)
{
ALIGNED uint8_t txData[4] = {0};  // minimum of four bytes
ALIGNED uint8_t rxData[4];
ALIGNED uint8_t buffer[WORD_PAD(len + 1)];

    txData[0] = (uint8_t)((opcode & 0xFF00) >> 8);  // opcode high byte
    txData[1] = (uint8_t)(opcode & 0x00FF);         // opcode low byte

    WaitOnBusy();
    SpiSelect();
    SpiTransfer(txData, rxData, 2);
    SpiDeselect(); 
    WaitOnBusy();
    SpiSelect();
    SpiRead(buffer, len + 1);  // could do transfer again, but need another buffer
    SpiDeselect();

    _status1 = rxData[0];
    _status2 = rxData[1];

    memcpy(data, &buffer[1], len);
}


void Lr11xxDriverBase::WriteBuffer(uint8_t* data, uint8_t len)
{
ALIGNED uint8_t txData[WORD_PAD(len + 2)] = {0};
ALIGNED uint8_t rxData[WORD_PAD(len + 2)];

    txData[0] = (uint8_t)((LR11XX_CMD_WRITE_BUFFER_8 & 0xFF00) >> 8);
    txData[1] = (uint8_t)(LR11XX_CMD_WRITE_BUFFER_8 & 0x00FF);

    memcpy(&txData[2], data, len);

    WaitOnBusy();
    SpiSelect();
    SpiTransfer(txData, rxData, len + 2);
    SpiDeselect();

    _status1 = rxData[0];
    _status2 = rxData[1];
}


void Lr11xxDriverBase::ReadBuffer(uint8_t offset, uint8_t* data, uint8_t len)
{
ALIGNED uint8_t txData[4] = {0};  // minimum of four bytes
ALIGNED uint8_t rxData[4];
ALIGNED uint8_t buffer[WORD_PAD(len + 1)];

    txData[0] = (uint8_t)((LR11XX_CMD_READ_BUFFER_8 & 0xFF00) >> 8);
    txData[1] = (uint8_t)(LR11XX_CMD_READ_BUFFER_8 & 0x00FF);
    txData[2] = offset;
    txData[3] = len;

    WaitOnBusy();
    SpiSelect();
    SpiTransfer(txData, rxData, 4);
    SpiDeselect(); 
    WaitOnBusy();
    SpiSelect();
    SpiRead(buffer, len + 1);  // could do transfer again, but need another buffer
    SpiDeselect();

    _status1 = rxData[0];
    _status2 = rxData[1];

    memcpy(data, &buffer[1], len);
}


// common methods

void Lr11xxDriverBase::GetStatus(uint8_t* Status1, uint8_t* Status2)
{
    ReadStatus(nullptr);
    
    *Status1 = _status1;
    *Status2 = _status2;
}

void Lr11xxDriverBase::GetLastStatus(uint8_t* Status1, uint8_t* Status2)
{
    *Status1 = _status1;
    *Status2 = _status2;
}


void Lr11xxDriverBase::SetStandby(uint8_t StandbyConfig)
{
    WriteCommand(LR11XX_CMD_SET_STANDBY, StandbyConfig);
}


void Lr11xxDriverBase::SetPacketType(uint8_t PacketType)
{
    WriteCommand(LR11XX_CMD_SET_PACKET_TYPE, PacketType);
}


void Lr11xxDriverBase::SetRfFrequency(uint32_t RfFrequency)
{
uint8_t buf[4];

    buf[0] = (uint8_t)((RfFrequency & 0xFF000000) >> 24);
    buf[1] = (uint8_t)((RfFrequency & 0x00FF0000) >> 16);
    buf[2] = (uint8_t)((RfFrequency & 0x0000FF00) >> 8);
    buf[3] = (uint8_t)(RfFrequency & 0x000000FF);

    WriteCommand(LR11XX_CMD_SET_RF_FREQUENCY, buf, 4);
}


void Lr11xxDriverBase::SetDioAsRfSwitch(uint8_t RfSwEnable, uint8_t RfSwStbyCfg, uint8_t RfSwRxCfg, uint8_t RfSwTxCfg, uint8_t TxHPCfg, uint8_t RfSwTxHfCfg, uint8_t RfSwGnssCfg, uint8_t RfSwWifiCfg)
{
uint8_t buf[8];

    buf[0] = RfSwEnable;
    buf[1] = RfSwStbyCfg;
    buf[2] = RfSwRxCfg;
    buf[3] = RfSwTxCfg;
    buf[4] = TxHPCfg;
    buf[5] = RfSwTxHfCfg;
    buf[6] = RfSwGnssCfg;  // per user manual 1.0, removed in latest firmware?
    buf[7] = RfSwWifiCfg;  // per user manual 1.0, removed in latest firmware?

    WriteCommand(LR11XX_CMD_SET_DIO_AS_RF_SWITCH, buf, 8);
}


void Lr11xxDriverBase::SetTcxoMode(uint8_t OutputVoltage, uint32_t Delay)
{
uint8_t buf[4];

    buf[0] = OutputVoltage;
    buf[1] = (uint8_t)((Delay & 0xFF0000) >> 16);
    buf[2] = (uint8_t)((Delay & 0x00FF00) >> 8);
    buf[3] = (uint8_t)(Delay & 0x0000FF);

    WriteCommand(LR11XX_CMD_SET_TCXO_MODE, buf, 4);
}


void Lr11xxDriverBase::SetModulationParams(uint8_t SpreadingFactor, uint8_t Bandwidth, uint8_t CodingRate, uint8_t LowDataRateOptimize)
{
uint8_t buf[4];

    buf[0] = SpreadingFactor;
    buf[1] = Bandwidth;
    buf[2] = CodingRate;
    buf[3] = LowDataRateOptimize;

    WriteCommand(LR11XX_CMD_SET_MODULATION_PARAMS, buf, 4);
}


void Lr11xxDriverBase::SetPacketParams(uint16_t PreambleLength, uint8_t HeaderType, uint8_t PayloadLength, uint8_t Crc, uint8_t InvertIQ)
{
uint8_t buf[6];

    buf[0] = (uint8_t)((PreambleLength & 0xFF00) >> 8);
    buf[1] = (uint8_t)(PreambleLength & 0x00FF);
    buf[2] = HeaderType;
    buf[3] = PayloadLength;
    buf[4] = Crc;
    buf[5] = InvertIQ;

    WriteCommand(LR11XX_CMD_SET_PACKET_PARAMS, buf, 6);
}


void Lr11xxDriverBase::SetDioIrqParams(uint32_t Irq1ToEnable, uint32_t Irq2ToEnable)
{
uint8_t buf[8];

    buf[0] = (uint8_t)((Irq1ToEnable & 0xFF000000) >> 24);
    buf[1] = (uint8_t)((Irq1ToEnable & 0x00FF0000) >> 16);
    buf[2] = (uint8_t)((Irq1ToEnable & 0x0000FF00) >> 8);
    buf[3] = (uint8_t)(Irq1ToEnable & 0x000000FF);
    buf[4] = (uint8_t)((Irq2ToEnable & 0xFF000000) >> 24);
    buf[5] = (uint8_t)((Irq2ToEnable & 0x00FF0000) >> 16);
    buf[6] = (uint8_t)((Irq2ToEnable & 0x0000FF00) >> 8);
    buf[7] = (uint8_t)(Irq2ToEnable & 0x000000FF);

    WriteCommand(LR11XX_CMD_SET_DIO_IRQ_PARAMS, buf, 8);
}


uint32_t Lr11xxDriverBase::GetIrqStatus()
{
uint8_t status[4];

    ReadStatus(status);

    return (uint32_t)((status[0] << 24) | (status[1] << 16) | (status[2] << 8)  | (status[3]));
}


void Lr11xxDriverBase::ClearIrq(uint32_t IrqToClear)
{
uint8_t buf[4];

    buf[0] = (uint8_t)((IrqToClear & 0xFF000000) >> 24);
    buf[1] = (uint8_t)((IrqToClear & 0x00FF0000) >> 16);
    buf[2] = (uint8_t)((IrqToClear & 0x0000FF00) >> 8);
    buf[3] = (uint8_t)(IrqToClear & 0x000000FF);

    WriteCommand(LR11XX_CMD_CLEAR_IRQ, buf, 4);
}


uint32_t Lr11xxDriverBase::GetAndClearIrqStatus(uint32_t IrqToClear)
{
    uint32_t irq_status = GetIrqStatus();

    ClearIrq(IrqToClear);

    return irq_status;
}


// Tx methods

void Lr11xxDriverBase::SetPaConfig(uint8_t PaSel, uint8_t RegPaSupply, uint8_t PaDutyCycle, uint8_t PaHPSel)
{
uint8_t buf[4];

    buf[0] = PaSel;
    buf[1] = RegPaSupply;
    buf[2] = PaDutyCycle;
    buf[3] = PaHPSel;

    WriteCommand(LR11XX_CMD_SET_PA_CONFIG, buf, 4);
}


void Lr11xxDriverBase::SetTxParams(uint8_t Power, uint8_t RampTime)
{
uint8_t buf[2];

    buf[0] = Power;
    buf[1] = RampTime;

    WriteCommand(LR11XX_CMD_SET_TX_PARAMS, buf, 2);
}


void Lr11xxDriverBase::SetTx(uint32_t TxTimeout)
{
uint8_t buf[3];

    // 24 bits timeout with base of 30.5 uS
    // TimeOut duration (us) = 30.5 * timeOut
    // Max timeout is 512 seconds

    buf[0] = (uint8_t)((TxTimeout & 0xFF0000) >> 16);
    buf[1] = (uint8_t)((TxTimeout & 0x00FF00) >> 8);
    buf[2] = (uint8_t) (TxTimeout & 0x0000FF);
    
    WriteCommand(LR11XX_CMD_SET_TX, buf, 3);
}


// Rx methods

void Lr11xxDriverBase::SetRx(uint32_t RxTimeout)
{
uint8_t buf[3];

    // Same units as SetTx

    buf[0] = (uint8_t)((RxTimeout & 0xFF0000) >> 16);
    buf[1] = (uint8_t)((RxTimeout & 0x00FF00) >> 8);
    buf[2] = (uint8_t) (RxTimeout & 0x0000FF);
    
    WriteCommand(LR11XX_CMD_SET_RX, buf, 3);
}


void Lr11xxDriverBase::GetPacketStatus(int16_t* RssiSync, int8_t* Snr)
{
uint8_t status[4];

    ReadCommand(LR11XX_CMD_GET_PACKET_STATUS, status, 4);

    // position 0 is status1, again
    // position 3 is SignalRssiPkt, RSSI of the despreaded LoRa signal

    *RssiSync = -(int16_t)(status[1] / 2);
    *Snr = (int8_t)status[2] / 4; // user manual says (((int8_t)rbuffer[1])+ 2) >> 2 ??
}


void Lr11xxDriverBase::GetRxBufferStatus(uint8_t* rxPayloadLength, uint8_t* rxStartBufferPointer)
{
uint8_t status[3];

    // position 0 is status1, again

    ReadCommand(LR11XX_CMD_GET_RX_BUFFER_STATUS, status, 3);

    *rxPayloadLength = status[1];
    *rxStartBufferPointer = status[2];
}


// auxiliary methods

void Lr11xxDriverBase::SetRegMode(uint8_t RegModeParam)
{
uint8_t buf[1];

    buf[0] = RegModeParam;
    
    WriteCommand(LR11XX_CMD_SET_REG_MODE, buf, 1);
}


void Lr11xxDriverBase::SetRxTxFallbackMode(uint8_t FallbackMode)
{
uint8_t buf[1];

    buf[0] = FallbackMode;
    
    WriteCommand(LR11XX_CMD_SET_RXTX_FALLBACK_MODE, buf, 1);
}


void Lr11xxDriverBase::SetFs(void)
{
    WriteCommand(LR11XX_CMD_SET_FS);
}


void Lr11xxDriverBase::SetRxBoosted(uint8_t RxBoosted)
{
uint8_t buf[1];

    buf[0] = RxBoosted;
    
    WriteCommand(LR11XX_CMD_SET_RX_BOOSTED, buf, 1);
}


void Lr11xxDriverBase::CalibImage(uint8_t Freq1, uint8_t Freq2)
{
uint8_t buf[2];

    buf[0] = Freq1;
    buf[1] = Freq2;
    
    WriteCommand(LR11XX_CMD_CALIB_IMAGE, buf, 2);
}

void Lr11xxDriverBase::CalibImage_mhz(uint16_t Freq1_mhz, uint16_t Freq2_mhz)
{
    // as recommended per the user manual
    uint8_t Freq1 = (Freq1_mhz - 1) / 4;
    uint8_t Freq2 = (Freq2_mhz + 1) / 4;

    CalibImage(Freq1, Freq2);
}

void Lr11xxDriverBase::ClearErrors(void)
{
    WriteCommand(LR11XX_CMD_CLEAR_ERRORS);
}

void Lr11xxDriverBase::EnableSx127xCompatibility(void)
{
    // makes SF6 compatible with SX127x chipsets, user manual 8.3.1
uint8_t buf[12];

    buf[0] = 0x00;
    buf[1] = 0xF2;
    buf[2] = 0x04;
    buf[3] = 0x14;
    buf[4] = 0x00; 
    buf[5] = 0b10000100;
    buf[6] = 0x00;
    buf[7] = 0x00;
    buf[8] = 0x00;
    buf[9] = 0b00000100;
    buf[10] = 0x00;
    buf[11] = 0x00;

    WriteCommand(LR11XX_CMD_WRITE_REG_MEM_MASK_32, buf, 12);
}


// GFSK methods

void Lr11xxDriverBase::SetModulationParamsGFSK(uint32_t br_bps, uint8_t PulseShape, uint8_t Bandwidth, uint32_t Fdev_hz)
{
uint8_t buf[10];

    buf[0] = (uint8_t)((br_bps & 0xFF000000) >> 24);
    buf[1] = (uint8_t)((br_bps & 0x00FF0000) >> 16);
    buf[2] = (uint8_t)((br_bps & 0x0000FF00) >> 8);
    buf[3] = (uint8_t)(br_bps & 0x000000FF);

    buf[4] = PulseShape;
    buf[5] = Bandwidth;

    buf[6] = (uint8_t)((Fdev_hz & 0xFF000000) >> 24);
    buf[7] = (uint8_t)((Fdev_hz & 0x00FF0000) >> 16);
    buf[8] = (uint8_t)((Fdev_hz & 0x0000FF00) >> 8);
    buf[9] = (uint8_t)(Fdev_hz & 0x000000FF);

    WriteCommand(LR11XX_CMD_SET_MODULATION_PARAMS, buf, 10);
}


void Lr11xxDriverBase::SetPacketParamsGFSK(uint16_t PreambleLength, uint8_t PreambleDetectorLength, uint8_t SyncWordLength, uint8_t AddrComp, uint8_t PacketType, uint8_t PayloadLength, uint8_t CRCType, uint8_t Whitening)
{
uint8_t buf[9];

    buf[0] = (uint8_t)((PreambleLength & 0xFF00) >> 8);
    buf[1] = (uint8_t)(PreambleLength & 0x00FF);
    buf[2] = PreambleDetectorLength;
    buf[3] = SyncWordLength;
    buf[4] = AddrComp;
    buf[5] = PacketType;
    buf[6] = PayloadLength;
    buf[7] = CRCType;
    buf[8] = Whitening;

    WriteCommand(LR11XX_CMD_SET_PACKET_PARAMS, buf, 9);
}

void Lr11xxDriverBase::SetSyncWordGFSK(uint16_t SyncWord)
{
uint8_t buf[8] = {0};  // sync word can be 64 bits

    buf[0] = (uint8_t)((SyncWord & 0xFF00) >> 8);
    buf[1] = (uint8_t)(SyncWord & 0x00FF);

    WriteCommand(LR11XX_CMD_SET_GFSK_SYNC_WORD, buf, 8);
}

void Lr11xxDriverBase::GetPacketStatusGFSK(int16_t* RssiSync)
{
uint8_t status[5];

    // position 0 is status1, again

    ReadCommand(LR11XX_CMD_GET_PACKET_STATUS, status, 3);

    *RssiSync = -(int16_t)(status[1] / 2);
}


// other methods

void Lr11xxDriverBase::GetVersion(uint8_t* HwVersion, uint8_t* UseCase, uint8_t* FwMajor, uint8_t* FwMinor)
{
uint8_t version[5];

    // position 0 is status1, again

    ReadCommand(LR11XX_CMD_GET_VERSION, version, 5);

    *HwVersion = version[1];
    *UseCase = version[2];
    *FwMajor = version[3];
    *FwMinor = version[4];
}