//*******************************************************
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// LR11XX standard interface
//*******************************************************

#include "lr11xx.h"

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
    WaitOnBusy();
    SpiSelect();
    SpiTransfer(opcode, &_status);
    if (len > 0) SpiWrite(data, len);
    SpiDeselect();
    WaitOnBusy(); // use busy instead of hardcoded delay
}


void Lr11xxDriverBase::ReadCommand(uint16_t opcode, uint8_t* data, uint8_t len)
{
    WaitOnBusy();
    SpiSelect();
    SpiTransfer(opcode, &_status);
    WaitOnBusy(); // replaces NOP
    SpiRead(data, len);
    SpiDeselect();
    WaitOnBusy();
}


void Lr11xxDriverBase::WriteBuffer(uint8_t offset, uint8_t* data, uint8_t len)
{
    WaitOnBusy();
    SpiSelect();
    SpiTransfer(LR11XX_CMD_WRITE_BUFFER_8, &_status);
    SpiWrite(offset);
    SpiWrite(data, len);
    SpiDeselect();
    WaitOnBusy();
}


void Lr11xxDriverBase::ReadBuffer(uint8_t offset, uint8_t* data, uint8_t len)
{
    WaitOnBusy();
    SpiSelect();
    SpiTransfer(LR11XX_CMD_READ_BUFFER_8, &_status);
    SpiWrite(offset);
    WaitOnBusy(); // replaces NOP
    SpiRead(data, len);
    SpiDeselect();
    WaitOnBusy(); // use busy instead of hardcoded delay
}


// common methods

void Lr11xxDriverBase::GetStatus(uint8_t* Status1, uint8_t* Status2, uint32_t* IrqStatus)
{
    uint8_t status[4];

    ReadCommand(LR11XX_CMD_GET_STATUS, status, 4);

    *Status1 = (_status >> 8) & 0xFF;
    *Status2 = _status & 0xFF;

    *IrqStatus = (status[0] << 24) | (status[1] << 16) | (status[2] << 8)  | (status[3]);
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


void Lr11xxDriverBase::SetDioAsRfSwitch(uint8_t RfSwEnable, uint8_t RfSwStbyCfg, uint8_t RfSwRxCfg, uint8_t RfSwTxCfg, uint8_t TxHPCfg, uint8_t RfSwTxHfCfg)
{
uint8_t buf[8];

    buf[0] = RfSwEnable;
    buf[1] = RfSwStbyCfg;
    buf[2] = RfSwRxCfg;
    buf[3] = RfSwTxCfg;
    buf[4] = TxHPCfg;
    buf[5] = RfSwTxHfCfg;
    buf[6] = 0;  // reserved for future use
    buf[7] = 0;  // reserved for future use

    WriteCommand(LR11XX_CMD_SET_DIO_AS_RF_SWITCH, buf, 8);
}


void Lr11xxDriverBase::SetTcxoMode(uint8_t OutputVoltage, uint32_t Delay)
{
uint8_t buf[4];

    buf[0] = OutputVoltage;
    buf[0] = (uint8_t)((Delay & 0xFF0000) >> 16);
    buf[1] = (uint8_t)((Delay & 0x00FF00) >> 8);
    buf[2] = (uint8_t)(Delay & 0x0000FF);

    WriteCommand(LR11XX_CMD_SET_TCXO_MODE, buf, 4);
}


void Lr11xxDriverBase::SetModulationParams(uint8_t SpreadingFactor, uint8_t Bandwidth, uint8_t CodingRate, uint8_t LowDataRateOptimize)
{
uint8_t buf[4];

    buf[0] = SpreadingFactor;
    buf[0] = Bandwidth;
    buf[1] = CodingRate;
    buf[2] = LowDataRateOptimize;

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
uint8_t status[4];
uint32_t irq_status;

    ReadCommand(LR11XX_CMD_GET_STATUS, status, 4);

    irq_status = (status[0] << 24) | (status[1] << 16) | (status[2] << 8)  | (status[3]);

    ClearIrq(IrqToClear);

    return irq_status;
}

