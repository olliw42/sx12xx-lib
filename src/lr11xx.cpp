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

void SetDioAsRfSwitch(uint8_t RfSwEnable, uint8_t RfSwStbyCfg, uint8_t RfSwRxCfg, uint8_t RfSwTxCfg, uint8_t TxHPCfg, uint8_t RfSwTxHfCfg);
void SetTcxoMode(uint8_t OutputVoltage, uint32_t delay_us);
// void SetBufferBaseAddress(uint8_t txBaseAdress, uint8_t rxBaseAdress); // no opcode in user manual?
void SetModulationParams(uint8_t SpreadingFactor, uint8_t Bandwidth, uint8_t CodingRate); // implied to be LoRa
void SetPacketParams(uint16_t PreambleLength, uint8_t HeaderType, uint8_t PayloadLength, uint8_t Crc, uint8_t InvertIQ); // implied to be LoRa

void SetDioIrqParams(uint32_t Irq1ToEnable, uint32_t Irq2ToEnable); // TODO - which IRQs needed? Table 4-2
void ClearIrq(uint32_t IrqToClear); // IrqToClear mask is identical to IrqToEnable assignment
uint32_t GetAndClearIrqStatus(uint32_t IrqToClear);  // No more GetIrqStatus, use GetStatus

