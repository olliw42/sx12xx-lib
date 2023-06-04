//*******************************************************
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// SX128x standard interface
//*******************************************************

#include "sx128x.h"

// semtech driver says:
// "observed BUSY time for Write* calls are 12-20uS after NSS de-assert
//  and state transitions require extra time depending on prior state"


// low level

void Sx128xDriverBase::WriteCommand(uint8_t opcode, uint8_t* data, uint8_t len)
{
uint8_t in_buf[SX128X_SPI_BUF_SIZE];

    WaitOnBusy();
    SpiSelect();
    SpiTransfer(opcode, &_status);
    if (len > 0) SpiTransfer(data, in_buf, len);
    SpiDeselect();
    SetDelay(12); // semtech driver says 12 us
}


void Sx128xDriverBase::ReadCommand(uint8_t opcode, uint8_t* data, uint8_t len)
{
uint8_t out_buf[SX128X_SPI_BUF_SIZE];

    for (uint8_t i = 0; i < len; i++) out_buf[i] = 0; // NOP

    WaitOnBusy();
    SpiSelect();
    SpiTransfer(opcode, &_status);
    SpiTransfer(0); // NOP
    SpiTransfer(out_buf, data, len);
    SpiDeselect();
    // no delay according to semtech driver
}


void Sx128xDriverBase::WriteRegister(uint16_t adr, uint8_t* data, uint8_t len)
{
uint8_t in_buf[SX128X_SPI_BUF_SIZE];

    WaitOnBusy();
    SpiSelect();
    SpiTransfer(SX1280_CMD_WRITE_REGISTER, &_status);
    SpiTransfer((adr & 0xFF00) >> 8);
    SpiTransfer(adr & 0x00FF);
    SpiTransfer(data, in_buf, len);
    SpiDeselect();
    SetDelay(12); // semtech driver says 12 us
}


void Sx128xDriverBase::ReadRegister(uint16_t adr, uint8_t* data, uint8_t len)
{
uint8_t out_buf[SX128X_SPI_BUF_SIZE];

    for (uint8_t i = 0; i < len; i++) out_buf[i] = 0; // NOP

    WaitOnBusy();
    SpiSelect();
    SpiTransfer(SX1280_CMD_READ_REGISTER, &_status);
    SpiTransfer((adr & 0xFF00) >> 8);
    SpiTransfer(adr & 0x00FF);
    SpiTransfer(0); // NOP
    SpiTransfer(out_buf, data, len);
    SpiDeselect();
    // no delay according to semtech driver
}


void Sx128xDriverBase::WriteBuffer(uint8_t offset, uint8_t* data, uint8_t len)
{
uint8_t in_buf[SX128X_SPI_BUF_SIZE];

    WaitOnBusy();
    SpiSelect();
    SpiTransfer(SX1280_CMD_WRITE_BUFFER, &_status);
    SpiTransfer(offset);
    SpiTransfer(data, in_buf, len);
    SpiDeselect();
    SetDelay(12); // semtech driver says 12 us
}


void Sx128xDriverBase::ReadBuffer(uint8_t offset, uint8_t* data, uint8_t len)
{
uint8_t out_buf[SX128X_SPI_BUF_SIZE];

    for (uint8_t i = 0; i < len; i++) out_buf[i] = 0; // NOP

    WaitOnBusy();
    SpiSelect();
    SpiTransfer(SX1280_CMD_READ_BUFFER, &_status);
    SpiTransfer(offset);
    SpiTransfer(0); // NOP
    SpiTransfer(out_buf, data, len);
    SpiDeselect();
    // no delay according to semtech driver
}


// common

uint8_t Sx128xDriverBase::GetStatus(void)
{
    WriteCommand(SX1280_CMD_GET_STATUS); // yes, this is correct !
    return _status;
}


void Sx128xDriverBase::SetStandby(uint8_t StandbyConfig)
{
    WriteCommand(SX1280_CMD_SET_STANDBY, StandbyConfig);
    switch (StandbyConfig) {
    case SX1280_STDBY_CONFIG_STDBY_XOSC:
        SetDelay(50); // semtech driver says 50 us
        break;
    default:
        SetDelay(1500); // semtech driver says 1500 us
    }
}


void Sx128xDriverBase::SetPacketType(uint8_t PacketType)
{
    WriteCommand(SX1280_CMD_SET_PACKET_TYPE, PacketType);
}


void Sx128xDriverBase::SetRfFrequency(uint32_t RfFrequency) // 24 bits only
{
uint8_t buf[3];

    buf[0] = (uint8_t)((RfFrequency & 0xFF0000) >> 16);
    buf[1] = (uint8_t)((RfFrequency & 0x00FF00) >> 8);
    buf[2] = (uint8_t)(RfFrequency & 0x0000FF);

    WriteCommand(SX1280_CMD_SET_RF_FREQUENCY, buf, 3);
}


void Sx128xDriverBase::SetBufferBaseAddress(uint8_t txBaseAdress, uint8_t rxBaseAdress)
{
uint8_t buf[2];

    buf[0] = txBaseAdress;
    buf[1] = rxBaseAdress;

    WriteCommand(SX1280_CMD_SET_BUFFER_BASEADDRESS, buf, 2);
}


void Sx128xDriverBase::SetModulationParams(uint8_t SpreadingFactor, uint8_t Bandwidth, uint8_t CodingRate)
{
uint8_t buf[3];

    buf[0] = SpreadingFactor;
    buf[1] = Bandwidth;
    buf[2] = CodingRate;

    WriteCommand(SX1280_CMD_SET_MODULATION_PARAMS, buf, 3);

    // datasheet after table 14-47, p. 131

    switch (SpreadingFactor) {
    case SX1280_LORA_SF5:
    case SX1280_LORA_SF6:
        WriteRegister(SX1280_REG_SFAdditionalConfiguration, 0x1E);
        break;
    case SX1280_LORA_SF7:
    case SX1280_LORA_SF8:
        WriteRegister(SX1280_REG_SFAdditionalConfiguration, 0x37);
        break;
    default:
        WriteRegister(SX1280_REG_SFAdditionalConfiguration, 0x32);
    }

    WriteRegister(SX1280_REG_FrequencyErrorCorrection, 0x1);
}


void Sx128xDriverBase::SetPacketParams(uint8_t PreambleLength, uint8_t HeaderType, uint8_t PayloadLength, uint8_t Crc, uint8_t InvertIQ)
{
uint8_t buf[7];
// param 6 & 7 are not used, send 0

    buf[0] = PreambleLength;
    buf[1] = HeaderType;
    buf[2] = PayloadLength;
    buf[3] = Crc;
    buf[4] = InvertIQ;
    buf[5] = 0;
    buf[6] = 0;

    WriteCommand(SX1280_CMD_SET_PACKET_PARAMS, buf, 7);
}


void Sx128xDriverBase::SetDioIrqParams(uint16_t IrqMask, uint16_t Dio1Mask, uint16_t Dio2Mask, uint16_t Dio3Mask)
{
uint8_t buf[8];

    buf[0] = (uint8_t)((IrqMask & 0xFF00) >> 8);
    buf[1] = (uint8_t)(IrqMask & 0x00FF);

    buf[2] = (uint8_t)((Dio1Mask & 0xFF00) >> 8);
    buf[3] = (uint8_t)(Dio1Mask & 0x00FF);

    buf[4] = (uint8_t)((Dio2Mask & 0xFF00) >> 8);
    buf[5] = (uint8_t)(Dio2Mask & 0x00FF);

    buf[6] = (uint8_t)((Dio3Mask & 0xFF00) >> 8);
    buf[7] = (uint8_t)(Dio3Mask & 0x00FF);

    WriteCommand(SX1280_CMD_SET_DIOIRQ_PARAMS, buf, 8);
}


uint16_t Sx128xDriverBase::GetIrqStatus(void)
{
uint8_t status[2];

    ReadCommand(SX1280_CMD_GET_IRQ_STATUS, status, 2);
    return ((uint16_t)status[0] << 8) + status[1];
}


void Sx128xDriverBase::ClearIrqStatus(uint16_t IrqMask)
{
uint8_t buf[2];

    buf[0] = (uint8_t)((IrqMask & 0xFF00) >> 8);
    buf[1] = (uint8_t)(IrqMask & 0x00FF);

    WriteCommand(SX1280_CMD_CLR_IRQ_STATUS, buf, 2);
}


uint16_t Sx128xDriverBase::GetAndClearIrqStatus(uint16_t IrqMask)
{
    uint16_t irq_status = GetIrqStatus();

    ClearIrqStatus(IrqMask);

    return irq_status;
}


// Tx

void Sx128xDriverBase::SetTxParams(uint8_t Power, uint8_t RampTime)
{
uint8_t buf[2];

    if (Power > 31) Power = 31;

    buf[0] = Power;
    buf[1] = RampTime;

    WriteCommand(SX1280_CMD_SET_TX_PARAMS, buf, 2);
}


void Sx128xDriverBase::SetTx(uint8_t PeriodBase, uint16_t PeriodBaseCount)
{
uint8_t buf[3];

    buf[0] = PeriodBase;
    buf[1] = (uint8_t)((PeriodBaseCount & 0xFF00) >> 8);
    buf[2] = (uint8_t)(PeriodBaseCount & 0x00FF);

    WriteCommand(SX1280_CMD_SET_TX, buf, 3);
    SetDelay(100); // semtech driver says 100 us
}


// Rx

void Sx128xDriverBase::SetRx(uint8_t PeriodBase, uint16_t PeriodBaseCount)
{
uint8_t buf[3];

    buf[0] = PeriodBase;
    buf[1] = (uint8_t)((PeriodBaseCount & 0xFF00) >> 8);
    buf[2] = (uint8_t)(PeriodBaseCount & 0x00FF);

    WriteCommand(SX1280_CMD_SET_RX, buf, 3);
    SetDelay(100); // semtech driver says 100 us
}


void Sx128xDriverBase::GetPacketStatus(int16_t* RssiSync, int8_t* Snr)
{
uint8_t status[5];

// param 3 & 4 & 5 are not used, must read them nevertheless

    ReadCommand(SX1280_CMD_GET_PACKET_STATUS, status, 5);

    *RssiSync = -(int16_t)(status[0] / 2);
    *Snr = (int8_t)status[1] / 4;
    if (*Snr < 0) {
        *RssiSync += *Snr;
    }

    // datasheet p. 93, table 11-66: If the SNR ≤ 0, RSSI_{packet, real} = RSSI_{packet,measured} – SNR_{measured}
    // datasheet p.134, point 4: Actual SNR in dB =SnrPkt/4 , noting that only negative values should be used.
}


void Sx128xDriverBase::GetRxBufferStatus(uint8_t* rxPayloadLength, uint8_t* rxStartBufferPointer)
{
uint8_t status[2];

    ReadCommand(SX1280_CMD_GET_RX_BUFFER_STATUS, status, 2);

    *rxPayloadLength = status[0];
    *rxStartBufferPointer = status[1];
}


// auxiliary

void Sx128xDriverBase::SetRegulatorMode(uint8_t RegModeParam)
{
    WriteCommand(SX1280_CMD_SET_REGULATOR_MODE, RegModeParam);
}


void Sx128xDriverBase::SetAutoFs(uint8_t flag)
{
    WriteCommand(SX1280_CMD_SET_AUTOFS, flag);
}


void Sx128xDriverBase::SetFs(void)
{
    WriteCommand(SX1280_CMD_SET_FS);
    SetDelay(70); // semtech driver says 70 us
}


void Sx128xDriverBase::SetLnaGainMode(uint8_t LnaGainMode)
{
uint8_t reg_rxgain;

    reg_rxgain = ReadRegister(SX1280_REG_RxGain);

    switch (LnaGainMode) {
    case SX1280_LNAGAIN_MODE_HIGH_SENSITIVITY:
        WriteRegister(SX1280_REG_RxGain, (reg_rxgain | 0xC0));
        break;
    default: // SX1280_LNAGAIN_MODE_LOW_POWER
        WriteRegister(SX1280_REG_RxGain, (reg_rxgain & 0x3F));
    };
}


uint16_t Sx128xDriverBase::GetFirmwareRev(void)
{
    return ((uint16_t)ReadRegister(SX1280_REG_FIRMWARE_VERSION_MSB) << 8) |
            ReadRegister(SX1280_REG_FIRMWARE_VERSION_MSB + 1);
}


// experimental
// - datasheet below table 14-54, p. 133
//   sync word XY, write X in 0x944 in position [7:4] wo modifying [3:0]
//                 write Y in 0x945 in position [7:4] wo modifying [3:0]
// - https://github.com/jgromes/RadioLib/commit/8be419f0073eb8499a678710e7f9c7f10e7de511
//   these set the low nibble to 0x04, but I found it can change
//   e.g. after reset it is 0x14,0x24 and after config 0x16,0x24
//   so I think their approach is not totally correct, but the datasheet is
void Sx128xDriverBase::SetSyncWord(uint8_t SyncWord)
{
uint8_t s1, s2;
uint8_t buf[2];

    s1 = ReadRegister(SX1280_REG_LoRaSynchWord);
    s2 = ReadRegister(SX1280_REG_LoRaSynchWord + 1);

    buf[0] = (SyncWord & 0xF0) | (s1 & 0x0F);
    buf[1] = ((SyncWord & 0x0F) << 4) | (s2 & 0x0F);

    WriteRegister(SX1280_REG_LoRaSynchWord, buf, 2);
}


uint32_t Sx128xDriverBase::GetFrequencyErrorIndicator(void)
{
uint8_t buf[3];
uint32_t fei;

    buf[0] = ReadRegister(SX1280_REG_FEI);
    buf[1] = ReadRegister(SX1280_REG_FEI + 1);
    buf[2] = ReadRegister(SX1280_REG_FEI + 2);

    fei = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | (uint32_t)buf[2];
    fei &= (uint32_t)0x0FFFFF;

    return fei;
}


// FLRC

void Sx128xDriverBase::SetModulationParamsFLRC(uint8_t Bandwidth, uint8_t CodingRate, uint8_t Bt)
{
uint8_t buf[3];

    buf[0] = Bandwidth;
    buf[1] = CodingRate;
    buf[2] = Bt;

    WriteCommand(SX1280_CMD_SET_MODULATION_PARAMS, buf, 3);
}


void Sx128xDriverBase::SetPacketParamsFLRC(uint8_t AGCPreambleLength, uint8_t SyncWordLength, uint8_t SyncWordMatch, uint8_t PacketType, uint8_t PayloadLength, uint8_t CrcLength, 
    uint16_t CrcSeed, uint32_t SyncWord, uint8_t CodingRate)
{
uint8_t buf[7];

    buf[0] = AGCPreambleLength;                     // AGC Preamble Length
    buf[1] = SyncWordLength;                        // Sync Word Length
    buf[2] = SyncWordMatch;                         // Sync Word Combination
    buf[3] = PacketType;                            // Packet Type
    buf[4] = PayloadLength;                         // Payload Length
    buf[5] = CrcLength;                             // CRC Length
    buf[6] = 0x08;                                  // Whitening (always disabled for FLRC) table 14-41, p. 125

    WriteCommand(SX1280_CMD_SET_PACKET_PARAMS, buf, 7);

    // CRC Seed
    buf[0] = (uint8_t)(CrcSeed >> 8);
    buf[1] = (uint8_t)CrcSeed;
    
    WriteRegister(SX1280_REG_CRCInitialValue, buf, 2);

    // Set Sync Word 1
    buf[0] = (uint8_t)(SyncWord >> 24);
    buf[1] = (uint8_t)(SyncWord >> 16);
    buf[2] = (uint8_t)(SyncWord >> 8);
    buf[3] = (uint8_t)SyncWord;

    // Borrowed from ELRS: https://github.com/ExpressLRS/ExpressLRS/blob/master/src/lib/SX1280Driver/SX1280.cpp#L382
    // DS_SX1280-1_V3.2.pdf - 16.4 FLRC Modem: Increased PER in FLRC Packets with Synch Word
    if (((CodingRate == SX1280_FLRC_CR_1_2) || (CodingRate == SX1280_FLRC_CR_3_4)) &&
        ((buf[0] == 0x8C && buf[1] == 0x38) || (buf[0] == 0x63 && buf[1] == 0x0E)))
    {
        uint8_t temp = buf[0];
        buf[0] = buf[1];
        buf[1] = temp;
        // For SX1280_FLRC_CR_3_4 the datasheet also says
        // "In addition to this the two LSB values XX XX must not be in the range 0x0000 to 0x3EFF"
        if (CodingRate == SX1280_FLRC_CR_3_4 && buf[3] <= 0x3E)
            buf[3] |= 0x80; // 0x80 or 0x40 would work
    }

    WriteRegister(SX1280_REG_FLRC_SyncWordAddress1, buf, 4);
}

