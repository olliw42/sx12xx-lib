//*******************************************************
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// LR20XX standard interface
//*******************************************************
// contributed by JLP, OlliW42
//*******************************************************

#include "lr20xx.h"

// spi methods

void Lr20xxDriverBase::SpiRead(uint8_t* datain, uint8_t len)
{
uint8_t dummy = 0; // NOP

    while (len) {
        SpiTransfer(dummy, datain);
        datain++;
        len--;
    }
}

void Lr20xxDriverBase::SpiWrite(uint8_t* dataout, uint8_t len)
{
uint8_t dummy;

    while (len) {
        SpiTransfer(*dataout, &dummy);
        dataout++;
        len--;
    }
}


// low level methods

void Lr20xxDriverBase::WriteCommand(uint16_t opcode, uint8_t* data, uint8_t len)
{
    WaitOnBusy();
    SpiSelect();
    SpiTransfer((uint8_t)((opcode & 0xFF00) >> 8), &_status1);
    SpiTransfer((uint8_t)(opcode & 0x00FF), &_status2);
    // note: the next 4 bytes give the uint32_t IrqStatus
    // we could read them in, but let's ignore them, since len is not guaranteed to be >= 4
    if (len > 0) SpiWrite(data, len);
    SpiDeselect();
}

void Lr20xxDriverBase::ReadCommand(uint16_t opcode, uint8_t* data, uint8_t len)
{
    WaitOnBusy();
    SpiSelect();
    SpiTransfer((uint8_t)((opcode & 0xFF00) >> 8), &_status1);
    SpiTransfer((uint8_t)(opcode & 0x00FF), &_status2);
    if (opcode != LR20XX_CMD_GET_STATUS) { // not needed for get status
        SpiDeselect();
        WaitOnBusy();
        SpiSelect();
    }
    SpiRead(data, len);
    SpiDeselect();
    if (len >= 2) { // the first two bytes of the response seem to always be the latest status
        _status1 = data[0];
        _status2 = data[1];
    }
}

void Lr20xxDriverBase::WriteRadioTxFifo(uint8_t* data, uint8_t len)
{
    WriteCommand(LR20XX_CMD_WRITE_RADIO_TX_FIFO, data, len);
}

void Lr20xxDriverBase::ReadRadioRxFifo(uint8_t* data, uint8_t len)
{
    WaitOnBusy();
    SpiSelect();
    SpiTransfer((uint8_t)((LR20XX_CMD_READ_RADIO_RX_FIFO & 0xFF00) >> 8), &_status1);
    SpiTransfer((uint8_t)(LR20XX_CMD_READ_RADIO_RX_FIFO & 0x00FF), &_status2);
    SpiRead(data, len);
    SpiDeselect();
}

void Lr20xxDriverBase::WriteRegMemMask32(uint32_t addr, uint32_t mask, uint32_t data)
{
uint8_t buf[11];

    WaitOnBusy();
    SpiSelect();
    SpiTransfer((uint8_t)((LR20XX_CMD_WRITE_REG_MEM_MASK_32 & 0xFF00) >> 8), &_status1);
    SpiTransfer((uint8_t)(LR20XX_CMD_WRITE_REG_MEM_MASK_32 & 0x00FF), &_status2);
    buf[0] = addr >> 16;
    buf[1] = addr >> 8;
    buf[2] = addr;
    buf[3] = mask >> 24;
    buf[4] = mask >> 16;
    buf[5] = mask >> 8;
    buf[6] = mask;
    buf[7] = data >> 24;
    buf[8] = data >> 16;
    buf[9] = data >> 8;
    buf[10] = data;
    SpiWrite(buf, 11);
    SpiDeselect();
}

void Lr20xxDriverBase::WriteRegMem32(uint32_t addr, uint32_t* data, uint8_t len) // len is number of uint32_t, must not exceed 32
{
uint8_t buf[4];

    WaitOnBusy();
    SpiSelect();
    SpiTransfer((uint8_t)((LR20XX_CMD_WRITE_REG_MEM_32 & 0xFF00) >> 8), &_status1);
    SpiTransfer((uint8_t)(LR20XX_CMD_WRITE_REG_MEM_32 & 0x00FF), &_status2);
    buf[0] = addr >> 16;
    buf[1] = addr >> 8;
    buf[2] = addr;
    SpiWrite(buf, 3);
    for (uint8_t i = 0; i < len; i++) {
        buf[0] = data[i] >> 24;
        buf[1] = data[i] >> 16;
        buf[2] = data[i] >> 8;
        buf[3] = data[i];
        SpiWrite(buf, 4);
    }
    SpiDeselect();
}

void Lr20xxDriverBase::ReadRegMem32(uint32_t addr, uint32_t* data, uint8_t len) // len is number of uint32_t, must not exceed 32
{
uint8_t buf[4];

    WaitOnBusy();
    SpiSelect();
    SpiTransfer((uint8_t)((LR20XX_CMD_READ_REG_MEM_32 & 0xFF00) >> 8), &_status1);
    SpiTransfer((uint8_t)(LR20XX_CMD_READ_REG_MEM_32 & 0x00FF), &_status2);
    buf[0] = addr >> 16;
    buf[1] = addr >> 8;
    buf[2] = addr;
    buf[3] = len;
    SpiWrite(buf, 4);
    SpiDeselect();

    WaitOnBusy();
    SpiSelect();
    SpiRead(buf, 2); // the first two bytes of the response are the latest status
    _status1 = buf[0];
    _status2 = buf[1];
    for (uint8_t i = 0; i < len; i++) {
        SpiRead(buf, 4);
        data[i] = ((uint32_t)buf[0] << 24) + ((uint32_t)buf[1] << 16) + ((uint32_t)buf[2] << 8) + buf[3];
    }
    SpiDeselect();
}


// System Configuration Commands

// would we also want IrqStatus ??
void Lr20xxDriverBase::GetStatus(uint8_t* Status1, uint8_t* Status2)
{
    WriteCommand(LR20XX_CMD_GET_STATUS); // doesn't need a response, so don't need to use ReadCommand

    *Status1 = _status1;
    *Status2 = _status2;
}

uint16_t Lr20xxDriverBase::GetStatus(void)
{
    WriteCommand(LR20XX_CMD_GET_STATUS); // doesn't need a response, so don't need to use ReadCommand

    return ((uint16_t)_status1 << 8) + _status2;
}

uint16_t Lr20xxDriverBase::GetLastStatus(void)
{
    return ((uint16_t)_status1 << 8) + _status2;
}

uint8_t Lr20xxDriverBase::GetLastStatusCmd(void)
{
    return (_status1 >> 1) & 0x07;
}

uint8_t Lr20xxDriverBase::GetLastStatusChipMode(void)
{
    return _status2 & 0x07;
}

void Lr20xxDriverBase::GetVersion(uint8_t* FwMajor, uint8_t* FwMinor)
{
uint8_t buf[4];

    ReadCommand(LR20XX_CMD_GET_VERSION, buf, 4);

    *FwMajor = buf[2];
    *FwMinor = buf[3];
}

uint16_t Lr20xxDriverBase::GetErrors(void)
{
uint8_t buf[4];

    ReadCommand(LR20XX_CMD_GET_ERRORS, buf, 4);

    return ((uint16_t)buf[2] << 8) + buf[3];
}

void Lr20xxDriverBase::ClearErrors(void)
{
    WriteCommand(LR20XX_CMD_CLEAR_ERRORS);
}

void Lr20xxDriverBase::SetDioFunction(uint8_t Dio, uint8_t Func, uint8_t PullDrive)
{
uint8_t buf[2];

    buf[0] = Dio; // allowed values are 5 - 11
    buf[1] = ((Func & 0x0F) << 4) + (PullDrive & 0xF);

    WriteCommand(LR20XX_CMD_SET_DIO_FUNCTION, buf, 2);
}

void Lr20xxDriverBase::SetDioRfSwitchConfig(uint8_t Dio, uint8_t Config)
{
uint8_t buf[2];

    buf[0] = Dio; // allowed values are 5 - 11
    buf[1] = (Config & 0x1F);

    WriteCommand(LR20XX_CMD_SET_DIO_RF_SWITCH_CONFIG, buf, 2);
}

void Lr20xxDriverBase::SetDioIrqConfig(uint8_t Dio, uint32_t Irq)
{
uint8_t buf[5];

    buf[0] = Dio;
    buf[1] = (uint8_t)((Irq & 0xFF000000) >> 24);
    buf[2] = (uint8_t)((Irq & 0x00FF0000) >> 16);
    buf[3] = (uint8_t)((Irq & 0x0000FF00) >> 8);
    buf[4] = (uint8_t) (Irq & 0x000000FF);

    WriteCommand(LR20XX_CMD_SET_DIO_IRQ_CONFIG, buf, 5);
}

void Lr20xxDriverBase::ClearIrq(uint32_t IrqsToClear)
{
uint8_t buf[4];

    buf[0] = (uint8_t)((IrqsToClear & 0xFF000000) >> 24);
    buf[1] = (uint8_t)((IrqsToClear & 0x00FF0000) >> 16);
    buf[2] = (uint8_t)((IrqsToClear & 0x0000FF00) >> 8);
    buf[3] = (uint8_t) (IrqsToClear & 0x000000FF);

    WriteCommand(LR20XX_CMD_CLEAR_IRQ, buf, 4);
}

uint32_t Lr20xxDriverBase::GetAndClearIrqStatus(void)
{
uint8_t buf[6];

    ReadCommand(LR20XX_CMD_GET_AND_CLEAR_IRQ_STATUS, buf, 6);

    return ((uint32_t)buf[2] << 24) + ((uint32_t)buf[3] << 16) + ((uint32_t)buf[4] << 8) + (uint32_t)buf[5];
}

uint16_t Lr20xxDriverBase::GetRxFifoLevel(void)
{
uint8_t buf[4];

    ReadCommand(LR20XX_CMD_GET_RX_FIFO_LEVEL, buf, 4);

    return ((uint16_t)buf[2] << 8) + (uint16_t)buf[3];
}

uint16_t Lr20xxDriverBase::GetTxFifoLevel(void)
{
uint8_t buf[4];

    ReadCommand(LR20XX_CMD_GET_TX_FIFO_LEVEL, buf, 4);

    return ((uint16_t)buf[2] << 8) + (uint16_t)buf[3];
}

void Lr20xxDriverBase::ClearRxFifo(void)
{
    WriteCommand(LR20XX_CMD_CLEAR_RX_FIFO);
}

void Lr20xxDriverBase::ClearTxFifo(void)
{
    WriteCommand(LR20XX_CMD_CLEAR_TX_FIFO);
}

void Lr20xxDriverBase::SetTcxoMode(uint8_t Tune, uint32_t StartTime)
{
uint8_t buf[5];

    buf[0] = Tune;
    buf[1] = (uint8_t)((StartTime & 0xFF000000) >> 24);
    buf[2] = (uint8_t)((StartTime & 0x00FF0000) >> 16);
    buf[3] = (uint8_t)((StartTime & 0x0000FF00) >> 8);
    buf[4] = (uint8_t) (StartTime & 0x000000FF);

    WriteCommand(LR20XX_CMD_SET_TCXO_MODE, buf, 5);
}

void Lr20xxDriverBase::SetRegMode(uint8_t SimoUsage)
{
uint8_t buf[1];

    buf[0] = SimoUsage;

    WriteCommand(LR20XX_CMD_SET_REG_MODE, buf, 1);
}

void Lr20xxDriverBase::Calibrate(uint8_t BlocksToCalibrate)
{
uint8_t buf[1];

    buf[0] = BlocksToCalibrate;

    WriteCommand(LR20XX_CMD_CALIBRATE, buf, 1);
}

void Lr20xxDriverBase::CalibFE(uint16_t Freq1, uint16_t Freq2, uint16_t Freq3)
{
uint8_t buf[6];

    buf[0] = (uint8_t)((Freq1 & 0xFF00) >> 8);
    buf[1] = (uint8_t) (Freq1 & 0x00FF);
    buf[2] = (uint8_t)((Freq2 & 0xFF00) >> 8);
    buf[3] = (uint8_t) (Freq2 & 0x00FF);
    buf[4] = (uint8_t)((Freq3 & 0xFF00) >> 8);
    buf[5] = (uint8_t) (Freq3 & 0x00FF);

    WriteCommand(LR20XX_CMD_CALIB_FE, buf, 6);
}

void Lr20xxDriverBase::CalibFE(void)
{
    WriteCommand(LR20XX_CMD_CALIB_FE);
}

void Lr20xxDriverBase::SetStandby(uint8_t StandbyMode)
{
uint8_t buf[1];

    buf[0] = StandbyMode;

    WriteCommand(LR20XX_CMD_SET_STANDBY, buf, 1);
}

void Lr20xxDriverBase::SetFs(void)
{
    WriteCommand(LR20XX_CMD_SET_FS);
}


// Common Radio Commands

void Lr20xxDriverBase::SetRfFrequency(uint32_t RfFreq_hz)
{
uint8_t buf[4];

    buf[0] = (uint8_t)((RfFreq_hz & 0xFF000000) >> 24);
    buf[1] = (uint8_t)((RfFreq_hz & 0x00FF0000) >> 16);
    buf[2] = (uint8_t)((RfFreq_hz & 0x0000FF00) >> 8);
    buf[3] = (uint8_t) (RfFreq_hz & 0x000000FF);

    WriteCommand(LR20XX_CMD_SET_RF_FREQUENCY, buf, 4);
}

void Lr20xxDriverBase::SetRxPath(uint8_t RxPath, uint8_t RxBoost)
{
uint8_t buf[2];

    buf[0] = (RxPath & 0x01);
    buf[1] = (RxBoost & 0x07);

    WriteCommand(LR20XX_CMD_SET_RX_PATH, buf, 2);
}

void Lr20xxDriverBase::SetPaConfig(uint8_t PaSel, uint8_t PaLfMode, uint8_t PaLfDutyCycle, uint8_t PaLfSlices, uint8_t PaHfDutyCycle)
{
uint8_t buf[3];

    buf[0] = (PaLfMode & 0x03) + ((PaSel & 0x01) << 7);
    buf[1] = (PaLfSlices & 0x0F) + ((PaLfDutyCycle & 0x0F) << 4);
    buf[2] = (PaHfDutyCycle & 0x1F);

    WriteCommand(LR20XX_CMD_SET_PA_CONFIG, buf, 3);
}

void Lr20xxDriverBase::SetPaConfig915Mhz(int8_t Power)
{
uint8_t PaLfDutyCycle = LR20XX_PA_LF_DUTY_CYCLE_DEFAULT;
uint8_t PaLfSlices = LR20XX_PA_LF_SLICES_DEFAULT;

#if 0
    // table 7-16
    // convert LF Power -19 ... 44 = -9.5 ... 22 dBm
    // into TargetPower of table 7-16
    int16_t TargetPower = Power - 20;
    switch (TargetPower) {
    case 0  /* 10_DBM   */: PaLfDutyCycle = 2; PaLfSlices = 1; break;
    case 1  /* 10p5_DBM */: PaLfDutyCycle = 3; PaLfSlices = 1; break;
    case 2  /* 11_DBM   */: PaLfDutyCycle = 2; PaLfSlices = 2; break;
    case 3  /* 11p5_DBM */: PaLfDutyCycle = 4; PaLfSlices = 2; break;
    case 4  /* 12_DBM   */: PaLfDutyCycle = 5; PaLfSlices = 1; break;
    case 5  /* 12p5_DBM */: PaLfDutyCycle = 4; PaLfSlices = 2; break;
    case 6  /* 13_DBM   */: PaLfDutyCycle = 4; PaLfSlices = 3; break;
    case 7  /* 13p5_DBM */: PaLfDutyCycle = 6; PaLfSlices = 1; break;
    case 8  /* 14_DBM   */: PaLfDutyCycle = 4; PaLfSlices = 2; break;
    case 9  /* 14p5_DBM */:
    case 10 /* 15_DBM   */:
    case 11 /* 15p5_DBM */: PaLfDutyCycle = 5; PaLfSlices = 4; break;
    case 12 /* 16_DBM   */: PaLfDutyCycle = 4; PaLfSlices = 4; break;
    case 13 /* 16p5_DBM */:
    case 14 /* 17_DBM   */:
    case 15 /* 17p5_DBM */:
    case 16 /* 18_DBM   */: PaLfDutyCycle = 5; PaLfSlices = 6; break;
    case 17 /* 18p5_DBM */:
    case 18 /* 19_DBM   */:
    case 19 /* 19p5_DBM */:
    case 20 /* 20_DBM   */: PaLfDutyCycle = 6; PaLfSlices = 6; break;
    case 21 /* 20p5_DBM */:
    case 22 /* 21_DBM   */:
    case 23 /* 21p5_DBM */: PaLfDutyCycle = 7; PaLfSlices = 7; break;
    case 24 /* 22_DBM   */: PaLfDutyCycle = 7; PaLfSlices = 6; break;
    }
#endif

    SetPaConfig(LR20XX_PA_SEL_LF, LR20XX_PA_LF_MODE_FSM, PaLfDutyCycle, PaLfSlices, LR20XX_PA_HF_DUTY_CYCLE_DEFAULT);
}

void Lr20xxDriverBase::SetPaConfig2p4Ghz(int8_t Power)
{
uint8_t PaHfDutyCycle = LR20XX_PA_HF_DUTY_CYCLE_DEFAULT;

#if 0
    // table 7-16
    // convert HF Power -39 ... 24 = -19.5 ... 12 dBm
    // into TargetPower of table 7-18
    int16_t TargetPower = Power / 2;
    switch (TargetPower) {
    case 0  /* 0_DBM */:  PaHfDutyCycle = 10; break;
    case 1  /* 1_DBM */:  PaHfDutyCycle = 30; break;
    case 2  /* 2_DBM */:  PaHfDutyCycle = 28;break;
    case 3  /* 3_DBM */:  PaHfDutyCycle = 25; break;
    case 4  /* 4_DBM */:  PaHfDutyCycle = 25; break;
    case 5  /* 5_DBM */:  PaHfDutyCycle = 31; break;
    case 6  /* 6_DBM */:  PaHfDutyCycle = 30; break;
    case 7  /* 7_DBM */:  PaHfDutyCycle = 30; break;
    case 8  /* 8_DBM */:  PaHfDutyCycle = 31; break;
    case 9  /* 9_DBM */:  PaHfDutyCycle = 30; break;
    case 10 /* 10_DBM */: PaHfDutyCycle = 30; break;
    case 11 /* 11_DBM */: PaHfDutyCycle = 26; break;
    case 12 /* 12_DBM */: PaHfDutyCycle = 16; break;
    }
#endif

    SetPaConfig(LR20XX_PA_SEL_HF, LR20XX_PA_LF_MODE_FSM,
                LR20XX_PA_LF_DUTY_CYCLE_DEFAULT, LR20XX_PA_LF_SLICES_DEFAULT, PaHfDutyCycle);
}

void Lr20xxDriverBase::SetPaConfig433Mhz(int8_t Power)
{
uint8_t PaLfDutyCycle = LR20XX_PA_LF_DUTY_CYCLE_DEFAULT;
uint8_t PaLfSlices = LR20XX_PA_LF_SLICES_DEFAULT;

    SetPaConfig(LR20XX_PA_SEL_LF, LR20XX_PA_LF_MODE_FSM, PaLfDutyCycle, PaLfSlices, LR20XX_PA_HF_DUTY_CYCLE_DEFAULT);
}

void Lr20xxDriverBase::SetTxParams(int8_t Power, uint8_t RampTime)
{
uint8_t buf[2];

    buf[0] = Power;
    buf[1] = RampTime;

    WriteCommand(LR20XX_CMD_SET_TX_PARAMS, buf, 2);
}

void Lr20xxDriverBase::SetRssiCalibration(uint8_t RxPathHf, uint8_t RxPathLf, uint8_t* table)
{
/* // TODO
uint8_t buf[1 + 2*(3*81)];

    buf[0] = ((RxPathHf & 0x01) << 1) + (RxPathLf & 0x01);
    for (uint8_t i = 0; i < 2*(3*81); i++) buf[i+1] = table[i];

    WriteCommand(LR20XX_CMD_SET_RSSI_CALIBRATION, buf, 1 + 2*(3*81)); */
}

void Lr20xxDriverBase::SetRxTxFallbackMode(uint8_t FallbackMode)
{
uint8_t buf[1];

    buf[0] = FallbackMode;

    WriteCommand(LR20XX_CMD_SET_RXTX_FALLBACK_MODE, buf, 1);
}

void Lr20xxDriverBase::SetPacketType(uint8_t PacketType)
{
uint8_t buf[1];

    buf[0] = PacketType;

    WriteCommand(LR20XX_CMD_SET_PACKET_TYPE, buf, 1);
}

uint8_t Lr20xxDriverBase::GetPacketType(void)
{
uint8_t buf[3];

    ReadCommand(LR20XX_CMD_GET_PACKET_TYPE, buf, 3);

    return buf[2];
}

void Lr20xxDriverBase::ResetRxStats(void)
{
    WriteCommand(LR20XX_CMD_RESET_RX_STATS);
}

int16_t Lr20xxDriverBase::GetRssiInst(void)
{
uint8_t buf[4];

    ReadCommand(LR20XX_CMD_GET_RSSI_INST, buf, 4);

    return -(int16_t)buf[2]; // rssi(dBm) = - RSSI/2 or just byte 3
}

void Lr20xxDriverBase::SetRx(uint32_t RxTimeout) // 24 bits only, similar to sx126x
{
uint8_t buf[3];

    buf[0] = (uint8_t)((RxTimeout & 0x00FF0000) >> 16);
    buf[1] = (uint8_t)((RxTimeout & 0x0000FF00) >> 8);
    buf[2] = (uint8_t) (RxTimeout & 0x000000FF);

    WriteCommand(LR20XX_CMD_SET_RX, buf, 3);
}

void Lr20xxDriverBase::SetRx(void) // timeout set with SetDefaultRxTxTimeout() is used
{
    WriteCommand(LR20XX_CMD_SET_RX);
}

void Lr20xxDriverBase::SetTx(uint32_t TxTimeout) // 24 bits only, similar to sx126x
{
uint8_t buf[3];

    buf[0] = (uint8_t)((TxTimeout & 0x00FF0000) >> 16);
    buf[1] = (uint8_t)((TxTimeout & 0x0000FF00) >> 8);
    buf[2] = (uint8_t) (TxTimeout & 0x000000FF);

    WriteCommand(LR20XX_CMD_SET_TX, buf, 3);
}

void Lr20xxDriverBase::SetTx(void) // timeout set with SetDefaultRxTxTimeout() is used
{
    WriteCommand(LR20XX_CMD_SET_TX);
}

void Lr20xxDriverBase::SelPa(uint8_t PaSel)
{
uint8_t buf[1];

    buf[0] = (PaSel & 0x01);

    WriteCommand(LR20XX_CMD_SEL_PA, buf, 1);
}

uint16_t Lr20xxDriverBase::GetRxPktLength(void)
{
uint8_t buf[4];

    ReadCommand(LR20XX_CMD_GET_RX_PKT_LENGTH, buf, 4);

    return ((uint16_t)buf[2] << 8) + (uint16_t)buf[3];
}

void Lr20xxDriverBase::SetDefaultRxTxTimeout(uint32_t RxTimeout, uint32_t TxTimeout) // 24 bits only
{
uint8_t buf[6];

    buf[0] = (uint8_t)((RxTimeout & 0x00FF0000) >> 16);
    buf[1] = (uint8_t)((RxTimeout & 0x0000FF00) >> 8);
    buf[2] = (uint8_t) (RxTimeout & 0x000000FF);

    buf[3] = (uint8_t)((TxTimeout & 0x00FF0000) >> 16);
    buf[4] = (uint8_t)((TxTimeout & 0x0000FF00) >> 8);
    buf[5] = (uint8_t) (TxTimeout & 0x000000FF);

    WriteCommand(LR20XX_CMD_SET_DEFAULT_RX_TX_TIMEOUT, buf, 6);
}

void Lr20xxDriverBase::SetAgcGainManual(uint8_t GainStep)
{
uint8_t buf[1];

    buf[0] = (GainStep & 0x0F);

    WriteCommand(LR20XX_CMD_SET_AGC_GAIN_MANUAL, buf, 1);
}


// LoRa Packet Radio Commands

void Lr20xxDriverBase::SetLoraModulationParams(uint8_t SpreadingFactor, uint8_t Bandwidth, uint8_t CodingRate, uint8_t LowDataRateOptimize)
{
uint8_t buf[2];

    buf[0] = ((SpreadingFactor & 0x0F) << 4) + (Bandwidth & 0x0F);
    buf[1] = ((CodingRate & 0x0F) << 4) + (LowDataRateOptimize & 0x03);

    WriteCommand(LR20XX_CMD_SET_LORA_MODULATION_PARAMS, buf, 2);
}

void Lr20xxDriverBase::SetLoraPacketParams(uint16_t PreambleLength, uint8_t HeaderType, uint8_t PayloadLength, uint8_t Crc, uint8_t InvertIQ)
{
uint8_t buf[4];

    buf[0] = (uint8_t)((PreambleLength & 0xFF00) >> 8);
    buf[1] = (uint8_t) (PreambleLength & 0x00FF);
    buf[2] = PayloadLength;
    buf[3] = ((HeaderType & 0x01) << 2) + ((Crc & 0x01) << 1) + (InvertIQ & 0x01);

    WriteCommand(LR20XX_CMD_SET_LORA_PACKET_PARAMS, buf, 4);
}

void Lr20xxDriverBase::GetLoraRxStats(
    uint16_t* pkt_rx, uint16_t* pkt_crc_error, uint16_t* header_crc_error, uint16_t* false_synch)
{
uint8_t buf[10];

    ReadCommand(LR20XX_CMD_GET_LORA_RX_STATS, buf, 10);

    *pkt_rx = ((uint16_t)buf[2] << 8) + buf[3];
    *pkt_crc_error = ((uint16_t)buf[4] << 8) + buf[5];
    *header_crc_error = ((uint16_t)buf[6] << 8) + buf[7];
    *false_synch = ((uint16_t)buf[8] << 8) + buf[9];
}

void Lr20xxDriverBase::GetLoraPacketStatus(
      int16_t* Rssi, int16_t* RssiSignal, int8_t* Snr,
      uint8_t* Crc, uint8_t* CR, uint16_t* PktLen, uint8_t* Detector)
{
uint8_t buf[8];

    ReadCommand(LR20XX_CMD_GET_LORA_PACKET_STATUS, buf, 8);

    *Crc = ((buf[2] & 0x10) >> 4);
    *CR = (buf[2] & 0x0F);
    *PktLen = buf[3];
    *Snr = (int8_t)buf[4] / 4; // snr_pkt is in 1/4 dB
    *Rssi = (int16_t)(((uint16_t)buf[5] << 1) + (((uint16_t)buf[7] & 0x02) >> 1));
    *RssiSignal = (int16_t)(((uint16_t)buf[6] << 1) + ((uint16_t)buf[7] & 0x01));
    *Detector = (buf[7] & 0x3C) >> 2;
}

void Lr20xxDriverBase::GetLoraPacketStatus(int16_t* Rssi, int16_t* RssiSignal, int8_t* Snr)
{
uint8_t buf[8];

    ReadCommand(LR20XX_CMD_GET_LORA_PACKET_STATUS, buf, 8);

    *Snr = (int8_t)buf[4] / 4; // snr_pkt is in 1/4 dB
    *Rssi = (int16_t)(((uint16_t)buf[5] << 1) + (((uint16_t)buf[7] & 0x02) >> 1));
    *RssiSignal = (int16_t)(((uint16_t)buf[6] << 1) + ((uint16_t)buf[7] & 0x01));
}


// FSK Packet Radio Commands

void Lr20xxDriverBase::SetModulationParamsFSK(uint32_t BitRate, uint8_t PulseShape, uint8_t Bandwidth, uint32_t Fdev_hz)
{
uint8_t buf[9];

    buf[0] = (uint8_t)((BitRate & 0xFF000000) >> 24);
    buf[1] = (uint8_t)((BitRate & 0x00FF0000) >> 16);
    buf[2] = (uint8_t)((BitRate & 0x0000FF00) >> 8);
    buf[3] = (uint8_t) (BitRate & 0x000000FF);
    buf[4] = PulseShape;
    buf[5] = Bandwidth;
    buf[6] = (uint8_t)((Fdev_hz & 0x00FF0000) >> 16);
    buf[7] = (uint8_t)((Fdev_hz & 0x0000FF00) >> 8);
    buf[8] = (uint8_t) (Fdev_hz & 0x000000FF);

    WriteCommand(LR20XX_CMD_SET_FSK_MODULATION_PARAMS, buf, 9);
}

void Lr20xxDriverBase::SetPacketParamsFSK(
    uint16_t PreambleLength, uint8_t PreambleDetectorLength,
    uint8_t long_preamble_mode, uint8_t pld_len_unit, uint8_t addr_comp,
    uint8_t PacketFormat, uint16_t PayloadLength, uint8_t Crc,
    uint8_t dc_free)
{
uint8_t buf[7];

    buf[0] = (uint8_t)((PreambleLength & 0xFF00) >> 8);
    buf[1] = (uint8_t) (PreambleLength & 0x00FF);
    buf[2] = PreambleDetectorLength;
    buf[3] = ((long_preamble_mode & 0x01) << 5) + ((pld_len_unit & 0x01) << 4) +
             ((addr_comp & 0x03) << 2) + (PacketFormat & 0x03);
    buf[4] = (uint8_t)((PayloadLength & 0xFF00) >> 8);
    buf[5] = (uint8_t) (PayloadLength & 0x00FF);
    buf[6] = ((Crc & 0x0F) << 4) + (dc_free & 0x0F);

    WriteCommand(LR20XX_CMD_SET_FSK_PACKET_PARAMS, buf, 7);
}

void Lr20xxDriverBase::SetWhiteningParamsFSK(uint8_t WhitenType, uint16_t Init)
{
uint8_t buf[2];

    buf[0] = ((WhitenType & 0x0F) << 4) + (uint8_t)((Init & 0x0F00) >> 8);
    buf[1] = (uint8_t)(Init & 0x00FF);

    WriteCommand(LR20XX_CMD_SET_FSK_WHITENING_PARAMS, buf, 2);
}

void Lr20xxDriverBase::SetCrcParamsFSK(uint32_t Polynom, uint32_t Init)
{
uint8_t buf[8];

    buf[0] = (uint8_t)((Polynom & 0xFF000000) >> 24);
    buf[1] = (uint8_t)((Polynom & 0x00FF0000) >> 16);
    buf[2] = (uint8_t)((Polynom & 0x0000FF00) >> 8);
    buf[3] = (uint8_t) (Polynom & 0x000000FF);

    buf[4] = (uint8_t)((Init & 0xFF000000) >> 24);
    buf[5] = (uint8_t)((Init & 0x00FF0000) >> 16);
    buf[6] = (uint8_t)((Init & 0x0000FF00) >> 8);
    buf[7] = (uint8_t) (Init & 0x000000FF);

    WriteCommand(LR20XX_CMD_SET_FSK_CRC_PARAMS, buf, 8);
}

void Lr20xxDriverBase::SetSyncWordFSK(uint64_t SyncWord, uint8_t bit_order, uint8_t nb_bits)
{
uint8_t buf[9];

    buf[0] = (uint8_t)((SyncWord & 0xFF00000000000000) >> 56);
    buf[1] = (uint8_t)((SyncWord & 0x00FF000000000000) >> 48);
    buf[2] = (uint8_t)((SyncWord & 0x0000FF0000000000) >> 40);
    buf[3] = (uint8_t)((SyncWord & 0x000000FF00000000) >> 32);
    buf[4] = (uint8_t)((SyncWord & 0x00000000FF000000) >> 24);
    buf[5] = (uint8_t)((SyncWord & 0x0000000000FF0000) >> 16);
    buf[6] = (uint8_t)((SyncWord & 0x000000000000FF00) >> 8);
    buf[7] = (uint8_t) (SyncWord & 0x00000000000000FF);

    buf[8] = ((bit_order & 0x01) << 7) + (nb_bits & 0x7F);

    WriteCommand(LR20XX_CMD_SET_FSK_SYNC_WORD, buf, 9);
}

void Lr20xxDriverBase::SetAddressFSK(uint8_t addr_node, uint8_t addr_bcast)
{
uint8_t buf[2];

    buf[0] = addr_node;
    buf[1] = addr_bcast;

    WriteCommand(LR20XX_CMD_SET_FSK_ADDRESS, buf, 2);
}

void Lr20xxDriverBase::GetRxStatsFSK(
    uint16_t* pkt_rx, uint16_t* pkt_crc_error, uint16_t* len_error, uint16_t* pbl_det,
    uint16_t* sync_ok, uint16_t* sync_fail, uint16_t* timeout)
{
uint8_t buf[16];

    ReadCommand(LR20XX_CMD_GET_FSK_RX_STATS, buf, 16);

    *pkt_rx = ((uint16_t)buf[2] << 8) + buf[3];
    *pkt_crc_error = ((uint16_t)buf[4] << 8) + buf[5];
    *len_error = ((uint16_t)buf[6] << 8) + buf[7];
    *pbl_det = ((uint16_t)buf[8] << 8) + buf[9];
    *sync_ok = ((uint16_t)buf[10] << 8) + buf[11];
    *sync_fail = ((uint16_t)buf[12] << 8) + buf[13];
    *timeout = ((uint16_t)buf[14] << 8) + buf[15];
}

void Lr20xxDriverBase::GetPacketStatusFSK(
    uint16_t* PktLen, int16_t* RssiAvg, int16_t* RssiSync, uint8_t* AddrMatchBcast, uint8_t* AddrMatchNode, int8_t* Lqi)
{
uint8_t buf[8];

    ReadCommand(LR20XX_CMD_GET_FSK_PACKET_STATUS, buf, 8);

    *PktLen = ((uint16_t)buf[2] << 8) + buf[3];
    *RssiAvg = (int16_t)(((uint16_t)buf[4] << 1) + ((buf[6] & 0x04) >> 2));
    *RssiSync = (int16_t)(((uint16_t)buf[5] << 1) + (buf[6] & 0x01));
    *AddrMatchBcast = (buf[6] & 0x20) >> 5;
    *AddrMatchNode = (buf[6] & 0x10) >> 4;
    *Lqi = (int8_t)buf[7];
}

void Lr20xxDriverBase::GetPacketStatusFSK(int16_t* RssiAvg, int16_t* RssiSync, int8_t* Lqi)
{
uint8_t buf[8];

    ReadCommand(LR20XX_CMD_GET_FSK_PACKET_STATUS, buf, 8);

    *RssiAvg = (int16_t)(((uint16_t)buf[4] << 1) + ((buf[6] & 0x04) >> 2));
    *RssiSync = (int16_t)(((uint16_t)buf[5] << 1) + (buf[6] & 0x01));
    *Lqi = (int8_t)buf[7];
}


// FLRC Packet Radio Commands

void Lr20xxDriverBase::SetModulationParamsFLRC(uint8_t BitrateBw, uint8_t CodingRate, uint8_t PulseShape)
{
uint8_t buf[2];

    buf[0] = BitrateBw;
    buf[1] = ((CodingRate & 0x0F) << 4) + (PulseShape & 0x0F);

    WriteCommand(LR20XX_CMD_SET_FLRC_MODULATION_PARAMS, buf, 2);
}

void Lr20xxDriverBase::SetPacketParamsFLRC(
    uint8_t AgcPblLen, uint8_t SyncWordLength,
    uint8_t SyncWordTx, uint8_t SyncWordMatch,
    uint8_t PacketFormat, uint8_t CrcLength,
    uint16_t PayloadLength)
{
uint8_t buf[4];

    buf[0] = ((AgcPblLen & 0x0F) << 2) + (SyncWordLength & 0x03);
    buf[1] = ((SyncWordTx & 0x03) << 6) + ((SyncWordMatch & 0x07) << 3) + ((PacketFormat & 0x01) << 2) + (CrcLength & 0x03);
    buf[2] = (uint8_t)((PayloadLength & 0xFF00) >> 8);
    buf[3] = (uint8_t) (PayloadLength & 0x00FF);

    WriteCommand(LR20XX_CMD_SET_FLRC_PACKET_PARAMS, buf, 4);
}

void Lr20xxDriverBase::GetRxStatsFLRC(int16_t* stats)
{
    while(1){}
}

void Lr20xxDriverBase::GetPacketStatusFLRC(uint16_t* PktLen, int16_t* RssiAvg, int16_t* RssiSync, uint8_t* SyncWordNum)
{
uint8_t buf[7];

    ReadCommand(LR20XX_CMD_GET_FLRC_PACKET_STATUS, buf, 7);

    *PktLen = ((uint16_t)buf[2] << 8) + buf[3];
    *RssiAvg = (int16_t)(((uint16_t)buf[4] << 1) + ((buf[6] & 0x04) >> 2));
    *RssiSync = (int16_t)(((uint16_t)buf[5] << 1) + (buf[6] & 0x01));
    *SyncWordNum = (buf[6] & 0xF0) >> 4;
}

void Lr20xxDriverBase::GetPacketStatusFLRC(int16_t* RssiAvg, int16_t* RssiSync)
{
uint8_t buf[7];

    ReadCommand(LR20XX_CMD_GET_FLRC_PACKET_STATUS, buf, 7);

    *RssiAvg = (int16_t)(((uint16_t)buf[4] << 1) + ((buf[6] & 0x04) >> 2));
    *RssiSync = (int16_t)(((uint16_t)buf[5] << 1) + (buf[6] & 0x01));
}

void Lr20xxDriverBase::SetSyncWordFLRC(uint8_t SyncWordNum, uint32_t SyncWord)
{
uint8_t buf[5];

    buf[0] = SyncWordNum;
    buf[1] = (uint8_t)((SyncWord & 0xFF000000) >> 24);
    buf[2] = (uint8_t)((SyncWord & 0x00FF0000) >> 16);
    buf[3] = (uint8_t)((SyncWord & 0x0000FF00) >> 8);
    buf[4] = (uint8_t) (SyncWord & 0x000000FF);

    WriteCommand(LR20XX_CMD_SET_FLRC_SYNCWORD, buf, 5);
}


// auxiliary methods

#define LR20XX_WORKAROUND_LORA_SX1276_COMPAT_REG_ADDR   0x00F30A14
#define LR20XX_WORKAROUND_LORA_SX1276_COMPAT_REG_MASK   (3 << 18)
#define LR20XX_WORKAROUND_LORA_SX1276_COMPAT_REG_VAL    (1 << 19)

void Lr20xxDriverBase::EnableSx127xCompatibility(void)
{
    WriteRegMemMask32(
        LR20XX_WORKAROUND_LORA_SX1276_COMPAT_REG_ADDR,
        LR20XX_WORKAROUND_LORA_SX1276_COMPAT_REG_MASK,
        LR20XX_WORKAROUND_LORA_SX1276_COMPAT_REG_VAL);
}


// Firmware Patch RAM (PRAM) methods, chapter 22.3

// The PRAM is LR2021 specific, is lost on reset and cold start, but survives sleep with retention.
// Load it as the first step after the reset: reset -> WaitOnBusy() -> LoadPram() -> SetRegMode()/...

#define LR20XX_PRAM_CHECK_ADDR            0x800FF8    // holds LR20XX_PRAM_CHECK_VALUE when PRAM is loaded
#define LR20XX_PRAM_CHECK_VALUE           0x600DB002  // looks like pram_lr2021[0]
#define LR20XX_PRAM_VERSION_ADDR          0x800FFC
#define LR20XX_PRAM_ADDR                  0x801000    // start address of the PRAM
#define LR20XX_PRAM_CHUNK_SIZE            32          // maximal number of uint32_t written per Write/ReadRegMem32() call

// LR2021 pram image, copy from Semtech's LR20xx driver.
// Clear BSD License, (c) Semtech 2025.
// https://github.com/Lora-net/usp/tree/branch/v1.1.2-feature-202604/smtc_rac_lib/radio_drivers/lr20xx_driver/inc

#define LR20XX_PRAM_LR2021_LEN            560 // number of uint32_t in the LR2021 pram image

static const uint32_t pram_lr2021[LR20XX_PRAM_LR2021_LEN] = {
    0x600DB002, 0x00031304, 0x0010104C, 0x00101018, 0x00000000, 0x00000000, 0x40C3C0F1, 0x00088000,
    0x0000212A, 0x0FF82184, 0x212BB986, 0x40C30000, 0x09680080, 0x1800D981, 0xB90D0041, 0x008040C3,
    0x794077F8, 0x7FE0C0D1, 0x78E0720C, 0x45CBC2E4, 0x77F80080, 0x10011D00, 0x41C358BF, 0x18C00010,
    0x008046CB, 0xE81A0968, 0x0F80212B, 0x02900000, 0x70148600, 0x41C3F2AC, 0x00088000, 0x0040202A,
    0x0FF82084, 0x202BB886, 0xD9810040, 0x40A1B90D, 0x1E007960, 0xF09C1041, 0x852F0B0E, 0x0307208A,
    0x0146208A, 0x001041C3, 0x0BE2134C, 0x1E00856F, 0x41C31001, 0x120C0010, 0x856F0BD2, 0x41C3D83D,
    0x11D80010, 0x856F0BC6, 0x41C3D835, 0x13940010, 0x856F0BBA, 0x0786208A, 0x001041C3, 0x0BAE13B4,
    0x208A856F, 0x41C30846, 0x14580010, 0x856F0B9E, 0x41C3D848, 0x143C0010, 0x856F0B92, 0x41C3D847,
    0x14240010, 0x856F0B86, 0x41C3D845, 0x15840010, 0x856F0B7A, 0x41C3D8E0, 0x15580010, 0x856F0B6E,
    0x41C3D8DC, 0x16080010, 0x856F0B62, 0x0304208A, 0x001041C3, 0x0B5615D0, 0x208A856F, 0x41C302C4,
    0x16680010, 0x856F0B46, 0x04C4208A, 0x001041C3, 0x0B3A1640, 0x208A856F, 0x41C30484, 0x13740010,
    0x856F0B2A, 0x0186208A, 0x001041C3, 0x0B1E1470, 0xD882856F, 0x001041C3, 0x0B121690, 0x208A856F,
    0x41C30504, 0x184C0010, 0x856F0B02, 0x0087208A, 0x001041C3, 0x0AF617F0, 0x208A856F, 0x41C30CC6,
    0x18040010, 0x856F0AE6, 0x0DC6208A, 0x001041C3, 0x0ADA15AC, 0xD8F6856F, 0x001040C3, 0x80401000,
    0x40C38021, 0x0FF80080, 0xA021A040, 0x0000090A, 0x00000CF6, 0xC6C4720C, 0x47CBC2E6, 0x020000F4,
    0xB98DD9F0, 0x87C07960, 0x87004508, 0x120126AD, 0x10710E0D, 0x020120AD, 0x0095080D, 0x10940E0D,
    0x00510809, 0x000008D2, 0xC6C640A1, 0x1CFCC2E6, 0xC1A1B6C8, 0x008044CB, 0x84800968, 0x46484768,
    0x43184528, 0x40C3EC0F, 0x77F80080, 0xEC0B8080, 0x41A14063, 0x7C6042C1, 0x208C43E1, 0xC0408FC3,
    0x238CF443, 0xF227B7C2, 0xB2C1238C, 0x238CF229, 0xF22DB682, 0x30310B63, 0x16004063, 0x00807080,
    0x7514600B, 0xC040700C, 0x710CF22F, 0x11340D5B, 0x40C1C040, 0x45CB5839, 0x60500080, 0x734C712C,
    0x5981A520, 0x10011D00, 0x084EE89F, 0x720C0000, 0x40A1F01A, 0x0C4641C1, 0x42E10020, 0x40A1F014,
    0x09EE41C1, 0x42E10020, 0x40A1F00E, 0x0A4A41C1, 0x42E10020, 0x44CBF008, 0x24E00000, 0x42C141A1,
    0x43E17C60, 0xC000C040, 0x7487780F, 0x341B1404, 0x78E0C6C6, 0x42C3C2E2, 0x002400F2, 0x008040C3,
    0x88000773, 0x00710811, 0x250582A0, 0x00F01F80, 0xF0270000, 0x00F441C3, 0x81000200, 0x001C2084,
    0x8004208C, 0x43C3F405, 0xCCCC0044, 0x43C3F004, 0xCCCC002C, 0x008044CB, 0xA460004C, 0x80001144,
    0x2D0070D3, 0x25040000, 0xFF0F1F80, 0xF788FFFF, 0x797DB894, 0xB897B896, 0xF003A420, 0xA200B897,
    0x70001600, 0x014400F4, 0xC6C25852, 0x1600C2E2, 0x0080708D, 0x42C30773, 0xFDC00000, 0x16007A40,
    0x00807081, 0x75300773, 0xFFE20F6C, 0x40A14508, 0x78E0C6C2, 0x003E0817, 0x40C34408, 0x09680080,
    0xB8028800, 0x204F7885, 0x788F004C, 0x000044CB, 0x7C00FEB0, 0x008041C3, 0x89200773, 0x42C3E987,
    0x002400F2, 0xB9AD8220, 0x41C3A220, 0x101C0001, 0x78E07900, 0x40C3C0F1, 0x13B00001, 0x40C37840,
    0x002400F2, 0xB98D8020, 0x7FE0C0D1, 0x0000A020, 0x70811600, 0x02A10080, 0x0837690A, 0x713400B5,
    0x00F341C3, 0x81000C14, 0x02822150, 0x0180206C, 0x0F012085, 0x8105A100, 0x07F02084, 0x000F2085,
    0x8200A105, 0x01C0206C, 0x7FE0B880, 0x7DE0A200, 0x00F340C3, 0x80200814, 0xB993B9D2, 0xB995B994,
    0xA0207FE0, 0x40C3C0F1, 0x269C0000, 0x0FA67840, 0x4300FFEF, 0x7FE0C0D1, 0x78E04060, 0xC1A1C3E2,
    0x45CBC404, 0x27500000, 0xC4407D60, 0xFFEF0F86, 0x40604300, 0x78E0C7C2, 0x42C3C0F1, 0x27B80000,
    0x0F727A40, 0x4300FFEF, 0x7FE0C0D1, 0x78E04060, 0x43C3C0F1, 0x4F280000, 0x42C37B40, 0x0D2400F3,
    0xB9DD8220, 0x7FE0C0D1, 0x78E0A220, 0x0872C2E4, 0x454881EF, 0x88418521, 0x8840A940, 0xA9418521,
    0x90218541, 0xAA22793D, 0x85618824, 0x008042C3, 0x793D77FC, 0x8824AB23, 0x88058862, 0x004C2144,
    0xB8046B32, 0x01032144, 0x204485C1, 0x23050C01, 0x78250300, 0x8240AE04, 0x2A418501, 0xA8250401,
    0x02012A41, 0xA8268501, 0xA8478501, 0x1D00730C, 0xC6C41201, 0x0CDEC2E4, 0x4548802F, 0x85014608,
    0xA8208E29, 0x85018E28, 0x8521A821, 0x781D9605, 0x9606A902, 0x781D8521, 0x886F0D06, 0x8521A903,
    0x9606A904, 0x20449625, 0x69120042, 0x20448521, 0x78450100, 0x8E12A905, 0xA9068521, 0x40C38561,
    0x78080080, 0x11C11D00, 0x8B458800, 0x79456833, 0xAB25730C, 0x78E0C6C4, 0x0FD2C2E2, 0x4508838F,
    0x000040C3, 0x78408DB8, 0x00F340C3, 0x80200B74, 0x04012184, 0x0411090F, 0x08BA8001, 0x780F836F,
    0xC6C2AD08, 0x43C3C2E2, 0x8EB80000, 0x45087B60, 0x00F344CB, 0x84600B50, 0x10022578, 0x2304BA18,
    0xF8FF0F81, 0x7945FFFF, 0xC6C2A420, 0x45CBC2E4, 0x014400F4, 0x000040C3, 0x78609FEC, 0x850085C0,
    0x781178C2, 0x00A070D3, 0x0D0C0001, 0xC6C4FFC6, 0x41C3C2E2, 0xA9680000, 0x082B7940, 0x45080131,
    0x820F0AEE, 0x0111081F, 0x00F442C3, 0x82200144, 0x008040C3, 0x18007800, 0xA0210041, 0x0180216C,
    0x40A1A200, 0x78E0C6C2, 0x41C3C2E2, 0xAAD40000, 0x082B7940, 0x45080171, 0x820F0AB6, 0x0111081F,
    0x00F442C3, 0x82200144, 0x008040C3, 0x18007800, 0xA0210041, 0x0180216C, 0x40A1A200, 0x78E0C6C2,
    0x40C3C0F1, 0xB15C0000, 0x42C37840, 0x78000080, 0xE9098220, 0x1A008221, 0x1E000001, 0x00F47040,
    0xC0D10144, 0x78E07EE0, 0x40C3C0F1, 0xB2B00000, 0x42C37840, 0x78000080, 0xE9098220, 0x1A008221,
    0x1E000001, 0x00F47040, 0xC0D10144, 0x78E07EE0, 0x41C3C2E2, 0xB3DC0000, 0x0A367940, 0x0827820F,
    0x16000151, 0x00F37000, 0x45CB0D3C, 0x77FC0080, 0x0B0AB854, 0xA500802F, 0x85008030, 0xB94C790C,
    0xC6C2A520, 0x008041C3, 0x40C37800, 0x00003930, 0x00011900, 0xA1017FE0, 0x1CFCC2E6, 0xC1A3B6C8,
    0x085B4528, 0x716F01B4, 0x8D838D05, 0x8D648DC0, 0x8D428D21, 0x008F206D, 0xC041B8C0, 0x12002C40,
    0x40C17B05, 0x0DBEC742, 0x1C0082AF, 0x26053081, 0xF415903E, 0x42C38D05, 0x083000F3, 0x78128220,
    0xB9B9B818, 0x800212FC, 0x0F802004, 0x00000200, 0x0B826038, 0x4100862F, 0x4063726F, 0x1404C0A3,
    0xC6C6341B, 0x0A76C2E6, 0x1048800F, 0x08A30080, 0x244A0071, 0x41C37200, 0x6FAA00B1, 0x00F342C3,
    0x40C30C18, 0x6FAA004E, 0x00501A04, 0x59C343C3, 0x1A04AE57, 0x22500010, 0x1A040281, 0x820000D0,
    0xFF7D44CB, 0x43C3D5F7, 0x7F5DFF7D, 0x0F802004, 0xFFFC07FF, 0x0F802005, 0x0001F800, 0xA180A200,
    0x8102A161, 0x9F4643C3, 0x206C0012, 0xB8850180, 0xA261A102, 0x800012F0, 0x43C370AD, 0x18B00010,
    0x0180206C, 0x2045BD93, 0x23400300, 0x41A1020C, 0x10CF2D41, 0x80001AF0, 0x02C020A8, 0x14C01401,
    0x04CE1301, 0xBE087825, 0x78C561F9, 0xC6C6A206, 0x40C3C0F1, 0x1AC00001, 0x0F4E7840, 0xC0D1FFCF,
    0x78E07EE0, 0xC1A5C3E6, 0xC00A4608, 0x4328C40C, 0xC50BC10E, 0xC040C70D, 0x9FC3248C, 0x20CA4081,
    0xE68B0321, 0xC541C144, 0x100124CA, 0x000145CB, 0x40C11E7C, 0xC7434161, 0xC4427D60, 0xE68A4508,
    0xFFEF0F06, 0x106125CA, 0xC7C640A1, 0x41C3C0F1, 0x60C90080, 0x1E008900, 0x00807002, 0x11FF0584,
    0x40C38081, 0x78080080, 0x0B76A020, 0x730C84EF, 0x884F0C0A, 0x7EE0C0D1, 0x02494E49, 0x001018C0,
    0xFFFFF8D0, 0x00800968, 0xFFFFF970, 0x008077F8, 0x00000014, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0xF7E41C09, 0x1F20201F, 0xE10F0FE1, 0xF9FFFFF9,
};

void Lr20xxDriverBase::EnablePram(void)
{
    WriteCommand(LR20XX_CMD_ENABLE_PRAM, 0x00);
}

void Lr20xxDriverBase::LoadPram(void)
{
    for (uint16_t addr = 0; addr < LR20XX_PRAM_LR2021_LEN; addr += LR20XX_PRAM_CHUNK_SIZE) {
        uint16_t chunk_len = LR20XX_PRAM_LR2021_LEN - addr;
        if (chunk_len > LR20XX_PRAM_CHUNK_SIZE) chunk_len = LR20XX_PRAM_CHUNK_SIZE;

        WriteRegMem32(LR20XX_PRAM_ADDR + addr * 4, (uint32_t*)pram_lr2021 + addr, chunk_len);
    }
}

// only valid if LoadPram() and EnablePram() executed
bool Lr20xxDriverBase::CheckPram(void)
{
uint32_t data;

    ReadRegMem32(LR20XX_PRAM_CHECK_ADDR, &data, 1);

    return (data == LR20XX_PRAM_CHECK_VALUE);
}

// pram_lr2021 starts with 0x600DB002, 0x00031304, which look like the check value and type/version words
// read back from 0x800FF8/0x800FFC. ?? does the chip republish them there, i.e. is this 0x0313 ??
// only valid if LoadPram() and EnablePram() executed, otherwise gives random value
// should be called only after CheckPram() returns valid
// currently gives 0x0313

// datasheet and Semtech driver provide inconsistent info
// datasheet:
//   to read version of PRAM: Read register value at 0x800FFC. The version is then ((returned_value >> 8) & 0xFFFF).
// driver:
//   reads register value at 0x800FFC into uint32_t pram_type_version_raw
//   pram_version->pram_type    = (uint8_t)( pram_type_version_raw >> 16 );
//   pram_version->pram_version = (uint8_t)( pram_type_version_raw >> 8 );

uint16_t Lr20xxDriverBase::GetPramVersion(void)
{
uint32_t data;

    ReadRegMem32(LR20XX_PRAM_VERSION_ADDR, &data, 1);

    return (uint16_t)((data >> 8) & 0xFFFF);
}


