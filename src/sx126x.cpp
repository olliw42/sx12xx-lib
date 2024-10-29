//*******************************************************
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// SX126x standard interface
//*******************************************************
// contributed by jinchuuriki
//*******************************************************

#include "sx126x.h"


// spi

void Sx126xDriverBase::SpiRead(uint8_t* datain, uint8_t len)
{
uint8_t dummy = 0; // NOP

    while (len) {
        SpiTransfer(dummy, datain);
        datain++;
        len--;
    }
}


void Sx126xDriverBase::SpiWrite(uint8_t* dataout, uint8_t len)
{
uint8_t dummy;

    while (len) {
        SpiTransfer(*dataout, &dummy);
        dataout++;
        len--;
    }
}


// low level methods

void Sx126xDriverBase::WriteCommand(uint8_t opcode, uint8_t* data, uint8_t len)
{
    WaitOnBusy();
    SpiSelect();
    SpiTransfer(opcode, &_status);
    if (len > 0) SpiWrite(data, len);
    SpiDeselect();
    SetDelay(12); // semtech driver says 12 us // TODO: ??
}


void Sx126xDriverBase::ReadCommand(uint8_t opcode, uint8_t* data, uint8_t len)
{
    WaitOnBusy();
    SpiSelect();
    SpiTransfer(opcode, &_status);
    SpiWrite(0); // NOP
    SpiRead(data, len);
    SpiDeselect();
    // no delay according to semtech driver // TODO: ??
}


void Sx126xDriverBase::WriteRegister(uint16_t adr, uint8_t* data, uint8_t len)
{
    WaitOnBusy();
    SpiSelect();
    SpiTransfer(SX126X_CMD_WRITE_REGISTER, &_status);
    SpiWrite((adr & 0xFF00) >> 8);
    SpiWrite(adr & 0x00FF);
    SpiWrite(data, len);
    SpiDeselect();
    SetDelay(12); // semtech driver says 12 us // TODO: ??
}


void Sx126xDriverBase::ReadRegister(uint16_t adr, uint8_t* data, uint8_t len)
{
    WaitOnBusy();
    SpiSelect();
    SpiTransfer(SX126X_CMD_READ_REGISTER, &_status);
    SpiWrite((adr & 0xFF00) >> 8);
    SpiWrite(adr & 0x00FF);
    SpiWrite(0); // NOP
    SpiRead(data, len);
    SpiDeselect();
    // no delay according to semtech driver // TODO: ??
}


void Sx126xDriverBase::WriteBuffer(uint8_t offset, uint8_t* data, uint8_t len)
{
    WaitOnBusy();
    SpiSelect();
    SpiTransfer(SX126X_CMD_WRITE_BUFFER, &_status);
    SpiWrite(offset);
    SpiWrite(data, len);
    SpiDeselect();
    SetDelay(12); // semtech driver says 12 us // TODO: ??
}


void Sx126xDriverBase::ReadBuffer(uint8_t offset, uint8_t* data, uint8_t len)
{
    WaitOnBusy();
    SpiSelect();
    SpiTransfer(SX126X_CMD_READ_BUFFER, &_status);
    SpiWrite(offset);
    SpiWrite(0); // NOP
    SpiRead(data, len);
    SpiDeselect();
    // no delay according to semtech driver // TODO: ??
}


// common methods

uint8_t Sx126xDriverBase::GetStatus(void)
{
    WriteCommand(SX126X_CMD_GET_STATUS); // yes, this is correct !
    return _status;
}


void Sx126xDriverBase::SetStandby(uint8_t StandbyConfig)
{
    WriteCommand(SX126X_CMD_SET_STANDBY, StandbyConfig);
    switch (StandbyConfig) {
    case SX126X_STDBY_CONFIG_STDBY_XOSC:
        SetDelay(50); // semtech driver says 50 us // TODO: ??
        break;
    default:
        SetDelay(1500); // semtech driver says 1500 us // TODO: ??
    }
}


void Sx126xDriverBase::SetPacketType(uint8_t PacketType)
{
    WriteCommand(SX126X_CMD_SET_PACKET_TYPE, PacketType);

    _packet_type = PacketType;
}


void Sx126xDriverBase::SetRfFrequency(uint32_t RfFrequency) // 24 bit sx1280, 32 bit sx1262
{
uint8_t buf[4];

    buf[0] = (uint8_t)((RfFrequency & 0xFF000000) >> 24);
    buf[1] = (uint8_t)((RfFrequency & 0x00FF0000) >> 16);
    buf[2] = (uint8_t)((RfFrequency & 0x0000FF00) >> 8);
    buf[3] = (uint8_t) (RfFrequency & 0x000000FF);

    WriteCommand(SX126X_CMD_SET_RF_FREQUENCY, buf, 4);
}


void Sx126xDriverBase::SetBufferBaseAddress(uint8_t txBaseAdress, uint8_t rxBaseAdress)
{
uint8_t buf[2];

    buf[0] = txBaseAdress;
    buf[1] = rxBaseAdress;

    WriteCommand(SX126X_CMD_SET_BUFFER_BASEADDRESS, buf, 2);
}


void Sx126xDriverBase::SetModulationParams(uint8_t SpreadingFactor, uint8_t Bandwidth, uint8_t CodingRate)
{
uint8_t buf[4];

   buf[0] = SpreadingFactor;
   buf[1] = Bandwidth;
   buf[2] = CodingRate;
   buf[3] = SX126X_LORA_LDRO_OFF;

   WriteCommand(SX126X_CMD_SET_MODULATION_PARAMS, buf, 4);

   // 15.1 Modulation Quality with 500 kHz LoRa Bandwidth, datasheet p. 105
   // 15.1.1 Description
   // Some sensitivity degradation may be observed on any LoRa device, when receiving signals transmitted by the SX1261/2
   // with a LoRa BW of 500 kHz.
   // 15.1.2 Workaround
   // Before any packet transmission, bit #2 at address 0x0889 shall be set to:
   //   0 if the LoRa BW = 500 kHz
   //   1 for any other LoRa BW
   //   1 for any (G)FSK configuration
   buf[0] = ReadRegister(SX126X_REG_TX_MODULATION);
   if (Bandwidth == SX126X_LORA_BW_500) {
       buf[0] &= 0xFB;
   } else {
       buf[0] |= 0x04;
   }
   WriteRegister(SX126X_REG_TX_MODULATION, buf[0]);
}


void Sx126xDriverBase::SetPacketParams(uint8_t PreambleLength, uint8_t HeaderType, uint8_t PayloadLength, uint8_t Crc, uint8_t InvertIQ)
{
uint8_t buf[6];

    buf[0] = 0;
    buf[1] = PreambleLength;
    buf[2] = HeaderType;
    buf[3] = PayloadLength;
    buf[4] = Crc;
    buf[5] = InvertIQ;

    WriteCommand(SX126X_CMD_SET_PACKET_PARAMS, buf, 6);

    // 15.4 Optimizing the Inverted IQ Operation, datasheet p. 106
    // 15.4.1 Description
    // When exchanging LoRa packets with inverted IQ polarity
    // some packet losses may be observed for longer packets.
    // 15.4.2 Workaround
    // Bit 2 at address 0x0736 must be set to:
    //   0 when using inverted IQ polarity (see the SetPacketParam(...) command)
    //   1 when using standard IQ polarity
    buf[0] = ReadRegister(SX126X_REG_IQ_POLARITY_SETUP);
    if (InvertIQ) {
        buf[0] &= ~0x04;
    } else {
        buf[0] |= 0x04;
    }
    WriteRegister(SX126X_REG_IQ_POLARITY_SETUP, buf[0]);

    _header_type = HeaderType;
}


void Sx126xDriverBase::SetDioIrqParams(uint16_t IrqMask, uint16_t Dio1Mask, uint16_t Dio2Mask, uint16_t Dio3Mask)
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

    WriteCommand(SX126X_CMD_SET_DIOIRQ_PARAMS, buf, 8);
}


uint16_t Sx126xDriverBase::GetIrqStatus(void)
{
uint8_t status[2];

    ReadCommand(SX126X_CMD_GET_IRQ_STATUS, status, 2);
    return ((uint16_t)status[0] << 8) + status[1];
}


void Sx126xDriverBase::ClearIrqStatus(uint16_t IrqMask)
{
uint8_t buf[2];

    buf[0] = (uint8_t)((IrqMask & 0xFF00) >> 8);
    buf[1] = (uint8_t)(IrqMask & 0x00FF);

    WriteCommand(SX126X_CMD_CLR_IRQ_STATUS, buf, 2);
}


uint16_t Sx126xDriverBase::GetAndClearIrqStatus(uint16_t IrqMask)
{
    uint16_t irq_status = GetIrqStatus();

    ClearIrqStatus(IrqMask);

    if (irq_status & SX126X_IRQ_RX_DONE) {
        ClearRxEvent();
    }

    return irq_status;
}


// Tx methods

void Sx126xDriverBase::SetTxParams(uint8_t Power, uint8_t RampTime)
{
uint8_t buf[2];

    buf[0] = Power;
    buf[1] = RampTime;

    WriteCommand(SX126X_CMD_SET_TX_PARAMS, buf, 2);
}


void Sx126xDriverBase::SetTx(uint32_t tmo_periodbase)
{
uint8_t buf[3];

    // 24 bits time out with base of 15.625us
    // TimeOut duration (us) = 15.625 * timeOut
    if (tmo_periodbase > 0xFFFFFF) tmo_periodbase = 0xFFFFFF;

    buf[0] = (uint8_t)((tmo_periodbase & 0xFF0000) >> 16);
    buf[1] = (uint8_t)((tmo_periodbase & 0x00FF00) >> 8);
    buf[2] = (uint8_t) (tmo_periodbase & 0x0000FF);
    WriteCommand(SX126X_CMD_SET_TX, buf, 3);

    SetDelay(100); // semtech driver says 100 us // TODO: ??
}


// Rx methods

void Sx126xDriverBase::SetRx(uint32_t tmo_periodbase)
{
uint8_t buf[3];

    // 24 bits time out with base of 15.625us
    // TimeOut duration (us) = 15.625 * timeOut
    if (tmo_periodbase > 0xFFFFFF) tmo_periodbase = 0xFFFFFF;

    buf[0] = (uint8_t)((tmo_periodbase & 0xFF0000) >> 16);
    buf[1] = (uint8_t)((tmo_periodbase & 0x00FF00) >> 8);
    buf[2] = (uint8_t) (tmo_periodbase & 0x0000FF);
    WriteCommand(SX126X_CMD_SET_RX, buf, 3);

    SetDelay(100); // semtech driver says 100 us // TODO: ??
}


void Sx126xDriverBase::GetPacketStatus(int16_t* RssiSync, int8_t* Snr)
{
uint8_t status[3];

    ReadCommand(SX126X_CMD_GET_PACKET_STATUS, status, 3);

    *RssiSync = -(int16_t)(status[0] / 2);
    *Snr = (int8_t)status[1] / 4;
}


void Sx126xDriverBase::GetRxBufferStatus(uint8_t* rxPayloadLength, uint8_t* rxStartBufferPointer)
{
uint8_t status[2];

    ReadCommand(SX126X_CMD_GET_RX_BUFFER_STATUS, status, 2);

    *rxPayloadLength = status[0];
    *rxStartBufferPointer = status[1];
}


void Sx126xDriverBase::ClearRxEvent(void)
{
    // 15.3 Implicit Header Mode Timeout Behavior, datasheet p. 106
    // 15.3.1 Description
    // When receiving LoRa packets in Rx mode with Timeout active, and no header (Implicit Mode), the timer responsible for
    // generating the Timeout (based on the RTC timer) is not stopped on RxDone event. Therefore, it may trigger an unexpected
    // timeout in any subsequent mode where the RTC isn't re-invoked, and therefore reset and re-programmed.
    // 15.3.2 Workaround
    // It is advised to add the following commands after ANY Rx with Timeout active sequence, which stop the RTC and clear the
    // timeout event, if any. The register at address 0x0902 will be used to stop the counter, while the register at address 0x0944
    // will clear the potential event.
    if (_packet_type == SX126X_PACKET_TYPE_LORA && _header_type == SX126X_LORA_HEADER_IMPLICIT) {
//??        uint8_t buf = ReadRegister(SX126X_REG_RTC_CONTROL);
        WriteRegister(SX126X_REG_RTC_CONTROL, 0x00); // stop the timer
        uint8_t data = ReadRegister(SX126X_REG_EVENT_MASK);
        data |= 0x02;
        WriteRegister(SX126X_REG_EVENT_MASK, data);
//??        WriteRegister(SX126X_REG_RTC_CONTROL, buf); // restart the timer
    }
}


// auxiliary methods

void Sx126xDriverBase::SetRegulatorMode(uint8_t RegModeParam)
{
    WriteCommand(SX126X_CMD_SET_REGULATOR_MODE, RegModeParam);
}


void Sx126xDriverBase::SetAutoFs(uint8_t flag)
{
uint8_t buf;

    buf = (flag) ? SX126X_RX_TX_FALLBACK_MODE_FS : SX126X_RX_TX_FALLBACK_MODE_STDBY_RC;
    WriteCommand(SX126X_CMD_SET_RX_TX_FALLBACK_MODE, buf);
}


void Sx126xDriverBase::SetFs(void)
{
    WriteCommand(SX126X_CMD_SET_FS);
    SetDelay(70); // semtech driver says 70 us // TODO: ??
}


uint16_t Sx126xDriverBase::GetFirmwareRev(void)
{
    return GetStatus();
}


void Sx126xDriverBase::SetSyncWord(uint16_t SyncWord)
{
uint8_t buf[2];

    buf[0] = (uint8_t)(SyncWord & 0x00FF);
    buf[1] = (uint8_t)((SyncWord & 0xFF00) >> 8);

    WriteRegister(SX126X_REG_LORA_SYNC_WORD_MSB, buf, 2);
}


void Sx126xDriverBase::ClearDeviceError(void)
{
uint8_t buf[] = {0, 0};

    WriteCommand(SX126X_CMD_CLEAR_DEVICE_ERRORS, buf, 2);
}


uint16_t Sx126xDriverBase::GetDeviceError(void)
{
uint8_t buf[2];

    ReadCommand(SX126X_CMD_GET_DEVICE_ERRORS, buf, 2);

    return ((uint16_t)buf[0] << 8) | buf[1];
}


// set DIO3 as TCXO control, DIO3 control output voltage to TCXO oscillator
void Sx126xDriverBase::SetDio3AsTcxoControl(uint8_t OutputVoltage, uint32_t delay_us)
{
uint8_t buf[4];

    buf[0] = OutputVoltage;
    buf[1] = (uint8_t)((delay_us >> 16) & 0xFF);
    buf[2] = (uint8_t)((delay_us >> 8) & 0xFF);
    buf[3] = (uint8_t)(delay_us & 0xFF);

    WriteCommand(SX126X_CMD_SET_DIO3_AS_TCXO_CTRL, buf, 4);
}


// set DIO2 as RF switching control
void Sx126xDriverBase::SetDio2AsRfSwitchControl(uint8_t Mode)
{
    WriteCommand(SX126X_CMD_SET_DIO2_AS_RF_SWITCH_CTRL, Mode);
}


// set LoRa symbol number time out for prevent of false LoRa preamble detection
void Sx126xDriverBase::SetSymbNumTimeout(uint8_t tmo_symbnum)
{
uint8_t mant = (((tmo_symbnum > SX126X_MAX_LORA_SYMB_NUM_TIMEOUT) ? SX126X_MAX_LORA_SYMB_NUM_TIMEOUT : tmo_symbnum) + 1) >> 1;
uint8_t exp  = 0;
uint8_t reg  = 0;

    while (mant > 31) {
        mant = (mant + 3) >> 2;
        exp++;
    }
    reg = mant << (2 * exp + 1);

    WriteCommand(SX126X_CMD_SET_LORA_SYMB_NUM_TIMEOUT, reg);

    if (tmo_symbnum > 0) {
        reg = exp + (mant << 3);
        WriteRegister(SX126X_REG_SYNCH_TIMEOUT, reg);
    }
}


void Sx126xDriverBase::SetRxGain(uint8_t RxGain)
{
    WriteRegister(SX126X_REG_RX_GAIN, RxGain);
}


void Sx126xDriverBase::SetTxClampConfig(void)
{
    // 15.2 Better Resistance of the SX1262 Tx to Antenna Mismatch, datasheet p. 105
    // 15.2.1 Description
    // The SX1261/2 platform embeds a Power Amplifier (PA) clamping mechanism, backing-off the power when over-voltage
    // conditions are detected internally. This method is put in place to protect the internal devices and ensure long-term
    // reliability of the chip. Considering a high-power operation of the SX1262 (supporting +22dBm on-chip), these "clamping"
    // devices are overly protective, causing the chip to back-down its output power when even a reasonable mismatch is
    // detected at the PA output. The observation is typically 5 to 6 dB less output power than the expected.
    // 15.2.2 Workaround
    // On the SX1262, during the chip initialization, the register TxClampConfig should be modified to optimize the PA clamping
    // threshold. Bits 4-1 must be set to "1111" (default value "0100").
    // This register modification must be done after a Power On Reset, or a wake-up from cold Start.
    uint8_t data = ReadRegister(SX126X_REG_TX_CLAMP_CONFIG);
    data |= 0x1E;
    WriteRegister(SX126X_REG_TX_CLAMP_CONFIG, data);
}


void Sx126xDriverBase::SetOverCurrentProtection(uint8_t OverCurrentProtection)
{
    WriteRegister(SX126X_REG_OCP_CONFIGURATION, OverCurrentProtection);
}


void  Sx126xDriverBase::SetPaConfig(uint8_t deviceSel, uint8_t paDutyCycle, uint8_t hpMax, uint8_t paLut)
{
uint8_t buf[4];

    buf[0] = paDutyCycle;
    buf[1] = hpMax;
    buf[2] = deviceSel;
    buf[3] = paLut;

    WriteCommand(SX126X_CMD_SET_PA_CONFIG, buf, 4);
}


void Sx126xDriverBase::SetPaConfig_22dbm(void)
{
uint8_t buf[4];

    buf[0] = SX126X_PA_CONFIG_22_DBM_PA_DUTY_CYCLE_MAX;
    buf[1] = SX126X_PA_CONFIG_22_DBM_HP_MAX;
    buf[2] = SX126X_PA_CONFIG_DEVICE_SEL_SX1262;
    buf[3] = SX126X_PA_CONFIG_22_DBM_PA_LUT;

    WriteCommand(SX126X_CMD_SET_PA_CONFIG, buf, 4);

    
}


void Sx126xDriverBase::SetPaConfig_Min(void)
{
uint8_t buf[4];

    buf[0] = SX126X_PA_CONFIG_22_DBM_PA_DUTY_CYCLE_MIN;
    buf[1] = SX126X_PA_CONFIG_22_DBM_HP_MIN;
    buf[2] = SX126X_PA_CONFIG_DEVICE_SEL_SX1262;
    buf[3] = SX126X_PA_CONFIG_22_DBM_PA_LUT;

    WriteCommand(SX126X_CMD_SET_PA_CONFIG, buf, 4);
}


void Sx126xDriverBase::CalibrateImage(uint8_t Freq1, uint8_t Freq2)
{
uint8_t buf[2];

    buf[0] = Freq1;
    buf[1] = Freq2;

    WriteCommand(SX126X_CMD_CALIBRATE_IMAGE, buf, 2);
}


// thanks to https://github.com/Lora-net/sx126x_driver
// they use Freq1 = Freq1_mhz / 4; Freq2 = (Freq2_mhz + 3) / 4;
// compared to table 9-2, p. 58:
// for 779-787: Freq1 is obtained as 0xC2 but is 0xC1 in the table
// else:        Freq2 is obtained one too low
// => we add +1 to Freq2, to reproduce table for most times
void Sx126xDriverBase::CalibrateImage_mhz(uint16_t Freq1_mhz, uint16_t Freq2_mhz)
{
    uint8_t Freq1 = Freq1_mhz / 4;
    uint8_t Freq2 = (Freq2_mhz + 3) / 4 + 1;

    CalibrateImage(Freq1, Freq2);
}


// GFSK methods

void Sx126xDriverBase::SetModulationParamsGFSK(uint32_t br_bps, uint8_t PulseShape, uint8_t Bandwidth, uint32_t Fdev_hz)
{
uint8_t buf[8];

    uint32_t br = (uint32_t)(32 * SX126X_FREQ_XTAL_HZ) / br_bps;

    buf[0] = (uint8_t)((br >> 16) & 0xFF);
    buf[1] = (uint8_t)((br >> 8) & 0xFF);
    buf[2] = (uint8_t)(br & 0xFF);

    buf[3] = PulseShape;
    buf[4] = Bandwidth;

    double freqStep = (double)SX126X_FREQ_XTAL_HZ / (double)(1 << 25);
    uint32_t Fdev = (uint32_t)((double)Fdev_hz / freqStep);

    buf[5] = (uint8_t)((Fdev >> 16) & 0xFF);
    buf[6] = (uint8_t)((Fdev >> 8) & 0xFF);
    buf[7] = (uint8_t)(Fdev & 0xFF);

    WriteCommand(SX126X_CMD_SET_MODULATION_PARAMS, buf, 8);

    // 15.1 Modulation Quality with 500 kHz LoRa Bandwidth, datasheet p. 105
    // 15.1.1 Description
    // Some sensitivity degradation may be observed on any LoRa device, when receiving signals transmitted by the SX1261/2
    // with a LoRa BW of 500 kHz.
    // 15.1.2 Workaround
    // Before any packet transmission, bit #2 at address 0x0889 shall be set to:
    //   0 if the LoRa BW = 500 kHz
    //   1 for any other LoRa BW
    //   1 for any (G)FSK configuration
    buf[0] = ReadRegister(SX126X_REG_TX_MODULATION);
    buf[0] |= 0x04;
    WriteRegister(SX126X_REG_TX_MODULATION, buf[0]);
}


void Sx126xDriverBase::SetPacketParamsGFSK(uint16_t PreambleLength, uint8_t PreambleDetectorLength, uint8_t SyncWordLength, uint8_t AddrComp, uint8_t PacketType, uint8_t PayloadLength, uint8_t CRCType, uint8_t Whitening)
{
uint8_t buf[9];

    buf[0] = (uint8_t)((PreambleLength >> 8) & 0xFF);
    buf[1] = (uint8_t)(PreambleLength & 0xFF);
    buf[2] = PreambleDetectorLength;
    buf[3] = SyncWordLength;
    buf[4] = AddrComp;
    buf[5] = PacketType;
    buf[6] = PayloadLength;
    buf[7] = CRCType;
    buf[8] = Whitening;

    WriteCommand(SX126X_CMD_SET_PACKET_PARAMS, buf, 9);
}

void Sx126xDriverBase::SetSyncWordGFSK(uint16_t SyncWord)
{
uint8_t buf[8];

    buf[0] = (uint8_t)((SyncWord >> 8) & 0xFF);
    buf[1] = (uint8_t)(SyncWord & 0xFF);
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;

    WriteRegister(SX126X_REG_SYNC_WORD_0, buf, 8);
}

void Sx126xDriverBase::GetPacketStatusGFSK(int16_t* RssiSync)
{
uint8_t status[3];

    ReadCommand(SX126X_CMD_GET_PACKET_STATUS, status, 3);

    *RssiSync = -(int16_t)(status[2] / 2);
}

