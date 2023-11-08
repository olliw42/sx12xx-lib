//*******************************************************
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// SX127x library
//*******************************************************
/*
*/
#ifndef SX127X_LIB_H
#define SX127X_LIB_H
#pragma once

#include <inttypes.h>

#define SX127X_FREQ_XTAL_HZ               32000000

#define SX127X_FREQ_MHZ_TO_REG(f_mhz)     (uint32_t)((double)f_mhz*1.0E6*(double)(1 << 19)/(double)SX127X_FREQ_XTAL_HZ)


#ifndef ALIGNED
#define ALIGNED  __attribute__((aligned(4)))
#endif


//-------------------------------------------------------
// Base Class
//-------------------------------------------------------

class Sx127xDriverBase
{
  public:
    Sx127xDriverBase(); // constructor

    // this you will have to fill in the derived class

    void Init(void) {};

    // these you have to supply in the derived class

    virtual void SpiSelect(void) = 0;
    virtual void SpiDeselect(void) = 0;
    virtual void SpiTransferByte(uint8_t* byteout, uint8_t* bytein) = 0;

    // spi methods

    void SpiRead(uint8_t* datain, uint8_t len);
    void SpiWrite(uint8_t* dataout, uint8_t len);
    void SpiTransfer(uint8_t dataout, uint8_t* datain) { SpiTransferByte(&dataout, datain); }
    void SpiRead(uint8_t* datain) { SpiRead(datain, 1); }
    void SpiWrite(uint8_t dataout) { SpiWrite(&dataout, 1); }

    // low level methods, usually no need to use them

    void WriteRegister(uint16_t adr, uint8_t* data, uint8_t len);
    void ReadRegister(uint16_t adr, uint8_t* data, uint8_t len);

    void WriteRegister(uint16_t adr, uint8_t data) { WriteRegister(adr, &data, 1); }
    uint8_t ReadRegister(uint16_t adr) { uint8_t data; ReadRegister(adr, &data, 1); return data; }
    void ReadWriteRegister(uint16_t adr, uint8_t mask, uint8_t data);

    void WriteBuffer(uint8_t offset, uint8_t* data, uint8_t len);
    void ReadBuffer(uint8_t offset, uint8_t* data, uint8_t len);

    // common methods
    
    uint8_t GetStatus(void);
    void SetSleep(void);
    void SetStandby(void);
    void SetOperationMode(uint8_t PacketType, uint8_t LowFrequencyMode);
    void SetRfFrequency(uint32_t RfFrequency); // 23 bits only
    void SetLnaParams(uint8_t LnaGain, uint8_t LnaBoostHf);
    void OptimizeSensitivity(uint8_t Bandwidth);
    void OptimizeReceiverResponse(uint8_t Bandwidth);
    void SetBufferBaseAddress(uint8_t txBaseAdress, uint8_t rxBaseAdress);
    void SetModulationParams(uint8_t SpreadingFactor, uint8_t Bandwidth, uint8_t CodingRate);
    void SetPacketParams(uint8_t PreambleLength, uint8_t HeaderType, uint8_t PayloadLength, uint8_t Crc, uint8_t InvertIQ);

    void SetDioIrqParams(uint8_t IrqMask, uint8_t Dio0Mask, uint8_t Dio1Mask); // SX127x has DIO0-DIO5, but we use just these two
    uint16_t GetIrqStatus(void);
    void ClearIrqStatus(uint8_t IrqMask);
    uint16_t GetAndClearIrqStatus(uint16_t IrqMask);

    // Tx methods

    void SetPowerParams(uint8_t PaSelect, uint8_t MaxPower, uint8_t OutputPower, uint8_t RampTime);
    void SetTx(void);

    // Rx methods

    void SetRxSingle(void);
    void SetRxContinuous(void);
    void SetRxTimeout(uint16_t tmo_symbols);
    void GetPacketStatus(int16_t* RssiSync, int8_t* Snr);
    void GetRxBufferStatus(uint8_t* rxPayloadLength, uint8_t* rxStartBufferPointer);

    // auxiliary methods

    uint8_t GetFirmwareRev(void);
    void SetSyncWord(uint8_t SyncWord);

    // AFC methods

    int32_t GetFreqError(void);

    // this implements a "canonical" AFC method
    // heavily inspired by ExpressLRS' method (https://github.com/ExpressLRS/ExpressLRS)
    void AfcSetRfFrequency(uint32_t RfFrequency);
    void AfcDo(void);

  private:
    int32_t _rf_freq_reg_correction;
    uint32_t _rf_freq_reg; // 0 indicates not initialized
};


//-------------------------------------------------------
// Enum Definitions
//-------------------------------------------------------
// extracted from datasheet by copy&paste and then formatting
// many registers function differently according to whether LoRa or FSK mode is used
// seperate enums have been created to address the overlap

typedef enum {
    //-- Common Registers, applicable to both LoRa and FSK modes

    SX1276_REG_Fifo                        = 0x00, // 7-0 Fifo
    SX1276_REG_OpMode                      = 0x01, // 7 LongRangeMode, 6 AccessSharedReg, 3 LowFrequencyModeOn, 2-0 Mode
    SX1276_REG_FrMsb                       = 0x06, // 7-0 Frf(23:16)
    SX1276_REG_FrMid                       = 0x07, // 7-0 Frf(15:8)
    SX1276_REG_FrLsb                       = 0x08, // 7-0 Frf(7:0)
    SX1276_REG_PaConfig                    = 0x09, // 7 PaSelect, 6-4 MaxPower, 3-0 OutputPower
    SX1276_REG_PaRamp                      = 0x0A, // 6-5 ModulationShaping, 3-0 PaRamp(3:0)
    SX1276_REG_Ocp                         = 0x0B, // 5 OcpOn, 4-0 OcpTrim
    SX1276_REG_Lna                         = 0x0C, // 7-5 LnaGain, 4-3 LnaBoostLf, 1-0 LnaBoostHf
    SX1276_REG_DioMapping1                 = 0x40, // 7-6 Dio0Mapping, 5-4 Dio1Mapping, 3-2 Dio2Mapping, 1-0 Dio3Mapping
    SX1276_REG_DioMapping2                 = 0x41, // 7-6 Dio4Mapping, 5-4 Dio5Mapping, 0 MapPreambleDetect
    SX1276_REG_Version                     = 0x42, // 7-0 Version
    SX1276_REG_Tcxo                        = 0x4B, // 4 TcxoInputOn
    SX1276_REG_PaDac                       = 0x4D, // 2-0 PaDac
    SX1276_REG_FormerTemp                  = 0x5B, // 7-0 FormerTemp
    SX1276_REG_AgcRef                      = 0x61, // 5-0 AgcReferenceLevel
    SX1276_REG_AgcThresh1                  = 0x62, // 4-0 AgcStep1,
    SX1276_REG_AgcThresh2                  = 0x63, // 7-4 AgcStep2, 3-0 AgcStep3
    SX1276_REG_AgcThresh3                  = 0x64, // 7-4 AgcStep4, 3-0 AgcStep5
    SX1276_REG_Pll                         = 0x70, // 7-6 PllBandwidth
} SX1276_REG_ENUM;

typedef enum {
    //-- LoRa Registers, applicable to only LoRa mode

    SX1276_REG_LORA_FifoAddrPtr            = 0x0D, // 7-0 FifoAddrPtr
    SX1276_REG_LORA_FifoTxBaseAddr         = 0x0E, // 7-0 FifoTxBaseAddr
    SX1276_REG_LORA_FifoRxBaseAddr         = 0x0F, // 7-0 FifoRxBaseAddr
    SX1276_REG_LORA_FifoRxCurrentAddr      = 0x10, // 7-0 FifoRxCurrentAddr
    SX1276_REG_LORA_IrqFlagsMask           = 0x11, // 7 RxTimeoutMask, 6 RxDoneMask, 5 PayloadCrcErrorMask, 4 ValidHeaderMask, 3 TxDoneMask, 2 CadDoneMask, 1 FhssChangeChannelMask, 0 CadDetectedMask
    SX1276_REG_LORA_IrqFlags               = 0x12, // 7 RxTimeout, 6 RxDone, 5 PayloadCrcError, 4 ValidHeader, 3 TxDone, 2 CadDone, 1 FhssChangeChannel, 0 CadDetected
    SX1276_REG_LORA_RxNbBytes              = 0x13, // 7-0 FifoRxBytesNb
    SX1276_REG_LORA_RxHeaderCntValueMsb    = 0x14, // 7-0 ValidHeaderCntMsb(15:8)
    SX1276_REG_LORA_RxHeaderCntValueLsb    = 0x15, // 7-0 ValidHeaderCntLsb(7:0)
    SX1276_REG_LORA_RxPacketCntValueMsb    = 0x16, // 7-0 ValidPacketCntMsb(15:8)
    SX1276_REG_LORA_RxPacketCntValueLsb    = 0x17, // 7-0 ValidPacketCntLsb(7:0)
    SX1276_REG_LORA_ModemStat              = 0x18, // 7-5 RxCodingRate, 4-0 ModemStatus
    SX1276_REG_LORA_PktSnrValue            = 0x19, // 7-0 PacketSnr
    SX1276_REG_LORA_PktRssiValue           = 0x1A, // 7-0 PacketRssi
    SX1276_REG_LORA_RssiValue              = 0x1B, // 7-0 Rssi
    SX1276_REG_LORA_HopChannel             = 0x1C, // 7 PllTimeout, 6 CrcOnPayload, 5-0 FhssPresentChannel
    SX1276_REG_LORA_ModemConfig1           = 0x1D, // 7-4 Bw, 3-1 CodingRate, 0 ImplicitHeaderModeOn
    SX1276_REG_LORA_ModemConfig2           = 0x1E, // 7-4 SpreadingFactor, 3 TxContinuousMode, 2 RxPayloadCrcOn, 1-0 SymbTimeout(9:8)
    SX1276_REG_LORA_SymbTimeoutLsb         = 0x1F, // 7-0 SymbTimeout(7:0)
    SX1276_REG_LORA_PreambleMsb            = 0x20, // 7-0 PreambleLength(15:8)
    SX1276_REG_LORA_PreambleLsb            = 0x21, // 7-0 PreambleLength(7:0)
    SX1276_REG_LORA_PayloadLength          = 0x22, // 7-0 PayloadLength(7:0)
    SX1276_REG_LORA_MaxPayloadLength       = 0x23, // 7-0 PayloadMaxLength(7:0)
    SX1276_REG_LORA_HopPeriod              = 0x24, // 7-0 FreqHoppingPeriod(7:0)
    SX1276_REG_LORA_FifoRxByteAddr         = 0x25, // 7-0 FifoRxByteAddrPtr
    SX1276_REG_LORA_ModemConfig3           = 0x26, // 3 LowDataRateOptimize, 2 AgcAutoOn
    SX1276_REG_LORA_0x27                   = 0x27, // 7-0 PpmCorrection
    SX1276_REG_LORA_FeiMsb                 = 0x28, // 3-0 FreqError(19:16)
    SX1276_REG_LORA_FeiMid                 = 0x29, // 7-0 FreqError(15:8)
    SX1276_REG_LORA_FeiLsb                 = 0x2A, // 7-0 FreqError(7:0)
    SX1276_REG_LORA_RssiWideband           = 0x2C, // 7-0 RssiWideband
    SX1276_REG_LORA_0x2F                   = 0x2F, // 7-0 IfFreq2, See errata note
    SX1276_REG_LORA_0x30                   = 0x30, // 7-0 IfFreq1, See errata note
    SX1276_REG_LORA_DetectOptimize         = 0x31, // 7 AutomaticIFOn, 2-0 DetectionOptimize
    SX1276_REG_LORA_InvertIQ               = 0x33, // 6 InvertIQ RX, 0 InvertIQ TX
    SX1276_REG_LORA_HighBWOptimize1        = 0x36, // 7-0 HighBWOptimize1, See errata note
    SX1276_REG_LORA_DetectionThreshold     = 0x37, // 7-0 DetectionThreshold
    SX1276_REG_LORA_SyncWord               = 0x39, // 7-0 SyncWord
    SX1276_REG_LORA_HighBWOptimize2        = 0x3A, // 7-0 HighBWOptimize2, See errata note
    SX1276_REG_LORA_InvertIQ2              = 0x3B, // 7-0 InvertIQ2
} SX1276_REG_LORA_ENUM;

typedef enum {
    //-- FSK Registers, applicable to only FSK mode

    SX1276_REG_FSK_BitrateMsb              = 0x02, // 7-0 BitRate(15:8)
    SX1276_REG_FSK_BitrateLsb              = 0x03, // 7-0 BitRate(7:0)
    SX1276_REG_FSK_FdevMsb                 = 0x04, // 5-0 Fdev(13:8)
    SX1276_REG_FSK_FdevLsb                 = 0x05, // 7-0 Fdev(7:0)
    SX1276_REG_FSK_RxConfig                = 0x0D, // 7 RestartRxOnCollision, 6 RestartRxWithoutPllLock, 5 RestartRxWithPllLock, 4 AfcAutoOn, 3 AgcAutoOn, 2-0 RxTrigger
    SX1276_REG_FSK_RssiConfig              = 0x0E, // 7-3 RssiOffset, 2-0 RssiSmoothing
    SX1276_REG_FSK_RssiCollision           = 0x0F, // 7-0 RssiCollisionThreshold
    SX1276_REG_FSK_RssiThresh              = 0x10, // 7-0 RssiThreshold
    SX1276_REG_FSK_RssiValue               = 0x11, // 7-0 RssiValue
    SX1276_REG_FSK_RxBw                    = 0x12, // 4-3 RxBwMant, 2-0 RxBwExp
    SX1276_REG_FSK_AfcBw                   = 0x13, // 4-3 RxBwMantAfc, 2-0 RxBwExpAfc
    SX1276_REG_FSK_OokPeak                 = 0x14, // 5 BitSyncOn, 4-3 OokThreshType, 2-0 OokPeakThreshStep
    SX1276_REG_FSK_OokFix                  = 0x15, // 7-0 OokFixedThreshold
    SX1276_REG_FSK_OokAvg                  = 0x16, // 7-5 OokPeakThreshDec, 3-2 OokAverageOffset, 1-0 OokAverageThreshFilt
    SX1276_REG_FSK_AfcFei                  = 0x1A, // 4 AgcStart, 1 AfcClear, 0 AfcAutoClearOn
    SX1276_REG_FSK_AfcMsb                  = 0x1B, // 7-0 AfcValue(15:8)
    SX1276_REG_FSK_AfcLsb                  = 0x1C, // 7-0 AfcValue(7:0)
    SX1276_REG_FSK_FeiMsb                  = 0x1D, // 7-0 FeiValue(15:8)
    SX1276_REG_FSK_FeiLsb                  = 0x1E, // 7-0 FeiValue(7:0)
    SX1276_REG_FSK_PreambleDetect          = 0x1F, // 7 PreambleDetectorOn, 6-5 PreambleDetectorSize, 4-0 PreambleDetectorTol
    SX1276_REG_FSK_RxTimeout1              = 0x20, // 7-0 TimeoutRxRssi
    SX1276_REG_FSK_RxTimeout2              = 0x21, // 7-0 TimeoutRxPreamble
    SX1276_REG_FSK_RxTimeout3              = 0x22, // 7-0 TimeoutSignalSync
    SX1276_REG_FSK_RxDelay                 = 0x23, // 7-0 InterPacketRxDelay
    SX1276_REG_FSK_Osc                     = 0x24, // 3 RcCalStart, 2-0 ClkOut
    SX1276_REG_FSK_PreambleMsb             = 0x25, // 7-0 PreambleSize(15:8)
    SX1276_REG_FSK_PreambleLsb             = 0x26, // 7-0 PreambleSize(7:0)
    SX1276_REG_FSK_SyncConfig              = 0x27, // 7-6 AutoRestartRxMode, 5 Preamble Polarity, 4 SyncOn, 2-0 SyncSize
    SX1276_REG_FSK_SyncValue1              = 0x28, // SyncValue(63:56)
    SX1276_REG_FSK_SyncValue2              = 0x29, // SyncValue(55:48)
    SX1276_REG_FSK_SyncValue3              = 0x2A, // SyncValue(47:40)
    SX1276_REG_FSK_SyncValue4              = 0x2B, // SyncValue(39:32)
    SX1276_REG_FSK_SyncValue5              = 0x2C, // SyncValue(31:24)
    SX1276_REG_FSK_SyncValue6              = 0x2D, // SyncValue(23:16)
    SX1276_REG_FSK_SyncValue7              = 0x2E, // SyncValue(15:8)
    SX1276_REG_FSK_SyncValue8              = 0x2F, // SyncValue(7:0)
    SX1276_REG_FSK_PacketConfig1           = 0x30, // 7 PacketFormat, 6-5 DcFree, 4 CrcOn, 3 CrcAutoClearOff, 2-1 AddressFiltering, 0 CrcWhiteningType
    SX1276_REG_FSK_PacketConfig2           = 0x31, // 6 DataMode, 5 IoHomeOn, 4 IoHomePowerFrame, 3 BeaconOn, 2-0 PayloadLength(10:8)
    SX1276_REG_FSK_PayloadLength           = 0x32, // 7-0 PayloadLength(7:0)
    SX1276_REG_FSK_NodeAdrs                = 0x33, // 7-0 NodeAddress
    SX1276_REG_FSK_BroadcastAdrs           = 0x34, // 7-0 BroadcastAddress
    SX1276_REG_FSK_FifoThresh              = 0x35, // 7 TxStartCondition, 5-0 FifoThreshold
    SX1276_REG_FSK_SeqConfig1              = 0x36, // 7 SequencerStart, 6 SequencerStop, 5 IdleMode, 4-3 FromStart, 2 LowPowerSelection, 1 FromIdle, 0 FromTransmit
    SX1276_REG_FSK_SeqConfig2              = 0x37, // 7-5 FromReceive, 4-3 FromRxTimeout, 2-0 FromPacketReceived
    SX1276_REG_FSK_TimerResol              = 0x38, // 3-2 Timer1Resolution, 1-0 Timer2Resolution
    SX1276_REG_FSK_Timer1Coef              = 0x39, // 7-0 Timer1Coefficient
    SX1276_REG_FSK_Timer2Coef              = 0x3A, // 7-0 Timer2Coefficient
    SX1276_REG_FSK_ImageCal                = 0x3B, // 7 AutoImageCalOn, 6 ImageCalStart, 5 ImageCalRunning, 3 TempChange, 2-1 TempThreshold, 0 TempMonitorOff
    SX1276_REG_FSK_RegTemp                 = 0x3C, // 7-0 TempValue
    SX1276_REG_FSK_RegLowBat               = 0x3D, // 3 LowBatOn, 2-0 LowBatTrim
    SX1276_REG_FSK_IrqFlags1               = 0x3E, // 7 ModeReady, 6 RxReady, 5 TxReady, 4 PllLock, 3 Rssi, 2 Timeout, 1 PreambleDetect, 0 SyncAddressMatch
    SX1276_REG_FSK_IrqFlags2               = 0x3F, // 7 FifoFull, 6 FifoEmpty, 5 FifoLevel, 4 FifoOverrun, 3 PacketSent, 2 PayloadReady, 1 CrcOk, 0 LowBat
    SX1276_REG_FSK_PllHop                  = 0x44, // 7 FastHopOn
    SX1276_REG_FSK_BitRateFrac             = 0x5D, // 3-0 BitRateFrac  
} SX1276_REG_FSK_ENUM;


//-------------------------------------------------------
// Enum Definitions Common Register Setting
//-------------------------------------------------------

// SX1276_REG_OpMode = 0x01

// 7 LongRangeMode
typedef enum {
    SX1276_PACKET_TYPE_FSK_OOK        = 0x00, // 0 : FSK/OOK Mode
    SX1276_PACKET_TYPE_LORA           = (1 << 7), // 1 : LoRa Mode
} SX1276_PACKET_TYPE_ENUM;

// 6 AccessSharedReg
typedef enum {
    SX1276_ACCESS_SHARED_REG_LORA     = 0x00, // 0 : Access LoRa registers page 0x0D: 0x3F
    SX1276_ACCESS_SHARED_REG_FSK      = (1 << 6), // 1 : Access FSK registers page (in mode LoRa) 0x0D: 0x3F
} SX1276_ACCESS_SHARED_REG_ENUM;

// 3 LowFrequencyModeOn, Access Low Frequency Mode registers
typedef enum {
    SX1276_LOW_FREQUENCY_MODE_OFF     = 0x00, // 0 : High Frequency Mode (access to HF test registers)
    SX1276_LOW_FREQUENCY_MODE_ON      = (1 << 3), // 1 : Low Frequency Mode (access to LF test registers)
} SX1276_LOW_FREQUENCY_MODE_ENUM;

// 2-0 Mode, Device modes
typedef enum {
    SX1276_MODE_SLEEP                 = 0x00, // 000 : SLEEP
    SX1276_MODE_STDBY                 = 0x01, // 001 : STDBY
    SX1276_MODE_FSTX                  = 0x02, // 010 : Frequency synthesis TX (FSTX)
    SX1276_MODE_TX                    = 0x03, // 011 : Transmit (TX)
    SX1276_MODE_FSRX                  = 0x04, // 100 : Frequency synthesis RX (FSRX)
    SX1276_MODE_RX_CONTINUOUS         = 0x05, // 101 : Receive continuous (RXCONTINUOUS)
    SX1276_MODE_RX_SINGLE             = 0x06, // 110 : receive single (RXSINGLE)
    SX1276_MODE_CAD                   = 0x07, // 111 : Channel activity detection (CAD)
} SX1276_MODE_ENUM;


// SX1276_REG_PaConfig = 0x09

// 7 PaSelect, Selects PA output pin
typedef enum {
    SX1276_PA_SELECT_RFO              = 0x00, // 0 : RFO pin. Output power is limited to +14 dBm.
    SX1276_PA_SELECT_PA_BOOST         = (1 << 7), // 1 : PA_BOOST pin. Output power is limited to +20 dBm
} SX1276_PA_SELECT_ENUM;

// 6-4 MaxPower, Select max output power: Pmax = 10.8 + 0.6 * MaxPower [dBm]
typedef enum {
    SX1276_MAX_POWER_10p8_DBM         = 0x00,
    SX1276_MAX_POWER_11p4_DBM         = (1 << 4),
    SX1276_MAX_POWER_12_DBM           = (2 << 4),
    SX1276_MAX_POWER_12p6_DBM         = (3 << 4),
    SX1276_MAX_POWER_13p2_DBM         = (4 << 4),
    SX1276_MAX_POWER_13p8_DBM         = (5 << 4),
    SX1276_MAX_POWER_14p4_DBM         = (6 << 4),
    SX1276_MAX_POWER_15_DBM           = (7 << 4),
} SX1276_MAX_POWER_ENUM;

// 3-0 OutputPower
// Pout = Pmax - (15 - OutputPower) if PaSelect = 0 (RFO pin)
// Pout = 17 - (15 - OutputPower) if PaSelect = 1 (PA_BOOST pin)
typedef enum {
    SX1276_OUTPUT_POWER_MIN           = 0,
    SX1276_OUTPUT_POWER_MAX           = 15,
} SX1276_OUTPUT_POWER_ENUM;


// SX1276_REG_PaRamp = 0x0A

// 3-0 PaRamp(3:0), Rise/Fall time of ramp up/down in FSK
typedef enum {
    SX1276_PA_RAMP_3p4_MS             = 0x00, // 0000 : 3.4 ms
    SX1276_PA_RAMP_2_MS               = 0x01, // 0001 : 2 ms
    SX1276_PA_RAMP_1_MS               = 0x02, // 0010 : 1 ms
    SX1276_PA_RAMP_500_US             = 0x03, // 0011 : 500 us
    SX1276_PA_RAMP_250_US             = 0x04, // 0100 : 250 us
    SX1276_PA_RAMP_125_US             = 0x05, // 0101 : 125 us
    SX1276_PA_RAMP_100_US             = 0x06, // 0110 : 100 us
    SX1276_PA_RAMP_62_US              = 0x07, // 0111 : 62 us
    SX1276_PA_RAMP_50_US              = 0x08, // 1000 : 50 us
    SX1276_PA_RAMP_40_US              = 0x09, // 1001 : 40 us
    SX1276_PA_RAMP_31_US              = 0x0A, // 1010 : 31 us
    SX1276_PA_RAMP_25_US              = 0x0B, // 1011 : 25 us
    SX1276_PA_RAMP_20_US              = 0x0C, // 1100 : 20 us
    SX1276_PA_RAMP_15_US              = 0x0D, // 1101 : 15 us
    SX1276_PA_RAMP_12_US              = 0x0E, // 1110 : 12 us
    SX1276_PA_RAMP_10_US              = 0x0F, // 1111 : 10 us
} SX1276_PA_RAMP_ENUM;


// SX1276_REG_Ocp = 0x0B

// 5 OcpOn, Enables overload current protection (OCP) for PA
typedef enum {
    SX1276_OCP_OFF                    = 0x00, // 0 : OCP disabled
    SX1276_OCP_ON                     = (1 << 5), // 1 : OCP enabled
} SX1276_OCP_ENUM;

// 4-0 OcpTrim
//  0 <= OcpTrim <= 15 (45 to 120 mA): Imax = 45 + 5 * OcpTrim [mA]
// 16 <= OcpTrim <= 27 (130 to 240 mA): Imax = -30 + 10 * OcpTrim [mA]
// 28 <= OcpTrim <= 31: Imax = 240mA
typedef enum {
    SX1276_OCP_TRIM_45_MA             = 0x00,
    SX1276_OCP_TRIM_50_MA             = 0x01, // 50 mA
    SX1276_OCP_TRIM_100_MA            = 0x0B, // 100 mA
    SX1276_OCP_TRIM_150_MA            = 0x12, // 150 mA
    SX1276_OCP_TRIM_240_MA            = 0x1F,
    SX1276_OCP_TRIM_MIN               = 0x00, // 45 mA
    SX1276_OCP_TRIM_DEFAULT           = 0x0B, // 100 mA
    SX1276_OCP_TRIM_MAX               = 0x1F, // 240 mA
} SX1276_OCP_TRIM_ENUM;


// SX1276_REG_Lna = 0x0C

// 7-5 LnaGain, LNA gain setting
typedef enum {
    SX1276_LNA_GAIN_G1                = (1 << 5), // 001 : G1 = maximum gain
    SX1276_LNA_GAIN_G2                = (2 << 5), // 010 : G2
    SX1276_LNA_GAIN_G3                = (3 << 5), // 011 : G3
    SX1276_LNA_GAIN_G4                = (4 << 5), // 100 : G4
    SX1276_LNA_GAIN_G5                = (5 << 5), // 101 : G5
    SX1276_LNA_GAIN_G6                = (6 << 5), // 110 : G6 = minimum gain
    SX1276_LNA_GAIN_DEFAULT           = SX1276_LNA_GAIN_G1,
} SX1276_LNA_GAIN_ENUM;

// 4-3 LnaBoostLf, Low Frequency (RFI_LF) LNA current adjustment
typedef enum {
    SX1276_LNA_BOOST_LF_DEFAULT       = 0x00, // 00 : Default LNA current
} SX1276_LNA_BOOST_LF_ENUM;

// 1-0 LnaBoostHf, High Frequency (RFI_HF) LNA current adjustment
typedef enum {
    SX1276_LNA_BOOST_HF_DEFAULT       = 0x00, // 00 : Default LNA current
    SX1276_LNA_BOOST_HF_ON            = 0x03, // 11 : Boost on, 150% LNA current
} SX1276_LNA_BOOST_HF_ENUM;


// Mapping of pins DIO0 to DIO5
// See Table 18 for mapping in LoRa mode

// SX1276_REG_DioMapping1 = 0x40

// 7-6 Dio0Mapping 
typedef enum {
    SX1276_DIO0_MAPPING_RX_DONE                 = 0x00,
    SX1276_DIO0_MAPPING_TX_DONE                 = (1 << 6),
    SX1276_DIO0_MAPPING_CAD_DONE                = (2 << 6),
    // from ExpressLRS:
    // undocumented "hack", looking at Table 18 from datasheet SX127X_REG_DIO_MAPPING_1 = 11 appears to be
    // unspported by infact it generates an intterupt on both RXdone and TXdone, this saves switching modes.
    SX1276_DIO0_MAPPING_RX_TX_DONE              = (3 << 6),
} SX1276_DIO0_MAPPING_ENUM;

// 5-4 Dio1Mapping
typedef enum {
    SX1276_DIO1_MAPPING_RX_TIMEOUT              = 0x00,
    SX1276_DIO1_MAPPING_FHSS_CHANGE_CHANNEL     = (1 << 4),
    SX1276_DIO1_MAPPING_CAD_DETECTED            = (2 << 4),
} SX1276_DIO1_MAPPING_ENUM;

// 3-2 Dio2Mapping
typedef enum {
    SX1276_DIO2_MAPPING_FHSS_CHANGE_CHANNEL     = 0x00,
} SX1276_DIO2_MAPPING_ENUM;

// 1-0 Dio3Mapping
typedef enum {
    SX1276_DIO3_MAPPING_CAD_DONE                = 0x00,
    SX1276_DIO3_MAPPING_VALID_HEADER            = 0x01,
    SX1276_DIO3_MAPPING_PAYLOAD_CRC_ERROR       = 0x02,
} SX1276_DIO3_MAPPING_ENUM; 

// SX1276_REG_DioMapping2 = 0x41

// 7-6 Dio4Mapping
typedef enum {
    SX1276_DIO4_MAPPING_CAD_DETECTED            = 0x00,
    SX1276_DIO4_MAPPING_PLL_LOCK                = (1 << 6),
} SX1276_DIO4_MAPPING_ENUM;

// 5-4 Dio5Mapping
typedef enum {
    SX1276_DIO5_MAPPING_MODE_READY              = 0x00,
    SX1276_DIO5_MAPPING_CLKOUT                  = (1 << 4),
} SX1276_DIO5_MAPPING_ENUM;

// 0 MapPreambleDetect, Allows the mapping of either Rssi Or PreambleDetect to the DIO pins, as summarized on Table 29 and Table 30
typedef enum {
    SX1276_MAP_PREAMBLE_DETECT_IRQ_RSSI             = 0x00, // 0 : Rssi interrupt
    SX1276_MAP_PREAMBLE_DETECT_IRQ_PREAMBLE_DETECT  = 0x01, // 1 : PreambleDetect interrupt
} SX1276_MAP_PREAMBLE_DETECT_ENUM;


// SX1276_REG_Tcxo = 0x4B

// 4 TcxoInputOn, Controls the crystal oscillator
typedef enum {
    SX1276_TCXO_INPUT_NORMAL          = 0x00, // 0 : Crystal Oscillator with external Crystal
    SX1276_TCXO_INPUT_CLIPPED         = (1 << 4), // 1 : External clipped sine TCXO AC-connected to XTA pin
} SX1276_TCXO_INPUT_ENUM;    


// SX1276_REG_PaDac = 0x4D

// 2-0 PaDac, Enables the +20dBm option on PA_BOOST pin
typedef enum {
    SX1276_PA_DAC_DEFAULT             = 0x04, // 0x04 : Default value
    SX1276_PA_DAC_BOOST               = 0x07, // 0x07 : +20dBm on PA_BOOST when OutputPower = 1111
} SX1276_PA_DAC_ENUM;


// SX1276_REG_PllLf = 0x70
// SX1276_REG_PllHf = 0x70

// 7-6 PllBandwidth, Controls the PLL bandwidth:
typedef enum {
    SX1276_PLL_BW_75_KHZ              = 0x00, // 00 : 75 kHz
    SX1276_PLL_BW_150_KHZ             = (1 << 6), // 01 : 150 kHz
    SX1276_PLL_BW_225_KHZ             = (2 << 6), // 10 : 225 kHz
    SX1276_PLL_BW_300_KHZ             = (3 << 6), // 11 : 300 kHz
} SX1276_PLL_BW_ENUM;


//-------------------------------------------------------
// Enum Definitions Lora Page Registers
//-------------------------------------------------------

// SX1276_REG_IrqFlagsMask = 0x11
// SX1276_REG_IrqFlags = 0x12
typedef enum {
    SX1276_IRQ_CAD_DETECTED           = 0x01, // 0 CadDetected, Valid Lora signal detected during CAD operation
    SX1276_IRQ_FHSS_CHANGE_CHANNEL    = 0x02, // 1 FhssChangeChannel, FHSS change channel interrupt
    SX1276_IRQ_CAD_DONE               = 0x04, // 2 CadDone, CAD complete: write to clear
    SX1276_IRQ_TX_DONE                = 0x08, // 3 TxDone, FIFO Payload transmission complete interrupt
    SX1276_IRQ_VALID_HEADER           = 0x10, // 4 ValidHeader, Valid header received in Rx
    SX1276_IRQ_PAYLOAD_CRC_ERROR      = 0x20, // 5 PayloadCrcError, Payload CRC error interrupt
    SX1276_IRQ_RX_DONE                = 0x40, // 6 RxDone, Packet reception complete interrupt
    SX1276_IRQ_RX_TIMEOUT             = 0x80, // 7 RxTimeout, Timeout interrupt
    SX1276_IRQ_ALL                    = 0xFF,
} SX1276_LORA_IRQ_ENUM;


// SX1276_REG_ModemStat = 0x18

// 4-0 ModemStatus: 4 Modem clear 3 Header info valid, 2 RX on-going, 1 Signal synchronized, 0 Signal detected
typedef enum {
    SX1276_MODEM_STAT_SIGNAL_DETECTED           = 0x01, // 0 : Signal detected
    SX1276_MODEM_STAT_SIGNAL_SYNCHRONIZED       = 0x02, // 1 : Signal synchronized
    SX1276_MODEM_STAT_RX_ONGOING                = 0x04, // 2 : RX on-going
    SX1276_MODEM_STAT_HEADER_INFO_VALID         = 0x08, // 3 : Header info valid
    SX1276_MODEM_STAT_MODEM_CLEAR               = 0x10, // 4 : Modem clear
    SX1276_MODEM_STAT_MASK                      = 0x1F,
} SX1276_LORA_MODEM_STAT_ENUM;


// SX1276_REG_HopChannel = 0x1C

// 7 PllTimeout, PLL failed to lock while attempting a TX/RX/CAD operation
typedef enum {
    SX1276_LORA_PLL_TIMEOUT_LOCK                = 0x00, // 0 : PLL did lock
    SX1276_LORA_PLL_TIMEOUT_NO_LOCK             = (1 << 7), // 1 : PLL did not lock
} SX1276_LORA_PLL_TIMEOUT_ENUM;

// 6 CrcOnPayload, CRC Information extracted from the received packet header (Explicit header mode only)
typedef enum {
    SX1276_LORA_HEADER_PAYLOAD_CRC_OFF          = 0x00, // 0 : Header indicates CRC off
    SX1276_LORA_HEADER_PAYLOAD_CRC_ON           = (1 << 6), // 1 : Header indicates CRC on
} SX1276_LORA_HEADER_PAYLOAD_CRC_ENUM;


// SX1276_REG_ModemConfig1 = 0x1D

// 7-4 Bw, Signal bandwidth
typedef enum {
    SX1276_LORA_BW_7p8                = 0x00, // 0000 : 7.8 kHz
    SX1276_LORA_BW_10p4               = (1 << 4), // 0001 : 10.4 kHz
    SX1276_LORA_BW_15p6               = (2 << 4), // 0010 : 15.6 kHz
    SX1276_LORA_BW_20p8               = (3 << 4), // 0011 : 20.8kHz
    SX1276_LORA_BW_31p25              = (4 << 4), // 0100 : 31.25 kHz
    SX1276_LORA_BW_41p7               = (5 << 4), // 0101 : 41.7 kHz
    SX1276_LORA_BW_62p5               = (6 << 4), // 0110 : 62.5 kHz
    SX1276_LORA_BW_125                = (7 << 4), // 0111 : 125 kHz
    SX1276_LORA_BW_250                = (8 << 4), // 1000 : 250 kHz
    SX1276_LORA_BW_500                = (9 << 4), // 1001 : 500 kHz
} SX1276_LORA_BW_ENUM;

// 3-1 CodingRate, Error coding rate
typedef enum {
    SX1276_LORA_CR_4_5                = (1 << 1), // 001 : 4/5
    SX1276_LORA_CR_4_6                = (2 << 1), // 010 : 4/6
    SX1276_LORA_CR_4_7                = (3 << 1), // 011 : 4/7
    SX1276_LORA_CR_4_8                = (4 << 1), // 100 : 4/8
} SX1276_LORA_CR_ENUM;

// 0 ImplicitHeaderModeOn
typedef enum {
    SX1276_LORA_HEADER_EXPLICIT       = 0x00, // 0 : Explicit Header mode
    SX1276_LORA_HEADER_IMPLICIT       = 0x01, // 1 : Implicit Header mode
    SX1276_LORA_HEADER_DISABLE        = SX1276_LORA_HEADER_IMPLICIT,
    SX1276_LORA_HEADER_ENABLE         = SX1276_LORA_HEADER_EXPLICIT,
} SX1276_LORA_HEADER_ENUM;


// SX1276_REG_ModemConfig2 = 0x1E

// 7-4 SpreadingFactor, SF rate (expressed as a base-2 logarithm)
typedef enum {
    SX1276_LORA_SF6                   = (6 << 4), // 6 : 64 chips / symbol
    SX1276_LORA_SF7                   = (7 << 4), // 7 : 128 chips / symbol
    SX1276_LORA_SF8                   = (8 << 4), // 8 : 256 chips / symbol
    SX1276_LORA_SF9                   = (9 << 4), // 9 : 512 chips / symbol
    SX1276_LORA_SF10                  = (10 << 4), // 10 : 1024 chips / symbol
    SX1276_LORA_SF11                  = (11 << 4), // 11 : 2048 chips / symbol
    SX1276_LORA_SF12                  = (12 << 4), // 12 : 4096 chips / symbol
} SX1276_LORA_SF_ENUM;

// 3 TxContinuousMode
typedef enum {
    SX1276_LORA_TX_MODE_NORMAL        = 0x00, // 0 : normal mode, a single packet is sent
    SX1276_LORA_TX_MODE_CONTINUOUS    = (1 << 3), // 1 : continuous mode, send multiple packets across the FIFO (used for spectral analysis)
} SX1276_LORA_TX_MODE_ENUM;

// 2 RxPayloadCrcOn, Enable CRC generation and check on payload
typedef enum {
    SX1276_LORA_CRC_DISABLE           = 0x00, // 0 : CRC disable
    SX1276_LORA_CRC_ENABLE            = (1 << 2), // 1 : CRC enable
} SX1276_LORA_CRC_ENUM;


// SX1276_REG_ModemConfig3 = 0x26

// 3 LowDataRateOptimize
typedef enum {
    SX1276_LORA_LOW_DATA_RATE_OPTIMIZE_OFF      = 0x00, // 0 : Disabled
    SX1276_LORA_LOW_DATA_RATE_OPTIMIZE_ON       = (1 << 3), // 1 : Enabled; mandated for when the symbol length exceeds 16ms
} SX1276_LORA_LOW_DATA_RATE_OPTIMIZE_ENUM;

// 2 AgcAutoOn
typedef enum {
    SX1276_LORA_AGC_AUTO_OFF                    = 0x00, // 0 : LNA gain set by register LnaGain
    SX1276_LORA_AGC_AUTO_ON                     = (1 << 2), // 1 : LNA gain set by the internal AGC loop
} SX1276_LORA_AGC_AUTO_ENUM;


// SX1276_REG_DetectOptimize = 0x31

// 7 AutomaticIFOn, Should be set to 0x0 after each reset (POR on manual) See errata note for more information
typedef enum {
    SX1276_LORA_AUTOMATIC_IF_OFF                = 0x00,
    SX1276_LORA_AUTOMATIC_IF_ON                 = (1 << 7),
} SX1276_LORA_AUTOMATIC_IF_ENUM;

// 2-0 DetectionOptimize, LoRa Detection Optimize
typedef enum {
    SX1276_LORA_DETECTION_OPTIMIZE_SF_6         = 0x05, // 0x05 : SF6
    SX1276_LORA_DETECTION_OPTIMIZE_SF_7_12      = 0x03, // 0x03 : SF7 to SF12
} SX1276_LORA_DETECTION_OPTIMIZE_ENUM;


// SX1276_REG_InvertIQ = 0x33
// I found that setting e.g. both RX and TX to zero makes it to fail!
// ELRS sets only bit6 !
// https://github.com/sandeepmistry/arduino-LoRa/issues/177 sets the register to 0x27 or 0x66,
// which means toggling the bits as 01 <-> 10
// also, what's up with SX1276_REG_InvertIQ2???
// I have tested all 16 combinations, result:
// Transmit 01 or 11 work with Rx 01 or 00
// Tx 00 or 10 work with Rx 10 or 11
// => the ELRS approach does not work for me ??
// => use toggle approach
/*
// 6 InvertIQ RX, Invert the LoRa I and Q signals in RX path
typedef enum {
    SX1276_LORA_IQ_RX_NORMAL          = 0x00, // 0 : normal mode
    SX1276_LORA_IQ_RX_INVERTED        = (1 << 6), // 1 : I and Q signals are inverted
} SX1276_LORA_IQ_RX_ENUM;

// 0 InvertIQ TX, Invert the LoRa I and Q signals in TX path
typedef enum {
    SX1276_LORA_IQ_TX_NORMAL          = 0x00, // 0 : normal mode
    SX1276_LORA_IQ_TX_INVERTED        = 0x01, // 1 : I and Q signals are inverted
} SX1276_LORA_IQ_TX_ENUM;
*/
typedef enum {
    SX1276_LORA_IQ_NORMAL             = 0x01, // 0b 0000 0001
    SX1276_LORA_IQ_INVERTED           = 0x40, // 0b 0100 0000
} SX1276_LORA_IQ_ENUM;


// SX1276_REG_DetectionThreshold = 0x37

// 7-0 DetectionThreshold, LoRa detection threshold
typedef enum {
    SX1276_LORA_DETECTION_TRESHOLD_SF_6         = 0x0C, // 0x0C : SF6
    SX1276_LORA_DETECTION_TRESHOLD_SF_7_12      = 0x0A, // 0x0A : SF7 to SF12
} SX1276_LORA_DETECTION_TRESHOLD_ENUM;


// SX1276_REG_InvertIQ2 = 0x3B

// 7-0 InvertIQ2, Set to 0x19 for inverted IQ
typedef enum {
    SX1276_LORA_IQ2_NORMAL            = 0x1D,
    SX1276_LORA_IQ2_INVERTED          = 0x19,
} SX1276_LORA_IQ2_ENUM;


#endif // SX127X_LIB_H







