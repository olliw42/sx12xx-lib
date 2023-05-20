//*******************************************************
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// SX128x library
//*******************************************************
/*
concerning Tx lock ups: https://forum.arduino.cc/t/lora-spi-corruption/614013/16
*/
#ifndef SX128X_LIB_H
#define SX128X_LIB_H
#pragma once

#include <inttypes.h>

#define SX1280_FREQ_XTAL_HZ               52000000

// this is what is suggested by Semtech
// #include <math.h>
// #define SX1280_FREQ_STEP                  ((double)(SX1280_FREQ_XTAL_HZ / pow(2.0,18.0))) // 198.3642578
// #define SX1280_FREQ_HZ_TO_REG(f_hz)       ((uint32_t)( (double)f_hz / (double)SX1280_FREQ_STEP ))
// the pow(x,y) is ugly, and using GHz units is more convenient

#define SX1280_FREQ_GHZ_TO_REG(f_ghz)     (uint32_t)((double)f_ghz*1.0E9*(double)(1 << 18)/(double)SX1280_FREQ_XTAL_HZ)


// not documented in the datasheet, but in Semtech code examples
#define SX1280_REG_FIRMWARE_VERSION_MSB   0x0153 // address of the register holding firmware version MSB


#define SX1280_POWER_DBM_TO_REG(dbm)      (uint8_t)( (int8_t)dbm + 18 ) // -18 dbm = power 0, 13 dbm = power 31


#ifndef ALIGNED
#define ALIGNED  __attribute__((aligned(4)))
#endif


#define SX128X_SPI_BUF_SIZE               256 // this must hold the max payload plus additional bytes


//-------------------------------------------------------
// Base Class
//-------------------------------------------------------

class Sx128xDriverBase
{
  public:
    Sx128xDriverBase() {} // constructor

    // this you will have to fill in the derived class

    void Init(void) {};

    // these you have to supply in the derived class

    virtual void SpiSelect(void) = 0;
    virtual void SpiDeselect(void) = 0;
    virtual void SpiTransfer(uint8_t* dataout, uint8_t* datain, uint8_t len) = 0;

    virtual void WaitOnBusy(void) {};
    virtual void SetDelay(uint16_t tmo_us) { (void)tmo_us; };

    // low level methods, usually no need to use them

    void SpiTransfer(uint8_t data, uint8_t* datain) { SpiTransfer(&data, datain, 1); }
    void SpiTransfer(uint8_t data) { uint8_t dummy; SpiTransfer(&data, &dummy, 1); }

    void WriteCommand(uint8_t opcode, uint8_t* data, uint8_t len);
    void ReadCommand(uint8_t opcode, uint8_t* data, uint8_t len);
    void WriteRegister(uint16_t adr, uint8_t* data, uint8_t len);
    void ReadRegister(uint16_t adr, uint8_t* data, uint8_t len);
    void WriteBuffer(uint8_t offset, uint8_t* data, uint8_t len);
    void ReadBuffer(uint8_t offset, uint8_t* data, uint8_t len);

    void WriteCommand(uint8_t opcode) { WriteCommand(opcode, nullptr, 0); }
    void WriteCommand(uint8_t opcode, uint8_t data) { WriteCommand(opcode, &data, 1); }
    void WriteRegister(uint16_t adr, uint8_t data) { WriteRegister(adr, &data, 1); }
    uint8_t ReadRegister(uint16_t adr) { uint8_t data; ReadRegister(adr, &data, 1); return data; }

    // common methods

    uint8_t GetStatus(void);
    void SetStandby(uint8_t StandbyConfig);
    void SetPacketType(uint8_t PacketType);
    void SetRfFrequency(uint32_t RfFrequency); // 24 bits only
    void SetBufferBaseAddress(uint8_t txBaseAdress, uint8_t rxBaseAdress);
    void SetModulationParams(uint8_t SpreadingFactor, uint8_t Bandwidth, uint8_t CodingRate);
    void SetPacketParams(uint8_t PreambleLength, uint8_t HeaderType, uint8_t PayloadLength, uint8_t Crc, uint8_t InvertIQ);
    void SetModulationParamsFLRC(uint8_t Bandwidth, uint8_t CodingRate, uint8_t Bt);
    void SetPacketParamsFLRC(uint8_t AGCPreambleLength, uint8_t PacketType, uint8_t PayloadLength, int16_t CrcSeed, uint32_t SyncWord, uint8_t CodingRate);

    void SetDioIrqParams(uint16_t IrqMask, uint16_t Dio1Mask, uint16_t Dio2Mask, uint16_t Dio3Mask);
    uint16_t GetIrqStatus(void);
    void ClearIrqStatus(uint16_t IrqMask);
    uint16_t GetAndClearIrqStatus(uint16_t IrqMask);

    // Tx methods

    void SetTxParams(uint8_t Power, uint8_t RampTime);
    void SetTx(uint8_t PeriodBase, uint16_t PeriodBaseCount);

    // Rx methods

    void SetRx(uint8_t PeriodBase, uint16_t PeriodBaseCount);
    void GetPacketStatus(int16_t* RssiSync, int8_t* Snr);
    void GetRxBufferStatus(uint8_t* rxPayloadLength, uint8_t* rxStartBufferPointer);

    // auxiliary methods

    void SetRegulatorMode(uint8_t RegModeParam);
    void SetAutoFs(uint8_t flag);
    void SetFs(void);
    void SetLnaGainMode(uint8_t LnaGainMode);

    // what else we like to have

    uint16_t GetFirmwareRev(void);
    void SetSyncWord(uint8_t SyncWord); // experimental
    uint32_t GetFrequencyErrorIndicator(void);

    uint8_t GetLastStatus(void) { return _status; }

  private:
    uint8_t _status; // all spi transfers yield the status, so we can just get it
};


//-------------------------------------------------------
// Enum Definitions
//-------------------------------------------------------

// WriteCommand(uint8_t opcode, uint8_t* data, uint8_t len)
typedef enum {
    SX1280_CMD_GET_STATUS                 = 0xC0, // table 12-1, p. 99
    SX1280_CMD_WRITE_REGISTER             = 0x18,
    SX1280_CMD_READ_REGISTER              = 0x19,
    SX1280_CMD_WRITE_BUFFER               = 0x1A,
    SX1280_CMD_READ_BUFFER                = 0x1B,
    SX1280_CMD_SET_SLEEP                  = 0x84,
    SX1280_CMD_SET_STANDBY                = 0x80,
    SX1280_CMD_SET_FS                     = 0xC1,
    SX1280_CMD_SET_TX                     = 0x83,
    SX1280_CMD_SET_RX                     = 0x82,
    SX1280_CMD_SET_RX_DUTYCYCLE           = 0x94,
    SX1280_CMD_SET_CAD                    = 0xC5,
    SX1280_CMD_SET_TX_CONTINUOUSWAVE      = 0xD1,
    SX1280_CMD_SET_TX_CONTINUOUSPREAMBLE  = 0xD2,
    SX1280_CMD_SET_PACKET_TYPE            = 0x8A,
    SX1280_CMD_GET_PACKET_TYPE            = 0x03,
    SX1280_CMD_SET_RF_FREQUENCY           = 0x86,
    SX1280_CMD_SET_TX_PARAMS              = 0x8E,
    SX1280_CMD_SET_CAD_PARAMS             = 0x88,
    SX1280_CMD_SET_BUFFER_BASEADDRESS     = 0x8F,
    SX1280_CMD_SET_MODULATION_PARAMS      = 0x8B,
    SX1280_CMD_SET_PACKET_PARAMS          = 0x8C,
    SX1280_CMD_GET_RX_BUFFER_STATUS       = 0x17,
    SX1280_CMD_GET_PACKET_STATUS          = 0x1D,
    SX1280_CMD_GET_RSSI_INST              = 0x1F,
    SX1280_CMD_SET_DIOIRQ_PARAMS          = 0x8D,
    SX1280_CMD_GET_IRQ_STATUS             = 0x15,
    SX1280_CMD_CLR_IRQ_STATUS             = 0x97,
    SX1280_CMD_SET_REGULATOR_MODE         = 0x96,
    SX1280_CMD_SET_SAVECONTEXT            = 0xD5,
    SX1280_CMD_SET_AUTOFS                 = 0x9E,
    SX1280_CMD_SET_AUTOTX                 = 0x98,
    SX1280_CMD_SET_LONG_PREAMBLE          = 0x9B,
    SX1280_CMD_SET_UART_SPEED             = 0x9D,
    SX1280_CMD_SET_RANGING_ROLE           = 0xA3,
    SX1280_CMD_SET_ADVANCED_RANGING       = 0x9A,
    // not in the datasheet table
    SX1280_CMD_CALIBRATE                  = 0x89,
} SX1280_CMD_ENUM;


// cmd 0x18 WriteRegister(uint16_t adr, uint8_t data, uint8_t len)
// cmd 0x19 ReadRegister(uint16_t adr, uint8_t* data, uint8_tlen)
typedef enum {
    SX1280_REG_RxGain                     = 0x891, // 0:7 rw 0x25 Register determining the LNA gain regime
    SX1280_REG_ManualGainSetting          = 0x895, // 0:7 rw 0x01 Manual Gain (See Section 4.2)
    SX1280_REG_LNAGainValue               = 0x89E, // 0:7 rw 0x0A The LNA gain value (See Section 4.2)
    SX1280_REG_LNAGainControl             = 0x89F, // 0:7 rw 0x4D Enable/Disable manual LNA gain control
    SX1280_REG_SynchPeakAttenuation       = 0x8C2, // 5:3 rw 0x4 dB Attenuation of the peak power during synch address.
    SX1280_REG_PayloadLength              = 0x901, // 0:7 rw - The length of the received LoRa payload
    SX1280_REG_LoRaHeaderMode             = 0x903, // 7 rw - Indicates the LoRa modem header mode
    SX1280_REG_RangingRequestAddress      = 0x912, // 4 bytes
    SX1280_REG_RangingDeviceAddress       = 0x916, // 4 bytes
    SX1280_REG_RangingFilterWindowSize    = 0x91E, // 0:7 r - The number of ranging samples over which the RSSI evaluated and the results averaged.
    SX1280_REG_ResetRangingFilter         = 0x923, // 6 w - Clears the samples stored in the ranging filter.
    SX1280_REG_RangingResultMUX           = 0x924, // 4:5 rw 0x3 Ranging result configuration.
    SX1280_REG_SFAdditionalConfiguration  = 0x925, // 0:7 rw - SF range selection in LoRa mode
    SX1280_REG_RangingCalibration         = 0x92B, // 3 bytes
    SX1280_REG_RangingIDCheckLength       = 0x931, // 0:7 rw 0x03 The number of bytes of the Ranging Slave ID that are checked.
    SX1280_REG_FrequencyErrorCorrection   = 0x93C, // 0:2 rw - Crystal frequency error correction mode
    SX1280_REG_LoRaSynchWord              = 0x944, // 2 bytes
    SX1280_REG_FEI                        = 0x954, // 3 bytes  LoRa Frequency error indicator (FEI)1
    SX1280_REG_RangingResult              = 0x961, // 3 bytes The result of the last ranging exchange.
    SX1280_REG_RangingRSSI                = 0x964, // 0:7 rw NA The RSSI value of the last ranging exchange
    SX1280_REG_FreezeRangingResult        = 0x97F, // 1 rw - Set to preserve the ranging result for reading
    SX1280_REG_PacketPreambleSettings     = 0x9C1, // 4:6 rw 0x00 Preamble length in GFSK and BLE:
    SX1280_REG_WhiteningInitialValue      = 0x9C5, // 0:7 rw 0x01 Data whitening seed for GFSK and BLE modulation.
    SX1280_REG_CRCPolynomialDefinition    = 0x9C6, // 2 bytes CRC Polynomial Definition for GFSK.
    SX1280_REG_CRCPolynomialSeed          = 0x9C7, // 3 bytes CRC Seed for BLE modulation.
    SX1280_REG_CRCInitialValue            = 0x9C8, // 2 bytes CRC Seed used for GFSK and FLRC modulation.
    SX1280_REG_SynchAddressControl        = 0x9CD, // 0:3 rw 0x80 The number of synch word bit errors tolerated in FLRC and GFSK modes
    SX1280_REG_SyncAddress1               = 0x9CE, // 5 bytes Synch Word 1 (Also used as the BLE Access Address)
    SX1280_REG_SyncAddress2               = 0x9D3, // 5 bytes SyncWord 2
    SX1280_REG_SyncAddress3               = 0x9D8, // 5 bytes SyncWord 3
    SX1280_REG_FLRCSyncWord               = 0x9CF, // 4 bytes SyncWord 1
} SX1280_REG_ENUM;


// cmd 0xC0 uint8_t GetStatus(void)
typedef enum {
    SX1280_STATUS_MODE_STDBY_RC           = 0x02 << 5, // table 11-5, p. 73
    SX1280_STATUS_MODE_STDBY_XOSC         = 0x03 << 5,
    SX1280_STATUS_MODE_FS                 = 0x04 << 5,
    SX1280_STATUS_MODE_RX                 = 0x05 << 5,
    SX1280_STATUS_MODE_TX                 = 0x06 << 5,
    SX1280_STATUS_MODE_MASK               = 0x07 << 5,
} SX1280_STATUS_MODE_ENUM;

typedef enum {
    SX1280_STATUS_CMD_PROCESSED           = 0x01 << 2, // table 11-5, p. 73
    SX1280_STATUS_CMD_DATA_AVAILABLE      = 0x02 << 2,
    SX1280_STATUS_CMD_TIMEOUT             = 0x03 << 2,
    SX1280_STATUS_CMD_PROCESSING_ERROR    = 0x04 << 2,
    SX1280_STATUS_CMD_EXEC_FAILURE        = 0x05 << 2,
    SX1280_STATUS_CMD_TX_DONE             = 0x06 << 2,
    SX1280_STATUS_CMD_MASK                = 0x07 << 2,
} SX1280_STATUS_CMD_ENUM;


//-------------------------------------------------------
// Enum Definitions Common
//-------------------------------------------------------

// cmd 0x80 SetStandby(uint8_t StandbyConfig)
typedef enum {
    SX1280_STDBY_CONFIG_STDBY_RC          = 0x00, // table 11-20, p. 78
    SX1280_STDBY_CONFIG_STDBY_XOSC        = 0x01
} SX1280_STDBY_CONFIG_ENUM;


// cmd 0x8A SetPacketType(uint8_t PacketType)
typedef enum {
    SX1280_PACKET_TYPE_GFSK               = 0x00, // table 11-42, p. 86
    SX1280_PACKET_TYPE_LORA               = 0x01,
    SX1280_PACKET_TYPE_RANGING            = 0x02,
    SX1280_PACKET_TYPE_FLRC               = 0x03,
    SX1280_PACKET_TYPE_BLE                = 0x04,
} SX1280_PACKET_TYPE_ENUM;


// cmd 0x86 SetRfFrequency(uint32_t RfFrequency) // 24 bits only
// cmd 0x8F SetBufferBaseAddress(uint8_t txBaseAdress, uint8_t rxBaseAdress)


// cmd 0x8B SetModulationParams(uint8_t SpreadingFactor, uint8_t Bandwidth, uint8_t CodingRate)
typedef enum {
    SX1280_LORA_SF5                       = 0x50, // table 14-47, p. 130
    SX1280_LORA_SF6                       = 0x60,
    SX1280_LORA_SF7                       = 0x70,
    SX1280_LORA_SF8                       = 0x80,
    SX1280_LORA_SF9                       = 0x90,
    SX1280_LORA_SF10                      = 0xA0,
    SX1280_LORA_SF11                      = 0xB0,
    SX1280_LORA_SF12                      = 0xC0,
} SX1280_LORA_SF_ENUM;

typedef enum {
    SX1280_LORA_BW_200                    = 0x34, // table 14-48, p. 131
    SX1280_LORA_BW_400                    = 0x26,
    SX1280_LORA_BW_800                    = 0x18,
    SX1280_LORA_BW_1600                   = 0x0A,
} SX1280_LORA_BW_ENUM;

typedef enum {
    SX1280_LORA_CR_4_5                    = 0x01, // table 14-49, p. 131
    SX1280_LORA_CR_4_6                    = 0x02,
    SX1280_LORA_CR_4_7                    = 0x03,
    SX1280_LORA_CR_4_8                    = 0x04,
    SX1280_LORA_CR_LI_4_5                 = 0x05,
    SX1280_LORA_CR_LI_4_6                 = 0x06,
    SX1280_LORA_CR_LI_4_7                 = 0x07,
} SX1280_LORA_CR_ENUM;


// cmd 0x8C SetPacketParams(uint8_t PreambleLength, uint8_t HeaderType, uint8_t PayloadLength, uint8_t CRC, uint8_t InvertIQ)
typedef enum {
    SX1280_LORA_HEADER_EXPLICIT           = 0x00, // table 14-51, p. 132
    SX1280_LORA_HEADER_IMPLICIT           = 0x80,
    SX1280_LORA_HEADER_DISABLE            = SX1280_LORA_HEADER_IMPLICIT,
    SX1280_LORA_HEADER_ENABLE             = SX1280_LORA_HEADER_EXPLICIT,
} SX1280_LORA_HEADER_ENUM;

typedef enum {
    SX1280_LORA_CRC_DISABLE               = 0x00, // table 14-53, p. 132
    SX1280_LORA_CRC_ENABLE                = 0x20,
} SX1280_LORA_CRC_ENUM;

typedef enum {
    SX1280_LORA_IQ_NORMAL                 = 0x40, // table 14-54, p. 133
    SX1280_LORA_IQ_INVERTED               = 0x00,
} SX1280_LORA_IQMODE_ENUM;


// cmd 0x8B SetModulationParamsFLRC(uint8_t Bandwidth, uint8_t CodingRate, uint8_t Bt)
typedef enum
{
    SX1280_FLRC_BR_1_300_BW_1_2 = 0x45,
    SX1280_FLRC_BR_1_000_BW_1_2 = 0x69,
    SX1280_FLRC_BR_0_650_BW_0_6 = 0x86,
    SX1280_FLRC_BR_0_520_BW_0_6 = 0xAA,
    SX1280_FLRC_BR_0_325_BW_0_3 = 0xC7,
    SX1280_FLRC_BR_0_260_BW_0_3 = 0xEB,
} SX1280_FLRC_BW_ENUM;

typedef enum
{
    SX1280_FLRC_CR_1_2 = 0x00,
    SX1280_FLRC_CR_3_4 = 0x02,
    SX1280_FLRC_CR_1_0 = 0x04,
} SX1280_FLRC_CR_ENUM;

typedef enum
{
    SX1280_FLRC_BT_DIS  = 0x00,
    SX1280_FLRC_BT_1    = 0x10,
    SX1280_FLRC_BT_0_5  = 0x20,
} SX1280_FLRC_GAUSSIAN_FILTER_ENUM;


// cmd 0x8C SetPacketParamsFLRC(uint8_t AGCPreambleLength, uint8_t PacketType, uint8_t PayloadLength, int16_t CrcSeed, uint32_t SyncWord, uint8_t CodingRate)
typedef enum
{
    SX1280_PREAMBLE_LENGTH_04_BITS = 0x00, //!< Preamble length: 04 bits (Reserved)
    SX1280_PREAMBLE_LENGTH_08_BITS = 0x10, //!< Preamble length: 08 bits
    SX1280_PREAMBLE_LENGTH_12_BITS = 0x20, //!< Preamble length: 12 bits
    SX1280_PREAMBLE_LENGTH_16_BITS = 0x30, //!< Preamble length: 16 bits
    SX1280_PREAMBLE_LENGTH_20_BITS = 0x40, //!< Preamble length: 20 bits
    SX1280_PREAMBLE_LENGTH_24_BITS = 0x50, //!< Preamble length: 24 bits
    SX1280_PREAMBLE_LENGTH_28_BITS = 0x60, //!< Preamble length: 28 bits
    SX1280_PREAMBLE_LENGTH_32_BITS = 0x70, //!< Preamble length: 32 bits
} SX1280_FLRC_PREAMBLE_LENGTH_ENUM;

typedef enum
{
    SX1280_FLRC_SYNC_NOSYNC        = 0x00,
    SX1280_FLRC_SYNC_WORD_LEN_P32S = 0x04,
} SX1280_FLRC_SYNC_WORD_LENGTH_ENUM;

typedef enum
{
    SX1280_FLRC_RX_DISABLE_SYNC_WORD     = 0x00,
    SX1280_FLRC_RX_MATCH_SYNC_WORD_1     = 0x10,
    SX1280_FLRC_RX_MATCH_SYNC_WORD_2     = 0x20,
    SX1280_FLRC_RX_MATCH_SYNC_WORD_1_2   = 0x30,
    SX1280_FLRC_RX_MATCH_SYNC_WORD_3     = 0x40,
    SX1280_FLRC_RX_MATCH_SYNC_WORD_1_3   = 0x50,
    SX1280_FLRC_RX_MATCH_SYNC_WORD_2_3   = 0x60,
    SX1280_FLRC_RX_MATCH_SYNC_WORD_1_2_3 = 0x70,
} SX1280_FLRC_SYNC_WORD_COMBINATION_ENUM;

typedef enum
{
    SX1280_FLRC_PACKET_FIXED_LENGTH    = 0x00,
    SX1280_FLRC_PACKET_VARIABLE_LENGTH = 0x20,
} SX1280_FLRC_PACKET_TYPE_ENUM;

typedef enum
{
    SX1280_FLRC_CRC_OFF    = 0x00,
    SX1280_FLRC_CRC_2_BYTE = 0x10,
    SX1280_FLRC_CRC_3_BYTE = 0x20,
    SX1280_FLRC_CRC_4_BYTE = 0x30,
} SX1280_FLRC_CRC_DEFINITION_ENUM;

enum
{
    // FLRC Error Packet Status
    SX1280_FLRC_PKT_ERROR_BUSY      = 1 << 0,
    SX1280_FLRC_PKT_ERROR_PKT_RCVD  = 1 << 1,
    SX1280_FLRC_PKT_ERROR_HDR_RCVD  = 1 << 2,
    SX1280_FLRC_PKT_ERROR_ABORT     = 1 << 3,
    SX1280_FLRC_PKT_ERROR_CRC       = 1 << 4,
    SX1280_FLRC_PKT_ERROR_LENGTH    = 1 << 5,
    SX1280_FLRC_PKT_ERROR_SYNC      = 1 << 6,
};


// cmd 0x8D SetDioIrqParams(uint16_t IrqMask, uint16_t Dio1Mask, uint16_t Dio2Mask, uint16_t Dio3Mask)
typedef enum {
    SX1280_IRQ_NONE                             = 0x0000,
    SX1280_IRQ_TX_DONE                          = 0x0001,
    SX1280_IRQ_RX_DONE                          = 0x0002,
    SX1280_IRQ_SYNCWORD_VALID                   = 0x0004, // not LORA
    SX1280_IRQ_SYNCWORD_ERROR                   = 0x0008, // not LORA
    SX1280_IRQ_HEADER_VALID                     = 0x0010,
    SX1280_IRQ_HEADER_ERROR                     = 0x0020,
    SX1280_IRQ_CRC_ERROR                        = 0x0040,
    SX1280_IRQ_RANGING_SLAVE_RESPONSE_DONE      = 0x0080, // not LORA
    SX1280_IRQ_RANGING_SLAVE_REQUEST_DISCARDED  = 0x0100, 
    SX1280_IRQ_RANGING_MASTER_RESULT_VALID      = 0x0200, // not LORA
    SX1280_IRQ_RANGING_MASTER_TIMEOUT           = 0x0400, // not LORA
    SX1280_IRQ_RANGING_SLAVE_REQUEST_VALID      = 0x0800, // not LORA
    SX1280_IRQ_CAD_DONE                         = 0x1000,
    SX1280_IRQ_CAD_DETECTED                     = 0x2000,
    SX1280_IRQ_RX_TX_TIMEOUT                    = 0x4000,
    SX1280_IRQ_PREAMBLE_DETECTED                = 0x8000, // only if long preamble enabled
    SX1280_IRQ_ALL                              = 0xFFFF,
} SX1280_IRQ_ENUM;


// cmd 0x83 SetTx(uint8_t PeriodBase, uint16_t PeriodBaseCount)
// cmd 0x82 SetRx(uint8_t PeriodBase, uint16_t PeriodBaseCount)
typedef enum {
    SX1280_PERIODBASE_15p625_US           = 0x00, // table 11-24, p. 79
    SX1280_PERIODBASE_62p5_US             = 0x01,
    SX1280_PERIODBASE_1_MS                = 0x02,
    SX1280_PERIODBASE_4_MS                = 0x03,
} SX1280_PERIODBASE_ENUM;

typedef enum {
    SX1280_TIMEOUT_TX_NONE                = 0, // p. 133
} SX1280_TIMEOUTTX_ENUM;

typedef enum {
    SX1280_TIMEOUT_RX_SINGLE              = 0, // p. 134
    SX1280_TIMEOUT_RX_CONTINUOUS          = 0xFFFF,
} SX1280_TIMEOUTRX_ENUM;


//-------------------------------------------------------
// Enum Definitions Tx
//-------------------------------------------------------

// cmd 0x8E SetTxParams(uint8_t Power, uint8_t RampTime)
typedef enum {
    SX1280_RAMPTIME_02_US                 = 0x00, // table 11-49, p. 88
    SX1280_RAMPTIME_04_US                 = 0x20,
    SX1280_RAMPTIME_06_US                 = 0x40,
    SX1280_RAMPTIME_08_US                 = 0x60,
    SX1280_RAMPTIME_10_US                 = 0x80,
    SX1280_RAMPTIME_12_US                 = 0xA0,
    SX1280_RAMPTIME_16_US                 = 0xC0,
    SX1280_RAMPTIME_20_US                 = 0xE0,
} SX1280_RAMPTIME_ENUM;

// added for our convenience
typedef enum {
    SX1280_POWER_m18_DBM                  = 0, // after table 11-48, p. 88
    SX1280_POWER_m10_DBM                  = 8, // 0.1 mW
    SX1280_POWER_0_DBM                    = 18, // 1 mW
    SX1280_POWER_3_DBM                    = 21, // 1.995 mW
    SX1280_POWER_6_DBM                    = 24, // 3.981 mW
    SX1280_POWER_10_DBM                   = 28, // 10 mW
    SX1280_POWER_12_DBM                   = 30,
    SX1280_POWER_12p5_DBM                 = 31, // 17.78 mW
    SX1280_POWER_MIN                      = SX1280_POWER_m18_DBM,
    SX1280_POWER_MAX                      = SX1280_POWER_12p5_DBM,
} SX1280_POWER_ENUM;


//-------------------------------------------------------
// Enum Definitions Rx
//-------------------------------------------------------

// cmd 0x1D GetPacketStatus(uint8_t* RssiSync, uint8_t* Snr)
// cmd 0x17 GetRxBufferStatus(uint8_t* rxPayloadLength, uint8_t* rxStartBufferPointer)
// cmd 0x1B ReadBuffer(uint8_t offset, uint8_t* data, uint8_t len)


//-------------------------------------------------------
// Enum Definitions Auxiliary
//-------------------------------------------------------

// cmd 0x96 SetRegulatorMode(uint8_t RegModeParam)
typedef enum {
    SX1280_REGULATOR_MODE_LDO             = 0x00, // table 14-66, p. 143
    SX1280_REGULATOR_MODE_DCDC            = 0x01
} SX1280_REGULATOR_MODE_ENUM;


// cmd 0x98 SetAutoFs(uint8_t flag)
typedef enum {
    SX1280_AUTOFS_DISABLE                 = 0x00, // table 11-38, p. 85
    SX1280_AUTOFS_ENABLE                  = 0x01,
} SX1280_AUTOFS_ENUM;


// reg 0x891 SetLnaGainMode(uint8_t LnaGainMode)
typedef enum {
  SX1280_LNAGAIN_MODE_LOW_POWER           = 0x00, // datasheet chapter 4.2.1, p. 30
  SX1280_LNAGAIN_MODE_HIGH_SENSITIVITY    = 0x01,
} SX1280_LNAGAIN_MODE_ENUM;



#endif // SX128X_LIB_H









