//*******************************************************
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// LR11xx library
//*******************************************************
// contributed by JLP
//*******************************************************

// Semtech User Manual: https://semtech.my.salesforce.com/sfc/p/E0000000JelG/a/RQ000005h0I1/uRUCgjGWaHW2B2wFchdm_w96ucy3g12TruwkrJkeBEE

#ifndef LR11XX_LIB_H
#define LR11XX_LIB_H
#pragma once

#include <inttypes.h>

#define LR11XX_FREQ_XTAL_HZ               32000000

#define LR11XX_FREQ_MHZ_TO_REG(f_mhz)     (uint32_t)((double)f_mhz*1.0E6)
#define LR11XX_FREQ_GHZ_TO_REG(f_ghz)     (uint32_t)((double)f_ghz*1.0E9)

#ifndef ALIGNED
#define ALIGNED  __attribute__((aligned(4)))
#endif


//-------------------------------------------------------
// Base Class
//-------------------------------------------------------

class Lr11xxDriverBase
{
  public:
    Lr11xxDriverBase() {} // constructor

    // this you will have to fill in the derived class

    void Init(void) {}

    // these you have to supply in the derived class

    virtual void SpiSelect(void) = 0;
    virtual void SpiDeselect(void) = 0;
    virtual void SpiTransfer(uint8_t* dataout, uint8_t* datain, uint8_t len) = 0;

    virtual void SpiRead(uint8_t* datain, uint8_t len);
    virtual void SpiWrite(uint8_t* dataout, uint8_t len);

    virtual void WaitOnBusy(void) {}
    virtual void SetDelay(uint16_t tmo_us) { (void)tmo_us; }

    // spi methods

    void SpiTransfer(uint8_t dataout, uint8_t* datain) { SpiTransfer(&dataout, datain, 1); }
    void SpiRead(uint8_t* datain) { SpiRead(datain, 1); }
    void SpiWrite(uint8_t dataout) { SpiWrite(&dataout, 1); }

    // low level methods, usually no need to use them

    void WriteCommand(uint16_t opcode, uint8_t* data, uint8_t len);
    void ReadCommand(uint16_t opcode, uint8_t* data, uint8_t len);
    void WriteBuffer(uint8_t* data, uint8_t len);
    void ReadBuffer(uint8_t offset, uint8_t* data, uint8_t len);

    void WriteCommand(uint16_t opcode) { WriteCommand(opcode, nullptr, 0); }
    void WriteCommand(uint16_t opcode, uint8_t data) { WriteCommand(opcode, &data, 1); }
    uint8_t ReadCommand(uint16_t opcode) { uint8_t data; ReadCommand(opcode, &data, 1); return data; }

    // common methods

    void GetStatus(uint8_t* Status1, uint8_t* Status2);
    void GetLastStatus(uint8_t* Status1, uint8_t* Status2);
    void SetStandby(uint8_t StandbyConfig);
    void SetPacketType(uint8_t PacketType);
    void SetRfFrequency(uint32_t RfFrequency);
    void SetDioAsRfSwitch(uint8_t RfSwEnable, uint8_t RfSwStbyCfg, uint8_t RfSwRxCfg, uint8_t RfSwTxCfg, uint8_t TxHPCfg, uint8_t RfSwTxHfCfg, uint8_t RfSwGnssCfg, uint8_t RfSwWifiCfg);
    void SetTcxoMode(uint8_t OutputVoltage, uint32_t Delay);  // delay is 24 bits only, in 30.52 uS steps
    void SetModulationParams(uint8_t SpreadingFactor, uint8_t Bandwidth, uint8_t CodingRate, uint8_t LowDataRateOptimize); // implied to be LoRa
    void SetPacketParams(uint16_t PreambleLength, uint8_t HeaderType, uint8_t PayloadLength, uint8_t Crc, uint8_t InvertIQ); // implied to be LoRa
    void SetDioIrqParams(uint32_t Irq1ToEnable, uint32_t Irq2ToEnable);
    uint32_t GetIrqStatus(void);
    void ClearIrq(uint32_t IrqToClear); // IrqToClear mask is identical to IrqToEnable assignment
    uint32_t GetAndClearIrqStatus(uint32_t IrqToClear);  // No more GetIrqStatus, use GetStatus

    // Tx methods
    
    void SetPaConfig(uint8_t PaSel, uint8_t RegPaSupply, uint8_t PaDutyCycle, uint8_t PaHPSel); // needed before SetTxParams 9.5.1
    void SetTxParams(uint8_t Power, uint8_t RampTime);
    void SetTx(uint32_t TxTimeout); // 24 bits only, similar to sx126x

    // Rx methods

    void SetRx(uint32_t RxTimeout); // 24 bits only, similar to sx126x
    void GetPacketStatus(int16_t* RssiSync, int8_t* Snr); // implied to be LoRa, RSSI only 8-bit 
    void GetRxBufferStatus(uint8_t* rxPayloadLength, uint8_t* rxStartBufferPointer);

    // auxiliary methods

    void SetRegMode(uint8_t RegModeParam);
    void SetRxTxFallbackMode(uint8_t FallbackMode); // replaces SetAutoFs found in sx126x / sx128x
    void SetFs(void);
    void SetRxBoosted(uint8_t RxBoosted); // similar to SetRxGain in sx126x and SetLnaGainMode in sx128x
    void CalibImage(uint8_t Freq1, uint8_t Freq2); // low frequency only, takes freq in MHz / 4
    void CalibImage_mhz(uint16_t Freq1_mhz, uint16_t Freq2_mhz);  // helper, takes freq in MHz
    void ClearErrors(void);
    void EnableSx127xCompatibility(void);

    // GFSK methods

    void SetModulationParamsGFSK(uint32_t br_bps, uint8_t PulseShape, uint8_t Bandwidth, uint32_t Fdev_hz);
    void SetPacketParamsGFSK(uint16_t PreambleLength, uint8_t PreambleDetectorLength, uint8_t SyncWordLength, uint8_t AddrComp, uint8_t PacketType, uint8_t PayloadLength, uint8_t CRCType, uint8_t Whitening);
    void SetSyncWordGFSK(uint16_t SyncWord);
    void GetPacketStatusGFSK(int16_t* RssiSync);
    
    // other methods

    void GetVersion(uint8_t* HwVersion, uint8_t* UseCase, uint8_t* FwMajor, uint8_t* FwMinor);



  private:
    uint8_t _status1; // status is now two bytes
    uint8_t _status2;
};


//-------------------------------------------------------
// Enum Definitions
//-------------------------------------------------------

// WriteCommand(uint16_t opcode, uint8_t* data, uint8_t len)
typedef enum {
    LR11XX_CMD_WRITE_REG_MEM_32               = 0x0105, // chapter 14
    LR11XX_CMD_READ_REG_MEM_32                = 0x0106,
    LR11XX_CMD_WRITE_BUFFER_8                 = 0x0109,
    LR11XX_CMD_READ_BUFFER_8                  = 0x010A,
    LR11XX_CMD_CLEAR_RX_BUFFER                = 0x010B,
    LR11XX_CMD_WRITE_REG_MEM_MASK_32          = 0x010C,
    LR11XX_CMD_GET_STATUS                     = 0x0100,
    LR11XX_CMD_GET_VERSION                    = 0x0101,
    LR11XX_CMD_GET_ERRORS                     = 0x010D,
    LR11XX_CMD_CLEAR_ERRORS                   = 0x010E,
    LR11XX_CMD_CALIBRATE                      = 0x010F,
    LR11XX_CMD_SET_REG_MODE                   = 0x0110,
    LR11XX_CMD_CALIB_IMAGE                    = 0x0111,
    LR11XX_CMD_SET_DIO_AS_RF_SWITCH           = 0x0112,
    LR11XX_CMD_SET_DIO_IRQ_PARAMS             = 0x0113,
    LR11XX_CMD_CLEAR_IRQ                      = 0x0114,
    LR11XX_CMD_CONFIG_LF_CLOCK                = 0x0116,
    LR11XX_CMD_SET_TCXO_MODE                  = 0x0117,
    LR11XX_CMD_REBOOT                         = 0x0118,
    LR11XX_CMD_GET_VBAT                       = 0x0119,
    LR11XX_CMD_GET_TEMP                       = 0x011A,
    LR11XX_CMD_SET_SLEEP                      = 0x011B,
    LR11XX_CMD_SET_STANDBY                    = 0x011C,
    LR11XX_CMD_SET_FS                         = 0x011D,
    LR11XX_CMD_GET_RANDOM_NUMBER              = 0x0120,
    LR11XX_CMD_ERASE_INFO_PAGE                = 0x0121,
    LR11XX_CMD_WRITE_INFO_PAGE                = 0x0122,
    LR11XX_CMD_READ_INFO_PAGE                 = 0x0123,
    LR11XX_CMD_GET_CHIP_EUI                   = 0x0125,
    LR11XX_CMD_GET_SEMTECH_JOIN_EUI           = 0x0126,
    LR11XX_CMD_DERIVE_ROOT_KEYS_AND_GET_PIN   = 0x0127,
    LR11XX_CMD_ENABLE_SPI_CRC                 = 0x0128,
    LR11XX_CMD_DRIVE_DIOS_IN_SLEEP_MODE       = 0x012A,
    LR11XX_CMD_RESET_STATS                    = 0x0200,
    LR11XX_CMD_GET_STATS                      = 0x0201,
    LR11XX_CMD_GET_PACKET_TYPE                = 0x0202,
    LR11XX_CMD_GET_RX_BUFFER_STATUS           = 0x0203,
    LR11XX_CMD_GET_PACKET_STATUS              = 0x0204,
    LR11XX_CMD_GET_RSSI_INT                   = 0x0205,
    LR11XX_CMD_SET_GFSK_SYNC_WORD             = 0x0206,
    LR11XX_CMD_SET_LORA_PUBLIC_NETWORK        = 0x0208,
    LR11XX_CMD_SET_RX                         = 0x0209,
    LR11XX_CMD_SET_TX                         = 0x020A,
    LR11XX_CMD_SET_RF_FREQUENCY               = 0x020B,
    LR11XX_CMD_AUTO_TXRX                      = 0x020C,
    LR11XX_CMD_SET_CAD_PARAMS                 = 0x020D,
    LR11XX_CMD_SET_PACKET_TYPE                = 0x020E,
    LR11XX_CMD_SET_MODULATION_PARAMS          = 0x020F,
    LR11XX_CMD_SET_PACKET_PARAMS              = 0x0210,
    LR11XX_CMD_SET_TX_PARAMS                  = 0x0211,
    LR11XX_CMD_SET_PACKET_ADRS                = 0x0212,
    LR11XX_CMD_SET_RXTX_FALLBACK_MODE         = 0x0213,
    LR11XX_CMD_SET_RX_DUTY_CYCLE              = 0x0214,
    LR11XX_CMD_SET_PA_CONFIG                  = 0x0215,
    LR11XX_CMD_STOP_TIMEOUT_ON_PREAMBLE       = 0x0217,
    LR11XX_CMD_SET_CAD                        = 0x0218,
    LR11XX_CMD_SET_TX_CW                      = 0x0219,
    LR11XX_CMD_SET_TX_INFINITE_PREAMBLE       = 0x021A,
    LR11XX_CMD_SET_LORA_SYNCH_TIMEOUT         = 0x021B,
    LR11XX_CMD_SET_GFSK_CRC_PARAMS            = 0x0224,
    LR11XX_CMD_SET_GFSK_WHIT_PARAMS           = 0x0225,
    LR11XX_CMD_SET_RX_BOOSTED                 = 0x0227,
    LR11XX_CMD_SET_RSSI_CALIBRATION           = 0x0229,
    LR11XX_CMD_SET_LORA_SYNC_WORD             = 0x022B,
    LR11XX_CMD_LR_FHSS_BUILD_FRAME            = 0x022C,
    LR11XX_CMD_LR_FHSS_SET_SYNC_WORD          = 0x022D,
    LR11XX_CMD_GET_LORA_RX_HEADER_INFOS       = 0x0230,
} LR11XX_CMD_ENUM;


// cmd 0x0100 void GetStatus(uint8_t* Stat1, uint8_t* Stat2, uint32_t* IrqStatus)
typedef enum {
    LR11XX_STATUS_CMD_FAIL                    = 0x00 << 1, // table 3-2, page 28
    LR11XX_STATUS_CMD_PERR                    = 0x01 << 1,
    LR11XX_STATUS_CMD_OK                      = 0x02 << 1,
    LR11XX_STATUS_CMD_DAT                     = 0x03 << 1,
} LR11XX_STATUS_COMMAND_ENUM;

typedef enum {
    LR11XX_STATUS_INTERRUPT_INACTIVE          = 0x00, // table 3-2, page 28
    LR11XX_STATUS_INTERRUPT_ACTIVE            = 0x01,
} LR11XX_STATUS_INTERRUPT_ENUM;

typedef enum {
    LR11XX_STATUS_RESET_CLEARED               = 0x00 << 4, // table 3-3, page 29
    LR11XX_STATUS_RESET_ANALOG                = 0x01 << 4,
    LR11XX_STATUS_RESET_EXTERNAL              = 0x02 << 4,
    LR11XX_STATUS_RESET_SYSTEM                = 0x03 << 4,
    LR11XX_STATUS_RESET_WATCHDOG              = 0x04 << 4,
    LR11XX_STATUS_RESET_WAKEUP                = 0x05 << 4,
    LR11XX_STATUS_RESET_RTC                   = 0x06 << 4,
} LR11XX_STATUS_RESET_ENUM;

typedef enum {
    LR11XX_STATUS_MODE_SLEEP                  = 0x00 << 1, // table 3-3, page 29
    LR11XX_STATUS_MODE_STANDBY_RC             = 0x01 << 1,
    LR11XX_STATUS_MODE_STANDBY_EXT            = 0x02 << 1,
    LR11XX_STATUS_MODE_FS                     = 0x03 << 1,
    LR11XX_STATUS_MODE_RX                     = 0x04 << 1,
    LR11XX_STATUS_MODE_TX                     = 0x05 << 1,
} LR11XX_STATUS_MODE_ENUM;

typedef enum {
    LR11XX_STATUS_BOOTLOADER_BOOTLOADER       = 0x00, // table 3-3, page 29
    LR11XX_STATUS_BOOTLOADER_FLASH            = 0x01,
} LR11XX_STATUS_BOOTLOADER_ENUM;


//-------------------------------------------------------
// Enum Definitions Common
//-------------------------------------------------------

 // cmd 0x011C void SetStandby(uint8_t StandbyConfig)
 typedef enum {
    LR11XX_STDBY_CONFIG_STDBY_RC          = 0x00, // table 2-1, page 15
    LR11XX_STDBY_CONFIG_STDBY_XOSC        = 0x01
} LR11XX_STDBY_CONFIG_ENUM;

// cmd 0x020E void SetPacketType(uint8_t PacketType)
typedef enum {
    LR11XX_PACKET_TYPE_NONE               = 0x00, // table 8-1, page 64
    LR11XX_PACKET_TYPE_GFSK               = 0x01,
    LR11XX_PACKET_TYPE_LORA               = 0x02,
    LR11XX_PACKET_TYPE_SIGFOX             = 0x03,
    LR11XX_PACKET_TYPE_GMSK               = 0x04,
} LR11XX_PACKET_TYPE_ENUM;

// cmd 0x020B void SetRfFrequency(uint32_t RfFrequency)

// cmd 0x0112 void SetDioAsRfSwitch(uint8_t RfSwEnable, uint8_t RfSwStbyCfg, uint8_t RfSwRxCfg, 
// uint8_t RfSwTxCfg, uint8_t TxHPCfg, uint8_t RfSwTxHfCfg)
typedef enum {                                   
    LR11XX_RF_SW_ENABLE_DIO5              = 0x01, // bit 0 table 4-5, page 39
    LR11XX_RF_SW_ENABLE_DIO6              = 0x02, // bit 1
    LR11XX_RF_SW_ENABLE_DIO7              = 0x04, // bit 2
    LR11XX_RF_SW_ENABLE_DIO8              = 0x08, // bit 3
    LR11XX_RF_SW_ENABLE_DIO10             = 0x10, // bit 4
} LR11XX_RF_SW_ENABLE_ENUM;

typedef enum {                                   
    LR11XX_RF_SW_STANDBY_CONFIG_DIO5      = 0x01, // table 4-5, page 39
    LR11XX_RF_SW_STANDBY_CONFIG_DIO6      = 0x02,
    LR11XX_RF_SW_STANDBY_CONFIG_DIO7      = 0x04,
    LR11XX_RF_SW_STANDBY_CONFIG_DIO8      = 0x08,
    LR11XX_RF_SW_STANDBY_CONFIG_DIO10     = 0x10,
} LR11XX_RF_SW_STANDBY_CONFIG_ENUM;

typedef enum {                                   
    LR11XX_RF_SW_RX_CONFIG_DIO5           = 0x01, // table 4-5, page 39
    LR11XX_RF_SW_RX_CONFIG_DIO6           = 0x02,
    LR11XX_RF_SW_RX_CONFIG_DIO7           = 0x04,
    LR11XX_RF_SW_RX_CONFIG_DIO8           = 0x08,
    LR11XX_RF_SW_RX_CONFIG_DIO10          = 0x10,
} LR11XX_RF_SW_RX_CONFIG_ENUM;

typedef enum {                                   
    LR11XX_RF_SW_TX_CONFIG_DIO5           = 0x01, // table 4-5, page 39
    LR11XX_RF_SW_TX_CONFIG_DIO6           = 0x02,
    LR11XX_RF_SW_TX_CONFIG_DIO7           = 0x04,
    LR11XX_RF_SW_TX_CONFIG_DIO8           = 0x08,
    LR11XX_RF_SW_TX_CONFIG_DIO10          = 0x10,
} LR11XX_RF_SW_TX_CONFIG_ENUM;

typedef enum {                                   
    LR11XX_RF_SW_TX_HP_CONFIG_DIO5        = 0x01, // table 4-5, page 39
    LR11XX_RF_SW_TX_HP_CONFIG_DIO6        = 0x02,
    LR11XX_RF_SW_TX_HP_CONFIG_DIO7        = 0x04,
    LR11XX_RF_SW_TX_HP_CONFIG_DIO8        = 0x08,
    LR11XX_RF_SW_TX_HP_CONFIG_DIO10       = 0x10,
} LR11XX_RF_SW_TX_HP_CONFIG_ENUM;

typedef enum {                                   
    LR11XX_RF_SW_TX_HF_CONFIG_DIO5        = 0x01, // table 4-5, page 39
    LR11XX_RF_SW_TX_HF_CONFIG_DIO6        = 0x02,
    LR11XX_RF_SW_TX_HF_CONFIG_DIO7        = 0x04,
    LR11XX_RF_SW_TX_HF_CONFIG_DIO8        = 0x08,
    LR11XX_RF_SW_TX_HF_CONFIG_DIO10       = 0x10,
} LR11XX_RF_SW_TX_HF_CONFIG_ENUM;

// cmd 0x0117 void SetTcxoMode(uint8_t OutputVoltage, uint32_t Delay)
typedef enum {                                   
    LR11XX_TCXO_OUTPUT_1_6                = 0x00, // // table 6-3, page 48, 1.6V
    LR11XX_TCXO_OUTPUT_1_7                = 0x01, // 1.7V
    LR11XX_TCXO_OUTPUT_1_8                = 0x02, // 1.8V
    LR11XX_TCXO_OUTPUT_2_2                = 0x03, // 2.2V
    LR11XX_TCXO_OUTPUT_2_4                = 0x04, // 2.4V
    LR11XX_TCXO_OUTPUT_2_7                = 0x05, // 2.7V
    LR11XX_TCXO_OUTPUT_3_0                = 0x06, // 3.0V
    LR11XX_TCXO_OUTPUT_3_3                = 0x07, // 3.3V
} LR11XX_TCXO_OUTPUT_ENUM;

// cmd 0x020F void SetModulationParams(uint8_t SpreadingFactor, uint8_t Bandwidth, uint8_t CodingRate, uint8_t LowDataRateOptimize)
typedef enum {
    LR11XX_LORA_SF5                       = 0x05, // table 8-4, page 67
    LR11XX_LORA_SF6                       = 0x06,
    LR11XX_LORA_SF7                       = 0x07,
    LR11XX_LORA_SF8                       = 0x08,
    LR11XX_LORA_SF9                       = 0x09,
    LR11XX_LORA_SF10                      = 0x0A,
    LR11XX_LORA_SF11                      = 0x0B,
    LR11XX_LORA_SF12                      = 0x0C,
} LR11XX_LORA_SF_ENUM;

typedef enum {
    LR11XX_LORA_BW_62                     = 0x03, // table 8-4, page 67
    LR11XX_LORA_BW_125                    = 0x04,
    LR11XX_LORA_BW_250                    = 0x05,
    LR11XX_LORA_BW_500                    = 0x06,
    LR11XX_LORA_BW_200                    = 0x0D, // 2.4 only
    LR11XX_LORA_BW_400                    = 0x0E, // 2.4 only
    LR11XX_LORA_BW_800                    = 0x0F, // 2.4 only
} LR11XX_LORA_BW_ENUM;

typedef enum {
    LR11XX_LORA_CR_4_5                    = 0x01, // table 8-4, page 67
    LR11XX_LORA_CR_4_6                    = 0x02,
    LR11XX_LORA_CR_4_7                    = 0x03,
    LR11XX_LORA_CR_4_8                    = 0x04,
    LR11XX_LORA_CR_LI_4_5                 = 0x05, // 2.4 only?
    LR11XX_LORA_CR_LI_4_6                 = 0x06, // 2.4 only?
    LR11XX_LORA_CR_LI_4_8                 = 0x07, // 2.4 only?
} LR11XX_LORA_CR_ENUM;

typedef enum {
    LR11XX_LORA_LDR_OFF                   = 0x00, // table 8-4, page 67
    LR11XX_LORA_LDR_ON                    = 0x01,
} LR11XX_LORA_LDR_ENUM;

// cmd 0x0210 void SetPacketParams(uint16_t PreambleLength, uint8_t HeaderType, uint8_t PayloadLength, uint8_t Crc, uint8_t InvertIQ)
typedef enum {
    LR11XX_LORA_HEADER_EXPLICIT           = 0x00, // table 8-5, page 68
    LR11XX_LORA_HEADER_IMPLICIT           = 0x01,
    LR11XX_LORA_HEADER_ENABLE             = LR11XX_LORA_HEADER_EXPLICIT, // compatiblility?
    LR11XX_LORA_HEADER_DISABLE            = LR11XX_LORA_HEADER_IMPLICIT, // compatiblility?
} LR11XX_LORA_HEADER_ENUM;

typedef enum {
    LR11XX_LORA_CRC_DISABLE               = 0x00, // table 8-5, page 68
    LR11XX_LORA_CRC_ENABLE                = 0x01,
} LR11XX_LORA_CRC_ENUM;

typedef enum {
    LR11XX_LORA_IQ_NORMAL                 = 0x00, // table 8-5, page 68
    LR11XX_LORA_IQ_INVERTED               = 0x01,
} LR11XX_LORA_IQMODE_ENUM;

// cmd 0x0113 void SetDioIrqParams(uint32_t Irq1ToEnable, uint32_t Irq2ToEnable)
typedef enum {
    LR11XX_IRQ_NONE                       = 0x00000000, // table 4-2, page 37
    LR11XX_IRQ_TX_DONE                    = 0x00000004, // bit 2
    LR11XX_IRQ_RX_DONE                    = 0x00000008, // bit 3
    LR11XX_IRQ_PREAMBLE_DETECTED          = 0x00000010, // bit 4
    LR11XX_IRQ_SYNC_WORD_VALID            = 0x00000020, // bit 5
    LR11XX_IRQ_HEADER_ERROR               = 0x00000040, // bit 6
    LR11XX_IRQ_PACKET_ERROR               = 0x00000080, // bit 7
    LR11XX_IRQ_CAD_DONE                   = 0x00000100, // bit 8
    LR11XX_IRQ_CAD_DETECTED               = 0x00000200, // bit 9
    LR11XX_IRQ_TIMEOUT                    = 0x00000400, // bit 10
    LR11XX_IRQ_LR_FHSS_HOP                = 0x00000800, // bit 11
    LR11XX_IRQ_LBD                        = 0x00200000, // bit 21
    LR11XX_IRQ_CMD_ERROR                  = 0x00400000, // bit 22
    LR11XX_IRQ_OTHER_ERROR                = 0x00800000, // bit 23
    LR11XX_IRQ_FSK_LEN_ERROR              = 0x01000000, // bit 24
    LR11XX_IRQ_FSK_ADDR_ERROR             = 0x02000000, // bit 25
    LR11XX_IRQ_LORA_RX_TIMESTAMP          = 0x08000000, // bit 27
    LR11XX_IRQ_ALL                        = 0xFFFFFFFF,
} LR11XX_IRQ_ENUM;

// cmd 0x020A void SetTx(uint32_t TxTimeout)
// cmd 0x0209 void SetRx(uint32_t RxTimeout)

typedef enum {
    LR11XX_TIMEOUT_TX_NONE                = 0x000000,
} LR11XX_TIMEOUT_TX_ENUM;

typedef enum {
    LR11XX_TIMEOUT_RX_SINGLE              = 0x000000,
    LR11XX_TIMEOUT_RX_CONTINUOUS          = 0xFFFFFF,
} LR11XX_TIMEOUT_RX_ENUM;


//-------------------------------------------------------
// Enum Definitions Tx
//-------------------------------------------------------

// cmd 0x0215 void SetPaConfig(uint8_t PaSel, uint8_t RegPaSupply, uint8_t PaDutyCycle, uint8_t PaHPSel)
typedef enum {
    LR11XX_PA_SELECT_LP_PA                = 0x00, // table 9-4, page 102
    LR11XX_PA_SELECT_HP_PA                = 0x01, 
    LR11XX_PA_SELECT_HF_PA                = 0x02,
} LR11XX_PA_SEL_ENUM;

typedef enum {
    LR11XX_REG_PA_SUPPLY_INTERNAL         = 0x00, // table 9-4, page 102
    LR11XX_REG_PA_SUPPLY_VBAT             = 0x01, 
} LR11XX_REG_PA_SUPPLY_ENUM;

typedef enum {
    LR11XX_PA_DUTY_CYCLE_22_DBM           = 0x04, // to be used with high power PA
    LR11XX_PA_DUTY_CYCLE_14_DBM           = 0x07, // to be used with low power PA
} LR11XX_PA_DUTY_CYCLE_ENUM;

typedef enum {
    LR11XX_PA_HP_SEL_22_DBM               = 0x07, // this setting only affects the high power PA
} LR11XX_PA_HP_SEL_ENUM;

// cmd 0x0211 void SetTxParams(uint8_t Power, uint8_t RampTime)
typedef enum {
    LR11XX_RAMPTIME_16_US                 = 0x00, // table 9-7, page 103
    LR11XX_RAMPTIME_32_US                 = 0x01,
    LR11XX_RAMPTIME_48_US                 = 0x02, // recommended by user manual
    LR11XX_RAMPTIME_64_US                 = 0x03,
    LR11XX_RAMPTIME_80_US                 = 0x04,
    LR11XX_RAMPTIME_96_US                 = 0x05,
    LR11XX_RAMPTIME_112_US                = 0x06,
    LR11XX_RAMPTIME_128_US                = 0x07,
    LR11XX_RAMPTIME_144_US                = 0x08,
    LR11XX_RAMPTIME_160_US                = 0x09,
    LR11XX_RAMPTIME_176_US                = 0x0A,
    LR11XX_RAMPTIME_192_US                = 0x0B,
    LR11XX_RAMPTIME_208_US                = 0x0C,
    LR11XX_RAMPTIME_240_US                = 0x0D,
    LR11XX_RAMPTIME_272_US                = 0x0E,
    LR11XX_RAMPTIME_304_US                = 0x0F,
} LR11XX_RAMPTIME_ENUM;

// added for convenience
// -9 (0xF7) to +22 (0x16) dBm by step of 1 dB if high power PA is selected
typedef enum {
    LR11XX_POWER_m9_DBM                   = -9, // 0.12 mW
    LR11XX_POWER_0_DBM                    = 0, // 1 mW
    LR11XX_POWER_10_DBM                   = 10, // 10 mW
    LR11XX_POWER_17_DBM                   = 17, // 50 mW
    LR11XX_POWER_20_DBM                   = 20, // 100 mW
    LR11XX_POWER_22_DBM                   = 22, // 158 mW
    LR11XX_POWER_MIN                      = LR11XX_POWER_m9_DBM,
    LR11XX_POWER_MAX                      = LR11XX_POWER_22_DBM,
} LR11XX_POWER_ENUM;


//-------------------------------------------------------
// Enum Definitions Rx
//-------------------------------------------------------

// cmd void 0x0204 GetPacketStatus(int16_t* RssiSync, int8_t* Snr)
// cmd void 0x0203 GetRxBufferStatus(uint8_t* rxPayloadLength, uint8_t* rxStartBufferPointer)
// cmd void 0x010A ReadBuffer(uint8_t offset, uint8_t* data, uint8_t len)

//-------------------------------------------------------
// Enum Definitions Auxiliary
//-------------------------------------------------------

// cmd 0x0110 void SetRegMode(uint8_t RegModeParam)
typedef enum {
    LR11XX_REGULATOR_MODE_LDO             = 0x00, // table 5-3, page 42
    LR11XX_REGULATOR_MODE_DCDC            = 0x01,
} LR11XX_REGULATOR_MODE_ENUM;

// cmd 0x0213 void SetRxTxFallbackMode(uint8_t FallbackMode)
typedef enum {
    LR11XX_RX_TX_FALLBACK_MODE_STDBY_RC   = 0x01, // table 7-5, page 53
    LR11XX_RX_TX_FALLBACK_MODE_STDBY_XOSC = 0x02,
    LR11XX_RX_TX_FALLBACK_MODE_FS         = 0x03, 
} LR11XX_FALLBACK_MODE_ENUM;

// cmd 0x011D void SetFs(void)

// cmd 0x0227 void SetRxBoosted(uint8_t RxBoosted)
typedef enum {
    LR11XX_RX_GAIN_POWER_SAVING           = 0x00, // table 7-15, page 58
    LR11XX_RX_GAIN_BOOSTED_GAIN           = 0x01,
} LR11XX_LNAGAIN_MODE_ENUM;

// cmd 0x0111 void CalibImage(uint8_t Freq1, uint8_t Freq2)
typedef enum {
    LR11XX_CAL_IMG_430_MHZ_1             = 0x6B, // table 2-3, page 16
    LR11XX_CAL_IMG_430_MHZ_2             = 0x6F,
    LR11XX_CAL_IMG_470_MHZ_1             = 0x75,
    LR11XX_CAL_IMG_470_MHZ_2             = 0x81,
    LR11XX_CAL_IMG_779_MHZ_1             = 0xC1,
    LR11XX_CAL_IMG_779_MHZ_2             = 0xC5,
    LR11XX_CAL_IMG_863_MHZ_1             = 0xD7,
    LR11XX_CAL_IMG_863_MHZ_2             = 0xDB,
    LR11XX_CAL_IMG_902_MHZ_1             = 0xE1,
    LR11XX_CAL_IMG_902_MHZ_2             = 0xE9,
} LR11XX_CALIBRATE_IMAGE_ENUM;

//-------------------------------------------------------
// Enum Definitions GFSK
//-------------------------------------------------------

// cmd 0x020F SetModulationParamsGFSK(uint32_t br_bps, uint8_t PulseShape, uint8_t Bandwidth, uint32_t Fdev_hz)
typedef enum {
    LR11XX_GFSK_PULSESHAPE_OFF           = 0x00,
    LR11XX_GFSK_PULSESHAPE_BT_03         = 0x08,
    LR11XX_GFSK_PULSESHAPE_BT_05         = 0x09,
    LR11XX_GFSK_PULSESHAPE_BT_07         = 0x0A,
    LR11XX_GFSK_PULSESHAPE_BT_1          = 0x0B,
    LR11XX_GFSK_PULSESHAPE_BPSK_BT_07    = 0x0B,
} LR11XX_GFSK_PULSESHAPE_ENUM;

typedef enum {
    LR11XX_GFSK_BW_4800             = 0x1F,
    LR11XX_GFSK_BW_5800             = 0x17,
    LR11XX_GFSK_BW_7300             = 0x0F,
    LR11XX_GFSK_BW_9700             = 0x1E,
    LR11XX_GFSK_BW_11700            = 0x16,
    LR11XX_GFSK_BW_14600            = 0x0E,
    LR11XX_GFSK_BW_19500            = 0x1D,
    LR11XX_GFSK_BW_23400            = 0x15,
    LR11XX_GFSK_BW_29300            = 0x0D,
    LR11XX_GFSK_BW_39000            = 0x1C,
    LR11XX_GFSK_BW_46900            = 0x14,
    LR11XX_GFSK_BW_58600            = 0x0C,
    LR11XX_GFSK_BW_78200            = 0x1B,
    LR11XX_GFSK_BW_93800            = 0x13,
    LR11XX_GFSK_BW_117300           = 0x0B,
    LR11XX_GFSK_BW_156200           = 0x1A,
    LR11XX_GFSK_BW_187200           = 0x12,
    LR11XX_GFSK_BW_234300           = 0x0A,
    LR11XX_GFSK_BW_312000           = 0x19,
    LR11XX_GFSK_BW_373600           = 0x11,
    LR11XX_GFSK_BW_467000           = 0x09,
} LR11XX_GFSK_BANDWIDTH_ENUM;
 
 
// cmd 0x0210 SetPacketParamsGFSK(uint16_t PreambleLength, uint8_t PreambleDetectorLength, uint8_t SyncWordLength, uint8_t AddrComp, 
//            uint8_t PacketType, uint8_t PayloadLength, uint8_t CRCType, uint8_t Whitening);
typedef enum {
    LR11XX_GFSK_PREAMBLE_DETECTOR_OFF                           = 0x00,
    LR11XX_GFSK_PREAMBLE_DETECTOR_LENGTH_8BITS                  = 0x04,
    LR11XX_GFSK_PREAMBLE_DETECTOR_LENGTH_16BITS                 = 0x05,
    LR11XX_GFSK_PREAMBLE_DETECTOR_LENGTH_24BITS                 = 0x06,
    LR11XX_GFSK_PREAMBLE_DETECTOR_LENGTH_32BITS                 = 0x07,
} LR11XX_GFSK_PREAMBLE_DETECTOR_LENGTH_ENUM;

typedef enum {
    LR11XX_GFSK_ADDRESS_FILTERING_DISABLE                       = 0x00,
    LR11XX_GFSK_ADDRESS_FILTERING_NODE_ADDRESS                  = 0x01,
    LR11XX_GFSK_ADDRESS_FILTERING_NODE_AND_BROADCAST_ADDRESSES  = 0x02,
} LR11XX_GFSK_ADDRESS_FILTERING_ENUM;

typedef enum {
    LR11XX_GFSK_PKT_FIX_LEN                                     = 0x00,
    LR11XX_GFSK_PKT_VAR_LEN                                     = 0x01,
    LR11XX_GFSK_PKT_VAR_LEN_SX128X_COMPAT                       = 0x02,
} LR11XX_GFSK_PKT_LEN_ENUM;

typedef enum {
    LR11XX_GFSK_CRC_OFF                                         = 0x01,
    LR11XX_GFSK_CRC_1_BYTE                                      = 0x00,
    LR11XX_GFSK_CRC_2_BYTES                                     = 0x02,
    LR11XX_GFSK_CRC_1_BYTE_INV                                  = 0x04,
    LR11XX_GFSK_CRC_2_BYTES_INV                                 = 0x06,
} LR11XX_GFSK_CRC_TYPES_ENUM;

typedef enum {
    LR11XX_GFSK_WHITENING_OFF                                   = 0x00,
    LR11XX_GFSK_WHITENING_ENABLE                                = 0x01,
    LR11XX_GFSK_WHITENING_ENABLE_SX128X_COMPAT                  = 0x03,
} LR11XX_GFSK_WHITENING_TYPES_ENUM;


#endif // LR11XX_LIB_H
