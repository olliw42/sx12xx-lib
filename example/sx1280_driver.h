//*******************************************************
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// SX1280 Driver
//*******************************************************
// this is not a complete working example
// it is for illustration only, to show how the sx128x-lib could be used
// in a practical code. You obviously have to provide some low level 
// functions to make this to work. To get the SX up an running
// - call Init()
// - check with isOk()
// - call StartUp()
// - call SetRfFrequency()
// the SX is then ready to be used
//*******************************************************
#ifndef SX1280_DRIVER_H
#define SX1280_DRIVER_H
#pragma once


//-------------------------------------------------------
// SX Driver
//-------------------------------------------------------

class SxDriver : public SxDriverBase
{
  public:

    //-- interface to SPI peripheral

#ifdef SX_BUSY
    void WaitOnBusy(void) override
    {
      while (sx_busy_read()) { __NOP(); };
    }
#else
    uint32_t timer_us_tmo = 0;
    uint32_t timer_us_start_tick = 0;

    void WaitOnBusy(void) override
    {
      if (timer_us_tmo) {
        while ((DWT->CYCCNT - timer_us_start_tick) < timer_us_tmo) { __NOP(); };
        timer_us_tmo = 0;
      }
    }

    void SetDelay(uint16_t tmo_us) override
    {
      timer_us_tmo = (uint32_t)tmo_us * (SystemCoreClock/1000000);
      timer_us_start_tick = DWT->CYCCNT;
    }
#endif

    void SpiSelect(void) override
    {
#ifndef SX_BUSY
        delay_ns(150); // datasheet says t9 = 100 ns, semtech driver doesn't do it, helps so do it
#endif
        spi_select();
        delay_ns(50); // datasheet says t1 = 25 ns, semtech driver doesn't do it, helps so do it
    }

    void SpiDeselect(void) override
    {
        delay_ns(50); // datasheet says t8 = 25 ns, semtech driver doesn't do it, helps so do it
        spi_deselect();
#ifndef SX_BUSY
        delay_ns(100); // well...
#endif
    }

    void SpiTransfer(uint8_t* dataout, uint8_t* datain, uint8_t len) override
    {
        spi_transfer(dataout, datain, len);
    }

    //-- init API functions

    void _reset(void)
    {
        gpio_low(SX_RESET);
        delay_ms(5); // 10 us seems to be sufficient, play it safe, semtech driver uses 50 ms
        gpio_high(SX_RESET);
        delay_ms(50); // semtech driver says "typically 2ms observed"
        WaitOnBusy();
    }

    void Init(void)
    {
        spi_init();
        sx_init_gpio();
        sx_dio1_init_exti_isroff();

        // no idea how long the SX1280 takes to boot up, so give it some good time
        // we could probably speed up by using WaitOnBusy()
        delay_ms(300);
        _reset(); // this is super crucial !
    }

    //-- high level API functions

    bool isOk(void)
    {
        uint16_t firmwareRev = GetFirmwareRev();
        return ((firmwareRev != 0) && (firmwareRev != 65535));
    }

    void Configure(void)
    {
        SetPacketType(SX1280_PACKET_TYPE_LORA);
        SetBufferBaseAddress(0, 0);

        SetAutoFs(true);

        SetLnaGainMode(SX1280_LNAGAIN_MODE_HIGH_SENSITIVITY);

        SetModulationParams(SX1280_LORA_SF5,
                        SX1280_LORA_BW_800,
                        SX1280_LORA_CR_LI_4_5);

        SetPacketParams(12, // preamble length
                        SX1280_LORA_HEADER_DISABLE,
                        FRAME_TX_RX_LEN,
                        SX1280_LORA_CRC_DISABLE,
                        SX1280_LORA_IQ_NORMAL);

#ifdef LORA_SYNCWORD
        SetSyncWord(LORA_SYNCWORD); // experimental, not confirmed
#endif

#ifdef DEVICE_IS_TRANSMITTER
        SetTxParams(SETUP_TX_POWER, SX1280_RAMPTIME_04_US);
#endif
#ifdef DEVICE_IS_RECEIVER
        SetTxParams(SETUP_RX_POWER, SX1280_RAMPTIME_04_US);
#endif

        SetDioIrqParams(SX1280_IRQ_ALL,
                        SX1280_IRQ_RX_DONE|SX1280_IRQ_TX_DONE|SX1280_IRQ_RX_TX_TIMEOUT,
                        SX1280_IRQ_NONE,
                        SX1280_IRQ_NONE);
        ClearIrqStatus(SX1280_IRQ_ALL);

        SetFs();
        delay_us(125); // may not be needed if busy available
    }

    void StartUp(void)
    {
        SetStandby(SX1280_STDBY_CONFIG_STDBY_RC); // should be in STDBY_RC after reset
        delay_us(1000); // this is important, 500 us ok

        Configure();

        sx_dio1_enable_isr();
    }

    //-- this are the API functions used in the loop

    void SendFrame(uint8_t* data, uint8_t len, uint16_t tmo_us = 100)
    {
        sx_amp_transmit();
        WriteBuffer(0, data, len);
        ClearIrqStatus(SX1280_IRQ_ALL);
        SetTx(SX1280_PERIODBASE_62p5_US, tmo_us*16); // if a Tx timeout occurs we have a serious problem
        delay_us(125); // may not be needed if busy available
    }

    void SetToRx(uint16_t tmo_us = 10)
    {
        sx_amp_receive();
        ClearIrqStatus(SX1280_IRQ_ALL);
        SetRx(SX1280_PERIODBASE_62p5_US, tmo_us*16);
        delay_us(125); // may not be needed if busy available
    }

    void ReadFrame(uint8_t* data, uint8_t len)
    {
        uint8_t rxStartBufferPointer;
        uint8_t rxPayloadLength;

        // rxPayloadLength is always 0 if no header
        GetRxBufferStatus(&rxPayloadLength, &rxStartBufferPointer);
        // if one wants it, it could be obtained from what had been set
        // rxPayloadLength = ReadRegister(SX1280_REG_PayloadLength);
        ReadBuffer(rxStartBufferPointer, data, len);
    }

    //-- helper

    void SetRfPower(uint8_t power)
    {
        SetTxParams(power, SX1280_RAMPTIME_04_US);
    }

    uint32_t TimeOverAir_us(void)
    {
        // cumbersome to calculate in general, so use hardcoded for a specific settings
        if (lora_configuration != nullptr) {
            return lora_configuration->TimeOverAir;
        }
        return 0;
    }
};



#endif // SX1280_DRIVER_H
