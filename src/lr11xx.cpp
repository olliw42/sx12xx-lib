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
    WaitOnBusy();
}

