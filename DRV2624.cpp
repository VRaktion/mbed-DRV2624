/*
 * mbed library program 
 *  Texas Instruments / DRV2624 Ultra Low Power Closed-Loop LRA/ERM Haptic Driver with Internal Memory
 *      http://www.ti.com/lit/ds/symlink/drv2624.pdf
 *
 * Copyright (c) 2019, Wenzel Reichmuth / VRaktion
 *  http://mbed.org/users/vraktion/
 *      Created: August      6th, 2019
 */

#ifndef MBED_DRV2624_CPP
#define MBED_DRV2624_CPP

#include "mbed.h"
#include "DRV2624.h"
// #include "DRV2624reg.h"

DRV2624::DRV2624(PinName p_sda, PinName p_scl)
    : i2c(new I2C(p_sda, p_scl))
{
    address = (char)(DRV2624_ADDR << 1);
}

DRV2624::DRV2624(I2C *p_i2c)
    : i2c(p_i2c)
{
    address = (char)(DRV2624_ADDR << 1);
}

// Play a Waveform or Waveform Sequence from the RAM Waveform Memory
// 1. Initialize the device as listed in the Initialization Procedure section:
//  1. After power-up, wait at least 1ms before the DRV2624 device accepts I2C commands.
//  2. Assert the NRST pin (logichigh). The NRST pin can be asserted anytime during or after the wait period.
// 3. Write the MODE parameter (address0x01) to value 0x00 to remove the device from standby mode.
// 4. Run auto-calibration to configure the DRV2624 device for the desired actuator. Alternatively, rewrite the results from a previous calibration.
// 5. If using the embedded RAM memory, populate the RAM with waveforms at this time.
// 2. Select the desired TRIG/INTZ pin function by changing the TRIG_PIN_FUNC parameter.
// 3. Identify the waveform index to be played and populate the waveformsequencer.
// 4. Trigger the waveform using the desired trigger method (GO bit, or external trigger). Note that if using the interrupt functionality, only the GO bit can be used to trigger the process.
// 5. Device will automatically go into standby upon completion of the playback

int DRV2624::init()
{
    // wait_ms(1);
    ThisThread::sleep_for(1);
    setMode(DRV2624reg::_07Mode::realTime);

    return 0;
}

//////REG07//////

int DRV2624::enableI2CBroadcast(bool en)
{
    // return enableRegisterFlag(REG07::_ADDR, REG07::I2C_BCAST_EN, en);
    DRV2624reg::_07 reg07;
    readRegister(0x07, &(reg07.reg));
    reg07.data.I2C_BCAST_EN = (unsigned)en;
    return writeRegister(0x07, &(reg07.reg));
}

int DRV2624::enableLraPeriodAverage(bool en)
{
    // return enableRegisterFlag(REG07::_ADDR, REG07::LRA_PERIOD_AVG_DIS, !en);
    DRV2624reg::_07 reg07;
    readRegister(0x07, &(reg07.reg));
    reg07.data.LRA_PERIOD_AVG_DIS = (unsigned)en;
    return writeRegister(0x07, &(reg07.reg));
}

int DRV2624::setTriggerPinFunction(DRV2624reg::_07TriggerPinFunction function)
{
    // return setRegisterValue(REG07::_ADDR, REG07::TRIG_PIN_FUNC, (char)function);
    DRV2624reg::_07 reg07;
    readRegister(0x07, &(reg07.reg));
    reg07.data.TRIG_PIN_FUNC = (unsigned)function;
    return writeRegister(0x07, &(reg07.reg));
}

int DRV2624::setMode(DRV2624reg::_07Mode mode)
{
    // return setRegisterValue(REG07::_ADDR, REG07::MODE, (char)mode);
    DRV2624reg::_07 reg07;
    readRegister(0x07, &(reg07.reg));
    reg07.data.MODE = (unsigned)mode;
    return writeRegister(0x07, &(reg07.reg));
}

//////REG08//////

int DRV2624::setLRA()
{
    // return enableRegisterFlag(REG08::_ADDR, REG08::LRA_ERM, true);
    DRV2624reg::_08 reg08;
    readRegister(0x08, &(reg08.reg));
    reg08.data.LRA_ERM = 1;
    return writeRegister(0x08, &(reg08.reg));
}

int DRV2624::setERM()
{
    // return enableRegisterFlag(REG08::_ADDR, REG08::LRA_ERM, false);
    DRV2624reg::_08 reg08;
    readRegister(0x08, &(reg08.reg));
    reg08.data.LRA_ERM = 0;
    return writeRegister(0x08, &(reg08.reg));
}

int DRV2624::enableControlLoop(bool en)
{
    // return enableRegisterFlag(REG08::_ADDR, REG08::CONTROL_LOOP, en);
    DRV2624reg::_08 reg08;
    readRegister(0x08, &(reg08.reg));
    reg08.data.CONTROL_LOOP = (unsigned)en;
    return writeRegister(0x08, &(reg08.reg));
}

int DRV2624::enableHybridLoop(bool en)
{
    // return enableRegisterFlag(REG08::_ADDR, REG08::HYBRID_LOOP, en);
    DRV2624reg::_08 reg08;
    readRegister(0x08, &(reg08.reg));
    reg08.data.HYBRID_LOOP = (unsigned)en;
    return writeRegister(0x08, &(reg08.reg));
}

//////REG0C//////

int DRV2624::go()
{
    char value = 0x01;
    return writeRegister(0x0C, &value);
}

int DRV2624::stop()
{
    char value = 0x00;
    return writeRegister(0x0C, &value);
}

//////REG0D//////

int DRV2624::setPlaybackInterval(DRV2624reg::_0dPlaybackInterval interval)
{
    // return enableRegisterFlag(REG0D::_ADDR, REG0D::PLAYBACK_INTERVAL, (bool)interval);
    DRV2624reg::_0D reg0D;
    readRegister(0x0D, &(reg0D.reg));
    reg0D.data.PLAYBACK_INTERVAL = (unsigned)interval;
    return writeRegister(0x0D, &(reg0D.reg));
}

int DRV2624::setDigMemGain(DRV2624reg::_0dDigMemGain gain)
{
    // return setRegisterValue(REG0D::_ADDR, REG0D::DIG_MEM_GAIN, (char)gain);
    DRV2624reg::_0D reg0D;
    readRegister(0x0D, &(reg0D.reg));
    reg0D.data.PLAYBACK_INTERVAL = (unsigned)gain;
    return writeRegister(0x0D, &(reg0D.reg));
}

//////REG0F - REG16//////

int DRV2624::setWaveFormSequence(char index, char value)
{
    value &= 0x7F;
    return writeRegister(indexToAddress(index), &value);
}

int DRV2624::setWaitTimeSequence(char index, char time)
{
    time |= 0x80;
    return writeRegister(indexToAddress(index), &time);
}

//////REG17 - REG18//////

int DRV2624::setWaveSequenceLoop(char index, char value)
{
    char regAddr;
    char mask;
    if (value > 3)
    {
        return -1;
    }

    if (index < 5)
    {
        DRV2624reg::_17 reg17;
        readRegister(0x17, &(reg17.reg));
        switch (index)
        {
        case 1:
            reg17.data.WAV1_SEQ_LOOP = (unsigned)value;
            break;
        case 2:
            reg17.data.WAV2_SEQ_LOOP = (unsigned)value;
            break;
        case 3:
            reg17.data.WAV3_SEQ_LOOP = (unsigned)value;
            break;
        case 4:
            reg17.data.WAV4_SEQ_LOOP = (unsigned)value;
            break;
        default:
            return -1;
            break;
        };
        return writeRegister(0x17, &(reg17.reg));
    }
    else
    {
        DRV2624reg::_18 reg18;
        readRegister(0x18, &(reg18.reg));
        switch (index)
        {
        case 5:
            reg18.data.WAV5_SEQ_LOOP = (unsigned)value;
            break;
        case 6:
            reg18.data.WAV6_SEQ_LOOP = (unsigned)value;
            break;
        case 7:
            reg18.data.WAV7_SEQ_LOOP = (unsigned)value;
            break;
        case 8:
            reg18.data.WAV8_SEQ_LOOP = (unsigned)value;
            break;
        default:
            return -1;
            break;
        };
        return writeRegister(0x18, &(reg18.reg));
    }
}

//////REG19//////

int DRV2624::setWaveSequenceMainLoop(char value)
{
    if (value > 7) //7.. infinte loop
    {
        return -1;
    }
    // return setRegisterValue(REG19::_ADDR, REG19::WAV_SEQ_MAIN_LOOP, value);
    DRV2624reg::_19 reg19;
    readRegister(0x19, &(reg19.reg));
    reg19.data.WAV_SEQ_MAIN_LOOP = (unsigned)value;
    return writeRegister(0x19, &(reg19.reg));
}

//////REG1A/////

int DRV2624::setOverdriveOpenLoop(char value)
{
    // return writeRegister(REG1A::_ADDR, &value);
    DRV2624reg::_1A reg1A;
    readRegister(0x1A, &(reg1A.reg));
    reg1A.data.ODT = (unsigned)value;
    return writeRegister(0x1A, &(reg1A.reg));
}

//////REG1F//////

int DRV2624::setRatedMotorVoltage(char value)
{
    // return writeRegister(REG1F::_ADDR, &value);
    DRV2624reg::_1F reg1F;
    readRegister(0x1F, &(reg1F.reg));
    reg1F.data.RATED_VOLTAGE = (unsigned)value;
    return writeRegister(0x1F, &(reg1F.reg));
}

//////REG27//////

int DRV2624::setLraMinFrequency(DRV2624reg::_27MinFreq freq)
{ //0..125Hz, 1..45Hz
    // return enableRegisterFlag(REG27::_ADDR, REG27::LRA_MIN_FREQ_SEL, (bool)freq);
    DRV2624reg::_27 reg27;
    readRegister(0x27, &(reg27.reg));
    reg27.data.LRA_MIN_FREQ_SEL = (unsigned)freq;
    return writeRegister(0x27, &(reg27.reg));
}

int DRV2624::setDriveTime(DRV2624reg::_27DriveTimeUs value)
{
    // return setRegisterValue(REG27::_ADDR, REG27::DRIVE_TIME, (char)value);
    DRV2624reg::_27 reg27;
    readRegister(0x27, &(reg27.reg));
    reg27.data.DRIVE_TIME = (unsigned)value;
    return writeRegister(0x27, &(reg27.reg));
}

//////REG2C//////

int DRV2624::setLraWaveShape(DRV2624reg::_2cWaveShape wave)
{
    // return enableRegisterFlag(REG2C::_ADDR, REG2C::LRA_WAVE_SHAPE, (bool)wave);
    DRV2624reg::_2C reg2C;
    readRegister(0x2C, &(reg2C.reg));
    reg2C.data.LRA_WAVE_SHAPE = (unsigned)wave;
    return writeRegister(0x2C, &(reg2C.reg));
}

//////REG2E-REG2F//////

uint16_t DRV2624::getLraPeriod()
{
    uint16_t period = 0;
    DRV2624reg::_2E reg2E;
    DRV2624reg::_2F reg2F;
    readRegister(0x2E, &(reg2E.reg));
    period |= reg2E.data.OL_LRA_PERIOD_HI << 8;
    readRegister(0x2F, &(reg2F.reg));
    period |= reg2F.data.OL_LRA_PERIOD_LO;
    return period;
}

//////REGFD-REGFF//////

int DRV2624::writeHeaderEntry(char index, uint16_t ramStartAddr, char length, char repeats)
{
    if (index < 1 || index > 127)
    { //0 is revision, max 127
        return -1;
    }

    if (!(length % 2) || length < 2 || length > 30) //5Bit
    {                                               //cannot be odd (time-value pairs), 0 is error, max 15 pairs
        return -1;
    }

    if (repeats > 7)
    { //3Bit
        return -1;
    }

    char configByte = ((repeats & 0x7) << 5) | length;
    char indexBuffer[3] = {
        (char)(ramStartAddr >> 8),     //start Address upper byte
        (char)(ramStartAddr & 0x00FF), //start Address lower byte
        configByte};

    uint16_t headerAddr = ((uint16_t)index - 1) * 3 + 1;
    return writeRAMBuffer(headerAddr, indexBuffer, 3);
}

int DRV2624::writeWaveFormToRAM(char index, uint16_t ramStartAddr, char *buffer, char length, char repeats)
{
    writeHeaderEntry(index, ramStartAddr, length, repeats);
    return writeRAMBuffer(ramStartAddr, buffer, length);
}

int DRV2624::setRAMAddr(uint16_t ramAddr)
{
    if (ramAddr > 0x7FF)
    {
        return -1;
    }

    char cmd[3] = {0xFD, (char)(ramAddr >> 8), (char)(ramAddr & 0x00FF)};
    return i2c->write((int)address, cmd, 3);

    // char ramAddrHi = (char)(ramAddr >> 8);
    // char ramAddrLo = (char)(ramAddr & 0x00FF);
    // writeRegister(REGFD::_ADDR, &ramAddrHi);
    // writeRegister(REGFE::_ADDR, &ramAddrLo);
}

int DRV2624::writeConstWave(char voltage, char time)
{
    char value = voltage & 0x7F;
    int status = 0;
    status &= writeRegister(0xFF, &value); //unset ramp bit
    status &= writeRegister(0xFF, &time);
    return status;
}

int DRV2624::writeRampWave(char voltage, char time)
{
    char value = voltage | 0x80;
    int status = 0;
    status &= writeRegister(0xFF, &value); //set ramp bit
    status &= writeRegister(0xFF, &time);
    return status;
}

int DRV2624::writeRAM1Byte(uint16_t ramAddr, char *value)
{
    setRAMAddr(ramAddr);
    return writeRegister(0xFF, value);
}

int DRV2624::readRAM1Byte(uint16_t ramAddr, char *value)
{
    setRAMAddr(ramAddr);
    return readRegister(0xFF, value);
}

int DRV2624::writeRAMBuffer(uint16_t ramAddr, char *buffer, char length)
{
    setRAMAddr(ramAddr);
    if (ramAddr + (uint16_t)length > 0x7FF)
    {
        return -1;
    }
    for (uint16_t i = 0; i < length; i++)
    {
        writeRegister(0xFF, buffer + i);
    }
    return 0;
}

int DRV2624::readRAMBuffer(uint16_t ramAddr, char *buffer, char length)
{
    setRAMAddr(ramAddr);
    if (ramAddr + (uint16_t)length > 0x7FF)
    {
        return -1;
    }
    for (uint16_t i = 0; i < length; i++)
    {
        readRegister(0xFF, buffer + i);
    }
    return 0;
}

//////HELPERS//////

char DRV2624::indexToAddress(char index)
{
    switch (index)
    {
    case 1:
        return 0x0F;
        break;
    case 2:
        return 0x10;
        break;
    case 3:
        return 0x11;
        break;
    case 4:
        return 0x12;
        break;
    case 5:
        return 0x13;
        break;
    case 6:
        return 0x14;
        break;
    case 7:
        return 0x15;
        break;
    case 8:
        return 0x16;
        break;
    default:
        return -1;
        break;
    }
}

int DRV2624::enableRegisterFlag(const char regAddr, char mask, bool en)
{
    char regValue;
    readRegister(regAddr, &regValue);
    if (en)
    {
        setBit(&regValue, mask);
    }
    else
    {
        unsetBit(&regValue, mask);
    }
    return writeRegister(regAddr, &regValue);
}

int DRV2624::setRegisterValue(const char regAddr, char mask, char value)
{
    uint8_t shift = 0;
    while (!(mask & (1 << shift++)))
        ; //getting the mask shift
    char regValue;

    readRegister(regAddr, &regValue);
    value &= ~mask;                   //bereich platt machen
    value |= (value << shift) & mask; //bereich neu belegen
    return writeRegister(regAddr, &regValue);
}

int DRV2624::writeRegister(const char reg, char *value)
{
    char cmd[2] = {reg, *value};
    return i2c->write((int)address, cmd, 2);
}

int DRV2624::readRegister(const char reg, char *value)
{
    i2c->write((int)address, (const char *)&reg, 1);
    return i2c->read((int)address, (char *)value, 1);
}

void DRV2624::setBit(char *reg, char mask)
{
    *reg |= mask;
}

void DRV2624::unsetBit(char *reg, char mask)
{
    *reg &= ~mask;
}

#endif //MBED_DRV2624_CPP