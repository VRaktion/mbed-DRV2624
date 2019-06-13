/*
 * mbed library program 
 *  Texas Instruments / DRV2624 Ultra Low Power Closed-Loop LRA/ERM Haptic Driver with Internal Memory
 *      http://www.ti.com/lit/ds/symlink/drv2624.pdf
 *
 * Copyright (c) 2019, Wenzel Reichmuth / VRaktion
 *  http://mbed.org/users/vraktion/
 *      Created: August      6th, 2019
 */

#include "mbed.h"
#include "DRV2624.h"
// #include "DRV2624reg.h"

DRV2624::DRV2624(PinName p_sda, PinName p_scl)
    : i2c(new I2C(p_sda, p_scl))
{
    address = (char)(DRV2624_ADDR << 1);
}

DRV2624::DRV2624(I2C &p_i2c)
    : i2c(p_i2c)
{
    address = (char)(DRV2624_ADDR << 1);
}

// Play a Waveform or Waveform Sequence from the RAM Waveform Memory
// 1. Initialize the device as listed in the Initialization Procedure section.
// 1. After power-up, wait at least 1ms before the DRV2624 device accepts I2C commands.
// 2. Assert the NRST pin (logichigh). The NRST pin can be asserted anytime during or after the wait period.
// 3. Write the MODE parameter (address0x01) to value 0x00 to remove the device from standby mode.
// 4. Run auto-calibration to configure the DRV2624 device for the desired actuator. Alternatively, rewrite the results from a previous calibration.
// 5. If using the embedded RAM memory, populate the RAM with waveforms at this time.
// 2. Select the desired TRIG/INTZ pin function by changing the TRIG_PIN_FUNC parameter.
// 3. Identify the waveform index to be played and populate the waveformsequencer.
// 4. Trigger the waveform using the desired trigger method (GO bit, or external trigger). Note that if using the interrupt functionality, only the GO bit can be used to trigger the process.
// 5. Device will automatically go into standby upon completion of the playback

int DRV2624::init()
{
    enablePeriodAverage(true);
    setLRA();
    enableControlLoop(true);

    return 0;
}

//////REG07//////

int DRV2624::enableI2CBroadcast(bool en)
{
    return enableFlag(REG07::_ADDR, REG07::I2C_BCAST_EN, en);
}

int DRV2624::enablePeriodAverage(bool en)
{
    return enableFlag(REG07::_ADDR, REG07::LRA_PERIOD_AVG_DIS, !en);
}

int DRV2624::setTriggerPinFunction(DRV2624::reg07TriggerPinFunction function)
{
    return setRegisterValue(REG07::_ADDR, REG07::TRIG_PIN_FUNC, (char)function);
}

int DRV2624::setMode(DRV2624::reg07Mode mode)
{
    return setRegisterValue(REG07::_ADDR, REG07::MODE, (char)mode);
}

//////REG08//////

int DRV2624::setLRA()
{
    return enableFlag(REG08::_ADDR, REG08::LRA_ERM, true);
}

int DRV2624::setERM()
{
    return enableFlag(REG08::_ADDR, REG08::LRA_ERM, false);
}

int DRV2624::enableControlLoop(bool en)
{
    return enableFlag(REG08::_ADDR, REG08::CONTROL_LOOP, en);
}

int DRV2624::enableHybridLoop(bool en)
{
    return enableFlag(REG08::_ADDR, REG08::HYBRID_LOOP, en);
}

//////REG0C//////

int DRV2624::go()
{
    return writeRegister(REG0C::_ADDR, 0x01);
}

int DRV2624::stop()
{
    return writeRegister(REG0C::_ADDR, 0x00);
}

//////REG0D//////

int DRV2624::setPlaybackInterval(reg0dPlaybackInterval interval)
{
    return enableRegisterFlag(REG0D::_ADDR, REG0D::PLAYBACK_INTERVAL, (bool)interval);
}

int DRV2624::setDigMemGain(DRV2624::reg0dDigMemGain gain)
{
    return setRegisterValue(REG0D::_ADDR, REG0D::DIG_MEM_GAIN, (char)gain);
}

//////REG0F - REG16//////

int DRV2624::setWaveFormSequence(char index, char value)
{
    value &= 0x7F;
    return writeRegister(indexToAddress(index), &value);
}

int DRV2624::setWaitTimeSequence(char index, char time)
{
    value |= 0x80;
    return writeRegister(indexToAddress(index), &value);
}

//////REG17 - REG18//////

int DRV2624::setWaveSequenceLoop(char index, char value)
{
    char regAddr;
    char mask;
    if (value < 0 || value > 3)
    {
        return -1;
    }
    switch (index)
    {
    case 1:
        regAddr = REG17::_ADDR;
        mask = REG17::WAV1_SEQ_LOOP;
        break;
    case 2:
        regAddr = REG17::_ADDR;
        mask = REG17::WAV2_SEQ_LOOP;
        break;
    case 3:
        regAddr = REG17::_ADDR;
        mask = REG17::WAV3_SEQ_LOOP;
        break;
    case 4:
        regAddr = REG17::_ADDR;
        mask = REG17::WAV4_SEQ_LOOP;
        break;
    case 5:
        regAddr = REG18::_ADDR;
        mask = REG18::WAV5_SEQ_LOOP;
        break;
    case 6:
        regAddr = REG18::_ADDR;
        mask = REG18::WAV6_SEQ_LOOP;
        break;
    case 7:
        regAddr = REG18::_ADDR;
        mask = REG18::WAV7_SEQ_LOOP;
        break;
    case 8:
        regAddr = REG18::_ADDR;
        mask = REG18::WAV8_SEQ_LOOP;
        break;
    default:
        return -1;
    }
    return setRegisterValue(regAddr, mask, value);
}

//////REG19//////

int DRV2624::setWaveSequenceMainLoop(char value)
{
    if (value < 0 || value > 7) //7.. infinte loop
    {
        return -1;
    }
    return setRegisterValue(REG19::_ADDR, REG19::WAV_SEQ_MAIN_LOOP, value);
}

//////REG1A/////

int DRV2624::setOverdriveOpenLoop(char value)
{
    return writeRegister(REG1A::_ADDR, &value);
}

//////REG1F//////

int DRV2624::setRatedMotorVoltage(char value)
{
    return writeRegister(REG1F::_ADDR, &value);
}

//////REG27//////

int DRV2624::setLraMinFrequency(reg27MinFreq freq)
{ //0..125Hz, 1..45Hz
    return enableRegisterFlag(REG27::_ADDR, REG27::LRA_MIN_FREQ_SEL, (bool)freq);
}

int DRV2624::setDriveTime(reg27DriveTimeUs value)
{
    return setRegisterValue(REG27::_ADDR, REG27::DRIVE_TIME, value);
}

//////REG2C//////

int DRV2624::setLraWaveShape(reg2cWaveShape wave)
{
    return enableRegisterFlag(REG2C::_ADDR, REG2C::LRA_WAVE_SHAPE, (bool)wave);
}

//////REG2E-REG2F//////

uint16_t DRV2624::getLraPeriod()
{
    uint16_t period = 0;
    char buf;
    readRegister(REG2E::_ADDR, &buf);
    period |= (buf & REG2E::OL_LRA_PERIOD) << 8;
    readRegister(REG2F::_ADDR, &buf);
    period |= buf;
    return period;
}

//////REGFD-REGFF//////

int DRV2624::writeWaveFormToRAM(char index, uint16_t ramStartAddr, char *buffer, char length, char repeats)
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
    char indexBuffer = {
        (char)(ramStartAddr >> 8),     //start Address upper byte
        (char)(ramStartAddr & 0x00FF), //start Address lower byte
        configByte};

    uint16_t headerAddr = ((uint16_t)index - 1) * 3 + 1;
    writeRAMBuffer(headerAddr, indexBuffer, 3);

    writeRAMBuffer(ramStartAddr, buffer, length);
}

int DRV2624::setRAMAddr(uint16_t ramAddr)
{
    if (ramAddr > 0x7FF)
    {
        return -1;
    }

    char cmd[2] = {REGFD::_ADDR, (char)(ramAddr >> 8), (char)(ramAddr & 0x00FF)};
    return i2c->write((int)address, cmd, 3);

    // char ramAddrHi = (char)(ramAddr >> 8);
    // char ramAddrLo = (char)(ramAddr & 0x00FF);
    // writeRegister(REGFD::_ADDR, &ramAddrHi);
    // writeRegister(REGFE::_ADDR, &ramAddrLo);
}

int DRV2624::writeRAM1Byte(uint16_t ramAddr, char *value)
{
    setRAMAddr(ramAddr);
    writeRegister(REGFF::_ADDR, value);
}

int DRV2624::readRAM1Byte(uint16_t ramAddr, char *value)
{
    setRAMAddr(ramAddr);
    readRegister(REGFF::_ADDR, value);
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
        writeRegister(REGFF::_ADDR, buffer + i);
    }
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
        readRegister(REGFF::_ADDR, buffer + i);
    }
}

//////HELPERS//////

char DRV2624::indexToAddress(char index)
{
    switch (index)
    {
    case 1:
        return REG0F::_ADDR;
        break;
    case 2:
        return REG10::_ADDR;
        break;
    case 3:
        return REG11::_ADDR;
        break;
    case 4:
        return REG12::_ADDR;
        break;
    case 5:
        return REG13::_ADDR;
        break;
    case 6:
        return REG14::_ADDR;
        break;
    case 7:
        return REG15::_ADDR;
        break;
    case 8:
        return REG16::_ADDR;
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
    while (!(bufMask & (1 << shift++)))
        ; //getting the mask shift
    char regValue;

    readRegister(regAddr, &regValue);
    value &= ~mask;                  //bereich platt machen
    value |= (value << shift) & mask //bereich neu belegen
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