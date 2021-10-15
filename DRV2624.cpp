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
    wait_us(1000);
    setMode(DRV2624reg::_07Mode::realTime); //remove Device from standby

    return 0;
}

int DRV2624::calibrateLRA() //for G1040003D @ 170Hz
{
    setMode(DRV2624reg::_07Mode::autoLevelCalib);
    setLRA();
    this->setRatedMotorVoltage(105);
    this->setODClamp(167);
    this->setDriveTime(DRV2624reg::_27DriveTimeUs::lra2900erm5800); //2.94ms

    //////////

    // this->setFbBrakeFactor(3);
    // this->setLoopGain(2);
    // this->setRatedMotorVoltage(0x6B);

    // this->setAutoCalTime(DRV2624reg::_2aAutoCalTime::_250ms);//DRV2624reg::_2aAutoCalTime::triggerControlled);//trigger is not connected
    // this->setDriveTime(DRV2624reg::_27DriveTimeUs::lra2500erm5000);//2.5ms == 200hz
    // this->setSampleTime(DRV2624reg::_29SampleTime::_300us);
    // this->setBlankingTime(DRV2624reg::_28BlankingOrIdissTimeUs::lra25erm75);
    // this->setIdissTime(DRV2624reg::_28BlankingOrIdissTimeUs::lra25erm75);
    // this->setZcDetTime(DRV2624reg::_29ZcDetTime::_100us);
    this->go();

    return 0;
}

//////REG00//////

char DRV2624::getDeviceId()
{
    DRV2624reg::_00 reg00;
    readRegister(0x00, &(reg00.reg));
    return reg00.data.CHIPID;
}

char DRV2624::getDeviceRevision()
{
    DRV2624reg::_00 reg00;
    readRegister(0x00, &(reg00.reg));
    return reg00.data.REV;
}

//////REG01//////

bool DRV2624::getDiagResFlag()
{
    DRV2624reg::_01 reg01;
    readRegister(0x01, &(reg01.reg));
    return reg01.data.DIAG_RESULT;
}

bool DRV2624::getPrgErrorFlag()
{
    DRV2624reg::_01 reg01;
    readRegister(0x01, &(reg01.reg));
    return reg01.data.PRG_ERROR;
}

bool DRV2624::getProcessDoneFlag()
{
    DRV2624reg::_01 reg01;
    readRegister(0x01, &(reg01.reg));
    return reg01.data.PROCESS_DONE;
}

bool DRV2624::getUVLOFlag()
{
    DRV2624reg::_01 reg01;
    readRegister(0x01, &(reg01.reg));
    return reg01.data.UVLO;
}

bool DRV2624::getOverTempFlag()
{
    DRV2624reg::_01 reg01;
    readRegister(0x01, &(reg01.reg));
    return reg01.data.OVER_TEMP;
}

bool DRV2624::getOCDetectFlag()
{
    DRV2624reg::_01 reg01;
    readRegister(0x01, &(reg01.reg));
    return reg01.data.OC_DETECT;
}

char DRV2624::getStatusFlags()
{
    DRV2624reg::_01 reg01;
    readRegister(0x01, &(reg01.reg));
    return reg01.reg;
}

//////REG03//////

char DRV2624::getDiagZResult()
{
    DRV2624reg::_03 reg03;
    readRegister(0x03, &(reg03.reg));
    return reg03.reg;
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
    reg08.data.LRA_ERM = 1; //0;//
    return writeRegister(0x08, &(reg08.reg));
}

int DRV2624::setERM()
{
    // return enableRegisterFlag(REG08::_ADDR, REG08::LRA_ERM, false);
    DRV2624reg::_08 reg08;
    readRegister(0x08, &(reg08.reg));
    reg08.data.LRA_ERM = 0; //1;//
    return writeRegister(0x08, &(reg08.reg));
}

int DRV2624::enableControlLoop(bool en)
{
    // return enableRegisterFlag(REG08::_ADDR, REG08::CONTROL_LOOP, en);
    DRV2624reg::_08 reg08;
    readRegister(0x08, &(reg08.reg));
    reg08.data.CONTROL_LOOP = (unsigned)!en;
    return writeRegister(0x08, &(reg08.reg));
}

int DRV2624::enableAutoBreak(bool en, bool openLoop)
{
    // return enableRegisterFlag(REG08::_ADDR, REG08::CONTROL_LOOP, en);
    DRV2624reg::_08 reg08;
    readRegister(0x08, &(reg08.reg));
    reg08.data.AUTO_BRK_INTO_STBY = (unsigned)en;
    if (openLoop)
    {
        reg08.data.AUTO_BRK_OL = (unsigned)en;
    }
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

//////REG0E//////

int DRV2624::setRealTimeAmplitude(char amplitude) //signed!
{
    return writeRegister(0x0E, &amplitude);
}

//////REG0F - REG16//////

int DRV2624::setWaveFormSequence(char index, char value)
{
    value &= 0x7F;
    return writeRegister(indexToHeaderAddress(index), &value);
}

int DRV2624::setWaitTimeSequence(char index, char time)
{
    time |= 0x80;
    return writeRegister(indexToHeaderAddress(index), &time);
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
    return 0;
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
    return writeRegister(0x1F, &value);
}

//////REG20//////

int DRV2624::setODClamp(char value)
{
    return writeRegister(0x20, &value);
}

//////REG21//////

char DRV2624::getVoltageCompensation()
{
    char res;
    readRegister(0x21, &res);
    return res;
}

int DRV2624::setVoltageCompensation(char value)
{
    return writeRegister(0x21, &value);
}

//////REG22//////

char DRV2624::getFeedbackValue()
{
    char res;
    readRegister(0x22, &res);
    return res;
}

int DRV2624::setFeedbackValue(char value)
{
    return writeRegister(0x22, &value);
}

//////REG23//////

int DRV2624::setLoopGain(char value)
{
    DRV2624reg::_23 reg23;
    readRegister(0x23, &(reg23.reg));
    reg23.data.LOOP_GAIN = (unsigned)value;
    return writeRegister(0x23, &(reg23.reg));
}

int DRV2624::setFbBrakeFactor(char value)
{
    DRV2624reg::_23 reg23;
    readRegister(0x23, &(reg23.reg));
    reg23.data.FB_BRAKE_FACTOR = (unsigned)value;
    return writeRegister(0x23, &(reg23.reg));
}

char DRV2624::getFeedbackGain()
{
    DRV2624reg::_23 reg23;
    readRegister(0x22, &(reg23.reg));
    return reg23.data.BEMF_GAIN;
}

int DRV2624::setFeedbackGain(DRV2624reg::_23BemfGain gain)
{
    DRV2624reg::_23 reg23;
    readRegister(0x23, &(reg23.reg));
    reg23.data.BEMF_GAIN = (unsigned)gain;
    return writeRegister(0x23, &(reg23.reg));
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

//////REG28//////

int DRV2624::setBlankingTime(DRV2624reg::_28BlankingOrIdissTimeUs time)
{
    // return setRegisterValue(REG27::_ADDR, REG27::DRIVE_TIME, (char)value);
    DRV2624reg::_28 reg28;
    readRegister(0x28, &(reg28.reg));
    reg28.data.BLANKING_TIME = (unsigned)time;
    return writeRegister(0x28, &(reg28.reg));
}

int DRV2624::setIdissTime(DRV2624reg::_28BlankingOrIdissTimeUs time)
{
    // return setRegisterValue(REG27::_ADDR, REG27::DRIVE_TIME, (char)value);
    DRV2624reg::_28 reg28;
    readRegister(0x28, &(reg28.reg));
    reg28.data.IDISS_TIME = (unsigned)time;
    return writeRegister(0x28, &(reg28.reg));
}

//////REG29//////

int DRV2624::setSampleTime(DRV2624reg::_29SampleTime time)
{
    DRV2624reg::_29 reg29;
    readRegister(0x29, &(reg29.reg));
    reg29.data.SAMPLE_TIME = (unsigned)time;
    return writeRegister(0x29, &(reg29.reg));
}

int DRV2624::setZcDetTime(DRV2624reg::_29ZcDetTime time)
{
    DRV2624reg::_29 reg29;
    readRegister(0x29, &(reg29.reg));
    reg29.data.ZC_DET_TIME = (unsigned)time;
    return writeRegister(0x29, &(reg29.reg));
}

//////REG2A//////

int DRV2624::setAutoCalTime(DRV2624reg::_2aAutoCalTime time)
{
    // return enableRegisterFlag(REG2C::_ADDR, REG2C::LRA_WAVE_SHAPE, (bool)wave);
    DRV2624reg::_2A reg2A;
    readRegister(0x2A, &(reg2A.reg));
    reg2A.data.AUTO_CAL_TIME = (unsigned)time;
    return writeRegister(0x2A, &(reg2A.reg));
}

//////REG2C//////

int DRV2624::setLraWaveShape(DRV2624reg::_2cWaveShape wave)
{
    // return enableRegisterFlag(REG2C::_ADDR, REG2C::LRA_WAVE_SHAPE, (bool)wave);
    DRV2624reg::_2C reg2C;
    int status = 0;
    status &= readRegister(0x2C, &(reg2C.reg));
    reg2C.data.LRA_WAVE_SHAPE = (unsigned)wave;
    status &= writeRegister(0x2C, &(reg2C.reg));
    return status;
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

// Serial pcd(USBTX, USBRX, "debug", 115200);

int DRV2624::writeHeaderEntry(char index, uint16_t ramStartAddr, char length, char repeats)
{
    if (index < 1 || index > 127)
    { //0 is revision, max 127
        return -1;
    }

    if (length % 2 || length < 2 || length > 30) //5Bit
    {                                            //cannot be odd (time-value pairs), 0 is error, max 15 pairs
        return -1;
    }

    if (repeats > 7)
    { //3Bit
        return -1;
    }

    char configByte = (((repeats - 1) & 0x7) << 5) | length; //0 = one repeat, 7 = infinite repeat
    char indexBuffer[3] = {
        (char)(ramStartAddr >> 8),     //start Address upper byte
        (char)(ramStartAddr & 0x00FF), //start Address lower byte
        configByte};

    uint16_t headerAddr = ((uint16_t)index - 1) * 3 + 1;
    return writeRAMBuffer(headerAddr, indexBuffer, 3);
}

int DRV2624::writeWaveFormToRAMsimple(char index, const char *buffer, char repeats)
{
    uint16_t ramStartAddr = indexToRamAddress(index);
    char length = sizeof(buffer);
    if (ramStartAddr != 0)
    {
        // pcd.printf("length %d \r\n", length);
        return writeWaveFormToRAM(index, ramStartAddr, (char *)buffer, length, repeats);
    }
    else
    {
        return -1;
    }
}

int DRV2624::writeWaveFormToRAM(char index, uint16_t ramStartAddr, char *buffer, char length, char repeats)
{
    int status = 0;
    status &= writeHeaderEntry(index, ramStartAddr, length, repeats);
    status &= writeRAMBuffer(ramStartAddr, buffer, length);
    return status;
}

int DRV2624::setRAMAddr(uint16_t ramAddr)
{
    if (ramAddr > 0x07FF)
    {
        // pcd.printf("ram addr error\r\n");
        return -1;
    }

    // char cmd[3] = {0xFD, (char)(ramAddr >> 8), (char)(ramAddr & 0x00FF)};
    // return i2c->write((int)address, cmd, 3);

    char ramAddrHi = (char)(ramAddr >> 8);
    char ramAddrLo = (char)(ramAddr & 0x00FF);
    int status = 0;
    status &= writeRegister(0xFD, &ramAddrHi);
    status &= writeRegister(0xFE, &ramAddrLo);
    return status;
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
    int status = 0;
    status &= setRAMAddr(ramAddr);
    status &= writeRegister(0xFF, value);
    return status;
}

int DRV2624::readRAM1Byte(uint16_t ramAddr, char *value)
{
    int status = 0;
    status &= setRAMAddr(ramAddr);
    status &= readRegister(0xFF, value);
    return status;
}

int DRV2624::writeRAMBuffer(uint16_t ramAddr, char *buffer, char length)
{
    int status = 0;
    status &= setRAMAddr(ramAddr);
    if (ramAddr + (uint16_t)length > 0x7FF)
    {
        return -1;
    }
    for (uint16_t i = 0; i < length; i++)
    {
        status &= writeRegister(0xFF, buffer + i);
    }
    return status;
}

int DRV2624::readRAMBuffer(uint16_t ramAddr, char *buffer, char length)
{
    int status = 0;
    status &= setRAMAddr(ramAddr);
    if (ramAddr + (uint16_t)length > 0x7FF)
    {
        return -1;
    }
    for (uint16_t i = 0; i < length; i++)
    {
        status &= readRegister(0xFF, buffer + i);
    }
    return status;
}

//////HELPERS//////

char DRV2624::indexToHeaderAddress(char index)
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
    return 0;
}

uint16_t DRV2624::indexToRamAddress(char id)
{
    if (id < 31)
    {
        return ((uint16_t)id - 1) * 0x1E + 0x5B;
    }
    else
    {
        return 0;
    }
}

int DRV2624::enableRegisterFlag(const char regAddr, char mask, bool en)
{
    char regValue;
    int status = 0;
    status &= readRegister(regAddr, &regValue);
    if (en)
    {
        setBit(&regValue, mask);
    }
    else
    {
        unsetBit(&regValue, mask);
    }
    status &= writeRegister(regAddr, &regValue);
    return status;
}

int DRV2624::setRegisterValue(const char regAddr, char mask, char value)
{
    uint8_t shift = 0;
    while (!(mask & (1 << shift++)) && shift < 8)
        ; //getting the mask shift

    if (shift > 8)
    {
        return 1;
    }

    char regValue;

    int status = 0;
    status &= readRegister(regAddr, &regValue);
    value &= ~mask;                   //bereich platt machen
    value |= (value << shift) & mask; //bereich neu belegen
    status &= writeRegister(regAddr, &regValue);
    return status;
}

int DRV2624::writeRegister(const char reg, char *value)
{
    char cmd[2] = {reg, *value};
    // pcd.printf("write reg %x: %x\r\n", reg, *value);

    int status = i2c->write((int)address, cmd, 2);

    if (status)
    {
        printf("ERROR writing DRV2624\n\r");
    }
    return status;
}

int DRV2624::readRegister(const char reg, char *value)
{
    int status = 0;

    status = i2c->write((int)address, (const char *)&reg, 1);
    if (status)
    {
        printf("ERROR reading DRV2624: write\n\r");
        return status;
    }
    status = i2c->read((int)address, (char *)value, 1);

    // pcd.printf("read reg %x: %x\r\n", reg, *value);
    if (status)
    {
        printf("ERROR reading DRV2624: read\n\r");
    }
    return status;
}

void DRV2624::setBit(char *reg, char mask)
{
    *reg |= mask;
}

void DRV2624::unsetBit(char *reg, char mask)
{
    *reg &= ~mask;
}