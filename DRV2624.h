/*
 * mbed library program 
 *  Texas Instruments / DRV2624 Ultra Low Power Closed-Loop LRA/ERM Haptic Driver with Internal Memory
 *      http://www.ti.com/lit/ds/symlink/drv2624.pdf
 *
 * Copyright (c) 2019, Wenzel Reichmuth / VRaktion
 *  http://mbed.org/users/vraktion/
 *      Created: August      6th, 2019
 */

#ifndef MBED_DRV2624_H
#define MBED_DRV2624_H

#include "DRV2624reg.h"

#define DRV2624_ADDR 0x5A ///< Device I2C address
#define DRV2624_BCAST_ADDR 0x58

class DRV2624
{
public:
    /** Configure data pin
      * @param data SDA and SCL pins
      * @param DRV8830 address (H/W configuration of A1,A0)
      */
    DRV2624(PinName p_sda, PinName p_scl);

    /** Configure data pin (with other devices on I2C line)
      * @param I2C previous definition
      * @param DRV8830 address (H/W configuration of A1,A0)
      */
    DRV2624(I2C *p_i2c);

    int init();
    int calibrateLRA();

    int enableI2CBroadcast(bool en);
    int enableLraPeriodAverage(bool en);
    int enableControlLoop(bool en);
    int enableHybridLoop(bool en);

    int go();
    int stop();

    int setTriggerPinFunction(DRV2624reg::_07TriggerPinFunction function);
    int setMode(DRV2624reg::_07Mode mode);
    int setLRA();
    int setERM();
    int setPlaybackInterval(DRV2624reg::_0dPlaybackInterval interval);
    int setDigMemGain(DRV2624reg::_0dDigMemGain gain);
    int setRealTimeAmplitude(char amplitude);

    int setWaveFormSequence(char index, char value);
    int setWaitTimeSequence(char index, char time);
    int setWaveSequenceLoop(char index, char value);
    int setWaveSequenceMainLoop(char value);

    int setOverdriveOpenLoop(char value);
    int setRatedMotorVoltage(char value);
    int setOCClamp(char value);
    int setAutoCalTime(DRV2624reg::_2aAutoCalTime time);
    int setBlankingTime(DRV2624reg::_28BlankingOrIdissTimeUs time);
    int setIdissTime(DRV2624reg::_28BlankingOrIdissTimeUs time);
    int setFbBrakeFactor(char value);
    int setLoopGain(char value);
    int setLraMinFrequency(DRV2624reg::_27MinFreq freq);
    int setDriveTime(DRV2624reg::_27DriveTimeUs value);
    int setLraWaveShape(DRV2624reg::_2cWaveShape wave);
    int setSampleTime(DRV2624reg::_29SampleTime time);
    int setZcDetTime(DRV2624reg::_29ZcDetTime time);

    char getDeviceId();
    char getDeviceRevision();
    bool getDiagResFlag();
    bool getPrgErrorFlag();
    bool getProcessDoneFlag();
    bool getUVLOFlag();
    bool getOverTempFlag();
    bool getOCDetectFlag();
    char getStatusFlags();
    char getDiagZResult();

    char getVoltageCompensation();
    int setVoltageCompensation(char value);
    char getFeedbackValue();
    int setFeedbackValue(char value);
    char getFeedbackGain();
    int setFeedbackGain(DRV2624reg::_23BemfGain gain);

    uint16_t getLraPeriod();

    int writeHeaderEntry(char index, uint16_t ramStartAddr, char length, char repeats);
    int writeWaveFormToRAMsimple(char index, const char *buffer, char repeats);
    int writeWaveFormToRAM(char index, uint16_t ramStartAddr, char *buffer, char length, char repeats);

    char indexToHeaderAddress(char index);

    int setRAMAddr(uint16_t ramAddr);

    int writeConstWave(char voltage, char time);
    int writeRampWave(char voltage, char time);

    int writeRAM1Byte(uint16_t ramAddr, char *value);
    int readRAM1Byte(uint16_t ramAddr, char *value);

    int writeRAMBuffer(uint16_t ramAddr, char *buffer, char length);
    int readRAMBuffer(uint16_t ramAddr, char *buffer, char length);

    int writeRegister(const char reg, char *value);
    int readRegister(const char reg, char *value);

    void setBit(char *reg, char mask);
    void unsetBit(char *reg, char mask);

    int enableRegisterFlag(const char regAddr, char mask, bool en);
    int setRegisterValue(const char regAddr, char mask, char value);

private:
    uint16_t indexToRamAddress(char id);

protected:
    char address;
    I2C *i2c;
};

#endif //MBED_DRV2624