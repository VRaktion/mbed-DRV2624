/*
 * mbed library program 
 *  Texas Instruments / DRV2624 Ultra Low Power Closed-Loop LRA/ERM Haptic Driver with Internal Memory
 *      http://www.ti.com/lit/ds/symlink/drv2624.pdf
 *
 * Copyright (c) 2019, Wenzel Reichmuth / VRaktion
 *  http://mbed.org/users/vraktion/
 *      Created: August      6th, 2019
 */

#ifndef MBED_DRV2624
#define MBED_DRV2624

#define DRV2624_ADDR 0x5A ///< Device I2C address
#define DRV2624_BCAST_ADDR 0x58

class DRV2624
{
public:
    // struct REGISTER
    // {
    // public:
        struct REG00
        {
            static const char _ADDR = 0x00;

            static const char CHIPID = 0xF0;
            static const char REV = 0x0F;
        };

        struct REG01
        {
            static const char _ADDR = 0x01;

            static const char DIAG_RESULT = 0x80;
            static const char PRG_ERROR = 0x10;
            static const char PROCESS_DONE = 0x08;
            static const char UVLO = 0x04;
            static const char OVER_TEMP = 0x02;
            static const char OC_DETECT = 0x01;
        };

        struct REG02
        {
            static const char _ADDR = 0x02;

            static const char INTZ_MASK = 0x1F;

            static const char TOGGLE_OC_DETECT = 0x01;
            static const char TOGGLE_OVER_TEMP = 0x02;
            static const char TOGGLE_UVLO = 0x04;
            static const char TOGGLE_PROCESS_DONE = 0x08;
            static const char TOGGLE_PRG_ERROR = 0x10;
        };

        struct REG03
        {
            static const char _ADDR = 0x03;

            static const char DIAG_Z_RESULT = 0xFF;
        };

        struct REG04
        {
            static const char _ADDR = 0x04;

            static const char VBAT = 0xFF;
        };

        struct REG05
        {
            static const char _ADDR = 0x05;

            static const char LRA_PERIOD_HI = 0x01;
        };

        struct REG06
        {
            static const char _ADDR = 0x06;

            static const char LRA_PERIOD_LO = 0xFF;
        };

        struct REG07
        {
            static const char _ADDR = 0x07;

            static const char I2C_BCAST_EN = 0x80;
            static const char LRA_PERIOD_AVG_DIS = 0x40;
            static const char LINEREG_COMP_SEL = 0x30;
            static const char TRIG_PIN_FUNC = 0x0C;
            static const char MODE = 0x03;
        };

        struct REG08
        {
            static const char _ADDR = 0x08;

            static const char LRA_ERM = 0x80;
            static const char CONTROL_LOOP = 0x40;
            static const char HYBRID_LOOP = 0x20;
            static const char AUTO_BRK_OL = 0x10;
            static const char AUTO_BRK_INTO_STBY = 0x08;
            static const char INPUT_SLOPE_CHECK = 0x04;
        };

        struct REG09
        {
            static const char _ADDR = 0x09;

            static const char BAT_LIFE_EXT_LVL_EN = 0xC0;
            static const char UVLO_THRES = 0x07;
        };

        struct REG0A
        {
            static const char _ADDR = 0x0A;

            static const char BAT_LIFE_ECT_LVL1 = 0xFF;
        };

        struct REG0B
        {
            static const char _ADDR = 0x0B;

            static const char BAT_LIFE_ECT_LVL2 = 0xFF;
        };

        struct REG0C
        {
            static const char _ADDR = 0x0C;

            static const char GO = 0x01;
        };

        struct REG0D
        {
            static const char _ADDR = 0x0D;

            static const char PLAYBACK_INTERVAL = 0x20;
            static const char DIG_MEM_GAIN = 0x03;
        };

        struct REG0E
        {
            static const char _ADDR = 0x0E;

            static const char RTP_INPUT = 0x7F;
        };

        struct REG0F
        {
            static const char _ADDR = 0x0F;

            static const char WAIT1 = 0x80;
            static const char WAV_FRM_SEQ1 = 0x7F;
        };

        struct REG10
        {
            static const char _ADDR = 0x10;

            static const char WAIT2 = 0x80;
            static const char WAV_FRM_SEQ2 = 0x7F;
        };

        struct REG11
        {
            static const char _ADDR = 0x11;

            static const char WAIT3 = 0x80;
            static const char WAV_FRM_SEQ3 = 0x7F;
        };

        struct REG12
        {
            static const char _ADDR = 0x12;

            static const char WAIT4 = 0x80;
            static const char WAV_FRM_SEQ4 = 0x7F;
        };

        struct REG13
        {
            static const char _ADDR = 0x13;

            static const char WAIT5 = 0x80;
            static const char WAV_FRM_SEQ5 = 0x7F;
        };

        struct REG14
        {
            static const char _ADDR = 0x14;

            static const char WAIT6 = 0x80;
            static const char WAV_FRM_SEQ6 = 0x7F;
        };

        struct REG15
        {
            static const char _ADDR = 0x15;

            static const char WAIT7 = 0x80;
            static const char WAV_FRM_SEQ7 = 0x7F;
        };

        struct REG16
        {
            static const char _ADDR = 0x16;

            static const char WAIT8 = 0x80;
            static const char WAV_FRM_SEQ8 = 0x7F;
        };

        struct REG17
        {
            static const char _ADDR = 0x17;

            static const char WAV4_SEQ_LOOP = 0xC0;
            static const char WAV3_SEQ_LOOP = 0x30;
            static const char WAV2_SEQ_LOOP = 0x0C;
            static const char WAV1_SEQ_LOOP = 0x03;
        };

        struct REG18
        {
            static const char _ADDR = 0x18;

            static const char WAV8_SEQ_LOOP = 0xC0;
            static const char WAV7_SEQ_LOOP = 0x30;
            static const char WAV6_SEQ_LOOP = 0x0C;
            static const char WAV5_SEQ_LOOP = 0x03;
        };

        struct REG19
        {
            static const char _ADDR = 0x19;

            static const char WAV_SEQ_MAIN_LOOP = 0x07;
        };

        struct REG1A
        {
            static const char _ADDR = 0x1A;

            static const char ODT = 0xFF;
        };

        struct REG1B
        {
            static const char _ADDR = 0x1B;

            static const char SPT = 0xFF;
        };

        struct REG1C
        {
            static const char _ADDR = 0x1C;

            static const char SNT = 0xFF;
        };

        struct REG1D
        {
            static const char _ADDR = 0x1D;

            static const char BRT = 0xFF;
        };

        struct REG1F
        {
            static const char _ADDR = 0x1F;

            static const char RATED_VOLTAGE = 0xFF;
        };

        struct REG20
        {
            static const char _ADDR = 0x20;

            static const char OD_CLAMP = 0xFF;
        };

        struct REG21
        {
            static const char _ADDR = 0x21;

            static const char A_CAL_COMP = 0xFF;
        };

        struct REG22
        {
            static const char _ADDR = 0x22;

            static const char A_CAL_BEMF = 0xFF;
        };

        struct REG23
        {
            static const char _ADDR = 0x23;

            static const char NG_THRESH = 0x80;
            static const char FB_BRAKE_FACTOR = 0x70;
            static const char LOOP_GAIN = 0x0C;
            static const char BEMF_GAIN = 0x03;
        };

        struct REG24
        {
            static const char _ADDR = 0x24;

            static const char RATED_VOLTAGE_CLAMP = 0xFF;
        };

        struct REG25
        {
            static const char _ADDR = 0x25;

            static const char OD_CLAMP_LVL1 = 0xFF;
        };

        struct REG26
        {
            static const char _ADDR = 0x26;

            static const char OD_CLAMP_LVL2 = 0xFF;
        };

        struct REG27
        {
            static const char _ADDR = 0x27;

            static const char LRA_MIN_FREQ_SEL = 0x80;
            static const char LRA_RESYNC_FORMAT = 0x40;
            static const char DRIVE_TIME = 0x1F;
        };

        struct REG28
        {
            static const char _ADDR = 0x28;

            static const char BLANKING_TIME = 0xF0;
            static const char IDISS_TIME = 0x0F;
        };

        struct REG29
        {
            static const char _ADDR = 0x29;

            static const char OD_CLAMP_TIME = 0x30;
            static const char SAMPLE_TIME = 0xC0;
            static const char ZC_DET_TIME = 0x03;
        };

        struct REG2A
        {
            static const char _ADDR = 0x2A;

            static const char AUTO_CAL_TIME = 0x03;
        };

        struct REG2C
        {
            static const char _ADDR = 0x2C;

            static const char LRA_AUTO_OPEN_LOOP = 0x80;
            static const char AUTO_OL_CNT = 0x60;
            static const char LRA_WAVE_SHAPE = 0x01;
        };

        struct REG2E
        {
            static const char _ADDR = 0x2E;

            static const char OL_LRA_PERIOD = 0x03;
        };

        struct REG2F
        {
            static const char _ADDR = 0x2F;

            static const char OL_LRA_PERIOD = 0xFF;
        };

        struct REG30
        {
            static const char _ADDR = 0x30;

            static const char CURRENT_K = 0xFF;
        };

        struct REGFD
        {
            static const char _ADDR = 0xFD;

            static const char RAM_ADDR_HI = 0xFF;
        };

        struct REGFE
        {
            static const char _ADDR = 0xFE;

            static const char RAM_ADDR_LO = 0xFF;
        };

        struct REGFF
        {
            static const char _ADDR = 0xFF;

            static const char RAM_DATA = 0xFF;
        };

    // };

    enum class reg07TriggerPinFunction
    {
        extPulse,
        extLevel,
        interrupt
    };

    enum class reg07Mode
    {
        realTime,
        waveformSequencer,
        diagnosticsRoutine,
        autoLevelCalib
    };

    enum class reg0dPlaybackInterval
    {
        _1ms,
        _5ms
    };

    enum class reg0dDigMemGain
    {
        _100perc,
        _75perc,
        _50perc,
        _25perc
    };

    enum class reg27MinFreq
    {
        _125Hz,
        _145Hz
    };

    enum class reg27DriveTimeUs
    {
        lra500erm1000,
        lra600erm1200,
        lra700erm1400,
        lra800erm1600,
        lra900erm1800,
        lra1000erm2000,
        lra1100erm2200,
        lra1200erm2400,
        lra1300erm2600,
        lra1400erm2800,
        lra1500erm3000,
        lra1600erm3200,
        lra1700erm3400,
        lra1800erm3600,
        lra1900erm3800,
        lra2000erm4000,
        lra2100erm4200,
        lra2200erm4400,
        lra2300erm4600,
        lra2400erm4800,
        lra2500erm5000,
        lra2600erm5200,
        lra2700erm5400,
        lra2800erm5600,
        lra2900erm5800,
        lra3000erm6000,
        lra3100erm6200,
        lra3200erm6400,
        lra3300erm6600,
        lra3400erm6800,
        lra3500erm7000,
        lra3600erm7200
    };

    enum class reg2cWaveShape
    {
        square,
        sine
    };

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

    int enableI2CBroadcast(bool en);
    int enableLraPeriodAverage(bool en);
    int enableControlLoop(bool en);
    int enableHybridLoop(bool en);

    int go();
    int stop();

    int setTriggerPinFunction(reg07TriggerPinFunction function);
    int setMode(reg07Mode mode);
    int setLRA();
    int setERM();
    int setPlaybackInterval(reg0dPlaybackInterval interval);
    int setDigMemGain(reg0dDigMemGain gain);

    int setWaveFormSequence(char index, char value);
    int setWaitTimeSequence(char index, char time);
    int setWaveSequenceLoop(char index, char value);
    int setWaveSequenceMainLoop(char value);

    int setOverdriveOpenLoop(char value);
    int setRatedMotorVoltage(char value);
    int setLraMinFrequency(reg27MinFreq freq);
    int setDriveTime(reg27DriveTimeUs value);
    int setLraWaveShape(reg2cWaveShape wave);

    uint16_t getLraPeriod();

    int writeWaveFormToRAM(char index, uint16_t ramStartAddr, char *buffer, char length, char repeats);

    char indexToAddress(char index);

    int setRAMAddr(uint16_t ramAddr);

    int writeRAM1Byte(uint16_t ramAddr, char *value);
    int readRAM1Byte(uint16_t ramAddr, char *value);

    int writeRAMBuffer(uint16_t ramAddr, char *buffer, char length);
    int readRAMBuffer(uint16_t ramAddr, char *buffer, char length);

    int writeRegister(const char reg, char *value);
    int readRegister(const char reg, char* value);

    void setBit(char *reg, char mask);
    void unsetBit(char *reg, char mask);

    int enableRegisterFlag(const char regAddr, char mask, bool en);
    int setRegisterValue(const char regAddr, char mask, char value);

protected:
    char address;
    I2C *i2c;
};

#endif //MBED_DRV2624