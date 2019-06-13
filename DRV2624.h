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
    struct REGISTER
    {
    public:
        struct REG00
        {
            static const uint8_t _ADDR = 0x00;

            static const uint8_t CHIPID = 0xF0;
            static const uint8_t REV = 0x0F;
        };

        struct REG01
        {
            static const uint8_t _ADDR = 0x01;

            static const uint8_t DIAG_RESULT = 0x80;
            static const uint8_t PRG_ERROR = 0x10;
            static const uint8_t PROCESS_DONE = 0x08;
            static const uint8_t UVLO = 0x04;
            static const uint8_t OVER_TEMP = 0x02;
            static const uint8_t OC_DETECT = 0x01;
        };

        struct REG02
        {
            static const uint8_t _ADDR = 0x02;

            static const uint8_t INTZ_MASK = 0x1F;

            static const uint8_t TOGGLE_OC_DETECT = 0x01;
            static const uint8_t TOGGLE_OVER_TEMP = 0x02;
            static const uint8_t TOGGLE_UVLO = 0x04;
            static const uint8_t TOGGLE_PROCESS_DONE = 0x08;
            static const uint8_t TOGGLE_PRG_ERROR = 0x10;
        };

        struct REG03
        {
            static const uint8_t _ADDR = 0x03;

            static const uint8_t DIAG_Z_RESULT = 0xFF;
        };

        struct REG04
        {
            static const uint8_t _ADDR = 0x04;

            static const uint8_t VBAT = 0xFF;
        };

        struct REG05
        {
            static const uint8_t _ADDR = 0x05;

            static const uint8_t LRA_PERIOD_HI = 0x01;
        };

        struct REG06
        {
            static const uint8_t _ADDR = 0x06;

            static const uint8_t LRA_PERIOD_LO = 0xFF;
        };

        struct REG07
        {
            static const uint8_t _ADDR = 0x07;

            static const uint8_t I2C_BCAST_EN = 0x80;
            static const uint8_t LRA_PERIOD_AVG_DIS = 0x40;
            static const uint8_t LINEREG_COMP_SEL = 0x30;
            static const uint8_t TRIG_PIN_FUNC = 0x0C;
            static const uint8_t MODE = 0x03;
        };

        enum class reg07TriggerPinFunction
        {
            extPulse,
            extLevel,
            interrupt
        }

        enum class reg07Mode
        {
            realTime,
            waveformSequencer,
            diagnosticsRoutine,
            autoLevelCalib
        }

        struct REG08
        {
            static const uint8_t _ADDR = 0x08;

            static const uint8_t LRA_ERM = 0x80;
            static const uint8_t CONTROL_LOOP = 0x40;
            static const uint8_t HYBRID_LOOP = 0x20;
            static const uint8_t AUTO_BRK_OL = 0x10;
            static const uint8_t AUTO_BRK_INTO_STBY = 0x08;
            static const uint8_t INPUT_SLOPE_CHECK = 0x04;
        };

        struct REG09
        {
            static const uint8_t _ADDR = 0x09;

            static const uint8_t BAT_LIFE_EXT_LVL_EN = 0xC0;
            static const uint8_t UVLO_THRES = 0x07;
        };

        struct REG0A
        {
            static const uint8_t _ADDR = 0x0A;

            static const uint8_t BAT_LIFE_ECT_LVL1 = 0xFF;
        };

        struct REG0B
        {
            static const uint8_t _ADDR = 0x0B;

            static const uint8_t BAT_LIFE_ECT_LVL2 = 0xFF;
        };

        struct REG0C
        {
            static const uint8_t _ADDR = 0x0C;

            static const uint8_t GO = 0x01;
        };

        struct REG0D
        {
            static const uint8_t _ADDR = 0x0D;

            static const uint8_t PLAYBACK_INTERVAL = 0x20;
            static const uint8_t DIG_MEM_GAIN = 0x03;
        };

        enum class reg0dPlaybackInterval
        {
            _1ms,
            _5ms
        }

        enum class reg0dDigMemGain
        {
            _100perc,
            _75perc,
            _50perc,
            _25perc
        }

        struct REG0E
        {
            static const uint8_t _ADDR = 0x0E;

            static const uint8_t RTP_INPUT = 0x7F;
        };

        struct REG0F
        {
            static const uint8_t _ADDR = 0x0F;

            static const uint8_t WAIT1 = 0x80;
            static const uint8_t WAV_FRM_SEQ1 = 0x7F;
        };

        struct REG10
        {
            static const uint8_t _ADDR = 0x10;

            static const uint8_t WAIT2 = 0x80;
            static const uint8_t WAV_FRM_SEQ2 = 0x7F;
        };

        struct REG11
        {
            static const uint8_t _ADDR = 0x11;

            static const uint8_t WAIT3 = 0x80;
            static const uint8_t WAV_FRM_SEQ3 = 0x7F;
        };

        struct REG12
        {
            static const uint8_t _ADDR = 0x12;

            static const uint8_t WAIT4 = 0x80;
            static const uint8_t WAV_FRM_SEQ4 = 0x7F;
        };

        struct REG13
        {
            static const uint8_t _ADDR = 0x13;

            static const uint8_t WAIT5 = 0x80;
            static const uint8_t WAV_FRM_SEQ5 = 0x7F;
        };

        struct REG14
        {
            static const uint8_t _ADDR = 0x14;

            static const uint8_t WAIT6 = 0x80;
            static const uint8_t WAV_FRM_SEQ6 = 0x7F;
        };

        struct REG15
        {
            static const uint8_t _ADDR = 0x15;

            static const uint8_t WAIT7 = 0x80;
            static const uint8_t WAV_FRM_SEQ7 = 0x7F;
        };

        struct REG16
        {
            static const uint8_t _ADDR = 0x16;

            static const uint8_t WAIT8 = 0x80;
            static const uint8_t WAV_FRM_SEQ8 = 0x7F;
        };

        struct REG17
        {
            static const uint8_t _ADDR = 0x17;

            static const uint8_t WAV4_SEQ_LOOP = 0xC0;
            static const uint8_t WAV3_SEQ_LOOP = 0x30;
            static const uint8_t WAV2_SEQ_LOOP = 0x0C;
            static const uint8_t WAV1_SEQ_LOOP = 0x03;
        };

        struct REG18
        {
            static const uint8_t _ADDR = 0x18;

            static const uint8_t WAV8_SEQ_LOOP = 0xC0;
            static const uint8_t WAV7_SEQ_LOOP = 0x30;
            static const uint8_t WAV6_SEQ_LOOP = 0x0C;
            static const uint8_t WAV5_SEQ_LOOP = 0x03;
        };

        struct REG19
        {
            static const uint8_t _ADDR = 0x19;

            static const uint8_t WAV_SEQ_MAIN_LOOP = 0x07;
        };

        struct REG1A
        {
            static const uint8_t _ADDR = 0x1A;

            static const uint8_t ODT = 0xFF;
        };

        struct REG1B
        {
            static const uint8_t _ADDR = 0x1B;

            static const uint8_t SPT = 0xFF;
        };

        struct REG1C
        {
            static const uint8_t _ADDR = 0x1C;

            static const uint8_t SNT = 0xFF;
        };

        struct REG1D
        {
            static const uint8_t _ADDR = 0x1D;

            static const uint8_t BRT = 0xFF;
        };

        struct REG1F
        {
            static const uint8_t _ADDR = 0x1F;

            static const uint8_t RATED_VOLTAGE = 0xFF;
        };

        struct REG20
        {
            static const uint8_t _ADDR = 0x20;

            static const uint8_t OD_CLAMP = 0xFF;
        };

        struct REG21
        {
            static const uint8_t _ADDR = 0x21;

            static const uint8_t A_CAL_COMP = 0xFF;
        };

        struct REG22
        {
            static const uint8_t _ADDR = 0x22;

            static const uint8_t A_CAL_BEMF = 0xFF;
        };

        struct REG23
        {
            static const uint8_t _ADDR = 0x23;

            static const uint8_t NG_THRESH = 0x80;
            static const uint8_t FB_BRAKE_FACTOR = 0x70;
            static const uint8_t LOOP_GAIN = 0x0C;
            static const uint8_t BEMF_GAIN = 0x03;
        };

        struct REG24
        {
            static const uint8_t _ADDR = 0x24;

            static const uint8_t RATED_VOLTAGE_CLAMP = 0xFF;
        };

        struct REG25
        {
            static const uint8_t _ADDR = 0x25;

            static const uint8_t OD_CLAMP_LVL1 = 0xFF;
        };

        struct REG26
        {
            static const uint8_t _ADDR = 0x26;

            static const uint8_t OD_CLAMP_LVL2 = 0xFF;
        };

        struct REG27
        {
            static const uint8_t _ADDR = 0x27;

            static const uint8_t LRA_MIN_FREQ_SEL = 0x80;
            static const uint8_t LRA_RESYNC_FORMAT = 0x40;
            static const uint8_t DRIVE_TIME = 0x1F;
        };

        enum class reg27MinFreq
        {
            _125Hz,
            _145Hz
        }

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
        }

        struct REG28
        {
            static const uint8_t _ADDR = 0x28;

            static const uint8_t BLANKING_TIME = 0xF0;
            static const uint8_t IDISS_TIME = 0x0F;
        };

        struct REG29
        {
            static const uint8_t _ADDR = 0x29;

            static const uint8_t OD_CLAMP_TIME = 0x30;
            static const uint8_t SAMPLE_TIME = 0xC0;
            static const uint8_t ZC_DET_TIME = 0x03;
        };

        struct REG2A
        {
            static const uint8_t _ADDR = 0x2A;

            static const uint8_t AUTO_CAL_TIME = 0x03;
        };

        struct REG2C
        {
            static const uint8_t _ADDR = 0x2C;

            static const uint8_t LRA_AUTO_OPEN_LOOP = 0x80;
            static const uint8_t AUTO_OL_CNT = 0x60;
            static const uint8_t LRA_WAVE_SHAPE = 0x01;
        };

        enum class reg2cWaveShape
        {
            square,
            sine
        }

        struct REG2E
        {
            static const uint8_t _ADDR = 0x2E;

            static const uint8_t OL_LRA_PERIOD = 0x03;
        };

        struct REG2F
        {
            static const uint8_t _ADDR = 0x2F;

            static const uint8_t OL_LRA_PERIOD = 0xFF;
        };

        struct REG30
        {
            static const uint8_t _ADDR = 0x30;

            static const uint8_t CURRENT_K = 0xFF;
        };

        struct REGFD
        {
            static const uint8_t _ADDR = 0xFD;

            static const uint8_t RAM_ADDR_HI = 0xFF;
        };

        struct REGFE
        {
            static const uint8_t _ADDR = 0xFE;

            static const uint8_t RAM_ADDR_LO = 0xFF;
        };

        struct REGFF
        {
            static const uint8_t _ADDR = 0xFF;

            static const uint8_t RAM_DATA = 0xFF;
        };

    }

    /** Configure data pin
      * @param data SDA and SCL pins
      * @param DRV8830 address (H/W configuration of A1,A0)
      */
    DRV2624(PinName p_sda, PinName p_scl);

    /** Configure data pin (with other devices on I2C line)
      * @param I2C previous definition
      * @param DRV8830 address (H/W configuration of A1,A0)
      */
    DRV2624(I2C &p_i2c);

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
    int setLraWaveShape(bool sel);

    uint16_t getLraPeriod();

    char indexToAddress(char index);

    int setRAMAddr(uint16_t ramAddr);

    int writeRAM1Byte(uint16_t ramAddr, char *value);
    int readRAM1Byte(uint16_t ramAddr, char *value);

    int writeRAMBuffer(uint16_t ramAddr, char *buffer, char length);
    int readRAMBuffer(uint16_t ramAddr, char *buffer, char length);

    int writeRegister(const char reg, char *value);
    int readRegister(const char reg, char* value));

    int setBit(char *reg, char mask);
    int unsetBit(char *reg, char mask);

    int enableRegisterFlag(const char regAddr, char mask, bool en);
    int setRegisterValue(const char regAddr, char mask, char value);

protected:
    char address;
    I2C *i2c;
};

#endif //MBED_DRV2624