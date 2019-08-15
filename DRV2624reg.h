#ifndef MBED_DRV2624_REG
#define MBED_DRV2624_REG

class DRV2624reg
{
public:
    union _00 {
        struct d
        {
            unsigned REV : 4;
            unsigned CHIPID : 4;
        } data;
        char reg;
    };

    union _01 {
        struct d
        {
            unsigned OC_DETECT : 1;
            unsigned OVER_TEMP : 1;
            unsigned UVLO : 1;
            unsigned PROCESS_DONE : 1;
            unsigned PRG_ERROR : 1;
            unsigned RESERVED : 2;
            unsigned DIAG_RESULT : 1;
        } data;
        char reg;
    };

    union _02 {
        struct d
        {
            unsigned TOGGLE_UVLO : 1;
            unsigned TOGGLE_PROCESS_DONE : 1;
            unsigned TOGGLE_PRG_ERROR : 1;
            unsigned TOGGLE_OVER_TEMP : 1;
            unsigned TOGGLE_OC_DETECT : 1;
            unsigned RESERVED : 3;
        } data;
        char reg;
    };

    union _03 {
        struct d
        {
            unsigned DIAG_Z_RESULT : 8;
        } data;
        char reg;
    };

    union _04 {
        struct d
        {
            unsigned VBAT : 8;
        } data;
        char reg;
    };

    union _05 {
        struct d
        {
            unsigned LRA_PERIOD_HI : 2;
            unsigned RESERVED : 6;
        } data;
        char reg;
    };

    union _06 {
        struct d
        {
            unsigned LRA_PERIOD_LO : 8;
        } data;
        char reg;
    };

    union _07 {
        struct d
        {
            unsigned MODE : 2;
            unsigned TRIG_PIN_FUNC : 2;
            unsigned LINEREG_COMP_SEL : 2;
            unsigned LRA_PERIOD_AVG_DIS : 1;
            unsigned I2C_BCAST_EN : 1;
        } data;
        char reg;
    };

    enum class _07TriggerPinFunction
    {
        extPulse,
        extLevel,
        interrupt
    };

    enum class _07Mode
    {
        realTime,
        waveformSequencer,
        diagnosticsRoutine,
        autoLevelCalib
    };

    union _08 {
        struct d
        {
            unsigned RESERVED : 2;
            unsigned INPUT_SLOPE_CHECK : 1;
            unsigned AUTO_BRK_INTO_STBY : 1;
            unsigned AUTO_BRK_OL : 1;
            unsigned HYBRID_LOOP : 1;
            unsigned CONTROL_LOOP : 1;
            unsigned LRA_ERM : 1;
        } data;
        char reg;
    };

    union _09 {
        struct d
        {
            unsigned UVLO_THRES : 3;
            unsigned RESERVED : 3;
            unsigned BAT_LIFE_EXT_LVL_EN : 2;
        } data;
        char reg;
    };

    union _0A {
        struct d
        {
            unsigned BAT_LIFE_ECT_LVL1 : 8;
        } data;
        char reg;
    };

    union _0B {
        struct d
        {
            unsigned BAT_LIFE_ECT_LVL2 : 8;
        } data;
        char reg;
    };

    union _0C {
        struct d
        {
            unsigned GO : 1;
            unsigned RESERVED : 7;
        } data;
        char reg;
    };

    union _0D {
        struct d
        {
            unsigned DIG_MEM_GAIN : 2;
            unsigned RESERVED1 : 3;
            unsigned PLAYBACK_INTERVAL : 1;
            unsigned RESERVED2 : 2;

        } data;
        char reg;
    };

    enum class _0dPlaybackInterval
    {
        _1ms,
        _5ms
    };

    enum class _0dDigMemGain
    {
        _100perc,
        _75perc,
        _50perc,
        _25perc
    };

    union _0E {
        struct d
        {
            unsigned RTP_INPUT : 8;
        } data;
        char reg;
    };

    union _0F {
        struct d
        {
            unsigned WAV_FRM_SEQ1 : 7;
            unsigned WAIT1 : 1;
        } data;
        char reg;
    };

    union _10 {
        struct d
        {
            unsigned WAV_FRM_SEQ2 : 7;
            unsigned WAIT2 : 1;
        } data;
        char reg;
    };

    union _11 {
        struct d
        {
            unsigned WAV_FRM_SEQ3 : 7;
            unsigned WAIT3 : 1;
        } data;
        char reg;
    };

    union _12 {
        struct d
        {
            unsigned WAV_FRM_SEQ4 : 7;
            unsigned WAIT4 : 1;
        } data;
        char reg;
    };

    union _13 {
        struct d
        {
            unsigned WAV_FRM_SEQ5 : 7;
            unsigned WAIT5 : 1;
        } data;
        char reg;
    };

    union _14 {
        struct d
        {
            unsigned WAV_FRM_SEQ6 : 7;
            unsigned WAIT6 : 1;
        } data;
        char reg;
    };

    union _15 {
        struct d
        {
            unsigned WAV_FRM_SEQ7 : 7;
            unsigned WAIT7 : 1;
        } data;
        char reg;
    };

    union _16 {
        struct d
        {
            unsigned WAV_FRM_SEQ8 : 7;
            unsigned WAIT8 : 1;
        } data;
        char reg;
    };

    union _17 {
        struct d
        {
            unsigned WAV1_SEQ_LOOP : 2;
            unsigned WAV2_SEQ_LOOP : 2;
            unsigned WAV3_SEQ_LOOP : 2;
            unsigned WAV4_SEQ_LOOP : 2;
        } data;
        char reg;
    };

    union _18 {
        struct d
        {
            unsigned WAV5_SEQ_LOOP : 2;
            unsigned WAV6_SEQ_LOOP : 2;
            unsigned WAV7_SEQ_LOOP : 2;
            unsigned WAV8_SEQ_LOOP : 2;
        } data;
        char reg;
    };

    union _19 {
        struct d
        {
            unsigned WAV_SEQ_MAIN_LOOP : 3;
            unsigned RESERVED : 5;
        } data;
        char reg;
    };

    union _1A {
        struct d
        {
            unsigned ODT : 8;
        } data;
        char reg;
    };

    union _1B {
        struct d
        {
            unsigned SPT : 8;
        } data;
        char reg;
    };

    union _1C {
        struct d
        {
            unsigned SNT : 8;
        } data;
        char reg;
    };

    union _1D {
        struct d
        {
            unsigned BRT : 8;
        } data;
        char reg;
    };

    union _1F {
        struct d
        {
            unsigned RATED_VOLTAGE : 8;
        } data;
        char reg;
    };

    union _20 {
        struct d
        {
            unsigned OD_CLAMP : 8;
        } data;
        char reg;
    };

    union _21 {
        struct d
        {
            unsigned A_CAL_COMP : 8;
        } data;
        char reg;
    };

    union _22 {
        struct d
        {
            unsigned A_CAL_BEMF : 8;
        } data;
        char reg;
    };

    union _23 {
        struct d
        {
            unsigned BEMF_GAIN : 2;
            unsigned LOOP_GAIN : 2;
            unsigned FB_BRAKE_FACTOR : 3;
            unsigned NG_THRESH : 1;
        } data;
        char reg;
    };

    enum class _23BemfGain {
        _5x,
        _10x,
        _20x,
        _30x
    };

    union _24 {
        struct d
        {
            unsigned RATED_VOLTAGE_CLAMP : 8;
        } data;
        char reg;
    };

    union _25 {
        struct d
        {
            unsigned OD_CLAMP_LVL1 : 8;
        } data;
        char reg;
    };

    union _26 {
        struct d
        {
            unsigned OD_CLAMP_LVL2 : 8;
        } data;
        char reg;
    };

    union _27 {
        struct d
        {
            unsigned DRIVE_TIME : 4;
            unsigned RESERVED : 1;
            unsigned LRA_RESYNC_FORMAT : 1;
            unsigned LRA_MIN_FREQ_SEL : 1;
        } data;
        char reg;
    };

    enum class _27MinFreq
    {
        _125Hz,
        _145Hz
    };

    enum class _27DriveTimeUs
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

    union _28 {
        struct d
        {
            unsigned IDISS_TIME : 4;
            unsigned BLANKING_TIME : 4;
        } data;
        char reg;
    };

    enum class _28BlankingOrIdissTimeUs {
        lra15erm45,
        lra25erm75,
        lra50erm150,
        lra75erm225,
        lra90,
        lra105,
        lra120,
        lra135,
        lra150,
        lra165,
        lra180,
        lra195,
        lra210,
        lra235,
        lra260,
        lra285,
    };

    union _29 {
        struct d
        {
            unsigned ZC_DET_TIME : 2;
            unsigned SAMPLE_TIME : 2;
            unsigned OD_CLAMP_TIME : 2;
            unsigned RESERVED : 2;
        } data;
        char reg;
    };

    enum class _29SampleTime {
        _150us,
        _200us,
        _250us,
        _300us
    };

    enum class _29ZcDetTime {
        _100us,
        _200us,
        _300us,
        _390us
    };

    union _2A {
        struct d
        {
            unsigned AUTO_CAL_TIME : 2;
            unsigned RESERVED : 6;
        } data;
        char reg;
    };

    enum class _2aAutoCalTime
    {
        _250ms,
        _500ms,
        _1s,
        triggerControlled
    };

    union _2C {
        struct d
        {
            unsigned LRA_WAVE_SHAPE : 1;
            unsigned RESERVED : 4;
            unsigned AUTO_OL_CNT : 2;
            unsigned LRA_AUTO_OPEN_LOOP : 1;
        } data;
        char reg;
    };

    enum class _2cWaveShape
    {
        square,
        sine
    };

    union _2E {
        struct d
        {
            unsigned OL_LRA_PERIOD_HI : 2;
            unsigned RESERVED : 6;
        } data;
        char reg;
    };

    union _2F {
        struct d
        {
            unsigned OL_LRA_PERIOD_LO : 8;
        } data;
        char reg;
    };

    union _30 {
        struct d
        {
            unsigned CURRENT_K : 8;
        } data;
        char reg;
    };

    union _FD {
        struct d
        {
            unsigned RAM_ADDR_HI : 8;
        } data;
        char reg;
    };

    union _FE {
        struct d
        {
            unsigned RAM_ADDR_LO : 8;
        } data;
        char reg;
    };

    union _FF {
        struct d
        {
            unsigned RAM_DATA : 8;
        } data;
        char reg;
    };
};

#endif //MBED_DRV2624_REG