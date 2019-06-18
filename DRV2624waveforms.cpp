#include "DRV2624waveforms.h"

const char waveform0[] = {
    0x7F & 127, 30, //fix
    0x80 & 0, 20,   //ramp
    0x7F & 0, 50,   //fix
};

const char waveform1[] = {
    0x80 | 127, 100, //ramp
    0x80 | 0, 100,   //ramp
    0x7F & 0, 50,    //fix
};