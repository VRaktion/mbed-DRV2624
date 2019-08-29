#ifndef MBED_DRV2624_WAVEFORMS_H
#define MBED_DRV2624_WAVEFORMS_H

// union waveFormData{
//     struct data{
//         unsigned ramp : 1;
//         unsigned voltage: 6;
//         unsigned time: 7;
//     };
//     struct reg{
//         char b0;
//         char b1;
//     };
// };

// struct waveFormData{
//     const char voltage;//0..127 | -63..63
//     const char time;// depending PLAYBACK_INTERVAL Bit x1ms | x5ms 0..255
// };

namespace DRV2624waveforms
{

const char rectShort[6] = {
    0x7F & 0,
    5,
    0x7F & 63,
    75,
    0x7F & 0,
    5,
};

const char rectLong[6] = {
    0x7F & 0,
    20,
    0x7F & 63,
    255,
    0x7F & 0,
    20,
};

const char rampUpShort[6] = {
    0x80 | 0,
    75,
    0x7F & 63,
    5,
    0x7F & 0,
    5,
};

const char rampUpLong[6] = {
    0x80 | 0,
    255,
    0x7F & 63,
    20,
    0x7F & 0,
    20,
};

const char rampDownShort[6] = {
    0x7F & 0,
    5,
    0x80 | 63,
    5,
    0x7F & 0,
    75,
};

const char rampDownLong[6] = {
    0x7F & 0,
    20,
    0x80 | 63,
    20,
    0x7F & 0,
    255,
};

const char triangleShort[6] = {
    0x80 | 0,
    35,
    0x80 | 63,
    35,
    0x7F & 0,
    5};

const char triangleLong[6] = {
    0x80 | 0,
    150,
    0x80 | 63,
    150,
    0x7F & 0,
    20};

const char sineShort[30] = {
    0x80 | 0,
    5,
    0x80 | 14,
    5,
    0x80 | 27,
    5,
    0x80 | 39,
    5,
    0x80 | 49,
    5,
    0x80 | 57,
    5,
    0x80 | 61,
    5,
    0x80 | 63,
    5,
    0x80 | 61,
    5,
    0x80 | 57,
    5,
    0x80 | 49,
    5,
    0x80 | 39,
    5,
    0x80 | 27,
    5,
    0x80 | 14,
    5,
    0x80 | 0,
    5,
};

const char sineLong[30] = {
    0x80 | 0,
    20,
    0x80 | 14,
    20,
    0x80 | 27,
    20,
    0x80 | 39,
    20,
    0x80 | 49,
    20,
    0x80 | 57,
    20,
    0x80 | 61,
    20,
    0x80 | 63,
    20,
    0x80 | 61,
    20,
    0x80 | 57,
    20,
    0x80 | 49,
    20,
    0x80 | 39,
    20,
    0x80 | 27,
    20,
    0x80 | 14,
    20,
    0x80 | 0,
    20,
};

const char quartSine[8] = {
    0, 14, 27, 39, 49, 57, 61, 63};

} // namespace DRV2624waveforms

#endif //MBED_DRV2624_WAVEFORMS_H