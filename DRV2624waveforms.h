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

struct waveFormData{
    const char voltage;//0..127 | -63..63
    const char time;// depending PLAYBACK_INTERVAL Bit x1ms | x5ms 0..255
};

extern const char waveform0[];
extern const char waveform1[];