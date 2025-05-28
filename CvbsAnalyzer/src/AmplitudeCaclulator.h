#ifndef AmplitudeCaclulator_H
#define AmplitudeCaclulator_H

#include "Arduino.h"

#define MAX_UINT_12BIT (4095)
#define INVALID_VALUE (-1)

enum class AmplitudeCaclulatorState : signed char
{
    k_noSamples = 0,
    k_needMoreSamples = 1,
    k_readyForCalculation = 2,
    k_calculation = 2,
    k_finished = 3,

    k_badData = -127,
    k_badAmplitudeTooLow,
    k_badAmplitudeTooHigh,
    k_badAmplitudeNoisy,
    k_samplesAccumulatedCountMismatch,
};

class AmplitudeCaclulator
{
    public:
    AmplitudeCaclulator()    
    {
        Reset();
    }
    void Reset();

    AmplitudeCaclulatorState PushSamples(const int16_t* newData, size_t newDataLen);

    AmplitudeCaclulatorState m_state = AmplitudeCaclulatorState::k_noSamples;

    int16_t m_minValue;
    int16_t m_syncValue;
    int16_t m_syncTreshold;
    int16_t m_colorMinValue;
    int16_t m_blankingValue;
    int16_t m_blackValue;
    int16_t m_whiteValue;
    int16_t m_colorMaxValue;
    int16_t m_maxValue;
    uint32_t m_samplesAccumulated;


    private:
    constexpr static size_t k_binsCount = 30;
    constexpr static int16_t k_minRange = k_binsCount; //Condition for k_badAmplitudeTooLow
    constexpr static float k_highestBinMaxWeight = 0.55f; //Condition for k_badAmplitudeTooHigh
    constexpr static size_t k_minSamplesForCalculation = 400;//200us*2Msample
    std::array<uint32_t, k_binsCount> m_amplitudeHistogram;

    //constexpr static float k_syncPulseOnlyAmplitudeColorbarsNtscM = 17.0f/170.0f;
    //constexpr static float k_syncPulseOnlyAmplitudeBlackNtscMWithColorBurst = 20.0f/60.0f;
    //constexpr static float k_syncPulseOnlyAmplitudeBlackNtscMBW = 40.0f/47.0f;
    //constexpr static float k_syncPulseOnlyAmplitudeColorbarsNtscJ = 7.0f/173.0f;
    //constexpr static float k_syncPulseOnlyAmplitudeColorbarsPal = 10.0f/176.0f;
    //
    //constexpr static float k_syncPulseMinWidthNtsc = 4.7f/63.5f;
    //constexpr static float k_syncPulseMinWidthPal = 4.7f/64.0f;//7%, byt can be ~5% in practice

    constexpr static float k_syncTresholdDefault = 0.15f;//Let's assume 15% of full signal amplitude
};

#endif