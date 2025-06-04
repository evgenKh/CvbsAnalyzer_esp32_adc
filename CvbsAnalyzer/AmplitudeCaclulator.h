#ifndef AmplitudeCaclulator_H
#define AmplitudeCaclulator_H

#include "Arduino.h"
#include "Histogram.h"
#include "CvbsAnalyzerGlobals.h"

constexpr uint16_t MAX_UINT_12BIT = 4095;
#define INVALID_VALUE (-1)

enum class AmplitudeCaclulatorState : signed char
{
    //k_noSamples = 0,
    k_needMoreSamples = 1,
    k_readyForCalculation,
    k_calculation,
    k_finished,

    k_badData = -127,
    k_badAmplitudeTooLow,
    k_badAmplitudeTooHigh,
    k_badAmplitudeNoisy,
    k_samplesAccumulatedCountMismatch,
};

class AmplitudeCaclulator
{
public:
    AmplitudeCaclulator() : m_amplitudeHistogram(0, MAX_UINT_12BIT)
    {
        Reset();
    }

    void Reset();

    AmplitudeCaclulatorState PushSamples(const uint16_t *newData, size_t newDataLen);
    AmplitudeCaclulatorState Calculate();
    void CalculateSyncTreshold();
    void CalculateWhiteLevel();
    void CalculateBlankingLevel();
    void Print() const;

    inline AmplitudeCaclulatorState GetState() const { return m_state; }
    inline bool IsInErrorState() const { return ((signed char)m_state < 0); }


    uint16_t m_syncTreshold;
    uint16_t m_whiteValue;

    //int16_t m_syncValue;
    //int16_t m_colorMinValue;
    //int16_t m_blankingValue;
    //int16_t m_blackValue;
    //int16_t m_colorMaxValue;

    constexpr static size_t k_binsCount = 100;//Do not lower! 30 it loo low.
    typedef Histogram<uint32_t, uint16_t, k_binsCount> HistogramType;
    HistogramType m_amplitudeHistogram;
private:

    constexpr static int16_t k_minRange = 3;//k_binsCount;        // Condition for k_badAmplitudeTooLow
    constexpr static float k_highestBinMaxWeight = 0.90f;     // Condition for k_badAmplitudeTooHigh

    constexpr static size_t k_minSamplesForCalculationUs = 1200;
    constexpr static size_t k_minSamplesForCalculation = UsToSamplesContexpr(k_minSamplesForCalculationUs);
    //Important, if stride>1, we consume more samples 

    // constexpr static float k_syncPulseOnlyAmplitudeColorbarsNtscM = 17.0f/170.0f;
    // constexpr static float k_syncPulseOnlyAmplitudeBlackNtscMWithColorBurst = 20.0f/60.0f;
    // constexpr static float k_syncPulseOnlyAmplitudeBlackNtscMBW = 40.0f/47.0f;
    // constexpr static float k_syncPulseOnlyAmplitudeColorbarsNtscJ = 7.0f/173.0f;
    // constexpr static float k_syncPulseOnlyAmplitudeColorbarsPal = 10.0f/176.0f;
    //
    // constexpr static float k_syncPulseMinWidthNtsc = 4.7f/63.5f;
    // constexpr static float k_syncPulseMinWidthPal = 4.7f/64.0f;//7%, byt can be ~5% in practice

    constexpr static float k_syncTresholdDefault = 0.15f; // Fallbak. Let's assume 15% of full signal amplitude
    // constexpr static float k_syncTresholdMax = 0.5f;//

    AmplitudeCaclulatorState m_state;
};

#endif