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
    AmplitudeCaclulator() : m_amplitudeHistogram(0, MAX_UINT_12BIT),
                            m_smallDiffsHistogram(0, MAX_UINT_12BIT)
    {
        Reset();
    }

    void Reset();

    AmplitudeCaclulatorState PushSamples(const uint16_t *newData, size_t newDataLen);
    AmplitudeCaclulatorState Calculate();
    void CalculateSyncTreshold();
    //void CalculateWhiteLevel();
    //void CalculateBlankingLevel();
    void Print() const;

    inline AmplitudeCaclulatorState GetState() const { return m_state; }
    inline bool IsInErrorState() const { return ((signed char)m_state < 0); }
    inline size_t GetMinSamplesForCalculation() const { return k_minSamplesForCalculation; }


    uint16_t m_syncTreshold;
    uint16_t m_whiteValue;

    //int16_t m_syncValue;
    //int16_t m_colorMinValue;
    //int16_t m_blankingValue;
    //int16_t m_blackValue;
    //int16_t m_colorMaxValue;

    constexpr static size_t k_binsCount = 128;//Do not lower! 30 it loo low.
    typedef Histogram<uint32_t, uint16_t, uint16_t, k_binsCount> HistogramType;
    HistogramType m_amplitudeHistogram;
    HistogramType m_smallDiffsHistogram; //Counting samples that are close to their neighbor.
                                         // Group by sample value.
                                         //For detecting flat lines, like sync pulse.
private:

    constexpr static int16_t k_minRange = 3;//k_binsCount;        // Condition for k_badAmplitudeTooLow
    constexpr static float k_highestBinMaxWeight = 0.90f;     // Condition for k_badAmplitudeTooHigh

    constexpr static size_t k_minSamplesForCalculationUs = 1200;//~20 TV lines
    constexpr static size_t k_minSamplesForCalculation = UsToSamplesContexpr(k_minSamplesForCalculationUs);
    
    
    constexpr static size_t k_minSamplesForSmallDiffTresholdCalculationUs = 140;//~2 TV lines
    constexpr static size_t k_minSamplesForSmallDiffTresholdCalculation = UsToSamplesContexpr(k_minSamplesForSmallDiffTresholdCalculationUs);

    constexpr static float k_syncTresholdDefault = 0.15f; // Fallbak. Let's assume 15% of full signal amplitude
    // constexpr static float k_syncTresholdMax = 0.5f;//

    AmplitudeCaclulatorState m_state;

    constexpr static float k_smallDiffTresholdRelativeToSamplesRange = 0.02; //2% of samples range
    uint16_t m_smallDiffTreshold = 0; //Threshold for diff between samples to consider it small, in ADC units.
    bool m_smallDiffTresholdFound = false; 
};

#endif