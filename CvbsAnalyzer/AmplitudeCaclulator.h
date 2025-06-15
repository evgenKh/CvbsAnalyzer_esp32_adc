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
    k_calcFromFlatness,
    k_calcFromHistogram,
    k_finished,

    k_badData = -127,
    k_badAmplitudeTooLow,
    k_badAmplitudeTooHigh,
    k_badAmplitudeNoisy,
    k_samplesAccumulatedCountMismatch,
    k_failedFlatnessNeedMoreSamples,
    k_failedFlatnessNoEdgesFound,
    k_failedHistogramNeedMoreSamples,
    k_failedHistogramNoEdgesFound,
};

class AmplitudeCaclulator
{
public:
    AmplitudeCaclulator() : m_amplitudeHistogram(0, MAX_UINT_12BIT),
                            m_flatnessAccumulator(0, MAX_UINT_12BIT)
    {
        Reset();
    }

    void Reset();

    AmplitudeCaclulatorState PushSamples(const uint16_t *newData, size_t newDataLen);
    AmplitudeCaclulatorState Calculate();

    void CalcSyncTresholdFromFlatness();
    void CalcSyncTresholdFromHistogram();
    void CalcSyncTresholdHardcoded();
    //void CalculateWhiteLevel();
    //void CalculateBlankingLevel();
    void Print() const;

    inline AmplitudeCaclulatorState GetState() const { return m_state; }
    inline bool IsInErrorState() const { return ((signed char)m_state < 0); }
    inline size_t GetMinSamplesForCalculation() const 
    {
         return std::max(
            k_minSamplesForCalcFromHistogram, 
            std::max(k_minSamplesForCalcFromFlatness, k_minSamplesForRangeCalculation)
        );
    }


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

    //This one is not actually a histogram, just using convinient wrapper.
    typedef Histogram<float, uint16_t, uint16_t, k_binsCount> FloatAccumulatorType;
    FloatAccumulatorType m_flatnessAccumulator; //Accumulating metric of line flatness
                                         //Group by sample value.
                                         //For detecting flat lines, like sync pulse.
private:

    constexpr static int16_t k_minRange = 3;//k_binsCount;        // Condition for k_badAmplitudeTooLow
    constexpr static float k_highestBinMaxWeight = 0.90f;     // Condition for k_badAmplitudeTooHigh

    constexpr static size_t k_minSamplesForCalcFromHistogramUs = 1200;//~20 TV lines
    constexpr static size_t k_minSamplesForCalcFromHistogram = UsToSamplesContexpr(k_minSamplesForCalcFromHistogramUs);
    
    
    constexpr static size_t k_minSamplesForRangeCalculationUs = 140;//~2 TV lines
    constexpr static size_t k_minSamplesForRangeCalculation = UsToSamplesContexpr(k_minSamplesForRangeCalculationUs);

    constexpr static size_t k_minSamplesForCalcFromFlatness = k_minSamplesForRangeCalculation;


    constexpr static float k_syncTresholdDefault = 0.15f; // Fallbak. Let's assume 15% of full signal amplitude
    // constexpr static float k_syncTresholdMax = 0.5f;//

    AmplitudeCaclulatorState m_state;

};

#endif