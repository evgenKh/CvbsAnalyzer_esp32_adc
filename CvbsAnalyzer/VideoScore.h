#ifndef VideoScore_H
#define VideoScore_H

#include "Arduino.h"
#include "SyncIntervalsCalculator.h"

//class SyncIntervalsCalculator;
enum class AmplitudeCaclulatorState : signed char;

class VideoScore
{
public:
    VideoScore()
    {
        Reset();
    }
    void CalculateFromSyncIntervals(const SyncIntervalsCalculator &syncIntervalsCalculator,
                                    const AmplitudeCaclulatorState amplitudeCalculatorState,
                                    //const uint32_t sampleRate,
                                    const bool dataWasInverted);

    void Reset()
    {
        m_isVideo = 0.0f;
        m_isInvertedVideo = 0.0f;
    };

    float m_isVideo;
    float m_isInvertedVideo;

private:    
    float GetWeightOfIntervalsWithLength(const SyncIntervalsCalculator::HistogramType &histogram,
                                         const size_t minIntervalLengthUs, const size_t maxIntervalLengthUs) const;
                                        
    constexpr static size_t k_notSyncIntervalGoodUs = 60;
    constexpr static size_t k_notSyncIntervalGoodDeviationUs = 3;//57-63us
};

#endif // VideoScore_H