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
    void CalculateFromSyncIntervalsSimple(const SyncIntervalsCalculator &syncIntervalsCalculator,
                                    const AmplitudeCaclulatorState amplitudeCalculatorState);

    void CalculateFromSyncIntervals(const SyncIntervalsCalculator &syncIntervalsCalculator,
                                    const AmplitudeCaclulatorState amplitudeCalculatorState);

    void Reset()
    {
        m_isVideo = 0.0f;
    };

    float m_isVideo;

private:    
    float GetWeightOfIntervalsWithLength(const SyncIntervalsCalculator::HistogramType &histogram,
                                         const size_t minIntervalLengthUs, const size_t maxIntervalLengthUs) const;
                                        
};

#endif // VideoScore_H