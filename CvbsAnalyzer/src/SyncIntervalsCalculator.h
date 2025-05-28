#ifndef SyncIntervalsCalculator_H
#define SyncIntervalsCalculator_H
#include "Arduino.h"
#include <vector>
#include "Histogram.h"

enum class SyncIntervalsCalculatorState : signed char
{
    k_noSamples = 0,
    k_sampling = 1,
    k_calculation = 2,
    k_finished = 3,

    k_badData = -1,
};

class SyncIntervalsCalculator
{
    public:
    SyncIntervalsCalculator():   
        m_syncSequenceLengthHistogram(0, k_maxSequenceLength),
        m_notSyncSequenceLengthHistogram(0, k_maxSequenceLength)
    {
        Reset();
    }
    void Reset();
    SyncIntervalsCalculatorState PushSamples(const int16_t* newData, size_t newDataLen, int16_t syncTreshold);

    SyncIntervalsCalculatorState m_state;
    //positive number in array - X samples in row, >=syncThreshold. 
    //Negative - X samples in row <syncTreshold
    //0 not allowed
    //Max sequence len = +-32767. For more split to multiple.
    //std::vector<int16_t> m_sampleSequences;
    constexpr static size_t k_binsCount = 40;
    constexpr static uint16_t k_maxSequenceLength = 200;//65535
    
    Histogram<uint32_t, uint32_t, k_binsCount> m_syncSequenceLengthHistogram;
    Histogram<uint32_t, uint32_t, k_binsCount> m_notSyncSequenceLengthHistogram;
    //constexpr static int16_t k_tooShortSequence = 1;//samples
};

#endif // SyncIntervalsCalculator_H