#ifndef SyncIntervalsCalculator_H
#define SyncIntervalsCalculator_H
#include "Arduino.h"
#include <vector>
#include "Histogram.h"

enum class SyncIntervalsCalculatorState : signed char
{
    //k_noSamples = 0,
    k_needMoreSamples = 1,
    //k_readyForCalculation = 2,
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
    inline SyncIntervalsCalculatorState GetState() const { return m_state; }

    //positive number in array - X samples in row, >=syncThreshold. 
    //Negative - X samples in row <syncTreshold
    //0 not allowed
    //Max sequence len = +-32767. For more split to multiple.
    //std::vector<int16_t> m_sampleSequences;
    constexpr static uint16_t k_maxSequenceLength = 200;//65535
    constexpr static size_t k_binsCount = k_maxSequenceLength;
    constexpr static size_t k_minSamplesForCalculation = 1400;
    
    Histogram<uint32_t, uint32_t, k_binsCount> m_syncSequenceLengthHistogram;
    Histogram<uint32_t, uint32_t, k_binsCount> m_notSyncSequenceLengthHistogram;

    private:
        SyncIntervalsCalculatorState m_state;    
        bool m_lastSampleWasSync = false;
        uint32_t m_samplesSinceLastChange = 1;
        uint32_t m_samplesProcessed = 0;//Not samples in hystograms, but total processed
    //constexpr static int16_t k_tooShortSequence = 1;//samples
};

#endif // SyncIntervalsCalculator_H