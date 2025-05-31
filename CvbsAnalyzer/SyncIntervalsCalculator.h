#ifndef SyncIntervalsCalculator_H
#define SyncIntervalsCalculator_H
#include "Arduino.h"
#include <vector>
#include "Histogram.h"
#include "CvbsAnalyzerGlobals.h"

enum class SyncIntervalsCalculatorState : signed char
{
    // k_noSamples = 0,
    k_needMoreSamples = 1,
    // k_readyForCalculation = 2,
    k_calculation,
    k_finished,

    k_badData = -1,
};

class SyncIntervalsCalculator
{
public:

    SyncIntervalsCalculator() : m_syncSequenceLengthHistogram(0, k_maxSequenceLength),
                                m_notSyncSequenceLengthHistogram(0, k_maxSequenceLength)
    {
        Reset();
    }
    void Reset();
    SyncIntervalsCalculatorState PushSamples(const uint16_t *newData, size_t newDataLen, int16_t syncTreshold, size_t dataStrideSamples);
    inline SyncIntervalsCalculatorState GetState() const { return m_state; }
    inline bool IsInErrorState() const { return ((signed char)m_state < 0); }
    inline size_t GetSamplesProcessed() const { return m_samplesProcessed; }

    void Print() const;

    // positive number in array - X samples in row, >=syncThreshold.
    // Negative - X samples in row <syncTreshold
    // 0 not allowed
    // Max sequence len = +-32767. For more split to multiple.
    // std::vector<int16_t> m_sampleSequences;
    constexpr static uint16_t k_maxSequenceLengthUs = 100;
    constexpr static uint16_t k_maxSequenceLength = UsToSamplesContexpr(k_maxSequenceLengthUs);
    constexpr static size_t k_binsCount = k_maxSequenceLength; // Makes sense to *2 for precision, if REAL sampling rate is 2Mhz


    typedef Histogram<uint32_t, uint32_t, k_binsCount> HistogramType;
    HistogramType m_syncSequenceLengthHistogram;
    HistogramType m_notSyncSequenceLengthHistogram;

private:
    // constexpr static int16_t k_tooShortSequence = 1;//samples
    constexpr static size_t k_minSamplesForCalculationUs = 700; //~10 TV lines
    constexpr static size_t k_minSamplesForCalculation = UsToSamplesContexpr(k_minSamplesForCalculationUs);
    constexpr static bool k_includeFirstSequence = false; //First and last sequences are probably incomplete.
    constexpr static bool k_includeLastSequence = false; //First and last sequences are probably incomplete.

    SyncIntervalsCalculatorState m_state;
    bool m_lastSampleWasSync = false;
    uint32_t m_samplesSinceLastChange = 1;
    uint32_t m_samplesProcessed = 0; // Not samples in hystograms, but total processed
    uint32_t m_sequencesProcessed = 0;//Should be equal to samples couns in histograms, minus last one if they'reskipped.
};

#endif // SyncIntervalsCalculator_H