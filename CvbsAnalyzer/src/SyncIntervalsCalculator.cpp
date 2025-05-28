#include "SyncIntervalsCalculator.h"


inline int16_t GetBinCenterValue(int16_t minValue, float binWidth, size_t binId)
{
    //First +0.5 is for centering, second +0.5 is for rounding
    return (minValue + static_cast<int16_t>((binId + 0.5f) * binWidth + 0.5f));
}

SyncIntervalsCalculatorState SyncIntervalsCalculator::PushSamples(const int16_t *newData, size_t newDataLen, int16_t syncTreshold)
{
    if(!newDataLen)
    {
        return SyncIntervalsCalculatorState::k_noSamples;
    }

    const float binWidth = static_cast<float>(k_maxSequenceLength + 1) / k_binsCount;

    bool lastSampleWasSync = (newData[0] < syncTreshold);
    uint32_t samplesSinceLastChange = 1;
    
    for (size_t i = 0; i < newDataLen; i++)
    {
        auto& histogram = (lastSampleWasSync ? m_syncSequenceLengthHistogram : m_notSyncSequenceLengthHistogram);
        bool currentSampleIsSync = (newData[i] < syncTreshold);
        if(currentSampleIsSync != lastSampleWasSync || samplesSinceLastChange >= k_maxSequenceLength)    
        {
            //Store previous sequnce len
            size_t binIndex = static_cast<size_t>((samplesSinceLastChange - 0) / binWidth);
            if (binIndex >= 0 && binIndex < k_binsCount) {
                histogram[binIndex]++;
            }
            lastSampleWasSync = currentSampleIsSync;
            samplesSinceLastChange = 1;
        }
        else 
        {
            samplesSinceLastChange++;
        }
    }

    Serial.printf("m_syncSequenceLengthHistogram = \n");
    for(int bin = 0; bin < k_binsCount; bin++)
    {
        Serial.printf("\t%d: %d\n", GetBinCenterValue(0, binWidth, bin), m_syncSequenceLengthHistogram[bin]);
    }
    Serial.printf("m_notSyncSequenceLengthHistogram = \n");
    for(int bin = 0; bin < k_binsCount; bin++)
    {
        Serial.printf("\t%d: %d\n", GetBinCenterValue(0, binWidth, bin), m_notSyncSequenceLengthHistogram[bin]);
    }

    m_state = SyncIntervalsCalculatorState::k_finished;
    return m_state;
}

void SyncIntervalsCalculator::Reset()
{
    m_state = SyncIntervalsCalculatorState::k_noSamples;
    m_syncSequenceLengthHistogram.fill(0);
    m_notSyncSequenceLengthHistogram.fill(0);
    //m_sampleSequences.clear();
}