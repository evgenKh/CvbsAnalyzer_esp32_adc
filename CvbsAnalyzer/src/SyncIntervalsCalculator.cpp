#include "SyncIntervalsCalculator.h"


SyncIntervalsCalculatorState SyncIntervalsCalculator::PushSamples(const int16_t *newData, size_t newDataLen, int16_t syncTreshold)
{
    if(!newDataLen)
    {
        return SyncIntervalsCalculatorState::k_noSamples;
    }

    bool lastSampleWasSync = (newData[0] < syncTreshold);
    uint32_t samplesSinceLastChange = 1;
    
    for (size_t i = 0; i < newDataLen; i++)
    {
        auto& histogram = (lastSampleWasSync ? m_syncSequenceLengthHistogram : m_notSyncSequenceLengthHistogram);
        bool currentSampleIsSync = (newData[i] < syncTreshold);
        if(currentSampleIsSync != lastSampleWasSync || samplesSinceLastChange >= k_maxSequenceLength)    
        {
            //Store previous sequnce len
            histogram.PushSample(samplesSinceLastChange);
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
        if(m_syncSequenceLengthHistogram[bin])
            Serial.printf("\t%d: %d\n", (int)m_syncSequenceLengthHistogram.GetBinCenter(bin), m_syncSequenceLengthHistogram[bin]);
    }
    Serial.printf("m_notSyncSequenceLengthHistogram = \n");
    for(int bin = 0; bin < k_binsCount; bin++)
    {
        if(m_notSyncSequenceLengthHistogram[bin])
            Serial.printf("\t%d: %d\n", (int)m_notSyncSequenceLengthHistogram.GetBinCenter(bin), m_notSyncSequenceLengthHistogram[bin]);
    }

    m_state = SyncIntervalsCalculatorState::k_finished;
    return m_state;
}

void SyncIntervalsCalculator::Reset()
{
    m_state = SyncIntervalsCalculatorState::k_noSamples;
    m_syncSequenceLengthHistogram.Reset();
    m_notSyncSequenceLengthHistogram.Reset();
    //m_sampleSequences.clear();
}