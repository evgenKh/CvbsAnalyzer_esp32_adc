#include "SyncIntervalsCalculator.h"



void SyncIntervalsCalculator::Reset()
{
    m_state = SyncIntervalsCalculatorState::k_needMoreSamples;
    m_syncSequenceLengthHistogram.Reset();
    m_notSyncSequenceLengthHistogram.Reset();
    //m_sampleSequences.clear();
    m_lastSampleWasSync = false;
    m_samplesSinceLastChange = 1;
    m_samplesProcessed = 0;
}

SyncIntervalsCalculatorState SyncIntervalsCalculator::PushSamples(const int16_t *newData, size_t newDataLen, int16_t syncTreshold)
{
    //Due to ADC hacks and calling zeroDma after ADC start, some datasets may have many zero samples at start.    
    size_t firstNonZeroSampleIndex = 0;
    for(firstNonZeroSampleIndex = 0; firstNonZeroSampleIndex < newDataLen; )
    {
        if(newData[firstNonZeroSampleIndex] != 0)
        {
            break;
        }
        firstNonZeroSampleIndex++;
    }
    const size_t newDataLenNoZeroes = newDataLen - firstNonZeroSampleIndex;

    if(!newDataLenNoZeroes)
    {
        return m_state;
    }

    if(!m_samplesProcessed)//Not samples in hystograms, but total processed
    {
        //For first run force calculate this.
        m_lastSampleWasSync = (newData[0] < syncTreshold);
        m_samplesSinceLastChange = 1;
    }
    
    for (size_t i = firstNonZeroSampleIndex; i < newDataLenNoZeroes; i++)
    {
        const bool currentSampleIsSync = (newData[i] < syncTreshold);
        if(currentSampleIsSync != m_lastSampleWasSync || m_samplesSinceLastChange >= k_maxSequenceLength)    
        {
            //Store previous sequnce len
            auto& histogram = (m_lastSampleWasSync ? m_syncSequenceLengthHistogram : m_notSyncSequenceLengthHistogram);
            histogram.PushSample(m_samplesSinceLastChange);
            m_lastSampleWasSync = currentSampleIsSync;
            m_samplesSinceLastChange = 1;
        }
        else 
        {
            m_samplesSinceLastChange++;
        }

        m_samplesProcessed++;
    }


    if(m_samplesProcessed < k_minSamplesForCalculation)
    {
        m_state = SyncIntervalsCalculatorState::k_needMoreSamples;
    }
    else
    {
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
    }
    return m_state;
}