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
    m_sequencesProcessed = 0;
}

SyncIntervalsCalculatorState SyncIntervalsCalculator::PushSamples(const uint16_t *newData, 
                                                                    size_t newDataLen,
                                                                    int16_t syncTreshold)
{
    if(!newDataLen)
    {
        return m_state;
    }

    if(!m_samplesProcessed)//Not samples in hystograms, but total processed
    {
        //For first run force calculate this.
        m_lastSampleWasSync = (newData[0] < syncTreshold);
        m_samplesSinceLastChange = 1;
    }
    
    for (size_t i = 0; i < newDataLen; i ++)
    {
        const bool currentSampleIsSync = (newData[i] < syncTreshold);
        if(currentSampleIsSync != m_lastSampleWasSync 
            || m_samplesSinceLastChange >= k_maxSequenceLength //break too long sequences
           // || (k_includeLastSequence && m_samplesProcessed >= k_minSamplesForCalculation && i == newDataLenNoZeroes - 1)//Last sequence of last(but maybe not!) Calculation cycle
        )    
        {
            //Store previous sequnce len
            const bool skipSequence = (m_sequencesProcessed == 0) && !k_includeFirstSequence;
            
            if(!skipSequence)
            {
                auto& histogram = (m_lastSampleWasSync ? m_syncSequenceLengthHistogram : m_notSyncSequenceLengthHistogram);
                histogram.PushSample(m_samplesSinceLastChange);
            }
            m_lastSampleWasSync = currentSampleIsSync;
            m_samplesSinceLastChange = 1;
            m_sequencesProcessed++;
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
        m_state = SyncIntervalsCalculatorState::k_finished;
    }
    return m_state;
}

void SyncIntervalsCalculator::Print() const
{
    CVBS_ANALYZER_LOG("\"SyncIntervalsCalculator\": {\n");
    CVBS_ANALYZER_LOG("\t\"state\": %d, \n", static_cast<int>(m_state));
    CVBS_ANALYZER_LOG("\t\"m_syncSequenceLengthHistogram\": \n");
    m_syncSequenceLengthHistogram.Print();
    CVBS_ANALYZER_LOG("\t\"m_notSyncSequenceLengthHistogram\": \n");
    m_notSyncSequenceLengthHistogram.Print();
    CVBS_ANALYZER_LOG("}, \n");

}