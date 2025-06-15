#include "AmplitudeCaclulator.h"
#include <math.h>
#include "SyncIntervalsCalculator.h"



void AmplitudeCaclulator::Reset()
{    
    m_state = AmplitudeCaclulatorState::k_needMoreSamples;

    m_syncTreshold = INVALID_VALUE;
    m_whiteValue = INVALID_VALUE;
    //m_syncValue = INVALID_VALUE;
    //m_colorMinValue = INVALID_VALUE;
    //m_blankingValue = INVALID_VALUE;
    //m_blackValue = INVALID_VALUE;
    //m_colorMaxValue = INVALID_VALUE;

    m_amplitudeHistogram.Reset();
    
    m_smallDiffTresholdFound = false;
    m_smallDiffTreshold = 0;
    m_smallDiffsHistogram.Reset();
}

AmplitudeCaclulatorState AmplitudeCaclulator::PushSamples(const uint16_t *newData, size_t newDataLen)
{
    if(newDataLen)
    {
        const size_t samplesInHistogramBeforePush = m_amplitudeHistogram.GetSamplesCount();
        //Histogram<uint32_t, uint16_t, k_binsCount> newDataHistogram(0u, MAX_UINT_12BIT);        
        
        for (size_t i = 0; i < newDataLen; i ++)
        {
            m_amplitudeHistogram.PushSample(newData[i]);
        }

        if((m_amplitudeHistogram.GetSamplesCount()-samplesInHistogramBeforePush) != newDataLen)
        {
            //Some samples were not pushed, probably out of bins range.
            //Could be an assert...
            m_state = AmplitudeCaclulatorState::k_samplesAccumulatedCountMismatch;
            return m_state;
        }
    }

    if(m_amplitudeHistogram[m_amplitudeHistogram.size()-1] > static_cast<uint32_t>(m_amplitudeHistogram.GetSamplesCount() * k_highestBinMaxWeight))
    {
        //Don't skip this dataSet, but 
        //TODO: advise user to increase attenuation
        //m_state = AmplitudeCaclulatorState::k_badAmplitudeTooHigh;
        //return m_state;
    }

    if((m_amplitudeHistogram.m_sampleValuesRange.second - m_amplitudeHistogram.m_sampleValuesRange.first) < k_minRange)
    {
        //Skip this dataSet
        //TODO: advise user to decrease attenuation
        m_state = AmplitudeCaclulatorState::k_badAmplitudeTooLow;
        return m_state;
    }

    if(m_amplitudeHistogram.GetSamplesCount() >= k_minSamplesForCalculation)
    {
        //Change to k_readyForCalculation after test calculation, or just compare with k_minSamplesForCalculation ?
        m_state = AmplitudeCaclulatorState::k_readyForCalculation;
    }else{        
        m_state = AmplitudeCaclulatorState::k_needMoreSamples;
    }

    if(!m_smallDiffTresholdFound && 
        m_amplitudeHistogram.GetSamplesCount() >= k_minSamplesForSmallDiffTresholdCalculation)
    {
        m_smallDiffTreshold = static_cast<uint16_t>(
            static_cast<float>(m_amplitudeHistogram.m_sampleValuesRange.second - m_amplitudeHistogram.m_sampleValuesRange.first)
             * k_smallDiffTresholdRelativeToSamplesRange
        );
        //0 is not not great, but OK value too.
        m_smallDiffTresholdFound = true;
    }
    if(m_smallDiffTresholdFound)
    {
        for(size_t i = 1; i < newDataLen; i++)
        {
            //Compare with previous sample
            const uint16_t diff = std::abs(static_cast<int16_t>(newData[i]) - static_cast<int16_t>(newData[i-1]));
            if(diff <= m_smallDiffTreshold)
            {
                m_smallDiffsHistogram.PushSample(newData[i]);//Count value of current sample
            }
        }
    }
    //Do not check m_smallDiffsHistogram.GetSamplesCount().
    //0% SNR data is still valid data, if m_amplitudeHistogram is full.

    return m_state;
}

AmplitudeCaclulatorState AmplitudeCaclulator::Calculate()
{
    if(m_state != AmplitudeCaclulatorState::k_readyForCalculation)
    {
        //In wrong state.
        return m_state;
    }

    CalculateSyncTreshold();
    //CalculateWhiteLevel();
    //CalculateBlankingLevel();


    m_state = AmplitudeCaclulatorState::k_finished;
    return m_state;
}


void AmplitudeCaclulator::CalculateSyncTreshold()
{    
    //Find first first reversed peak in histogram.
    const uint32_t k_syncLevelSpikeRisingEdgeMinDelta = std::max(1.0f, std::ceil(m_amplitudeHistogram.GetSamplesCount() * 0.015f)); //2.5% of histogram amplitude;
    const uint32_t k_syncLevelSpikeFallingEdgeMinDelta = std::max(1.0f, std::ceil(m_amplitudeHistogram.GetSamplesCount() * 0.005f)); //2.5% of histogram amplitude;
    const uint32_t k_nextEdgeMinDelta = 2;

    std::array<HistogramType::iterator, 3> edgeIterators;
    std::array<size_t, 3> edgeBins;
    edgeIterators.fill(m_amplitudeHistogram.end());

    edgeIterators[0] = m_amplitudeHistogram.FindRisingEdge(
        m_amplitudeHistogram.begin(), m_amplitudeHistogram.end(), k_syncLevelSpikeRisingEdgeMinDelta);

    if(edgeIterators[0] != m_amplitudeHistogram.end())
    {
        edgeIterators[1] = m_amplitudeHistogram.FindFallingEdge(
            std::next(edgeIterators[0]), m_amplitudeHistogram.end(), k_syncLevelSpikeFallingEdgeMinDelta);
    }
    
    if(edgeIterators[1] != m_amplitudeHistogram.end())
    {
        edgeIterators[2] = m_amplitudeHistogram.FindRisingEdge(
            std::next(edgeIterators[1]), m_amplitudeHistogram.end(), k_nextEdgeMinDelta);
    }

    if(edgeIterators[2] != m_amplitudeHistogram.end())
    {
        edgeBins[0] = std::distance(m_amplitudeHistogram.begin(), edgeIterators[0]);
        edgeBins[1] = std::distance(m_amplitudeHistogram.begin(), edgeIterators[1]);
        edgeBins[2] = std::distance(m_amplitudeHistogram.begin(), edgeIterators[2]);
        //CVBS_ANALYZER_LOG_INFO("Sync bins in m_amplitudeHistogram: %d, %d, %d\n", edgeBins[0], edgeBins[1], edgeBins[2]);
        m_syncTreshold = ((float)m_amplitudeHistogram.GetBinCenter(edgeBins[1]) + (float)m_amplitudeHistogram.GetBinCenter(edgeBins[2]) ) / 2.0f;
        //m_syncTreshold = m_amplitudeHistogram.GetBinCenter(edgeBins[1]);
    }
    else
    {
        //Falling enge not found.
        //Ask for more samples, and after more failes, give up and set fallback value?
        //Doing lerp
        m_syncTreshold = m_amplitudeHistogram.m_sampleValuesRange.first + 
                (m_amplitudeHistogram.m_sampleValuesRange.second - m_amplitudeHistogram.m_sampleValuesRange.first)
                *k_syncTresholdDefault; 
    }
}
/*
void AmplitudeCaclulator::CalculateBlankingLevel()
{ 
    if(whiteBin != INVALID_VALUE && syncTresholdBin != INVALID_VALUE && syncTresholdBin < whiteBin - 1)
    {
        size_t maxCount = 0;
        size_t maxCountBin = syncTresholdBin;
        //Find most popular bin BETWEEN sync and white bins
        for(size_t bin = syncTresholdBin; bin < whiteBin; bin++)
        {
            if(m_amplitudeHistogram[bin] > maxCount)
            {
                maxCount = m_amplitudeHistogram[bin];
                maxCountBin = bin;
            }
        }
        m_blankingValue = GetBinCenterValue(m_minValue, binWidth, maxCountBin);
    }
}*/

void AmplitudeCaclulator::Print() const
{
    CVBS_ANALYZER_LOG("\"AmplitudeCaclulator\": {\n");
    CVBS_ANALYZER_LOG("\t\"state\": %d, \n", static_cast<int>(m_state));
    CVBS_ANALYZER_LOG("\t\"m_syncTreshold\": %d, \n", m_syncTreshold);
    CVBS_ANALYZER_LOG("\t\"m_whiteValue\": %d, \n", m_whiteValue);
    CVBS_ANALYZER_LOG("}, \n");
}

/*
void AmplitudeCaclulator::CalculateWhiteLevel()
{
    constexpr uint32_t k_edgesMinDelta = 1;
    //Find rising edge in right half of histogram.
    auto lastRisingEdgeIt = m_amplitudeHistogram.FindRisingEdge(
        m_amplitudeHistogram.rbegin(), m_amplitudeHistogram.rend(), k_edgesMinDelta);

    int16_t whiteBin = INVALID_VALUE;
    if(lastRisingEdgeIt != m_amplitudeHistogram.rend())
    {
        whiteBin = std::distance(m_amplitudeHistogram.begin(), std::next(lastRisingEdgeIt).base());
        m_whiteValue = m_amplitudeHistogram.GetBinCenter(whiteBin);
    }
}
*/
