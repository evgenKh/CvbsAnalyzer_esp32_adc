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
}

AmplitudeCaclulatorState AmplitudeCaclulator::PushSamples(const uint16_t *newData, size_t newDataLen, size_t dataStrideSamples)
{
    //Due to ADC hacks and calling zeroDma after ADC start, some datasets may have many zero samples at start.    
    size_t firstNonZeroSampleIndex = 0;
    if(k_skipLeadingZeroSamples)
    {
        for(firstNonZeroSampleIndex = 0; firstNonZeroSampleIndex < newDataLen; )
        {
            if(newData[firstNonZeroSampleIndex]!= 0)
            {
                break;
            }
            firstNonZeroSampleIndex += dataStrideSamples;
        }
    }
    const size_t newDataLenNoZeroes = newDataLen - firstNonZeroSampleIndex;

    if(newDataLenNoZeroes)
    {
        Histogram<uint32_t, uint16_t, k_binsCount> newDataHistogram(0u, MAX_UINT_12BIT);        
        
        for (size_t i = firstNonZeroSampleIndex; i < newDataLen; i += dataStrideSamples)
        {
            //TODO: try skip repeated samples
            newDataHistogram.PushSample(newData[i]);
        }

        if(newDataHistogram.GetSamplesCount() * dataStrideSamples != newDataLen - firstNonZeroSampleIndex)
        {
            //Some samples were not pushed, probably out of bins range.
            //Could be an assert...
            m_state = AmplitudeCaclulatorState::k_samplesAccumulatedCountMismatch;
            return m_state;
        }

        if(newDataHistogram.GetSamplesCount() > 0)
        {
            //Merge local histogram to main one.
            m_amplitudeHistogram.Extend(newDataHistogram);
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
        return m_state;
    }

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
    CalculateWhiteLevel();
    //CalculateBlankingLevel();


    m_state = AmplitudeCaclulatorState::k_finished;
    return m_state;
}


void AmplitudeCaclulator::CalculateSyncTreshold()
{    
    //Find first first reversed peak in histogram.
    constexpr uint32_t k_edgesMinDelta = 1;
    int16_t syncTresholdBin = INVALID_VALUE;

    auto firstRisingAfterFallingIt = m_amplitudeHistogram.end();

    auto firstLeftFallingEdgeIt = m_amplitudeHistogram.FindFallingEdge(
        m_amplitudeHistogram.begin(), m_amplitudeHistogram.end(), k_edgesMinDelta);


    if(firstLeftFallingEdgeIt != m_amplitudeHistogram.end())
    {        
        assert(std::next(firstLeftFallingEdgeIt) != m_amplitudeHistogram.end()); 

        firstRisingAfterFallingIt = m_amplitudeHistogram.FindRisingEdge(
            std::next(firstLeftFallingEdgeIt), m_amplitudeHistogram.end(), k_edgesMinDelta);
    }
    if(firstRisingAfterFallingIt != m_amplitudeHistogram.end())
    {
        syncTresholdBin = std::distance(m_amplitudeHistogram.begin(), firstRisingAfterFallingIt);
        m_syncTreshold = m_amplitudeHistogram.GetBinCenter(syncTresholdBin);
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

void AmplitudeCaclulator::CalculateBlankingLevel()
{ 
    /*if(whiteBin != INVALID_VALUE && syncTresholdBin != INVALID_VALUE && syncTresholdBin < whiteBin - 1)
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
    }*/
}

void AmplitudeCaclulator::Print() const
{
    CVBS_ANALYZER_LOG("AmplitudeCaclulator state: %d\n", static_cast<int>(m_state));
    CVBS_ANALYZER_LOG("\tm_syncTreshold = %d\n", m_syncTreshold);
    //CVBS_ANALYZER_LOG("m_blankingValue= %d\n", m_blankingValue);
    CVBS_ANALYZER_LOG("\tm_whiteValue = %d\n", m_whiteValue);
}

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

