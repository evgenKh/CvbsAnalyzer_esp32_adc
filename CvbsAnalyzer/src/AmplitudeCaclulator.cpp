#include "AmplitudeCaclulator.h"
#include <math.h>
#include "SyncIntervalsCalculator.h"



void AmplitudeCaclulator::Reset()
{    
    m_state = AmplitudeCaclulatorState::k_needMoreSamples;

    m_syncValue = INVALID_VALUE;
    m_syncTreshold = INVALID_VALUE;
    m_colorMinValue = INVALID_VALUE;
    m_blankingValue = INVALID_VALUE;
    m_blackValue = INVALID_VALUE;
    m_whiteValue = INVALID_VALUE;
    m_colorMaxValue = INVALID_VALUE;

    m_amplitudeHistogram.Reset();
}

AmplitudeCaclulatorState AmplitudeCaclulator::PushSamples(const int16_t *newData, size_t newDataLen)
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

    if(newDataLenNoZeroes)
    {
        Histogram<uint32_t, int16_t, k_binsCount> newDataHistogram(0, MAX_UINT_12BIT);        
        
        for (size_t i = firstNonZeroSampleIndex; i < newDataLen; i++)
        {
            //TODO: try skip repeated samples
            newDataHistogram.PushSample(newData[i]);
        }

        if(newDataHistogram.GetSamplesCount() != newDataLen - firstNonZeroSampleIndex)
        {
            //Some samples were not pushed, probably out of bins range.
            //Could be an assert...
            m_state = AmplitudeCaclulatorState::k_samplesAccumulatedCountMismatch;
            return m_state;
        }

        if(newDataHistogram.GetSamplesCount() > 0)
        {
            //Merge local histogram to main one.
            m_amplitudeHistogram.Extent(newDataHistogram);
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

    //Find first rising edge in left half of histogram.
    constexpr int16_t k_histogramFallingEdgeMinDelta = 1;
    constexpr int16_t k_histogramRisingEdgeMinDelta = 1;
    int16_t syncTresholdBin = INVALID_VALUE;
    bool histogramWasFalling = false;
    for(size_t bin = 0; bin < m_amplitudeHistogram.size()-1; bin++)
    {
        int16_t delta = m_amplitudeHistogram[bin+1] - m_amplitudeHistogram[bin];
        //Falling
        if(delta <= (-k_histogramFallingEdgeMinDelta))
        {
             histogramWasFalling = true;
        }
        //Rising
        if(histogramWasFalling && delta >= k_histogramRisingEdgeMinDelta)
        {
            syncTresholdBin = bin;
            //m_syncValue = m_amplitudeHistogram.GetBinCenter(bin);
            //Value of bin before rising edge
            m_syncTreshold = m_amplitudeHistogram.GetBinCenter(bin);
            break;
        }
    }
    if(syncTresholdBin == INVALID_VALUE)
    {
        //Falling enge not found.
        //Ask for more samples, and after more failes, give up and set fallback value?
        //Doing lerp
        m_syncTreshold = m_amplitudeHistogram.m_sampleValuesRange.first + 
                (m_amplitudeHistogram.m_sampleValuesRange.second - m_amplitudeHistogram.m_sampleValuesRange.first)
                *k_syncTresholdDefault; 
    }

     //Find rising edge in right half of histogram.
    int16_t whiteBin = INVALID_VALUE;
    for(size_t bin = k_binsCount - 1; bin > k_binsCount/2; bin--)
    {
        if(m_amplitudeHistogram[bin] - m_amplitudeHistogram[bin-1] > k_histogramFallingEdgeMinDelta)
        {
            //Value of right popular bin
            m_whiteValue = m_amplitudeHistogram.GetBinCenter(bin);
            whiteBin = bin;
            break;
        }
    }

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

    Serial.printf("m_syncValue = %d\n", m_syncValue);
    Serial.printf("m_syncTreshold = %d\n", m_syncTreshold);
    Serial.printf("m_blankingValue= %d\n", m_blankingValue);
    Serial.printf("m_whiteValue = %d\n", m_whiteValue);

    m_state = AmplitudeCaclulatorState::k_finished;
    return m_state;
}
