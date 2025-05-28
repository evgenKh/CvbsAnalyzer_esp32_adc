#include "AmplitudeCaclulator.h"
#include <math.h>
#include "SyncIntervalsCalculator.h"



void AmplitudeCaclulator::Reset()
{    
    m_state = AmplitudeCaclulatorState::k_noSamples;

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
    if(!newDataLen)
    {
        return AmplitudeCaclulatorState::k_noSamples;
    }

    int16_t newDataMinValue = MAX_UINT_12BIT;
    int16_t newDataMaxValue = 0;
    for (size_t i = 0; i < newDataLen; i++)
    {
        int16_t value = newData[i];
        if (value > newDataMaxValue)
            newDataMaxValue = value;
        if (value < newDataMinValue)
            newDataMinValue = value;
    }
    int16_t newDataRange = (newDataMaxValue - newDataMinValue);
    if(newDataRange < k_minRange)
    {
        //Skip this dataSet
        return AmplitudeCaclulatorState::k_badAmplitudeTooLow;
    }

    //Resetting min, max, range to standard values for easier merge of multiple sampleSets into one histogram
    newDataMinValue = 0;
    newDataMaxValue = MAX_UINT_12BIT;
    newDataRange = newDataMaxValue - newDataMinValue;

    // Calculate bin width
    const float binWidth = static_cast<float>(newDataRange + 1) / k_binsCount;

    Histogram<uint32_t, int16_t, k_binsCount> newDataHistogram(0, MAX_UINT_12BIT);
    
    for (size_t i = 0; i < newDataLen; i++)
    {
        newDataHistogram.PushSample(newData[i]);
    }

    if(newDataHistogram[k_binsCount-1] > static_cast<uint32_t>(newDataLen * k_highestBinMaxWeight))
    {
        //Skip this dataSet
        return AmplitudeCaclulatorState::k_badAmplitudeTooHigh;
    }

    if(newDataHistogram.m_totalCount != newDataLen)
    {
        m_state = AmplitudeCaclulatorState::k_samplesAccumulatedCountMismatch;
        return m_state;
    }

    //Merge local histogram to main one.
    assert(m_amplitudeHistogram.m_binsRangeMin == newDataHistogram.m_binsRangeMin);
    assert(m_amplitudeHistogram.m_binsRangeMax == newDataHistogram.m_binsRangeMax);
    for(size_t bin = 0; bin < k_binsCount; bin++)
    {
        m_amplitudeHistogram[bin] += newDataHistogram[bin];
    }
    m_amplitudeHistogram.m_totalCount += newDataHistogram.m_totalCount;

    if(m_amplitudeHistogram.m_totalCount >= k_minSamplesForCalculation)
    {
        //Change to k_readyForCalculation after test calculation, or just compare with k_minSamplesForCalculation ?
        m_state = AmplitudeCaclulatorState::k_readyForCalculation;
    }else{        
        m_state = AmplitudeCaclulatorState::k_needMoreSamples;
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
        m_syncTreshold = m_amplitudeHistogram.m_sampleMin + (m_amplitudeHistogram.m_sampleMax - m_amplitudeHistogram.m_sampleMin)*k_syncTresholdDefault; 
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