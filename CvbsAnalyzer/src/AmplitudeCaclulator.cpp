#include "AmplitudeCaclulator.h"
#include <math.h>

AmplitudeCaclulator::AmplitudeCaclulator()
{
    Reset();
}

void AmplitudeCaclulator::Reset()
{    
    m_state = AmplitudeCaclulatorState::k_noData;

    m_minValue = MAX_UINT_12BIT;
    m_syncValue = INVALID_VALUE;
    m_syncTreshold = INVALID_VALUE;
    m_colorMinValue = INVALID_VALUE;
    m_blankingValue = INVALID_VALUE;
    m_blackValue = INVALID_VALUE;
    m_whiteValue = INVALID_VALUE;
    m_colorMaxValue = INVALID_VALUE;
    m_maxValue = 0;

    m_amplitudeHistogram.fill(0);
}

inline int16_t GetBinValue(int16_t minValue, float binWidth, size_t binId)
{
    return (minValue + static_cast<int16_t>(binId * binWidth + 0.5f));
}

AmplitudeCaclulatorState AmplitudeCaclulator::PushData(const int16_t *newData, size_t newDataLen)
{
    if(!newDataLen)
    {
        return AmplitudeCaclulatorState::k_noData;
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
    const int16_t newDataRange = (newDataMaxValue - newDataMinValue);
    if(newDataRange < k_minRange)
    {
        return AmplitudeCaclulatorState::k_badAmplitudeTooLow;
    }

    // Calculate bin width
    const float binWidth = static_cast<float>(newDataRange + 1) / k_binsCount;

    std::array<uint32_t, k_binsCount> newDataHistogram;
    newDataHistogram.fill(0);
    for (size_t i = 0; i < newDataLen; i++)
    {
        size_t binIndex = static_cast<size_t>((newData[i] - newDataMinValue) / binWidth);
        if (binIndex >= 0 && binIndex < k_binsCount) {
            newDataHistogram[binIndex]++;
        }
    }

    if(newDataHistogram[k_binsCount-1] > static_cast<uint32_t>(newDataLen * k_highestBinMaxWeight))
    {
        return AmplitudeCaclulatorState::k_badAmplitudeTooHigh;
    }

    //Find falling edge in left half of histogram.
    constexpr int16_t k_histogramFallingEdgeMinDelta = 1;
    for(size_t bin = 0; bin < k_binsCount/2; bin++)
    {
        if(newDataHistogram[bin] - newDataHistogram[bin+1] > k_histogramFallingEdgeMinDelta)
        {
            m_syncValue = GetBinValue(newDataMinValue, binWidth, bin);
            //Value of next (non-popular) bin
            m_syncTreshold = GetBinValue(newDataMinValue, binWidth, bin+1);
            break;
        }
    }

     //Find rising edge in right half of histogram.
    for(size_t bin = k_binsCount - 1; bin > k_binsCount/2; bin--)
    {
        if(newDataHistogram[bin] - newDataHistogram[bin-1] > k_histogramFallingEdgeMinDelta)
        {
            //Value of right popular bin
            m_whiteValue = GetBinValue(newDataMinValue, binWidth, bin);
            break;
        }
    }
    Serial.printf("m_syncTreshold = %d\n", m_syncTreshold);
    Serial.printf("m_syncValue = %d\n", m_syncValue);
    Serial.printf("m_whiteValue = %d\n", m_whiteValue);

    return AmplitudeCaclulatorState::k_finished;
}