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
    
    m_flatnessAccumulator.Reset();
}

AmplitudeCaclulatorState AmplitudeCaclulator::PushSamples(const uint16_t *newData, size_t newDataLen)
{
    if(newDataLen)
    {
        const size_t samplesInHistogramBeforePush = m_amplitudeHistogram.GetSamplesCount();
        //Histogram<uint32_t, uint16_t, k_binsCount> newDataHistogram(0u, MAX_UINT_12BIT);        
        
        for (size_t i = 0; i < newDataLen; i++)
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

    if(m_amplitudeHistogram.GetSamplesCount() >= k_minSamplesForCalcFromHistogram)
    {
        //Change to k_readyForCalculation after test calculation, or just compare with k_minSamplesForCalculation ?
        m_state = AmplitudeCaclulatorState::k_readyForCalculation;
    }else{        
        m_state = AmplitudeCaclulatorState::k_needMoreSamples;
    }

    //SamplesRange should be valid now, so we can use it for relative(normalized) differential.
    if(m_amplitudeHistogram.GetSamplesCount() >= k_minSamplesForRangeCalculation)
    {
        const float samplesRange = static_cast<float>(m_amplitudeHistogram.m_sampleValuesRange.second - m_amplitudeHistogram.m_sampleValuesRange.first);
        static constexpr float k_sampleTimeUs = 1000000.0f / k_sampleRate;
        for(size_t i = 1; i < newDataLen; i++)
        {
            //Compare with previous sample
            const uint16_t diffAbsPerSample = std::abs(static_cast<int16_t>(newData[i]) - static_cast<int16_t>(newData[i-1]));
            //const float diffAbsPerUs = static_cast<float>(diffAbsPerSample) / k_sampleTimeUs;
            //It will be a problem to normalize to [0, 1] if sample is less than 1us.
            const float diffNormalized = static_cast<float>(diffAbsPerSample) / samplesRange;
            assert(diffNormalized <= 1.0f);

            static constexpr float k_logEpsilon = 0.001f; //To avoid log2(0) = -inf
            const float diffLog = -std::log2f(diffNormalized + k_logEpsilon);

            m_flatnessAccumulator.PushSample(newData[i], diffLog);
        }
    }
    //Do not check m_flatnessAccumulator.GetSamplesCount().
    //Even 0% SNR data is still valid data, if m_amplitudeHistogram is full.

    return m_state;
}

AmplitudeCaclulatorState AmplitudeCaclulator::Calculate()
{
    if(m_state != AmplitudeCaclulatorState::k_readyForCalculation)
    {
        //In wrong state.
        CVBS_ANALYZER_LOG_DEBUG("AmplitudeCaclulator::Calculate() called in unexpected state %d.\n", static_cast<int>(m_state));
        return m_state;
    }
    assert((m_amplitudeHistogram.m_sampleValuesRange.second - m_amplitudeHistogram.m_sampleValuesRange.first) >= k_minRange);
    assert(m_amplitudeHistogram.GetSamplesCount() >= k_minSamplesForCalcFromHistogram);

    m_state = AmplitudeCaclulatorState::k_calcFromFlatness;
    CalcSyncTresholdFromFlatness();
    if(IsInErrorState())
    {
        //Fallback to Histogram method.
        m_state = AmplitudeCaclulatorState::k_calcFromHistogram;
        CalcSyncTresholdFromHistogram();

        if(IsInErrorState())
        {
            //Fallback to hardcoded treshold.
            CalcSyncTresholdHardcoded();
        }
    }
    //CalculateWhiteLevel();
    //CalculateBlankingLevel();
    
    assert(m_state == AmplitudeCaclulatorState::k_finished);//We have fallbacks for everything
    return m_state;
}

void AmplitudeCaclulator::CalcSyncTresholdFromFlatness()
{
    if(m_flatnessAccumulator.GetSamplesCount() < k_minSamplesForCalcFromFlatness)
    {
        m_state = AmplitudeCaclulatorState::k_failedFlatnessNeedMoreSamples;
        return;
    }
    //Find edges
    
    //Find first first peak in histogram.
    //Data in m_flatnessAccumulator is already normalized to [0, 1] range, no need to scale deltas.
    const float k_flatnessSpikeRisingEdgeMinDelta = 0.1f;
    const float k_flatnessSpikeFallingEdgeMinDelta = 0.05f;
    const float k_nextEdgeMinDelta = 0.01f;

    std::array<FloatAccumulatorType::iterator, 3> edgeIterators;
    std::array<size_t, 3> edgeBins;
    edgeIterators.fill(m_flatnessAccumulator.end());

    edgeIterators[0] = m_flatnessAccumulator.FindRisingEdge(
        m_flatnessAccumulator.begin(), m_flatnessAccumulator.end(), k_flatnessSpikeRisingEdgeMinDelta);

    if(edgeIterators[0] != m_flatnessAccumulator.end())
    {
        edgeIterators[1] = m_flatnessAccumulator.FindFallingEdge(
            std::next(edgeIterators[0]), m_flatnessAccumulator.end(), k_flatnessSpikeFallingEdgeMinDelta);
    }

    if(edgeIterators[1] != m_flatnessAccumulator.end())
    {
        edgeIterators[2] = m_flatnessAccumulator.FindRisingEdge(
            std::next(edgeIterators[1]), m_flatnessAccumulator.end(), k_nextEdgeMinDelta);
    }

    if(edgeIterators[2] != m_flatnessAccumulator.end())
    {
        edgeBins[0] = std::distance(m_flatnessAccumulator.begin(), edgeIterators[0]);
        edgeBins[1] = std::distance(m_flatnessAccumulator.begin(), edgeIterators[1]);
        edgeBins[2] = std::distance(m_flatnessAccumulator.begin(), edgeIterators[2]);
        CVBS_ANALYZER_LOG_DEBUG("Sync bins in m_flatnessAccumulator: %d, %d, %d\n", edgeBins[0], edgeBins[1], edgeBins[2]);
        
        m_syncTreshold = ((float)m_flatnessAccumulator.GetBinCenter(edgeBins[1]) + (float)m_flatnessAccumulator.GetBinCenter(edgeBins[2]) ) / 2.0f;
        //m_syncTreshold = m_flatnessAccumulator.GetBinCenter(edgeBins[1]);
        m_state = AmplitudeCaclulatorState::k_finished;    
    }
    else
    {
        //Falling enge not found.
        //Ask for more samples, and after more failes, give up and set fallback value?
        m_state = AmplitudeCaclulatorState::k_failedFlatnessNoEdgesFound;
    }
}

void AmplitudeCaclulator::CalcSyncTresholdFromHistogram()
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
        
        CVBS_ANALYZER_LOG_DEBUG("Sync bins in m_amplitudeHistogram: %d, %d, %d\n", edgeBins[0], edgeBins[1], edgeBins[2]);
        
        m_syncTreshold = ((float)m_amplitudeHistogram.GetBinCenter(edgeBins[1]) + (float)m_amplitudeHistogram.GetBinCenter(edgeBins[2]) ) / 2.0f;
        //m_syncTreshold = m_amplitudeHistogram.GetBinCenter(edgeBins[1]);
        m_state = AmplitudeCaclulatorState::k_finished;    
    }
    else
    {
        //Falling enge not found.
        //Ask for more samples, and after more failes, give up and set fallback value?
        m_state = AmplitudeCaclulatorState::k_failedHistogramNoEdgesFound;
    }
}
void AmplitudeCaclulator::CalcSyncTresholdHardcoded()
{
    //Doing lerp
    m_syncTreshold = m_amplitudeHistogram.m_sampleValuesRange.first + 
            (m_amplitudeHistogram.m_sampleValuesRange.second - m_amplitudeHistogram.m_sampleValuesRange.first)
            *k_syncTresholdDefault; 

    CVBS_ANALYZER_LOG_DEBUG("AmplitudeCaclulator::CalcSyncTresholdHardcoded() called, using default treshold %d.\n", m_syncTreshold);
    m_state = AmplitudeCaclulatorState::k_finished;    
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
