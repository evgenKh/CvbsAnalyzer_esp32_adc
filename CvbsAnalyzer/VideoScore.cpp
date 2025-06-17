#include "VideoScore.h"
#include "SyncIntervalsCalculator.h"
#include "AmplitudeCaclulator.h"
#include "CvbsAnalyzerGlobals.h"

void VideoScore::CalculateFromSyncIntervals(const SyncIntervalsCalculator &syncIntervalsCalculator,
                                            const AmplitudeCaclulatorState amplitudeCalculatorState,
                                            //const uint32_t sampleRate,
                                            const bool dataWasInverted)
{
    // Early exit when states not perfect.
    if (syncIntervalsCalculator.GetState() != SyncIntervalsCalculatorState::k_finished ||
        amplitudeCalculatorState != AmplitudeCaclulatorState::k_finished)
    {
        m_isVideo = 0.0f;
        return;
    }
  

    // 80-100% notSyncIntervals are 57-63us 
    const float notSync60usGoodSamplesWeight = GetWeightOfIntervalsWithLength(
        syncIntervalsCalculator.m_notSyncSequenceLengthHistogram,
        57, 63);        
    const float notSync60usGoodSamplesScore = notSync60usGoodSamplesWeight / 0.8f;

    // 80-100% notSyncIntervals are 5-7us
    const float notSync5usGoodSamplesWeight = GetWeightOfIntervalsWithLength(
        syncIntervalsCalculator.m_notSyncSequenceLengthHistogram,
        5, 7);        
    const float notSync5usGoodSamplesScore = notSync5usGoodSamplesWeight / 0.2f;

    // 80-100% notSyncIntervals are 29-33us
    const float notSync30usGoodSamplesWeight = GetWeightOfIntervalsWithLength(
        syncIntervalsCalculator.m_notSyncSequenceLengthHistogram,
        29, 33);        
    const float notSync30usGoodSamplesScore = notSync30usGoodSamplesWeight / 0.2f;

    // 70-100% syncIntervals are 5-7us -> notSyncGoodSamplesScore=1
    const float sync5usGoodSamplesWeight = GetWeightOfIntervalsWithLength(
        syncIntervalsCalculator.m_syncSequenceLengthHistogram,
        5, 7);        
    const float sync5usGoodSamplesScore = sync5usGoodSamplesWeight / 0.6f;

    m_isVideo = 0.0f;
    m_isVideo += std::max(-1.0f,std::min(1.0f, notSync60usGoodSamplesScore)) * 0.7f;//70% decision weight from 60us not-sync sequences
    m_isVideo += std::max(-1.0f,std::min(1.0f, sync5usGoodSamplesScore)) * 0.2f;//20% decision weight from 5us sync sequences
    m_isVideo += std::max(-1.0f,std::min(1.0f, notSync5usGoodSamplesScore)) * 0.05f;//5% decision weight from 5us sync sequences
    m_isVideo += std::max(-1.0f,std::min(1.0f, notSync30usGoodSamplesScore)) * 0.05f;//5% decision weight from 5us sync sequences
}

float VideoScore::GetWeightOfIntervalsWithLength(const SyncIntervalsCalculator::HistogramType &histogram,
     const size_t minIntervalLengthUs, const size_t maxIntervalLengthUs) const
{
    const size_t samplesMin = UsToSamples(minIntervalLengthUs);
    const size_t samplesMax = UsToSamples(maxIntervalLengthUs);

    assert(samplesMin >= histogram.m_binsRange.first);
    assert(samplesMax <= histogram.m_binsRange.second);

    const size_t binIndexMin = histogram.GetBinIndexForValue(samplesMin);
    const size_t binIndexMax = histogram.GetBinIndexForValue(samplesMax);

    size_t samplesCountAccumulator = 0;
    for (int bin = binIndexMin; bin <= binIndexMax; bin++)
    {
        samplesCountAccumulator += histogram[bin];
    }

    const float weight = (float)samplesCountAccumulator / (float)histogram.GetSamplesCount();
    return weight;
}
