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
        m_isInvertedVideo = 0.0f;
        return;
    }

    assert(k_notSyncIntervalGoodDeviationUs < k_notSyncIntervalGoodUs);    

    // 80-100% notSyncIntervals are 57-63us -> notSyncGoodSamplesScore=1
    const float notSyncGoodSamplesWeight = GetWeightOfIntervalsWithLength(
        syncIntervalsCalculator.m_notSyncSequenceLengthHistogram,
        k_notSyncIntervalGoodUs - k_notSyncIntervalGoodDeviationUs,
        k_notSyncIntervalGoodUs + k_notSyncIntervalGoodDeviationUs);

    const float notSyncGoodSamplesScore = std::min(1.0f, notSyncGoodSamplesWeight / 0.8f);

    const float notSyncGoodSamplesScoreWeight = 1.0f; // 100% of m_isVideo score consists of notSyncGoodSamplesScore
    m_isVideo = notSyncGoodSamplesScore * notSyncGoodSamplesScoreWeight;
}

float VideoScore::GetWeightOfIntervalsWithLength(const SyncIntervalsCalculator::HistogramType &histogram,
     const size_t minIntervalLengthUs, const size_t maxIntervalLengthUs) const
{
    const size_t samplesMin = UsToSamples(k_notSyncIntervalGoodUs - k_notSyncIntervalGoodDeviationUs);
    const size_t samplesMax = UsToSamples(k_notSyncIntervalGoodUs + k_notSyncIntervalGoodDeviationUs);

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
