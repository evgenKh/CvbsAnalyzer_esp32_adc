#include "VideoScore.h"
#include "SyncIntervalsCalculator.h"
#include "AmplitudeCaclulator.h"
#include "CvbsAnalyzerGlobals.h"

void VideoScore::CalculateFromSyncIntervalsSimple(const SyncIntervalsCalculator &syncIntervalsCalculator,
                                            const AmplitudeCaclulatorState amplitudeCalculatorState)
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


void VideoScore::CalculateFromSyncIntervals(const SyncIntervalsCalculator &syncIntervalsCalculator,
                                            const AmplitudeCaclulatorState amplitudeCalculatorState)
{
    // Early exit when states not perfect.
    if (syncIntervalsCalculator.GetState() != SyncIntervalsCalculatorState::k_finished ||
        amplitudeCalculatorState != AmplitudeCaclulatorState::k_finished)
    {
        m_isVideo = 0.0f;
        return;
    }
    
    // Range of interest on sync histogram - 

    const size_t syncHistBeginBin = 1;    //skip bin [0]
    const size_t syncHistEndBin = std::min( 
        syncIntervalsCalculator.m_syncSequenceLengthHistogram.GetBinIndexForValue(
        UsToSamplesContexpr(10)),
        syncIntervalsCalculator.k_binsCount - 2// skip last bin
    );
    const float syncHistMeanSamples = syncIntervalsCalculator.m_syncSequenceLengthHistogram.CalculateMean(
                                            syncHistBeginBin,
                                            syncHistEndBin
                                         );
    const float syncHistStDeviation = syncIntervalsCalculator.m_syncSequenceLengthHistogram.CalculateStdDeviation(
                                            syncHistBeginBin,
                                            syncHistEndBin,
                                            syncHistMeanSamples
                                         );

    //Break not-sync hyst to ranges

   
    const size_t notSyncHistBeginBin = syncIntervalsCalculator.m_notSyncSequenceLengthHistogram.GetBinIndexForValue(
        UsToSamplesContexpr(40));
    const size_t notSyncHistEndBin = std::min( 
        syncIntervalsCalculator.m_notSyncSequenceLengthHistogram.GetBinIndexForValue(
        UsToSamplesContexpr(80)),
        syncIntervalsCalculator.k_binsCount - 2// skip last bin
    );
    const float notSyncHistMeanSamples = syncIntervalsCalculator.m_notSyncSequenceLengthHistogram.CalculateMean(
                                            notSyncHistBeginBin,
                                            notSyncHistEndBin
                                         );
    const float notSyncHistStDeviation = syncIntervalsCalculator.m_notSyncSequenceLengthHistogram.CalculateStdDeviation(
                                            notSyncHistBeginBin,
                                            notSyncHistEndBin,
                                            notSyncHistMeanSamples
                                         );
    

    CVBS_ANALYZER_LOG_INFO("syncHistMeanSamples=%f syncHistStDeviation=%f notSyncHistMeanSamples=%f notSyncHistStDeviation=%f\n",
                            syncHistMeanSamples, syncHistStDeviation,
                            notSyncHistMeanSamples, notSyncHistStDeviation);

    //Probably doing double work, first calculating mean+dev, then interpolating them, and get rid of one of steps...

    //https://www.wolframalpha.com/input?i=y%3D+max%280%2C+1+-+%28%28x-4.2%29%2F3%29%5E4%29
    const float syncMeanScore = std::max(0.0f, 1.0f - powf((syncHistMeanSamples - UsToSamplesf(4.2f))/3.0f, 4.0f));

    //https://www.wolframalpha.com/input?i=y%3D+max%280%2C+1+-+%28%28x-0%29%2F13%29%5E4%29
    const float syncStDevScore = std::max(0.0f, 1.0f - powf((syncHistStDeviation - 0.0f)/13.0f, 4.0f));

    
    //https://www.wolframalpha.com/input?i=y%3D+max%280%2C+1+-+%28%28x-60%29%2F4%29%5E4%29
    const float notSyncMeanScore = std::max(0.0f, 1.0f - powf((notSyncHistMeanSamples - UsToSamplesf(60.0f))/4.0f, 4.0f));

    //https://www.wolframalpha.com/input?i=y%3D+max%280%2C+1+-+%28%28x-0%29%2F20%29%5E4%29
    const float notSyncStDevScore = std::max(0.0f, 1.0f - powf((notSyncHistStDeviation - 0.0f)/20.0f, 4.0f));

    const float totalScore = ((syncMeanScore * syncStDevScore)* 0.6f + (notSyncMeanScore * notSyncStDevScore)) /
                                             (0.6f + 1.0f);
    m_isVideo = totalScore;
}