#include "CvbsAnalyzer.h"
#include <math.h>

#if CVBS_ANALYZER_PROFILER
#   define TO_STR(x) #x
void CvbsAnalyzerProfiler::Start()
{
    if(!m_started)
    {
        m_started = true;            
        m_startTime = esp_timer_get_time();
    }
}
void CvbsAnalyzerProfiler::Stop()
{
    if(m_started)
    {
        m_microsecondsAccumulator += (esp_timer_get_time() - m_startTime);
        m_started = false;
    }
}
#endif // CVBS_ANALYZER_PROFILER

void CvbsAnalyzer::Reset()
{
    m_samplesPreFilter.Reset();
    m_amplitudeCaclulator.Reset();
    m_syncIntervalsCalculator.Reset();
    m_rssiAverageFilter.Reset();
    m_videoScore.Reset();
    m_pinAverage = 0;
    m_rawSamplesReadTotal = 0;
    m_invertDataCurrentValue = false;

#if CVBS_ANALYZER_PROFILER
    m_stateProfilers = {
        { CvbsAnalyzerState::k_notInitialized, CvbsAnalyzerProfiler( TO_STR(k_notInitialized) ) },
        { CvbsAnalyzerState::k_initializing, CvbsAnalyzerProfiler( TO_STR(k_initializing) ) },
        { CvbsAnalyzerState::k_initializedAndIdle, CvbsAnalyzerProfiler( TO_STR(k_initializedAndIdle) ) },
        { CvbsAnalyzerState::k_samplingAndPreFiltering, CvbsAnalyzerProfiler( TO_STR(k_samplingAndPreFiltering) ) },
        //{ CvbsAnalyzerState::k_amplitudeSampling, CvbsAnalyzerProfiler( TO_STR(k_amplitudeSampling) ) },
        { CvbsAnalyzerState::k_averageCalculation, CvbsAnalyzerProfiler( TO_STR(k_averageCalculation) ) },
        { CvbsAnalyzerState::k_amplitudeCalculation, CvbsAnalyzerProfiler( TO_STR(k_amplitudeCalculation) ) },
        //{ CvbsAnalyzerState::k_syncIntervalsSampling, CvbsAnalyzerProfiler( TO_STR(k_syncIntervalsSampling) ) },
        { CvbsAnalyzerState::k_syncIntervalsCalculation, CvbsAnalyzerProfiler( TO_STR(k_syncIntervalsCalculation) ) },
        { CvbsAnalyzerState::k_videoScoreCalculation, CvbsAnalyzerProfiler( TO_STR(k_videoScoreCalculation) ) },
        { CvbsAnalyzerState::k_restartInverted, CvbsAnalyzerProfiler( TO_STR(k_restartInverted) ) },
        { CvbsAnalyzerState::k_stopADC, CvbsAnalyzerProfiler( TO_STR(k_stopADC) ) },
        { CvbsAnalyzerState::k_finished, CvbsAnalyzerProfiler( TO_STR(k_finished) ) },
        { CvbsAnalyzerState::k_failedBadState, CvbsAnalyzerProfiler( TO_STR(k_failedBadState) ) },
        { CvbsAnalyzerState::k_failedFastADCInitialization, CvbsAnalyzerProfiler( TO_STR(k_failedFastADCInitialization) ) },
        { CvbsAnalyzerState::k_failedSampling, CvbsAnalyzerProfiler( TO_STR(k_failedSampling) ) },
        { CvbsAnalyzerState::k_failedAmplitude, CvbsAnalyzerProfiler( TO_STR(k_failedAmplitude) ) },
        { CvbsAnalyzerState::k_failedSyncIntervals, CvbsAnalyzerProfiler( TO_STR(k_failedSyncIntervals) ) },
        { CvbsAnalyzerState::k_failedVideoScore, CvbsAnalyzerProfiler( TO_STR(k_failedVideoScore) ) },
        { CvbsAnalyzerState::k_failedFastADCStop, CvbsAnalyzerProfiler( TO_STR(k_failedFastADCStop) ) },
        { CvbsAnalyzerState::k_failedUnknownError, CvbsAnalyzerProfiler( TO_STR(k_failedUnknownError) ) },
        { CvbsAnalyzerState::k_totalAnalyzeTime, CvbsAnalyzerProfiler( TO_STR(k_totalAnalyzeTime) ) }
    };
    #endif // CVBS_ANALYZER_PROFILER
    
    //Or deinitialize FastADC?
    switch(m_fastAdc.GetState())
    {
        case FastADCState::k_notInitialized:
            SetState(CvbsAnalyzerState::k_notInitialized);
            break;
        case FastADCState::k_initializedAdcStopped:
            SetState(CvbsAnalyzerState::k_initializedAndIdle);
            break;
        default:
            SetState(CvbsAnalyzerState::k_failedBadFastADCState);
            CVBS_ANALYZER_LOG_INFO("FastADC is in unexpected state %d, cannot reset CvbsAnalyzer!\n", (int)m_fastAdc.GetState());
            break;
    }
}

CvbsAnalyzerState CvbsAnalyzer::InitializeFastADC()
{
    if(m_state != CvbsAnalyzerState::k_notInitialized)
    {
        return SetState(CvbsAnalyzerState::k_failedBadState);
    }

    const FastADCState fastAdcState = m_fastAdc.Initialize();
    if (fastAdcState != FastADCState::k_initializedAdcStopped)
    {
        return SetState(CvbsAnalyzerState::k_failedFastADCInitialization);
    }
    else
    {
        SetState(CvbsAnalyzerState::k_initializedAndIdle);
    }

    return m_state;        
}

CvbsAnalyzerState CvbsAnalyzer::DeinitializeFastADC()
{
    m_fastAdc.Deinitialize();
    SetState(CvbsAnalyzerState::k_notInitialized);
    return m_state;
}


CvbsAnalyzerState CvbsAnalyzer::ExecuteJob(const CvbsAnalyzerJob& job)
{
    if (m_state != CvbsAnalyzerState::k_initializedAndIdle && m_state != CvbsAnalyzerState::k_finished)
    {
        return SetState(CvbsAnalyzerState::k_failedBadState);
    }

    //Reset state vars and members
    m_samplesPreFilter.Reset();
    m_rssiAverageFilter.Reset();
    m_amplitudeCaclulator.Reset();
    m_syncIntervalsCalculator.Reset();
    m_videoScore.Reset();
    m_videoScoreFromInverted.Reset();
    m_rawSamplesReadTotal = 0;

    const bool invertByHardware = false;//Use for test purposes
    m_invertDataCurrentValue = false;//Will use software
    
    if(job.m_inversionType == CvbsAnalyzerInversionType::k_invertedOnly)
    {
        m_invertDataCurrentValue = true;    
    }

#if CVBS_ANALYZER_PROFILER
    for(auto& pair: m_stateProfilers)
    {
        pair.second.Stop();
        pair.second.m_microsecondsAccumulator = 0;
    }
#endif // CVBS_ANALYZER_PROFILER


    CVBS_ANALYZER_LOG("Starting ExecuteJob for gpioPin %d, invertByHardware=%d, invertBySoftware=%d...\n", 
                                    (int)job.m_gpioPin, (invertByHardware ? 1 : 0), (m_invertDataCurrentValue ? 1 : 0));
    
    
    //Prepare calculation states list

    std::vector<CvbsAnalyzerState> calculationStates;

    if(job.m_type == CvbsAnalyzerJobType::k_videoScore)
    {
        calculationStates = { CvbsAnalyzerState::k_amplitudeCalculation, 
                              CvbsAnalyzerState::k_syncIntervalsCalculation, 
                              CvbsAnalyzerState::k_videoScoreCalculation };
        if(job.m_inversionType == CvbsAnalyzerInversionType::k_nonInvertedThenInverted)
        {
            calculationStates.insert(calculationStates.end(), 
                { CvbsAnalyzerState::k_restartInverted,
                  CvbsAnalyzerState::k_amplitudeCalculation, 
                  CvbsAnalyzerState::k_syncIntervalsCalculation, 
                  CvbsAnalyzerState::k_videoScoreCalculation }
            );
        }
    }
    else if(job.m_type == CvbsAnalyzerJobType::k_averageRssi)
    {
        calculationStates = { CvbsAnalyzerState::k_averageCalculation };
    }
    
    size_t minSamplesEnoughForCalculationStates = 0;
    for(CvbsAnalyzerState calcState : calculationStates)
    {
        switch(calcState)
        {
            case CvbsAnalyzerState::k_amplitudeCalculation:
                minSamplesEnoughForCalculationStates = std::max(m_amplitudeCaclulator.GetMinSamplesForCalculation(),
                                                                 minSamplesEnoughForCalculationStates);
                break;
            case CvbsAnalyzerState::k_syncIntervalsCalculation:
                minSamplesEnoughForCalculationStates = std::max(m_syncIntervalsCalculator.GetMinSamplesForCalculation(),
                                                                 minSamplesEnoughForCalculationStates);
                break;
            case CvbsAnalyzerState::k_averageCalculation:
                minSamplesEnoughForCalculationStates = std::max(m_rssiAverageFilter.GetMinSamplesForCalculation(),
                                                                 minSamplesEnoughForCalculationStates);
                break;
        }
    }

    //Samples that cacl.states want, won't fint into pre-filter buffer!
    assert(minSamplesEnoughForCalculationStates <= m_samplesPreFilter.k_preFilteredSamplesBufLenSamples);

    #if CVBS_ANALYZER_PROFILER
    m_stateProfilers[CvbsAnalyzerState::k_totalAnalyzeTime].Start();
#endif // CVBS_ANALYZER_PROFILER    

    FastADCState fastAdcState = m_fastAdc.StartADCSampling(job.m_gpioPin, invertByHardware);
    if (fastAdcState != FastADCState::k_adcStarted)
    {
        CVBS_ANALYZER_LOG("FastADC::StartADCSampling() for gpioPin %d failed with state %d!\n", (int)job.m_gpioPin, (int)fastAdcState);
        return SetState(CvbsAnalyzerState::k_failedSampling);
    }

    SetState(CvbsAnalyzerState::k_samplingAndPreFiltering);

    size_t newRawSamplesRead = 0;
    int run;
    for (run = 0; run < k_maxDmaReadsPerAnalyzePin; run++)
    {
        size_t bytesRead = m_fastAdc.ReadSamplesBlockingTo(m_rawAdcSamplesBuf, sizeof(m_rawAdcSamplesBuf));
        newRawSamplesRead = bytesRead / sizeof(uint16_t);
        m_rawSamplesReadTotal += newRawSamplesRead;

        if (newRawSamplesRead < 1)
        {            
            CVBS_ANALYZER_LOG("Little amount of samples read from FastADC on run %d, samplesRead=%d!\n",
                 run, job.m_gpioPin, newRawSamplesRead);
            continue;
        }

        const size_t newPreFilteredSamples = m_samplesPreFilter.PushSamples(m_rawAdcSamplesBuf,
                                                newRawSamplesRead,
                                                k_skipLeadingZeroSamples,
                                                m_invertDataCurrentValue);

        //const uint16_t* newPreFilteredSamplesStart = m_samplesPreFilter.m_samplesBuf 
        //                                            + m_samplesPreFilter.m_samplesCount 
        //                                            - newPreFilteredSamples;

        if(m_samplesPreFilter.m_samplesCount >= minSamplesEnoughForCalculationStates)
        {
            break; //We have enough samples for calculations
        }
    }

    if(m_samplesPreFilter.m_samplesCount < minSamplesEnoughForCalculationStates)
    {
        CVBS_ANALYZER_LOG("Not enough pre-filtered samples collected from FastADC!, expceted%d, has %d.\n", 
                          minSamplesEnoughForCalculationStates, m_samplesPreFilter.m_samplesCount);
        //SetState(CvbsAnalyzerState::
        //This is not actually an error. Probably 0volts on pin.
        //TODO: what state to go to?
    }

    SetState(CvbsAnalyzerState::k_stopADC);
    fastAdcState = m_fastAdc.StopADCSampling();

    if (fastAdcState == FastADCState::k_initializedAdcStopped)
    {
        CVBS_ANALYZER_LOG("Stopped ADC sampling for gpioPin %d, m_samplesPreFilter.m_samplesCount = %d,\
                            m_rawSamplesReadTotal = %d, CvbsAnalyzerState = %d\n", 
                      job.m_gpioPin, m_samplesPreFilter.m_samplesCount, m_rawSamplesReadTotal, (int)m_state);
        //SetState(CvbsAnalyzerState::k_finished);
    }
    else
    {
        CVBS_ANALYZER_LOG("FastADC::StopADCSampling() for gpioPin %d failed with state %d!", job.m_gpioPin, (int)fastAdcState);
        SetState(CvbsAnalyzerState::k_failedFastADCStop);
    }

    //Now proceed with calculations

    for(auto calcStateIt = calculationStates.begin(); calcStateIt != calculationStates.end(); calcStateIt++)
    {
        if(IsInErrorState()){
            break;
        }
        SetState(*calcStateIt);

        if(m_state == CvbsAnalyzerState::k_amplitudeCalculation)
        {
            assert(m_amplitudeCaclulator.GetState() == AmplitudeCaclulatorState::k_needMoreSamples);
            m_amplitudeCaclulator.PushSamples(m_samplesPreFilter.m_samplesBuf, m_samplesPreFilter.m_samplesCount);
            
            if(m_amplitudeCaclulator.GetState() == AmplitudeCaclulatorState::k_readyForCalculation)
            {
                m_amplitudeCaclulator.Calculate();
                if(m_amplitudeCaclulator.GetState() == AmplitudeCaclulatorState::k_finished)
                {
                    continue; //Go to next state
                }
                else 
                {                    
                    CVBS_ANALYZER_LOG("AmplitudeCaclulator in error state %d\n", (int)m_amplitudeCaclulator.GetState());
                    SetState(CvbsAnalyzerState::k_failedAmplitude);
                }
            }
            else
            {
                SetState(CvbsAnalyzerState::k_failedAmplitude);
            }
        }
        else if(m_state == CvbsAnalyzerState::k_syncIntervalsCalculation)
        {
            assert(m_syncIntervalsCalculator.GetState() == SyncIntervalsCalculatorState::k_needMoreSamples);
            m_syncIntervalsCalculator.PushSamples(m_samplesPreFilter.m_samplesBuf, 
                                                  m_samplesPreFilter.m_samplesCount,
                                                  m_amplitudeCaclulator.m_syncTreshold);

            if(m_syncIntervalsCalculator.GetState() == SyncIntervalsCalculatorState::k_finished)
            {
                continue; //Go to next state
            }
            else
            {
                CVBS_ANALYZER_LOG("SyncIntervalsCalculator in error state %d\n", (int)m_syncIntervalsCalculator.GetState()); 
                SetState(CvbsAnalyzerState::k_failedSyncIntervals);
            }
        }
        else if(m_state == CvbsAnalyzerState::k_averageCalculation)
        {
            assert(m_rssiAverageFilter.GetState() == AverageFilterState::k_needMoreSamples);

            m_rssiAverageFilter.PushSamples(m_samplesPreFilter.m_samplesBuf, m_samplesPreFilter.m_samplesCount);
            if(m_rssiAverageFilter.GetState() == AverageFilterState::k_finished)
            {
                m_pinAverage = m_rssiAverageFilter.GetAverage();
                continue; //Go to next state
            }
            else
            {
                CVBS_ANALYZER_LOG("AverageFilter in error state %d\n", (int)m_rssiAverageFilter.GetState()); 
                SetState(CvbsAnalyzerState::k_failedAverageFilter);
            }
        }
        else if(m_state == CvbsAnalyzerState::k_videoScoreCalculation)
        {
            VideoScore& videoScore = (m_invertDataCurrentValue ? m_videoScoreFromInverted : m_videoScore);

            videoScore.Reset();
            videoScore.CalculateFromSyncIntervals(m_syncIntervalsCalculator,
                                                    m_amplitudeCaclulator.GetState(),
                                                    m_invertDataCurrentValue);
        }
        else if(m_state == CvbsAnalyzerState::k_restartInverted)
        {
            assert(!m_invertDataCurrentValue);
            m_invertDataCurrentValue = true;

            m_samplesPreFilter.InvertPrefilteredSamples();
            m_amplitudeCaclulator.Reset();
            m_syncIntervalsCalculator.Reset();
        }
        assert(m_state != CvbsAnalyzerState::k_stopADC &&
               m_state != CvbsAnalyzerState::k_finished &&
               m_state != CvbsAnalyzerState::k_stopADC);
    }//for   

#if CVBS_ANALYZER_PROFILER
    m_stateProfilers[CvbsAnalyzerState::k_totalAnalyzeTime].Stop();
#endif // CVBS_ANALYZER_PROFILER        

    //PrintJson();
    if(k_printCsvLearningData)
    {
        PrintCsv();
    }
    
    if(IsInErrorState())
    {
        CVBS_ANALYZER_LOG("Finishing CvbsAnalyzer::ExecuteJob() with error state %d!\n", (int)m_state);
    }

    SetState(CvbsAnalyzerState::k_finished);//Setting finished despite of error state.
    return m_state;
}

void CvbsAnalyzer::PrintJson()
{
    CVBS_ANALYZER_LOG("{\n");
    CVBS_ANALYZER_LOG("\"CvbsAnalyzer\": {\n");
    CVBS_ANALYZER_LOG("\t\"m_invertDataCurrentValue\": %d,\n", m_invertDataCurrentValue ? 1 : 0);
    CVBS_ANALYZER_LOG("\t\"m_state\": %d,\n", (int)m_state);
    CVBS_ANALYZER_LOG("\t\"m_rawSamplesReadTotal\": %d,\n", m_rawSamplesReadTotal);
    CVBS_ANALYZER_LOG("\t\"k_sampleRate\": %d,\n", k_sampleRate);
    CVBS_ANALYZER_LOG("\t\"k_sampleRateWithSkippedOversamples\": %d,\n", k_sampleRateWithSkippedOversamples);
    CVBS_ANALYZER_LOG("\t\"k_adcDataStrideSamples\": %d,\n", k_adcDataStrideSamples);
    CVBS_ANALYZER_LOG("\t\"k_i2sSampleRate\": %d,\n", k_i2sSampleRate);
    CVBS_ANALYZER_LOG("\t\"k_oversamplingMultiplier\": %d,\n", k_oversamplingMultiplier);
    CVBS_ANALYZER_LOG("\t\"k_dmaBufLenSamples\": %d,\n", k_dmaBufLenSamples);
    CVBS_ANALYZER_LOG("\t\"k_dmaBufsCount\": %d,\n", k_dmaBufsCount);
    CVBS_ANALYZER_LOG("\t},\n");
#if CVBS_ANALYZER_PROFILER
    CVBS_ANALYZER_LOG("\"m_stateProfilers\": {\n");
    for(auto& pair: m_stateProfilers)
    {
        pair.second.Stop();
        if(pair.second.m_microsecondsAccumulator)
            CVBS_ANALYZER_LOG("\t\"%s\": %lld,\n", pair.second.m_name, pair.second.m_microsecondsAccumulator);
    }
    CVBS_ANALYZER_LOG("},\n");
#endif // CVBS_ANALYZER_PROFILER
    CVBS_ANALYZER_LOG("\"FastADC\": { \"state\": %d, \"m_adcChannel\": --- },\n", (int)m_fastAdc.GetState());
    m_amplitudeCaclulator.Print();
    m_syncIntervalsCalculator.Print();
    CVBS_ANALYZER_LOG("}\n");
}

void CvbsAnalyzer::PrintCsv()
{
    static bool headerPrinted = false;
    if(!headerPrinted)
    {
        CVBS_ANALYZER_LOG_INFO("_Comment,_IsVideoLearning,\
            m_invertDataCurrentValue,\
            CvbsAnalyzerState,\
            m_videoScore.m_isVideo,\
            m_videoScore.m_isInvertedVideo,\
            m_rawSamplesRead,\
            k_sampleRate,\
            m_syncTreshold,\
            m_syncSequenceLengthHistogram.m_binsRange.min,\
            m_syncSequenceLengthHistogram.m_binsRange.max,\
            m_syncSequenceLengthHistogram.k_binsCount,\
            m_syncSequenceLengthHistogram.m_samplesCount,\
            m_syncSequenceLengthHistogram.bins_weights,");
        for(int i=0; i<m_syncIntervalsCalculator.m_syncSequenceLengthHistogram.size();i++)
            CVBS_ANALYZER_LOG_INFO("S%d,", m_syncIntervalsCalculator.m_syncSequenceLengthHistogram.GetBinCenter(i));
        
        CVBS_ANALYZER_LOG_INFO("m_notSyncSequenceLengthHistogram.m_binsRange.min,\
            m_notSyncSequenceLengthHistogram.m_binsRange.max,\
            m_notSyncSequenceLengthHistogram.k_binsCount,\
            m_notSyncSequenceLengthHistogram.m_samplesCount,\
            m_notSyncSequenceLengthHistogram.bins_weights,");
        for(int i=0; i<m_syncIntervalsCalculator.m_notSyncSequenceLengthHistogram.size();i++)
            CVBS_ANALYZER_LOG_INFO("N%d,", m_syncIntervalsCalculator.m_notSyncSequenceLengthHistogram.GetBinCenter(i));
        
        for(int i = 0; i < m_amplitudeCaclulator.m_amplitudeHistogram.size();i++)
        {
            CVBS_ANALYZER_LOG_INFO("m_amplitudeHistogram.%d,", i);
        }
        
#if CVBS_ANALYZER_PROFILER
        for(auto& pair: m_stateProfilers)
        {
            CVBS_ANALYZER_LOG_INFO("m_stateProfilers.%s,", pair.second.m_name);
        }
#endif // CVBS_ANALYZER_PROFILER

        CVBS_ANALYZER_LOG_INFO("\n");
        headerPrinted = true;
    }

    CVBS_ANALYZER_LOG_INFO(",,%d,%d,%f,%f,%d,%d,%d,%d,%d,%d,%d,,",
        m_invertDataCurrentValue ? 1 : 0, 
        (int)m_state, 
        m_videoScore.m_isVideo,
        m_videoScore.m_isInvertedVideo,
        (int)m_rawSamplesReadTotal,
        (int)k_sampleRate, 
        (int)m_amplitudeCaclulator.m_syncTreshold,
        (int)m_syncIntervalsCalculator.m_syncSequenceLengthHistogram.m_binsRange.first,
        (int)m_syncIntervalsCalculator.m_syncSequenceLengthHistogram.m_binsRange.second,
        (int)m_syncIntervalsCalculator.m_syncSequenceLengthHistogram.size(),
        (int)m_syncIntervalsCalculator.m_syncSequenceLengthHistogram.GetSamplesCount());


    for(int i=0; i<m_syncIntervalsCalculator.m_syncSequenceLengthHistogram.size();i++)
    {
        const float samplesCountF = std::max((uint32_t)1u, m_syncIntervalsCalculator.m_syncSequenceLengthHistogram.GetSamplesCount());
        const float binWeight = m_syncIntervalsCalculator.m_syncSequenceLengthHistogram[i] / samplesCountF;
        CVBS_ANALYZER_LOG_INFO("%f,", binWeight);
        //CVBS_ANALYZER_LOG_INFO("%d,", (int)m_syncIntervalsCalculator.m_syncSequenceLengthHistogram[i]);
    }

    CVBS_ANALYZER_LOG_INFO("%d,%d,%d,%d,,",
        (int)m_syncIntervalsCalculator.m_notSyncSequenceLengthHistogram.m_binsRange.first,
        (int)m_syncIntervalsCalculator.m_notSyncSequenceLengthHistogram.m_binsRange.second,
        (int)m_syncIntervalsCalculator.m_notSyncSequenceLengthHistogram.size(),
        (int)m_syncIntervalsCalculator.m_notSyncSequenceLengthHistogram.GetSamplesCount());

    for(int i=0; i<m_syncIntervalsCalculator.m_notSyncSequenceLengthHistogram.size();i++)
    {
        const float samplesCountF = std::max((uint32_t)1u, m_syncIntervalsCalculator.m_notSyncSequenceLengthHistogram.GetSamplesCount());
        const float binWeight = m_syncIntervalsCalculator.m_notSyncSequenceLengthHistogram[i] / samplesCountF;
        CVBS_ANALYZER_LOG_INFO("%f,", binWeight);
        //CVBS_ANALYZER_LOG_INFO("%d,", (int)m_syncIntervalsCalculator.m_notSyncSequenceLengthHistogram[i]);
    }

    const float amplitudeHistogramDivider = std::max((uint32_t)1u, m_amplitudeCaclulator.m_amplitudeHistogram.GetSamplesCount());
    for(int i = 0; i < m_amplitudeCaclulator.m_amplitudeHistogram.size();i++)
    {
        CVBS_ANALYZER_LOG_INFO("%f,", (float)m_amplitudeCaclulator.m_amplitudeHistogram[i]/amplitudeHistogramDivider);
    }
    
#if CVBS_ANALYZER_PROFILER
    for(auto& pair: m_stateProfilers)
    {
        CVBS_ANALYZER_LOG_INFO("%lld,", pair.second.m_microsecondsAccumulator);
    }
#endif // CVBS_ANALYZER_PROFILER
    
    CVBS_ANALYZER_LOG_INFO("\n");
}