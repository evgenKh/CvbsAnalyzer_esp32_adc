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
    m_rawSamplesRead = 0;
    m_invertDataCurrentValue = false;

#if CVBS_ANALYZER_PROFILER
    m_stateProfilers = {
        { CvbsAnalyzerState::k_notInitialized, CvbsAnalyzerProfiler( TO_STR(k_notInitialized) ) },
        { CvbsAnalyzerState::k_initializing, CvbsAnalyzerProfiler( TO_STR(k_initializing) ) },
        { CvbsAnalyzerState::k_initializedAndIdle, CvbsAnalyzerProfiler( TO_STR(k_initializedAndIdle) ) },
        { CvbsAnalyzerState::k_amplitudeSampling, CvbsAnalyzerProfiler( TO_STR(k_amplitudeSampling) ) },
        { CvbsAnalyzerState::k_amplitudeCalculation, CvbsAnalyzerProfiler( TO_STR(k_amplitudeCalculation) ) },
        { CvbsAnalyzerState::k_syncIntervalsSampling, CvbsAnalyzerProfiler( TO_STR(k_syncIntervalsSampling) ) },
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
    m_rawSamplesRead = 0;

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
                                    (int)gpioPin, (invertByHardware ? 1 : 0), (m_invertDataCurrentValue ? 1 : 0));
    
#if CVBS_ANALYZER_PROFILER
    m_stateProfilers[CvbsAnalyzerState::k_totalAnalyzeTime].Start();
#endif // CVBS_ANALYZER_PROFILER    

    FastADCState fastAdcState = m_fastAdc.StartADCSampling(job.m_gpioPin, invertByHardware);
    if (fastAdcState != FastADCState::k_adcStarted)
    {
        CVBS_ANALYZER_LOG("FastADC::StartADCSampling() for gpioPin %d failed with state %d!\n", (int)job.m_gpioPin, (int)fastAdcState);
        return SetState(CvbsAnalyzerState::k_failedSampling);
    }

    SetState(CvbsAnalyzerState::k_amplitudeSampling);

    size_t samplesRead = 0;    
    
    int run;
    for (run = 0; run < k_maxDmaReadsPerAnalyzePin; run++)
    {
        size_t bytesRead = m_fastAdc.ReadSamplesBlockingTo(m_rawAdcSamplesBuf, sizeof(m_rawAdcSamplesBuf));
        samplesRead = bytesRead / sizeof(uint16_t);
        m_rawSamplesRead += samplesRead;

        if (samplesRead < 1)
        {            
            CVBS_ANALYZER_LOG("Little amount of samples read from FastADC on run %d, samplesRead=%d!\n", run, (int)gpioPin, samplesRead);
            continue;
        }

        if(k_printRawAdcData)
        {
            CVBS_ANALYZER_LOG_INFO("Printing %d samples from run #%d ----\n", samplesRead, run);
            for (int i = 0; i < samplesRead; i+=k_adcDataStrideSamples)
            {
                CVBS_ANALYZER_LOG_INFO("%d,", m_rawAdcSamplesBuf[i]);
            }
            CVBS_ANALYZER_LOG_INFO("%d samples printed.(run %d) ----\n", samplesRead);
        }

        const size_t newPreFilteredSamples = m_samplesPreFilter.PushSamples(m_rawAdcSamplesBuf,
                                                samplesRead,
                                                k_skipLeadingZeroSamples,
                                                m_invertDataCurrentValue);

        const uint16_t* newPreFilteredSamplesStart = m_samplesPreFilter.m_samplesBuf 
                                                    + m_samplesPreFilter.m_samplesCount 
                                                    - newPreFilteredSamples;

        //handling non-error states
        if(m_state == CvbsAnalyzerState::k_amplitudeSampling)
        {
            if (m_amplitudeCaclulator.GetState() == AmplitudeCaclulatorState::k_needMoreSamples)
            {
                m_amplitudeCaclulator.PushSamples(newPreFilteredSamplesStart, newPreFilteredSamples);
            }
            // no else if
            if (m_amplitudeCaclulator.GetState() == AmplitudeCaclulatorState::k_readyForCalculation)
            {
                SetState(CvbsAnalyzerState::k_amplitudeCalculation);                
            }
            if(m_amplitudeCaclulator.IsInErrorState())
            {
                CVBS_ANALYZER_LOG("AmplitudeCaclulator in error state %d\n", (int)m_amplitudeCaclulator.GetState());
                SetState(CvbsAnalyzerState::k_failedAmplitude);
            }
        } // CvbsAnalyzerState::k_amplitudeSampling

        if(m_state == CvbsAnalyzerState::k_amplitudeCalculation)
        {
            assert(m_amplitudeCaclulator.GetState() == AmplitudeCaclulatorState::k_readyForCalculation);
            m_amplitudeCaclulator.Calculate();
            if(m_amplitudeCaclulator.GetState() == AmplitudeCaclulatorState::k_finished)
            {
                SetState(CvbsAnalyzerState::k_syncIntervalsSampling);
            }
            if(m_amplitudeCaclulator.IsInErrorState())
            {
                CVBS_ANALYZER_LOG("AmplitudeCaclulator in error state %d\n", (int)m_amplitudeCaclulator.GetState());
                SetState(CvbsAnalyzerState::k_failedAmplitude);
            }
        } // CvbsAnalyzerState::k_amplitudeCalculation

        if(m_state == CvbsAnalyzerState::k_syncIntervalsSampling)
        {
            assert(m_amplitudeCaclulator.GetState() == AmplitudeCaclulatorState::k_finished);

            if (m_syncIntervalsCalculator.GetState() == SyncIntervalsCalculatorState::k_needMoreSamples 
                || m_syncIntervalsCalculator.GetState() == SyncIntervalsCalculatorState::k_finished)
            {
                m_syncIntervalsCalculator.PushSamples(newPreFilteredSamplesStart,
                                                        newPreFilteredSamples,
                                                         m_amplitudeCaclulator.m_syncTreshold);
            }

            if(m_syncIntervalsCalculator.GetState() == SyncIntervalsCalculatorState::k_finished )
            {
                //Actually calculation done by this moment, but using this state to get back to sampling if needed
                SetState(CvbsAnalyzerState::k_syncIntervalsCalculation);
            }
            if(m_syncIntervalsCalculator.IsInErrorState())
            {
                SetState(CvbsAnalyzerState::k_failedSyncIntervals);
                CVBS_ANALYZER_LOG("SyncIntervalsCalculator in error state %d\n", (int)m_syncIntervalsCalculator.GetState());
            }            
        } // CvbsAnalyzerState::k_syncIntervalsSampling

        if(m_state == CvbsAnalyzerState::k_syncIntervalsCalculation)
        {
            //Maybe some fails are ok?
            assert(m_syncIntervalsCalculator.GetState() == SyncIntervalsCalculatorState::k_finished);
            if(k_syncIntervalsCalculatorConsumeMaxDmaReads && run < k_maxDmaReadsPerAnalyzePin-1)
            {
                //Actually calculation done by this moment, but we can afford more sampling for precision
                SetState(CvbsAnalyzerState::k_syncIntervalsSampling);
                continue;
            }
            SetState(CvbsAnalyzerState::k_videoScoreCalculation);
        } // CvbsAnalyzerState::k_syncIntervalsCalculation

        //All other states to be processed outside of loop
        if(m_state == CvbsAnalyzerState::k_videoScoreCalculation || IsInErrorState()){
                break;
        }
        assert(m_state != CvbsAnalyzerState::k_stopADC &&
               m_state != CvbsAnalyzerState::k_finished &&
               m_state != CvbsAnalyzerState::k_restartInverted &&
               m_state != CvbsAnalyzerState::k_stopADC);
    } // for

    if(!m_rawSamplesRead)
    {
        SetState(CvbsAnalyzerState::k_failedSampling);
    }
    if(m_syncIntervalsCalculator.GetState() != SyncIntervalsCalculatorState::k_finished)
    {
        CVBS_ANALYZER_LOG("SyncIntervalsCalculator in state %d, not finished! m_samplesProcessed=%d, run=%d\n",
             (int)m_syncIntervalsCalculator.GetState(),
            m_syncIntervalsCalculator.GetSamplesProcessed(),
        run);
        SetState(CvbsAnalyzerState::k_failedSyncIntervals);
    }

    if(m_state == CvbsAnalyzerState::k_videoScoreCalculation || IsInErrorState())
    {
        //We still can score the failure as valid no-video result.
        m_videoScore.Reset();
        m_videoScore.CalculateFromSyncIntervals(m_syncIntervalsCalculator,
                                                m_amplitudeCaclulator.GetState(),
                                                //k_sampleRate,
                                                m_invertDataCurrentValue);

        bool shouldTryFlipInvertFlag = false;   //TODO: more logic to skip invert if no sense to do it?
        if(shouldTryFlipInvertFlag){
            SetState(CvbsAnalyzerState::k_restartInverted);
        }else{
            SetState(CvbsAnalyzerState::k_stopADC);
        }
    } // CvbsAnalyzerState::k_videoScoreCalculation

    if(m_state == CvbsAnalyzerState::k_restartInverted)
    {
        //No! do it outside of CvbsAlalyzer state machine.
        //Use this state just to request restart from outside.Maybe without stipping ADC.
        //TODO:Save results of non-inverted analysis

    } // CvbsAnalyzerState::k_restartInverted

    //if(m_state == CvbsAnalyzerState::k_stopADC)
    {
        //Stop ADC unconditionally from any state. Except k_restartInverted ??
        fastAdcState = m_fastAdc.StopADCSampling();
    } // CvbsAnalyzerState::k_stopADC

#if CVBS_ANALYZER_PROFILER
    m_stateProfilers[CvbsAnalyzerState::k_totalAnalyzeTime].Stop();
#endif // CVBS_ANALYZER_PROFILER    

    CVBS_ANALYZER_LOG("Stopped ADC sampling for gpioPin %d, samplesReadTotal = %d, CvbsAnalyzerState = %d\n", 
                      (int)gpioPin, (int)m_samplesReadTotal, (int)m_state);

    if (fastAdcState == FastADCState::k_initializedAdcStopped)
    {
        SetState(CvbsAnalyzerState::k_finished);
    }
    else
    {
        CVBS_ANALYZER_LOG("FastADC::StopADCSampling() for gpioPin %d failed with state %d!", (int)gpioPin, (int)fastAdcState);
        SetState(CvbsAnalyzerState::k_failedFastADCStop);
    }

    //PrintJson();
    if(k_printCsvLearningData)
    {
        PrintCsv();
    }
    if(k_printVideoScore)
    {
        CVBS_ANALYZER_LOG_INFO("VideoScore: isVideo=%f\n",  m_videoScore.m_isVideo);
    }


    return m_state;
}

void CvbsAnalyzer::PrintJson()
{
    CVBS_ANALYZER_LOG("{\n");
    CVBS_ANALYZER_LOG("\"CvbsAnalyzer\": {\n");
    CVBS_ANALYZER_LOG("\t\"m_invertDataCurrentValue\": %d,\n", m_invertDataCurrentValue ? 1 : 0);
    CVBS_ANALYZER_LOG("\t\"m_state\": %d,\n", (int)m_state);
    CVBS_ANALYZER_LOG("\t\"m_samplesReadTotal\": %d,\n", m_samplesReadTotal);
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
    CVBS_ANALYZER_LOG("\"FastADC\": { \"state\": %d, \"m_adcChannel\": %d },\n", (int)m_fastAdc.GetState(), (int)m_fastAdc.GetAdcChannel());
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
        (int)m_rawSamplesRead,
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
    }\

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