#include "CvbsAnalyzer.h"

CvbsAnalyzerState CvbsAnalyzer::InitializeFastADC()
{
    if(m_state != CvbsAnalyzerState::k_notInitialized)
    {
        m_state = CvbsAnalyzerState::k_failedBadState;
        return  m_state;
    }

    const FastADCState fastAdcState = m_fastAdc.Initialize();
    if (fastAdcState != FastADCState::k_initializedAdcStopped)
    {
        m_state = CvbsAnalyzerState::k_failedFastADCInitialization;
        return m_state;
    }
    else
    {
        m_state = CvbsAnalyzerState::k_initializedAndIdle;
    }

    return m_state;        
}

CvbsAnalyzerState CvbsAnalyzer::DeinitializeFastADC()
{
    m_fastAdc.Deinitialize();
    m_state = CvbsAnalyzerState::k_notInitialized;
    return m_state;
}

void PreProcessBuf(uint16_t* samples, size_t samplesCount, bool invertData, size_t strideSamples)
{
#if OPT_PREPROCESS_BUF
    if((samplesCount * sizeof(uint16_t)) % sizeof(uint32_t) == 0 && !strideSamples)
    {
        //Can process 32but a time
        uint32_t* samples32 = reinterpret_cast<uint32_t*>(samples);
        const uint32_t samples32Count = samplesCount * sizeof(uint16_t)/ sizeof(uint32_t);
        constexpr uint32_t mask12bit = (uint32_t)k_adcDataMask | ((uint32_t)k_adcDataMask << 16);
        constexpr uint32_t invertMask = (uint32_t)k_adcDataXorMaskForInvert|((uint32_t)k_adcDataXorMaskForInvert<<16);
        
        if(invertData)
        {
            //Process 32bit a time
            for(size_t i = 0; i < samples32Count; i++)
            {
                samples32[i] = (samples32[i] ^ invertMask) & mask12bit;
            }
        }
        else
        {
            //Process 32bit a time
            for(size_t i = 0; i < samples32Count; i++)
            {
                samples32[i] &= mask12bit;
            }
        }
    }
    else
#endif
    {
        //Process 16bit a time
        if(invertData)
        {
            for(size_t i = 0; i < samplesCount; i += strideSamples)
            {
                samples[i] = (samples[i] ^ k_adcDataXorMaskForInvert) & k_adcDataMask;
            }
        }
        else
        {
            for(size_t i = 0; i < samplesCount; i += strideSamples)
            {
                samples[i] &= k_adcDataMask;
            }
        }         
    }
}

CvbsAnalyzerState CvbsAnalyzer::AnalyzePin(int gpioPin)
{
    if (m_state != CvbsAnalyzerState::k_initializedAndIdle)
    {
        m_state = CvbsAnalyzerState::k_failedBadState;
        return m_state;
    }

    m_amplitudeCaclulator.Reset();
    m_syncIntervalsCalculator.Reset();

    bool invertByHardware = false;
    bool invertBySoftware = false;

    CVBS_ANALYZER_LOG("Starting AnalyzePin for gpioPin %d, invertByHardware=%d, invertBySoftware=%d...\n", 
                                    (int)gpioPin, (invertByHardware ? 1 : 0), (invertBySoftware ? 1 : 0));

    FastADCState fastAdcState = m_fastAdc.StartADCSampling(gpioPin, invertByHardware);
    if (fastAdcState != FastADCState::k_adcStarted)
    {
        CVBS_ANALYZER_LOG("FastADC::StartADCSampling() for gpioPin %d failed with state %d!\n", (int)gpioPin, (int)fastAdcState);
        m_state = CvbsAnalyzerState::k_failedSampling;
        return m_state;
    }

    m_state = CvbsAnalyzerState::k_amplitudeSampling;

    static uint16_t buf[k_dmaBufLenSamples];//align to 4 bytes!
    size_t samplesRead = 0;
    size_t samplesReadTotal = 0;

    for (int run = 0; run < k_maxDmaReadsPerAnalyzePin; run++)
    {
        size_t bytesRead = m_fastAdc.ReadSamplesBlockingTo(buf, sizeof(buf));
        samplesRead = bytesRead / sizeof(uint16_t);
        samplesReadTotal += samplesRead;

        if (samplesRead == 0)
            continue;

        PreProcessBuf(buf, samplesRead, invertBySoftware, k_adcDataStrideSamples);

        if(k_printRawAdcData)
        {
            CVBS_ANALYZER_LOG("Printing %d samples from run #%d ----\n", samplesRead, run);
            for (int i = 0; i < samplesRead; i+=k_adcDataStrideSamples)
            {
                CVBS_ANALYZER_LOG("%d\n", buf[i]);
            }
            CVBS_ANALYZER_LOG("%d samples printed.(run %d) ----\n", samplesRead);
        }

        //handling non-error states
        if(m_state == CvbsAnalyzerState::k_amplitudeSampling)
        {
            if (m_amplitudeCaclulator.GetState() == AmplitudeCaclulatorState::k_needMoreSamples)
            {
                m_amplitudeCaclulator.PushSamples(buf, samplesRead, k_adcDataStrideSamples);
            }
            // no else if
            if (m_amplitudeCaclulator.GetState() == AmplitudeCaclulatorState::k_readyForCalculation)
            {
                m_state = CvbsAnalyzerState::k_amplitudeCalculation;                
            }
            if(m_amplitudeCaclulator.IsInErrorState())
            {
                m_state = CvbsAnalyzerState::k_failedAmplitude;
            }
        } // CvbsAnalyzerState::k_amplitudeSampling

        if(m_state == CvbsAnalyzerState::k_amplitudeCalculation)
        {
            assert(m_amplitudeCaclulator.GetState() == AmplitudeCaclulatorState::k_readyForCalculation);
            m_amplitudeCaclulator.Calculate();
            if(m_amplitudeCaclulator.GetState() == AmplitudeCaclulatorState::k_finished)
            {
                m_state = CvbsAnalyzerState::k_syncIntervalsSampling;
            }
            if(m_amplitudeCaclulator.IsInErrorState())
            {
                m_state = CvbsAnalyzerState::k_failedAmplitude;
            }
        } // CvbsAnalyzerState::k_amplitudeCalculation

        if(m_state == CvbsAnalyzerState::k_syncIntervalsSampling)
        {
            assert(m_amplitudeCaclulator.GetState() == AmplitudeCaclulatorState::k_finished);

            if (m_syncIntervalsCalculator.GetState() == SyncIntervalsCalculatorState::k_needMoreSamples 
                || m_syncIntervalsCalculator.GetState() == SyncIntervalsCalculatorState::k_finished)
            {
                m_syncIntervalsCalculator.PushSamples(buf, samplesRead, m_amplitudeCaclulator.m_syncTreshold, k_adcDataStrideSamples);
            }

            if(m_syncIntervalsCalculator.GetState() == SyncIntervalsCalculatorState::k_finished )
            {
                //Actually calculation done by this moment, but using this state to get back to sampling if needed
                m_state = CvbsAnalyzerState::k_syncIntervalsCalculation;
            }
            if(m_syncIntervalsCalculator.IsInErrorState())
            {
                m_state = CvbsAnalyzerState::k_failedSyncIntervals;
                CVBS_ANALYZER_LOG("SyncIntervalsCalculator in error state %d\n", (int)m_syncIntervalsCalculator.GetState());
            }            
        } // CvbsAnalyzerState::k_syncIntervalsSampling

        if(m_state == CvbsAnalyzerState::k_syncIntervalsCalculation)
        {
            assert(m_syncIntervalsCalculator.GetState() == SyncIntervalsCalculatorState::k_finished);
            if(k_syncIntervalsCalculatorConsumeMaxDmaReads && run < k_maxDmaReadsPerAnalyzePin-1)
            {
                //Actually calculation done by this moment, but we can afford more sampling for precision
                m_state = CvbsAnalyzerState::k_syncIntervalsSampling;
                continue;
            }
            m_state = CvbsAnalyzerState::k_videoScoreCalculation;
        } // CvbsAnalyzerState::k_syncIntervalsCalculation

        if(m_state == CvbsAnalyzerState::k_videoScoreCalculation)
        {
            assert(m_syncIntervalsCalculator.GetState() == SyncIntervalsCalculatorState::k_finished);
            //TODO:
            bool shouldTryInverted = false;
            if(shouldTryInverted)
            {
                m_state = CvbsAnalyzerState::k_restartInverted;
            }else{
                m_state = CvbsAnalyzerState::k_stopADC;
            }
            //TODO: handle score errors
        } // CvbsAnalyzerState::k_videoScoreCalculation

        if(m_state == CvbsAnalyzerState::k_restartInverted)
        {
            //TODO:
        } // CvbsAnalyzerState::k_restartInverted

        if(m_state == CvbsAnalyzerState::k_stopADC)
        {
            //Will do it out of cycle.
            break;
        } // CvbsAnalyzerState::k_stopADC

        //Error states handling
        if(IsInErrorState())
        {
            CVBS_ANALYZER_LOG("CvbsAnalyzer in error state %d\n", (int)m_state);
            break;
        }
    } // for

    if(!samplesReadTotal)
    {
        m_state = CvbsAnalyzerState::k_failedSampling;
    }

    //Stop ADC unconditionally from any state.
    fastAdcState = m_fastAdc.StopADCSampling();
    CVBS_ANALYZER_LOG("Stopping ADC sampling for gpioPin %d, samplesReadTotal = %d, CvbsAnalyzerState = %d\n", 
                      (int)gpioPin, (int)samplesReadTotal, (int)m_state);

    if (fastAdcState == FastADCState::k_initializedAdcStopped)
    {
        m_state = CvbsAnalyzerState::k_finished;
    }
    else
    {
        CVBS_ANALYZER_LOG("FastADC::StopADCSampling() for gpioPin %d failed with state %d!", (int)gpioPin, (int)fastAdcState);
        m_state = CvbsAnalyzerState::k_failedFastADCStop;
    }

    m_amplitudeCaclulator.Print();
    m_syncIntervalsCalculator.Print();

    //CVBS_ANALYZER_LOG("Printing last page of %d samples ----------------\n", samplesRead);
    //for (int i = 0; i < samplesRead; i+=k_adcDataStrideSamples)
    //{
    //    CVBS_ANALYZER_LOG("%d\n", buf[i]);
    //}
    return m_state;
}
