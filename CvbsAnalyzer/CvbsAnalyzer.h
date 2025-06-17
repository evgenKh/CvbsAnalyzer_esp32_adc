#ifndef CvbsAnalyzer_H
#define CvbsAnalyzer_H

#include "Arduino.h"
#include "CvbsAnalyzerGlobals.h"
#include "FastADC.h"
#include "AmplitudeCaclulator.h"
#include "SyncIntervalsCalculator.h"
#include "VideoScore.h"
#include "AverageFilter.h"
#include "CvbsAnalyzerJob.h"
#include "SamplesPreFilter.h"
#include <map>


#if CVBS_ANALYZER_PROFILER
struct CvbsAnalyzerProfiler
{
    CvbsAnalyzerProfiler() = default;
    CvbsAnalyzerProfiler(const char* name): m_name(name){};
    void Start();
    void Stop();

    const char* m_name = "";
    int64_t m_startTime = 0;
    bool m_started = false;
    int64_t m_microsecondsAccumulator = 0;    
};
#endif // CVBS_ANALYZER_PROFILER


class CvbsAnalyzer
{
    public:
    CvbsAnalyzer()
    {
        Reset();
        assert(s_instance == nullptr); //In theory we may use many instances, but it will need some additional states handling.
        s_instance = this;
    }
    ~CvbsAnalyzer()
    {
        assert(m_state == CvbsAnalyzerState::k_notInitialized);//You must deInit FastADC first.
        s_instance = nullptr;
    }

    void Reset();

    CvbsAnalyzerState InitializeFastADC();
    CvbsAnalyzerState DeinitializeFastADC();

    CvbsAnalyzerState AnalyzePin(int gpioPin)
    {
        CvbsAnalyzerJob job(gpioPin, CvbsAnalyzerJobType::k_videoScore, 
                                    CvbsAnalyzerInversionType::k_nonInvertedThenInverted);
        return ExecuteJob(job);
    }

    CvbsAnalyzerState AnalyzePinAverage(int gpioPin, bool invertData)
    {
        CvbsAnalyzerJob job(gpioPin, CvbsAnalyzerJobType::k_averageRssi,
                                     CvbsAnalyzerInversionType::k_nonInvertedOnly);
        
        return ExecuteJob(job);
    }

    CvbsAnalyzerState ExecuteJob(const CvbsAnalyzerJob& job);
    
    inline const CvbsAnalyzerJob::Result GetJobResult() const { return m_result; }

    inline CvbsAnalyzerState GetState() const { return m_state; }
    inline bool IsInErrorState() const { return ((signed char)m_state < 0); }
    void PrintJson();
    void PrintCsv();
    void SetLearningRssi(float rssi) { m_learningRssi = rssi; }
    

private:
    inline CvbsAnalyzerState SetState(CvbsAnalyzerState state)
    {
        if(m_state != state)
        {
            CVBS_ANALYZER_LOG("CvbsAnalyzer state changed from %d to %d\n", (int)m_state, (int)state);
#if CVBS_ANALYZER_PROFILER
            if(m_stateProfilers.count(m_state)) m_stateProfilers[m_state].Stop();
#endif
            m_state = state;
            m_result.m_cvbsAnalyzerState = m_state; 
            
#if CVBS_ANALYZER_PROFILER
            if(m_stateProfilers.count(m_state)) m_stateProfilers[m_state].Start();
#endif            
        }
        return m_state;
    }

    CvbsAnalyzerState m_state = CvbsAnalyzerState::k_notInitialized;

#if CVBS_ANALYZER_PROFILER
    std::map<CvbsAnalyzerState, CvbsAnalyzerProfiler> m_stateProfilers;
#endif // CVBS_ANALYZER_PROFILER
    void PrintProfilersJson();

    constexpr static bool k_syncIntervalsCalculatorConsumeMaxDmaReads = false; 

    constexpr static bool k_printRawAdcData = false; //Slow!
    constexpr static bool k_printCsvLearningData = false;
    constexpr static bool k_printJsonLearningData = false;
    

    bool m_invertDataCurrentValue;
    size_t m_rawSamplesReadTotal = 0;
    uint16_t m_rawAdcSamplesBuf[k_dmaBufLenSamples];//align to 4 bytes!

    
    FastADC m_fastAdc;
    AmplitudeCaclulator m_amplitudeCaclulator;
    SyncIntervalsCalculator m_syncIntervalsCalculator;
    MeanAverageFilter m_rssiAverageFilter;
    SamplesPreFilter m_samplesPreFilter;
    
    //Results
    CvbsAnalyzerJob::Result m_result;

    float m_learningRssi = -1.0f; 

    //AmplitudeCaclulator m_invertedAmplitudeCaclulator;
    //SyncIntervalsCalculator m_invertedIntervalsCalculator;

    static CvbsAnalyzer* s_instance;

};

#endif // CvbsAnalyzer_H