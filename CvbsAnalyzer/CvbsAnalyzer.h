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
    }

    void Reset();

    CvbsAnalyzerState InitializeFastADC();
    CvbsAnalyzerState DeinitializeFastADC();

    CvbsAnalyzerState AnalyzePin(int gpioPin, bool invertData)
    {
        return ExecuteJob(gpioPin, CvbsAnalyzerJobType::k_videoScore, invertData);
    }
    CvbsAnalyzerState AnalyzePinAverage(int gpioPin, bool invertData)
    {
        return ExecuteJob(gpioPin, CvbsAnalyzerJobType::k_averageRssi, invertData);
    }

    CvbsAnalyzerState ExecuteJob(int gpioPin, CvbsAnalyzerJobType jobType, bool invertData);
    

    const VideoScore& GetVideoScore() const { return m_videoScore; }
    uint16_t GetPinAverage() const { return m_pinAverage; }

    inline CvbsAnalyzerState GetState() const { return m_state; }
    inline bool IsInErrorState() const { return ((signed char)m_state < 0); }
    void PrintJson();
    void PrintCsv();

private:
    inline CvbsAnalyzerState SetState(CvbsAnalyzerState state)
    {
        if(m_state != state)
        {
            CVBS_ANALYZER_LOG("CvbsAnalyzer state changed from %d to %d\n", (int)m_state, (int)state);
#if CVBS_ANALYZER_PROFILER
            m_stateProfilers[m_state].Stop();
#endif
            m_state = state;
#if CVBS_ANALYZER_PROFILER
            m_stateProfilers[m_state].Start();
#endif            
        }
        return m_state;
    }

    CvbsAnalyzerState m_state = CvbsAnalyzerState::k_notInitialized;

#if CVBS_ANALYZER_PROFILER
    std::map<CvbsAnalyzerState, CvbsAnalyzerProfiler> m_stateProfilers;
#endif // CVBS_ANALYZER_PROFILER

    constexpr static size_t k_maxDmaReadsPerAnalyzePin = k_dmaBufsCount;//Each has timeout of k_dmaReadTimeoutMs and size of k_dmaBufLenSamples
    constexpr static bool k_syncIntervalsCalculatorConsumeMaxDmaReads = true; 

    constexpr static bool k_printRawAdcData = false; //Slow!
    constexpr static bool k_printCsvLearningData = false;
    constexpr static bool k_printVideoScore = false;
    constexpr static size_t k_preFilteredSamplesBufLenSamples = k_dmaBufLenSamples * k_dmaBufsCount/ k_adcDataStrideSamples; 
    

    bool m_invertDataCurrentValue;
    size_t m_samplesReadTotal = 0;
    uint16_t m_rawAdcSamplesBuf[k_dmaBufLenSamples];//align to 4 bytes!

    //uint16_t m_preFilteredSamplesBuf[k_preFilteredSamplesBufLenSamples];
    //size_t m_preFilteredSamplesBufHead = 0;
    
    FastADC m_fastAdc;
    AmplitudeCaclulator m_amplitudeCaclulator;
    SyncIntervalsCalculator m_syncIntervalsCalculator;
    AverageFilter m_rssiAverageFilter;
    
    //Results
    VideoScore m_videoScore;
    uint16_t m_pinAverage;

    //AmplitudeCaclulator m_invertedAmplitudeCaclulator;
    //SyncIntervalsCalculator m_invertedIntervalsCalculator;


};

#endif // CvbsAnalyzer_H