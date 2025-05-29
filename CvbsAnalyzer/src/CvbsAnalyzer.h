#ifndef CvbsAnalyzer_H
#define CvbsAnalyzer_H

#include "Arduino.h"
#include "CvbsAnalyzerGlobals.h"
#include "FastADC.h"
#include "AmplitudeCaclulator.h"
#include "SyncIntervalsCalculator.h"

enum class CvbsAnalyzerState : signed char
{
    k_notInitialized = 0,
    k_initializing,
    k_initializedAndIdle,

    //k_sampling,
    k_amplitudeSampling,
    k_amplitudeCalculation,
    k_syncIntervalsSampling,
    k_syncIntervalsCalculation,
    k_videoScoreCalculation,
    k_restartInverted,
    k_stopADC,
    k_finished,

    k_failedBadState = -127,
    k_failedFastADCInitialization,
    k_failedSampling,
    k_failedAmplitude,
    k_failedSyncIntervals,
    k_failedVideoScore,
    k_failedFastADCStop,
    k_failedUnknownError
};    

class CvbsAnalyzer
{
    public:
    CvbsAnalyzer(){}

    FastADC m_fastAdc;
    AmplitudeCaclulator m_amplitudeCaclulator;
    SyncIntervalsCalculator m_syncIntervalsCalculator;

    AmplitudeCaclulator m_invertedAmplitudeCaclulator;
    SyncIntervalsCalculator m_invertedIntervalsCalculator;

    CvbsAnalyzerState InitializeFastADC();
    CvbsAnalyzerState DeinitializeFastADC();
    CvbsAnalyzerState AnalyzePin(int gpioPin);
    inline CvbsAnalyzerState GetState() const { return m_state; }
    inline bool IsInErrorState() const { return ((signed char)m_state < 0); }

    private:
    CvbsAnalyzerState m_state = CvbsAnalyzerState::k_notInitialized;

    constexpr static size_t k_maxDmaReadsPerAnalyzePin = 8;//k_dmaBufsCount;//Each has timeout of k_dmaReadTimeoutMs and size of k_dmaBufLenSamples
    constexpr static bool k_syncIntervalsCalculatorConsumeMaxDmaReads = true; 
    constexpr static bool k_printRawAdcData = false; //May add delays!
};

#endif // CvbsAnalyzer_H