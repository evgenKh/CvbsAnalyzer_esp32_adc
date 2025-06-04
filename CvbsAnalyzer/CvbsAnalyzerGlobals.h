#ifndef CvbsAnalyzerGlobals_H
#define CvbsAnalyzerGlobals_H
#include "Arduino.h"

#include "esp_arduino_version.h"

#define CVBS_ANALYZER_LOG_INFO(...) Serial.printf(__VA_ARGS__);
//#define CVBS_ANALYZER_LOG_INFO(...)
//#define CVBS_ANALYZER_LOG_DEBUG(...) Serial.printf(__VA_ARGS__);
#define CVBS_ANALYZER_LOG_DEBUG(...)
#define CVBS_ANALYZER_LOG(...) CVBS_ANALYZER_LOG_DEBUG(__VA_ARGS__);

#define CVBS_ANALYZER_PROFILER 1

#ifndef ESP_ARDUINO_VERSION_MAJOR
    #error "Can't find esp32 core version, maybe esp_arduino_version.h not included"
#endif

#if (ESP_ARDUINO_VERSION_MAJOR <= 2 && ESP_ARDUINO_VERSION_PATCH <= 17)
    #define USE_FAST_ADC_CONTINUOUS 0
    #define FAST_ADC_2Mhz 0
    #define FAST_ADC_1Mhz 1
#else
    //in newer sdk use contnuous driver
    #define USE_FAST_ADC_CONTINUOUS 1
    #define FAST_ADC_2Mhz 0
    #define FAST_ADC_1Mhz 1
#endif

#if FAST_ADC_2Mhz
    constexpr uint32_t k_i2sSampleRate = 2*1000*1000;
    constexpr uint32_t k_oversamplingMultiplier = 1;
    constexpr size_t k_adcDataStrideSamples = 1; //1 sample per read, no stride
#elif FAST_ADC_1Mhz
    //This one gives the most smooth data.

    constexpr uint32_t k_i2sSampleRate = 1*1000*1000;
    //This is not real oversampling, but for some reason each sample repeated twice with this settings.
    //I2S feeds us 2Msamples, but every sample repeats twice
    constexpr uint32_t k_oversamplingMultiplier = 2;
    constexpr size_t k_adcDataStrideSamples = 2; //1 good sample, 1 possibly corrupted sample
#endif

    constexpr uint32_t k_dmaBufLenSamples = 800;//Min 2 TV lines(2*64us). Align to 4. 
                                                //Not too smal, otherwise we'll spend to much time swapping buffers.
                                                //k_dmaBufLenSamples*channels*(sampleSizeBits/8) not higher than 4096 
                                                //For 16bit*1chan max=1024

    constexpr uint8_t k_dmaBufsCount = 12;//increase if we want to print values since print is slow

    constexpr uint32_t k_sampleRate = k_i2sSampleRate * k_oversamplingMultiplier / k_adcDataStrideSamples;
    constexpr uint32_t k_sampleRateWithSkippedOversamples = k_i2sSampleRate;
    constexpr bool k_skipLeadingZeroSamples = true;

    //inline uint16_t PreProcessSample(const uint16_t sample, const uint16_t invertMask = 0x00)
    //{
    //    return (sample & k_adcDataMask) ^ invertMask;
    //}
    //inline bool IsErrorState(signed char state){ return (state < 0); }
    inline int32_t UsToSamples(const int32_t microseconds)
    {
        static_assert(k_sampleRate == 1000000 || k_sampleRate == 2000000);
        //k_sampleRate is 1Mhz or 2Mhz, so we can use integer math.
        return (microseconds * (k_sampleRate / 1000000));
    }
    constexpr int32_t UsToSamplesContexpr(const int32_t microseconds)
    {
        static_assert(k_sampleRate == 1000000 || k_sampleRate == 2000000);
        //k_sampleRate is 1Mhz or 2Mhz, so we can use integer math.
        return (microseconds * (k_sampleRate / 1000000));
    }

    //Use in case k_sampleRate is not 1Mhz or 2Mhz.
    //inline int32_t UsToSamples(const int32_t microseconds)
    //{
    //    //0.5f is for rounding
    //    return (int32_t)(microseconds * ((float)k_sampleRate / 1000000.0f) + 0.5f);
    //}
    //constexpr int32_t UsToSamplesContexpr(const int32_t microseconds)
    //{
    //    //0.5f is for rounding
    //    return (int32_t)(microseconds * ((float)k_sampleRate / 1000000.0f) + 0.5f);
    //}

enum class CvbsAnalyzerState : signed char
{
    k_notInitialized = 0,
    k_initializing,
    k_initializedAndIdle,

    //k_sampling,
    k_preFilteringSamples,
    k_averageCalculation,
    k_amplitudeSampling,
    k_amplitudeCalculation,
    k_syncIntervalsSampling,
    k_syncIntervalsCalculation,
    k_videoScoreCalculation,
    k_restartInverted,
    k_stopADC,
    k_finished,
    k_totalAnalyzeTime,//not a state, just for profiling

    k_failedBadState = -127,
    k_failedBadFastADCState,
    k_failedFastADCInitialization,
    k_failedSampling,
    k_failedAmplitude,
    k_failedSyncIntervals,
    k_failedVideoScore,
    k_failedFastADCStop,
    k_failedUnknownError,
};    


#endif // CvbsAnalyzerGlobals_H