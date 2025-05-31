#ifndef FastAdcContinuous_H
#define FastAdcContinuous_H

#if USE_FAST_ADC_CONTINUOUS

//Uses new driver included in ArduinoIDE

#include "Arduino.h"
#include "driver/adc.h"
#include "CvbsAnalyzerGlobals.h"


enum class FastADCState : signed char{
    k_notInitialized = 0,
    k_initializing = 1,
    k_initializedAdcStopped = 2,
    k_adcStarting = 3,
    k_adcStarted = 4,

    k_failedBadState = -127,
    k_initFailedInstallDriverInvalidArg,
    k_initFailedInstallDriverOutOfMem,
    k_initFailedInstallDriverPortInUse,
    k_initFailedUnknownError,
    k_startFailedBadPin,
    k_startFailedSetAdcMode,
    k_startFailedZeroDma,
    k_startFailedAdcEnable,
    k_stopFailedAdcDisable,
};

class FastADC
{
    public:
    FastADC();

    FastADCState Initialize();
    FastADCState Deinitialize();
    FastADCState StartADCSampling(int8_t gpioPin, bool invertData = false);
    FastADCState StopADCSampling();
    
    size_t ReadAndPrintSamples();
    size_t ReadSamplesBlockingTo(uint16_t* outBuf, size_t bufSizeBytes);

    inline FastADCState GetState() const { return m_state; }
    inline bool IsInErrorState() const { return ((signed char)m_state < 0); }
    adc1_channel_t GetAdcChannel() const { return m_adcChannel; }


    private:
    FastADCState m_state = FastADCState::k_notInitialized;
    //int8_t m_gpioPin = -1;
    adc1_channel_t m_adcChannel = adc1_channel_t::ADC1_CHANNEL_MAX;

    bool m_adcPreviousDataInvertEnabled;

    static constexpr adc_unit_t k_adcUnit = ADC_UNIT_1;//esp32 suppors only ADC_UNIT_1

    static constexpr adc_atten_t k_adcAttenuation = ADC_ATTEN_DB_12;

    static constexpr adc_bits_width_t k_adcWidth = ADC_WIDTH_BIT_12;

    static constexpr TickType_t k_dmaReadTimeoutMs = 1;
    static constexpr TickType_t k_dmaReadTimeout = k_dmaReadTimeoutMs * portTICK_PERIOD_MS; //default: portMAX_DELAY

#if FAST_ADC_2Mhz

#elif FAST_ADC_1Mhz

#endif

};


#endif // USE_FAST_ADC_CONTINUOUS

#endif // FastAdcContinuous_H