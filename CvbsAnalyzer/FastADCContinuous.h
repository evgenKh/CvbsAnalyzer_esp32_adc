#ifndef FastAdcContinuous_H
#define FastAdcContinuous_H

#include "CvbsAnalyzerGlobals.h"

#if USE_FAST_ADC_CONTINUOUS

//Uses new driver included in ArduinoIDE

#include "Arduino.h"
#include "driver/adc.h"
#include "esp_adc/adc_continuous.h"


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
    void SetClkDiv(uint16_t integer, uint16_t denominator, uint16_t numerator);
    
    FastADCState m_state = FastADCState::k_notInitialized;
    adc_continuous_handle_t m_handle = NULL;
    //int8_t m_gpioPin = -1;
    adc1_channel_t m_adcChannel = adc1_channel_t::ADC1_CHANNEL_MAX;

    bool m_adcPreviousDataInvertEnabled;

    static constexpr adc_unit_t k_adcUnit = ADC_UNIT_1;//esp32 suppors only ADC_UNIT_1

    static constexpr adc_atten_t k_adcAttenuation = ADC_ATTEN_DB_12;

    static constexpr adc_bits_width_t k_adcWidth = ADC_WIDTH_BIT_12;
    static constexpr uint8_t k_adcWidthBits = 12;

    static constexpr TickType_t k_dmaReadTimeoutMs = 1;
    static constexpr TickType_t k_dmaReadTimeout = k_dmaReadTimeoutMs * portTICK_PERIOD_MS; //default: portMAX_DELAY

    //static constexpr uint16_t k_adcMeasNum = 255;//This was 255 on old versions of Arduino, but 10 on newer. Keep it 255
    static constexpr uint16_t k_i2sMclkDiv = 20;// I2S module clock devider, Fmclk = Fsclk /(mclk_div+b/a)
    static constexpr uint32_t k_adcSampleCycle = 4;//The number of ADC sampling cycles. Range: 1 ~ 7.
    static constexpr uint16_t k_i2sRxBckDiv = 2;//Bit clock configuration bits. I don't know what this means.

#if FAST_ADC_2Mhz

#elif FAST_ADC_1Mhz

#endif

};


#endif // USE_FAST_ADC_CONTINUOUS

#endif // FastAdcContinuous_H