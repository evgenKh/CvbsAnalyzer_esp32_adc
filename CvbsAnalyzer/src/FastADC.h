#ifndef FastADC_H
#define FastADC_H

//Uses old ADC+I2S drivers, since arduino framework in platformio is old.

//Tested on:
// Platformio
// Platform espressif32 @ 6.10.0
// framework-arduinoespressif32 @ 3.20017.241212+sha.dcc1105b (https://github.com/platformio/platform-espressif32)
//  L Contains Arduino support - v2.0.17 (based on IDF v4.4.7)
//  L Contains ESP-IDF support(without arduino) - v5.4.0


#include "Arduino.h"
//#include "hal/i2c_ll.h"
//#include "hal/i2s_types.h"
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

    QueueHandle_t m_i2sEventQueue = nullptr;


    private:
    FastADCState m_state = FastADCState::k_notInitialized;
    //int8_t m_gpioPin = -1;
    adc1_channel_t m_adcChannel = adc1_channel_t::ADC1_CHANNEL_MAX;

    bool m_adcPreviousDataInvertEnabled;

    static constexpr adc_unit_t k_adcUnit = ADC_UNIT_1;//esp32 suppors only ADC_UNIT_1

    static constexpr adc_atten_t k_adcAttenuation = ADC_ATTEN_DB_12;

    static constexpr adc_bits_width_t k_adcWidth = ADC_WIDTH_BIT_12;
    static constexpr uint8_t k_i2sEventQueueSize = 0;
    static constexpr uint32_t k_startAdcDelayMs = 10;
    static constexpr TickType_t k_dmaReadTimeoutMs = 1;
    static constexpr TickType_t k_dmaReadTimeout = k_dmaReadTimeoutMs * portTICK_PERIOD_MS; //default: portMAX_DELAY

#if FAST_ADC_2Mhz
    static constexpr uint8_t k_adcAPBClockDiv = 2;//ADC clock divider, ADC clock is divided from APB clock
    static constexpr uint32_t k_adcSampleCycle = 2;//The number of ADC sampling cycles. Range: 1 ~ 7.
    //static constexpr uint32_t k_i2sSampleRate = 2*1000*1000;//defined in globals
    static constexpr uint16_t k_i2sMclkDiv = 20;// I2S module clock devider, Fmclk = Fsclk /(mclk_div+b/a)
    static constexpr uint16_t k_i2sRxBckDiv = 2;//Bit clock configuration bits. I don't know what this means.
#elif FAST_ADC_1Mhz
    //This one gives the most smooth data.
    static constexpr uint8_t k_adcAPBClockDiv = 2;//ADC clock divider, ADC clock is divided from APB clock
    static constexpr uint32_t k_adcSampleCycle = 4;//The number of ADC sampling cycles. Range: 1 ~ 7.
    //This in not true sample rate... 
    //static constexpr uint32_t k_i2sSampleRate = 1*1000*1000;//defined in globals
    static constexpr uint16_t k_i2sMclkDiv = 20;// I2S module clock devider, Fmclk = Fsclk /(mclk_div+b/a)
    static constexpr uint16_t k_i2sRxBckDiv = 2;//Bit clock configuration bits. I don't know what this means.
#endif

};

#endif // FastADC_H