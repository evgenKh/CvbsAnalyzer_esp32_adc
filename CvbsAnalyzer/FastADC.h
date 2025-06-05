#ifndef FastADC_H
#define FastADC_H

#include "Arduino.h"
#include "CvbsAnalyzerGlobals.h"



//Uses old ADC+I2S drivers, since arduino framework in platformio is old.

//Tested on:
// Platformio
// Platform espressif32 @ 6.10.0
// framework-arduinoespressif32 @ 3.20017.241212+sha.dcc1105b (https://github.com/platformio/platform-espressif32)
//  L Contains Arduino support - v2.0.17 (based on IDF v4.4.7)
//  L Contains ESP-IDF support(without arduino) - v5.4.0

//#   include "hal/adc_ll.h"
//#   include "driver/adc.h"
#include "hal/adc_types.h" // for adc_atten_t

//#pragma GCC diagnostic ignored "-fpermissive"
//#   include "driver/i2s.h"

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
    k_startFailedConfig,
    k_startFailedContinuousStart,
    k_stopFailedAdcDisable,
    k_i2sReadFailed,
    k_continuousReadFailed,
    k_continuousReadFailedInvalidData,
};

struct adc_continuous_ctx_t;
typedef struct adc_continuous_ctx_t *adc_continuous_handle_t;

class FastADC
{
public:
    FastADC();

    FastADCState Initialize();
    FastADCState Deinitialize();
    FastADCState StartADCSampling(int8_t gpioPin, bool invertData = false);
    FastADCState StopADCSampling();

    void PrintADCRegisters();
    
    size_t ReadSamplesBlockingTo(uint16_t* outBuf, size_t bufSizeBytes);

    inline FastADCState GetState() const { return m_state; }
    inline bool IsInErrorState() const { return ((signed char)m_state < 0); }

    QueueHandle_t m_i2sEventQueue = nullptr;

private:
    void SetClkDiv(uint16_t integer, uint16_t denominator, uint16_t numerator);
    void DrainDMA();

    FastADCState m_state = FastADCState::k_notInitialized;
    adc_continuous_handle_t m_handle = nullptr;
    bool m_adcPreviousDataInvertEnabled;

    static constexpr bool k_printRegisters = false; 


    static constexpr adc_atten_t k_adcAttenuation = ADC_ATTEN_DB_12;
    //static constexpr adc_bits_width_t k_adcWidth = ADC_WIDTH_BIT_12;
    static constexpr uint8_t k_adcWidthBits = 12;

    //static constexpr uint32_t k_startAdcDelayMs = 10;
    static constexpr TickType_t k_dmaReadTimeoutMs = 2* (1+((k_i2sSampleRate * k_oversamplingMultiplier) / 1000 / k_dmaBufLenSamples));
    static constexpr TickType_t k_dmaReadTimeout = k_dmaReadTimeoutMs * portTICK_PERIOD_MS; //default: portMAX_DELAY

    static constexpr uint8_t k_i2sEventQueueSize = 0;

#if FAST_ADC_2Mhz
    static constexpr uint8_t k_adcAPBClockDiv = 2;//ADC clock divider, ADC clock is divided from APB clock
    static constexpr uint32_t k_adcSampleCycle = 2;//The number of ADC sampling cycles. Range: 1 ~ 7.
    static constexpr uint16_t k_adcMeasNum = 255;//This was 255 on old versions of Arduino, but 10 on newer. Keep it 255
    
    static constexpr uint16_t k_i2sMclkDiv = 20;// I2S module clock devider, Fmclk = Fsclk /(mclk_div+b/a)
    static constexpr uint16_t k_i2sRxBckDiv = 2;//Bit clock configuration bits. I don't know what this means.
    static constexpr uint16_t k_i2sClkmDivA = 1;//This was 1 on old versions of Arduino, but 2 on newer. Keep it 1
#elif FAST_ADC_1Mhz
    //This one gives the most smooth data.
    static constexpr uint8_t k_adcAPBClockDiv = 2;//ADC clock divider, ADC clock is divided from APB clock
    static constexpr uint32_t k_adcSampleCycle = 4;//The number of ADC sampling cycles. Range: 1 ~ 7.
    static constexpr uint16_t k_adcMeasNum = 255;//This was 255 on old versions of Arduino, but 10 on newer. Keep it 255
    //This in not true sample rate... 
    static constexpr uint16_t k_i2sMclkDiv = 20;// I2S module clock devider, Fmclk = Fsclk /(mclk_div+b/a)
    static constexpr uint16_t k_i2sRxBckDiv = 2;//Bit clock configuration bits. I don't know what this means.
    static constexpr uint16_t k_i2sClkmDivA = 1;//This was 1 on old versions of Arduino, but 2 on newer. Keep it 1
#endif

};

#endif // FastADC_H
