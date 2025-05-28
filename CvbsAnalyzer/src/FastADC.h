#ifndef FastADC_H
#define FastADC_H

#include "Arduino.h"
//#include "hal/i2c_ll.h"
//#include "hal/i2s_types.h"
#include "driver/adc.h"

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
    FastADCState StartADCSampling(int8_t gpioPin);
    FastADCState StopADCSampling();
    
    size_t ReadSamplesBlocking();
    size_t ReadSamplesBlockingTo(int16_t* outBuf, size_t bufSizeBytes);

    QueueHandle_t m_i2sEventQueue = nullptr;


    private:
    FastADCState m_state = FastADCState::k_notInitialized;
    //int8_t m_gpioPin = -1;
    adc1_channel_t m_adcChannel = adc1_channel_t::ADC1_CHANNEL_MAX;

    static constexpr adc_unit_t k_adcUnit = ADC_UNIT_1;//esp32 suppors only ADC_UNIT_1

    static constexpr uint32_t k_dmaBufLenSamples = 400;//Min 2 TV lines(2*64us). Align to 4. 
                                                        //Not too smal, otherwise we'll spend to much time swapping buffers.
                                                        //k_dmaBufLenSamples*channels*(sampleSizeBits/8) not higher than 4096 
                                                        //For 16bit*1chan max=1024
    static constexpr uint8_t k_dmaBufsCount = 3;
    static constexpr adc_atten_t k_adcAttenuation = ADC_ATTEN_DB_12;
    static constexpr adc_bits_width_t k_adcWidth = ADC_WIDTH_BIT_12;
    static constexpr uint8_t k_i2sEventQueueSize = 0;
    static constexpr uint32_t k_startAdcDelayMs = 10;

#define FAST_ADC_2Mhz 1
#define FAST_ADC_1Mhz 0
#if FAST_ADC_2Mhz
    static constexpr uint32_t k_sampleRate = 2*1000*1000;
    static constexpr uint8_t k_adcAPBClockDiv = 2;//ADC clock divider, ADC clock is divided from APB clock
    static constexpr uint32_t k_adcSampleCycle = 2;//The number of ADC sampling cycles. Range: 1 ~ 7.
    static constexpr uint16_t k_i2sMclkDiv = 20;// I2S module clock devider, Fmclk = Fsclk /(mclk_div+b/a)
#elif FAST_ADC_1Mhz
    static constexpr uint32_t k_sampleRate = 1*1000*1000;
    static constexpr uint8_t k_adcAPBClockDiv = 2;//ADC clock divider, ADC clock is divided from APB clock
    static constexpr uint32_t k_adcSampleCycle = 4;//The number of ADC sampling cycles. Range: 1 ~ 7.
    static constexpr uint16_t k_i2sMclkDiv = 40;// I2S module clock devider, Fmclk = Fsclk /(mclk_div+b/a)
#endif
    adc1_channel_t gpioToAdc1Channel(int gpio);

};

#endif // FastADC_H