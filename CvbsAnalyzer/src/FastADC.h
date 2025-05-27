#ifndef FastADC_H
#define FastADC_H

#include "Arduino.h"
#include "hal/i2s_types.h"
#include "driver/adc_common.h"


class FastADC
{
    public:
    FastADC(int gpioPin);

    bool begin();
    void end();
    bool BeginADCContinuous();
    bool BeginADCI2S();
    bool BeginADCDetectedVideo();
    void i2s_read_samples();

    private:
    int m_gpioPin;
    adc1_channel_t m_adcChannel;
    static constexpr adc_unit_t k_adcUnit = ADC_UNIT_1;
    static constexpr uint32_t k_dmaBufLenSamples = 1024;//Min 2 TV lines(2*64us). Align to 4. Not too smal, otherwise we'll spend to much time swapping buffers.
    static constexpr uint8_t k_dmaBufsCount = 3;
    static constexpr uint32_t k_sampleRate = 2*1000*1000;//1Ms/s = 2MB/s?
    static constexpr adc_atten_t k_adcAttenuation = ADC_ATTEN_DB_6;
    static constexpr adc_bits_width_t k_adcWidth = ADC_WIDTH_BIT_12;

    adc1_channel_t gpioToAdcChannel(int gpio);

};

#endif // FastADC_H