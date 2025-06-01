#include "FastADCContinuous.h"

#if USE_FAST_ADC_CONTINUOUS
#include "driver/adc.h"
#include "hal/adc_hal.h"
#include "esp32-hal-adc.h"
#include "esp_adc/adc_continuous.h"
#include <map>


const std::map<int8_t, adc1_channel_t> k_gpioToAdc1Channel = {
    {36, ADC1_CHANNEL_0},
    {37, ADC1_CHANNEL_1},
    {38, ADC1_CHANNEL_2},
    {39, ADC1_CHANNEL_3},
    {32, ADC1_CHANNEL_4},
    {33, ADC1_CHANNEL_5},
    {34, ADC1_CHANNEL_6},
    {35, ADC1_CHANNEL_7}
};

FastADC::FastADC()
{

}

FastADCState FastADC::Initialize()
{    
    if(m_state != FastADCState::k_notInitialized)
    {
        m_state = FastADCState::k_failedBadState;
        return m_state;
    }
    m_state = FastADCState::k_initializing;

    m_handle = NULL;

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = k_dmaBufLenSamples*sizeof(uint16_t)*k_dmaBufsCount,
        .conv_frame_size = k_dmaBufLenSamples*sizeof(uint16_t),
    };
    
    esp_err_t err = adc_continuous_new_handle(&adc_config, &m_handle);

    switch (err)
    {
    case ESP_ERR_INVALID_ARG:
        m_state = FastADCState::k_initFailedInstallDriverInvalidArg;
        return m_state;
    case ESP_ERR_NO_MEM:
        m_state = FastADCState::k_initFailedInstallDriverOutOfMem;
        return m_state;
    case ESP_ERR_INVALID_STATE:
        m_state = FastADCState::k_initFailedInstallDriverPortInUse;
        return m_state;
    case ESP_OK:
        break;    
    default:
        m_state = FastADCState::k_initFailedUnknownError;
        return m_state;
    }

    m_state = FastADCState::k_initializedAdcStopped;
    return m_state;
}
FastADCState FastADC::Deinitialize()
{
    ESP_ERROR_CHECK(adc_continuous_deinit(m_handle));
    return m_state;
}
FastADCState FastADC::StartADCSampling(int8_t gpioPin, bool invertData)
{

    if(k_gpioToAdc1Channel.count(gpioPin) == 0)
    {
        m_state = FastADCState::k_startFailedBadPin;
        return m_state;
    }
    m_state = FastADCState::k_adcStarting;
    m_adcChannel = k_gpioToAdc1Channel.at(gpioPin);

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    
    
    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = k_i2sSampleRate,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };

    dig_cfg.pattern_num = 1;

    for (int i = 0; i < 1; i++) {
        adc_pattern[i].atten = k_adcAttenuation;
        adc_pattern[i].channel = m_adcChannel & 0x7;
        adc_pattern[i].unit = k_adcUnit;
        adc_pattern[i].bit_width = k_adcWidth;

        ESP_LOGI(TAG, "adc_pattern[%d].atten is :%"PRIx8, i, adc_pattern[i].atten);
        ESP_LOGI(TAG, "adc_pattern[%d].channel is :%"PRIx8, i, adc_pattern[i].channel);
        ESP_LOGI(TAG, "adc_pattern[%d].unit is :%"PRIx8, i, adc_pattern[i].unit);
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(m_handle, &dig_cfg));
    ESP_ERROR_CHECK(adc_continuous_start(m_handle));

    m_state = FastADCState::k_adcStarted;
    return m_state;
}
FastADCState FastADC::StopADCSampling()
{
    if(m_state != FastADCState::k_adcStarted)
    {
        m_state = FastADCState::k_failedBadState;
        return m_state;
    }

    esp_err_t err = adc_continuous_stop(m_handle);
    if(err != ESP_OK)
    {
        m_state = FastADCState::k_stopFailedAdcDisable;
        return m_state;
    } 

    m_state = FastADCState::k_initializedAdcStopped;
    return m_state;
}

size_t FastADC::ReadAndPrintSamples()
{
    esp_err_t ret;

    static uint16_t buf[k_dmaBufLenSamples];
    uint8_t* bufAsBytes = (uint8_t*)buf;
    int samples = 0;
    uint32_t bytes_read = 0;

    ret = adc_continuous_read(m_handle, bufAsBytes, k_dmaBufLenSamples * sizeof(uint16_t), &bytes_read, k_dmaReadTimeoutMs);
    if (ret == ESP_OK) {
        ESP_LOGI("TASK", "ret is %x, ret_num is %"PRIu32" bytes", ret, bytes_read);
        for (int i = 0; i < bytes_read; i += SOC_ADC_DIGI_RESULT_BYTES) {
            adc_digi_output_data_t *p = (adc_digi_output_data_t*)&bufAsBytes[i];
            uint32_t chan_num = p->type1.channel;
            uint32_t data = p->type1.data;
            /* Check the channel number validation, the data is invalid if the channel num exceed the maximum channel */
            if (chan_num < SOC_ADC_CHANNEL_NUM(k_adcUnit)) {
                ESP_LOGI(TAG, "Unit: %s, Channel: %"PRIu32", Value: %"PRIx32, unit, chan_num, data);
            } else {
                ESP_LOGW(TAG, "Invalid data [%s_%"PRIu32"_%"PRIx32"]", unit, chan_num, data);
            }
        }
    }
    return bytes_read;
}
size_t FastADC::ReadSamplesBlockingTo(uint16_t* outBuf, size_t bufSizeBytes)
{
    uint8_t* bufAsBytes = (uint8_t*)outBuf;
    uint32_t bytes_read = 0;

    esp_err_t ret = adc_continuous_read(m_handle, bufAsBytes, bufSizeBytes, &bytes_read, k_dmaReadTimeoutMs);
    if (ret == ESP_OK) {
        ESP_LOGI("TASK", "ret is %x, ret_num is %"PRIu32" bytes", ret, bytes_read);
        for (int i = 0; i < bytes_read; i += SOC_ADC_DIGI_RESULT_BYTES) {
            adc_digi_output_data_t *p = (adc_digi_output_data_t*)&bufAsBytes[i];
            uint32_t chan_num = p->type1.channel;
            uint32_t data = p->type1.data;
            /* Check the channel number validation, the data is invalid if the channel num exceed the maximum channel */
            if (chan_num < SOC_ADC_CHANNEL_NUM(k_adcUnit)) {
                ESP_LOGI(TAG, "Unit: %s, Channel: %"PRIu32", Value: %"PRIx32, unit, chan_num, data);
            } else {
                ESP_LOGW(TAG, "Invalid data [%s_%"PRIu32"_%"PRIx32"]", unit, chan_num, data);
            }
        }
    }
    return bytes_read;
}

#endif // USE_FAST_ADC_CONTINUOUS