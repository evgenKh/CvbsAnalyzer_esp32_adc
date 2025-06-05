
#include "CvbsAnalyzerGlobals.h"

#include "FastADC.h"

#if USE_FAST_ADC_CONTINUOUS
#include "driver/adc.h"
#include "driver/i2s.h"
#include "hal/adc_hal.h"
#include "esp32-hal-adc.h"
#include "esp_adc/adc_continuous.h"
#include <map>

#define ADC_CONVERT_LIMIT_DISABLE do{ SYSCON.saradc_ctrl2.meas_num_limit=0; }while(false)
#define ADC_CONVERT_LIMIT_ENABLE do{ SYSCON.saradc_ctrl2.meas_num_limit=1; }while(false)

static constexpr adc_unit_t k_adcUnit = ADC_UNIT_1;//esp32 suppors only ADC_UNIT_1
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
    adc_continuous_deinit(m_handle);
    return m_state;
}


void FastADC::SetClkDiv(uint16_t integer, uint16_t denominator, uint16_t numerator)
{
#if (ESP_ARDUINO_VERSION_MAJOR <= 2 && ESP_ARDUINO_VERSION_PATCH <= 17)
    //2.0.17        
    i2s_ll_mclk_div_t i2sLowLevelClkConfig{ .mclk_div = integer, .a = denominator, .b = numerator };
    i2s_ll_rx_set_clk(&I2S0, &i2sLowLevelClkConfig);//Fmclk = Fsclk /(mclk_div+b/a)

#elif (ESP_ARDUINO_VERSION_MAJOR <= 3 && ESP_ARDUINO_VERSION_MINOR <= 0 && ESP_ARDUINO_VERSION_PATCH <= 0)
    //3.0.0
    i2s_ll_mclk_div_t i2sLowLevelClkConfig{ .mclk_div = integer, .a = denominator, .b = numerator };
    i2s_ll_rx_set_mclk(&I2S0, set);
#else 
    // newer versions
    i2s_ll_set_raw_mclk_div(&I2S0, integer, denominator, numerator);
#endif 
}


FastADCState FastADC::StartADCSampling(int8_t gpioPin, bool invertData)
{

    if(k_gpioToAdc1Channel.count(gpioPin) == 0)
    {
        m_state = FastADCState::k_startFailedBadPin;
        return m_state;
    }
    m_state = FastADCState::k_adcStarting;
    adc1_channel_t adcChannel = k_gpioToAdc1Channel.at(gpioPin);
    
    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    
    
    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = k_i2sSampleRate,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };

    dig_cfg.pattern_num = 1;

    for (int i = 0; i < 1; i++) {
        adc_pattern[i].atten = k_adcAttenuation;
        adc_pattern[i].channel = adcChannel & 0x7;
        adc_pattern[i].unit = k_adcUnit;
        adc_pattern[i].bit_width = k_adcWidthBits;
    }
    dig_cfg.adc_pattern = adc_pattern;
    esp_err_t err = adc_continuous_config(m_handle, &dig_cfg);
    if(err != ESP_OK)
    {
        m_state = FastADCState::k_startFailedConfig;
        return m_state;
    }
    //ADC_CONVERT_LIMIT_DISABLE;

    adc_set_data_inv(k_adcUnit, invertData);
    adc_set_clk_div(2);
    adc_ll_set_sample_cycle(k_adcSampleCycle);   
    SetClkDiv(k_i2sMclkDiv, 1, 0);// I2S module clock devider, Fmclk = Fsclk /(mclk_div+b/a)
    i2s_ll_rx_set_bck_div_num(&I2S0, k_i2sRxBckDiv);//Bit clock configuration bits. if data_bits == 8 { 2 } else { 1 };
    adc_ll_digi_set_convert_limit_num(255);//Maybe not needed idk


    err = adc_continuous_start(m_handle);
    if(err != ESP_OK)
    {
        m_state = FastADCState::k_startFailedContinuousStart;
        return m_state;
    }
    
    adc_set_clk_div(2);
    adc_ll_set_sample_cycle(k_adcSampleCycle);   
    SetClkDiv(k_i2sMclkDiv, 1, 0);// I2S module clock devider, Fmclk = Fsclk /(mclk_div+b/a)
    i2s_ll_rx_set_bck_div_num(&I2S0, k_i2sRxBckDiv);//Bit clock configuration bits. if data_bits == 8 { 2 } else { 1 };
    adc_ll_digi_set_convert_limit_num(255);//Maybe not needed idk


    //Settings done.
    DrainDMA(); //Erase data that was sampled with default clock settings.
    
    if(!IsInErrorState())
    {
        m_state = FastADCState::k_adcStarted;
    }
    return m_state;
}

void FastADC::DrainDMA()
{
    //esp_err_t err = adc_continuous_flush_pool(m_handle);
    constexpr static size_t k_readCyclesNonBlocking = 
                                ((k_dmaBufsCount * k_dmaBufLenSamples * sizeof(uint16_t)) + (k_dmaDrainDummyBufSizeBytes - 1))
                                / k_dmaDrainDummyBufSizeBytes;

    constexpr static size_t k_readCyclesBlocking = 2 * k_readCyclesNonBlocking / k_dmaBufsCount;

    static uint8_t s_dummyBuf[k_dmaDrainDummyBufSizeBytes];
    for(int i = 0; i < k_readCyclesNonBlocking; i++)
    {
        //Drain all samples that could be read prior to settings update
        uint32_t bytes_read = 0;
        esp_err_t ret = adc_continuous_read(m_handle, s_dummyBuf, k_dmaDrainDummyBufSizeBytes, &bytes_read, 0);
        if (ret != ESP_OK || bytes_read == 0){
            break;
        }
    }

    //I dont know why 1 blocking read is not enough after all non-blocking ones.
    //So reading all +1 that could been in flight between ADc and DMA, all blocking.
    for(int i = 0; i <  k_readCyclesBlocking; i++)
    {
        size_t bytes_read = ReadSamplesBlockingTo((uint16_t*)s_dummyBuf, k_dmaDrainDummyBufSizeBytes);
        //CVBS_ANALYZER_LOG_INFO("bytes_read in blocking read #%d in DrainDMA(): %d\n", i, bytes_read);
    }
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

size_t FastADC::ReadSamplesBlockingTo(uint16_t* outBuf, size_t bufSizeBytes)
{
    uint8_t* bufAsBytes = (uint8_t*)outBuf;
    uint32_t bytes_read = 0;

    esp_err_t err = adc_continuous_read(m_handle, bufAsBytes, bufSizeBytes, &bytes_read, k_dmaReadTimeoutMs);
    if (err == ESP_OK) {
        for (int i = 0; i < bytes_read; i += SOC_ADC_DIGI_RESULT_BYTES) {
            adc_digi_output_data_t *p = (adc_digi_output_data_t*)&bufAsBytes[i];
            uint32_t chan_num = p->type1.channel;
            uint32_t data = p->type1.data;
            /* Check the channel number validation, the data is invalid if the channel num exceed the maximum channel */
            if (chan_num >= SOC_ADC_CHANNEL_NUM(k_adcUnit)) {
                CVBS_ANALYZER_LOG_ERROR("Invalid ADC channel %d, data %x\n", (int)chan_num, (int)data);
                m_state = FastADCState::k_continuousReadFailedInvalidData;
            }
        }
    }
    else
    {
        CVBS_ANALYZER_LOG_DEBUG("adc_continuous_read returned %d (%s)\n", (int)err, esp_err_to_name(err));
        //It's ok
        //m_state = FastADCState::k_continuousReadFailed;
    }
    return bytes_read;
}

#endif // USE_FAST_ADC_CONTINUOUS