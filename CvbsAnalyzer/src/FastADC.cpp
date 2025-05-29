#include "FastADC.h"
#include <map>
#include "hal/adc_ll.h"

#include "soc/i2s_struct.h"
#include "driver/i2s.h"
#include "hal/i2s_types.h"
#include "hal/i2s_ll.h"

//#include "hal/adc_hal_conf.h"

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
constexpr adc_ll_num_t k_adcLowLevelUnit = ADC_NUM_1;


FastADC::FastADC()
{
}

//bool FastADC::BeginADCContinuous()
//{    
//    //Need newer version of ARduino SDK in platformio, or go through adc_ll wrappers manually....
//    return false;
//}

FastADCState FastADC::Initialize()
{
    if(m_state != FastADCState::k_notInitialized)
    {
        m_state = FastADCState::k_failedBadState;
        return m_state;
    }
    m_state = FastADCState::k_initializing;

    i2s_config_t i2s_conf = {
    	.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
	    .sample_rate = k_i2sSampleRate,
	    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
	    .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
	    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
	    .intr_alloc_flags = 0,
	    .dma_buf_count = k_dmaBufsCount,
	    .dma_buf_len = k_dmaBufLenSamples,
	    .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0,//used only with apll,
        .mclk_multiple = I2S_MCLK_MULTIPLE_DEFAULT,//mclk = sample_rate * 256
        .bits_per_chan = I2S_BITS_PER_CHAN_DEFAULT//equals to data bit-width
    };

    esp_err_t err = i2s_driver_install(I2S_NUM_0, &i2s_conf, k_i2sEventQueueSize, (k_i2sEventQueueSize ? &m_i2sEventQueue : nullptr));
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
    esp_err_t err = i2s_driver_uninstall(I2S_NUM_0);
    //if(err == ESP_OK)
    {
        m_state = FastADCState::k_notInitialized;
    }
    return m_state;
}

FastADCState FastADC::StartADCSampling(int8_t gpioPin)
{
    //m_gpioPin = gpioPin;
    if(k_gpioToAdc1Channel.count(gpioPin) == 0)
    {
        m_state = FastADCState::k_startFailedBadPin;
        return m_state;
    }
    m_state = FastADCState::k_adcStarting;
    m_adcChannel = k_gpioToAdc1Channel.at(gpioPin);
    
    esp_err_t err = i2s_set_adc_mode(k_adcUnit, m_adcChannel);
    if(err != ESP_OK)
    {
        m_state = FastADCState::k_startFailedSetAdcMode;
        return m_state;
    }
    
    err = i2s_zero_dma_buffer(I2S_NUM_0);
    if(err != ESP_OK)
    {
        m_state = FastADCState::k_startFailedZeroDma;
        return m_state;
    }

    adc_ll_digi_convert_limit_disable();
    
    err = i2s_adc_enable(I2S_NUM_0);
    if(err != ESP_OK)
    {
        m_state = FastADCState::k_startFailedAdcEnable;
        return m_state;
    }

    //WARNING! i2s_adc_enable already started conversion with default params, now we change them, but dma buf will have some sampled data with default ones. 
    //TODO: try just i2s_zero_dma_buffer after we did all changes.
    //TODO 2: write custom i2s_adc_enable()

    // delay for I2S bug workaround
    //See https://github.com/espressif/esp-idf/pull/1991#issuecomment-1157404298
    //vTaskDelay(k_startAdcDelayMs / portTICK_PERIOD_MS);//10ms??

    // Enable continuous adc sampling
    //See https://github.com/espressif/esp-idf/pull/1991#issuecomment-1157404298
    adc_ll_digi_convert_limit_disable();

    // ADC setting
    adc_digi_pattern_config_t adcDigiPattern{
        .atten = k_adcAttenuation,
        .channel = m_adcChannel,
        .unit = k_adcUnit,
        .bit_width = k_adcWidth,
    };

    adc_ll_digi_clear_pattern_table(k_adcLowLevelUnit);
    adc_ll_digi_set_pattern_table_len(k_adcLowLevelUnit, 1);
    adc_ll_digi_set_pattern_table(k_adcLowLevelUnit, m_adcChannel, adcDigiPattern);
    
    // reduce sample time  
    adc_set_clk_div(k_adcAPBClockDiv);

    adc_ll_set_sample_cycle(k_adcSampleCycle);   

    // sampling rate 2Msps setting
    // see https://esp32.com/viewtopic.php?t=1988
    i2s_ll_mclk_div_t i2sLowLevelClkConfig{
        .mclk_div = k_i2sMclkDiv,// I2S module clock devider, Fmclk = Fsclk /(mclk_div+b/a)
        .a = 1,//
        .b = 0//
    };
    i2s_ll_rx_set_clk(&I2S0, &i2sLowLevelClkConfig);
    
    i2s_ll_rx_set_bck_div_num(&I2S0, k_i2sRxBckDiv);//Bit clock configuration bits. if data_bits == 8 { 2 } else { 1 };

    //Calling i2s_zero_dma_buffer to erase data that was sampled with default clock settings.
    err = i2s_zero_dma_buffer(I2S_NUM_0);
    if(err != ESP_OK)
    {
        m_state = FastADCState::k_startFailedZeroDma;
        return m_state;
    }

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

    esp_err_t err = i2s_adc_disable(I2S_NUM_0);
    if(err != ESP_OK)
    {
        m_state = FastADCState::k_stopFailedAdcDisable;
        return m_state;
    } 

    adc_ll_digi_clear_pattern_table(k_adcLowLevelUnit);

    m_state = FastADCState::k_initializedAdcStopped;
    return m_state;
}

#define TAG "I2S"

size_t FastADC::ReadSamplesBlocking()
{
    static uint16_t buf[k_dmaBufLenSamples];
    int samples = 0;
    size_t bytes_read;
    int i;

    for(int run=0;run<5;run++)
    {
        if (i2s_read(I2S_NUM_0, buf, k_dmaBufLenSamples * sizeof(uint16_t), &bytes_read, portMAX_DELAY) != ESP_OK) {
            printf("i2s_read() fail");
            continue;
        }
        //StopADCSampling();
        samples += k_dmaBufLenSamples;

        //4 run dry, print 5th
        //if (samples >= k_dmaBufLenSamples*5) 
        {
            samples = 0;
            
            // output only 256 samples
    	    for (i = 0; i < bytes_read / 2; i++) {
                printf("%d\n", buf[i] & 0x0fff);
	        }
            printf("%d samples printed.(run %d) ----------------\n", bytes_read / 2, run);
            //break;
        }
    }
    return bytes_read;
}

size_t FastADC::ReadSamplesBlockingTo(int16_t *outBuf, size_t bufSizeBytes)
{
    size_t bytes_read;
    if (i2s_read(I2S_NUM_0, outBuf, bufSizeBytes, &bytes_read, portMAX_DELAY) != ESP_OK) {
            printf("i2s_read() fail");
        }
    return bytes_read;
}
