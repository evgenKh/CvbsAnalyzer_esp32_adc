#include "FastADC.h"
#include "CvbsAnalyzerGlobals.h"
#include "esp_arduino_version.h"

#if !USE_FAST_ADC_CONTINUOUS

#include <map>
#include "hal/adc_ll.h"
#include "hal/adc_types.h"

#include "soc/i2s_struct.h"
#include "driver/i2s.h"
#include "hal/i2s_types.h"

#include "hal/i2s_ll.h"

//#include "hal/adc_hal_conf.h"

#ifndef ESP_ARDUINO_VERSION_MAJOR
    #error "Can't find esp32 core version, maybe esp_arduino_version.h not included"
#endif

#if (ESP_ARDUINO_VERSION_MAJOR <= 2 && ESP_ARDUINO_VERSION_PATCH <= 17)
    //2.0.17 and older
    static constexpr adc_unit_t k_adcUnit = ADC_UNIT_1;// =1 in pltformio, =0 i arduinoIDE!
    static constexpr adc_ll_num_t k_adcLowLevelUnit = ADC_NUM_1;// =0 in pltformio, in arduinoIDE use adc_unit_t instead adc_ll_num_t!
#else 
    //Newer
    static constexpr adc_unit_t k_adcUnit = ADC_UNIT_1;// =1 in pltformio, =0 i arduinoIDE!
    static constexpr adc_unit_t k_adcLowLevelUnit = ADC_UNIT_1;// =0 in pltformio, in arduinoIDE use adc_unit_t instead adc_ll_num_t!    
#endif
    static constexpr i2s_port_t k_i2sPort = I2S_NUM_0; 

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


#define ADC_CONVERT_LIMIT_DISABLE do{ SYSCON.saradc_ctrl2.meas_num_limit=0; }while(false)
#define ADC_CONVERT_LIMIT_ENABLE do{ SYSCON.saradc_ctrl2.meas_num_limit=1; }while(false)


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
        //.mclk_multiple = I2S_MCLK_MULTIPLE_DEFAULT,//mclk = sample_rate * 256
        //.bits_per_chan = I2S_BITS_PER_CHAN_DEFAULT//equals to data bit-width
    };

    esp_err_t err = i2s_driver_install(k_i2sPort, &i2s_conf, k_i2sEventQueueSize, (k_i2sEventQueueSize ? &m_i2sEventQueue : nullptr));
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
    esp_err_t err = i2s_driver_uninstall(k_i2sPort);
    //if(err == ESP_OK)
    {
        m_state = FastADCState::k_notInitialized;
    }
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
    
    adc1_channel_t adcChannel = k_gpioToAdc1Channel.at(gpioPin);
    
    m_adcPreviousDataInvertEnabled = SYSCON.saradc_ctrl2.sar1_inv;//Save previous value asap

    SetClkDiv(5, 1, 0);//Doing this to set'a' to 1 early.

    i2s_ll_rx_force_enable_fifo_mod(&I2S0, false);//Was true on old versions of Arduino, but false on newer ones.
    adc_ll_digi_set_data_source(false);//initial value of .data_to_i2s was 0 in pltformio, but becomes 1 later anyway.

    if(k_printRegisters)
    {
        CVBS_ANALYZER_LOG_INFO("registers right before i2s_set_adc_mode:\n");
        PrintADCRegisters();
    }

    esp_err_t err = i2s_set_adc_mode(k_adcUnit, adcChannel);
    if(err != ESP_OK)
    {
        m_state = FastADCState::k_startFailedSetAdcMode;
        return m_state;
    }
    
    err = i2s_zero_dma_buffer(k_i2sPort);
    if(err != ESP_OK)
    {
        m_state = FastADCState::k_startFailedZeroDma;
        return m_state;
    }

    ADC_CONVERT_LIMIT_DISABLE;
    adc_ll_digi_set_convert_limit_num(k_adcMeasNum);//Maybe not needed idk

    if(k_printRegisters)
    {
        CVBS_ANALYZER_LOG_INFO("registers right before i2s_adc_enable:\n");
        PrintADCRegisters();
    }
    err = i2s_adc_enable(k_i2sPort);
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
    ADC_CONVERT_LIMIT_DISABLE;
    adc_ll_digi_set_convert_limit_num(k_adcMeasNum);//Maybe not needed idk
    
    //adc_ll_digi_output_invert(k_adcLowLevelUnit, !invertData); // Inverted is actually non inverted haha
    SYSCON.saradc_ctrl2.sar1_inv = !invertData;
    

    // ADC setting
    adc_digi_pattern_config_t adcDigiPattern{
        .atten = k_adcAttenuation,
        .channel = adcChannel,
        .unit = k_adcUnit,// 0 or 1 ??????????????
        .bit_width = k_adcWidthBits,
    };

    adc_ll_digi_clear_pattern_table(k_adcLowLevelUnit);
    adc_ll_digi_set_pattern_table_len(k_adcLowLevelUnit, 1);
    adc_ll_digi_set_pattern_table(k_adcLowLevelUnit, adcChannel, adcDigiPattern);
    
    
    // reduce sample time  
    adc_set_clk_div(k_adcAPBClockDiv);

    adc_ll_set_sample_cycle(k_adcSampleCycle);   

    // sampling rate 2Msps setting
    // see https://esp32.com/viewtopic.php?t=1988

    
    SetClkDiv(k_i2sMclkDiv, 1, 0);// I2S module clock devider, Fmclk = Fsclk /(mclk_div+b/a)
    
    i2s_ll_rx_set_bck_div_num(&I2S0, k_i2sRxBckDiv);//Bit clock configuration bits. if data_bits == 8 { 2 } else { 1 };
    if(k_printRegisters)
    {
        CVBS_ANALYZER_LOG_INFO("registers right before i2s_zero_dma_buffer:\n");
        PrintADCRegisters();
    }

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
    //esp_err_t err = i2s_zero_dma_buffer(k_i2sPort);
    //if(err != ESP_OK)
    //{
    //    m_state = FastADCState::k_startFailedZeroDma;
    //}

    static uint8_t s_dummyBuf[k_dmaDrainDummyBufSizeBytes];

    constexpr static size_t k_readCyclesNonBlocking = 
    ((k_dmaBufsCount * k_dmaBufLenSamples * sizeof(uint16_t)) + (k_dmaDrainDummyBufSizeBytes - 1))
     / k_dmaDrainDummyBufSizeBytes;

    constexpr static size_t k_readCyclesBlocking = 2 * k_readCyclesNonBlocking / k_dmaBufsCount;

    for(int i = 0; i < k_readCyclesNonBlocking; i++)
    {
        //Drain all samples that could be read prior to settings update
        //Non-blocking!
        uint32_t bytes_read = 0;
        esp_err_t err = i2s_read(k_i2sPort, s_dummyBuf, k_dmaDrainDummyBufSizeBytes, &bytes_read, 0);
        if (err != ESP_OK || bytes_read == 0){
            break;
        }
    }

    //I dont know why 1 blocking read is not enough after all non-blocking ones.
    //So reading all +1 that could been in flight between ADc and DMA, all blocking.
    for(int i = 0; i < k_readCyclesBlocking; i++)
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

    esp_err_t err = i2s_adc_disable(k_i2sPort);
    if(err != ESP_OK)
    {
        m_state = FastADCState::k_stopFailedAdcDisable;
        return m_state;
    } 

    //restore settings
    //adc_ll_digi_output_invert(k_adcLowLevelUnit, m_adcPreviousDataInvertEnabled); // Enable data invert for ADC1
    SYSCON.saradc_ctrl2.sar1_inv = m_adcPreviousDataInvertEnabled;

    adc_ll_digi_clear_pattern_table(k_adcLowLevelUnit);

    ADC_CONVERT_LIMIT_ENABLE;

    m_state = FastADCState::k_initializedAdcStopped;
    return m_state;
}


size_t FastADC::ReadSamplesBlockingTo(uint16_t *outBuf, size_t bufSizeBytes)
{
    size_t bytes_read;
    if (i2s_read(k_i2sPort, outBuf, bufSizeBytes, &bytes_read, k_dmaReadTimeout) != ESP_OK) {
        //Unlinke continuous read, here error is real error, not some timeouts?
        CVBS_ANALYZER_LOG_ERROR("i2s_read() failed");
        m_state = FastADCState::k_i2sReadFailed;
    }
    return bytes_read;
}

void FastADC::PrintADCRegisters()
{
    
    CVBS_ANALYZER_LOG_INFO("SYSCON.saradc_ctrl.start_force=%d,\n\
        SAYSCON.saradc_ctrl.start=%d,\n\
        SYSCON.saradc_ctrl.sar2_mux=%d,\n\
        SYSCON.saradc_ctrl.work_mode=%d,\n\
        SYSCON.saradc_ctrl.sar_sel=%d,\n\
        SYSCON.saradc_ctrl.sar_clk_gated=%d,\n\
        SYSCON.saradc_ctrl.sar_clk_div=%d,\n\
        SYSCON.saradc_ctrl.sar1_patt_len=%d,\n\
        SYSCON.saradc_ctrl.sar2_patt_len=%d,\n\
        SYSCON.saradc_ctrl.sar1_patt_p_clear=%d,\n\
        SYSCON.saradc_ctrl.sar2_patt_p_clear=%d,\n\
        SYSCON.saradc_ctrl.data_sar_sel=%d,\n\
        SYSCON.saradc_ctrl.data_to_i2s=%d,\n\
        SYSCON.saradc_ctrl.reserved27=%d\n",
        SYSCON.saradc_ctrl.start_force,
        SYSCON.saradc_ctrl.start,
        SYSCON.saradc_ctrl.sar2_mux,
        SYSCON.saradc_ctrl.work_mode,
        SYSCON.saradc_ctrl.sar_sel,
        SYSCON.saradc_ctrl.sar_clk_gated,
        SYSCON.saradc_ctrl.sar_clk_div,
        SYSCON.saradc_ctrl.sar1_patt_len,
        SYSCON.saradc_ctrl.sar2_patt_len,
        SYSCON.saradc_ctrl.sar1_patt_p_clear,
        SYSCON.saradc_ctrl.sar2_patt_p_clear,
        SYSCON.saradc_ctrl.data_sar_sel,
        SYSCON.saradc_ctrl.data_to_i2s,
        SYSCON.saradc_ctrl.reserved27);

    CVBS_ANALYZER_LOG_INFO("SYSCON.saradc_fsm.rstb_wait=%d,\n\
         SYSCON.saradc_fsm.standby_wait=%d,\n\
          SYSCON.saradc_fsm.start_wait=%d,\n\
          SYSCON.saradc_fsm.sample_cycle=%d\n",
        SYSCON.saradc_fsm.rstb_wait,
        SYSCON.saradc_fsm.standby_wait,
        SYSCON.saradc_fsm.start_wait,
        SYSCON.saradc_fsm.sample_cycle);

    CVBS_ANALYZER_LOG_INFO("SYSCON.saradc_ctrl2.meas_num_limit=%d,\n\
        SYSCON.saradc_ctrl2.max_meas_num=%d,\n\
        SYSCON.saradc_ctrl2.sar1_inv=%d,\n\
        SYSCON.saradc_ctrl2.sar2_inv=%d,\n\
        SYSCON.saradc_ctrl2.reserved11=%d\n",
        SYSCON.saradc_ctrl2.meas_num_limit,
        SYSCON.saradc_ctrl2.max_meas_num,
        SYSCON.saradc_ctrl2.sar1_inv,
        SYSCON.saradc_ctrl2.sar2_inv,
        SYSCON.saradc_ctrl2.reserved11);

    CVBS_ANALYZER_LOG_INFO("I2S0.clkm_conf.clkm_div_num=%d,\n\
        I2S0.clkm_conf.clkm_div_b=%d,\n\
        I2S0.clkm_conf.clkm_div_a=%d\n",
        I2S0.clkm_conf.clkm_div_num,
        I2S0.clkm_conf.clkm_div_b,
        I2S0.clkm_conf.clkm_div_a);
        
    CVBS_ANALYZER_LOG_INFO("I2S0.sample_rate_conf.tx_bck_div_num=%d,\n\
        I2S0.sample_rate_conf.rx_bck_div_num=%d,\n\
        I2S0.sample_rate_conf.tx_bits_mod=%d,\n\
        I2S0.sample_rate_conf.rx_bits_mod=%d\n",
        I2S0.sample_rate_conf.tx_bck_div_num,
        I2S0.sample_rate_conf.rx_bck_div_num,
        I2S0.sample_rate_conf.tx_bits_mod,
        I2S0.sample_rate_conf.rx_bits_mod);

    CVBS_ANALYZER_LOG_INFO("I2S0.conf.rx_start=%d,\n\
        I2S0.conf.tx_start=%d,\n\
        I2S0.conf.rx_reset=%d,\n\
        I2S0.conf.tx_reset=%d,\n\
        I2S0.conf.rx_slave_mod=%d,\n\
        ",
        I2S0.conf.rx_start,
        I2S0.conf.tx_start,
        I2S0.conf.rx_reset,
        I2S0.conf.tx_reset,
        I2S0.conf.rx_slave_mod);
        
    CVBS_ANALYZER_LOG_INFO("I2S0.fifo_conf.rx_data_num=%d,\n\
        I2S0.fifo_conf.tx_data_num=%d,\n\
        I2S0.fifo_conf.dscr_en=%d,\n\
        I2S0.fifo_conf.tx_fifo_mod=%d,\n\
        I2S0.fifo_conf.rx_fifo_mod=%d,\n\
        I2S0.fifo_conf.tx_fifo_mod_force_en=%d\n\
        I2S0.fifo_conf.rx_fifo_mod_force_en=%d\n",
        I2S0.fifo_conf.rx_data_num,
        I2S0.fifo_conf.tx_data_num,
        I2S0.fifo_conf.dscr_en,
        I2S0.fifo_conf.tx_fifo_mod,
        I2S0.fifo_conf.rx_fifo_mod,
        I2S0.fifo_conf.tx_fifo_mod_force_en,
        I2S0.fifo_conf.rx_fifo_mod_force_en);

    CVBS_ANALYZER_LOG_INFO("I2S0.conf_chan.tx_chan_mod=%d,\n\
        I2S0.conf_chan.rx_chan_mod=%d,\n",
        I2S0.conf_chan.tx_chan_mod,
        I2S0.conf_chan.rx_chan_mod);
}
#endif // !USE_FAST_ADC_CONTINUOUS
