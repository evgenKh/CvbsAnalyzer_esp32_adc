#include "FastADC.h"
#include <map>
#include "hal/adc_ll.h"
#include "driver/i2s.h"
#include "hal/i2s_hal.h"
#include "hal/i2s_types.h"
#include "hal/misc.h"
#include "soc/i2s_periph.h"
#include "soc/i2s_struct.h"
#include "hal/i2s_ll.h"

FastADC::FastADC(int gpioPin)
{
    m_gpioPin = gpioPin;
    m_adcChannel = gpioToAdcChannel(gpioPin);
}


adc1_channel_t FastADC::gpioToAdcChannel(int gpioPin) {
    static std::map<int, adc1_channel_t> gpioToChannel = {
        {36, ADC1_CHANNEL_0}, {37, ADC1_CHANNEL_1}, {38, ADC1_CHANNEL_2}, {39, ADC1_CHANNEL_3},
        {32, ADC1_CHANNEL_4}, {33, ADC1_CHANNEL_5}, {34, ADC1_CHANNEL_6}, {35, ADC1_CHANNEL_7}
    };
    if (gpioToChannel.count(gpioPin)) {
        return gpioToChannel[gpioPin];
    } else {
        Serial.println("Invalid GPIO for ADC1. Using default GPIO32.");
        return ADC1_CHANNEL_4;
    }
}


bool FastADC::BeginADCContinuous()
{    
    //Need newer version of ARduino SDK in platformio, or go through adc_ll wrappers manually....
    return false;
}

bool FastADC::BeginADCI2S()
{
    i2s_config_t i2s_conf = {
    	.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
	    .sample_rate = k_sampleRate,
	    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
	    .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
	    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
	    .intr_alloc_flags = 0,
	    .dma_buf_count = k_dmaBufsCount,
	    .dma_buf_len = k_dmaBufLenSamples,
	    .use_apll = false
    };
    Serial.print("1");
    ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM_0, &i2s_conf, 0, NULL));
    Serial.print("2");
    ESP_ERROR_CHECK(i2s_set_adc_mode(k_adcUnit, m_adcChannel));
    Serial.print("3");
    ESP_ERROR_CHECK(i2s_adc_enable(I2S_NUM_0));
    Serial.print("4");

    // delay for I2S bug workaround
    vTaskDelay(10 / portTICK_PERIOD_MS);

    // ***IMPORTANT*** enable continuous adc sampling
    //See https://github.com/espressif/esp-idf/pull/1991#issuecomment-1157404298
    SYSCON.saradc_ctrl2.meas_num_limit = 0;

    // ADC setting
    SYSCON.saradc_sar1_patt_tab[0] = ((m_adcChannel << 4) | (k_adcWidth << 2) | k_adcAttenuation) << 24;
    SYSCON.saradc_ctrl.sar1_patt_len = 0;

    
    // reduce sample time for 2Msps
    SYSCON.saradc_ctrl.sar_clk_div = 4;//2;
    SYSCON.saradc_fsm.sample_cycle = 4;//2;

    // sampling rate 2Msps setting
    I2S0.clkm_conf.clkm_div_num = 20;//This one probably most important, see https://esp32.com/viewtopic.php?t=1988
    I2S0.clkm_conf.clkm_div_b = 0;
    I2S0.clkm_conf.clkm_div_a = 1;
    I2S0.sample_rate_conf.rx_bck_div_num = 2;
    
    return true;
}

bool FastADC::BeginADCDetectedVideo()
{
    i2s_config_t i2s_conf = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
        .sample_rate = 100000,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 16,
        .dma_buf_len = 1024,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };
    Serial.print("1");
    ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM_0, &i2s_conf, 0, NULL));
    Serial.print("2");
    ESP_ERROR_CHECK(i2s_set_adc_mode(k_adcUnit, m_adcChannel));
    Serial.print("3");
    ESP_ERROR_CHECK(i2s_adc_enable(I2S_NUM_0));
    Serial.print("4");

    // delay for I2S bug workaround
    vTaskDelay(10 / portTICK_PERIOD_MS);

    // ***IMPORTANT*** enable continuous adc sampling
    //See https://github.com/espressif/esp-idf/pull/1991#issuecomment-1157404298
    SYSCON.saradc_ctrl2.meas_num_limit = 0;

    // ADC setting
    SYSCON.saradc_sar1_patt_tab[0] = ((m_adcChannel << 4) | (k_adcWidth << 2) | k_adcAttenuation) << 24;
    SYSCON.saradc_ctrl.sar1_patt_len = 0;
    return false;
}

#define TAG "I2S"

void FastADC::i2s_read_samples()
{
    static uint16_t buf[k_dmaBufLenSamples];
    int samples = 0;
    size_t bytes_read;
    int i;

    while (1) {
        if (i2s_read(I2S_NUM_0, buf, k_dmaBufLenSamples * sizeof(uint16_t), &bytes_read, portMAX_DELAY) != ESP_OK) {
            ESP_LOGW(TAG, "i2s_read() fail");
            continue;
        }

        samples += k_dmaBufLenSamples;

        // output samples every 5sec
        if (samples >= 100000 * 5) {
            samples = 0;
            
            // output only 256 samples
    	    for (i = 0; i < bytes_read / 2; i++) {
                printf("%d\n", buf[i] & 0x0fff);
	        }
            printf("%d samples printed. ----------------\n", bytes_read / 2);
        }
    }
}