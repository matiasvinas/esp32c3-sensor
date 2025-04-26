/*
 * ESP32 Soil Moisture Sensor YL-69 or HL-69 Driver
 */

#include "yl69.h"
#define EXAMPLE_ADC1_CHAN4			ADC_CHANNEL_4
#define EXAMPLE_ADC_ATTEN           ADC_ATTEN_DB_12

const static char *ADC_TAG = "ADC_YL69";


//static adc1_channel_t channel;
//static esp_adc_cal_characteristics_t *adc_chars;

//static const adc_unit_t unit = ADC_UNIT_1;
//static const adc_atten_t attenuation = ADC_ATTEN_DB_12;

static adc_oneshot_unit_handle_t adc1_handle;

static adc_oneshot_unit_init_cfg_t init_config1 = {
    .unit_id = ADC_UNIT_2,
};

static adc_oneshot_chan_cfg_t config = {
    .bitwidth = ADC_BITWIDTH_DEFAULT,
    .atten = EXAMPLE_ADC_ATTEN,
};

static adc_cali_handle_t adc1_cali_chan0_handle = NULL;


bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(ADC_TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(ADC_BATTERY_TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(ADC_TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(ADC_TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(ADC_TAG, "Invalid arg or no memory");
    }

    return calibrated;
}


void yl69_init(adc_channel_t channel) {
    
	//-------------ADC1 Init---------------//
    //ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

	//-------------ADC1 Config---------------//
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, channel, &config));

    //-------------ADC1 Calibration Init---------------//
    example_adc_calibration_init(ADC_UNIT_1, channel, EXAMPLE_ADC_ATTEN, &adc1_cali_chan0_handle);    

    //Configure ADC
    //adc1_config_width(ADC_WIDTH_BIT_12);
    //adc1_config_channel_atten(channel, attenuation);
    //Characterize ADC
    //adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    //esp_adc_cal_characterize(unit, attenuation, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
}













/* 
uint32_t yl69_read()
{
	int adc_raw;
	int dummy = 0;
	//int voltage;
	
	//Multisampling
	for (int i = 0; i < NO_OF_SAMPLES; i++) {
		ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN4, &adc_raw));
		dummy += adc_raw;
	}
	
	dummy /= NO_OF_SAMPLES;
	
	ESP_LOGI(ADC_TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN4, dummy);
	
	//ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_raw, &voltage));
	
    //ESP_LOGI(ADC_BATTERY_TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN3, voltage);
    
    return  dummy;

//    uint32_t adc_reading = 0;
//    //Multisampling
//    for (int i = 0; i < NO_OF_SAMPLES; i++) {
//        adc_reading += adc1_get_raw((adc1_channel_t)channel);
//    }
//    adc_reading /= NO_OF_SAMPLES;
//    //Convert adc_reading to voltage in mV
//    // uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
//    return adc_reading;
}
*/
uint32_t yl69_normalization(uint32_t value_t) {
    return (value_t * 100) / VALUE_MAX;
}
