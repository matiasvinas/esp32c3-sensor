/*
 * ESP32 Battery Level Sensor  Driver
 */

#include "adc_sensors.h"


const static char *TAG_adc_battery = "ADC for Battery sensor";
const static char *TAG_adc_moisture = "ADC for Moisture sensor";

static adc_oneshot_unit_handle_t adc1_handle;

static adc_oneshot_unit_init_cfg_t init_config1 = {
    .unit_id = ADC_UNIT_1,
};

static adc_oneshot_chan_cfg_t config = {
    .bitwidth = ADC_BITWIDTH_DEFAULT,
    .atten = EXAMPLE_ADC_ATTEN,
};

static adc_cali_handle_t adc1_cali_chan3_handle = NULL;
static adc_cali_handle_t adc1_cali_chan4_handle = NULL;

/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG_adc_battery, "calibration scheme version is %s", "Curve Fitting");
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
        ESP_LOGI(TAG_adc_battery, "calibration scheme version is %s", "Line Fitting");
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
        ESP_LOGI(TAG_adc_battery, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG_adc_battery, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG_adc_battery, "Invalid arg or no memory");
    }

    return calibrated;
}

void adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG_adc_battery, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG_adc_battery, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}

void adc_sensors_init(void)
{
	//-------------New Unit for ADC1---------------//
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

	//-------------ADC1 Config---------------//
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_BATTERY, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_YL69, &config));

    //-------------ADC1 Calibration Init---------------//
    adc_calibration_init(ADC_UNIT_1, ADC_CHANNEL_BATTERY, EXAMPLE_ADC_ATTEN, &adc1_cali_chan3_handle);
	adc_calibration_init(ADC_UNIT_1, ADC_CHANNEL_YL69, EXAMPLE_ADC_ATTEN, &adc1_cali_chan4_handle);
}

void adc_sensors_deinit(void)
{
	ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
	adc_calibration_deinit(adc1_cali_chan3_handle);
	adc_calibration_deinit(adc1_cali_chan4_handle);
}

uint32_t adc_battery_read(void)
{
	int adc_one_shot = 0;
	int voltage = 0;
	int adc_value = 0;

	//Multisampling
	for (int i = 0; i < NO_OF_SAMPLES; i++) {
		ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_BATTERY, &adc_one_shot));
		adc_value += adc_one_shot;
	}
	
	adc_value /= NO_OF_SAMPLES;

	ESP_LOGI(TAG_adc_battery, "ADC%d Channel[%d] Battery Raw Data: %d", ADC_UNIT_1 + 1, ADC_CHANNEL_BATTERY, adc_value);
	ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan3_handle, adc_value, &voltage));
    ESP_LOGI(TAG_adc_battery, "ADC%d Channel[%d] Battery Voltage: %d mV", ADC_UNIT_1 + 1, ADC_CHANNEL_BATTERY, voltage);
    
    return voltage;
}

float adc_yl69_read()
{
	int adc_one_shot = 0;
	int adc_value = 0;
	float moisture = 0;
	
	//Multisampling
	for (int i = 0; i < NO_OF_SAMPLES; i++) {
		ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_YL69, &adc_one_shot));
		adc_value += adc_one_shot;
	}
	
	adc_value /= NO_OF_SAMPLES;	
	
	ESP_LOGI(TAG_adc_moisture, "ADC%d Channel[%d] Moisture Raw Data: %d", ADC_UNIT_1 + 1, ADC_CHANNEL_YL69, adc_value);
	
	moisture = (float) ( YL69_VALUE_MAX - adc_value ) * 0.0244; // 100/4095 = 0.0244
	
	ESP_LOGI(TAG_adc_moisture, "ADC%d Channel[%d] Moisture: %f ", ADC_UNIT_1 + 1, ADC_CHANNEL_YL69, moisture);
	
    return  moisture;
}
