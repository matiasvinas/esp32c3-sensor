/*
 * ESP32 Soil Moisture Sensor YL-69 or HL-69 Driver
 */

#include "moisture.h"


const static char *ADC_TAG = "ADC_YL69";

//static adc_oneshot_unit_handle_t adc1_handle;

//static adc_oneshot_unit_init_cfg_t init_config1 = {
//    .unit_id = ADC_UNIT_2,
//};

static adc_oneshot_chan_cfg_t config = {
    .bitwidth = ADC_BITWIDTH_DEFAULT,
    .atten = EXAMPLE_ADC_ATTEN,
};

static adc_cali_handle_t adc1_cali_chan4_handle = NULL;

bool moisture_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
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
        ESP_LOGI(ADC_TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(ADC_TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(ADC_TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

void moisture_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(ADC_TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(ADC_TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}

void moisture_init(adc_oneshot_unit_handle_t adc1_handle) {
	//-------------ADC1 Config---------------//
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_YL69, &config));

    //-------------ADC1 Calibration Init---------------//
    moisture_calibration_init(ADC_UNIT_1, ADC_CHANNEL_YL69, EXAMPLE_ADC_ATTEN, &adc1_cali_chan4_handle);
}

void moisture_deinit()
{
	moisture_calibration_deinit(adc1_cali_chan4_handle);
}

float moisture_read(adc_oneshot_unit_handle_t adc1_handle)
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
	
	//Obtain voltage and convert to V
	int voltage_mv = 0;
	ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan4_handle, adc_value, &voltage_mv));
	ESP_LOGI(ADC_TAG, "ADC%d Channel[%d] Voltage: %d mV", ADC_UNIT_1 + 1, ADC_CHANNEL_YL69, voltage_mv);
	
	//Calculate moisture
	float voltage = ( (float) voltage_mv ) * 0.001;
	moisture = -27.654*pow(voltage,3) + 177.63*pow(voltage,2) - 388.04*voltage + 308.98; 
	moisture = roundf(moisture * 100) * 0.01;
	if(moisture > 100) {moisture = 99.99; };
	if(moisture < 0) {moisture = 0; };
	ESP_LOGI(ADC_TAG, "ADC%d Channel[%d] Moisture: %.2f ", ADC_UNIT_1 + 1, ADC_CHANNEL_YL69, moisture);
	
    return  moisture;
}

uint32_t moisture_normalization(uint32_t value_t) {
    return (value_t * 100) / VALUE_MAX;
}
