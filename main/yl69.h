/*
 * ESP32 Soil Moisture Sensor YL-69 or HL-69 Driver
 */

#ifndef MAIN_YL69_H_
#define MAIN_YL69_H_

//#include "driver/adc.h"
//#include "driver/gpio.h"
//#include "esp_adc_cal.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/adc_types.h"
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define DEFAULT_VREF 			1100 			//Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES 			64 				//Multisampling
#define ADC_CHANNEL_YL69		ADC_CHANNEL_4
#define EXAMPLE_ADC_ATTEN       ADC_ATTEN_DB_12
#define VALUE_MAX 				4095 			// Max ADV value of soil mosture

bool moisture_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
void moisture_calibration_deinit(adc_cali_handle_t handle);

void moisture_init(adc_oneshot_unit_handle_t adc1_handle);
void moisture_deinit();

float moisture_read(adc_oneshot_unit_handle_t adc1_handle);

uint32_t moisture_normalization(uint32_t value_t);

#endif /* MAIN_YL69_H_ */
