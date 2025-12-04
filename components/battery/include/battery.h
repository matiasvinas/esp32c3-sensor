/*
 * adc_sensors.h
 *
 *  Created on: 20 abr. 2025
 *      Author: matia
 */

#ifndef MAIN_BATTERY_H_
#define MAIN_BATTERY_H_

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

#define DEFAULT_VREF 				1100 			//Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES 				64 				//Multisampling
#define ADC_CHANNEL_BATTERY			ADC_CHANNEL_3
#define EXAMPLE_ADC_ATTEN           ADC_ATTEN_DB_12
#define YL69_VALUE_MAX 				4095 			// Max ADV value of soil mosture

bool battery_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
void battery_calibration_deinit(adc_cali_handle_t handle);

void battery_init(adc_oneshot_unit_handle_t adc1_handle);
void battery_deinit();

uint32_t battery_read(adc_oneshot_unit_handle_t adc1_handle);

#endif /* MAIN_BATTERY_H_ */
