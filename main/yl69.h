/*
 * ESP32 Soil Moisture Sensor YL-69 or HL-69 Driver
 */

#ifndef MAIN_YL69_H_
#define MAIN_YL69_H_

#include "freertos/FreeRTOS.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "esp_adc_cal.h"
#include <stdio.h>
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

#define DEFAULT_VREF 1100 //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES 64 //Multisampling

#define VALUE_MAX 4095 // Max ADV value of soil mosture

void yl69_init(adc_channel_t _channel);
//uint32_t yl69_read();
uint32_t yl69_normalization(uint32_t value_t);

#endif /* MAIN_YL69_H_ */
