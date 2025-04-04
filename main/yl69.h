/*
 * yl69.h
 *
 *  Created on: 4 mar. 2025
 *      Author: matia
 */

#ifndef MAIN_YL69_H_
#define MAIN_YL69_H_

/*
 * ESP32 Soil Moisture Sensor YL-69 or HL-69 Driver
 * Copyright 2021, Lorenzo Carnevale <lcarnevale@unime.it>
 */

#include "freertos/FreeRTOS.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "esp_adc_cal.h"

#define DEFAULT_VREF 1100 //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES 64 //Multisampling

#define VALUE_MAX 4095 // Max ADV value of soil mosture

void yl69_setup(adc1_channel_t _channel);
uint32_t yl69_read();
uint32_t yl69_normalization(uint32_t value_t);

#endif /* MAIN_YL69_H_ */
