/*
 * ESP32 Temperature Sensor ds18b20
 * using esp managed component ds18b20
 */

#ifndef DS18B20_CUSTOM_H_
#define DS18B20_CUSTOM_H_

#include "ds18b20.h"
#include "onewire_bus.h"
#include "esp_log.h"

#define ONEWIRE_BUS_GPIO    8
#define ONEWIRE_MAX_DS18B20 2

/* func declaratins */
void ds18b20_init(void);

void ds18b20_deinit(void);

float ds18b20_read(void);

void float_to_uint(float var1, uint8_t * var);

#endif /* DS18B20_CUSTOM_H_ */
