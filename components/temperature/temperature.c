/*
 * ESP32 Temperature Sensor ds18b20
 * using esp managed component ds18b20
 */

#include "temperature.h"

static int s_ds18b20_device_num = 0;
static ds18b20_device_handle_t s_ds18b20s[ONEWIRE_MAX_DS18B20];
static onewire_bus_handle_t bus = NULL;

void ds18b20_init(void)
{
    // install 1-wire bus
    onewire_bus_config_t bus_config = {
        .bus_gpio_num = ONEWIRE_BUS_GPIO,
    };
    onewire_bus_rmt_config_t rmt_config = {
        .max_rx_bytes = 10, // 1byte ROM command + 8byte ROM number + 1byte device command
    };
    ESP_ERROR_CHECK(onewire_new_bus_rmt(&bus_config, &rmt_config, &bus));

    onewire_device_iter_handle_t iter = NULL;
    onewire_device_t next_onewire_device;
    esp_err_t search_result = ESP_OK;

    // create 1-wire device iterator, which is used for device search
    ESP_ERROR_CHECK(onewire_new_device_iter(bus, &iter));
    ESP_LOGI("DS18B20", "Device iterator created, start searching...");
    do {
        search_result = onewire_device_iter_get_next(iter, &next_onewire_device);
        if (search_result == ESP_OK) { // found a new device, let's check if we can upgrade it to a DS18B20
            ds18b20_config_t ds_cfg = {};
            // check if the device is a DS18B20, if so, return the ds18b20 handle
            if (ds18b20_new_device(&next_onewire_device, &ds_cfg, &s_ds18b20s[s_ds18b20_device_num]) == ESP_OK) {
                ESP_LOGI("DS18B20", "Found a DS18B20[%d], address: %016llX", s_ds18b20_device_num, next_onewire_device.address);
                s_ds18b20_device_num++;
            } else {
                ESP_LOGI("DS18B20", "Found an unknown device, address: %016llX", next_onewire_device.address);
            }
        }
    } while (search_result != ESP_ERR_NOT_FOUND);
    ESP_ERROR_CHECK(onewire_del_device_iter(iter));
    ESP_LOGI("DS18B20", "Searching done, %d DS18B20 device(s) found", s_ds18b20_device_num);
}

void ds18b20_deinit(void)
{
	ESP_LOGI("DS18B20", "Deinitializing ds18b20 sensor");
	//delete ds18b20 device handles
	for (int i = 0; i < s_ds18b20_device_num; i++) 
	{
    	ESP_ERROR_CHECK(ds18b20_del_device(s_ds18b20s[i]));
	}
	s_ds18b20_device_num = 0;
	
	//delete 1-wire bus
	ESP_ERROR_CHECK(onewire_bus_del(bus));
}

float ds18b20_read(void)
{
	float s_temperature = 0.0;
		
    for (int i = 0; i < s_ds18b20_device_num; i ++) {
        ESP_ERROR_CHECK(ds18b20_trigger_temperature_conversion(s_ds18b20s[i]));
        ESP_ERROR_CHECK(ds18b20_get_temperature(s_ds18b20s[i], &s_temperature));
        ESP_LOGI("DS18B20", "temperature read from DS18B20[%d]: %.2fC", i, s_temperature);
    }
    return s_temperature;

}

void float_to_uint(float var1, uint8_t * var)
{
	var[0] = (uint8_t)var1;
	var[1] = (uint8_t)((var1 - var[0]) * 100);
}