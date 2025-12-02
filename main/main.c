/* 
 * Theme: Sensor Firmware - ESP32-C3-WROOM-2
 * Author: Matias Ezequiel Vinas
 * Date: February 2025 
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "esp_log.h"
#include "esp_sleep.h"
#include "host/ble_gap.h"
#include "mesh/main.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

// power management libs
#include "esp_pm.h"
#include "esp_bt.h"

// ble mesh libs
#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_sensor_model_api.h"
#include "ble_mesh_example_init.h"

#include "esp_ble_mesh_networking_api.h"


// sensors and leds
#include "ds18b20_custom.h"
#include "adc_sensors.h"
#include "led_strip.h"

#define TAG_bt "Bluetooth"
#define TAG_main "Main"
#define TAG_sensor "Sensor"

// delay time between each sensor reading in miliseconds
#define SENSOR_TASK_DELAY			30000
#define SENSOR_MAIN_DELAY			30000


#define CID_ESP     0x02E5

// Sensor Property ID
#define SENSOR_PROPERTY_ID_0        0x0056  // Temperature
#define SENSOR_PROPERTY_ID_1        0x005B  // Soil Moisture
#define SENSOR_PROPERTY_ID_2        0x005C  // Battery

// ADC channels
#define ADC_YL69_CHANNEL			ADC1_CHANNEL_4
#define ADC_BATTERY_CHANNEL			ADC1_CHANNEL_3

// CPU Freq Ranges
#define MAX_CPU_FREQ_MHZ			160
#define MIN_CPU_FREQ_MHZ			40

// Sensor Descriptor state parameters
#define SENSOR_POSITIVE_TOLERANCE   ESP_BLE_MESH_SENSOR_UNSPECIFIED_POS_TOLERANCE
#define SENSOR_NEGATIVE_TOLERANCE   ESP_BLE_MESH_SENSOR_UNSPECIFIED_NEG_TOLERANCE
#define SENSOR_SAMPLE_FUNCTION      ESP_BLE_MESH_SAMPLE_FUNC_UNSPECIFIED
#define SENSOR_MEASURE_PERIOD       ESP_BLE_MESH_SENSOR_NOT_APPL_MEASURE_PERIOD
#define SENSOR_UPDATE_INTERVAL      ESP_BLE_MESH_SENSOR_NOT_APPL_UPDATE_INTERVAL

static float init_float_var = 0;
static uint8_t init_int_var = 0;
static float s_temperature = 0.0;
static uint8_t delay_sensor_routine = 1;

static bool provisioned = false;
static bool friendship_established = false;

static esp_ble_mesh_msg_ctx_t ctx_for_gateway;
static esp_pm_lock_handle_t pm_lock;

void sensor_hum_readTask(void);
void sensor_temp_readTask(void);
void sensor_battery_readTask(void);

/* los primeros 2 bytes se usan para identificar a todos los nodos de la malla*/
/* el 3° byte se usa para distinguir cada nodo de la malla */
#define SENSOR_ID_MESH_0                    0x32    
#define SENSOR_ID_MESH_1                    0x10
#define SENSOR_ID_NODE      				0x03
static uint8_t dev_uuid[ESP_BLE_MESH_OCTET16_LEN] = { SENSOR_ID_MESH_0, SENSOR_ID_MESH_1, SENSOR_ID_NODE };

static esp_ble_mesh_cfg_srv_t config_server = {
    /* 3 transmissions with 20ms interval */
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay = ESP_BLE_MESH_RELAY_DISABLED,
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .beacon = ESP_BLE_MESH_BEACON_ENABLED,
//#if defined(CONFIG_BLE_MESH_GATT_PROXY_SERVER)
//    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_ENABLED,
//#else
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
//#endif
//#if defined(CONFIG_BLE_MESH_FRIEND)
//    .friend_state = ESP_BLE_MESH_FRIEND_ENABLED,
//#else
    .friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
//#endif
    .default_ttl = 7,
};

NET_BUF_SIMPLE_DEFINE_STATIC(data_temperature, 24);
NET_BUF_SIMPLE_DEFINE_STATIC(data_soil_moisture, 16);
NET_BUF_SIMPLE_DEFINE_STATIC(data_battery_level, 16);



NET_BUF_SIMPLE_DEFINE_STATIC(delta_down, 24);
NET_BUF_SIMPLE_DEFINE_STATIC(delta_up, 24);
NET_BUF_SIMPLE_DEFINE_STATIC(low, 24);
NET_BUF_SIMPLE_DEFINE_STATIC(high, 24);

static esp_ble_mesh_sensor_cadence_t sensor_cadence = {
    .period_divisor = 0, 				/* set as needed */
    .trigger_type = 0,					/* set as needed */
    .trigger_delta_down = &delta_down, 	/* pointer to net_buf_simple, properly initialized */
    .trigger_delta_up = &delta_up,		/* pointer to net_buf_simple, properly initialized */
    .min_interval = 1,					/* set as needed */
    .fast_cadence_low = &low, 			/* pointer to net_buf_simple, properly initialized */
    .fast_cadence_high = &high, 		/* pointer to net_buf_simple, properly initialized */
};



static esp_ble_mesh_sensor_state_t sensor_states[3] = {
    /* Mesh Model Spec:
     * Multiple instances of the Sensor states may be present within the same model,
     * provided that each instance has a unique value of the Sensor Property ID to
     * allow the instances to be differentiated. Such sensors are known as multisensors.
     * In this example, two instances of the Sensor states within the same model are
     * provided.
     */
    [0] = {
        /* Mesh Model Spec:
         * Sensor Property ID is a 2-octet value referencing a device property
         * that describes the meaning and format of data reported by a sensor.
         * 0x0000 is prohibited.
         */
        .sensor_property_id = SENSOR_PROPERTY_ID_0,
        /* Mesh Model Spec:
         * Sensor Descriptor state represents the attributes describing the sensor
         * data. This state does not change throughout the lifetime of an element.
         */
        .descriptor = {
            .positive_tolerance = SENSOR_POSITIVE_TOLERANCE,
            .negative_tolerance = SENSOR_NEGATIVE_TOLERANCE,
            .sampling_function = SENSOR_SAMPLE_FUNCTION,
            .measure_period = SENSOR_MEASURE_PERIOD,
            .update_interval = SENSOR_UPDATE_INTERVAL,
        },
        .sensor_data = {
            .format = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A,
            .length = 2, /* 0 represents the length is 1 */
            .raw_value = &data_temperature,
        },
        .cadence = &sensor_cadence,
    },
    [1] = {
        .sensor_property_id = SENSOR_PROPERTY_ID_1,
        .descriptor = {
            .positive_tolerance = SENSOR_POSITIVE_TOLERANCE,
            .negative_tolerance = SENSOR_NEGATIVE_TOLERANCE,
            .sampling_function = SENSOR_SAMPLE_FUNCTION,
            .measure_period = SENSOR_MEASURE_PERIOD,
            .update_interval = SENSOR_UPDATE_INTERVAL,
        },
        .sensor_data = {
            .format = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A,
            .length = 1, /* 0 represents the length is 1 */
            .raw_value = &data_soil_moisture,
        },
    },
    [2] = {
        .sensor_property_id = SENSOR_PROPERTY_ID_2,
        .descriptor = {
            .positive_tolerance = SENSOR_POSITIVE_TOLERANCE,
            .negative_tolerance = SENSOR_NEGATIVE_TOLERANCE,
            .sampling_function = SENSOR_SAMPLE_FUNCTION,
            .measure_period = SENSOR_MEASURE_PERIOD,
            .update_interval = SENSOR_UPDATE_INTERVAL,
        },
        .sensor_data = {
            .format = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A,
            .length = 1, /* 0 represents the length is 1 */
            .raw_value = &data_battery_level,
        },
    }
};

/* 20 octets is large enough to hold two Sensor Descriptor state values. */
ESP_BLE_MESH_MODEL_PUB_DEFINE(sensor_pub, 30, ROLE_NODE);
static esp_ble_mesh_sensor_srv_t sensor_server = {
    .rsp_ctrl = {
        .get_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
        .set_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
    },
    .state_count = ARRAY_SIZE(sensor_states),
    .states = sensor_states,
};

ESP_BLE_MESH_MODEL_PUB_DEFINE(sensor_setup_pub, 30, ROLE_NODE);
static esp_ble_mesh_sensor_setup_srv_t sensor_setup_server = {
    .rsp_ctrl = {
        .get_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
        .set_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
    },
    .state_count = ARRAY_SIZE(sensor_states),
    .states = sensor_states,
};

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
    ESP_BLE_MESH_MODEL_SENSOR_SRV(&sensor_pub, &sensor_server),
    ESP_BLE_MESH_MODEL_SENSOR_SETUP_SRV(&sensor_setup_pub, &sensor_setup_server),
};

static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, ESP_BLE_MESH_MODEL_NONE),
};

static esp_ble_mesh_comp_t composition = {
    .cid = CID_ESP,
    .element_count = ARRAY_SIZE(elements),
    .elements = elements,
};

static esp_ble_mesh_prov_t provision = {
    .uuid = dev_uuid,
};

void static enable_lpn_node(){
	esp_err_t err = bt_mesh_lpn_set(true, false);
	if (err == ESP_OK) {
	    ESP_LOGI(TAG_main, "LPN mode enabled");
	} else {
	    ESP_LOGE(TAG_main, "Failed to enable LPN (%d)", err);
	}
}

void read_sensor_temperature(void)
{
        uint8_t is_temp_below_zero = false;
        
        /* read data from sensor*/
        s_temperature = ds18b20_read() * 100;
        
        /*check if is negative value*/
        if(s_temperature < 0)
        {
			s_temperature = s_temperature * (-1);
			is_temp_below_zero = true;
		}
		
        ESP_LOGI(TAG_sensor, "temperature value: %f", s_temperature);
        
        /* Save data in memory for BLE Mesh */
		net_buf_simple_reset(&data_temperature);
        net_buf_simple_add_le16(&data_temperature, s_temperature);
        net_buf_simple_add_u8(&data_temperature, is_temp_below_zero);
        
        ESP_LOGI(TAG_sensor, "temperature[0]: %x", data_temperature.data[0]);
        ESP_LOGI(TAG_sensor, "temperature[1]: %x", data_temperature.data[1]);
        ESP_LOGI(TAG_sensor, "temperature[2]: %x", data_temperature.data[2]);
        ESP_LOGI(TAG_sensor, "temperature[3]: %x", data_temperature.data[3]);
}

void read_sensor_moisture(void)
{				
		/* read data from sensor */
		float adc_moisture_reading = adc_yl69_read();
		
		ESP_LOGI(TAG_sensor, "moisture value: %f", adc_moisture_reading);
		
		/*preparing data to send*/
		float s_moisture = adc_moisture_reading * 100;
				
		/* Save data in memory for BLE Mesh */
		net_buf_simple_reset(&data_soil_moisture);
        net_buf_simple_add_le16(&data_soil_moisture, s_moisture);
        ESP_LOGI(TAG_sensor, "moisture[0]: %x", data_soil_moisture.data[0]);
        ESP_LOGI(TAG_sensor, "moisture[1]: %x", data_soil_moisture.data[1]);        
}

void read_sensor_battery(void)
{				
		/* read data from sensor */
		uint16_t adc_battery_level = adc_battery_read();
		
		ESP_LOGI(TAG_sensor, "Tension read from adc: %d", (int) adc_battery_level);
		
		float result = adc_battery_level * 3.2;
		uint16_t bat_tension = (uint16_t) result;
		
		ESP_LOGI(TAG_sensor, "Battery tension: %d", (int) bat_tension);
				
		/* Save data in memory for BLE Mesh */
		net_buf_simple_reset(&data_battery_level);
        net_buf_simple_add_le16(&data_battery_level, bat_tension);
        ESP_LOGI(TAG_sensor, "battery[0]: %x", data_battery_level.data[0]);
        ESP_LOGI(TAG_sensor, "battery[1]: %x", data_battery_level.data[1]);        
}

void sensor_task(void *arg)
{
    while (1) {
		    ESP_LOGI(TAG_main, "Initializing sensor task");
        // Acquire PM lock to prevent sleep
        esp_pm_lock_acquire(pm_lock);

		    vTaskDelay(pdMS_TO_TICKS(2500));
		
        // Power sensors ON
        ds18b20_init();
		    adc_sensors_init();
				
		    // Read sensors and publish data
		    read_sensor_temperature();
		    read_sensor_moisture();
		    read_sensor_battery();
 		
        // Trigger Friend Poll
        bt_mesh_lpn_poll();

        // Power sensors OFF
        adc_sensors_deinit();
		    ds18b20_deinit();
		
		    vTaskDelay(pdMS_TO_TICKS(2500));
		
        // Release PM lock
        esp_pm_lock_release(pm_lock);
        
        // Sleep until next cycle
		    ESP_LOGI(TAG_main, "going to sleep for %i minutes", delay_sensor_routine);
        vTaskDelay(pdMS_TO_TICKS(delay_sensor_routine * 60 * 1000));
    }
}

void static register_lpn_routine(){
			xTaskCreatePinnedToCore(
		    sensor_task,         // Task function
		    "sensor_task",        	// Name (for debugging)
		    4096,              // Stack size in words (≈16 KB)
		    NULL,              // Task parameter
		    5,                   // Priority
		    NULL,             // Task handle (optional)
		    tskNO_AFFINITY        	// Core affinity (ESP32-C3 is single-core)
			);
}

static void prov_complete(uint16_t net_idx, uint16_t addr, uint8_t flags, uint32_t iv_index)
{
    ESP_LOGI(TAG_bt, "net_idx 0x%03x, addr 0x%04x", net_idx, addr);
    ESP_LOGI(TAG_bt, "flags 0x%02x, iv_index 0x%08" PRIx32, flags, iv_index);
    
    /* Initialize the temperature and humidity variables  */
    net_buf_simple_add_le24(&data_temperature, init_float_var);
    net_buf_simple_add_le16(&data_soil_moisture, init_int_var);
    net_buf_simple_add_le16(&data_battery_level, init_int_var);
}

static void ble_mesh_provisioning_cb(esp_ble_mesh_prov_cb_event_t event,
                                             esp_ble_mesh_prov_cb_param_t *param)
{
    switch (event) {
    case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
        ESP_LOGI(TAG_bt, "ESP_BLE_MESH_PROV_REGISTER_COMP_EVT, err_code %d", param->prov_register_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT:
        ESP_LOGI(TAG_bt, "ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT, err_code %d", param->node_prov_enable_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT:
        ESP_LOGI(TAG_bt, "ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT, bearer %s",
            param->node_prov_link_open.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT:
        ESP_LOGI(TAG_bt, "ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT, bearer %s",
            param->node_prov_link_close.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT:
        ESP_LOGI(TAG_bt, "ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT");
        prov_complete(param->node_prov_complete.net_idx, param->node_prov_complete.addr,
            param->node_prov_complete.flags, param->node_prov_complete.iv_index);
		
        break;
    case ESP_BLE_MESH_NODE_PROV_RESET_EVT:
        ESP_LOGI(TAG_bt, "ESP_BLE_MESH_NODE_PROV_RESET_EVT");
        break;
    case ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT:
        ESP_LOGI(TAG_bt, "ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT, err_code %d", param->node_set_unprov_dev_name_comp.err_code);
        break;
    case ESP_BLE_MESH_LPN_ENABLE_COMP_EVT:
        ESP_LOGI(TAG_bt, "ESP_BLE_MESH_LPN_ENABLE_COMP_EVT");
        break;
    case ESP_BLE_MESH_LPN_DISABLE_COMP_EVT:
        ESP_LOGI(TAG_bt, "ESP_BLE_MESH_LPN_DISABLE_COMP_EVT");
        break;
    case ESP_BLE_MESH_LPN_POLL_COMP_EVT:
        ESP_LOGI(TAG_bt, "ESP_BLE_MESH_LPN_POLL_COMP_EVT");
        break;
    case ESP_BLE_MESH_LPN_FRIENDSHIP_ESTABLISH_EVT:
        ESP_LOGI(TAG_bt, "ESP_BLE_MESH_LPN_FRIENDSHIP_ESTABLISH_EVT");
        register_lpn_routine();
        break;
    case ESP_BLE_MESH_LPN_FRIENDSHIP_TERMINATE_EVT:
        ESP_LOGI(TAG_bt, "ESP_BLE_MESH_LPN_FRIENDSHIP_TERMINATE_EVT");
        enable_lpn_node();
        
        break;
    case ESP_BLE_MESH_FRIEND_FRIENDSHIP_ESTABLISH_EVT:
        ESP_LOGI(TAG_bt, "ESP_BLE_MESH_LPN_FRIENDSHIP_TERMINATE_EVT");
        break;
    case ESP_BLE_MESH_FRIEND_FRIENDSHIP_TERMINATE_EVT:
        ESP_LOGI(TAG_bt, "ESP_BLE_MESH_LPN_FRIENDSHIP_TERMINATE_EVT");
        break;
    default:
        break;
    }
    taskYIELD(); 
}

static void ble_mesh_config_server_cb(esp_ble_mesh_cfg_server_cb_event_t event,
                                              esp_ble_mesh_cfg_server_cb_param_t *param)
{
    if (event == ESP_BLE_MESH_CFG_SERVER_STATE_CHANGE_EVT) {                
        switch (param->ctx.recv_op) {
        case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
            ESP_LOGI(TAG_bt, "ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD");
            ESP_LOGI(TAG_bt, "net_idx 0x%04x, app_idx 0x%04x",
                param->value.state_change.appkey_add.net_idx,
                param->value.state_change.appkey_add.app_idx);
            ESP_LOG_BUFFER_HEX("AppKey", param->value.state_change.appkey_add.app_key, 16);
            ctx_for_gateway.net_idx = param->value.state_change.appkey_add.net_idx;
            ctx_for_gateway.app_idx = param->value.state_change.appkey_add.app_idx;
            ctx_for_gateway.addr = 0x0001;
            ctx_for_gateway.send_ttl = 0;
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND:
            ESP_LOGI(TAG_bt, "ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND");
            ESP_LOGI(TAG_bt, "elem_addr 0x%04x, app_idx 0x%04x, cid 0x%04x, mod_id 0x%04x",
                param->value.state_change.mod_app_bind.element_addr,
                param->value.state_change.mod_app_bind.app_idx,
                param->value.state_change.mod_app_bind.company_id,
                param->value.state_change.mod_app_bind.model_id);
            //enable_lpn_node();
            
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD:
            ESP_LOGI(TAG_bt, "ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD");
            ESP_LOGI(TAG_bt, "elem_addr 0x%04x, sub_addr 0x%04x, cid 0x%04x, mod_id 0x%04x",
                param->value.state_change.mod_sub_add.element_addr,
                param->value.state_change.mod_sub_add.sub_addr,
                param->value.state_change.mod_sub_add.company_id,
                param->value.state_change.mod_sub_add.model_id);
            enable_lpn_node();
            
            
            break;
        default:
            break;
        }
    }
}

struct sensor_descriptor {
    uint16_t sensor_prop_id;
    uint32_t pos_tolerance:12,
             neg_tolerance:12,
             sample_func:8;
    uint8_t  measure_period;
    uint8_t  update_interval;
} __attribute__((packed));

static void ble_mesh_send_sensor_descriptor_status(esp_ble_mesh_sensor_server_cb_param_t *param)
{
    struct sensor_descriptor descriptor = {0};
    uint8_t *status = NULL;
    uint16_t length = 0;
    esp_err_t err;
    int i;

    status = (uint8_t *)calloc(1, ARRAY_SIZE(sensor_states) * ESP_BLE_MESH_SENSOR_DESCRIPTOR_LEN);
    if (!status) {
        ESP_LOGE(TAG_bt, "No memory for sensor descriptor status!");
        return;
    }

    if (param->value.get.sensor_descriptor.op_en == false) {
        /* Mesh Model Spec:
         * Upon receiving a Sensor Descriptor Get message with the Property ID field
         * omitted, the Sensor Server shall respond with a Sensor Descriptor Status
         * message containing the Sensor Descriptor states for all sensors within the
         * Sensor Server.
         */
        for (i = 0; i < ARRAY_SIZE(sensor_states); i++) {
            descriptor.sensor_prop_id = sensor_states[i].sensor_property_id;
            descriptor.pos_tolerance = sensor_states[i].descriptor.positive_tolerance;
            descriptor.neg_tolerance = sensor_states[i].descriptor.negative_tolerance;
            descriptor.sample_func = sensor_states[i].descriptor.sampling_function;
            descriptor.measure_period = sensor_states[i].descriptor.measure_period;
            descriptor.update_interval = sensor_states[i].descriptor.update_interval;
            memcpy(status + length, &descriptor, ESP_BLE_MESH_SENSOR_DESCRIPTOR_LEN);
            length += ESP_BLE_MESH_SENSOR_DESCRIPTOR_LEN;
        }
        goto send;
    }

    for (i = 0; i < ARRAY_SIZE(sensor_states); i++) {
        if (param->value.get.sensor_descriptor.property_id == sensor_states[i].sensor_property_id) {
            descriptor.sensor_prop_id = sensor_states[i].sensor_property_id;
            descriptor.pos_tolerance = sensor_states[i].descriptor.positive_tolerance;
            descriptor.neg_tolerance = sensor_states[i].descriptor.negative_tolerance;
            descriptor.sample_func = sensor_states[i].descriptor.sampling_function;
            descriptor.measure_period = sensor_states[i].descriptor.measure_period;
            descriptor.update_interval = sensor_states[i].descriptor.update_interval;
            memcpy(status, &descriptor, ESP_BLE_MESH_SENSOR_DESCRIPTOR_LEN);
            length = ESP_BLE_MESH_SENSOR_DESCRIPTOR_LEN;
            goto send;
        }
    }

    /* Mesh Model Spec:
     * When a Sensor Descriptor Get message that identifies a sensor descriptor
     * property that does not exist on the element, the Descriptor field shall
     * contain the requested Property ID value and the other fields of the Sensor
     * Descriptor state shall be omitted.
     */
    memcpy(status, &param->value.get.sensor_descriptor.property_id, ESP_BLE_MESH_SENSOR_PROPERTY_ID_LEN);
    length = ESP_BLE_MESH_SENSOR_PROPERTY_ID_LEN;

send:
    ESP_LOG_BUFFER_HEX("Sensor Descriptor", status, length);

    err = esp_ble_mesh_server_model_send_msg(param->model, &param->ctx,
            ESP_BLE_MESH_MODEL_OP_SENSOR_DESCRIPTOR_STATUS, length, status);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_bt, "Failed to send Sensor Descriptor Status");
    }
    free(status);
}


static void ble_mesh_send_sensor_cadence_status(esp_ble_mesh_sensor_server_cb_param_t *param)
{
    esp_err_t err;

    /* Sensor Cadence state is not supported currently. */
    err = esp_ble_mesh_server_model_send_msg(param->model, &param->ctx,
            ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_STATUS,
            ESP_BLE_MESH_SENSOR_PROPERTY_ID_LEN,
            (uint8_t *)&param->value.get.sensor_cadence.property_id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_bt, "Failed to send Sensor Cadence Status");
    }
}

static void ble_mesh_send_sensor_settings_status(esp_ble_mesh_sensor_server_cb_param_t *param)
{
    esp_err_t err;

    /* Sensor Setting state is not supported currently. */
    err = esp_ble_mesh_server_model_send_msg(param->model, &param->ctx,
            ESP_BLE_MESH_MODEL_OP_SENSOR_SETTINGS_STATUS,
            ESP_BLE_MESH_SENSOR_PROPERTY_ID_LEN,
            (uint8_t *)&param->value.get.sensor_settings.property_id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_bt, "Failed to send Sensor Settings Status");
    }
}

struct sensor_setting {
    uint16_t sensor_prop_id;
    uint16_t sensor_setting_prop_id;
} __attribute__((packed));

static void ble_mesh_send_sensor_setting_status(esp_ble_mesh_sensor_server_cb_param_t *param)
{
    struct sensor_setting setting = {0};
    esp_err_t err;

    /* Mesh Model Spec:
     * If the message is sent as a response to the Sensor Setting Get message or
     * a Sensor Setting Set message with an unknown Sensor Property ID field or
     * an unknown Sensor Setting Property ID field, the Sensor Setting Access
     * field and the Sensor Setting Raw field shall be omitted.
     */

    setting.sensor_prop_id = param->value.get.sensor_setting.property_id;
    setting.sensor_setting_prop_id = param->value.get.sensor_setting.setting_property_id;

    err = esp_ble_mesh_server_model_send_msg(param->model, &param->ctx,
            ESP_BLE_MESH_MODEL_OP_SENSOR_SETTING_STATUS,
            sizeof(setting), (uint8_t *)&setting);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_bt, "Failed to send Sensor Setting Status");
    }
}

static uint16_t ble_mesh_get_sensor_data(esp_ble_mesh_sensor_state_t *state, uint8_t *data)
{
    uint8_t mpid_len = 0, data_len = 0;
    uint32_t mpid = 0;

    if (state == NULL || data == NULL) {
        ESP_LOGE(TAG_bt, "%s, Invalid parameter", __func__);
        return 0;
    }

    if (state->sensor_data.length == ESP_BLE_MESH_SENSOR_DATA_ZERO_LEN) {
        /* For zero-length sensor data, the length is 0x7F, and the format is Format B. */
        mpid = ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID(state->sensor_data.length, state->sensor_property_id);
        mpid_len = ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID_LEN;
        data_len = 0;
    } else {
        if (state->sensor_data.format == ESP_BLE_MESH_SENSOR_DATA_FORMAT_A) {
            mpid = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A_MPID(state->sensor_data.length, state->sensor_property_id);
            mpid_len = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A_MPID_LEN;
        } else {
            mpid = ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID(state->sensor_data.length, state->sensor_property_id);
            mpid_len = ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID_LEN;
        }
        /* Use "state->sensor_data.length + 1" because the length of sensor data is zero-based. */
        data_len = state->sensor_data.length + 1;
    }

    memcpy(data, &mpid, mpid_len);
    memcpy(data + mpid_len, state->sensor_data.raw_value->data, data_len);

    return (mpid_len + data_len);
}

static void ble_mesh_send_sensor_status(esp_ble_mesh_sensor_server_cb_param_t *param)
{
    uint8_t *status = NULL;
    uint16_t buf_size = 0;
    uint16_t length = 0;
    uint32_t mpid = 0;
    esp_err_t err;
    int i;

    /**
     * Sensor Data state from Mesh Model Spec
     * |--------Field--------|-Size (octets)-|------------------------Notes-------------------------|
     * |----Property ID 1----|-------2-------|--ID of the 1st device property of the sensor---------|
     * |-----Raw Value 1-----|----variable---|--Raw Value field defined by the 1st device property--|
     * |----Property ID 2----|-------2-------|--ID of the 2nd device property of the sensor---------|
     * |-----Raw Value 2-----|----variable---|--Raw Value field defined by the 2nd device property--|
     * | ...... |
     * |----Property ID n----|-------2-------|--ID of the nth device property of the sensor---------|
     * |-----Raw Value n-----|----variable---|--Raw Value field defined by the nth device property--|
     */
    for (i = 0; i < ARRAY_SIZE(sensor_states); i++) {
        esp_ble_mesh_sensor_state_t *state = &sensor_states[i];
        if (state->sensor_data.length == ESP_BLE_MESH_SENSOR_DATA_ZERO_LEN) {
            buf_size += ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID_LEN;
        } else {
            /* Use "state->sensor_data.length + 1" because the length of sensor data is zero-based. */
            if (state->sensor_data.format == ESP_BLE_MESH_SENSOR_DATA_FORMAT_A) {
                buf_size += ESP_BLE_MESH_SENSOR_DATA_FORMAT_A_MPID_LEN + state->sensor_data.length + 1;
            } else {
                buf_size += ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID_LEN + state->sensor_data.length + 1;
            }
        }
    }

    status = (uint8_t *)calloc(1, buf_size);
    if (!status) {
        ESP_LOGE(TAG_bt, "No memory for sensor status!");
        return;
    }

    if (param->value.get.sensor_data.op_en == false) {
        /* Mesh Model Spec:
         * If the message is sent as a response to the Sensor Get message, and if the
         * Property ID field of the incoming message is omitted, the Marshalled Sensor
         * Data field shall contain data for all device properties within a sensor.
         */
        for (i = 0; i < ARRAY_SIZE(sensor_states); i++) {
            length += ble_mesh_get_sensor_data(&sensor_states[i], status + length);
        }
        goto send;
    }

    /* Mesh Model Spec:
     * Otherwise, the Marshalled Sensor Data field shall contain data for the requested
     * device property only.
     */
    for (i = 0; i < ARRAY_SIZE(sensor_states); i++) {
        if (param->value.get.sensor_data.property_id == sensor_states[i].sensor_property_id) {
            length = ble_mesh_get_sensor_data(&sensor_states[i], status);
            goto send;
        }
    }

    /* Mesh Model Spec:
     * Or the Length shall represent the value of zero and the Raw Value field shall
     * contain only the Property ID if the requested device property is not recognized
     * by the Sensor Server.
     */
    mpid = ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID(ESP_BLE_MESH_SENSOR_DATA_ZERO_LEN,
            param->value.get.sensor_data.property_id);
    memcpy(status, &mpid, ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID_LEN);
    length = ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID_LEN;

send:
    ESP_LOG_BUFFER_HEX("Sensor Data", status, length);

    err = esp_ble_mesh_server_model_send_msg(param->model, &param->ctx,
            ESP_BLE_MESH_MODEL_OP_SENSOR_STATUS, length, status);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_bt, "Failed to send Sensor Status");
    }
    free(status);
    ESP_LOGI(TAG_bt, "SENT MESSAGE STATUS - EXITING");
    
}

// Function to send data to node gateway without waiting a GET message.
static void ble_mesh_publish_sensor_status()
{
    uint8_t *status = NULL;
    uint16_t buf_size = 0;
    uint16_t length = 0;
    esp_err_t err;
    int i;

    /**
     * Sensor Data state from Mesh Model Spec
     * |--------Field--------|-Size (octets)-|------------------------Notes-------------------------|
     * |----Property ID 1----|-------2-------|--ID of the 1st device property of the sensor---------|
     * |-----Raw Value 1-----|----variable---|--Raw Value field defined by the 1st device property--|
     * |----Property ID 2----|-------2-------|--ID of the 2nd device property of the sensor---------|
     * |-----Raw Value 2-----|----variable---|--Raw Value field defined by the 2nd device property--|
     * | ...... |
     * |----Property ID n----|-------2-------|--ID of the nth device property of the sensor---------|
     * |-----Raw Value n-----|----variable---|--Raw Value field defined by the nth device property--|
     */
    for (i = 0; i < ARRAY_SIZE(sensor_states); i++) {
        esp_ble_mesh_sensor_state_t *state = &sensor_states[i];
        if (state->sensor_data.length == ESP_BLE_MESH_SENSOR_DATA_ZERO_LEN) {
            buf_size += ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID_LEN;
        } else {
            /* Use "state->sensor_data.length + 1" because the length of sensor data is zero-based. */
            if (state->sensor_data.format == ESP_BLE_MESH_SENSOR_DATA_FORMAT_A) {
                buf_size += ESP_BLE_MESH_SENSOR_DATA_FORMAT_A_MPID_LEN + state->sensor_data.length + 1;
            } else {
                buf_size += ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID_LEN + state->sensor_data.length + 1;
            }
        }
    }

    status = (uint8_t *)calloc(1, buf_size);
    if (!status) {
        ESP_LOGE(TAG_bt, "No memory for sensor status!");
        return;
    }

 
    for (i = 0; i < ARRAY_SIZE(sensor_states); i++) {
        length += ble_mesh_get_sensor_data(&sensor_states[i], status + length);
    }

    ESP_LOG_BUFFER_HEX("Sensor Data", status, length);

	err = esp_ble_mesh_server_model_send_msg(sensor_server.model, 
			&ctx_for_gateway, ESP_BLE_MESH_MODEL_OP_SENSOR_STATUS,
			length, status);

    if (err != ESP_OK) {
        ESP_LOGE(TAG_bt, "Failed to publish Sensor Status");
    }
    free(status);
    ESP_LOGI(TAG_bt, "PUBLISH MESSAGE STATUS - EXITING");
    
    
}

static void ble_mesh_send_sensor_column_status(esp_ble_mesh_sensor_server_cb_param_t *param)
{
    uint8_t *status = NULL;
    uint16_t length = 0;
    esp_err_t err;

    length = ESP_BLE_MESH_SENSOR_PROPERTY_ID_LEN +param->value.get.sensor_column.raw_value_x->len;

    status = (uint8_t *)calloc(1, length);
    if (!status) {
        ESP_LOGE(TAG_bt, "No memory for sensor column status!");
        return;
    }

    memcpy(status, &param->value.get.sensor_column.property_id, ESP_BLE_MESH_SENSOR_PROPERTY_ID_LEN);
    memcpy(status + ESP_BLE_MESH_SENSOR_PROPERTY_ID_LEN, param->value.get.sensor_column.raw_value_x->data,
        param->value.get.sensor_column.raw_value_x->len);

    err = esp_ble_mesh_server_model_send_msg(param->model, &param->ctx,
            ESP_BLE_MESH_MODEL_OP_SENSOR_COLUMN_STATUS, length, status);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_bt, "Failed to send Sensor Column Status");
    }
    free(status);
}

static void ble_mesh_send_sensor_series_status(esp_ble_mesh_sensor_server_cb_param_t *param)
{
    esp_err_t err;

    err = esp_ble_mesh_server_model_send_msg(param->model, &param->ctx,
            ESP_BLE_MESH_MODEL_OP_SENSOR_SERIES_STATUS,
            ESP_BLE_MESH_SENSOR_PROPERTY_ID_LEN,
            (uint8_t *)&param->value.get.sensor_series.property_id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_bt, "Failed to send Sensor Column Status");
    }
}

static void ble_mesh_sensor_server_cb(esp_ble_mesh_sensor_server_cb_event_t event,
                                              esp_ble_mesh_sensor_server_cb_param_t *param)
{
    ESP_LOGI(TAG_bt, "Sensor server, event %d, src 0x%04x, dst 0x%04x, model_id 0x%04x",
        event, param->ctx.addr, param->ctx.recv_dst, param->model->model_id);

    switch (event) {
    case ESP_BLE_MESH_SENSOR_SERVER_RECV_GET_MSG_EVT:
        switch (param->ctx.recv_op) {
        case ESP_BLE_MESH_MODEL_OP_SENSOR_DESCRIPTOR_GET:
            ESP_LOGI(TAG_bt, "ESP_BLE_MESH_MODEL_OP_SENSOR_DESCRIPTOR_GET");
            ble_mesh_send_sensor_descriptor_status(param);
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_GET:
            ESP_LOGI(TAG_bt, "ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_GET");
            //ble_mesh_send_sensor_cadence_status(param);
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_SETTINGS_GET:
            ESP_LOGI(TAG_bt, "ESP_BLE_MESH_MODEL_OP_SENSOR_SETTINGS_GET");
            ble_mesh_send_sensor_settings_status(param);
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_SETTING_GET:
            ESP_LOGI(TAG_bt, "ESP_BLE_MESH_MODEL_OP_SENSOR_SETTINGS_GET");
            ble_mesh_send_sensor_setting_status(param);
            break;
            
            /* OP SENSOR GET CALLBACK */
            /* Here functions are executed when receives a get from Gateway*/
        case ESP_BLE_MESH_MODEL_OP_SENSOR_GET:
            ESP_LOGI(TAG_bt, "ESP_BLE_MESH_MODEL_OP_SENSOR_GET");
            ble_mesh_send_sensor_status(param);
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_COLUMN_GET:
            ESP_LOGI(TAG_bt, "ESP_BLE_MESH_MODEL_OP_SENSOR_COLUMN_GET");
            ble_mesh_send_sensor_column_status(param);
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_SERIES_GET:
            ESP_LOGI(TAG_bt, "ESP_BLE_MESH_MODEL_OP_SENSOR_SERIES_GET");
            ble_mesh_send_sensor_series_status(param);
            break;
        default:
            ESP_LOGE(TAG_bt, "Unknown Sensor Get opcode 0x%04" PRIx32, param->ctx.recv_op);
            return;
        }
        break;
    case ESP_BLE_MESH_SENSOR_SERVER_RECV_SET_MSG_EVT:
        switch (param->ctx.recv_op) {
        case ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_SET:
            ESP_LOGI(TAG_bt, "ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_SET");
         
            ESP_LOG_BUFFER_HEX("Cadence Data", param->value.set.sensor_cadence.cadence->data,
                    param->value.set.sensor_cadence.cadence->len);
              
            // Process and store cadence data
            uint8_t * data = param->value.set.sensor_cadence.cadence->data;
		    uint8_t cadence = data[5];
		    
		    delay_sensor_routine = cadence;
		    
		    ESP_LOGI(TAG_bt, "value obtained: %i minutes", delay_sensor_routine );
		    
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_SET_UNACK:
            ESP_LOGI(TAG_bt, "ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_SET_UNACK");
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_SETTING_SET:
            ESP_LOGI(TAG_bt, "ESP_BLE_MESH_MODEL_OP_SENSOR_SETTING_SET");
            ble_mesh_send_sensor_setting_status(param);
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_SETTING_SET_UNACK:
            ESP_LOGI(TAG_bt, "ESP_BLE_MESH_MODEL_OP_SENSOR_SETTING_SET_UNACK");
            break;
        default:
            ESP_LOGE(TAG_bt, "Unknown Sensor Set opcode 0x%04" PRIx32, param->ctx.recv_op);
            break;
        } 
        break;
    default:
        ESP_LOGE(TAG_bt, "Unknown Sensor Server event %d", event);
        break;
    }
}

static esp_err_t ble_mesh_init(void)
{
    esp_err_t err;

    esp_ble_mesh_register_prov_callback(ble_mesh_provisioning_cb);
    esp_ble_mesh_register_config_server_callback(ble_mesh_config_server_cb);
    esp_ble_mesh_register_sensor_server_callback(ble_mesh_sensor_server_cb);

    err = esp_ble_mesh_init(&provision, &composition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_bt, "Failed to initialize mesh stack");
        return err;
    }

    err = esp_ble_mesh_node_prov_enable((esp_ble_mesh_prov_bearer_t)(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT));
    if (err != ESP_OK) {
        ESP_LOGE(TAG_bt, "Failed to enable mesh node");
        return err;
    }

    ESP_LOGI(TAG_bt, "BLE Mesh sensor server initialized");

    return ESP_OK;
}

void app_main(void)
{
    esp_err_t err;

	ESP_LOGI(TAG_main, "Erasing...");
    //Borra toda la NVS
    err = nvs_flash_erase();
    if (err != ESP_OK) {
		ESP_LOGE(TAG_main, "Error erasing NVS: %d", err);
      return;
    }

    ESP_LOGI(TAG_main, "Initializing...");

    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    //candece config purposes
	  net_buf_simple_add_le16(&delta_up, 0);
	  net_buf_simple_add_le16(&low, 0);
	  net_buf_simple_add_le16(&high, 0);
	  net_buf_simple_add_le16(&delta_down, 0);

    err = bluetooth_init();
    if (err) {
        ESP_LOGE(TAG_main, "esp32_bluetooth_init failed (err %d)", err);
        return;
    }

    ble_mesh_get_dev_uuid(dev_uuid);

    /* Initialize Bluetooth Mesh Subsystem */
    err = ble_mesh_init();
    if (err) {
        ESP_LOGE(TAG_main, "Bluetooth mesh init failed (err %d)", err);
    }

    // Configure dynamic frequency scaling:
    // maximum and minimum frequencies are set in sdkconfig,
    // automatic light sleep is enabled if tickless idle support is enabled.

    esp_pm_config_t pm_config = {
            .max_freq_mhz = MAX_CPU_FREQ_MHZ,
            .min_freq_mhz = MIN_CPU_FREQ_MHZ,
            .light_sleep_enable = true
    };
    ESP_ERROR_CHECK( esp_pm_configure(&pm_config) );
    

}
