/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: LicenseRef-Included
 *
 * Zigbee Gateway Example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */
#include <fcntl.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/usb_serial_jtag.h"
#include "esp_coexist.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_eventfd.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include "esp_zigbee_gateway.h"
#include "zb_config_platform.h"
#include "zcl/esp_zigbee_zcl_command.h"

static const char *TAG = "ESP_ZB_GATEWAY";

/* Note: Please select the correct console output port based on the development board in menuconfig */
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
esp_err_t esp_zb_gateway_console_init(void)
{
	esp_err_t ret = ESP_OK;
	/* Disable buffering on stdin */
	setvbuf(stdin, NULL, _IONBF, 0);

	/* Minicom, screen, idf_monitor send CR when ENTER key is pressed */
	usb_serial_jtag_vfs_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
	/* Move the caret to the beginning of the next line on '\n' */
	usb_serial_jtag_vfs_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);

	/* Enable non-blocking mode on stdin and stdout */
	fcntl(fileno(stdout), F_SETFL, O_NONBLOCK);
	fcntl(fileno(stdin), F_SETFL, O_NONBLOCK);

	usb_serial_jtag_driver_config_t usb_serial_jtag_config = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
	ret = usb_serial_jtag_driver_install(&usb_serial_jtag_config);
	usb_serial_jtag_vfs_use_driver();
	uart_vfs_dev_register();
	return ret;
}
#endif

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
	ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, , TAG, "Failed to start Zigbee bdb commissioning");
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
	uint32_t *p_sg_p       = signal_struct->p_app_signal;
	esp_err_t err_status = signal_struct->esp_err_status;
	esp_zb_app_signal_type_t sig_type = *p_sg_p;
	esp_zb_zdo_signal_device_annce_params_t *dev_annce_params = NULL;

	switch (sig_type) {
	case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
#if CONFIG_EXAMPLE_CONNECT_WIFI
#if CONFIG_ESP_COEX_SW_COEXIST_ENABLE
		esp_coex_wifi_i154_enable();
#endif /* CONFIG_ESP_COEX_SW_COEXIST_ENABLE */
		ESP_RETURN_ON_FALSE(example_connect() == ESP_OK, , TAG, "Failed to connect to Wi-Fi");
		ESP_RETURN_ON_FALSE(esp_wifi_set_ps(WIFI_PS_MIN_MODEM) == ESP_OK, , TAG, "Failed to set Wi-Fi minimum modem power save type");
#endif /* CONFIG_EXAMPLE_CONNECT_WIFI */
		ESP_LOGI(TAG, "Initialize Zigbee stack");
		esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
		break;
	case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
	case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
		if (err_status == ESP_OK) {
			ESP_LOGI(TAG, "Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
			if (esp_zb_bdb_is_factory_new()) {
				ESP_LOGI(TAG, "Start network formation");
				esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_FORMATION);
			} else {
				esp_zb_bdb_open_network(180);
				ESP_LOGI(TAG, "Device rebooted");
			}
		} else {
			ESP_LOGE(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
		}
		break;
	case ESP_ZB_BDB_SIGNAL_FORMATION:
		if (err_status == ESP_OK) {
			esp_zb_ieee_addr_t ieee_address;
			esp_zb_get_long_address(ieee_address);
			ESP_LOGI(TAG, "Formed network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
					 ieee_address[7], ieee_address[6], ieee_address[5], ieee_address[4],
					 ieee_address[3], ieee_address[2], ieee_address[1], ieee_address[0],
					 esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
			esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
		} else {
			ESP_LOGI(TAG, "Restart network formation (status: %s)", esp_err_to_name(err_status));
			esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_FORMATION, 1000);
		}
		break;
	case ESP_ZB_BDB_SIGNAL_STEERING:
		if (err_status == ESP_OK) {
			ESP_LOGI(TAG, "Network steering started");
		}
		break;


	case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE:
		dev_annce_params = (esp_zb_zdo_signal_device_annce_params_t *)esp_zb_app_signal_get_params(p_sg_p);
		ESP_LOGI(TAG, "New device commissioned or rejoined (short: 0x%04hx)", dev_annce_params->device_short_addr);







        // Ask device to report temperature
        static uint16_t temp_change_threshold = 100; // 1.0°C change threshold (in 0.01°C units)

        esp_zb_zcl_config_report_record_t report_record = {
            .direction = ESP_ZB_ZCL_REPORT_DIRECTION_SEND,
            .attributeID = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
            .attrType = ESP_ZB_ZCL_ATTR_TYPE_S16,
            .min_interval = 10,  // report no more often than 10s
            .max_interval = 60,  // report at least every 60s
            .reportable_change = &temp_change_threshold,
        };

        esp_zb_zcl_config_report_cmd_t req = {
            .zcl_basic_cmd = {
                .dst_addr_u.addr_short = dev_annce_params->device_short_addr,
                .dst_endpoint = 1,
                .src_endpoint = ESP_ZB_GATEWAY_ENDPOINT,
            },
            .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
            .clusterID = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
            .manuf_specific = 0,
            .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
            .dis_default_resp = 0,
            .manuf_code = 0,
            .record_number = 1,
            .record_field = &report_record,
        };
        esp_zb_zcl_config_report_cmd_req(&req);
        ESP_LOGI(TAG, "Requested temperature reporting from 0x%04hx",
                 dev_annce_params->device_short_addr);





		break;


	case ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS:
		if (err_status == ESP_OK) {
			if (*(uint8_t *)esp_zb_app_signal_get_params(p_sg_p)) {
				ESP_LOGI(TAG, "Network(0x%04hx) is open for %d seconds", esp_zb_get_pan_id(), *(uint8_t *)esp_zb_app_signal_get_params(p_sg_p));
			} else {
				ESP_LOGW(TAG, "Network(0x%04hx) closed, devices joining not allowed.", esp_zb_get_pan_id());
			}
		}
		break;
	case ESP_ZB_ZDO_SIGNAL_PRODUCTION_CONFIG_READY:
		ESP_LOGI(TAG, "Production configuration is %s", err_status == ESP_OK ? "ready" : "not present");
		esp_zb_set_node_descriptor_manufacturer_code(ESP_MANUFACTURER_CODE);
		break;
	default:
		ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
				 esp_err_to_name(err_status));
		break;
	}
}





static void tuya_cluster_parse_and_print(uint8_t *data, uint16_t len)
{
    if (len < 7) { // minimal Tuya payload length
        ESP_LOGW("TUYA", "Invalid payload (len=%d)", len);
        return;
    }

    uint8_t status = data[0];
    uint8_t seq    = data[1];
    uint8_t dpid   = data[2];
    uint8_t dtype  = data[3];
    uint16_t dlen  = (data[4] << 8) | data[5];

    if (6 + dlen > len) {
        ESP_LOGW("TUYA", "Length mismatch (dlen=%d, total=%d)", dlen, len);
        return;
    }

    ESP_LOGI("TUYA", "DPID=0x%02X, type=0x%02X, len=%d, seq=%d", dpid, dtype, dlen, seq);

    if (dtype == 0x02 && dlen == 4) { // value, int32
        int value = (data[6] << 24) | (data[7] << 16) | (data[8] << 8) | data[9];

        switch (dpid) {
        case 0x01: // temperature (*10)
            ESP_LOGI("TEMP", "🌡️  Temperature = %.1f °C", value / 10.0f);
            break;
        case 0x02: // humidity
            ESP_LOGI("TUYA", "💧 Humidity = %d %%", value);
            break;
        case 0x04: // battery
            ESP_LOGI("TUYA", "🔋 Battery = %d %%", value);
            break;
        default:
            ESP_LOGI("TUYA", "Unhandled DPID=0x%02X, value=%d", dpid, value);
            break;
        }
    } else {
        ESP_LOGI("TUYA", "Unhandled data type=0x%02X", dtype);
    }
}









static esp_err_t zb_app_signal_handler_impl(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;

    switch (callback_id) {
    case ESP_ZB_CORE_REPORT_ATTR_CB_ID: {
        esp_zb_zcl_report_attr_message_t *msg = (esp_zb_zcl_report_attr_message_t *)message;
        if (msg && msg->cluster == ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT &&
            msg->attribute.id == ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID) {
            int16_t raw = *(int16_t *)msg->attribute.data.value;
            float temperature = raw / 100.0f; // Zigbee encodes temp in 0.01 °C
            ESP_LOGI("TEMP", "Temperature: %.2f °C", temperature);
        }
        break;
    }
    case ESP_ZB_CORE_CMD_CUSTOM_CLUSTER_REQ_CB_ID: {
        esp_zb_zcl_custom_cluster_command_message_t *msg = (esp_zb_zcl_custom_cluster_command_message_t *)message;
        if (msg && msg->info.cluster == 0xef00) {
            ESP_LOGI(TAG, "Received custom cluster 0xef00 command: cmd_id=0x%02x, src_addr=0x%04x, data_len=%d",
                     msg->info.command.id, msg->info.src_address.u.short_addr, msg->data.size);

            if (msg->data.size > 0 && msg->data.value) {
                ESP_LOG_BUFFER_HEX(TAG, msg->data.value, msg->data.size);

                tuya_cluster_parse_and_print(msg->data.value, msg->data.size);

            }
        }
        break;
    }
    default:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
    }

    return ret;
}



static void esp_zb_task(void *pvParameters)
{
	/* initialize Zigbee stack */
	esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZC_CONFIG();
	esp_zb_init(&zb_nwk_cfg);
	esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
	esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
	esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
	esp_zb_endpoint_config_t endpoint_config = {
		.endpoint = ESP_ZB_GATEWAY_ENDPOINT,
		.app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
		.app_device_id = ESP_ZB_HA_REMOTE_CONTROL_DEVICE_ID,
		.app_device_version = 0,
	};

	esp_zb_attribute_list_t *basic_cluser = esp_zb_basic_cluster_create(NULL);
	esp_zb_basic_cluster_add_attr(basic_cluser, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, ESP_MANUFACTURER_NAME);
	esp_zb_basic_cluster_add_attr(basic_cluser, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, ESP_MODEL_IDENTIFIER);
	esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluser, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
	esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_identify_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);


	esp_zb_temperature_meas_cluster_cfg_t temp_cfg = {
	.measured_value = 2000,   // in 0.01°C, so this = 20.00 °C
	.min_value = -4000,       // -40.00 °C
	.max_value = 8500,        // 85.00 °C
	};

	esp_zb_attribute_list_t *temp_cluster = esp_zb_temperature_meas_cluster_create(&temp_cfg);

	/* Add mandatory attribute MeasuredValue (0x0000), default to 0 */
	esp_zb_temperature_meas_cluster_add_attr(
		temp_cluster,
		ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
		0);

	esp_zb_cluster_list_add_temperature_meas_cluster(
		cluster_list,
		temp_cluster,
		ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

	// Add custom cluster 0xef00 (commonly used by Tuya devices)
	esp_zb_attribute_list_t *custom_cluster = esp_zb_zcl_attr_list_create(0xef00);
	esp_zb_cluster_list_add_custom_cluster(cluster_list, custom_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);

	esp_zb_ep_list_add_gateway_ep(ep_list, cluster_list, endpoint_config);
	esp_zb_device_register(ep_list);
	esp_zb_core_action_handler_register(zb_app_signal_handler_impl);
	ESP_ERROR_CHECK(esp_zb_start(false));
	esp_zb_stack_main_loop();
	vTaskDelete(NULL);
}

void app_main(void)
{
	esp_zb_platform_config_t config = {
		.radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
		.host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
	};

	ESP_ERROR_CHECK(esp_zb_platform_config(&config));
	ESP_ERROR_CHECK(nvs_flash_init());
	ESP_ERROR_CHECK(esp_netif_init());
	ESP_ERROR_CHECK(esp_event_loop_create_default());
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
	ESP_ERROR_CHECK(esp_zb_gateway_console_init());
#endif
	xTaskCreate(esp_zb_task, "Zigbee_main", 8192, NULL, 5, NULL);
}
