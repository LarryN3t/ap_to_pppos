/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

/*
 * softAP to PPPoS Example (network_dce)
*/

#include <string.h>
#include "esp_netif.h"
#include "esp_modem_api.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_modem_api.h"

static esp_modem_dce_t *dce = NULL;
static const char *TAG = "dce_setup";

#define MODEM_PWKEY          GPIO_NUM_4
#define MODEM_POWER_ON       GPIO_NUM_23
#define MODEM_RST            GPIO_NUM_5

esp_err_t modem_init_network(esp_netif_t *netif)
{
        
    ESP_LOGE(TAG, "Eccoci");
    //digitalWrite(MODEM_PWKEY, LOW);
    gpio_set_direction(MODEM_PWKEY, GPIO_MODE_OUTPUT);
     gpio_set_level(MODEM_PWKEY,0);
    //digitalWrite(MODEM_RST, HIGH);
    gpio_set_direction(MODEM_RST, GPIO_MODE_OUTPUT);
     gpio_set_level(MODEM_RST,1);
    //digitalWrite(MODEM_POWER_ON, HIGH);
    gpio_set_direction(MODEM_POWER_ON, GPIO_MODE_OUTPUT);
    gpio_set_level(MODEM_POWER_ON,1);
    vTaskDelay(10000/ portTICK_PERIOD_MS);
    // setup the DCE
    esp_modem_dte_config_t dte_config = ESP_MODEM_DTE_DEFAULT_CONFIG();
    esp_modem_dce_config_t dce_config = ESP_MODEM_DCE_DEFAULT_CONFIG(CONFIG_EXAMPLE_MODEM_PPP_APN);
 
    /* Configure the DTE */
    //esp_modem_dte_config_t dte_config = ESP_MODEM_DTE_DEFAULT_CONFIG();
    /* setup UART specific configuration based on kconfig options */
    dte_config.uart_config.tx_io_num = CONFIG_EXAMPLE_MODEM_UART_TX_PIN;
    dte_config.uart_config.rx_io_num = CONFIG_EXAMPLE_MODEM_UART_RX_PIN;
    dte_config.uart_config.rts_io_num = CONFIG_EXAMPLE_MODEM_UART_RTS_PIN;
    dte_config.uart_config.cts_io_num = CONFIG_EXAMPLE_MODEM_UART_CTS_PIN;
    dte_config.uart_config.flow_control = CONFIG_EXAMPLE_FLOW_CONTROL_NONE;
    dte_config.uart_config.rx_buffer_size = CONFIG_EXAMPLE_MODEM_UART_RX_BUFFER_SIZE;
    dte_config.uart_config.tx_buffer_size = CONFIG_EXAMPLE_MODEM_UART_TX_BUFFER_SIZE;
    dte_config.uart_config.event_queue_size = CONFIG_EXAMPLE_MODEM_UART_EVENT_QUEUE_SIZE;
    dte_config.task_stack_size = CONFIG_EXAMPLE_MODEM_UART_EVENT_TASK_STACK_SIZE;
    dte_config.task_priority = CONFIG_EXAMPLE_MODEM_UART_EVENT_TASK_PRIORITY;
    dte_config.dte_buffer_size = CONFIG_EXAMPLE_MODEM_UART_RX_BUFFER_SIZE / 2;


    ESP_LOGI(TAG, "Initializing esp_modem for the SIM800 module...");
    esp_modem_dce_t *dce = esp_modem_new_dev(ESP_MODEM_DCE_SIM800, &dte_config, &dce_config, netif);
    ESP_LOGI(TAG, "Initialized");

    assert(dce);
    if (dte_config.uart_config.flow_control == ESP_MODEM_FLOW_CONTROL_HW) {
        esp_err_t err = esp_modem_set_flow_control(dce, 2, 2);  //2/2 means HW Flow Control.
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set the set_flow_control mode");
        }
        ESP_LOGI(TAG, "HW set_flow_control OK");
    }

     //   dce = esp_modem_new(&dte_config, &dce_config, netif);
    if (!dce) {
        ESP_LOGE(TAG, "Fallito");
        return ESP_FAIL;
    }




    int rssi, ber;
    esp_err_t err = esp_modem_get_signal_quality(dce, &rssi, &ber);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "esp_modem_get_signal_quality failed with %d %s", err, esp_err_to_name(err));
    }
    ESP_LOGI(TAG, "Signal quality: rssi=%d, ber=%d", rssi, ber);

        err = esp_modem_set_mode(dce, ESP_MODEM_MODE_DATA);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "esp_modem_set_mode(ESP_MODEM_MODE_DATA) failed with %d", err);
    }

    return ESP_OK;
}

void modem_deinit_network(void)
{
    if (dce) {
        esp_modem_destroy(dce);
        dce = NULL;
    }
}

bool modem_start_network()
{
    return esp_modem_set_mode(dce, ESP_MODEM_MODE_DATA) == ESP_OK;
}

bool modem_stop_network()
{
    return esp_modem_set_mode(dce, ESP_MODEM_MODE_COMMAND);
}

bool modem_check_sync()
{
    return esp_modem_sync(dce) == ESP_OK;
}

void modem_reset()
{
    esp_modem_reset(dce);
}

bool modem_check_signal()
{
    int rssi, ber;
    esp_err_t err = esp_modem_get_signal_quality(dce, &rssi, &ber);
    return rssi;
    
}
