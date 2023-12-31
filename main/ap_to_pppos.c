/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

/*
 *  softAP to PPPoS Example
*/
#include <string.h>
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_idf_version.h"
#include "nvs_flash.h"
#include "lwip/netif.h"
#include "lwip/lwip_napt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "network_dce.h"
#include "esp_netif_ppp.h"
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
#include "esp_mac.h"
#include "dhcpserver/dhcpserver.h"
#endif
#include <ds18x20.h>
#include "mqtt_client.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_WIFI_CHANNEL   CONFIG_ESP_WIFI_CHANNEL
#define EXAMPLE_MAX_STA_CONN       CONFIG_ESP_MAX_STA_CONN

#define DEVICE_TEMP_POWER_ON       GPIO_NUM_0
#define DEVICE_BAT_POWER_ON        GPIO_NUM_33
static const gpio_num_t SENSOR_GPIO = 4;
#define SETUP_ADC_CHANNEL ADC_CHANNEL_4 
#define SETUP_ADC_BITWIDTH ADC_BITWIDTH_DEFAULT 
static const char *mqttdestt = "beefarm/collector/c1/temp";
static const char *mqttdestv = "beefarm/collector/c1/vbat";
static const char *MQTTURI = "mqtt://balance:b4l4nc3!@collector.mielediorso.it:2783";

static const char *TAG = "ap_to_pppos";

static EventGroupHandle_t event_group = NULL;
static const int CONNECT_BIT = BIT0;
static const int DISCONNECT_BIT = BIT1;

static void on_ip_event(void *arg, esp_event_base_t event_base,
                        int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "IP event! %" PRId32, event_id);
    if (event_id == IP_EVENT_PPP_GOT_IP) {
        esp_netif_dns_info_t dns_info;
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        esp_netif_t *netif = event->esp_netif;

        ESP_LOGI(TAG, "Modem Connect to PPP Server");
        ESP_LOGI(TAG, "~~~~~~~~~~~~~~");
        ESP_LOGI(TAG, "IP          : " IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "Netmask     : " IPSTR, IP2STR(&event->ip_info.netmask));
        ESP_LOGI(TAG, "Gateway     : " IPSTR, IP2STR(&event->ip_info.gw));
        esp_netif_get_dns_info(netif, 0, &dns_info);
        ESP_LOGI(TAG, "Name Server1: " IPSTR, IP2STR(&dns_info.ip.u_addr.ip4));
        esp_netif_get_dns_info(netif, 1, &dns_info);
        ESP_LOGI(TAG, "Name Server2: " IPSTR, IP2STR(&dns_info.ip.u_addr.ip4));
        ESP_LOGI(TAG, "~~~~~~~~~~~~~~");
        xEventGroupSetBits(event_group, CONNECT_BIT);

        ESP_LOGI(TAG, "GOT ip event!!!");
    } else if (event_id == IP_EVENT_PPP_LOST_IP) {
        ESP_LOGI(TAG, "Modem Disconnect from PPP Server");
        xEventGroupSetBits(event_group, DISCONNECT_BIT);
    } else if (event_id == IP_EVENT_GOT_IP6) {
        ESP_LOGI(TAG, "GOT IPv6 event!");
        ip_event_got_ip6_t *event = (ip_event_got_ip6_t *)event_data;
        ESP_LOGI(TAG, "Got IPv6 address " IPV6STR, IPV62STR(event->ip6_info.ip));
    }
}

static void on_ppp_changed(void *arg, esp_event_base_t event_base,
                           int32_t event_id, void *event_data)
{
    ESP_LOGI(TAG, "PPP state changed event %" PRIu32, event_id);
    if (event_id == NETIF_PPP_ERRORUSER) {
        /* User interrupted event from esp-netif */
        esp_netif_t **p_netif = event_data;
        ESP_LOGI(TAG, "User interrupted event from netif:%p", *p_netif);
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        //rtc_gpio_isolate(GPIO_NUM_12);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;
    case MQTT_EVENT_SUBSCRIBED:
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        break;
    case MQTT_EVENT_PUBLISHED:
        break;
    case MQTT_EVENT_DATA:
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

static void sensor_monitor(esp_mqtt_client_handle_t client)
{
    onewire_addr_t addrs[1];
    size_t sensor_count = 0;
    int vbat;

    gpio_set_direction(DEVICE_TEMP_POWER_ON, GPIO_MODE_OUTPUT);
    gpio_set_level(DEVICE_TEMP_POWER_ON,1);
    vTaskDelay(5000/ portTICK_PERIOD_MS);
    
    gpio_set_pull_mode(SENSOR_GPIO, GPIO_PULLUP_ONLY);
    ds18x20_scan_devices(SENSOR_GPIO, addrs, 1, &sensor_count);
    float temperature = 127;
    ds18x20_measure_and_read(SENSOR_GPIO, addrs[0], &temperature);

    char temperaturec[12];
    sprintf(temperaturec, "%.2f", temperature);
    esp_mqtt_client_publish(client, mqttdestt, temperaturec, 0, 1, 0);

        //-------------ADC1 Init---------------//
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = SETUP_ADC_BITWIDTH,
        .atten = ADC_ATTEN_DB_0,
    };


    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, SETUP_ADC_CHANNEL, &config));
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle,SETUP_ADC_CHANNEL , &vbat));
    char vbatt[12];
    sprintf(vbatt, "%i", vbat);
    esp_mqtt_client_publish(client, mqttdestv, vbatt, 0, 1, 0);

}


void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTTURI,
    };
#if CONFIG_BROKER_URL_FROM_STDIN
    char line[128];

    if (strcmp(mqtt_cfg.broker.address.uri, "FROM_STDIN") == 0) {
        int count = 0;
        printf("Please enter url of mqtt broker\n");
        while (count < 128) {
            int c = fgetc(stdin);
            if (c == '\n') {
                line[count] = '\0';
                break;
            } else if (c > 0 && c < 127) {
                line[count] = c;
                ++count;
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        mqtt_cfg.broker.address.uri = line;
        printf("Broker url: %s\n", line);
    } else {
        ESP_LOGE(TAG, "Configuration mismatch: wrong broker url");
        abort();
    }
#endif /* CONFIG_BROKER_URL_FROM_STDIN */

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
    while(true)
        {
                sensor_monitor(client);
                vTaskDelay(300000/ portTICK_PERIOD_MS);
                ESP_LOGE(TAG, "Send mqtt message");
        }
        

}


static esp_err_t set_dhcps_dns(esp_netif_t *netif, uint32_t addr)
{
    esp_netif_dns_info_t dns;
    dns.ip.u_addr.ip4.addr = addr;
    dns.ip.type = IPADDR_TYPE_V4;
    dhcps_offer_t dhcps_dns_value = OFFER_DNS;
    ESP_ERROR_CHECK(esp_netif_dhcps_option(netif, ESP_NETIF_OP_SET, ESP_NETIF_DOMAIN_NAME_SERVER, &dhcps_dns_value, sizeof(dhcps_dns_value)));
    ESP_ERROR_CHECK(esp_netif_set_dns_info(netif, ESP_NETIF_DNS_MAIN, &dns));
    return ESP_OK;
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}


void wifi_init_softap(void)
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                    ESP_EVENT_ANY_ID,
                    &wifi_event_handler,
                    NULL,
                    NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .ssid_hidden = 1,
            .ssid_len = strlen(EXAMPLE_ESP_WIFI_SSID),
            .channel = EXAMPLE_ESP_WIFI_CHANNEL,
            .password = EXAMPLE_ESP_WIFI_PASS,
            .max_connection = EXAMPLE_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS, EXAMPLE_ESP_WIFI_CHANNEL);
}

void start_network(void)
{
    EventBits_t bits = 0;
    vTaskDelay(pdMS_TO_TICKS(20000));
    /*
    while ((bits & CONNECT_BIT) == 0) {
        
        if (!modem_check_sync()) {
            ESP_LOGE(TAG, "Modem does not respond, maybe in DATA mode? ...exiting network mode");
            modem_stop_network();
            if (!modem_check_sync()) {
                ESP_LOGE(TAG, "Modem does not respond to AT ...restarting");
                modem_reset();
                ESP_LOGI(TAG, "Restarted");
            }
            continue;
        }
        
        if (!modem_check_signal()) {
            ESP_LOGI(TAG, "Poor signal ...will check after 5s");
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }
        if (!modem_start_network()) {
            ESP_LOGE(TAG, "Modem could not enter network mode ...will retry after 10s");
            vTaskDelay(pdMS_TO_TICKS(10000));
            continue;
        }
        bits = xEventGroupWaitBits(event_group, (DISCONNECT_BIT | CONNECT_BIT), pdTRUE, pdFALSE, pdMS_TO_TICKS(30000));
        if (bits & DISCONNECT_BIT) {
            ESP_LOGE(TAG, "Modem got disconnected ...retrying");
            modem_stop_network();
        }
    } */
}

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize esp_netif and default event loop
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    event_group = xEventGroupCreate();

    // Initialize lwip network interface in PPP mode
    esp_netif_config_t ppp_netif_config = ESP_NETIF_DEFAULT_PPP();
    esp_netif_t *ppp_netif = esp_netif_new(&ppp_netif_config);
    assert(ppp_netif);

    // Initialize the PPP network and register for IP event
    ESP_ERROR_CHECK(modem_init_network(ppp_netif));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, on_ip_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(NETIF_PPP_STATUS, ESP_EVENT_ANY_ID, &on_ppp_changed, NULL));

    // Start the PPP network and wait for connection
    start_network();

    // Initialize the AP and setup the NAT
    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
    assert(ap_netif);
    esp_netif_dns_info_t dns;
    ESP_ERROR_CHECK(esp_netif_get_dns_info(ppp_netif, ESP_NETIF_DNS_MAIN, &dns));
    set_dhcps_dns(ap_netif, dns.ip.u_addr.ip4.addr);

    wifi_init_softap();
    ip_napt_enable(_g_esp_netif_soft_ap_ip.ip.addr, 1);
    xTaskCreate(mqtt_app_start,"sensormon", 4096, NULL, 5, NULL);

    // Provide a recovery if disconnection of some kind registered
    while (true) {
        EventBits_t bits = xEventGroupWaitBits(event_group, DISCONNECT_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
        if (bits & DISCONNECT_BIT) {
            modem_stop_network();
            start_network();
            esp_restart();
        }
    }
}
