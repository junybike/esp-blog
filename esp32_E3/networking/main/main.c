#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"

#include "esp_http_client.h"
#include "mqtt_client.h"

/* CONFIG */

#define WIFI_SSID       "HomeMesh"
#define WIFI_PASS       "87654321"
#define HTTP_POST_URL   "http://httpbin.org/post"
#define MQTT_URI        "mqtt://broker.hivemq.com"

static const char *TAG = "MQTT";

/* EVENT GROUP */

#define WIFI_CONNECTED_BIT BIT0

static EventGroupHandle_t wifi_event_group;
static esp_mqtt_client_handle_t mqtt_client;

/* WIFI EVENT HANDLER */

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        ESP_LOGW("WIFI", "Disconnected. Retrying...");
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI("WIFI", "IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/* WIFI INIT */

static void wifi_init_sta(void)
{
    wifi_event_group = xEventGroupCreate();

    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();

    ESP_LOGI(TAG, "WIFI initialized");
}

/* MQTT EVENT HANDLER */

static void mqtt_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;

    switch (event->event_id)
    {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT connected");
            esp_mqtt_client_subscribe(event->client, "esp32/demo/cmd", 0);
            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT disconnected");
            break;

        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT RX [%.*s]: %.*s",
                     event->topic_len, event->topic,
                     event->data_len, event->data);

            // Example: simple command parsing
            if (strncmp(event->data, "ping", event->data_len) == 0)
            {
                esp_mqtt_client_publish(
                    event->client,
                    "esp32/demo/status",
                    "pong",
                    0, 1, 0);
            }
            break;

        default:
            break;
    }
}

static void mqtt_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_URI,
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(
        mqtt_client,
        ESP_EVENT_ANY_ID,
        mqtt_event_handler,
        NULL
    );

    esp_mqtt_client_start(mqtt_client);
}

/* MQTT PUBLISH TASK */

void mqtt_publish_task(void *arg)
{
    /* Wait for Wi-Fi */
    xEventGroupWaitBits(
        wifi_event_group,
        WIFI_CONNECTED_BIT,
        pdFALSE,
        pdTRUE,
        portMAX_DELAY);

    mqtt_start();

    int count = 0;

    while (1)
    {
        char msg[64];
        snprintf(msg, sizeof(msg),
                 "{\"count\": %d}", count++);

        esp_mqtt_client_publish(
            mqtt_client,
            "esp32/demo/sensor",
            msg,
            0, 1, 0);

        ESP_LOGI(TAG, "Published: %s", msg);

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

/* HTTP POST */

static void http_post(void)
{
    char payload[64];
    snprintf(payload, sizeof(payload), "{\"temperature\": %.2f}", 24.75);

    esp_http_client_config_t config = {
        .url = HTTP_POST_URL,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 5000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);

    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, payload, strlen(payload));

    ESP_LOGI(TAG, "Sending POST: %s", payload);

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "HTTP POST Status = %d, Content-Length = %lld",
                 esp_http_client_get_status_code(client),
                 esp_http_client_get_content_length(client));
    }
    else
    {
        ESP_LOGE(TAG, "HTTP POST failed: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
}

/* HTTP TASK */

void http_task(void *arg)
{
    xEventGroupWaitBits(
        wifi_event_group,
        WIFI_CONNECTED_BIT,
        pdFALSE,
        pdTRUE,
        portMAX_DELAY
    );

    ESP_LOGI(TAG, "WIFI connected. Starting HTTP task");

    while (1)
    {
        http_post();
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void app_main(void)
{
    nvs_flash_init();
    wifi_init_sta();

    xTaskCreate(mqtt_publish_task, "mqtt_task", 4096, NULL, 5, NULL);
    
    // xTaskCreate(http_task, "http_task", 4096, NULL, 5, NULL);
}
