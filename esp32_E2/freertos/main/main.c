#include <stdio.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_attr.h"

/* Pins */
#define LED_GPIO     GPIO_NUM_25
#define BUTTON_GPIO  GPIO_NUM_0

/* Timing */
#define DEBOUNCE_US      30000      // 30 ms
#define LONG_PRESS_US   500000      // 500 ms

#define BLINK_SLOW_US 1000000       // 1 s
#define BLINK_FAST_US  250000       // 250 ms

/* Logging */
static const char *TAG = "STEP4";

/* Events */
typedef enum {
    EVT_BTN_SHORT,
    EVT_BTN_LONG
} app_event_t;

typedef struct {
    app_event_t type;
} app_event_msg_t;

/* LED Commands */
typedef enum {
    LED_CMD_OFF,
    LED_CMD_BLINK_SLOW,
    LED_CMD_BLINK_FAST
} led_cmd_t;

/* Button State */
typedef enum {
    BTN_IDLE = 0,
    BTN_PRESSED
} button_state_t;

static button_state_t btn_state = BTN_IDLE;
static int64_t press_time_us = 0;

/* Handles */
static QueueHandle_t app_event_queue;
static QueueHandle_t led_cmd_queue;

static esp_timer_handle_t debounce_timer;
static esp_timer_handle_t blink_timer;

/* Blink Timer Callback */
static void blink_timer_cb(void *arg)
{
    static bool led_on = false;
    led_on = !led_on;
    gpio_set_level(LED_GPIO, led_on);
}

/* GPIO ISR */
static void IRAM_ATTR button_isr_handler(void *arg)
{
    esp_timer_start_once(debounce_timer, DEBOUNCE_US);
}

/* Debounce Timer */
static void debounce_timer_cb(void *arg)
{
    int level = gpio_get_level(BUTTON_GPIO);

    // Button pressed (active-low)
    if (level == 0 && btn_state == BTN_IDLE) {
        btn_state = BTN_PRESSED;
        press_time_us = esp_timer_get_time();
    }
    // Button released
    else if (level == 1 && btn_state == BTN_PRESSED) {
        int64_t duration = esp_timer_get_time() - press_time_us;

        app_event_msg_t evt = {
            .type = (duration >= LONG_PRESS_US) ? EVT_BTN_LONG : EVT_BTN_SHORT
        };

        BaseType_t hp_task_woken = pdFALSE;
        xQueueSendFromISR(app_event_queue, &evt, &hp_task_woken);
        portYIELD_FROM_ISR(hp_task_woken);

        btn_state = BTN_IDLE;
    }
}

/* LED Task */
void led_task(void *arg)
{
    led_cmd_t cmd;

    while (1) {
        if (xQueueReceive(led_cmd_queue, &cmd, portMAX_DELAY)) {
            switch (cmd) {

            case LED_CMD_OFF:
                esp_timer_stop(blink_timer);
                gpio_set_level(LED_GPIO, 0);
                ESP_LOGI(TAG, "LED OFF");
                break;

            case LED_CMD_BLINK_SLOW:
                esp_timer_stop(blink_timer);
                esp_timer_start_periodic(blink_timer, BLINK_SLOW_US);
                ESP_LOGI(TAG, "LED BLINK SLOW");
                break;

            case LED_CMD_BLINK_FAST:
                esp_timer_stop(blink_timer);
                esp_timer_start_periodic(blink_timer, BLINK_FAST_US);
                ESP_LOGI(TAG, "LED BLINK FAST");
                break;
            }
        }
    }
}

/* App Task */
void app_task(void *arg)
{
    app_event_msg_t evt;
    bool led_on = false;

    while (1) {
        if (xQueueReceive(app_event_queue, &evt, portMAX_DELAY)) {
            switch (evt.type) {

            case EVT_BTN_SHORT:
                led_on = !led_on;

                led_cmd_t cmd =
                    led_on ? LED_CMD_BLINK_SLOW : LED_CMD_OFF;

                xQueueSend(led_cmd_queue, &cmd, portMAX_DELAY);
                ESP_LOGI(TAG, "SHORT PRESS");
                break;

            case EVT_BTN_LONG:
                led_cmd_t fast = LED_CMD_BLINK_FAST;
                xQueueSend(led_cmd_queue, &fast, portMAX_DELAY);
                ESP_LOGI(TAG, "LONG PRESS");
                break;
            }
        }
    }
}

/* app_main */
void app_main(void)
{
    /* LED GPIO */
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    /* Button GPIO */
    gpio_config_t btn_cfg = {
        .pin_bit_mask = 1ULL << BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    gpio_config(&btn_cfg);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_GPIO, button_isr_handler, NULL);

    /* Queues */
    app_event_queue = xQueueCreate(8, sizeof(app_event_msg_t));
    led_cmd_queue   = xQueueCreate(4, sizeof(led_cmd_t));

    /* Blink timer */
    const esp_timer_create_args_t blink_args = {
        .callback = blink_timer_cb,
        .name = "blink_timer"
    };
    esp_timer_create(&blink_args, &blink_timer);

    /* Debounce timer */
    const esp_timer_create_args_t debounce_args = {
        .callback = debounce_timer_cb,
        .name = "debounce_timer"
    };
    esp_timer_create(&debounce_args, &debounce_timer);

    xTaskCreate(app_task, "app_task", 2048, NULL, 5, NULL);
    xTaskCreate(led_task, "led_task", 2048, NULL, 4, NULL);

    ESP_LOGI(TAG, "System ready");

    vTaskDelay(portMAX_DELAY);
}

