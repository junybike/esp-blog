#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "esp_log.h"

#define LONG_PRESS_US 500000
#define DEBOUNCE_US 30000
#define LED_GPIO GPIO_NUM_25
#define BUTTON_GPIO GPIO_NUM_0

typedef enum {
    LED_OFF = 0,
    LED_ON
} led_state_t;

typedef enum {
    BTN_IDLE = 0,
    BTN_PRESSED
} button_state_t;

static led_state_t led_state = LED_OFF;
static button_state_t btn_state = BTN_IDLE;

static int64_t press_time_us = 0;

static esp_timer_handle_t debounce_timer;
static esp_timer_handle_t blink_timer;

static TaskHandle_t app_task_handle = NULL;

static void IRAM_ATTR button_isr_handler(void *arg)
{
    esp_timer_start_once(debounce_timer, DEBOUNCE_US);
}

static void debounce_timer_cb(void *arg)
{
    int level = gpio_get_level(BUTTON_GPIO);

    if (level == 0 && btn_state == BTN_IDLE)
    {
        btn_state = BTN_PRESSED;
        press_time_us = esp_timer_get_time();
    }
    else if (level == 1 && btn_state == BTN_PRESSED)
    {
        int64_t duration = esp_timer_get_time() - press_time_us;
        BaseType_t hp_task_woken = pdFALSE;

        if (duration >= LONG_PRESS_US)
        {
            xTaskNotifyFromISR(app_task_handle, 1, eSetValueWithOverwrite, &hp_task_woken);
        }
        else
        {
            xTaskNotifyFromISR(app_task_handle, 0, eSetValueWithOverwrite, &hp_task_woken);
        }

        btn_state = BTN_IDLE;
        portYIELD_FROM_ISR(hp_task_woken);
    }

}

static void blink_timer_cb(void *arg)
{
    static bool led_on = false;
    led_on = !led_on;
    gpio_set_level(LED_GPIO, led_on);
}

void app_task(void* arg)
{
    uint32_t press_type;
    
    while(1)
    {
        xTaskNotifyWait(0, 0xFFFFFFFF, &press_type, portMAX_DELAY);

        if (press_type == 0)
        {
            printf("SHORT PRESS");
             ESP_LOGI("", "short");
        }
        else
        {
            printf("LONG PRESS");
             ESP_LOGI("", "long");
        }
    }
}

void app_main(void)
{
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_GPIO, button_isr_handler, NULL);

    const esp_timer_create_args_t blink_args = {
        .callback = &blink_timer_cb,
        .name = "blink_timer"
    };
    esp_timer_create(&blink_args, &blink_timer);

    const esp_timer_create_args_t debounce_args = {
        .callback = &debounce_timer_cb,
        .name = "debounce_timer"
    };
    esp_timer_create(&debounce_args, &debounce_timer);

    xTaskCreate(app_task, "app_task", 2048, NULL, 5, &app_task_handle);

    vTaskDelay(portMAX_DELAY);
}
