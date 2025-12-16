#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_attr.h"

#define LED_GPIO GPIO_NUM_25
#define BUTTON_GPIO GPIO_NUM_0
#define SLOW_PERIOD_US 1000000
#define FAST_PERIOD_US 250000

static bool led_on = false;
static bool led_enabled = false;
static volatile bool button_pressed = false;
static TaskHandle_t app_task_handle = NULL;

static esp_timer_handle_t blink_timer;
static bool fast_mode = false;

static void timer_callback(void *arg)
{
    led_on = !led_on;
    gpio_set_level(LED_GPIO, led_on);
}
static void IRAM_ATTR button_isr_handler(void *arg)
{
    BaseType_t task_woken = pdFALSE;
    xTaskNotifyFromISR(app_task_handle, 0, eNoAction, &task_woken);
    portYIELD_FROM_ISR(task_woken);
}

void app_task_led_fast_slow(void *arg)
{
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        fast_mode = !fast_mode;
        esp_timer_stop(blink_timer);

        uint64_t period = fast_mode ? FAST_PERIOD_US : SLOW_PERIOD_US;
        esp_timer_start_periodic(blink_timer, period);

        printf("Blink mode: %s\n", fast_mode ? "FAST" : "SLOW");
    }
}

void app_task_led_powerswitch(void *arg)
{
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        led_enabled = !led_enabled;

        if (led_enabled)
        {
            esp_timer_start_periodic(blink_timer, 500000);
            printf("LED ON\n");
        }
        else
        {
            esp_timer_stop(blink_timer);
            gpio_set_level(LED_GPIO, 0);
            printf("LED OFF\n");
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
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_GPIO, button_isr_handler, NULL);

    const esp_timer_create_args_t timer_args = {
        .callback = &timer_callback,
        .name = "blink_timer"
    };

    esp_timer_create(&timer_args, &blink_timer);
    // esp_timer_start_periodic(blink_timer, SLOW_PERIOD_US);

    xTaskCreate(app_task_led_powerswitch, "app_task", 2048, NULL, 5, &app_task_handle);
    vTaskDelay(portMAX_DELAY);
}

