#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/adc.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "esp_log.h"

#define LED_GPIO GPIO_NUM_25
#define BUTTON_GPIO GPIO_NUM_0
#define ADC_CH ADC1_CHANNEL_6   // GPIO 34

#define DEBOUNCE_US 30000
#define LONG_PRESS_US 500000

#define SLOW_PERIOD_US 1000000
#define FAST_PERIOD_US 250000

#define ADC_PERIOD_MS 200
#define ADC_SAMPLES 16
#define ADC_DARK_TH 1000
#define ADC_BRIGHT_TH 2500

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_HIGH_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_GPIO LED_GPIO

#define LEDC_DUTY_RES LEDC_TIMER_13_BIT
#define LEDC_FREQUENCY 5000

typedef enum {
    EVT_BTN_SHORT,
    EVT_BTN_LONG,
    EVT_ADC_UPDATE,
    EVT_ENV_DARK,
    EVT_ENV_BRIGHT,
} app_event_t;

typedef enum {
    LED_CMD_ON,
    LED_CMD_OFF,
    LED_CMD_BLINK_SLOW,
    LED_CMD_BLINK_FAST,
    LED_CMD_SET_BRIGHTNESS
} led_cmd_type_t;

typedef enum {
    BTN_IDLE = 0,
    BTN_PRESSED
} button_state_t;

typedef struct {
    app_event_t type;
    int value;
} app_event_msg_t;

typedef struct {
    led_cmd_type_t type;
    uint32_t duty;
} led_cmd_t;

static uint32_t current_duty = 0;
static bool led_enabled = false;

static button_state_t btn_state = BTN_IDLE;
static int64_t press_time_us = 0;
 
static QueueHandle_t event_queue;
static QueueHandle_t led_queue;

static esp_timer_handle_t blink_timer;
static esp_timer_handle_t debounce_timer;

static uint32_t adc_to_duty(int adc)
{
    const uint32_t max_duty = (1 << 13) - 1;
    return (adc * max_duty) / 4095;
}

static void blink_timer_cb(void *arg)
{
    static bool led_on = false;
    led_on = !led_on;

    uint32_t duty = (led_on && led_enabled) ? current_duty : 0;

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

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

    if (level == 1 && btn_state == BTN_PRESSED)
    {
        int64_t duration = esp_timer_get_time() - press_time_us;
        app_event_msg_t evt = {
            .type = EVT_BTN_SHORT,
            .value = 0
        };

        if (duration >= LONG_PRESS_US) evt.type = EVT_BTN_LONG;

        BaseType_t hp_task_woken = pdFALSE;
        xQueueSendFromISR(event_queue, &evt, &hp_task_woken);
        portYIELD_FROM_ISR(hp_task_woken);

        btn_state = BTN_IDLE;
    }
}

void adc_task(void *arg)
{
    while (1)
    {
        int sum = 0;

        for (int i = 0; i < ADC_SAMPLES; i++)
        {
            sum += adc1_get_raw(ADC_CH);
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        
        int avg = sum / ADC_SAMPLES;

        app_event_msg_t evt = {
            .type = EVT_ADC_UPDATE,
            .value = avg
        };
        xQueueSend(event_queue, &evt, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(ADC_PERIOD_MS));
    }
}

void app_task(void *arg)
{
    app_event_msg_t evt;
    led_cmd_t cmd;
    bool dark = false;

    while (1)
    {
        if (xQueueReceive(event_queue, &evt, portMAX_DELAY))
        {
            switch (evt.type)
            {
                case EVT_BTN_SHORT:
                    led_enabled = !led_enabled;

                    cmd.type = led_enabled ? LED_CMD_BLINK_SLOW : LED_CMD_OFF;
                    xQueueSend(led_queue, &cmd, portMAX_DELAY);

                    ESP_LOGI("APP", "Short press");
                    break;

                case EVT_BTN_LONG:
                    led_enabled = !led_enabled;

                    cmd.type = led_enabled ? LED_CMD_BLINK_FAST : LED_CMD_OFF;
                    xQueueSend(led_queue, &cmd, portMAX_DELAY); 

                    ESP_LOGI("APP", "Long press");
                    break;

                case EVT_ADC_UPDATE:
                    current_duty = cmd.duty;
                    if (led_enabled)
                    {
                        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, current_duty);
                        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                    }
                    /*
                    if (!dark && evt.value < ADC_DARK_TH)
                    {
                        dark = true;
                        app_event_msg_t e = { .type = EVT_ENV_DARK };
                        xQueueSend(event_queue, &e, 0);
                    }
                    else if (dark && evt.value > ADC_BRIGHT_TH)
                    {
                        dark = false;
                        app_event_msg_t e = { .type = EVT_ENV_BRIGHT };
                        xQueueSend(event_queue, &e, 0);
                    }
                    */

                    ESP_LOGI("APP", "ADC value: %d", evt.value);
                    break;

                case EVT_ENV_DARK:
                    ESP_LOGI("APP", "Env: dark");
                    break;

                case EVT_ENV_BRIGHT:
                    ESP_LOGI("APP", "Env: bright");
                    break;
                    
                default:
                    break;
            }
        }
    }
}

void led_task(void *arg)
{
    led_cmd_t cmd;

    while (1)
    {
        if (xQueueReceive(led_queue, &cmd, portMAX_DELAY))
        {
            switch(cmd.type)
            {
                case LED_CMD_ON:
                    esp_timer_stop(blink_timer);
                    gpio_set_level(LED_GPIO, 1);
                    break;

                case LED_CMD_OFF:
                    esp_timer_stop(blink_timer);
                    gpio_set_level(LED_GPIO, 0);
                    break;

                case LED_CMD_BLINK_SLOW:
                    esp_timer_stop(blink_timer);
                    esp_timer_start_periodic(blink_timer, SLOW_PERIOD_US);
                    break;
                    
                case LED_CMD_BLINK_FAST:
                    esp_timer_stop(blink_timer);
                    esp_timer_start_periodic(blink_timer, FAST_PERIOD_US);
                    break;
                
                case LED_CMD_SET_BRIGHTNESS:
                    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, cmd.duty);
                    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                    break;
            }
        }
    }
}

void app_main(void)
{
    gpio_reset_pin(LED_GPIO);
    // gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_GPIO, button_isr_handler, NULL);
    
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_GPIO,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel); 

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC_CH, ADC_ATTEN_DB_11);

    event_queue = xQueueCreate(10, sizeof(app_event_msg_t));
    led_queue = xQueueCreate(4, sizeof(led_cmd_t));

    const esp_timer_create_args_t timer_args = {
        .callback = &blink_timer_cb,
        .name = "blink_timer"
    };
    esp_timer_create(&timer_args, &blink_timer);

    const esp_timer_create_args_t debounce_args = {
        .callback = &debounce_timer_cb,
        .name = "debounce_timer"
    };
    esp_timer_create(&debounce_args, &debounce_timer);

    xTaskCreate(app_task, "app_task", 3072, NULL, 5, NULL);
    xTaskCreate(led_task, "led_task", 2048, NULL, 4, NULL);
    xTaskCreate(adc_task, "adc_task", 2048, NULL, 3, NULL);

    vTaskDelay(portMAX_DELAY);
}

