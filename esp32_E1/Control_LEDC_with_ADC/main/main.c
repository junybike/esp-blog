#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/ledc.h"
#include "esp_log.h"

#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO      GPIO_NUM_2
#define LEDC_CHANNEL        LEDC_CHANNEL_0
#define LEDC_DUTY_RES       LEDC_TIMER_8_BIT    // 0–255
#define LEDC_FREQUENCY      1000                // Hz

#define ADC_PIN ADC1_CHANNEL_6 // GPIO 34
#define LED_PIN GPIO_NUM_2
#define BUTTON_PIN GPIO_NUM_0

static const char *TAG = "APP";
    
void app_main(void)
{
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t channel = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = LEDC_OUTPUT_IO,
        .duty = 0,
    };
    ledc_channel_config(&channel);

	adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC_PIN, ADC_ATTEN_DB_11);

    while (1)
    {
        int val = adc1_get_raw(ADC_PIN);
        // ESP_LOGI(TAG, "ADC: %d", val);
        val = val / 16;
        
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, val);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
