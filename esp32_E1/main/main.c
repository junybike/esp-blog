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

#define ADC_PIN ADC1_CHANNEL_6
#define LED_PIN GPIO_NUM_2
#define BUTTON_PIN GPIO_NUM_0

static const char *TAG = "APP";
    
void app_main(void)
{
	adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC_PIN, ADC_ATTEN_DB_11);

    for (int i = 0; i < 5; i++)
    {
        int val = adc1_get_raw(ADC_PIN);
        ESP_LOGI(TAG, "ADC: %d", val);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

}
