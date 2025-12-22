#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/uart.h"
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

#define EVT_BTN_SHORT_BIT BIT0
#define EVT_BTN_LONG_BIT BIT1
#define EVT_ADC_UPDATE_BIT BIT2

#define UART_PORT UART_NUM_0
#define UART_RX_BUF_SIZE 1024
#define UART_TX_BUF_SIZE 1024

#define BIN_SOF           0xAA
#define BIN_CMD_ON        0x01
#define BIN_CMD_OFF       0x02
#define BIN_CMD_SLOW      0x03
#define BIN_CMD_FAST      0x04
#define BIN_CMD_BRIGHT    0x05
#define BIN_CMD_STATUS    0x06

#define BIN_CMD_STATUS_RESP 0x80

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
    led_cmd_type_t type;
    uint32_t duty;
} led_cmd_t;

typedef enum {
    SYS_OFF,
    SYS_ACTIVE
} system_state_t;

typedef struct __attribute__((packed)) {
    uint8_t sof;
    uint8_t cmd;
    uint16_t value;
    uint8_t crc;
} bin_pkt_t;

typedef enum {
    BIN_WAIT_SOF,
    BIN_WAIT_CMD,
    BIN_WAIT_VAL_L,
    BIN_WAIT_VAL_H,
    BIN_WAIT_CRC
} bin_state_t;

static bin_state_t bin_state = BIN_WAIT_SOF;
static bin_pkt_t bin_pkt;

static system_state_t sys_state = SYS_OFF;
static EventGroupHandle_t system_events;

static uint32_t adc_duty = 0;
static uint32_t current_duty = 0;
static bool led_enabled = false;
static bool led_blinks = false;

static button_state_t btn_state = BTN_IDLE;
static int64_t press_time_us = 0;
 
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

        if (duration >= LONG_PRESS_US) 
        {
            xEventGroupSetBitsFromISR(system_events, EVT_BTN_LONG_BIT, NULL);
        }
        else
        {
            xEventGroupSetBitsFromISR(system_events, EVT_BTN_SHORT_BIT, NULL);
        }

        BaseType_t hp_task_woken = pdFALSE;
        portYIELD_FROM_ISR(hp_task_woken);

        btn_state = BTN_IDLE;
    }
}

static uint8_t bin_crc(const bin_pkt_t *p)
{
    return p->sof ^ p->cmd ^ (p->value & 0xFF) ^ (p->value >> 8);
}

static void uart_send(const char *s)
{
    uart_write_bytes(UART_PORT, s, strlen(s));
}

static void handle_uart_command(const char *cmd)
{
    if (strcmp(cmd, "on") == 0)
    {
        xEventGroupSetBits(system_events, EVT_BTN_SHORT_BIT);
    }
    else if (strcmp(cmd, "off") == 0)
    {
        xEventGroupSetBits(system_events, EVT_BTN_SHORT_BIT);
    }
    else if (strcmp(cmd, "slow") == 0)
    {
        xEventGroupSetBits(system_events, EVT_BTN_SHORT_BIT);
    }
    else if (strcmp(cmd, "fast") == 0)
    {
        xEventGroupSetBits(system_events, EVT_BTN_LONG_BIT);
    }
    else if (strcmp(cmd, "brightness ") == 0)
    {
        int value = atoi(cmd + 11);
        value = value < 0 ? 0 : value;
        value = value > 4095 ? 4095 : value;

        adc_duty = adc_to_duty(value);
        xEventGroupSetBits(system_events, EVT_ADC_UPDATE_BIT);
    }
    else if (strcmp(cmd, "status") == 0)
    {
        uart_send(sys_state == SYS_ACTIVE ? "ACTIVE\n" : "OFF\n");
    }
    else
    {
        uart_send("ERR\n");
    }
}

static void send_bin_status(void)
{
    bin_pkt_t p = {
        .sof = BIN_SOF,
        .cmd = BIN_CMD_STATUS_RESP,
        .value = (sys_state == SYS_ACTIVE),
    };
    p.crc = bin_crc(&p);
    uart_write_bytes(UART_PORT, (const char *)&p, sizeof(p));
}

static void handle_bin_command(const bin_pkt_t *p)
{
    switch (p->cmd)
    {
        case BIN_CMD_ON:
            xEventGroupSetBits(system_events, EVT_BTN_SHORT_BIT);
            break;

        case BIN_CMD_OFF:
            xEventGroupSetBits(system_events, EVT_BTN_SHORT_BIT);
            break;

        case BIN_CMD_SLOW:
            xEventGroupSetBits(system_events, EVT_BTN_SHORT_BIT);
            break;

        case BIN_CMD_FAST:
            xEventGroupSetBits(system_events, EVT_BTN_LONG_BIT);
            break;

        case BIN_CMD_BRIGHT:
            if (p->value > 4095) return;
            adc_duty = adc_to_duty(p->value);
            xEventGroupSetBits(system_events, EVT_ADC_UPDATE_BIT);
            break;

        case BIN_CMD_STATUS:
            send_bin_status();
            break;

        default:
            break;
    }
}

static void bin_parse_byte(uint8_t b)
{
    switch (bin_state)
    {
        case BIN_WAIT_SOF:
            if (b == BIN_SOF) 
            {
                bin_pkt.sof = b;
                bin_state = BIN_WAIT_CMD;
            }
            break;

        case BIN_WAIT_CMD:
            bin_pkt.cmd = b;
            bin_state = BIN_WAIT_VAL_L;
            break;

        case BIN_WAIT_VAL_L:
            bin_pkt.value = b;
            bin_state = BIN_WAIT_VAL_H;
            break;

        case BIN_WAIT_VAL_H:
            bin_pkt.value |= (b << 8);
            bin_state = BIN_WAIT_CRC;
            break;

        case BIN_WAIT_CRC:
            bin_pkt.crc = b;
            if (bin_crc(&bin_pkt) == bin_pkt.crc) 
            {
                handle_bin_command(&bin_pkt);
            }
            bin_state = BIN_WAIT_SOF;
            break;
    }
}

void uart_rx_task(void *arg)
{
    uint8_t rx_buf[128];
    char line[128];
    int idx = 0;

    while (1)
    {
        int len = uart_read_bytes(
            UART_PORT,
            rx_buf,
            sizeof(rx_buf),
            pdMS_TO_TICKS(100)
        );

        for (int i = 0; i < len; i++)
        {
            char c = rx_buf[i];
            uint8_t b = rx_buf[i];

            if (b == BIN_SOF || bin_state != BIN_WAIT_SOF)
            {
                bin_parse_byte(b);
            }
            else
            {
                if (c == '\n' || c == '\r')
                {
                    line[idx] = '\0';
                    idx = 0;
                    handle_uart_command(line);
                }
                else if (idx < sizeof(line) - 1)
                {
                    line[idx++] = c;
                }
            }
        }
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
        
        adc_duty = adc_to_duty(sum / ADC_SAMPLES);
        xEventGroupSetBits(system_events, EVT_ADC_UPDATE_BIT);

        vTaskDelay(pdMS_TO_TICKS(ADC_PERIOD_MS));
    }
}

void system_task(void *arg)
{
    while (1)
    {
        EventBits_t bits = xEventGroupWaitBits(
            system_events,
            EVT_BTN_SHORT_BIT | EVT_BTN_LONG_BIT | EVT_ADC_UPDATE_BIT,
            pdTRUE,
            pdFALSE,
            portMAX_DELAY
        );

        switch (sys_state)
        {
            case SYS_OFF:
                if (bits & EVT_BTN_SHORT_BIT) {
                    sys_state = SYS_ACTIVE;
                    led_cmd_t cmd = { .type = LED_CMD_BLINK_SLOW };
                    xQueueSend(led_queue, &cmd, 0);
                }
                break;

            case SYS_ACTIVE:
                if (bits & EVT_BTN_SHORT_BIT) {
                    sys_state = SYS_OFF;
                    led_cmd_t cmd = { .type = LED_CMD_OFF };
                    xQueueSend(led_queue, &cmd, 0);
                }

                if (bits & EVT_BTN_LONG_BIT) {
                    led_cmd_t cmd = { .type = LED_CMD_BLINK_FAST };
                    xQueueSend(led_queue, &cmd, 0);
                }

                if ((bits & EVT_ADC_UPDATE_BIT) && sys_state == SYS_ACTIVE) {
                    led_cmd_t cmd = {
                        .type = LED_CMD_SET_BRIGHTNESS,
                        .duty = adc_duty
                    };
                    xQueueSend(led_queue, &cmd, 0);
                }
                break;
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
                    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 1);
                    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                    break;

                case LED_CMD_OFF:
                    led_enabled = false;
                    esp_timer_stop(blink_timer);
                    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
                    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                    break;

                case LED_CMD_BLINK_SLOW:
                    led_enabled = true;
                    led_blinks = false;
                    esp_timer_stop(blink_timer);
                    esp_timer_start_periodic(blink_timer, SLOW_PERIOD_US);
                    break;
                    
                case LED_CMD_BLINK_FAST:
                    led_enabled = true;
                    led_blinks = false;
                    esp_timer_stop(blink_timer);
                    esp_timer_start_periodic(blink_timer, FAST_PERIOD_US);
                    break;
                
                case LED_CMD_SET_BRIGHTNESS:
                    current_duty = cmd.duty;
                    if (led_enabled)
                    {
                        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, current_duty);
                        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                    }
                    break;
            }
        }
    }
}

void app_main(void)
{
    // gpio_reset_pin(LED_GPIO);
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

    uart_config_t ucfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_driver_install(UART_PORT, UART_RX_BUF_SIZE, UART_TX_BUF_SIZE, 0, NULL, 0);
    uart_param_config(UART_PORT, &ucfg);

    system_events = xEventGroupCreate();

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

    xTaskCreate(system_task, "systen_task", 3072, NULL, 5, NULL);
    xTaskCreate(led_task, "led_task", 2048, NULL, 4, NULL);
    xTaskCreate(adc_task, "adc_task", 2048, NULL, 3, NULL);
    xTaskCreate(uart_rx_task, "uart_rx", 3072, NULL, 3, NULL);

    vTaskDelay(portMAX_DELAY);
}

