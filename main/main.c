#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"

#define PWM_GPIO        12
#define PWM_FREQ_HZ     5000
#define PWM_RESOLUTION  LEDC_TIMER_13_BIT   // 13-bit resolution (0-8191)
#define PWM_MODE        LEDC_HIGH_SPEED_MODE
#define PWM_TIMER       LEDC_TIMER_0
#define PWM_CHANNEL     LEDC_CHANNEL_0

static const char *TAG = "PWM_APP";

static uint32_t max_duty = 0;

void pwm_init(void)
{
    // Configure timer
    ledc_timer_config_t timer_conf = {
        .speed_mode       = PWM_MODE,
        .duty_resolution  = PWM_RESOLUTION,
        .timer_num        = PWM_TIMER,
        .freq_hz          = PWM_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    // Configure channel
    ledc_channel_config_t channel_conf = {
        .gpio_num       = PWM_GPIO,
        .speed_mode     = PWM_MODE,
        .channel        = PWM_CHANNEL,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = PWM_TIMER,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_conf));

    // Calculate max duty value
    max_duty = (1 << 13) - 1;  // 8191 for 13-bit
}

void set_pwm_duty_percent(float percent)
{
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;

    uint32_t duty = (uint32_t)((percent / 100.0) * max_duty);

    ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, PWM_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, PWM_CHANNEL));

    ESP_LOGI(TAG, "Duty set to %.2f%% (%lu)", percent, duty);
}

void app_main(void)
{
    pwm_init();

    printf("\nESP32 PWM Control (ESP-IDF)\n");
    printf("Enter duty cycle (0-100):\n");

    char input[32];

    while (1)
    {
        if (fgets(input, sizeof(input), stdin) != NULL)
        {
            float duty_percent = atof(input);
            set_pwm_duty_percent(duty_percent * 11);
            printf("Enter duty cycle (0-100):\n");
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
