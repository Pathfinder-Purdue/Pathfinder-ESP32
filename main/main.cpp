/*
 * NeoPixel (RGB 3535 / WS2812-class) single LED demo on GPIO38.
 */

#include <stdint.h>
#include <string.h>

#include "driver/rmt_tx.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip_encoder.h"

static const char *TAG = "neopixel";

static constexpr gpio_num_t NEOPIXEL_GPIO = GPIO_NUM_38;
static constexpr uint32_t RMT_LED_RESOLUTION_HZ = 10000000; // 10 MHz
static constexpr uint32_t STEP_DELAY_MS = 500;

// WS2812-class LEDs typically use GRB byte order.
static uint8_t s_pixel[3] = {0, 0, 0};

static void set_pixel_rgb(uint8_t r, uint8_t g, uint8_t b) {
	s_pixel[0] = g;
	s_pixel[1] = r;
	s_pixel[2] = b;
}

extern "C" void app_main(void) {
	ESP_LOGI(TAG, "Init RMT TX on GPIO %d", (int)NEOPIXEL_GPIO);

	rmt_channel_handle_t led_chan = NULL;
	rmt_tx_channel_config_t tx_chan_cfg = {};
	tx_chan_cfg.clk_src = RMT_CLK_SRC_DEFAULT;
	tx_chan_cfg.gpio_num = NEOPIXEL_GPIO;
	tx_chan_cfg.mem_block_symbols = 64;
	tx_chan_cfg.resolution_hz = RMT_LED_RESOLUTION_HZ;
	tx_chan_cfg.trans_queue_depth = 4;
	ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_cfg, &led_chan));

	rmt_encoder_handle_t led_encoder = NULL;
	led_strip_encoder_config_t encoder_cfg = {};
	encoder_cfg.resolution = RMT_LED_RESOLUTION_HZ;
	ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&encoder_cfg, &led_encoder));

	ESP_ERROR_CHECK(rmt_enable(led_chan));

	rmt_transmit_config_t tx_cfg = {};
	tx_cfg.loop_count = 0;

	ESP_LOGI(TAG, "NeoPixel color cycle started");
	while (true) {
		set_pixel_rgb(255, 0, 0); // red
		ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, s_pixel,
									 sizeof(s_pixel), &tx_cfg));
		ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
		vTaskDelay(pdMS_TO_TICKS(STEP_DELAY_MS));

		set_pixel_rgb(0, 255, 0); // green
		ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, s_pixel,
									 sizeof(s_pixel), &tx_cfg));
		ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
		vTaskDelay(pdMS_TO_TICKS(STEP_DELAY_MS));

		set_pixel_rgb(0, 0, 255); // blue
		ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, s_pixel,
									 sizeof(s_pixel), &tx_cfg));
		ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
		vTaskDelay(pdMS_TO_TICKS(STEP_DELAY_MS));

		set_pixel_rgb(255, 255, 255); // white
		ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, s_pixel,
									 sizeof(s_pixel), &tx_cfg));
		ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
		vTaskDelay(pdMS_TO_TICKS(STEP_DELAY_MS));

		memset(s_pixel, 0, sizeof(s_pixel)); // off
		ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, s_pixel,
									 sizeof(s_pixel), &tx_cfg));
		ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
		vTaskDelay(pdMS_TO_TICKS(STEP_DELAY_MS));
	}
}
