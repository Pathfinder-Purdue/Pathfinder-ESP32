/*
 * VL53L5CX SATEL bring-up for ESP32 (ESP-IDF).
 *
 * Wiring (SATEL pin -> ESP32):
 * - SDA     -> GPIO defined by VL53L5CX_PIN_SDA
 * - SCL     -> GPIO defined by VL53L5CX_PIN_SCL
 * - INT     -> GPIO defined by VL53L5CX_PIN_INT (optional, input only)
 * - I2C_RST -> GPIO defined by VL53L5CX_PIN_I2C_RST (optional, output)
 * - LPN     -> GPIO defined by VL53L5CX_PIN_LPN (XSHUT/LPn, output)
 * - PWREN   -> GPIO defined by VL53L5CX_PIN_PWREN (optional, output)
 * - AVDD    -> 3V3
 * - IOVDD   -> 3V3
 * - GND     -> GND
 *
 * Notes:
 * - Use one common ground between ESP32 and SATEL board.
 * - I2C address for VL53L5CX is 0x52 in 8-bit form, 0x29 in 7-bit form.
 */

#include <stdio.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "vl53l5cx_api.h"

// Update these GPIOs to match your ESP32 wiring.
static constexpr gpio_num_t VL53L5CX_PIN_SDA = GPIO_NUM_15;
static constexpr gpio_num_t VL53L5CX_PIN_SCL = GPIO_NUM_16;
static constexpr gpio_num_t VL53L5CX_PIN_INT = GPIO_NUM_NC;
static constexpr gpio_num_t VL53L5CX_PIN_I2C_RST = GPIO_NUM_NC;
static constexpr gpio_num_t VL53L5CX_PIN_LPN = GPIO_NUM_5;
static constexpr gpio_num_t VL53L5CX_PIN_PWREN = GPIO_NUM_NC;

static constexpr i2c_port_num_t VL53L5CX_I2C_PORT = I2C_NUM_0;
static constexpr uint32_t VL53L5CX_I2C_SPEED_HZ = 100000;
static constexpr uint32_t VL53L5CX_PROBE_TIMEOUT_MS = 200;

static void setup_optional_pins() {
	if (VL53L5CX_PIN_INT != GPIO_NUM_NC) {
		gpio_config_t int_cfg = {
			.pin_bit_mask = 1ULL << VL53L5CX_PIN_INT,
			.mode = GPIO_MODE_INPUT,
			.pull_up_en = GPIO_PULLUP_DISABLE,
			.pull_down_en = GPIO_PULLDOWN_DISABLE,
			.intr_type = GPIO_INTR_DISABLE,
		};
		ESP_ERROR_CHECK(gpio_config(&int_cfg));
	}

	gpio_num_t outputs[] = {VL53L5CX_PIN_PWREN, VL53L5CX_PIN_I2C_RST,
							VL53L5CX_PIN_LPN};
	for (size_t i = 0; i < sizeof(outputs) / sizeof(outputs[0]); ++i) {
		if (outputs[i] == GPIO_NUM_NC) {
			continue;
		}

		gpio_config_t out_cfg = {
			.pin_bit_mask = 1ULL << outputs[i],
			.mode = GPIO_MODE_OUTPUT,
			.pull_up_en = GPIO_PULLUP_DISABLE,
			.pull_down_en = GPIO_PULLDOWN_DISABLE,
			.intr_type = GPIO_INTR_DISABLE,
		};
		ESP_ERROR_CHECK(gpio_config(&out_cfg));
	}

	// Power-on/reset sequence for optional control pins.
	if (VL53L5CX_PIN_PWREN != GPIO_NUM_NC) {
		ESP_ERROR_CHECK(gpio_set_level(VL53L5CX_PIN_PWREN, 1));
	}

	if (VL53L5CX_PIN_I2C_RST != GPIO_NUM_NC) {
		ESP_ERROR_CHECK(gpio_set_level(VL53L5CX_PIN_I2C_RST, 0));
		vTaskDelay(pdMS_TO_TICKS(2));
		ESP_ERROR_CHECK(gpio_set_level(VL53L5CX_PIN_I2C_RST, 1));
	}

	if (VL53L5CX_PIN_LPN != GPIO_NUM_NC) {
		ESP_ERROR_CHECK(gpio_set_level(VL53L5CX_PIN_LPN, 0));
		vTaskDelay(pdMS_TO_TICKS(5));
		ESP_ERROR_CHECK(gpio_set_level(VL53L5CX_PIN_LPN, 1));
		printf("LPn/XSHUT driven high on GPIO %d\n", VL53L5CX_PIN_LPN);
	}

	vTaskDelay(pdMS_TO_TICKS(20));
}

extern "C" void app_main(void) {
	setup_optional_pins();

	i2c_master_bus_config_t i2c_bus_cfg = {};
	i2c_bus_cfg.i2c_port = VL53L5CX_I2C_PORT;
	i2c_bus_cfg.sda_io_num = VL53L5CX_PIN_SDA;
	i2c_bus_cfg.scl_io_num = VL53L5CX_PIN_SCL;
	i2c_bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
	i2c_bus_cfg.glitch_ignore_cnt = 7;
	i2c_bus_cfg.intr_priority = 0;
	i2c_bus_cfg.trans_queue_depth = 0;
	i2c_bus_cfg.flags.enable_internal_pullup = true;

	i2c_master_bus_handle_t bus_handle = nullptr;
	ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &bus_handle));

	i2c_device_config_t dev_cfg = {};
	dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
	dev_cfg.device_address = (VL53L5CX_DEFAULT_I2C_ADDRESS >> 1);
	dev_cfg.scl_speed_hz = VL53L5CX_I2C_SPEED_HZ;

	esp_err_t probe_err =
		i2c_master_probe(bus_handle, dev_cfg.device_address,
						 pdMS_TO_TICKS(VL53L5CX_PROBE_TIMEOUT_MS));
	if (probe_err != ESP_OK) {
		printf("VL53L5CX probe failed at 0x%02X (%s)\n", dev_cfg.device_address,
			   esp_err_to_name(probe_err));
		return;
	}

	printf("VL53L5CX detected at 0x%02X\n", dev_cfg.device_address);

	static VL53L5CX_Configuration dev = {};
	static VL53L5CX_ResultsData results = {};
	dev.platform.bus_config = i2c_bus_cfg;

	ESP_ERROR_CHECK(
		i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev.platform.handle));

	// If LPN is not externally controlled, keep reset_gpio as GPIO_NUM_NC.
	dev.platform.reset_gpio = VL53L5CX_PIN_LPN;

	uint8_t status = 0;
	uint8_t is_alive = 0;

	status = vl53l5cx_is_alive(&dev, &is_alive);
	if ((status != VL53L5CX_STATUS_OK) || (is_alive == 0)) {
		printf("VL53L5CX not detected (status=%u, alive=%u)\n", status,
			   is_alive);
		return;
	}

	status = vl53l5cx_init(&dev);
	if (status != VL53L5CX_STATUS_OK) {
		printf("vl53l5cx_init failed (status=%u)\n", status);
		return;
	}

	status = vl53l5cx_set_resolution(&dev, VL53L5CX_RESOLUTION_4X4);
	status |= vl53l5cx_set_ranging_mode(&dev, VL53L5CX_RANGING_MODE_AUTONOMOUS);
	status |= vl53l5cx_set_integration_time_ms(&dev, 20);
	status |= vl53l5cx_set_ranging_frequency_hz(&dev, 10);
	if (status != VL53L5CX_STATUS_OK) {
		printf("VL53L5CX config failed (status=%u)\n", status);
		return;
	}

	status = vl53l5cx_start_ranging(&dev);
	if (status != VL53L5CX_STATUS_OK) {
		printf("vl53l5cx_start_ranging failed (status=%u)\n", status);
		return;
	}

	printf("VL53L5CX started. Streaming 4x4 distances in mm.\n");
	printf("Format: r0c0,...,r0c3;r1c0,...,r1c3;r2...;r3...\n");

	while (true) {
		uint8_t data_ready = 0;
		status = vl53l5cx_check_data_ready(&dev, &data_ready);
		if ((status == VL53L5CX_STATUS_OK) && data_ready) {
			status = vl53l5cx_get_ranging_data(&dev, &results);
			if (status == VL53L5CX_STATUS_OK) {
				for (uint8_t row = 0; row < 4; ++row) {
					for (uint8_t col = 0; col < 4; ++col) {
						uint8_t idx = (row * 4U) + col;
						uint16_t mm =
							results
								.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE * idx];
						printf("%u,", mm);
					}
					printf(";");
				}
				printf("\n");
				fflush(stdout);
			}
		}

		vTaskDelay(pdMS_TO_TICKS(5));
	}
}
