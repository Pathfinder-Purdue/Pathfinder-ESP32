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
 * - If your SATEL board already ties PWREN/I2C_RST/LPN internally, set the
 *   corresponding pin to GPIO_NUM_NC below.
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
static constexpr gpio_num_t VL53L5CX_PIN_SDA = GPIO_NUM_21;
static constexpr gpio_num_t VL53L5CX_PIN_SCL = GPIO_NUM_22;
static constexpr gpio_num_t VL53L5CX_PIN_INT = GPIO_NUM_NC;
static constexpr gpio_num_t VL53L5CX_PIN_I2C_RST = GPIO_NUM_NC;
static constexpr gpio_num_t VL53L5CX_PIN_LPN = GPIO_NUM_5;
static constexpr gpio_num_t VL53L5CX_PIN_PWREN = GPIO_NUM_NC;

static constexpr i2c_port_num_t VL53L5CX_I2C_PORT = I2C_NUM_0;
static constexpr uint32_t VL53L5CX_I2C_SPEED_HZ = 100000;
static constexpr uint32_t VL53L5CX_PROBE_TIMEOUT_MS = 200;

static void print_i2c_line_state() {
	gpio_config_t cfg = {
		.pin_bit_mask = (1ULL << VL53L5CX_PIN_SDA) | (1ULL << VL53L5CX_PIN_SCL),
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = GPIO_PULLUP_ENABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};
	ESP_ERROR_CHECK(gpio_config(&cfg));
	vTaskDelay(pdMS_TO_TICKS(2));

	int sda = gpio_get_level(VL53L5CX_PIN_SDA);
	int scl = gpio_get_level(VL53L5CX_PIN_SCL);
	printf("I2C line state (idle): SDA=%d, SCL=%d (expected both 1)\n", sda,
		   scl);
}

static bool probe_vl53l5cx(i2c_master_bus_handle_t bus_handle,
						   uint16_t addr_7bit) {
	esp_err_t err = i2c_master_probe(bus_handle, addr_7bit,
									 pdMS_TO_TICKS(VL53L5CX_PROBE_TIMEOUT_MS));
	if (err == ESP_OK) {
		printf("I2C probe OK at 0x%02X\n", addr_7bit);
		return true;
	}

	printf("I2C probe failed at 0x%02X (%s)\n", addr_7bit,
		   esp_err_to_name(err));
	return false;
}

static void setup_control_pin(gpio_num_t pin, int level) {
	if (pin == GPIO_NUM_NC) {
		return;
	}
	gpio_config_t cfg = {
		.pin_bit_mask = 1ULL << pin,
		.mode = GPIO_MODE_OUTPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};
	ESP_ERROR_CHECK(gpio_config(&cfg));
	ESP_ERROR_CHECK(gpio_set_level(pin, level));
}

static bool staged_control_pin_probe(i2c_master_bus_handle_t bus_handle,
									 uint16_t addr_7bit) {
	printf("Running control-pin staged probe...\n");

	setup_control_pin(GPIO_NUM_5, 1);
	vTaskDelay(pdMS_TO_TICKS(20));
	printf("  Stage A: LPn=GPIO5 HIGH\n");
	if (probe_vl53l5cx(bus_handle, addr_7bit)) {
		return true;
	}

	setup_control_pin(GPIO_NUM_19, 1);
	vTaskDelay(pdMS_TO_TICKS(20));
	printf("  Stage B: PWREN=GPIO19 HIGH, LPn HIGH\n");
	if (probe_vl53l5cx(bus_handle, addr_7bit)) {
		return true;
	}

	setup_control_pin(GPIO_NUM_18, 0);
	vTaskDelay(pdMS_TO_TICKS(5));
	setup_control_pin(GPIO_NUM_18, 1);
	vTaskDelay(pdMS_TO_TICKS(20));
	printf("  Stage C: I2C_RST pulse on GPIO18 with LPn/PWREN HIGH\n");
	if (probe_vl53l5cx(bus_handle, addr_7bit)) {
		return true;
	}

	printf("  No response after control-pin staged probe\n");
	return false;
}

static void scan_i2c_bus(i2c_master_bus_handle_t bus_handle) {
	printf("I2C scan start...\n");
	bool found = false;
	bool bus_timeout = false;
	for (uint16_t addr = 0x08; addr <= 0x77; ++addr) {
		esp_err_t err = i2c_master_probe(bus_handle, addr, pdMS_TO_TICKS(5));
		if (err == ESP_OK) {
			printf("  Found device at 0x%02X\n", addr);
			found = true;
		} else if (err == ESP_ERR_TIMEOUT) {
			bus_timeout = true;
			break;
		}
	}
	if (bus_timeout) {
		printf("  Scan aborted: probe timeout indicates a bus-level issue\n");
		return;
	}
	if (!found) {
		printf("  No I2C devices responded\n");
	}
}

static bool probe_mapping(i2c_port_num_t port, gpio_num_t sda, gpio_num_t scl,
						  uint16_t addr_7bit) {
	i2c_master_bus_config_t cfg = {};
	cfg.i2c_port = port;
	cfg.sda_io_num = sda;
	cfg.scl_io_num = scl;
	cfg.clk_source = I2C_CLK_SRC_DEFAULT;
	cfg.glitch_ignore_cnt = 7;
	cfg.intr_priority = 0;
	cfg.trans_queue_depth = 0;
	cfg.flags.enable_internal_pullup = true;

	i2c_master_bus_handle_t bus = nullptr;
	esp_err_t err = i2c_new_master_bus(&cfg, &bus);
	if (err != ESP_OK) {
		printf(
			"Mapping test failed to init bus (port=%d, SDA=%d, SCL=%d): %s\n",
			port, sda, scl, esp_err_to_name(err));
		return false;
	}

	err = i2c_master_probe(bus, addr_7bit, pdMS_TO_TICKS(50));
	i2c_del_master_bus(bus);

	if (err == ESP_OK) {
		printf("Mapping test ACK at 0x%02X on port=%d SDA=%d SCL=%d\n",
			   addr_7bit, port, sda, scl);
		return true;
	}

	printf("Mapping test no ACK on port=%d SDA=%d SCL=%d (%s)\n", port, sda,
		   scl, esp_err_to_name(err));
	return false;
}

static void diagnose_alternate_mappings(uint16_t addr_7bit) {
	printf("Running alternate I2C mapping diagnostics...\n");
	bool any = false;
	any |= probe_mapping(I2C_NUM_0, GPIO_NUM_15, GPIO_NUM_16, addr_7bit);
	any |= probe_mapping(I2C_NUM_1, GPIO_NUM_21, GPIO_NUM_22, addr_7bit);
	any |= probe_mapping(I2C_NUM_1, GPIO_NUM_15, GPIO_NUM_16, addr_7bit);
	if (!any) {
		printf("No alternate mapping responded to address 0x%02X\n", addr_7bit);
	}
}

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
	print_i2c_line_state();

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

	printf("I2C bus: port=%d, SDA=%d, SCL=%d, speed=%lu\n", VL53L5CX_I2C_PORT,
		   VL53L5CX_PIN_SDA, VL53L5CX_PIN_SCL,
		   (unsigned long)VL53L5CX_I2C_SPEED_HZ);
	printf("Probing VL53L5CX 7-bit address 0x%02X...\n",
		   dev_cfg.device_address);
	if (!probe_vl53l5cx(bus_handle, dev_cfg.device_address)) {
		staged_control_pin_probe(bus_handle, dev_cfg.device_address);
		scan_i2c_bus(bus_handle);
		i2c_del_master_bus(bus_handle);
		diagnose_alternate_mappings(dev_cfg.device_address);
		printf("VL53L5CX did not ACK. Check power and pull-ups on SDA/SCL.\n");
		printf("Typical pull-ups are 2.2k-10k to 3V3, commonly 4.7k.\n");
		printf("Also verify common GND and that LPn/XSHUT is high.\n");
		return;
	}

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
