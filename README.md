ESP-IDF template app
====================

This is a template application to be used with [Espressif IoT Development Framework](https://github.com/espressif/esp-idf).

Please check [ESP-IDF docs](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html) for getting started instructions.

*Code in this repository is in the Public Domain (or CC0 licensed, at your option.)
Unless required by applicable law or agreed to in writing, this
software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
CONDITIONS OF ANY KIND, either express or implied.*









TOF:
- TOF shall communicate over I2C:

Initialization of I2C can be evinced by initilization code at beginning of TOF_task

(
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
)


- TOF shall read values at a minimum rate of 5 Hz:

Measured via outputs to terminal.
(
)



IMU:
- IMU shall communicate over SPI:

Initialization of SPI can be evinced by use of global constants in BNO08xGlobalTypses.hpp
(
        bno08x_config_t(bool install_isr_service = true)
            : spi_peripheral((spi_host_device_t) CONFIG_ESP32_BNO08x_SPI_HOST)
            , io_mosi(static_cast<gpio_num_t>(CONFIG_ESP32_BNO08X_GPIO_DI))       // default: 23
            , io_miso(static_cast<gpio_num_t>(CONFIG_ESP32_BNO08X_GPIO_SDA))      // default: 19
            , io_sclk(static_cast<gpio_num_t>(CONFIG_ESP32_BNO08X_GPIO_SCL))      // default: 18
            , io_cs(static_cast<gpio_num_t>(CONFIG_ESP32_BNO08X_GPIO_CS))         // default: 33
            , io_int(static_cast<gpio_num_t>(CONFIG_ESP32_BNO08X_GPIO_HINT))      // default: 26
            , io_rst(static_cast<gpio_num_t>(CONFIG_ESP32_BNO08X_GPIO_RST))       // default: 32
            , sclk_speed(static_cast<uint32_t>(CONFIG_ESP32_BNO08X_SCL_SPEED_HZ)) // default: 2MHz
            , install_isr_service(install_isr_service)                            // default: true
)

- Microcontroller shall read values at a minimum rate of 5 Hz:

Measured via outputs to terminal.
(
)



PWM / Motor Drivers:
- Microcontroller outputs PWM to each of the motor drivers:

Initialization of PWM can be evinced by pwm initialization in the PWM_init function
(
    ledc_timer_config_t timer_cfg = {};
    timer_cfg.speed_mode = PWM_MODE;
    timer_cfg.duty_resolution = PWM_RES;
    timer_cfg.timer_num = PWM_TIMER;
    timer_cfg.freq_hz = PWM_FREQ_HZ;
    timer_cfg.clk_cfg = LEDC_AUTO_CLK;
    timer_cfg.deconfigure = false;
)


- Microcontroller takes <= 100ms to power the motor drivers after receiving instructions from onboard computer

Measured via outputs to terminal.
(
)


UART Communication:
- The Microcontroller shall compile data from multiple sensors into a single package:


- The Microcontroller shall communicate with the Onboard Computer over UART:


- The Microcontroller shall send sensor data to the onboard computer:


- The Microcontroller shall receive motor strength data from the onboard computer:

