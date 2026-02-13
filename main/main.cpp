//#include <stdint.h>
#include <stdio.h>
//#include <string.h>        // <-- important for strlen
#include "BNO08xGlobalTypes.hpp"
#include "driver/spi_common.h"
#include "driver/uart.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "hal/gpio_types.h"
#include "hal/spi_types.h"
#include "BNO08x.hpp"
#include <cstdio>
#include <cstdint>
#include <cstring>

#define UART_PORT_NUM      UART_NUM_1
#define UART_BAUD_RATE     9600
#define UART_TX_PIN        17   // U1TXD on ESP32-S3-DevKitC-1
#define UART_RX_PIN        18   // U1RXD on ESP32-S3-DevKitC-1
#define UART_BUF_SIZE      1024

#define UART2_PORT_NUM      UART_NUM_2
#define UART2_BAUD_RATE     9600
#define UART2_TX_PIN        21   // U2TXD on ESP32-S3-DevKitC-1
#define UART2_RX_PIN        47   // U2RXD on ESP32-S3-DevKitC-1
#define UART2_BUF_SIZE      1024

#define PIN_NUM_MOSI 11
#define PIN_NUM_MISO 13
#define PIN_NUM_CLK 12
#define PIN_NUM_CS 10
#define PIN_NUM_INT 9
#define PIN_NUM_RST 8

#define MOTOR_PWM_GPIO 40

#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_CHANNEL LEDC_CHANNEL_0

#define PWM_FREQ_HZ 20000
#define PWM_RES LEDC_TIMER_10_BIT


#define I2C_MASTER_NUM        I2C_NUM_0
#define I2C_MASTER_SDA_IO    15
#define I2C_MASTER_SCL_IO    16
#define I2C_MASTER_FREQ_HZ   400000   // u-blox supports 400 kHz
#define I2C_MASTER_TIMEOUT_MS 1000

#define UBLOX_I2C_ADDR       0x42     // 7-bit address


static const uint32_t MAX_DUTY = (1U << 10) - 1;




static spi_device_handle_t spi_handle; // accessible by task

static const char *TAG = "UART_TEST";
QueueHandle_t uart_queue;
QueueHandle_t uart2_queue;

/*
// Motor PWM initialization
static void motor_pwm_init(void)
{
    ledc_timer_config_t timer_cfg = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = PWM_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = PWM_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK,
        .deconfigure      = false
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    ledc_channel_config_t ch_cfg = {
        .gpio_num       = MOTOR_PWM_GPIO,
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER,
        .duty           = 0,
        .hpoint         = 0,
        .sleep_mode     = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
        .flags          = { .output_invert = 0 }
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg));
}

// PWM motor speed function
static void motor_set_speed_percent(int percent)
{
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;

    uint32_t duty = (uint32_t)((percent * (int)MAX_DUTY) / 100);

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
}

// PWM ramping function
static void motor_ramp_to(int target_percent, int step_percent, int step_ms)
{
    static int current = 0;

    if (target_percent < 0) target_percent = 0;
    if (target_percent > 100) target_percent = 100;
    if (step_percent <= 0) step_percent = 1;

    while (current != target_percent) {
        if (current < target_percent) current += step_percent;
        else current -= step_percent;

        if (current > 100) current = 100;
        if (current < 0)   current = 0;

        motor_set_speed_percent(current);
        vTaskDelay(pdMS_TO_TICKS(step_ms));
    }
}

// PWM motor task
static void motor_task(void *arg)
{
    while (1) {
        motor_ramp_to(60, 2, 20);               // ramp to 60%
        vTaskDelay(pdMS_TO_TICKS(2000));

        motor_ramp_to(20, 2, 20);               // slow down
        vTaskDelay(pdMS_TO_TICKS(1500));

        motor_ramp_to(0, 3, 20);                // stop
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
*/

// GPS UART initialization
static void uart_init(void)
{
    // 1) Install driver with BOTH RX and TX buffers
    ESP_ERROR_CHECK(uart_driver_install(
        UART_PORT_NUM,
        UART_BUF_SIZE * 2,   // RX buffer
        UART_BUF_SIZE * 2,   // TX buffer
        10,
        &uart_queue,
        0
    ));

    // 2) Configure UART parameters
    uart_config_t uart_config{};  // zero-initialize the whole struct

    uart_config.baud_rate           = UART_BAUD_RATE;
    uart_config.data_bits           = UART_DATA_8_BITS;
    uart_config.parity              = UART_PARITY_DISABLE;
    uart_config.stop_bits           = UART_STOP_BITS_1;
    uart_config.flow_ctrl           = UART_HW_FLOWCTRL_DISABLE;
    uart_config.rx_flow_ctrl_thresh = 0;
    uart_config.source_clk          = UART_SCLK_DEFAULT;
    //uart_config.flags               = 0;   // plus the nested backup struct is 0 as well


    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));

    // 3) Route UART signals to the pins
    ESP_ERROR_CHECK(uart_set_pin(
        UART_PORT_NUM,
        UART_TX_PIN,
        UART_RX_PIN,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    ));

    // internal loopback, so I don't need a wire
    
    //ESP_ERROR_CHECK(uart_set_loop_back(UART_PORT_NUM, true));
}

// GPS UART task
static void uart_task(void *pvParameters)
{
    uint8_t data[128];

    while (1) {
        size_t buffered_len = 0;
        ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_PORT_NUM, &buffered_len));

        // Clamp to buffer size so we don't overflow 'data'
        if (buffered_len > sizeof(data)) {
            buffered_len = sizeof(data);
        }

        int len = uart_read_bytes(
            UART_PORT_NUM,
            data,
            buffered_len,
            100 / portTICK_PERIOD_MS
        );

        // NOTE: use %zu for size_t in C++
        std::printf("Buffer Length: %zu\n", buffered_len);

        if (len > 0) {
            // len is an int, so %d is correct
            ESP_LOGI(TAG, "RX %d bytes", len);
            std::printf("UART RX: ");
            std::fwrite(data, 1, len, stdout);
            std::printf("\n");
        } else {
            ESP_LOGI(TAG, "No data received");
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}


// GPS I2C initialization
static void i2c_init(void)
{
    i2c_config_t conf;
    memset(&conf, 0, sizeof(conf));

    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
    conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
}

// GPS I2C Task
static void i2c_gps_task(void *pvParameters)
{
    uint8_t buffer[128];

    while (1) {
        esp_err_t err = i2c_master_read_from_device(
            I2C_MASTER_NUM,
            UBLOX_I2C_ADDR,
            buffer,
            sizeof(buffer),
            pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)
        );

        if (err == ESP_OK) {
            printf("I2C RX: ");
            fwrite(buffer, 1, sizeof(buffer), stdout);
            printf("\n");
        } else if (err == ESP_ERR_TIMEOUT) {
            ESP_LOGW("I2C", "Read timeout");
        } else {
            ESP_LOGE("I2C", "Read failed: %s", esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}


// ESP-RPI uart init
static void uart_init_2(void)
{
    // 1) Install driver with BOTH RX and TX buffers
    ESP_ERROR_CHECK(uart_driver_install(
        UART2_PORT_NUM,
        UART2_BUF_SIZE * 2,   // RX buffer
        UART2_BUF_SIZE * 2,   // TX buffer
        10,
        &uart2_queue,
        0
    ));

    // 2) Configure UART parameters
    uart_config_t uart2_config{};  // zero-initialize the whole struct

    uart2_config.baud_rate           = UART2_BAUD_RATE;
    uart2_config.data_bits           = UART_DATA_8_BITS;
    uart2_config.parity              = UART_PARITY_DISABLE;
    uart2_config.stop_bits           = UART_STOP_BITS_1;
    uart2_config.flow_ctrl           = UART_HW_FLOWCTRL_DISABLE;
    uart2_config.rx_flow_ctrl_thresh = 0;
    uart2_config.source_clk          = UART_SCLK_DEFAULT;
    //uart_config.flags               = 0;   // plus the nested backup struct is 0 as well


    ESP_ERROR_CHECK(uart_param_config(UART2_PORT_NUM, &uart2_config));

    // 3) Route UART signals to the pins
    ESP_ERROR_CHECK(uart_set_pin(
        UART2_PORT_NUM,
        UART2_TX_PIN,
        UART2_RX_PIN,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    ));

    // internal loopback, so I don't need a wire
    
//    ESP_ERROR_CHECK(uart_set_loop_back(UART2_PORT_NUM, true));
}

// ESP-RPI Task
static void uart_task_2(void *pvParameters)
{
	float ToF_data [16] = {0.001, 0.11, 0.0, 0.22, 1.54, 2.15, 55.6, 889, 637.34, 88.2, 1, 1, 1, 1, 1, 1};
	float IMU_data [3] = {0, 23, 0};
	float GPS_data [2] = {42.3588337, -71.0578303};
    uint8_t data2[256];
    uint8_t msg[256];

    while (1) {
        size_t buffered_len2 = 0;
        
        int msg_len = snprintf((char *)msg, sizeof(msg),
                           "TOF: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, "
                           "%.3f, %.3f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f; "
                           "IMU: %.2f, %.2f, %.2f; "
                           "GPS: %.7f, %.7f\n",

                           ToF_data[0], ToF_data[1], ToF_data[2], ToF_data[3],
                           ToF_data[4], ToF_data[5], ToF_data[6], ToF_data[7],
                           ToF_data[8], ToF_data[9], ToF_data[10], ToF_data[11],
                           ToF_data[12], ToF_data[13], ToF_data[14], ToF_data[15],

                           IMU_data[0], IMU_data[1], IMU_data[2],

                           GPS_data[0], GPS_data[1]);

        int written = uart_write_bytes(UART2_PORT_NUM, (const char *)msg, etl::strlen(msg));
        
        
        ESP_ERROR_CHECK(uart_get_buffered_data_len(UART2_PORT_NUM, &buffered_len2));

        // Clamp to buffer size so we don't overflow 'data'
        if (buffered_len2 > sizeof(data2)) {
            buffered_len2 = sizeof(data2);
        }

        int len = uart_read_bytes(
            UART2_PORT_NUM,
            data2,
            buffered_len2,
            100 / portTICK_PERIOD_MS
        );

        // NOTE: use %zu for size_t in C++
        std::printf("Buffer 2 Length: %zu\n", buffered_len2);

        if (len > 0) {
            // len is an int, so %d is correct
            ESP_LOGI(TAG, "RX2 %d bytes", len);
            std::printf("UART2 RX: ");
            std::fwrite(data2, 1, len, stdout);
            std::printf("\n");
        } else {
            ESP_LOGI(TAG, "No data received");
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}



static BNO08x imu;

extern "C" void app_main(void)
{
	//motor_pwm_init();
    //xTaskCreate(motor_task, "motor_task", 2048, NULL, 5, NULL);
	
	i2c_init();

    xTaskCreate(i2c_gps_task,"i2c_gps_task",4096,nullptr,10,nullptr);
	
    //uart_init();
    //uart_init_2();
    //spi_init();
//    bno_gpio_init();

	// Create UART Task
    //xTaskCreate(uart_task, "uart_task", 4096, NULL, 5, NULL);
    //xTaskCreate(uart_task_2, "uart2_task", 4096, NULL, 5, NULL);
    


/*
	//// IMU Tasks
	bool imu_ok = imu.initialize();
	if (!imu_ok) {
	    ESP_LOGE(TAG, "BNO08x init failure, continuing without IMU");
	}

	
	// Enable useful reports
	imu.rpt.rv_game.enable(100000UL); // game rotation vector
	imu.rpt.cal_gyro.enable(100000UL); // calibrated gyro
   

    // Let FreeRTOS scheduler handle rest
    while (true) {
		if (imu_ok && imu.data_available()) {
			// get orientation
			if (imu.rpt.rv_game.has_new_data()) {
				bno08x_euler_angle_t euler = imu.rpt.rv_game.get_euler();
				ESP_LOGI(TAG, "Euler: roll = %.2f, pitch = %.2f, yaw = %.2f", euler.x, euler.y, euler.z);
			}
			
			// get angular velocity
			if (imu.rpt.cal_gyro.has_new_data()) {
				bno08x_gyro_t g = imu.rpt.cal_gyro.get();
				ESP_LOGI(TAG, "Gyro: x = %.2f, y = %.2f, z = %.2f rad/s", g.x, g.y, g.z);
			}
		}
		else {
			ESP_LOGI(TAG, "IMU Not working");
		}
		vTaskDelay(pdMS_TO_TICKS(10));
    }
   */ 
    

}

// GPIO pin testing code
/*
void app_main(void)
{
    gpio_reset_pin(18);
    gpio_set_direction(18, GPIO_MODE_OUTPUT);

    while (1) {
        gpio_set_level(18, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(18, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
*/

