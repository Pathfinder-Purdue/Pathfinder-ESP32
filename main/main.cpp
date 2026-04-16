#include <stdint.h>
//#include <stdio.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>        // <-- important for strlen
#include "BNO08xGlobalTypes.hpp"
#include "driver/spi_common.h"
#include "driver/uart.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "driver/i2c_master.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
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
#include <cstdlib>
#include "portmacro.h"
#include "soc/gpio_num.h"
#include "vl53l5cx_api.h"
#include "esp_check.h"
#include "esp_log.h"
#include "led_strip_encoder.h"


#define UART_PORT_NUM      UART_NUM_1
#define UART_BAUD_RATE     9600
#define UART_TX_PIN        17   // U1TXD on ESP32-S3-DevKitC-1
#define UART_RX_PIN        18   // U1RXD on ESP32-S3-DevKitC-1
#define UART_BUF_SIZE      1024

#define UART2_PORT_NUM      UART_NUM_2
#define UART2_BAUD_RATE     115200
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


#define I2C_MASTER_NUM        I2C_NUM_0
#define I2C_MASTER_SDA_IO    15
#define I2C_MASTER_SCL_IO    16
#define I2C_MASTER_FREQ_HZ   400000   // u-blox supports 400 kHz
#define I2C_MASTER_TIMEOUT_MS 1000

//#define UBLOX_I2C_ADDR       0x42     // 7-bit address
#define UBLOX_I2C_ADDR 0x21

#define PWM_GPIO_1        35
#define PWM_GPIO_2        36
#define PWM_GPIO_3        37
#define PWM_GPIO_4        39
#define PWM_GPIO_5        40
#define PWM_FREQ_HZ     5000
#define PWM_RES         LEDC_TIMER_10_BIT   // 0..1023
#define PWM_MODE        LEDC_LOW_SPEED_MODE
#define PWM_TIMER       LEDC_TIMER_0
#define PWM_CHANNEL_0     LEDC_CHANNEL_0
#define PWM_CHANNEL_1     LEDC_CHANNEL_1
#define PWM_CHANNEL_2     LEDC_CHANNEL_2
#define PWM_CHANNEL_3     LEDC_CHANNEL_3
#define PWM_CHANNEL_4     LEDC_CHANNEL_4


static const char *TAG2 = "neopixel";

static constexpr gpio_num_t NEOPIXEL_GPIO = GPIO_NUM_48;
static constexpr uint32_t RMT_LED_RESOLUTION_HZ = 10000000; // 10 MHz
static constexpr uint32_t STEP_DELAY_MS = 500;




// Create Queues
typedef struct {
    uint8_t length;
    uint8_t payload[32];
} SensorMessage;

typedef struct {
    uint8_t length;
    uint8_t payload[5];
    int64_t rx_time_us;
} PWMMessage;

typedef struct {
    uint8_t euler_length;
    uint8_t g_length;
    float payload[6];
} IMUMessage;

typedef struct {
    uint8_t length;
    uint16_t payload[16];
} TOFMessage;

typedef struct {
    uint8_t length;
    double payload[2];
    char time[16];
} GPSMessage;

typedef struct {
    SensorMessage messages[3];
} SensorBatch;

QueueHandle_t sensorQueue;


// From nonfunctional GPS integration
/*
static i2c_master_bus_handle_t g_i2c_bus = nullptr;
static i2c_master_dev_handle_t g_gps_dev = nullptr;
static i2c_master_dev_handle_t g_tof_dev = nullptr;
*/


// From functional TOF code
static constexpr gpio_num_t VL53L5CX_PIN_SDA = GPIO_NUM_15;
static constexpr gpio_num_t VL53L5CX_PIN_SCL = GPIO_NUM_16;
static constexpr gpio_num_t VL53L5CX_PIN_INT = GPIO_NUM_NC;
static constexpr gpio_num_t VL53L5CX_PIN_I2C_RST = GPIO_NUM_NC;
static constexpr gpio_num_t VL53L5CX_PIN_LPN = GPIO_NUM_7;
static constexpr gpio_num_t VL53L5CX_PIN_PWREN = GPIO_NUM_NC;

static constexpr i2c_port_num_t VL53L5CX_I2C_PORT = I2C_NUM_0;
static constexpr uint32_t VL53L5CX_I2C_SPEED_HZ = 100000;
static constexpr uint32_t VL53L5CX_PROBE_TIMEOUT_MS = 200;


static const uint32_t MAX_DUTY = (1U << 10) - 1;




static spi_device_handle_t spi_handle; // accessible by task

static const char *TAG = "UART_TEST";
QueueHandle_t uart_queue;
QueueHandle_t uart2_queue;


// Declare Queues
QueueHandle_t GPSQueue;
QueueHandle_t IMUQueue;
QueueHandle_t TOFQueue;
QueueHandle_t PWMQueue;
QueueHandle_t DRIVERQueue;




/// -------------------------------------------------------------------------------------------------------------- ///

// LED Activation

// WS2812-class LEDs typically use GRB byte order.
static uint8_t s_pixel[3] = {0, 0, 0};

static void set_pixel_rgb(uint8_t r, uint8_t g, uint8_t b) {
	s_pixel[0] = g;
	s_pixel[1] = r;
	s_pixel[2] = b;
}

static void RGB_task(void *pvParameters) {
	ESP_LOGI(TAG2, "Init RMT TX on GPIO %d", (int)NEOPIXEL_GPIO);

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


/// -------------------------------------------------------------------------------------------------------------- ///
// GPS UART data parsing function
bool parse_rmc_minimal(const char *line, double &lat, double &lon, char *time_out)
{
    if (strncmp(line, "$GNRMC", 6) != 0) return false;

    char copy[128];
    strcpy(copy, line);

    char *token = strtok(copy, ",");
    int field = 0;

    const char *time = nullptr;
    const char *lat_str = nullptr;
    const char *lon_str = nullptr;
    char lat_hemi = 0;
    char lon_hemi = 0;

    while (token != nullptr) {
        switch (field) {
            case 1: time = token; break;
            case 2: if (token[0] != 'A'){
				lat = 0;
				lon = 0;
				strcpy(time_out, "0");
				return true;
			} break; // invalid fix
            case 3: lat_str = token; break;
            case 4: lat_hemi = token[0]; break;
            case 5: lon_str = token; break;
            case 6: lon_hemi = token[0]; break;
        }
        token = strtok(nullptr, ",");
        field++;
    }

    if (!lat_str || !lon_str) return false;

    // Convert to decimal
    auto convert = [](const char *coord, char hemi) {
        double val = atof(coord);
        int deg = (int)(val / 100);
        double min = val - deg * 100;
        double dec = deg + min / 60.0;
        return (hemi == 'S' || hemi == 'W') ? -dec : dec;
    };

    lat = convert(lat_str, lat_hemi);
    lon = convert(lon_str, lon_hemi);

    if (time && time_out) {
        strcpy(time_out, time);
    }

    return true;
}

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
    uint8_t rx_chunk[256];
    char line_buffer[1024];
    size_t line_len = 0;
    
    GPSMessage GPS_data_in;

    while (1) {
        int len = uart_read_bytes(
            UART_PORT_NUM,
            rx_chunk,
            sizeof(rx_chunk),
            pdMS_TO_TICKS(100)
        );

        if (len > 0) {
            for (int i = 0; i < len; i++) {
                char c = (char)rx_chunk[i];

                if (line_len < sizeof(line_buffer) - 1) {
                    line_buffer[line_len++] = c;
                } else {
                    // overflow protection: reset buffer
                    line_len = 0;
                }

                if (c == '\n') {
                    line_buffer[line_len] = '\0';

                    //ESP_LOGI(TAG, "GPS line: %s", line_buffer);

					if (parse_rmc_minimal(line_buffer, GPS_data_in.payload[1], GPS_data_in.payload[1], GPS_data_in.time)) {
    					//ESP_LOGI("GPS", "Time=%s Lat=%.6f Lon=%.6f", GPS_data_in.payload[1], GPS_data_in.payload[1], GPS_data_in.time);
    					
    					GPS_data_in.length = 3;
    					
    					xQueueOverwrite(GPSQueue, &GPS_data_in);
					}

                    line_len = 0;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}




/// -------------------------------------------------------------------------------------------------------------- ///





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
	uint16_t TOF_data [16] = {0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1};
	float IMU_data [6] = {0, 23, 0, 0, 0, 0};
	double GPS_data [2] = {42.3588337, -71.0578303};
	char GPS_time [16] = "30301.00";
    uint8_t received_data[256];
    uint8_t msg[256];
    int inc = 0;
    
    
    GPSMessage GPS_data_out;
    IMUMessage IMU_data_out;
    TOFMessage TOF_data_out;
    PWMMessage PWM_data_in;
    



    while (1) {
		
		
        size_t buffered_len2 = 0;
        


        // Retrieve IMU data from queue
        xQueuePeek(IMUQueue, &IMU_data_out, 0);
        
        if (IMU_data_out.g_length == 3 && IMU_data_out.euler_length == 3) {
			IMU_data[0] = IMU_data_out.payload[0];
			IMU_data[1] = IMU_data_out.payload[1];
			IMU_data[2] = IMU_data_out.payload[2];
			IMU_data[3] = IMU_data_out.payload[3];
			IMU_data[4] = IMU_data_out.payload[4];
			IMU_data[5] = IMU_data_out.payload[5];
		}
		else {
			ESP_LOGE(TAG, "IMU Queue Data Size Incorrect");
		}
		
		// Retrieve GPS data from queue
		xQueuePeek(GPSQueue, &GPS_data_out, 0);
		
		if (GPS_data_out.length == 3) {
			strcpy(GPS_time, GPS_data_out.time);
			GPS_data[0] = GPS_data_out.payload[0];
			GPS_data[1] = GPS_data_out.payload[1];
		}
		else{
			ESP_LOGE(TAG, "GPS Queue Data Size Incorrect");
			
			strcpy(GPS_time, GPS_data_out.time);
			GPS_data[0] = GPS_data_out.payload[0];
			GPS_data[1] = GPS_data_out.payload[1];
		}
		
		ESP_LOGI(TAG, "GP to RPI: %s %f %f", GPS_time, GPS_data_out.payload[0], GPS_data_out.payload[1]);
		
		
		// Retrieve TOF data from queue
		xQueuePeek(TOFQueue, &TOF_data_out, 0);
		
		if (TOF_data_out.length == 16) {
			memcpy(TOF_data, TOF_data_out.payload, sizeof(TOF_data));
			//ESP_LOGI(TAG, "TOF to RPI: %u %u %u %u; %u %u %u %u; %u %u %u %u; %u %u %u %u", TOF_data_out.payload[0], TOF_data_out.payload[1], TOF_data_out.payload[2], TOF_data_out.payload[3], TOF_data_out.payload[4], TOF_data_out.payload[5], TOF_data_out.payload[6], TOF_data_out.payload[7], TOF_data_out.payload[8], TOF_data_out.payload[9], TOF_data_out.payload[10], TOF_data_out.payload[11], TOF_data_out.payload[12], TOF_data_out.payload[13], TOF_data_out.payload[14], TOF_data_out.payload[15]);
		}
		else {
			ESP_LOGE(TAG, "TOF Queue Data Size Incorrect");
		}
        
        //ESP_LOGI(TAG, "IMU to RPI: %.2f %.2f %.2f %.2f %.2f %.2f", IMU_data_out.payload[0], IMU_data_out.payload[1], IMU_data_out.payload[2], IMU_data_out.payload[3], IMU_data_out.payload[4], IMU_data_out.payload[5]);
        
        
        inc = (inc + 1) % 101;
        
        int msg_len = snprintf((char *)msg, sizeof(msg),
                           "Inc: %d; "
                           "TOF: %u, %u, %u, %u, %u, %u, %u, %u, "
                           "%u, %u, %u, %u, %u, %u, %u, %u; "
                           "IMU: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f; "
                           "GPS: %s, %.7f, %.7f\n",

                           inc,
                           TOF_data[0], TOF_data[1], TOF_data[2], TOF_data[3],
                           TOF_data[4], TOF_data[5], TOF_data[6], TOF_data[7],
                           TOF_data[8], TOF_data[9], TOF_data[10], TOF_data[11],
                           TOF_data[12], TOF_data[13], TOF_data[14], TOF_data[15],

                           IMU_data[0], IMU_data[1], IMU_data[2], IMU_data[3], IMU_data[4], IMU_data[5],

                           GPS_time, GPS_data[0], GPS_data[1]);

        int written = uart_write_bytes(UART2_PORT_NUM, (const char *)msg, etl::strlen(msg));
        
        
        ESP_ERROR_CHECK(uart_get_buffered_data_len(UART2_PORT_NUM, &buffered_len2));

        // Clamp to buffer size so we don't overflow 'data'
        if (buffered_len2 > sizeof(received_data)) {
            buffered_len2 = sizeof(received_data);
        }

        int len = uart_read_bytes(
            UART2_PORT_NUM,
            received_data,
            buffered_len2,
            100 / portTICK_PERIOD_MS
        );

        // NOTE: use %zu for size_t in C++
        std::printf("Buffer 2 Length: %zu\n", buffered_len2);

        if (len > 0) {
			// Display recieved data in terminal
            // len is an int, so %d is correct
            ESP_LOGI(TAG, "RX2 %d bytes", len);
            std::printf("UART2 RX: ");
            std::fwrite(received_data, 1, len, stdout);
            std::printf("\n");
           
            // Parse recieved data
            if (len >= sizeof(received_data)) len = sizeof(received_data) - 1;
    		received_data[len] = '\0';
    		
		    int p1, p2, p3, p4, p5;
		
		    int parsed = sscanf((char *)received_data, "%d,%d,%d,%d,%d",
		                        &p1, &p2, &p3, &p4, &p5);
		
		    if (parsed == 5) {
		        PWM_data_in.length = 5;
		
		        PWM_data_in.payload[0] = (uint8_t)p1;
		        PWM_data_in.payload[1] = (uint8_t)p2;
		        PWM_data_in.payload[2] = (uint8_t)p3;
		        PWM_data_in.payload[3] = (uint8_t)p4;
		        PWM_data_in.payload[4] = (uint8_t)p5;
		        PWM_data_in.rx_time_us = esp_timer_get_time();
		       
		
		
				// Display what was parsed
				ESP_LOGI(TAG, "Raw Data: %s", (char*)received_data);
		        ESP_LOGI(TAG, "Parsed: %d %d %d %d %d", p1, p2, p3, p4, p5);
		    }
		    else {
        		ESP_LOGW(TAG, "Parse failed: %s", (char*)received_data);
        		
	            PWM_data_in.payload[0] = 0;
				PWM_data_in.payload[1] = 0;
				PWM_data_in.payload[2] = 0;
				PWM_data_in.payload[3] = 0;
				PWM_data_in.payload[4] = 0;
        		PWM_data_in.length = 0;
        		PWM_data_in.rx_time_us = esp_timer_get_time();
    		}
		            
            
        } else {
            ESP_LOGI(TAG, "No data received");
           
           	// assign 0s to all motor drivers when no data is recieved
            PWM_data_in.payload[0] = 0;
			PWM_data_in.payload[1] = 0;
			PWM_data_in.payload[2] = 0;
			PWM_data_in.payload[3] = 0;
			PWM_data_in.payload[4] = 0;
			PWM_data_in.length = 5;
			PWM_data_in.rx_time_us = esp_timer_get_time();
        }
        
        
        // Testing hard coding:
        
        //PWM_data_in.payload[0] = (uint8_t) 100;
		//PWM_data_in.payload[1] = (uint8_t) 90;
		//PWM_data_in.payload[2] = (uint8_t) 80;
		//PWM_data_in.payload[3] = (uint8_t) 70;
		//PWM_data_in.payload[4] = (uint8_t) 60;
		//PWM_data_in.length = 5;
		
		
		//// Disabled for debugging
		
		xQueueOverwrite(PWMQueue, &PWM_data_in);
		
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    

}







/// -------------------------------------------------------------------------------------------------------------- ///



static BNO08x imu;

// IMU handler task
static void imu_task(void *pvParameters) {
	TickType_t last_imu_tick = 0;
	float imu_hz = 0.0f;
	uint32_t imu_sample_count = 0;
	bool got_new_imu_sample = false;
	
	IMUMessage IMU_data_in;

	//// IMU Tasks
	bool imu_ok = imu.initialize();
	if (!imu_ok) {
	    ESP_LOGE(TAG, "BNO08x init failure, continuing without IMU");
	}
	if (imu_ok) {
		ESP_LOGI(TAG, "BNO08x initialized correctly");
	}

	
	
	// Enable useful reports
	imu.rpt.rv.enable(100000UL); // game rotation vector
	imu.rpt.cal_gyro.enable(100000UL); // calibrated gyro

    // Let FreeRTOS scheduler handle rest
    while (true) {
		got_new_imu_sample = false;
		
		
		if (imu_ok && imu.data_available()) {
			// get orientation
			if (imu.rpt.rv.has_new_data()) {
				bno08x_euler_angle_t euler = imu.rpt.rv.get_euler();
				ESP_LOGI(TAG, "Euler: roll = %.2f, pitch = %.2f, yaw = %.2f", euler.x, euler.y, euler.z);
				
		        IMU_data_in.payload[0] = (float) euler.x;
				IMU_data_in.payload[1] = (float) euler.y;
				IMU_data_in.payload[2] = (float) euler.z;
				IMU_data_in.euler_length = 3;
				got_new_imu_sample = true;
				
				
			}
			
			// get angular velocity
			if (imu.rpt.cal_gyro.has_new_data()) {
				bno08x_gyro_t g = imu.rpt.cal_gyro.get();
				ESP_LOGI(TAG, "Gyro: x = %.2f, y = %.2f, z = %.2f rad/s", g.x, g.y, g.z);
				IMU_data_in.payload[3] = (float) g.x;
				IMU_data_in.payload[4] = (float) g.y;
				IMU_data_in.payload[5] = (float) g.z;
				IMU_data_in.g_length = 3;
				got_new_imu_sample = true;
			}
			
			
		}
		else {
			ESP_LOGI(TAG, "IMU Not working");
		}
		
		
		
		
		if (got_new_imu_sample) {
		    TickType_t now = xTaskGetTickCount();
		
		    if (last_imu_tick != 0) {
		        float dt = (now - last_imu_tick) * (portTICK_PERIOD_MS / 1000.0f);
		        if (dt > 0.0f) {
		            imu_hz = 1.0f / dt;
		        }
		    }
		
		    last_imu_tick = now;
		    imu_sample_count++;
		
		    ESP_LOGI(TAG, "IMU rate: %.2f Hz", imu_hz);
		    xQueueOverwrite(IMUQueue, &IMU_data_in);
		}
		
		
		//xQueueOverwrite(IMUQueue, &IMU_data_in);
		vTaskDelay(pdMS_TO_TICKS(10));
		
    }
    
	    
    
    
}





/// -------------------------------------------------------------------------------------------------------------- ///

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

extern "C" void tof_task(void *pvParameters) {
	TickType_t last_tof_tick = 0;
	float tof_hz = 0.0f;
	uint32_t tof_sample_count = 0;
	
	TOFMessage TOF_data_in;
	
	
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
						
						TOF_data_in.payload[idx] = mm;
					}
					printf(";");
				}
				printf("\n");
				fflush(stdout);
				
				TOF_data_in.length = 16;
				
		        TickType_t now = xTaskGetTickCount();
		        if (last_tof_tick != 0) {
		            float dt = (now - last_tof_tick) * (portTICK_PERIOD_MS / 1000.0f);
		            if (dt > 0.0f) {
		                tof_hz = 1.0f / dt;
		            }
		        }
		
		        last_tof_tick = now;
		        tof_sample_count++;
		
		        ESP_LOGI(TAG, "TOF rate: %.2f Hz", tof_hz);
			}
			else {
				TOF_data_in.length = 0;
				memset(TOF_data_in.payload, 0, sizeof(TOF_data_in.payload));
			}
		}
		
		xQueueOverwrite(TOFQueue, &TOF_data_in);
		vTaskDelay(pdMS_TO_TICKS(5));
	}
}




/// -------------------------------------------------------------------------------------------------------------- ///


// Motor Driver PWM Initialization
static void pwm_init(void)
{
    ledc_timer_config_t timer_cfg = {};
    timer_cfg.speed_mode = PWM_MODE;
    timer_cfg.duty_resolution = PWM_RES;
    timer_cfg.timer_num = PWM_TIMER;
    timer_cfg.freq_hz = PWM_FREQ_HZ;
    timer_cfg.clk_cfg = LEDC_AUTO_CLK;
    timer_cfg.deconfigure = false;

    ledc_channel_config_t ch_cfg1 = {};
    ch_cfg1.gpio_num = PWM_GPIO_1;
    ch_cfg1.speed_mode = PWM_MODE;
    ch_cfg1.channel = PWM_CHANNEL_0;
    ch_cfg1.intr_type = LEDC_INTR_DISABLE;
    ch_cfg1.timer_sel = PWM_TIMER;
    ch_cfg1.duty = 0;
    ch_cfg1.hpoint = 0;

    ledc_channel_config_t ch_cfg2 = {};
    ch_cfg2.gpio_num = PWM_GPIO_2;
    ch_cfg2.speed_mode = PWM_MODE;
    ch_cfg2.channel = PWM_CHANNEL_1;
    ch_cfg2.intr_type = LEDC_INTR_DISABLE;
    ch_cfg2.timer_sel = PWM_TIMER;
    ch_cfg2.duty = 0;
    ch_cfg2.hpoint = 0;

    ledc_channel_config_t ch_cfg3 = {};
    ch_cfg3.gpio_num = PWM_GPIO_3;
    ch_cfg3.speed_mode = PWM_MODE;
    ch_cfg3.channel = PWM_CHANNEL_2;
    ch_cfg3.intr_type = LEDC_INTR_DISABLE;
    ch_cfg3.timer_sel = PWM_TIMER;
    ch_cfg3.duty = 0;
    ch_cfg3.hpoint = 0;
    
    ledc_channel_config_t ch_cfg4 = {};
    ch_cfg3.gpio_num = PWM_GPIO_4;
    ch_cfg3.speed_mode = PWM_MODE;
    ch_cfg3.channel = PWM_CHANNEL_3;
    ch_cfg3.intr_type = LEDC_INTR_DISABLE;
    ch_cfg3.timer_sel = PWM_TIMER;
    ch_cfg3.duty = 0;
    ch_cfg3.hpoint = 0;
    
    ledc_channel_config_t ch_cfg5 = {};
    ch_cfg3.gpio_num = PWM_GPIO_5;
    ch_cfg3.speed_mode = PWM_MODE;
    ch_cfg3.channel = PWM_CHANNEL_4;
    ch_cfg3.intr_type = LEDC_INTR_DISABLE;
    ch_cfg3.timer_sel = PWM_TIMER;
    ch_cfg3.duty = 0;
    ch_cfg3.hpoint = 0;

    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));
    ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg1));
    ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg2));
    ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg3));
    ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg4));
    ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg5));
}

// Motor Driver Percentage Setter
static void set_pwm_percent(uint8_t percent1, uint8_t percent2, uint8_t percent3, uint8_t percent4, uint8_t percent5)
{
    if (percent1 > 100) percent1 = 100;
    if (percent2 > 100) percent2 = 100;
    if (percent3 > 100) percent3 = 100;
    if (percent4 > 100) percent4 = 100;
    if (percent5 > 100) percent5 = 100;

    uint32_t max_duty = (1 << 10) - 1;   // 10-bit resolution => 1023
    uint32_t duty1 = (percent1 * max_duty) / 100;
    uint32_t duty2 = (percent2 * max_duty) / 100;
    uint32_t duty3 = (percent3 * max_duty) / 100;
    uint32_t duty4 = (percent4 * max_duty) / 100;
    uint32_t duty5 = (percent5 * max_duty) / 100;


    ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, PWM_CHANNEL_0, duty1));
    ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, PWM_CHANNEL_0));
    
    ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, PWM_CHANNEL_1, duty2));
    ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, PWM_CHANNEL_1));
    
    ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, PWM_CHANNEL_2, duty3));
    ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, PWM_CHANNEL_2));
    
    ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, PWM_CHANNEL_3, duty4));
    ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, PWM_CHANNEL_3));
    
    ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, PWM_CHANNEL_4, duty5));
    ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, PWM_CHANNEL_4));
}

// Motor Driver Handler
static void pwm_handler(void *pvParameters) {
	
	// Dummy Variables for testing
	/*
	uint8_t percent1 = 100;
	uint8_t percent2 = 100;
	uint8_t percent3 = 100;
	uint8_t percent4 = 100;
	uint8_t percent5 = 100;
	*/
	
	PWMMessage activation = {};
	uint8_t testPercent = 50;

	
	while(1) {
		
		//xQueuePeek(PWMQueue, &activation, 0);
		
		if (xQueueReceive(PWMQueue, &activation, portMAX_DELAY) == pdTRUE) {
			
			if (activation.length == 5) {
				set_pwm_percent(activation.payload[0], activation.payload[1], activation.payload[2], activation.payload[3], activation.payload[4]);
				
				int64_t pwm_applied_us = esp_timer_get_time();
				int64_t latency_us = pwm_applied_us - activation.rx_time_us;
				ESP_LOGI("PWM Task", "PWM latency: %lld us (%.3f ms)",
				         latency_us, latency_us / 1000.0);
			}
			else {
				ESP_LOGE("PWM Task: ", "PWM Task recieved incomplete activation data.\n");
				
				int64_t pwm_applied_us = esp_timer_get_time();
				int64_t latency_us = pwm_applied_us - activation.rx_time_us;
				ESP_LOGI("PWM Task", "PWM latency: %lld us (%.3f ms)",
				         latency_us, latency_us / 1000.0);
			}
			
			// Hard coding for debugging:
			//set_pwm_percent(testPercent, testPercent, testPercent, testPercent, testPercent);
			ESP_LOGI("PWM Task: ", "PWM Set to: %d, %d, %d, %d, %d.\n", activation.payload[0], activation.payload[1], activation.payload[2], activation.payload[3], activation.payload[4]);
			
			//vTaskDelay(pdMS_TO_TICKS(500));
		}
	}
}






/// -------------------------------------------------------------------------------------------------------------- ///




// Main program cycle

extern "C" void app_main(void)
{
	// Init queues
	GPSQueue = xQueueCreate(1, sizeof(SensorMessage));
	IMUQueue = xQueueCreate(1, sizeof(IMUMessage));
	TOFQueue = xQueueCreate(1, sizeof(TOFMessage));
	PWMQueue = xQueueCreate(1, sizeof(PWMMessage));
	DRIVERQueue = xQueueCreate(1, sizeof(SensorBatch));
	
	
	//// RGB Task
	xTaskCreate(RGB_task, "rgb_task", 4096, nullptr, 10, nullptr);
	

    //// Create imu task
    //xTaskCreate(imu_task, "imu_task", 4096, nullptr, 10, nullptr);
 	
 	
    //// Create tof task
    //xTaskCreate(tof_task, "tof_task", 4096, NULL, 10, NULL);
    
    
    
    //// Create motor driver tasks
    //pwm_init();
    //xTaskCreate(pwm_handler, "motor_driver_task", 4096, nullptr, 10, nullptr);
    
    
	
	//// Create GPS UART Task
	//uart_init();
    //xTaskCreate(uart_task, "uart_task", 4096, NULL, 5, NULL);
    
    
    
    
    //// Create ESP-RPI UART Task
    //uart_init_2();
    //xTaskCreate(uart_task_2, "uart2_task", 4096, nullptr, 5, nullptr);
	
    

   
    

}


// GPIO pin testing code
/*
extern "C" void app_main(void)
{
    gpio_reset_pin(GPIO_NUM_8);
    gpio_reset_pin(GPIO_NUM_11);
    gpio_reset_pin(GPIO_NUM_10);
    gpio_reset_pin(GPIO_NUM_12);
    gpio_reset_pin(GPIO_NUM_13);
    gpio_reset_pin(GPIO_NUM_9);
    
    
    
    gpio_set_direction(GPIO_NUM_8, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_NUM_11, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_NUM_10, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_NUM_12, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_NUM_13, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_NUM_9, GPIO_MODE_OUTPUT);
    
    



    while (1) {
        gpio_set_level(GPIO_NUM_8, 1);
	    gpio_set_level(GPIO_NUM_11, 1);
	    gpio_set_level(GPIO_NUM_10, 1);
	    gpio_set_level(GPIO_NUM_12, 1);
	    gpio_set_level(GPIO_NUM_13, 1);
	    gpio_set_level(GPIO_NUM_9, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
        
        gpio_set_level(GPIO_NUM_8, 0);
	    gpio_set_level(GPIO_NUM_11, 0);
	    gpio_set_level(GPIO_NUM_10, 0);
	    gpio_set_level(GPIO_NUM_12, 0);
	    gpio_set_level(GPIO_NUM_13, 0);
	    gpio_set_level(GPIO_NUM_9, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    

}
*/
