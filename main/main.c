/* Test code for TOF VL53L5CX using components/VL53L5CX-Library
 * Default I2C pins: SDA=21, SCL=22 (I2C_NUM_0)
 * Optional XSHUT pin wired to GPIO5 for sensor reset
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "vl53l5cx_api.h"

void app_main(void)
{
    // i2c config using default ESP32 I2C pins (SDA=21, SCL=22)
    i2c_port_t i2c_port = I2C_NUM_0;
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = i2c_port,
        .scl_io_num = 22,
        .sda_io_num = 21,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    // i2c device config
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = VL53L5CX_DEFAULT_I2C_ADDRESS >> 1,
        .scl_speed_hz = VL53L5CX_MAX_CLK_SPEED,
    };

    // TOF variables
    uint8_t status, isAlive, isReady;
    uint32_t loop;
    static VL53L5CX_Configuration Dev;   // Sensor configuration (static to avoid stack overflow)
    static VL53L5CX_ResultsData Results; // Results data from VL53L5CX (static to avoid stack overflow)

    Dev.platform.bus_config = i2c_mst_config;

    // Register the device on the bus
    i2c_master_bus_add_device(bus_handle, &dev_cfg, &Dev.platform.handle);

    // (optional) reset sensor - set reset pin if wired to a GPIO
    // wire XSHUT to the GPIO below This clears MCU error states
    Dev.platform.reset_gpio = GPIO_NUM_5;
    VL53L5CX_Reset_Sensor(&(Dev.platform));
    VL53L5CX_WaitMs(&(Dev.platform), 10);

    // check sensor connection
    status = vl53l5cx_is_alive(&Dev, &isAlive);
    if (!isAlive || status)
    {
        printf("vl53l5cx_is_alive failed\n");
        return;
    }

    // init TOF sensor
    status = vl53l5cx_init(&Dev);
    if (status)
    {
        printf("vl53l5cx_init failed\n");
        return;
    }

   // set resolution to 4x4 (16 zones) to reduce result size for stack overflow and easier calculation
    status = vl53l5cx_set_resolution(&Dev, VL53L5CX_RESOLUTION_4X4);
    if (status)
    {
        printf("vl53l5cx_set_resolution failed, status %u\n", status);
        return;
    }

    printf("VL53L5CX ready!\n");

    // set ranging mode autonomous (basically just configures stuff for us)
    status = vl53l5cx_set_ranging_mode(&Dev, VL53L5CX_RANGING_MODE_AUTONOMOUS);
    if (status)
    {
        printf("vl53l5cx_set_ranging_mode failed, status %u\n", status);
        return;
    }
    
    status = vl53l5cx_set_integration_time_ms(&Dev, 20); // set integration time (ms) for autonomous mode
    status |= vl53l5cx_set_ranging_frequency_hz(&Dev, 10); // set ranging frequency (Hz) for autonomous mode
    status = vl53l5cx_start_ranging(&Dev); // start ranging

    loop = 0;
    while (true)
    {
        status = vl53l5cx_check_data_ready(&Dev, &isReady);
        if (isReady)
        {
            vl53l5cx_get_ranging_data(&Dev, &Results);

            // 4x4 grid with screen clear
            // printf("\033[2J\033[H");
            // printf("VL53L5CX 4x4 (stream %3u)\n\n", Dev.streamcount);

            for (uint8_t row = 0; row < 4; row++)
            {
                for (uint8_t col = 0; col < 4; col++)
                {
                    uint8_t idx = row * 4 + col;
                    uint16_t dist = Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE * idx];
                    // uint8_t st = Results.target_status[VL53L5CX_NB_TARGET_PER_ZONE * idx];

                    if (dist == 0)
                    {
                        // no target / invalid
                        printf("0,");
                    }
                    else
                    {
                        printf("%d,", dist);
                    }
                }
                printf(";");
            }
            printf("\n");
            fflush(stdout);
            loop++;
        }

        // small delay to avoid I2C overload
        VL53L5CX_WaitMs(&(Dev.platform), 5);
    }

    status = vl53l5cx_stop_ranging(&Dev);
    printf("vl53l5cx_stop_ranging called\n");
    VL53L5CX_WaitMs(&(Dev.platform), 1);
}
