/* SPI Master example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "drivers/nrf24l01.h"

static const char* TAG = "nrf24_example";

//////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////// Please update the following configuration according to your HardWare spec
////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
#define NRF24_SPI_HOST SPI2_HOST

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   4
#define PIN_NUM_CE   5

// cppcheck-suppress unusedFunction
void app_main(void) {
    esp_err_t ret;

    nrf24_config_t nrf24_config = {
        SPI2_HOST,
        PIN_NUM_CLK,
        PIN_NUM_MISO,
        PIN_NUM_MOSI,
        PIN_NUM_CS,
        PIN_NUM_CE,
        4 * 1000 * 1000,  // Clock out at 4 MHz
        115,
        32,
        5,
        NRF24_DATARATE_1MBPS,
        NRF24_PA_LOW,
        NRF24_CRC_8BIT,
        15,
        15,
        false,
    };
    // Initialise the radio
    nrf24_handle_t radio;

    ret = nrf24_init(&nrf24_config, &radio);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nrf24_init failed: %s", esp_err_to_name(ret));
        return;
    }

    nrf24_set_pa_level(radio, NRF24_PA_HIGH);

    nrf24_set_data_rate(radio, NRF24_DATARATE_1MBPS);

    if (!nrf24_is_chip_connected(radio)) {
        ESP_LOGE(TAG, "nRF24L01+ not responding on SPI — check wiring/power");
        return;
    }

    const uint8_t address[5] = {0xe1, 0xe1, 0xe1, 0xe1, 0xe1};
    nrf24_set_rx_address(radio, 0, address, 5);
    nrf24_set_tx_address(radio, address, 5);
    nrf24_start_listening(radio);

    // nrf24_stop_listening(radio);

    // uint8_t payload[32] = {0xAB};

    // while (1)
    // {
    //     ESP_LOGI(TAG, "Transmitting data ...");
    //     ret = nrf24_write(radio, payload, pdMS_TO_TICKS(500));
    //     if (ret != ESP_OK) {
    //         ESP_LOGE(TAG, "Error transmitting data %x", ret);
    //     }
    //     vTaskDelay(pdMS_TO_TICKS(10000));
    // }

    uint8_t buf[32];

    while (1) {
        uint8_t pipe;
        if (nrf24_available(radio, &pipe)) {
            if (nrf24_read(radio, buf) == ESP_OK) {
                ESP_LOGI(TAG, "Received %d bytes on pipe %d", nrf24_config.payload_size, pipe);
                ESP_LOG_BUFFER_HEX(TAG, buf, 8);
                /* Hand `buf` off to your sensor-link subsystem for
                 * framing/validation here — this driver does not interpret
                 * payload contents. */
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
