/**
 * @file nrf24l01.h
 * @brief Minimal, dependency-light ESP-IDF driver for the Nordic nRF24L01+
 *        2.4GHz transceiver, using the native spi_master and gpio drivers.
 *
 * Scope (v1 / first iteration):
 *  - Fixed-length payloads only (1-32 bytes), no dynamic payload / ACK payload
 *    support. Add as a later iteration if needed.
 *  - Polling-based (no IRQ pin support). Fine for a low-throughput sensor
 *    link; revisit if you need lower receive latency or lower CPU usage.
 *  - Hardware auto-ack + auto-retransmit is used for nrf24_write() reliability;
 *    this driver does not implement any additional software-level retry.
 *
 * IMPORTANT: This code has been written carefully against the nRF24L01+
 * datasheet timing/register behavior, but has NOT been compiled, flashed, or
 * validated against real hardware in this environment. Treat it as a
 * first-draft reference implementation. Before relying on it:
 *  - Build it against your actual ESP-IDF version and target chip.
 *  - Verify nrf24_is_chip_connected() returns true.
 *  - Run a basic loopback test between two boards.
 *  - Check timing assumptions (delays below) against your specific IDF
 *    version if you see intermittent failures.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/spi_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NRF24_MAX_PAYLOAD_SIZE 32
#define NRF24_ADDR_MIN_WIDTH   3
#define NRF24_ADDR_MAX_WIDTH   5
#define NRF24_PIPE_COUNT       6

typedef enum {
    NRF24_DATARATE_250KBPS = 0,
    NRF24_DATARATE_1MBPS,
    NRF24_DATARATE_2MBPS,
} nrf24_datarate_t;

typedef enum {
    NRF24_PA_MIN = 0, /* -18 dBm */
    NRF24_PA_LOW,     /* -12 dBm */
    NRF24_PA_HIGH,    /*  -6 dBm */
    NRF24_PA_MAX,     /*   0 dBm */
} nrf24_pa_level_t;

typedef enum {
    NRF24_CRC_DISABLED = 0,
    NRF24_CRC_8BIT,
    NRF24_CRC_16BIT,
} nrf24_crc_t;

typedef struct {
    spi_host_device_t spi_host; /* e.g. SPI2_HOST */
    int pin_sck;
    int pin_miso;
    int pin_mosi;
    int pin_csn;
    int pin_ce;
    int spi_clock_hz; /* <= 10,000,000; 4,000,000 is a safe default */

    uint8_t channel;       /* 0-125 -> 2400 + channel MHz */
    uint8_t payload_size;  /* 1-32 bytes, fixed-length payload */
    uint8_t address_width; /* 3-5 bytes */
    nrf24_datarate_t data_rate;
    nrf24_pa_level_t pa_level;
    nrf24_crc_t crc_length;

    uint8_t retry_delay_x250us; /* 0-15 -> (value+1) * 250us between retries */
    uint8_t retry_count;        /* 0-15, 0 disables auto-retransmit */

    bool auto_ack_enabled; /* enable hardware ACK on all pipes */
} nrf24_config_t;

/** Opaque handle to an initialized radio instance. */
typedef struct nrf24_dev* nrf24_handle_t;

/**
 * @brief Initialize the SPI bus/device and the nRF24L01+ radio.
 *
 * On success, the radio is powered up and in Standby-I (CE low, not yet
 * listening or transmitting). Call nrf24_set_rx_address()/nrf24_set_tx_address()
 * and nrf24_start_listening() as appropriate before use.
 *
 * If another device already shares the given SPI host/bus, spi_bus_initialize()
 * returning ESP_ERR_INVALID_STATE is treated as non-fatal (bus already up).
 */
esp_err_t nrf24_init(const nrf24_config_t* config, nrf24_handle_t* out_handle);

/** @brief Remove the SPI device and free driver resources. Does not call
 *         spi_bus_free() since other devices may share the bus. */
esp_err_t nrf24_deinit(nrf24_handle_t handle);

/**
 * @brief Basic sanity check that SPI communication with the chip is working,
 *        by verifying a register readback is within its valid range.
 *        This does NOT verify RF link quality or antenna presence.
 */
bool nrf24_is_chip_connected(nrf24_handle_t handle);

static esp_err_t nrf24_set_channel(nrf24_handle_t handle, uint8_t channel);
esp_err_t nrf24_set_pa_level(nrf24_handle_t handle, nrf24_pa_level_t level);
esp_err_t nrf24_set_data_rate(nrf24_handle_t handle, nrf24_datarate_t rate);

/**
 * @brief Set the RX address for a pipe (0-5) and enable that pipe.
 *
 * Pipes 0 and 1 use the full configured address width. Pipes 2-5 only carry
 * one unique address byte in hardware; the remaining bytes are implicitly
 * shared with pipe 1's address (this is nRF24L01+ hardware behavior, not a
 * driver limitation) — pass a 1-byte address for pipes 2-5.
 */
esp_err_t nrf24_set_rx_address(nrf24_handle_t handle, uint8_t pipe, const uint8_t* address,
                               uint8_t address_len);

/**
 * @brief Set the TX destination address. Also programs pipe 0's RX address
 *        to match, which is required by the hardware so this device can
 *        receive the auto-ack response from the recipient.
 */
esp_err_t nrf24_set_tx_address(nrf24_handle_t handle, const uint8_t* address, uint8_t address_len);

/** @brief Enter RX mode (CE high, PRIM_RX set). */
esp_err_t nrf24_start_listening(nrf24_handle_t handle);

/** @brief Leave RX mode, returning to Standby-I. */
esp_err_t nrf24_stop_listening(nrf24_handle_t handle);

/**
 * @brief Non-blocking check for a received payload.
 * @param pipe_num If non-NULL, set to the pipe number the payload arrived on
 *                 (0-5), or 0xFF if it could not be determined.
 * @return true if a payload is waiting in the RX FIFO.
 */
bool nrf24_available(nrf24_handle_t handle, uint8_t* pipe_num);

/**
 * @brief Read one payload from the RX FIFO into buf.
 * @param buf Must point to at least config->payload_size bytes.
 */
esp_err_t nrf24_read(nrf24_handle_t handle, void* buf);

/**
 * @brief Transmit one payload to the configured TX address, blocking until
 *        the hardware reports success/failure or timeout_ticks elapses.
 *
 * If the radio was listening, it is temporarily taken out of RX mode for the
 * duration of the transmission and restored afterward.
 *
 * @param buf Must point to exactly config->payload_size bytes.
 * @return ESP_OK on acknowledged delivery.
 *         ESP_FAIL if the hardware exhausted its auto-retransmit attempts
 *           (peer unreachable / no ACK received) — TX FIFO is flushed.
 *         ESP_ERR_TIMEOUT if timeout_ticks elapsed before either outcome.
 */
esp_err_t nrf24_write(nrf24_handle_t handle, const void* buf, TickType_t timeout_ticks);

static esp_err_t nrf24_flush_rx(nrf24_handle_t handle);
static esp_err_t nrf24_flush_tx(nrf24_handle_t handle);

/** @brief Power down the radio (lowest power state; register config is retained). */
esp_err_t nrf24_power_down(nrf24_handle_t handle);

/** @brief Power the radio back up. Includes the datasheet-required settle
 *         delay for the crystal oscillator before returning. */
static esp_err_t nrf24_power_up(nrf24_handle_t handle);

/** @brief Read the raw STATUS register — useful for diagnostics/logging. */
static esp_err_t nrf24_get_status(nrf24_handle_t handle, uint8_t* status);

/**
 * @brief Read the OBSERVE_TX register: lost-packet and retransmit counters.
 *        Both are 4-bit saturating counters that reset when the channel
 *        (RF_CH) is rewritten. Useful as an early "degraded link" signal —
 *        see the fault-evaluation subsystem design.
 */
esp_err_t nrf24_get_observe_tx(nrf24_handle_t handle, uint8_t* lost_packet_count,
                               uint8_t* retransmit_count);

#ifdef __cplusplus
}
#endif
