/**
 * @file nrf24l01.c
 * @brief See nrf24l01.h for scope, usage notes, and the hardware-validation
 *        disclaimer. This file has not been compiled or hardware-tested.
 */

#include <string.h>
#include <stdlib.h>

#include "../include/nrf24l01.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"   /* esp_rom_delay_us() */

/* ---------------------------------------------------------------------- */
/* SPI command bytes                                                      */
/* ---------------------------------------------------------------------- */
#define CMD_R_REGISTER      0x00   /* OR with 5-bit register address */
#define CMD_W_REGISTER      0x20   /* OR with 5-bit register address */
#define CMD_R_RX_PAYLOAD    0x61
#define CMD_W_TX_PAYLOAD    0xA0
#define CMD_FLUSH_TX        0xE1
#define CMD_FLUSH_RX        0xE2
#define CMD_NOP             0xFF

/* ---------------------------------------------------------------------- */
/* Register addresses                                                     */
/* ---------------------------------------------------------------------- */
#define REG_CONFIG          0x00
#define REG_EN_AA           0x01
#define REG_EN_RXADDR       0x02
#define REG_SETUP_AW        0x03
#define REG_SETUP_RETR      0x04
#define REG_RF_CH           0x05
#define REG_RF_SETUP        0x06
#define REG_STATUS          0x07
#define REG_OBSERVE_TX      0x08
#define REG_RX_ADDR_P0      0x0A
#define REG_RX_ADDR_P1      0x0B
#define REG_RX_ADDR_P2      0x0C
#define REG_RX_ADDR_P3      0x0D
#define REG_RX_ADDR_P4      0x0E
#define REG_RX_ADDR_P5      0x0F
#define REG_TX_ADDR         0x10
#define REG_RX_PW_P0        0x11
#define REG_RX_PW_P1        0x12
#define REG_RX_PW_P2        0x13
#define REG_RX_PW_P3        0x14
#define REG_RX_PW_P4        0x15
#define REG_RX_PW_P5        0x16
#define REG_FIFO_STATUS     0x17

/* ---------------------------------------------------------------------- */
/* Bit masks                                                               */
/* ---------------------------------------------------------------------- */
#define CONFIG_EN_CRC        (1 << 3)
#define CONFIG_CRCO          (1 << 2)
#define CONFIG_PWR_UP        (1 << 1)
#define CONFIG_PRIM_RX       (1 << 0)

#define STATUS_RX_DR         (1 << 6)
#define STATUS_TX_DS         (1 << 5)
#define STATUS_MAX_RT        (1 << 4)
#define STATUS_RX_P_NO_MASK  0x0E

#define FIFO_STATUS_RX_EMPTY (1 << 0)

#define RF_SETUP_RF_DR_LOW   (1 << 5)
#define RF_SETUP_RF_DR_HIGH  (1 << 3)
#define RF_SETUP_RF_PWR_MASK 0x06

/* Pipes 0 and 1 each have a full-width address register (3-5 bytes).
 * Pipes 2-5 only store one unique address byte in hardware and implicitly
 * share the remaining upper bytes with pipe 1's address — this is
 * nRF24L01+ silicon behavior, not a driver-imposed limitation. */
#define NRF24_PIPES_WITH_FULL_ADDRESS 2

struct nrf24_dev {
    spi_device_handle_t spi;
    int pin_ce;
    uint8_t payload_size;
    uint8_t address_width;
    bool listening;
};

/* ---------------------------------------------------------------------- */
/* Low-level SPI + CE helpers                                              */
/* ---------------------------------------------------------------------- */

static inline void ce_high(nrf24_handle_t h) { gpio_set_level(h->pin_ce, 1); }
static inline void ce_low(nrf24_handle_t h)  { gpio_set_level(h->pin_ce, 0); }

/* Full-duplex SPI transaction. tx/rx must each be `len` bytes; rx[0] always
 * receives the STATUS register (returned by the chip during the command
 * byte's transfer, per nRF24L01+ SPI protocol). */
static esp_err_t spi_txn(nrf24_handle_t h, const uint8_t *tx, uint8_t *rx, size_t len)
{
    if (len == 0 || len > (NRF24_MAX_PAYLOAD_SIZE + 1)) {
        return ESP_ERR_INVALID_ARG;
    }
    spi_transaction_t t = {0};
    t.length = len * 8;
    t.tx_buffer = tx;
    t.rx_buffer = rx;
    return spi_device_polling_transmit(h->spi, &t);
}

static esp_err_t read_register(nrf24_handle_t h, uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t tx[1 + NRF24_MAX_PAYLOAD_SIZE] = {0};
    uint8_t rx[1 + NRF24_MAX_PAYLOAD_SIZE] = {0};
    tx[0] = CMD_R_REGISTER | (reg & 0x1F);
    esp_err_t err = spi_txn(h, tx, rx, (size_t)len + 1);
    if (err == ESP_OK && data) {
        memcpy(data, &rx[1], len);
    }
    return err;
}

static esp_err_t write_register(nrf24_handle_t h, uint8_t reg, const uint8_t *data, uint8_t len)
{
    uint8_t tx[1 + NRF24_MAX_PAYLOAD_SIZE] = {0};
    uint8_t rx[1 + NRF24_MAX_PAYLOAD_SIZE] = {0};
    tx[0] = CMD_W_REGISTER | (reg & 0x1F);
    memcpy(&tx[1], data, len);
    return spi_txn(h, tx, rx, (size_t)len + 1);
}

static esp_err_t read_register_byte(nrf24_handle_t h, uint8_t reg, uint8_t *value)
{
    return read_register(h, reg, value, 1);
}

static esp_err_t write_register_byte(nrf24_handle_t h, uint8_t reg, uint8_t value)
{
    return write_register(h, reg, &value, 1);
}

static esp_err_t send_command(nrf24_handle_t h, uint8_t cmd, uint8_t *status_out)
{
    uint8_t tx[1] = { cmd };
    uint8_t rx[1] = { 0 };
    esp_err_t err = spi_txn(h, tx, rx, 1);
    if (err == ESP_OK && status_out) {
        *status_out = rx[0];
    }
    return err;
}

/* ---------------------------------------------------------------------- */
/* Public API                                                              */
/* ---------------------------------------------------------------------- */

esp_err_t nrf24_init(const nrf24_config_t *config, nrf24_handle_t *out_handle)
{
    if (!config || !out_handle) {
        return ESP_ERR_INVALID_ARG;
    }
    if (config->payload_size == 0 || config->payload_size > NRF24_MAX_PAYLOAD_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }
    if (config->address_width < NRF24_ADDR_MIN_WIDTH ||
        config->address_width > NRF24_ADDR_MAX_WIDTH) {
        return ESP_ERR_INVALID_ARG;
    }
    if (config->channel > 125) {
        return ESP_ERR_INVALID_ARG;
    }

    struct nrf24_dev *h = calloc(1, sizeof(struct nrf24_dev));
    if (!h) {
        return ESP_ERR_NO_MEM;
    }

    h->pin_ce = config->pin_ce;
    h->payload_size = config->payload_size;
    h->address_width = config->address_width;
    h->listening = false;

    esp_err_t err;

    gpio_config_t ce_conf = {
        .pin_bit_mask = 1ULL << config->pin_ce,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    err = gpio_config(&ce_conf);
    if (err != ESP_OK) goto fail;
    ce_low(h);

    spi_bus_config_t buscfg = {
        .mosi_io_num = config->pin_mosi,
        .miso_io_num = config->pin_miso,
        .sclk_io_num = config->pin_sck,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = NRF24_MAX_PAYLOAD_SIZE + 1,
    };
    err = spi_bus_initialize(config->spi_host, &buscfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        /* INVALID_STATE => bus already initialized by another device; that's fine. */
        goto fail;
    }

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = config->spi_clock_hz > 0 ? config->spi_clock_hz : 4000000,
        .mode = 0,
        .spics_io_num = config->pin_csn,
        .queue_size = 1,
        .cs_ena_pretrans = 2,
    };
    err = spi_bus_add_device(config->spi_host, &devcfg, &h->spi);
    if (err != ESP_OK) goto fail;

    /* Datasheet: allow >=100ms from power-on before the chip is guaranteed
     * to respond correctly to CSN/SPI activity. */
    vTaskDelay(pdMS_TO_TICKS(100));

    /* Configure with the radio powered down, then power up at the end. */
    err = write_register_byte(h, REG_CONFIG, 0x00);
    if (err != ESP_OK) goto fail;

    uint8_t setup_retr = ((config->retry_delay_x250us & 0x0F) << 4) |
                          (config->retry_count & 0x0F);
    err = write_register_byte(h, REG_SETUP_RETR, setup_retr);
    if (err != ESP_OK) goto fail;

    err = nrf24_set_channel(h, config->channel);
    if (err != ESP_OK) goto fail;

    uint8_t rf_setup = 0;
    switch (config->data_rate) {
        case NRF24_DATARATE_250KBPS: rf_setup |= RF_SETUP_RF_DR_LOW; break;
        case NRF24_DATARATE_2MBPS:   rf_setup |= RF_SETUP_RF_DR_HIGH; break;
        case NRF24_DATARATE_1MBPS:
        default:
            break; /* both rate bits 0 == 1Mbps */
    }
    rf_setup |= ((uint8_t)(config->pa_level << 1) & RF_SETUP_RF_PWR_MASK);
    err = write_register_byte(h, REG_RF_SETUP, rf_setup);
    if (err != ESP_OK) goto fail;

    err = write_register_byte(h, REG_SETUP_AW, (config->address_width - 2) & 0x03);
    if (err != ESP_OK) goto fail;

    uint8_t en_aa = config->auto_ack_enabled ? 0x3F : 0x00;
    err = write_register_byte(h, REG_EN_AA, en_aa);
    if (err != ESP_OK) goto fail;

    /* No RX pipes enabled yet; nrf24_set_rx_address() enables them individually. */
    err = write_register_byte(h, REG_EN_RXADDR, 0x00);
    if (err != ESP_OK) goto fail;

    uint8_t config_reg = 0;
    if (config->crc_length != NRF24_CRC_DISABLED) {
        config_reg |= CONFIG_EN_CRC;
        if (config->crc_length == NRF24_CRC_16BIT) {
            config_reg |= CONFIG_CRCO;
        }
    }
    err = write_register_byte(h, REG_CONFIG, config_reg);
    if (err != ESP_OK) goto fail;

    err = nrf24_flush_rx(h);
    if (err != ESP_OK) goto fail;
    err = nrf24_flush_tx(h);
    if (err != ESP_OK) goto fail;
    err = write_register_byte(h, REG_STATUS, STATUS_RX_DR | STATUS_TX_DS | STATUS_MAX_RT);
    if (err != ESP_OK) goto fail;

    err = nrf24_power_up(h);
    if (err != ESP_OK) goto fail;

    *out_handle = h;
    return ESP_OK;

fail:
    if (h->spi) {
        spi_bus_remove_device(h->spi);
    }
    free(h);
    return err;
}

esp_err_t nrf24_deinit(nrf24_handle_t handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    ce_low(handle);
    esp_err_t err = spi_bus_remove_device(handle->spi);
    free(handle);
    return err;
}

bool nrf24_is_chip_connected(nrf24_handle_t handle)
{
    if (!handle) {
        return false;
    }
    /* SETUP_AW's two low bits are always 0b01/0b10/0b11 after nrf24_init()
     * has run (address width 3/4/5 bytes). A stuck-at-0x00 or 0xFF readback
     * (typical of a floating/disconnected MISO line) fails this check. */
    uint8_t setup_aw;
    if (read_register_byte(handle, REG_SETUP_AW, &setup_aw) != ESP_OK) {
        return false;
    }
    uint8_t aw_bits = setup_aw & 0x03;
    return (aw_bits >= 1 && aw_bits <= 3);
}

esp_err_t nrf24_set_channel(nrf24_handle_t handle, uint8_t channel)
{
    if (!handle || channel > 125) {
        return ESP_ERR_INVALID_ARG;
    }
    return write_register_byte(handle, REG_RF_CH, channel);
}

esp_err_t nrf24_set_pa_level(nrf24_handle_t handle, nrf24_pa_level_t level)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t rf_setup;
    esp_err_t err = read_register_byte(handle, REG_RF_SETUP, &rf_setup);
    if (err != ESP_OK) {
        return err;
    }
    rf_setup = (rf_setup & ~RF_SETUP_RF_PWR_MASK) |
               ((uint8_t)(level << 1) & RF_SETUP_RF_PWR_MASK);
    return write_register_byte(handle, REG_RF_SETUP, rf_setup);
}

esp_err_t nrf24_set_data_rate(nrf24_handle_t handle, nrf24_datarate_t rate)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t rf_setup;
    esp_err_t err = read_register_byte(handle, REG_RF_SETUP, &rf_setup);
    if (err != ESP_OK) {
        return err;
    }
    rf_setup &= ~(RF_SETUP_RF_DR_LOW | RF_SETUP_RF_DR_HIGH);
    switch (rate) {
        case NRF24_DATARATE_250KBPS: rf_setup |= RF_SETUP_RF_DR_LOW; break;
        case NRF24_DATARATE_2MBPS:   rf_setup |= RF_SETUP_RF_DR_HIGH; break;
        case NRF24_DATARATE_1MBPS:
        default:
            break;
    }
    return write_register_byte(handle, REG_RF_SETUP, rf_setup);
}

esp_err_t nrf24_set_rx_address(nrf24_handle_t handle, uint8_t pipe,
                                const uint8_t *address, uint8_t address_len)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (address == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (pipe >= NRF24_PIPE_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    static const uint8_t pipe_addr_reg[NRF24_PIPE_COUNT] = {
        REG_RX_ADDR_P0, REG_RX_ADDR_P1, REG_RX_ADDR_P2,
        REG_RX_ADDR_P3, REG_RX_ADDR_P4, REG_RX_ADDR_P5,
    };
    static const uint8_t pipe_pw_reg[NRF24_PIPE_COUNT] = {
        REG_RX_PW_P0, REG_RX_PW_P1, REG_RX_PW_P2,
        REG_RX_PW_P3, REG_RX_PW_P4, REG_RX_PW_P5,
    };

    esp_err_t err;
    if (pipe < NRF24_PIPES_WITH_FULL_ADDRESS) {
        if (address_len != handle->address_width) {
            return ESP_ERR_INVALID_ARG;
        }
        err = write_register(handle, pipe_addr_reg[pipe], address, address_len);
    } else {
        /* Pipes 2-5: hardware only stores one unique LSB; upper bytes are
         * implicitly shared with pipe 1's address. */
        if (address_len != 1) {
            return ESP_ERR_INVALID_ARG;
        }
        err = write_register(handle, pipe_addr_reg[pipe], address, 1);
    }
    if (err != ESP_OK) {
        return err;
    }

    uint8_t en_rxaddr;
    err = read_register_byte(handle, REG_EN_RXADDR, &en_rxaddr);
    if (err != ESP_OK) {
        return err;
    }
    en_rxaddr |= (1 << pipe);
    err = write_register_byte(handle, REG_EN_RXADDR, en_rxaddr);
    if (err != ESP_OK) {
        return err;
    }

    return write_register_byte(handle, pipe_pw_reg[pipe], handle->payload_size);
}

esp_err_t nrf24_set_tx_address(nrf24_handle_t handle, const uint8_t *address, uint8_t address_len)
{
    if (!handle || !address || address_len != handle->address_width) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = write_register(handle, REG_TX_ADDR, address, address_len);
    if (err != ESP_OK) {
        return err;
    }
    /* Required for auto-ack: this device must listen on pipe 0 at the same
     * address it transmits to, in order to receive the peer's ACK. */
    return write_register(handle, REG_RX_ADDR_P0, address, address_len);
}

esp_err_t nrf24_start_listening(nrf24_handle_t handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t config_reg;
    esp_err_t err = read_register_byte(handle, REG_CONFIG, &config_reg);
    if (err != ESP_OK) {
        return err;
    }
    config_reg |= CONFIG_PRIM_RX;
    err = write_register_byte(handle, REG_CONFIG, config_reg);
    if (err != ESP_OK) {
        return err;
    }
    write_register_byte(handle, REG_STATUS, STATUS_RX_DR | STATUS_TX_DS | STATUS_MAX_RT);
    nrf24_flush_rx(handle);
    ce_high(handle);
    handle->listening = true;
    /* Datasheet: >=130us needed after CE high before valid RX can occur.
     * Using a short blocking delay here (below FreeRTOS tick granularity). */
    esp_rom_delay_us(130);
    return ESP_OK;
}

esp_err_t nrf24_stop_listening(nrf24_handle_t handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    ce_low(handle);
    uint8_t config_reg;
    esp_err_t err = read_register_byte(handle, REG_CONFIG, &config_reg);
    if (err != ESP_OK) {
        return err;
    }
    config_reg &= ~CONFIG_PRIM_RX;
    err = write_register_byte(handle, REG_CONFIG, config_reg);
    handle->listening = false;
    return err;
}

bool nrf24_available(nrf24_handle_t handle, uint8_t *pipe_num)
{
    if (!handle) {
        return false;
    }
    uint8_t fifo_status;
    if (read_register_byte(handle, REG_FIFO_STATUS, &fifo_status) != ESP_OK) {
        return false;
    }
    if (fifo_status & FIFO_STATUS_RX_EMPTY) {
        return false;
    }
    if (pipe_num) {
        uint8_t status;
        if (nrf24_get_status(handle, &status) == ESP_OK) {
            *pipe_num = (status & STATUS_RX_P_NO_MASK) >> 1;
        } else {
            *pipe_num = 0xFF;
        }
    }
    return true;
}

esp_err_t nrf24_read(nrf24_handle_t handle, void *buf)
{
    if (!handle || !buf) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t tx[1 + NRF24_MAX_PAYLOAD_SIZE] = {0};
    uint8_t rx[1 + NRF24_MAX_PAYLOAD_SIZE] = {0};
    tx[0] = CMD_R_RX_PAYLOAD;
    esp_err_t err = spi_txn(handle, tx, rx, (size_t)handle->payload_size + 1);
    if (err != ESP_OK) {
        return err;
    }
    memcpy(buf, &rx[1], handle->payload_size);

    /* Clear RX_DR. If another payload remains in the FIFO, hardware re-sets
     * this bit automatically -- caller should re-check nrf24_available(). */
    return write_register_byte(handle, REG_STATUS, STATUS_RX_DR);
}

esp_err_t nrf24_write(nrf24_handle_t handle, const void *buf, TickType_t timeout_ticks)
{
    if (!handle || !buf) {
        return ESP_ERR_INVALID_ARG;
    }

    bool was_listening = handle->listening;
    if (was_listening) {
        nrf24_stop_listening(handle);
    }

    uint8_t tx[1 + NRF24_MAX_PAYLOAD_SIZE] = {0};
    uint8_t rx[1 + NRF24_MAX_PAYLOAD_SIZE] = {0};
    tx[0] = CMD_W_TX_PAYLOAD;
    memcpy(&tx[1], buf, handle->payload_size);
    esp_err_t err = spi_txn(handle, tx, rx, (size_t)handle->payload_size + 1);
    if (err != ESP_OK) {
        if (was_listening) {
            nrf24_start_listening(handle);
        }
        return err;
    }

    /* Datasheet: CE must be held high >=10us to trigger transmission; the
     * radio's own state machine handles the rest (send, wait for ack,
     * retry per SETUP_RETR) without further SPI/CE activity from us. */
    ce_high(handle);
    esp_rom_delay_us(15);
    ce_low(handle);

    TickType_t start = xTaskGetTickCount();
    esp_err_t result = ESP_ERR_TIMEOUT;
    for (;;) {
        uint8_t status;
        if (nrf24_get_status(handle, &status) != ESP_OK) {
            result = ESP_FAIL;
            break;
        }
        if (status & STATUS_TX_DS) {
            write_register_byte(handle, REG_STATUS, STATUS_TX_DS);
            result = ESP_OK;
            break;
        }
        if (status & STATUS_MAX_RT) {
            write_register_byte(handle, REG_STATUS, STATUS_MAX_RT);
            nrf24_flush_tx(handle);
            result = ESP_FAIL;
            break;
        }
        if (timeout_ticks != portMAX_DELAY &&
            (xTaskGetTickCount() - start) >= timeout_ticks) {
            break; /* result stays ESP_ERR_TIMEOUT */
        }
        vTaskDelay(1);
    }

    if (was_listening) {
        nrf24_start_listening(handle);
    }
    return result;
}

esp_err_t nrf24_flush_rx(nrf24_handle_t handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    return send_command(handle, CMD_FLUSH_RX, NULL);
}

esp_err_t nrf24_flush_tx(nrf24_handle_t handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    return send_command(handle, CMD_FLUSH_TX, NULL);
}

esp_err_t nrf24_power_down(nrf24_handle_t handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    ce_low(handle);
    uint8_t config_reg;
    esp_err_t err = read_register_byte(handle, REG_CONFIG, &config_reg);
    if (err != ESP_OK) {
        return err;
    }
    config_reg &= ~CONFIG_PWR_UP;
    return write_register_byte(handle, REG_CONFIG, config_reg);
}

esp_err_t nrf24_power_up(nrf24_handle_t handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t config_reg;
    esp_err_t err = read_register_byte(handle, REG_CONFIG, &config_reg);
    if (err != ESP_OK) {
        return err;
    }
    if (config_reg & CONFIG_PWR_UP) {
        return ESP_OK; /* already up */
    }
    config_reg |= CONFIG_PWR_UP;
    err = write_register_byte(handle, REG_CONFIG, config_reg);
    if (err != ESP_OK) {
        return err;
    }
    /* Datasheet Tpd2stby: allow the crystal oscillator to stabilize. */
    vTaskDelay(pdMS_TO_TICKS(5));
    return ESP_OK;
}

esp_err_t nrf24_get_status(nrf24_handle_t handle, uint8_t *status)
{
    if (!handle || !status) {
        return ESP_ERR_INVALID_ARG;
    }
    return send_command(handle, CMD_NOP, status);
}

esp_err_t nrf24_get_observe_tx(nrf24_handle_t handle, uint8_t *lost_packet_count,
                                uint8_t *retransmit_count)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t val;
    esp_err_t err = read_register_byte(handle, REG_OBSERVE_TX, &val);
    if (err != ESP_OK) {
        return err;
    }
    if (lost_packet_count) {
        *lost_packet_count = (val >> 4) & 0x0F;
    }
    if (retransmit_count) {
        *retransmit_count = val & 0x0F;
    }
    return ESP_OK;
}
