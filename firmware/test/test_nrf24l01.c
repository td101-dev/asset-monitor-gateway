#include "unity.h"
#include "nrf24l01.h"

void setUp(void) {}
void tearDown(void) {}

void test_nrf24_stop_listening_NullHandle_ReturnsInvalidArg(void) {
    esp_err_t result = nrf24_stop_listening(NULL);

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, result);
    // No mock_spi_master_* or mock_gpio_* expectations are set here —
    // if the code somehow reaches a hardware call before the null check,
    // CMock will fail this test with an "unexpected call" error.
}
