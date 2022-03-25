/* 
    LR1110 SPI driver for ESP32-C3
*/

#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"

/*
 * @brief Write this to SPI bus while reading data, or as a dummy/placeholder
 */
#define LR1110_NOP ( 0x00 )

/**
 * @brief LR1110 status
 */
typedef enum lr1110_status_e
{
    LR1110_STATUS_OK    = 0,
    LR1110_STATUS_ERROR = 3,
} lr1110_status_t;

void waitForLowState(gpio_num_t pin)
{
    while(gpio_get_level(pin) != 0)
    {
    };
}

/*
esp_err_t spi_lr1110_write(const void* radio, const uint8_t* data, const uint16_t data_length)
{
    
    esp_err_t err;
    err = spi_device_acquire_bus(&radio->spi, portMAX_DELAY);
    if (err != ESP_OK) return err;

    spi_transaction_t t = {
        .cmd = data,
        .length = data_length,
        .flags = SPI_TRANS_USE_TXDATA,
        .tx_data = {data},
        .user = radio,
    };
    err = spi_device_polling_transmit(&radio->spi, &t);

    spi_device_release_bus(&radio->spi);
    return err;
    
}

esp_err_t spi_lr1110_read(const void* radio, uint8_t* rbuffer, const uint16_t rbuffer_length, uint8_t dummy_byte)
{
    
    spi_transaction_t t = {
        .cmd = CMD_READ | (addr & ADDR_MASK),
        .rxlength = 8,
        .flags = SPI_TRANS_USE_RXDATA,
        .user = radio,
    };
    esp_err_t err = spi_device_polling_transmit(radio->spi, &t);
    if (err!= ESP_OK) return err;

    *out_data = t.rx_data[0];
    return ESP_OK;
    
}
*/