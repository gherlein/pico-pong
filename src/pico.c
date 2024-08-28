
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include "radio-config.h"

void SPI_Init(spi_inst_t *spi)
{

    spi_init(spi, 1000 * 1000);
    gpio_set_function(RADIO_MISO, GPIO_FUNC_SPI);
    gpio_set_function(RADIO_SCLK, GPIO_FUNC_SPI);
    gpio_set_function(RADIO_MOSI, GPIO_FUNC_SPI);

    // Set SPI format
    spi_set_format(spi, // SPI instance
                   8,   // Number of bits per transfer
                   0,   // Polarity (CPOL)
                   0,   // Phase (CPHA)
                   SPI_MSB_FIRST);

    return;
}

void GPIO_Init()
{
    // RADIO_BUSY is an input
    gpio_init(RADIO_BUSY);
    gpio_set_dir(RADIO_NSS, GPIO_IN);

    // Reset is active-low
    gpio_init(RADIO_RESET);
    gpio_set_dir(RADIO_RESET, GPIO_OUT);
    gpio_put(RADIO_RESET, 1);

    // Chip select NSS is active-low, so we'll initialise it to a driven-high state
    gpio_init(RADIO_NSS);
    gpio_set_dir(RADIO_NSS, GPIO_OUT);
    gpio_put(RADIO_NSS, 1);
}
